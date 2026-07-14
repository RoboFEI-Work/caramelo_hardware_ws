#include "caramelo_hardware/maxon_motors_node.hpp"

#include <algorithm>
#include <array>
#include <cerrno>
#include <cmath>
#include <cstdint>
#include <chrono>
#include <cstdio>
#include <cstring>
#include <fcntl.h>
#include <poll.h>
#include <unistd.h>

#if defined(CARAMELO_HAS_PIGPIO) && CARAMELO_HAS_PIGPIO
#include <pigpiod_if2.h>
#include <pigpio.h>
#endif

namespace mobile_base_hardware
{

namespace
{
constexpr double kTwoPi = 6.28318530717958647692;
constexpr int kInvalidCallbackId = -1;
#if defined(PI_EITHER_EDGE)
constexpr unsigned kEitherEdge = PI_EITHER_EDGE;
#elif defined(EITHER_EDGE)
constexpr unsigned kEitherEdge = EITHER_EDGE;
#else
constexpr unsigned kEitherEdge = 2U;
#endif

int clamp_int(int value, int min_v, int max_v)
{
	return std::min(std::max(value, min_v), max_v);
}

#if defined(CARAMELO_HAS_PIGPIO) && CARAMELO_HAS_PIGPIO
uint32_t bit(int gpio)
{
	return (1u << gpio);
}
#endif

}  // namespace

MaxonMotorsNode::MaxonMotorsNode()
: Node("maxon_motors_node")
{
	velocity_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>(
		"maxon/wheel_velocity", rclcpp::QoS(10));
}

MaxonMotorsNode::~MaxonMotorsNode()
{
	shutdown_hardware();
}

bool MaxonMotorsNode::initialize(
	const MaxonDriverConfig & driver_config,
	const std::vector<MaxonMotorConfig> & motor_configs)
{
	shutdown_hardware();
	driver_config_ = driver_config;

	if (motor_configs.empty()) {
		RCLCPP_ERROR(get_logger(), "MaxonMotorsNode recebeu lista vazia de motores.");
		return false;
	}

	const double counts_per_rev = std::max(1.0, driver_config_.encoder_counts_per_wheel_rev);
	rad_per_count_ = kTwoPi / counts_per_rev;

#if defined(CARAMELO_HAS_PIGPIO) && CARAMELO_HAS_PIGPIO
	char * pigpio_host = driver_config_.pigpio_host.empty()
		? nullptr
		: const_cast<char *>(driver_config_.pigpio_host.c_str());
	char * pigpio_port = driver_config_.pigpio_port.empty()
		? nullptr
		: const_cast<char *>(driver_config_.pigpio_port.c_str());

	pi_handle_ = pigpio_start(pigpio_host, pigpio_port);
	if (pi_handle_ < 0) {
		RCLCPP_ERROR(
			get_logger(),
			"pigpio_start falhou com codigo %d. Host='%s' porta='%s'. "
			"Na Raspberry, confira: sudo systemctl status pigpiod",
			pi_handle_,
			driver_config_.pigpio_host.empty() ? "localhost" : driver_config_.pigpio_host.c_str(),
			driver_config_.pigpio_port.empty() ? "default" : driver_config_.pigpio_port.c_str());
		return false;
	}

	motors_.clear();
	motors_.resize(motor_configs.size());
	cb_ctx_a_.resize(motor_configs.size());
	cb_ctx_b_.resize(motor_configs.size());
	encoder_notify_.clear();
	encoder_notify_.resize(1);
	counts_size_ = motor_configs.size();
	counts_.reset(new std::atomic<int64_t>[counts_size_]);
	last_counts_.assign(motor_configs.size(), 0);

	uint32_t combined_notify_mask = 0;
	uint32_t combined_initial_level = 0;

	for (std::size_t i = 0; i < motor_configs.size(); ++i) {
		auto & motor = motors_[i];
		motor.config = motor_configs[i];

		if (motor.config.pwm_gpio == 17 || motor.config.pwm_gpio == 24) {
			motor.config.command_sign = -1.0;
			motor.config.feedback_sign = -1.0;
		}

		set_mode(pi_handle_, motor.config.pwm_gpio, PI_OUTPUT);
		set_servo_pulsewidth(pi_handle_, motor.config.pwm_gpio, neutral_pulse_width_us());

		set_mode(pi_handle_, motor.config.enc_a_gpio, PI_INPUT);
		set_mode(pi_handle_, motor.config.enc_b_gpio, PI_INPUT);
		set_pull_up_down(pi_handle_, motor.config.enc_a_gpio, PI_PUD_UP);
		set_pull_up_down(pi_handle_, motor.config.enc_b_gpio, PI_PUD_UP);
		set_glitch_filter(pi_handle_, motor.config.enc_a_gpio, 0);
		set_glitch_filter(pi_handle_, motor.config.enc_b_gpio, 0);

		const int a0 = gpio_read(pi_handle_, motor.config.enc_a_gpio);
		const int b0 = gpio_read(pi_handle_, motor.config.enc_b_gpio);
		motor.last_state.store(((a0 != 0) << 1) | (b0 != 0));
		motor.encoder_count.store(0);

		cb_ctx_a_[i] = CallbackContext{this, i};
		cb_ctx_b_[i] = CallbackContext{this, i};

		const uint32_t a_bit = bit(motor.config.enc_a_gpio);
		const uint32_t b_bit = bit(motor.config.enc_b_gpio);
		combined_notify_mask |= a_bit | b_bit;
		if (a0 != 0) {
			combined_initial_level |= a_bit;
		}
		if (b0 != 0) {
			combined_initial_level |= b_bit;
		}

		motor.callback_a = kInvalidCallbackId;
		motor.callback_b = kInvalidCallbackId;
		counts_[i].store(0);
	}

	auto & notify = encoder_notify_.front();
	notify.mask = combined_notify_mask;
	notify.last_level = combined_initial_level & notify.mask;
	notify.notify_handle = notify_open(pi_handle_);
	if (notify.notify_handle < 0) {
		RCLCPP_ERROR(
			get_logger(),
			"notify_open falhou para mascara combinada GPIO 0x%08x com codigo %d.",
			notify.mask,
			notify.notify_handle);
		shutdown_hardware();
		return false;
	}
	if (notify_begin(pi_handle_, notify.notify_handle, notify.mask) < 0) {
		RCLCPP_ERROR(
			get_logger(),
			"notify_begin falhou para mascara combinada GPIO 0x%08x.",
			notify.mask);
		shutdown_hardware();
		return false;
	}

	char devname[64];
	snprintf(devname, sizeof(devname), "/dev/pigpio%d", notify.notify_handle);
	notify.notify_fd = open(devname, O_RDONLY | O_NONBLOCK);
	if (notify.notify_fd < 0) {
		RCLCPP_ERROR(
			get_logger(),
			"Falha ao abrir %s para encoders da base: %s.",
			devname,
			std::strerror(errno));
		shutdown_hardware();
		return false;
	}

	gpioReport_t rep{};
	ssize_t n = read(notify.notify_fd, &rep, sizeof(rep));
	if (n == static_cast<ssize_t>(sizeof(rep))) {
		notify.last_level = rep.level & notify.mask;
	}

	last_update_time_ = now();
	initialized_ = true;

	encoder_threads_running_.store(true);
	encoder_threads_.clear();
	encoder_threads_.emplace_back(&MaxonMotorsNode::encoder_read_loop, this, 0);

	control_thread_running_.store(true);
	control_thread_ = std::thread(&MaxonMotorsNode::control_loop, this);

	RCLCPP_INFO(
		get_logger(),
		"MaxonMotorsNode inicializado com %zu motores via pigpio handle %d.",
		motors_.size(),
		pi_handle_);
	return true;
#else
	(void)motor_configs;
	RCLCPP_ERROR(
		get_logger(),
		"Backend pigpio nao foi habilitado neste build de caramelo_hardware. "
		"Instale headers/libs pigpio na Raspberry e recompile o pacote.");
	return false;
#endif
}

void MaxonMotorsNode::shutdown_hardware()
{
#if defined(CARAMELO_HAS_PIGPIO) && CARAMELO_HAS_PIGPIO
	control_thread_running_.store(false);
	if (control_thread_.joinable()) {
		control_thread_.join();
	}

	// Primeiro sinaliza e faz join (o poll() tem timeout de 100ms e reavalia a
	// flag); so depois fecha os fds — fechar um fd em uso por poll() de outro
	// thread e comportamento indefinido e era a causa do travamento no shutdown.
	encoder_threads_running_.store(false);
	for (auto & thread : encoder_threads_) {
		if (thread.joinable()) {
			thread.join();
		}
	}
	encoder_threads_.clear();

	for (auto & notify : encoder_notify_) {
		if (notify.notify_fd >= 0) {
			close(notify.notify_fd);
			notify.notify_fd = -1;
		}
	}

	if (pi_handle_ >= 0) {
		stop_all_motors();
		for (auto & notify : encoder_notify_) {
			if (notify.notify_handle >= 0) {
				notify_begin(pi_handle_, notify.notify_handle, 0);
			}
			if (notify.notify_handle >= 0) {
				notify_close(pi_handle_, notify.notify_handle);
				notify.notify_handle = -1;
			}
		}
		pigpio_stop(pi_handle_);
		pi_handle_ = -1;
	}
	motors_.clear();
	cb_ctx_a_.clear();
	cb_ctx_b_.clear();
	encoder_notify_.clear();
	counts_.reset();
	counts_size_ = 0;
	last_counts_.clear();
#endif
	initialized_ = false;
}

bool MaxonMotorsNode::is_initialized() const
{
	return initialized_;
}

void MaxonMotorsNode::set_command_velocity(std::size_t motor_index, double wheel_velocity_rad_s)
{
	if (motor_index >= motors_.size()) {
		return;
	}
	motors_[motor_index].command_rad_s.store(wheel_velocity_rad_s);
}

bool MaxonMotorsNode::get_velocity(std::size_t motor_index, double & velocity_rad_s) const
{
	if (motor_index >= motors_.size()) {
		return false;
	}
	velocity_rad_s = motors_[motor_index].velocity_rad_s.load();
	return true;
}

bool MaxonMotorsNode::get_feedback(
	std::size_t motor_index, double & position_rad, double & velocity_rad_s) const
{
	if (motor_index >= motors_.size()) {
		return false;
	}
	position_rad = motors_[motor_index].position_rad.load();
	velocity_rad_s = motors_[motor_index].velocity_rad_s.load();
	return true;
}

void MaxonMotorsNode::stop_all_motors()
{
#if defined(CARAMELO_HAS_PIGPIO) && CARAMELO_HAS_PIGPIO
	if (!initialized_ || pi_handle_ < 0) {
		return;
	}

	for (auto & motor : motors_) {
		motor.command_rad_s.store(0.0);
		set_servo_pulsewidth(pi_handle_, motor.config.pwm_gpio, neutral_pulse_width_us());
	}
#endif
}

void MaxonMotorsNode::encoder_callback(
	int /*pi*/, unsigned /*gpio*/, unsigned /*level*/, uint32_t /*tick*/, void * userdata)
{
	auto * ctx = static_cast<CallbackContext *>(userdata);
	if (ctx == nullptr || ctx->self == nullptr) {
		return;
	}
	ctx->self->handle_encoder_edge(ctx->motor_index);
}

void MaxonMotorsNode::handle_encoder_edge(std::size_t motor_index)
{
#if defined(CARAMELO_HAS_PIGPIO) && CARAMELO_HAS_PIGPIO
	if (motor_index >= motors_.size() || pi_handle_ < 0) {
		return;
	}

	auto & motor = motors_[motor_index];
	const int a = gpio_read(pi_handle_, motor.config.enc_a_gpio);
	const int b = gpio_read(pi_handle_, motor.config.enc_b_gpio);
	if (a < 0 || b < 0) {
		return;
	}

	const int new_state = ((a != 0) << 1) | (b != 0);
	const int prev_state = motor.last_state.exchange(new_state);
	const int idx = (prev_state << 2) | new_state;
	static constexpr int8_t kQuadTable[16] = {
		0, -1, 1, 0,
		1, 0, 0, -1,
		-1, 0, 0, 1,
		0, 1, -1, 0
	};
	const int delta = kQuadTable[idx];
	if (delta != 0) {
		motor.encoder_count.fetch_add(delta, std::memory_order_relaxed);
	}
#else
	(void)motor_index;
#endif
}

void MaxonMotorsNode::update_feedback(double dt_seconds)
{
	(void)dt_seconds;
	update_cycle();
}

void MaxonMotorsNode::apply_pwm()
{
	update_cycle();
}

void MaxonMotorsNode::encoder_read_loop(std::size_t motor_index)
{
#if defined(CARAMELO_HAS_PIGPIO) && CARAMELO_HAS_PIGPIO
	(void)motor_index;
	if (encoder_notify_.empty()) {
		return;
	}

	auto & notify = encoder_notify_.front();
	std::array<gpioReport_t, 32> reports{};
	static constexpr int8_t kQuadTable[16] = {
		0, -1, 1, 0,
		1, 0, 0, -1,
		-1, 0, 0, 1,
		0, 1, -1, 0
	};

	while (encoder_threads_running_.load()) {
		if (notify.notify_fd < 0) {
			break;
		}

		pollfd pfd;
		pfd.fd = notify.notify_fd;
		pfd.events = POLLIN;
		pfd.revents = 0;

		// Timeout finito: close() em outro thread NAO acorda um poll() bloqueado,
		// entao o timeout garante que o loop reavalie encoder_threads_running_
		// periodicamente e o shutdown consiga fazer join() sem travar (SIGKILL).
		const int poll_ret = poll(&pfd, 1, 100);
		if (poll_ret <= 0) {
			continue;
		}

		if ((pfd.revents & (POLLERR | POLLHUP | POLLNVAL)) != 0) {
			break;
		}

		if ((pfd.revents & POLLIN) == 0) {
			continue;
		}

		ssize_t nr = read(notify.notify_fd, reports.data(), reports.size() * sizeof(gpioReport_t));
		if (nr <= 0) {
			if (errno == EAGAIN || errno == EWOULDBLOCK) {
				continue;
			}
			break;
		}

		const std::size_t report_count = static_cast<std::size_t>(nr) / sizeof(gpioReport_t);
		for (std::size_t report_index = 0; report_index < report_count; ++report_index) {
			const uint32_t level = reports[report_index].level;
			const uint32_t changed = (notify.last_level ^ level) & notify.mask;

			for (std::size_t i = 0; i < motors_.size(); ++i) {
				auto & motor = motors_[i];
				const uint32_t a_bit = bit(motor.config.enc_a_gpio);
				const uint32_t b_bit = bit(motor.config.enc_b_gpio);
				if ((changed & (a_bit | b_bit)) == 0) {
					continue;
				}

				const int a_now = (level & a_bit) ? 1 : 0;
				const int b_now = (level & b_bit) ? 1 : 0;
				const int new_state = (a_now << 1) | b_now;
				const int last_state = motor.last_state.exchange(new_state);
				const int idx = (last_state << 2) | new_state;
				const int delta = kQuadTable[idx];
				if (delta != 0) {
					counts_[i].fetch_add(delta, std::memory_order_relaxed);
				}
			}

			notify.last_level = level & notify.mask;
		}
	}
#else
	(void)motor_index;
#endif
}

void MaxonMotorsNode::control_loop()
{
#if defined(CARAMELO_HAS_PIGPIO) && CARAMELO_HAS_PIGPIO
	while (control_thread_running_.load()) {
		update_cycle();
		std::this_thread::sleep_for(std::chrono::milliseconds(10));
	}
#endif
}

void MaxonMotorsNode::update_cycle()
{
#if defined(CARAMELO_HAS_PIGPIO) && CARAMELO_HAS_PIGPIO
	std::lock_guard<std::mutex> lock(update_cycle_mutex_);

	if (!initialized_ || pi_handle_ < 0) {
		return;
	}

	const auto now_time = now();
	const double dt = std::max(1e-6, (now_time - last_update_time_).seconds());
	last_update_time_ = now_time;
	std_msgs::msg::Float64MultiArray msg;
	msg.data.resize(motors_.size(), 0.0);

	for (std::size_t i = 0; i < motors_.size(); ++i) {
		auto & motor = motors_[i];
		const int64_t count_now = counts_[i].load(std::memory_order_relaxed);
		const int64_t delta_count = count_now - last_counts_[i];
		last_counts_[i] = count_now;

		const double delta_rad =
			static_cast<double>(delta_count) * rad_per_count_ * motor.config.feedback_sign;
		const double new_position = motor.position_rad.load() + delta_rad;
		const double new_velocity = delta_rad / dt;
		motor.position_rad.store(new_position);
		motor.velocity_rad_s.store(new_velocity);
		msg.data[i] = new_velocity;

		const double cmd_signed = motor.command_rad_s.load() * motor.config.command_sign;
		const int pulse_us = velocity_to_pulse_width_us(cmd_signed);
		set_servo_pulsewidth(pi_handle_, motor.config.pwm_gpio, pulse_us);
	}

	velocity_pub_->publish(msg);
#endif
}

int MaxonMotorsNode::velocity_to_pulse_width_us(double wheel_velocity_rad_s) const
{
	const double max_rad = std::max(1e-6, driver_config_.max_wheel_rad_per_sec);
	const double norm = std::clamp(wheel_velocity_rad_s / max_rad, -1.0, 1.0);

	if (norm > 0.0) {
		const double pulse_f =
			static_cast<double>(MaxonDriverConfig::kPulseUsForwardMin) +
			norm * static_cast<double>(
				MaxonDriverConfig::kPulseUsForwardMax - MaxonDriverConfig::kPulseUsForwardMin);
		const int pulse_us = static_cast<int>(std::lround(pulse_f));
		return clamp_int(
			pulse_us,
			MaxonDriverConfig::kPulseUsForwardMin,
			MaxonDriverConfig::kPulseUsForwardMax);
	}

	if (norm < 0.0) {
		const double reverse_norm = -norm;
		const double pulse_f =
			static_cast<double>(MaxonDriverConfig::kPulseUsReverseMin) -
			reverse_norm * static_cast<double>(
				MaxonDriverConfig::kPulseUsReverseMin - MaxonDriverConfig::kPulseUsReverseMax);
		const int pulse_us = static_cast<int>(std::lround(pulse_f));
		return clamp_int(
			pulse_us,
			MaxonDriverConfig::kPulseUsReverseMax,
			MaxonDriverConfig::kPulseUsReverseMin);
	}

	return neutral_pulse_width_us();
}

int MaxonMotorsNode::neutral_pulse_width_us() const
{
	return clamp_int(
		MaxonDriverConfig::kPulseUsNeutral,
		MaxonDriverConfig::kPulseUsNeutralMin,
		MaxonDriverConfig::kPulseUsNeutralMax);
}

}  // namespace mobile_base_hardware

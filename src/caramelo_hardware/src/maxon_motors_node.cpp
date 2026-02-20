#include "caramelo_hardware/maxon_motors_node.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <fcntl.h>
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

uint32_t bit(int gpio)
{
	return (1u << gpio);
}
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
		return false;
	}

	const double counts_per_rev = std::max(1.0, driver_config_.encoder_counts_per_wheel_rev);
	rad_per_count_ = kTwoPi / counts_per_rev;

#if defined(CARAMELO_HAS_PIGPIO) && CARAMELO_HAS_PIGPIO
	pi_handle_ = pigpio_start(nullptr, nullptr);
	if (pi_handle_ < 0) {
		return false;
	}

	motors_.clear();
	motors_.resize(motor_configs.size());
	cb_ctx_a_.resize(motor_configs.size());
	cb_ctx_b_.resize(motor_configs.size());
	counts_.assign(motor_configs.size(), 0);
	last_counts_.assign(motor_configs.size(), 0);
	notify_mask_ = 0;

	for (std::size_t i = 0; i < motor_configs.size(); ++i) {
		auto & motor = motors_[i];
		motor.config = motor_configs[i];

		set_mode(pi_handle_, motor.config.pwm_gpio, PI_OUTPUT);
		set_PWM_frequency(
			pi_handle_, motor.config.pwm_gpio, static_cast<unsigned>(MaxonDriverConfig::kPwmFrequencyHz));
		set_PWM_range(
			pi_handle_, motor.config.pwm_gpio, static_cast<unsigned>(MaxonDriverConfig::kPwmRange));
		set_PWM_dutycycle(pi_handle_, motor.config.pwm_gpio, neutral_duty());

		set_mode(pi_handle_, motor.config.enc_a_gpio, PI_INPUT);
		set_mode(pi_handle_, motor.config.enc_b_gpio, PI_INPUT);
		set_pull_up_down(pi_handle_, motor.config.enc_a_gpio, PI_PUD_OFF);
		set_pull_up_down(pi_handle_, motor.config.enc_b_gpio, PI_PUD_OFF);
		set_glitch_filter(pi_handle_, motor.config.enc_a_gpio, 1);
		set_glitch_filter(pi_handle_, motor.config.enc_b_gpio, 1);

		const int a0 = gpio_read(pi_handle_, motor.config.enc_a_gpio);
		const int b0 = gpio_read(pi_handle_, motor.config.enc_b_gpio);
		motor.last_state = ((a0 != 0) << 1) | (b0 != 0);

		cb_ctx_a_[i] = CallbackContext{this, i};
		cb_ctx_b_[i] = CallbackContext{this, i};

		notify_mask_ |= bit(motor.config.enc_a_gpio);
		notify_mask_ |= bit(motor.config.enc_b_gpio);

		motor.callback_a = kInvalidCallbackId;
		motor.callback_b = kInvalidCallbackId;
	}

	notify_handle_ = notify_open(pi_handle_);
	if (notify_handle_ < 0) {
		shutdown_hardware();
		return false;
	}
	if (notify_begin(pi_handle_, notify_handle_, notify_mask_) < 0) {
		shutdown_hardware();
		return false;
	}

	char devname[64];
	snprintf(devname, sizeof(devname), "/dev/pigpio%d", notify_handle_);
	notify_fd_ = open(devname, O_RDONLY | O_NONBLOCK);
	if (notify_fd_ < 0) {
		shutdown_hardware();
		return false;
	}

	gpioReport_t rep{};
	ssize_t n = read(notify_fd_, &rep, sizeof(rep));
	if (n == static_cast<ssize_t>(sizeof(rep))) {
		last_level_ = rep.level;
		last_tick_ = rep.tick;
	} else {
		uint32_t lvl = 0;
		for (std::size_t i = 0; i < motors_.size(); ++i) {
			if (gpio_read(pi_handle_, motors_[i].config.enc_a_gpio)) {
				lvl |= bit(motors_[i].config.enc_a_gpio);
			}
			if (gpio_read(pi_handle_, motors_[i].config.enc_b_gpio)) {
				lvl |= bit(motors_[i].config.enc_b_gpio);
			}
		}
		last_level_ = lvl;
		last_tick_ = get_current_tick(pi_handle_);
	}

	last_update_time_ = now();
	update_timer_ = create_wall_timer(
		std::chrono::milliseconds(100),
		std::bind(&MaxonMotorsNode::update_cycle, this));

	initialized_ = true;
	return true;
#else
	(void)motor_configs;
	return false;
#endif
}

void MaxonMotorsNode::shutdown_hardware()
{
#if defined(CARAMELO_HAS_PIGPIO) && CARAMELO_HAS_PIGPIO
	update_timer_.reset();
	if (pi_handle_ >= 0) {
		stop_all_motors();
		if (notify_handle_ >= 0) {
			notify_begin(pi_handle_, notify_handle_, 0);
		}
		for (auto & motor : motors_) {
			if (motor.callback_a >= 0) {
				callback_cancel(motor.callback_a);
				motor.callback_a = kInvalidCallbackId;
			}
			if (motor.callback_b >= 0) {
				callback_cancel(motor.callback_b);
				motor.callback_b = kInvalidCallbackId;
			}
		}
		if (notify_fd_ >= 0) {
			close(notify_fd_);
			notify_fd_ = -1;
		}
		if (notify_handle_ >= 0) {
			notify_close(pi_handle_, notify_handle_);
			notify_handle_ = -1;
		}
		pigpio_stop(pi_handle_);
		pi_handle_ = -1;
	}
	motors_.clear();
	cb_ctx_a_.clear();
	cb_ctx_b_.clear();
	counts_.clear();
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
	std::lock_guard<std::mutex> lock(state_mutex_);
	motors_[motor_index].command_rad_s = wheel_velocity_rad_s;
}

bool MaxonMotorsNode::get_feedback(
	std::size_t motor_index, double & position_rad, double & velocity_rad_s) const
{
	if (motor_index >= motors_.size()) {
		return false;
	}
	std::lock_guard<std::mutex> lock(state_mutex_);
	position_rad = motors_[motor_index].position_rad;
	velocity_rad_s = motors_[motor_index].velocity_rad_s;
	return true;
}

void MaxonMotorsNode::stop_all_motors()
{
#if defined(CARAMELO_HAS_PIGPIO) && CARAMELO_HAS_PIGPIO
	if (!initialized_ || pi_handle_ < 0) {
		return;
	}

	std::lock_guard<std::mutex> lock(state_mutex_);
	for (auto & motor : motors_) {
		motor.command_rad_s = 0.0;
		set_PWM_dutycycle(pi_handle_, motor.config.pwm_gpio, neutral_duty());
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

	const int delta = (a == b) ? 1 : -1;
	motor.encoder_count.fetch_add(delta, std::memory_order_relaxed);
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

void MaxonMotorsNode::update_cycle()
{
#if defined(CARAMELO_HAS_PIGPIO) && CARAMELO_HAS_PIGPIO
	if (!initialized_ || pi_handle_ < 0) {
		return;
	}

	const auto now_time = now();
	const double dt = std::max(1e-6, (now_time - last_update_time_).seconds());
	last_update_time_ = now_time;

	if (notify_fd_ >= 0) {
		while (true) {
			gpioReport_t r;
			ssize_t nr = read(notify_fd_, &r, sizeof(r));
			if (nr == static_cast<ssize_t>(sizeof(r))) {
				uint32_t level = r.level;
				uint32_t changed = (last_level_ ^ level) & notify_mask_;

				for (std::size_t i = 0; i < motors_.size(); ++i) {
					uint32_t aBit = bit(motors_[i].config.enc_a_gpio);
					if (changed & aBit) {
						bool aNow = (level & aBit);
						if (aNow) {
							bool bNow = (level & bit(motors_[i].config.enc_b_gpio));
							counts_[i] += (bNow ? -1 : +1);
						}
					}
				}

				last_level_ = level;
				last_tick_ = r.tick;
			} else {
				break;
			}
		}
		last_tick_ = get_current_tick(pi_handle_);
	}

	std::lock_guard<std::mutex> lock(state_mutex_);
	std_msgs::msg::Float64MultiArray msg;
	msg.data.resize(motors_.size(), 0.0);

	for (std::size_t i = 0; i < motors_.size(); ++i) {
		auto & motor = motors_[i];

		const int64_t count_now = counts_[i];
		const int64_t delta_count = count_now - last_counts_[i];
		last_counts_[i] = count_now;

		const double delta_rad =
			static_cast<double>(delta_count) * rad_per_count_ * motor.config.feedback_sign;
		motor.position_rad += delta_rad;
		motor.velocity_rad_s = delta_rad / dt;
		msg.data[i] = motor.velocity_rad_s;

		const double cmd_signed = motor.command_rad_s * motor.config.command_sign;
		const int duty = velocity_to_duty(cmd_signed);
		set_PWM_dutycycle(pi_handle_, motor.config.pwm_gpio, duty);
	}

	velocity_pub_->publish(msg);
#endif
}

int MaxonMotorsNode::velocity_to_duty(double wheel_velocity_rad_s) const
{
	const double max_rad = std::max(1e-6, driver_config_.max_wheel_rad_per_sec);
	const double norm = std::clamp(wheel_velocity_rad_s / max_rad, -1.0, 1.0);
	const double half_span = static_cast<double>(MaxonDriverConfig::kPwmRange) * 0.5;
	const int duty = static_cast<int>(
		std::lround(MaxonDriverConfig::kPwmNeutralDuty + norm * half_span));
	return clamp_int(duty, 0, MaxonDriverConfig::kPwmRange);
}

int MaxonMotorsNode::neutral_duty() const
{
	return clamp_int(MaxonDriverConfig::kPwmNeutralDuty, 0, MaxonDriverConfig::kPwmRange);
}

}  // namespace mobile_base_hardware

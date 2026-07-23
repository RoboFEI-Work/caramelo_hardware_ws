#include "caramelo_hardware/maxon_motors_node.hpp"

#include <algorithm>
#include <array>
#include <cerrno>
#include <cmath>
#include <cstdint>
#include <chrono>
#include <cstdio>
#include <cstring>

#if defined(CARAMELO_HAS_LGPIO) && CARAMELO_HAS_LGPIO
#include <lgpio.h>
#endif

namespace mobile_base_hardware
{

namespace
{
constexpr double kTwoPi = 6.28318530717958647692;
// Frequencia dos pulsos servo para os ESCs (mesma dos 50Hz default do pigpio).
constexpr int kServoFrequencyHz = 50;

int clamp_int(int value, int min_v, int max_v)
{
	return std::min(std::max(value, min_v), max_v);
}

#if defined(CARAMELO_HAS_LGPIO) && CARAMELO_HAS_LGPIO
// Trampoline C chamado pelo thread de alertas do lgpio (um thread por chip,
// entregas em ordem — sem corrida entre linhas A/B do mesmo motor).
void encoder_alerts_trampoline(int num_alerts, lgGpioAlert_p alerts, void * userdata)
{
	auto * ctx = static_cast<MaxonMotorsNode::CallbackContext *>(userdata);
	if (ctx == nullptr || ctx->self == nullptr) {
		return;
	}
	for (int i = 0; i < num_alerts; ++i) {
		const int level = alerts[i].report.level;
		if (level != 0 && level != 1) {
			continue;  // 2 = watchdog/timeout do lgpio, nao e borda real
		}
		ctx->self->handle_encoder_alert(ctx->motor_index, ctx->is_line_b, level);
	}
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

#if defined(CARAMELO_HAS_LGPIO) && CARAMELO_HAS_LGPIO
	// Abre o gpiochip do header de 40 pinos. Na Pi 5 as linhas ficam no RP1
	// (label "pinctrl-rp1", chip 4 no kernel Ubuntu 24.04; chips 0-3 sao do
	// SoC e NAO vao ao header). -1 = varre pelos labels; numero explicito no
	// URDF (<param name="gpiochip_device">) pula a deteccao.
	int chip_number = driver_config_.gpiochip_device;
	if (chip_number >= 0) {
		chip_handle_ = lgGpiochipOpen(chip_number);
	} else {
		chip_handle_ = -1;
		for (int c = 0; c < 16 && chip_handle_ < 0; ++c) {
			const int h = lgGpiochipOpen(c);
			if (h < 0) {
				continue;
			}
			lgChipInfo_t info;
			if (lgGpioGetChipInfo(h, &info) == LG_OKAY &&
				std::strstr(info.label, "rp1") != nullptr)
			{
				chip_handle_ = h;
				chip_number = c;
			} else {
				lgGpiochipClose(h);
			}
		}
		if (chip_handle_ < 0) {
			// Pi 4 e anteriores: header no gpiochip0.
			chip_number = 0;
			chip_handle_ = lgGpiochipOpen(0);
		}
	}
	if (chip_handle_ < 0) {
		RCLCPP_ERROR(
			get_logger(),
			"lgGpiochipOpen falhou com codigo %d (chip %d). Confira permissao de "
			"/dev/gpiochip* (grupo gpio) e o parametro gpiochip_device.",
			chip_handle_,
			chip_number);
		return false;
	}

	motors_.clear();
	motors_.resize(motor_configs.size());
	cb_ctx_a_.resize(motor_configs.size());
	cb_ctx_b_.resize(motor_configs.size());
	counts_size_ = motor_configs.size();
	counts_.reset(new std::atomic<int64_t>[counts_size_]);
	last_counts_.assign(motor_configs.size(), 0);

	for (std::size_t i = 0; i < motor_configs.size(); ++i) {
		auto & motor = motors_[i];
		motor.config = motor_configs[i];

		if (motor.config.pwm_gpio == 17 || motor.config.pwm_gpio == 24) {
			motor.config.command_sign = -1.0;
			motor.config.feedback_sign = -1.0;
		}

		if (lgGpioClaimOutput(chip_handle_, 0, motor.config.pwm_gpio, 0) < 0) {
			RCLCPP_ERROR(
				get_logger(),
				"lgGpioClaimOutput falhou para PWM GPIO %d.",
				motor.config.pwm_gpio);
			shutdown_hardware();
			return false;
		}
		lgTxServo(
			chip_handle_, motor.config.pwm_gpio,
			neutral_pulse_width_us(), kServoFrequencyHz, 0, 0);

		// Alerta ja reivindica a linha como entrada; LG_BOTH_EDGES cobre a
		// decodificacao de quadratura (mesmo papel do notify do pigpio).
		if (lgGpioClaimAlert(
				chip_handle_, LG_SET_PULL_UP, LG_BOTH_EDGES,
				motor.config.enc_a_gpio, -1) < 0 ||
			lgGpioClaimAlert(
				chip_handle_, LG_SET_PULL_UP, LG_BOTH_EDGES,
				motor.config.enc_b_gpio, -1) < 0)
		{
			RCLCPP_ERROR(
				get_logger(),
				"lgGpioClaimAlert falhou para encoders GPIO %d/%d.",
				motor.config.enc_a_gpio,
				motor.config.enc_b_gpio);
			shutdown_hardware();
			return false;
		}

		const int a0 = lgGpioRead(chip_handle_, motor.config.enc_a_gpio);
		const int b0 = lgGpioRead(chip_handle_, motor.config.enc_b_gpio);
		motor.last_state.store(((a0 > 0) << 1) | (b0 > 0));
		motor.encoder_count.store(0);
		counts_[i].store(0);

		cb_ctx_a_[i] = CallbackContext{this, i, false};
		cb_ctx_b_[i] = CallbackContext{this, i, true};
		lgGpioSetAlertsFunc(
			chip_handle_, motor.config.enc_a_gpio,
			&encoder_alerts_trampoline, &cb_ctx_a_[i]);
		lgGpioSetAlertsFunc(
			chip_handle_, motor.config.enc_b_gpio,
			&encoder_alerts_trampoline, &cb_ctx_b_[i]);
	}

	last_update_time_ = now();
	initialized_ = true;

	control_thread_running_.store(true);
	control_thread_ = std::thread(&MaxonMotorsNode::control_loop, this);

	RCLCPP_INFO(
		get_logger(),
		"MaxonMotorsNode inicializado com %zu motores via lgpio no gpiochip%d.",
		motors_.size(),
		chip_number);
	return true;
#else
	(void)motor_configs;
	RCLCPP_ERROR(
		get_logger(),
		"Backend lgpio nao foi habilitado neste build de caramelo_hardware. "
		"Instale liblgpio-dev na Raspberry e recompile o pacote.");
	return false;
#endif
}

void MaxonMotorsNode::shutdown_hardware()
{
#if defined(CARAMELO_HAS_LGPIO) && CARAMELO_HAS_LGPIO
	control_thread_running_.store(false);
	if (control_thread_.joinable()) {
		control_thread_.join();
	}

	if (chip_handle_ >= 0) {
		stop_all_motors();
		// Cancela os callbacks ANTES de destruir os contextos: o thread de
		// alertas do lgpio pode estar entregando bordas neste exato momento.
		for (auto & motor : motors_) {
			lgGpioSetAlertsFunc(chip_handle_, motor.config.enc_a_gpio, nullptr, nullptr);
			lgGpioSetAlertsFunc(chip_handle_, motor.config.enc_b_gpio, nullptr, nullptr);
			lgTxServo(chip_handle_, motor.config.pwm_gpio, 0, kServoFrequencyHz, 0, 0);
			lgGpioFree(chip_handle_, motor.config.pwm_gpio);
			lgGpioFree(chip_handle_, motor.config.enc_a_gpio);
			lgGpioFree(chip_handle_, motor.config.enc_b_gpio);
		}
		lgGpiochipClose(chip_handle_);
		chip_handle_ = -1;
	}
	motors_.clear();
	cb_ctx_a_.clear();
	cb_ctx_b_.clear();
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
	last_command_ns_.store(
		std::chrono::duration_cast<std::chrono::nanoseconds>(
			std::chrono::steady_clock::now().time_since_epoch()).count(),
		std::memory_order_relaxed);
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
#if defined(CARAMELO_HAS_LGPIO) && CARAMELO_HAS_LGPIO
	if (chip_handle_ < 0) {
		return;
	}

	for (auto & motor : motors_) {
		motor.command_rad_s.store(0.0);
		lgTxServo(
			chip_handle_, motor.config.pwm_gpio,
			neutral_pulse_width_us(), kServoFrequencyHz, 0, 0);
	}
#endif
}

void MaxonMotorsNode::handle_encoder_alert(
	std::size_t motor_index, bool is_line_b, int level)
{
#if defined(CARAMELO_HAS_LGPIO) && CARAMELO_HAS_LGPIO
	if (motor_index >= motors_.size() || motor_index >= counts_size_) {
		return;
	}

	// O alerta traz o nivel da PROPRIA linha; o da outra vem do estado anterior.
	// Todos os callbacks do chip chegam do mesmo thread do lgpio, em ordem —
	// unico escritor de last_state, entao load+store simples e suficiente.
	auto & motor = motors_[motor_index];
	const int prev_state = motor.last_state.load(std::memory_order_relaxed);
	const int bit_now = (level != 0) ? 1 : 0;
	const int new_state = is_line_b
		? ((prev_state & 0x2) | bit_now)
		: ((bit_now << 1) | (prev_state & 0x1));
	if (new_state == prev_state) {
		return;
	}
	motor.last_state.store(new_state, std::memory_order_relaxed);

	static constexpr int8_t kQuadTable[16] = {
		0, -1, 1, 0,
		1, 0, 0, -1,
		-1, 0, 0, 1,
		0, 1, -1, 0
	};
	const int delta = kQuadTable[(prev_state << 2) | new_state];
	if (delta != 0) {
		counts_[motor_index].fetch_add(delta, std::memory_order_relaxed);
	}
#else
	(void)motor_index;
	(void)is_line_b;
	(void)level;
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

void MaxonMotorsNode::control_loop()
{
#if defined(CARAMELO_HAS_LGPIO) && CARAMELO_HAS_LGPIO
	while (control_thread_running_.load()) {
		update_cycle();
		std::this_thread::sleep_for(std::chrono::milliseconds(10));
	}
#endif
}

void MaxonMotorsNode::update_cycle()
{
#if defined(CARAMELO_HAS_LGPIO) && CARAMELO_HAS_LGPIO
	std::lock_guard<std::mutex> lock(update_cycle_mutex_);

	if (!initialized_ || chip_handle_ < 0) {
		return;
	}

	const auto now_time = now();
	const double dt = std::max(1e-6, (now_time - last_update_time_).seconds());
	last_update_time_ = now_time;
	std_msgs::msg::Float64MultiArray msg;
	msg.data.resize(motors_.size(), 0.0);

	// Watchdog de comando: se o write() do ros2_control parar de chegar (processo
	// travado/morto), comanda neutro em vez de congelar o ultimo PWM. Critico com
	// este firmware de ESC: perda do sinal PWM provoca reversao total (ver
	// docs/esc_stm32_comportamento_e_riscos.md), entao manter pulso neutro valido
	// e sempre a opcao segura.
	const int64_t last_cmd_ns = last_command_ns_.load(std::memory_order_relaxed);
	const int64_t now_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
		std::chrono::steady_clock::now().time_since_epoch()).count();
	const bool command_stale =
		(last_cmd_ns == 0) ||
		((now_ns - last_cmd_ns) >
			static_cast<int64_t>(driver_config_.command_timeout_s * 1e9));

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

		const double cmd_signed = command_stale
			? 0.0
			: motor.command_rad_s.load() * motor.config.command_sign;
		const int pulse_us = velocity_to_pulse_width_us(cmd_signed);
		lgTxServo(chip_handle_, motor.config.pwm_gpio, pulse_us, kServoFrequencyHz, 0, 0);
	}

	velocity_pub_->publish(msg);
#endif
}

int MaxonMotorsNode::velocity_to_pulse_width_us(double wheel_velocity_rad_s) const
{
	// GUARDA CRITICA: o ros2_control inicializa os comandos das juntas como NaN
	// ate o controlador ativar. NaN aqui passava pelas comparacoes (todas
	// falsas), virava lround(NaN) e o clamp final cravava 1000us = RE MAXIMA:
	// os 4 ESCs recebiam ~8s de re total durante o carregamento do bringup
	// (flagrado com monitor pigpio nos GPIOs 17/23/24/25 em 2026-07-18).
	if (!std::isfinite(wheel_velocity_rad_s)) {
		return neutral_pulse_width_us();
	}
	// O firmware do ESC (B-G431B-ESC1) roda controle de velocidade em MALHA FECHADA
	// (FOC + sensores hall) e interpreta o pulso de forma AFIM com um PISO.
	// Firmware Caramelo 2026-07 (banda morta alargada + speed_min 300 rpm):
	//   1540us -> piso (speed_min, 300 rpm = 1.12 rad/s de roda)
	//   2000us -> escala cheia (speed_max, 5364 rpm = 20.06 rad/s de roda)
	// Inverso correto: pulso = 1540 + (|cmd| - piso) * 460 / (max - piso).
	// (Os valores 1540/460 vem das constantes kPulseUs*; o piso/max vem do URDF
	// min/max_wheel_rad_per_sec — manter os TRES sincronizados com o firmware.)
	const double floor_rad = std::max(0.0, driver_config_.min_wheel_rad_per_sec);
	const double max_rad = std::max(floor_rad + 1e-6, driver_config_.max_wheel_rad_per_sec);
	const double magnitude = std::abs(wheel_velocity_rad_s);

	// Politica "mais proximo executavel": o ESC nao gira abaixo do piso, entao
	// comandos menores que meio piso viram neutro (parado) e entre meio piso e o
	// piso viram o piso. Sem isso qualquer comando minusculo executaria >= 3.74
	// rad/s e distorceria a mistura cinematica mecanum em manobras lentas.
	if (magnitude < std::max(1e-9, 0.5 * floor_rad)) {
		return neutral_pulse_width_us();
	}

	const double clamped = std::clamp(magnitude, floor_rad, max_rad);
	const double norm = (clamped - floor_rad) / (max_rad - floor_rad);

	// Margem de partida: pulso EXATAMENTE no limiar (1540/1460us) fica na
	// fronteira de arme do firmware e, com a tolerancia do oscilador de cada
	// ESC (ate ~+-22us @1500us), a placa pode ler o pulso abaixo do limiar e
	// nao partir. 30us de margem garante partida mesmo na pior placa medida.
	constexpr int kMargemPartidaUs = 30;

	if (wheel_velocity_rad_s > 0.0) {
		const double pulse_f =
			static_cast<double>(MaxonDriverConfig::kPulseUsForwardMin) +
			norm * static_cast<double>(
				MaxonDriverConfig::kPulseUsForwardMax - MaxonDriverConfig::kPulseUsForwardMin);
		const int pulse_us = static_cast<int>(std::lround(pulse_f));
		return clamp_int(
			pulse_us,
			MaxonDriverConfig::kPulseUsForwardMin + kMargemPartidaUs,
			MaxonDriverConfig::kPulseUsForwardMax);
	}

	const double pulse_f =
		static_cast<double>(MaxonDriverConfig::kPulseUsReverseMin) -
		norm * static_cast<double>(
			MaxonDriverConfig::kPulseUsReverseMin - MaxonDriverConfig::kPulseUsReverseMax);
	const int pulse_us = static_cast<int>(std::lround(pulse_f));
	return clamp_int(
		pulse_us,
		MaxonDriverConfig::kPulseUsReverseMax,
		MaxonDriverConfig::kPulseUsReverseMin - kMargemPartidaUs);
}

int MaxonMotorsNode::neutral_pulse_width_us() const
{
	return clamp_int(
		MaxonDriverConfig::kPulseUsNeutral,
		MaxonDriverConfig::kPulseUsNeutralMin,
		MaxonDriverConfig::kPulseUsNeutralMax);
}

}  // namespace mobile_base_hardware

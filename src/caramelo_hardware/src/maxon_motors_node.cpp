#include "caramelo_hardware/maxon_motors_node.hpp"

#include <algorithm>
#include <array>
#include <cerrno>
#include <cmath>
#include <cstdint>
#include <chrono>
#include <cstdio>
#include <cstring>

#include <pthread.h>
#include <sched.h>

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

int64_t steady_now_ns()
{
	return std::chrono::duration_cast<std::chrono::nanoseconds>(
		std::chrono::steady_clock::now().time_since_epoch()).count();
}

#if defined(CARAMELO_HAS_LGPIO) && CARAMELO_HAS_LGPIO
// Trampoline C chamado pelo thread de alertas do lgpio. 2026-07-27: alerts SO
// na borda de SUBIDA do canal A (decodificacao 1x). A versao anterior (x4 com
// callbacks por linha) reconstruia a quadratura com o bit da linha irma
// CONGELADO — qualquer lote com 2+ bordas somava zero e a odometria colapsava
// em velocidade (o EKF achava o robo parado e o Nav2 rampava comando).
void encoder_alerts_trampoline(int num_alerts, lgGpioAlert_p alerts, void * userdata)
{
	auto * ctx = static_cast<MaxonMotorsNode::CallbackContext *>(userdata);
	if (ctx == nullptr || ctx->self == nullptr) {
		return;
	}
	for (int i = 0; i < num_alerts; ++i) {
		if (alerts[i].report.level != 1) {
			continue;  // so' subida real (2 = watchdog/timeout do lgpio)
		}
		ctx->self->handle_encoder_pulse(ctx->motor_index);
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
	// (label "pinctrl-rp1"; o NUMERO muda com o kernel). -1 = varre pelos
	// labels; numero explicito no URDF (<param name="gpiochip_device">) pula
	// a deteccao.
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
	counts_size_ = motor_configs.size();
	counts_.reset(new std::atomic<int64_t>[counts_size_]);
	enc_dir_.reset(new std::atomic<int>[counts_size_]);
	last_counts_.assign(motor_configs.size(), 0);

	for (std::size_t i = 0; i < motor_configs.size(); ++i) {
		auto & motor = motors_[i];
		motor.config = motor_configs[i];

		// Rodas ESQUERDAS (GPIO 17/24) montadas espelhadas: comando e feedback
		// invertidos. ATENCAO: amarrado ao NUMERO do pino — remapear pinos exige
		// revisar aqui.
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
		if (!send_servo_pulse(i, neutral_pulse_width_us(), true)) {
			RCLCPP_ERROR(
				get_logger(),
				"lgTxServo inicial (neutro) falhou para PWM GPIO %d.",
				motor.config.pwm_gpio);
			shutdown_hardware();
			return false;
		}

		// Decodificacao 1x: alerta SO na SUBIDA do canal A; sentido pelo ultimo
		// comando (enc_dir_). O canal B NAO e' mais reclamado: le-lo por evento
		// corrompia o sentido em velocidade, e como alert dobraria a taxa de
		// eventos (x4 total ~730k/s no maximo — inviavel em userspace).
		if (lgGpioClaimAlert(
				chip_handle_, LG_SET_PULL_UP, LG_RISING_EDGE,
				motor.config.enc_a_gpio, -1) < 0)
		{
			RCLCPP_ERROR(
				get_logger(),
				"Claim do encoder falhou (alert A GPIO %d).",
				motor.config.enc_a_gpio);
			shutdown_hardware();
			return false;
		}
		// Debounce de 4us no kernel: ringing na subida de A gerava 2 eventos por
		// borda real (BR contava 2x SEMPRE: marca 6,1 voltas vs encoder 11,9;
		// FR intermitente — bancada 29/07). Borda legitima mais curta do robo:
		// ~11us a 20 rad/s (fase alta ~5,5us) -> 4us filtra so o eco.
		if (lgGpioSetDebounce(chip_handle_, motor.config.enc_a_gpio, 4) < 0) {
			RCLCPP_WARN(
				get_logger(),
				"lgGpioSetDebounce falhou p/ GPIO %d — seguindo SEM filtro de eco "
				"(risco de contagem dobrada).",
				motor.config.enc_a_gpio);
		}

		counts_[i].store(0);
		// +1 ate o primeiro comando nao-neutro (roda girada na mao antes disso
		// conta com modulo certo e sinal "para frente" — limite documentado).
		enc_dir_[i].store(1);

		cb_ctx_a_[i] = CallbackContext{this, i};
		if (lgGpioSetAlertsFunc(
				chip_handle_, motor.config.enc_a_gpio,
				&encoder_alerts_trampoline, &cb_ctx_a_[i]) < 0)
		{
			RCLCPP_ERROR(
				get_logger(),
				"lgGpioSetAlertsFunc falhou para encoder A GPIO %d.",
				motor.config.enc_a_gpio);
			shutdown_hardware();
			return false;
		}
	}

	last_update_time_ = now();
	last_cycle_ns_.store(steady_now_ns());
	initialized_ = true;

	control_thread_running_.store(true);
	control_thread_ = std::thread(&MaxonMotorsNode::control_loop, this);
	failsafe_thread_running_.store(true);
	failsafe_thread_ = std::thread(&MaxonMotorsNode::failsafe_loop, this);

	RCLCPP_INFO(
		get_logger(),
		"MaxonMotorsNode inicializado com %zu motores via lgpio no gpiochip%d "
		"(decode 1x, offsets escalonados, failsafe ativo).",
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
	failsafe_thread_running_.store(false);
	if (failsafe_thread_.joinable()) {
		failsafe_thread_.join();
	}
	control_thread_running_.store(false);
	if (control_thread_.joinable()) {
		control_thread_.join();
	}

	if (chip_handle_ >= 0) {
		// Cancela os callbacks ANTES de destruir os contextos: o thread de
		// alertas do lgpio pode estar entregando bordas neste exato momento.
		for (auto & motor : motors_) {
			lgGpioSetAlertsFunc(chip_handle_, motor.config.enc_a_gpio, nullptr, nullptr);
		}
		// Shutdown ORDENADO (2026-07-27): neutro -> 120 ms no fio (>=5 pulsos,
		// o firmware ve o neutro de verdade) -> corta os pulsos (largura 0; o
		// firmware detecta a perda e PARA os motores) -> libera linhas.
		// A versao anterior cortava imediatamente apos o neutro: o neutro NUNCA
		// chegava ao fio.
		stop_all_motors();
		std::this_thread::sleep_for(std::chrono::milliseconds(120));
		for (std::size_t i = 0; i < motors_.size(); ++i) {
			lgTxServo(
				chip_handle_, motors_[i].config.pwm_gpio, 0,
				kServoFrequencyHz, servo_offset_us(i), 0);
			lgGpioFree(chip_handle_, motors_[i].config.pwm_gpio);
			lgGpioFree(chip_handle_, motors_[i].config.enc_a_gpio);
		}
		lgGpiochipClose(chip_handle_);
		chip_handle_ = -1;
	}
	motors_.clear();
	cb_ctx_a_.clear();
	counts_.reset();
	enc_dir_.reset();
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
	last_command_ns_.store(steady_now_ns(), std::memory_order_relaxed);
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

bool MaxonMotorsNode::send_servo_pulse(std::size_t motor_index, int pulse_us, bool force)
{
#if defined(CARAMELO_HAS_LGPIO) && CARAMELO_HAS_LGPIO
	if (chip_handle_ < 0 || motor_index >= motors_.size()) {
		return false;
	}
	auto & motor = motors_[motor_index];
	// So' reprograma o lgTxServo quando o valor MUDA: cada chamada cancela e
	// reinicia o trem de pulsos, e re-armar a 100 Hz um trem de 50 Hz caia
	// DENTRO do pulso alto e truncava Ton (pulso truncado cai na banda de RE
	// do firmware, que arma re' sem ARMING_TIME — "roda inverte sozinha").
	if (!force && motor.last_pulse_us == pulse_us) {
		return true;
	}
	const int rc = lgTxServo(
		chip_handle_, motor.config.pwm_gpio, pulse_us,
		kServoFrequencyHz, servo_offset_us(motor_index), 0);
	if (rc < 0) {
		// NUNCA falhar em silencio: pulso congelado sem log era o mecanismo do
		// "mandei parar e nao para" quando uma chamada falhava.
		RCLCPP_ERROR_THROTTLE(
			get_logger(), *get_clock(), 1000,
			"lgTxServo falhou (rc=%d) no GPIO %d (pulso %d us).",
			rc, motor.config.pwm_gpio, pulse_us);
		return false;
	}
	motor.last_pulse_us = pulse_us;
	return true;
#else
	(void)motor_index;
	(void)pulse_us;
	(void)force;
	return false;
#endif
}

void MaxonMotorsNode::stop_all_motors()
{
#if defined(CARAMELO_HAS_LGPIO) && CARAMELO_HAS_LGPIO
	if (chip_handle_ < 0) {
		return;
	}

	for (std::size_t i = 0; i < motors_.size(); ++i) {
		motors_[i].command_rad_s.store(0.0);
		motors_[i].moving = false;
		send_servo_pulse(i, neutral_pulse_width_us(), true);
	}
#endif
}

void MaxonMotorsNode::handle_encoder_pulse(std::size_t motor_index)
{
#if defined(CARAMELO_HAS_LGPIO) && CARAMELO_HAS_LGPIO
	if (motor_index >= counts_size_) {
		return;
	}
	// CAMINHO CRITICO (~27k chamadas/s por roda a 6 rad/s): APENAS o
	// incremento atomico. Qualquer syscall aqui (a versao anterior fazia
	// lgGpioRead do canal B) atrasa a fila de alertas e corrompe o sentido —
	// ver comentario de enc_dir_ no .hpp.
	counts_[motor_index].fetch_add(
		enc_dir_[motor_index].load(std::memory_order_relaxed),
		std::memory_order_relaxed);
#else
	(void)motor_index;
#endif
}

void MaxonMotorsNode::control_loop()
{
#if defined(CARAMELO_HAS_LGPIO) && CARAMELO_HAS_LGPIO
	// Relogio ABSOLUTO (sleep_until): o sleep_for antigo derivava com a carga
	// e o periodo real do ciclo esticava sob CPU alta.
	auto next_wake = std::chrono::steady_clock::now();
	while (control_thread_running_.load()) {
		update_cycle();
		next_wake += std::chrono::milliseconds(10);
		const auto now_tp = std::chrono::steady_clock::now();
		if (next_wake < now_tp) {
			next_wake = now_tp + std::chrono::milliseconds(10);
		}
		std::this_thread::sleep_until(next_wake);
	}
#endif
}

void MaxonMotorsNode::failsafe_loop()
{
#if defined(CARAMELO_HAS_LGPIO) && CARAMELO_HAS_LGPIO
	// Trabalho minimo e SEM o mutex do update_cycle (um holder travado e'
	// exatamente o cenario vigiado). Tenta prioridade RT — sem privilegio,
	// segue em SCHED_OTHER (ainda util: quase nunca preemptada, quase nao roda).
	sched_param sp{};
	sp.sched_priority = 10;
	if (pthread_setschedparam(pthread_self(), SCHED_FIFO, &sp) != 0) {
		RCLCPP_WARN_ONCE(
			get_logger(),
			"failsafe: sem permissao para SCHED_FIFO (ver docs/raspberry_tempo_real.md); "
			"seguindo em SCHED_OTHER.");
	}

	bool pulses_cut = false;
	while (failsafe_thread_running_.load()) {
		std::this_thread::sleep_for(std::chrono::milliseconds(50));
		if (!initialized_ || chip_handle_ < 0) {
			continue;
		}
		const int64_t stale_ns = steady_now_ns() - last_cycle_ns_.load();
		if (stale_ns < 250LL * 1000 * 1000) {
			pulses_cut = false;
			continue;
		}
		if (stale_ns < 1000LL * 1000 * 1000) {
			// Loop de controle engasgado: forca NEUTRO direto (bypass do mutex).
			RCLCPP_ERROR_THROTTLE(
				get_logger(), *get_clock(), 1000,
				"FAILSAFE: control_loop parado ha %.0f ms — forcando neutro.",
				stale_ns / 1e6);
			for (std::size_t i = 0; i < motors_.size(); ++i) {
				lgTxServo(
					chip_handle_, motors_[i].config.pwm_gpio,
					neutral_pulse_width_us(), kServoFrequencyHz,
					servo_offset_us(i), 0);
				motors_[i].last_pulse_us = neutral_pulse_width_us();
			}
		} else if (!pulses_cut) {
			// Travado de verdade: corta os pulsos — o firmware dos ESCs detecta
			// a perda de sinal e PARA os motores (~500 ms).
			RCLCPP_FATAL(
				get_logger(),
				"FAILSAFE: control_loop parado ha %.1f s — cortando os pulsos "
				"(firmware dos ESCs para os motores).",
				stale_ns / 1e9);
			for (std::size_t i = 0; i < motors_.size(); ++i) {
				lgTxServo(
					chip_handle_, motors_[i].config.pwm_gpio, 0,
					kServoFrequencyHz, servo_offset_us(i), 0);
				motors_[i].last_pulse_us = -1;
			}
			pulses_cut = true;
		}
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

	// Watchdog de comando: se o write() do ros2_control parar de chegar,
	// comanda neutro em vez de congelar o ultimo PWM.
	const int64_t last_cmd_ns = last_command_ns_.load(std::memory_order_relaxed);
	const int64_t now_ns = steady_now_ns();
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
		const int pulse_us = velocity_to_pulse_width_us(cmd_signed, motor.moving);
		motor.moving = (pulse_us != neutral_pulse_width_us());
		// Sentido de contagem do encoder acompanha o pulso comandado (espaco de
		// pulso, uniforme p/ as 4 rodas; feedback_sign converte p/ junta). Em
		// neutro mantem o ultimo (inercia). Ver enc_dir_ no .hpp.
		if (pulse_us > neutral_pulse_width_us()) {
			enc_dir_[i].store(1, std::memory_order_relaxed);
		} else if (pulse_us < neutral_pulse_width_us()) {
			enc_dir_[i].store(-1, std::memory_order_relaxed);
		}
		send_servo_pulse(i, pulse_us, false);
	}

	velocity_pub_->publish(msg);
	last_cycle_ns_.store(now_ns, std::memory_order_relaxed);
#endif
}

int MaxonMotorsNode::velocity_to_pulse_width_us(
	double wheel_velocity_rad_s, bool currently_moving) const
{
	// GUARDA CRITICA: o ros2_control inicializa os comandos das juntas como NaN
	// ate o controlador ativar. NaN aqui passava pelas comparacoes (todas
	// falsas), virava lround(NaN) e o clamp final cravava 1000us = RE MAXIMA:
	// os 4 ESCs recebiam ~8s de re total durante o carregamento do bringup
	// (flagrado com monitor pigpio nos GPIOs 17/23/24/25 em 2026-07-18).
	if (!std::isfinite(wheel_velocity_rad_s)) {
		return neutral_pulse_width_us();
	}
	// O firmware do ESC (B-G431B-ESC1) roda controle de velocidade em MALHA
	// FECHADA (FOC + halls) e interpreta o pulso de forma AFIM com um PISO.
	// speed_min/max do firmware <-> min/max_wheel_rad_per_sec do URDF: manter
	// SEMPRE casados com o binario gravado nos 4 ESCs (2026-07-27: 650 rpm ->
	// piso 2.43 rad/s de roda).
	const double floor_rad = std::max(0.0, driver_config_.min_wheel_rad_per_sec);
	const double max_rad = std::max(floor_rad + 1e-6, driver_config_.max_wheel_rad_per_sec);
	const double magnitude = std::abs(wheel_velocity_rad_s);

	// Politica "mais proximo executavel" COM HISTERESE (2026-07-27): o ESC nao
	// gira abaixo do piso. LIGAR exige |cmd| >= piso; DESLIGAR so' abaixo de
	// 0.4*piso. Sem histerese, comandos perto do limiar oscilavam 0 <-> piso a
	// 100 Hz (chattering) — pior desde que o piso dobrou (650 rpm).
	const double on_threshold = currently_moving ? 0.4 * floor_rad : floor_rad;
	if (magnitude < std::max(1e-9, on_threshold)) {
		return neutral_pulse_width_us();
	}

	const double clamped = std::clamp(magnitude, floor_rad, max_rad);
	const double norm = (clamped - floor_rad) / (max_rad - floor_rad);

	// Margem de partida: pulso EXATAMENTE no limiar (1540/1460us) fica na
	// fronteira de arme do firmware e, com a tolerancia do oscilador de cada
	// ESC (ate ~+-22us @1500us), a placa pode ler o pulso abaixo do limiar e
	// nao partir. 30us de margem garante partida mesmo na pior placa medida.
	// 2026-08-03: a reta agora ANCORA em piso+margem (1570/1430us) casada com
	// ESC_TON_MAP_MIN/ESC_TREV_MAP_MIN do firmware — antes a margem era so um
	// clamp e virava +307rpm de referencia no piso (comandado 650, rodava 957).
	// Mudar a margem/ancora la = mudar aqui.
	constexpr int kMargemPartidaUs = 30;
	// Piso da margem: com trim NEGATIVO a ancora nao pode escorregar de volta
	// para dentro da banda morta (ancora < 1540/>1460 = ESC nao parte).
	constexpr int kMargemMinimaUs = 10;

	// Trim por ramo (ver .hpp): positivo = ramo mais rapido. Desloca o mapa E
	// a ancora do clamp juntos — no piso de operacao o pulso fica cravado no
	// clamp inferior, entao trim so' no valor computado seria engolido.
	// 2026-09-01: esta funcao foi reconstruida apos o merge 101ad74, que
	// concatenou os dois lados do conflito e deixou o pacote SEM COMPILAR
	// (pulse_us redeclarado, lo/hi inexistentes). As duas intencoes sao
	// ORTOGONAIS e ambas valem: a ancora em piso+margem vem de 1ca3e21 (dev) e
	// o trim por ramo vem de 16df821 (rasp_5). ATENCAO: o trim de 8us foi
	// calibrado contra a ancora ANTIGA (1540, 26.09us por rad/s); com a ancora
	// nova (1570, 24.39us por rad/s) a inclinacao mudou -6.5% e o VALOR
	// NUMERICO precisa ser reconfirmado no teste reto de 3,4m.
	if (wheel_velocity_rad_s > 0.0) {
		const int trim = static_cast<int>(std::lround(driver_config_.pulse_trim_forward_us));
		const double pulse_f =
			static_cast<double>(MaxonDriverConfig::kPulseUsForwardMin + kMargemPartidaUs) +
			norm * static_cast<double>(
				MaxonDriverConfig::kPulseUsForwardMax - MaxonDriverConfig::kPulseUsForwardMin -
				kMargemPartidaUs) +
			static_cast<double>(trim);
		const int lo = clamp_int(
			MaxonDriverConfig::kPulseUsForwardMin + kMargemPartidaUs + trim,
			MaxonDriverConfig::kPulseUsForwardMin + kMargemMinimaUs,
			MaxonDriverConfig::kPulseUsForwardMax);
		const int pulse_us = static_cast<int>(std::lround(pulse_f));
		return clamp_int(pulse_us, lo, MaxonDriverConfig::kPulseUsForwardMax);
	}

	// Re: o pulso DESCE com a velocidade, entao "mais rapido" = subtrair o trim.
	const int trim = static_cast<int>(std::lround(driver_config_.pulse_trim_reverse_us));
	const double pulse_f =
		static_cast<double>(MaxonDriverConfig::kPulseUsReverseMin - kMargemPartidaUs) -
		norm * static_cast<double>(
			MaxonDriverConfig::kPulseUsReverseMin - kMargemPartidaUs -
			MaxonDriverConfig::kPulseUsReverseMax) -
		static_cast<double>(trim);
	const int hi = clamp_int(
		MaxonDriverConfig::kPulseUsReverseMin - kMargemPartidaUs - trim,
		MaxonDriverConfig::kPulseUsReverseMax,
		MaxonDriverConfig::kPulseUsReverseMin - kMargemMinimaUs);
	const int pulse_us = static_cast<int>(std::lround(pulse_f));
	return clamp_int(pulse_us, MaxonDriverConfig::kPulseUsReverseMax, hi);
}

int MaxonMotorsNode::neutral_pulse_width_us() const
{
	return clamp_int(
		MaxonDriverConfig::kPulseUsNeutral,
		MaxonDriverConfig::kPulseUsNeutralMin,
		MaxonDriverConfig::kPulseUsNeutralMax);
}

}  // namespace mobile_base_hardware

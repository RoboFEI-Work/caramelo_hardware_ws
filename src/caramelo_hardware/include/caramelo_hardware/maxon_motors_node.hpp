#ifndef CARAMELO_HARDWARE__MAXON_MOTORS_NODE_HPP_
#define CARAMELO_HARDWARE__MAXON_MOTORS_NODE_HPP_

#include <atomic>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

namespace mobile_base_hardware
{

struct MaxonMotorConfig
{
	int pwm_gpio = -1;
	int dir_gpio = -1;
	int enc_a_gpio = -1;
	int enc_b_gpio = -1;
	double command_sign = 1.0;
	double feedback_sign = 1.0;
};

struct MaxonDriverConfig
{
	// /dev/gpiochipN do header de 40 pinos. -1 = auto-detecta pelo label
	// ("rp1" na Pi 5; fallback gpiochip0 nas Pi 4 e anteriores).
	// 2026-07-27: agora e' lido de verdade do URDF (<param name="gpiochip_device">).
	int gpiochip_device = -1;
	// 2026-07: bandas casadas com o firmware corrigido dos ESCs (banda morta
	// alargada p/ tolerar o oscilador +-1.5% das placas): forward >=1540us,
	// reverso <=1460us, neutro 1500us. NAO usar com firmware antigo (1520/1480).
	static constexpr int kPulseUsReverseMax = 1000;
	static constexpr int kPulseUsReverseMin = 1460;
	static constexpr int kPulseUsNeutralMin = 1460;
	static constexpr int kPulseUsNeutralMax = 1540;
	static constexpr int kPulseUsForwardMin = 1540;
	static constexpr int kPulseUsForwardMax = 2000;
	static constexpr int kPulseUsNeutral =
		(kPulseUsNeutralMin + kPulseUsNeutralMax) / 2;
	// Encoder em decodificacao 1x (2026-07-27): SO bordas de subida do canal A
	// (sentido pelo nivel de B). 1024 sinais/volta de motor x gearbox 1:28.
	// A quadratura x4 (114688) gerava ate ~730k eventos/s no total — inviavel
	// em userspace (perdia bordas e esfomeava a thread de PWM do lgpio).
	double encoder_counts_per_wheel_rev = 1024.0 * 28.0;
	// Mapa AFIM do firmware Caramelo (B-G431B-ESC1 / MCSDK modificado):
	//   motor_rpm = speed_min + (pulso_us - 1540) * (speed_max - speed_min) / 460
	// speed_min/max vem do firmware (mc_parameters.c: speed_min_valueRPM) e o
	// equivalente em rad/s de RODA (gearbox 1:28) vem do URDF
	// (<param name="min/max_wheel_rad_per_sec">) — manter os DOIS casados com o
	// firmware gravado nos 4 ESCs. 2026-07-27: speed_min 650 rpm -> 2.43 rad/s.
	double min_wheel_rad_per_sec = 2.43;
	double max_wheel_rad_per_sec = 20.06;
	// Trim de calibracao POR RAMO (us; positivo = roda mais rapido naquele
	// ramo). Medido no chao 2026-07-29 (3m + trena, ida e volta): o ramo de
	// FRENTE rodava ~8% abaixo do previsto pelo mapa enquanto o de RE batia
	// exato -> robo guinava ~23 graus em 3,4m. Deficit equivalente: ~7,3us.
	// Valores vem do URDF (<param name="pulse_trim_forward/reverse_us">).
	double pulse_trim_forward_us = 0.0;
	double pulse_trim_reverse_us = 0.0;
	// Watchdog: sem set_command_velocity() por mais que isso -> PWM neutro
	// (protege contra morte do write()/controller_manager com PWM congelado).
	double command_timeout_s = 0.5;
};

class MaxonMotorsNode : public rclcpp::Node
{
public:
	MaxonMotorsNode();
	~MaxonMotorsNode() override;

	bool initialize(
		const MaxonDriverConfig & driver_config,
		const std::vector<MaxonMotorConfig> & motor_configs);

	void shutdown_hardware();
	bool is_initialized() const;

	void set_command_velocity(std::size_t motor_index, double wheel_velocity_rad_s);
	bool get_velocity(std::size_t motor_index, double & velocity_rad_s) const;
	bool get_feedback(std::size_t motor_index, double & position_rad, double & velocity_rad_s) const;

	void stop_all_motors();

	// Chamado pelo thread de alertas do lgpio a cada borda de SUBIDA do canal A
	// (decodificacao 1x; sentido pelo ULTIMO COMANDO nao-neutro — ver
	// enc_dir_). Publico apenas para o trampoline C do lgpio; nao usar
	// diretamente.
	void handle_encoder_pulse(std::size_t motor_index);

	// Contexto passado como userdata aos alertas do lgpio (idem: publico
	// apenas para o trampoline).
	struct CallbackContext
	{
		MaxonMotorsNode * self = nullptr;
		std::size_t motor_index = 0;
	};

private:
	struct MotorRuntime
	{
		MotorRuntime() = default;

		MotorRuntime(const MotorRuntime & other)
		: config(other.config),
			position_rad(other.position_rad.load()),
			velocity_rad_s(other.velocity_rad_s.load()),
			command_rad_s(other.command_rad_s.load()),
			last_pulse_us(other.last_pulse_us),
			moving(other.moving)
		{
		}

		MotorRuntime & operator=(const MotorRuntime & other)
		{
			if (this != &other) {
				config = other.config;
				position_rad.store(other.position_rad.load());
				velocity_rad_s.store(other.velocity_rad_s.load());
				command_rad_s.store(other.command_rad_s.load());
				last_pulse_us = other.last_pulse_us;
				moving = other.moving;
			}
			return *this;
		}

		MotorRuntime(MotorRuntime && other) noexcept
		: config(std::move(other.config)),
			position_rad(other.position_rad.load()),
			velocity_rad_s(other.velocity_rad_s.load()),
			command_rad_s(other.command_rad_s.load()),
			last_pulse_us(other.last_pulse_us),
			moving(other.moving)
		{
		}

		MotorRuntime & operator=(MotorRuntime && other) noexcept
		{
			if (this != &other) {
				config = std::move(other.config);
				position_rad.store(other.position_rad.load());
				velocity_rad_s.store(other.velocity_rad_s.load());
				command_rad_s.store(other.command_rad_s.load());
				last_pulse_us = other.last_pulse_us;
				moving = other.moving;
			}
			return *this;
		}

		MaxonMotorConfig config;
		std::atomic<double> position_rad{0.0};
		std::atomic<double> velocity_rad_s{0.0};
		std::atomic<double> command_rad_s{0.0};
		// Ultimo pulso efetivamente programado (us); -1 = nenhum. Usado para so'
		// reprogramar o lgTxServo quando o valor MUDA (reprogramar a 100 Hz um
		// trem de 50 Hz truncava pulsos — auditoria do port, bug #2).
		int last_pulse_us = -1;
		// Histerese do piso: ligar exige |cmd| >= floor; desligar so' abaixo de
		// 0.4*floor (mata o chattering 0 <-> piso a 100 Hz).
		bool moving = false;
	};

	void control_loop();
	void failsafe_loop();
	void update_cycle();
	// force = envia mesmo sem mudanca (init/stop/failsafe). Retorna false se o
	// lgTxServo falhar (logado com throttle; erro NUNCA e' silencioso).
	bool send_servo_pulse(std::size_t motor_index, int pulse_us, bool force);
	int velocity_to_pulse_width_us(double wheel_velocity_rad_s, bool currently_moving) const;
	int neutral_pulse_width_us() const;
	// Offset do pulso de cada roda dentro do periodo de 20 ms: 0/5/10/15 ms.
	// Com offset=0 nas 4, a thread de tx do lgpio agendava as 4 bordas para o
	// MESMO instante -> skew sistematico por ordem de atendimento (~40-80us =
	// ate ~2-3 rad/s de diferenca entre rodas; era o "curvar do nada").
	// Nota 2026-07-27: sonda trocando FL<->FR de slot provou que o slot NAO
	// influi na partida (+200ms da FL seguiu a RODA, nao o slot -> hardware
	// FL, provavel hall; ver docs). Qualquer permutacao de slots serve.
	static int servo_offset_us(std::size_t motor_index)
	{
		return static_cast<int>(motor_index) * 5000;
	}

	MaxonDriverConfig driver_config_;
	std::vector<MotorRuntime> motors_;
	std::vector<CallbackContext> cb_ctx_a_;

	double rad_per_count_ = 0.0;
	int chip_handle_ = -1;
	bool initialized_ = false;
	rclcpp::Time last_update_time_;

	rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr velocity_pub_;

	std::unique_ptr<std::atomic<int64_t>[]> counts_;
	// Sentido de contagem por motor (+1/-1), em ESPACO DE PULSO (uniforme p/
	// as 4 rodas; feedback_sign converte p/ junta). Atualizado no update_cycle
	// pelo ultimo pulso NAO-neutro; em neutro mantem (inercia segue o ultimo
	// sentido). 2026-07-27: o sentido vinha de lgGpioRead(B) DENTRO do callback
	// — a ~27k eventos/s a fila atrasava, o B lido ja tinha mudado e o sinal de
	// cada contagem virava aleatorio (comando +6 rad/s media MEDIDA -2 a -4
	// rad/s; encoder andava PARA TRAS com a roda indo para frente). Custo da
	// troca: girar a roda por fora (mao/empurrao) conta no sentido do ultimo
	// comando — limite conhecido ate termos contagem por hardware (PIO/halls).
	std::unique_ptr<std::atomic<int>[]> enc_dir_;
	std::size_t counts_size_ = 0;
	std::vector<int64_t> last_counts_;
	std::thread control_thread_;
	std::atomic<bool> control_thread_running_{false};
	// Failsafe (2026-07-27): o lgTxServo com cycles=0 e' AUTONOMO — se o
	// control_loop congelar, o ultimo pulso continua saindo para sempre, valido
	// e fresco, e nenhum watchdog (driver ou firmware) dispara. Esta thread
	// dedicada (trabalho minimo, tenta SCHED_FIFO) vigia o heartbeat do
	// update_cycle: stale > 250 ms -> neutro; > 1 s -> corta os pulsos (o
	// firmware entao PARA os motores em ~500 ms).
	std::thread failsafe_thread_;
	std::atomic<bool> failsafe_thread_running_{false};
	std::atomic<int64_t> last_cycle_ns_{0};
	mutable std::mutex update_cycle_mutex_;

	// Instante (steady clock, ns) do ultimo comando recebido via
	// set_command_velocity(); usado pelo watchdog de comando no update_cycle().
	std::atomic<int64_t> last_command_ns_{0};
};

}  // namespace mobile_base_hardware

#endif  // CARAMELO_HARDWARE__MAXON_MOTORS_NODE_HPP_

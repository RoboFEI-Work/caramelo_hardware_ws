#ifndef CARAMELO_HARDWARE__MAXON_MOTORS_NODE_HPP_
#define CARAMELO_HARDWARE__MAXON_MOTORS_NODE_HPP_

#include <atomic>
#include <cstddef>
#include <cstdint>
#include <memory>
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
	std::string pigpio_host;
	std::string pigpio_port;
	static constexpr int kPulseUsReverseMax = 1000;
	static constexpr int kPulseUsReverseMin = 1480;
	static constexpr int kPulseUsNeutralMin = 1481;
	static constexpr int kPulseUsNeutralMax = 1520;
	static constexpr int kPulseUsForwardMin = 1521;
	static constexpr int kPulseUsForwardMax = 2000;
	static constexpr int kPulseUsNeutral =
		(kPulseUsNeutralMin + kPulseUsNeutralMax) / 2;
	double encoder_counts_per_wheel_rev = 28672.0;
	double max_wheel_rad_per_sec = 21.3;
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
	void update_feedback(double dt_seconds);
	void apply_pwm();

	void stop_all_motors();

private:
	struct MotorRuntime
	{
		MotorRuntime() = default;

		MotorRuntime(const MotorRuntime & other)
		: config(other.config),
			encoder_count(other.encoder_count.load()),
			previous_count(other.previous_count),
			position_rad(other.position_rad.load()),
			velocity_rad_s(other.velocity_rad_s.load()),
			command_rad_s(other.command_rad_s.load()),
			last_state(other.last_state.load()),
			callback_a(other.callback_a),
			callback_b(other.callback_b)
		{
		}

		MotorRuntime & operator=(const MotorRuntime & other)
		{
			if (this != &other) {
				config = other.config;
				encoder_count.store(other.encoder_count.load());
				previous_count = other.previous_count;
				position_rad.store(other.position_rad.load());
				velocity_rad_s.store(other.velocity_rad_s.load());
				command_rad_s.store(other.command_rad_s.load());
				last_state.store(other.last_state.load());
				callback_a = other.callback_a;
				callback_b = other.callback_b;
			}
			return *this;
		}

		MotorRuntime(MotorRuntime && other) noexcept
		: config(std::move(other.config)),
			encoder_count(other.encoder_count.load()),
			previous_count(other.previous_count),
			position_rad(other.position_rad.load()),
			velocity_rad_s(other.velocity_rad_s.load()),
			command_rad_s(other.command_rad_s.load()),
			last_state(other.last_state.load()),
			callback_a(other.callback_a),
			callback_b(other.callback_b)
		{
		}

		MotorRuntime & operator=(MotorRuntime && other) noexcept
		{
			if (this != &other) {
				config = std::move(other.config);
				encoder_count.store(other.encoder_count.load());
				previous_count = other.previous_count;
				position_rad.store(other.position_rad.load());
				velocity_rad_s.store(other.velocity_rad_s.load());
				command_rad_s.store(other.command_rad_s.load());
				last_state.store(other.last_state.load());
				callback_a = other.callback_a;
				callback_b = other.callback_b;
			}
			return *this;
		}

		MaxonMotorConfig config;
		std::atomic<int64_t> encoder_count{0};
		int64_t previous_count = 0;
		std::atomic<double> position_rad{0.0};
		std::atomic<double> velocity_rad_s{0.0};
		std::atomic<double> command_rad_s{0.0};
		std::atomic<int> last_state{0};
		int callback_a = -1;
		int callback_b = -1;
	};

	struct CallbackContext
	{
		MaxonMotorsNode * self = nullptr;
		std::size_t motor_index = 0;
	};

	static void encoder_callback(
		int pi, unsigned gpio, unsigned level, uint32_t tick, void * userdata);
	void handle_encoder_edge(std::size_t motor_index);
	void encoder_read_loop(std::size_t motor_index);
	void control_loop();
	void update_cycle();
	int velocity_to_pulse_width_us(double wheel_velocity_rad_s) const;
	int neutral_pulse_width_us() const;

	struct EncoderNotifyRuntime
	{
		int notify_handle = -1;
		int notify_fd = -1;
		uint32_t last_level = 0;
		uint32_t mask = 0;
	};

	MaxonDriverConfig driver_config_;
	std::vector<MotorRuntime> motors_;
	std::vector<CallbackContext> cb_ctx_a_;
	std::vector<CallbackContext> cb_ctx_b_;

	double rad_per_count_ = 0.0;
	int pi_handle_ = -1;
	bool initialized_ = false;
	rclcpp::Time last_update_time_;

	rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr velocity_pub_;

	std::unique_ptr<std::atomic<int64_t>[]> counts_;
	std::size_t counts_size_ = 0;
	std::vector<int64_t> last_counts_;
	std::vector<EncoderNotifyRuntime> encoder_notify_;
	std::vector<std::thread> encoder_threads_;
	std::atomic<bool> encoder_threads_running_{false};
	std::thread control_thread_;
	std::atomic<bool> control_thread_running_{false};
};

}  // namespace mobile_base_hardware

#endif  // CARAMELO_HARDWARE__MAXON_MOTORS_NODE_HPP_

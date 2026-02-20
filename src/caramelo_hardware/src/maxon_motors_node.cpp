#include "caramelo_hardware/maxon_motors_node.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <fcntl.h>
#include <functional>
#include <sys/stat.h>
#include <sys/types.h>
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
    RCLCPP_ERROR(get_logger(), "No motors configured for MaxonMotorsNode.");
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

    const int a0 = gpio_read(pi_handle_, motor.config.enc_a_gpio);
    const int b0 = gpio_read(pi_handle_, motor.config.enc_b_gpio);
    motor.last_state = ((a0 != 0) << 1) | (b0 != 0);

    cb_ctx_a_[i] = CallbackContext{this, i};
    cb_ctx_b_[i] = CallbackContext{this, i};

    motor.callback_a = callback_ex(
      pi_handle_,
      static_cast<unsigned>(motor.config.enc_a_gpio),
      kEitherEdge,
      &MaxonMotorsNode::encoder_callback,
      &cb_ctx_a_[i]);
    motor.callback_b = callback_ex(
      pi_handle_,
      static_cast<unsigned>(motor.config.enc_b_gpio),
      kEitherEdge,
      &MaxonMotorsNode::encoder_callback,
      &cb_ctx_b_[i]);

  }

  last_update_time_ = now();

  initialized_ = true;
  return true;
#else
  (void)motor_configs;
  RCLCPP_ERROR(
    get_logger(),
    "This build was compiled without pigpio support. Install pigpio dev headers/libs and rebuild.");
  return false;
#endif
}

void MaxonMotorsNode::shutdown_hardware()
{
#if defined(CARAMELO_HAS_PIGPIO) && CARAMELO_HAS_PIGPIO
  update_timer_.reset();
  if (pi_handle_ >= 0) {
    stop_all_motors();
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
    pigpio_stop(pi_handle_);
    pi_handle_ = -1;
  }
  motors_.clear();
  cb_ctx_a_.clear();
  cb_ctx_b_.clear();
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

  const int new_state = ((a != 0) << 1) | (b != 0);
  const int transition = (motor.last_state << 2) | new_state;
  static constexpr std::array<int8_t, 16> kQuadratureDelta{
    0, -1, +1, 0,
    +1, 0, 0, -1,
    -1, 0, 0, +1,
    0, +1, -1, 0};

  const int8_t delta = kQuadratureDelta[transition];
  if (delta != 0) {
    motor.encoder_count.fetch_add(delta, std::memory_order_relaxed);
  }
  motor.last_state = new_state;
#else
  (void)motor_index;
#endif
}

void MaxonMotorsNode::update_cycle()
{
#if defined(CARAMELO_HAS_PIGPIO) && CARAMELO_HAS_PIGPIO
  (void)initialized_;
  (void)pi_handle_;
#endif
}

void MaxonMotorsNode::apply_pwm()
{
#if defined(CARAMELO_HAS_PIGPIO) && CARAMELO_HAS_PIGPIO
  if (!initialized_ || pi_handle_ < 0) {
    return;
  }

  std::lock_guard<std::mutex> lock(state_mutex_);
  for (auto & motor : motors_) {
    const double cmd_signed = motor.command_rad_s * motor.config.command_sign;
    const int duty = velocity_to_duty(cmd_signed);
    set_PWM_dutycycle(pi_handle_, motor.config.pwm_gpio, duty);
  }
#endif
}

void MaxonMotorsNode::update_feedback(double dt_seconds)
{
#if defined(CARAMELO_HAS_PIGPIO) && CARAMELO_HAS_PIGPIO
  if (!initialized_ || pi_handle_ < 0) {
    return;
  }

  const double dt = std::max(1e-6, dt_seconds);

  std::lock_guard<std::mutex> lock(state_mutex_);
  std_msgs::msg::Float64MultiArray msg;
  msg.data.resize(motors_.size(), 0.0);

  for (std::size_t i = 0; i < motors_.size(); ++i) {
    auto & motor = motors_[i];

    const int64_t count_now = motor.encoder_count.load(std::memory_order_relaxed);
    const int64_t delta_count = count_now - motor.previous_count;
    motor.previous_count = count_now;

    const double delta_rad =
      static_cast<double>(delta_count) * rad_per_count_ * motor.config.feedback_sign;
    motor.position_rad += delta_rad;
    motor.velocity_rad_s = delta_rad / dt;
    msg.data[i] = motor.velocity_rad_s;
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

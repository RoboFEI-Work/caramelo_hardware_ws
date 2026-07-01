#include "manip_hardware/arm_hardware_interface.hpp"

#include <algorithm>
#include <exception>
#include <string>

namespace arm_hardware {

hardware_interface::CallbackReturn ArmHardwareInterface::on_init
    (const hardware_interface::HardwareComponentInterfaceParams & params)
{
    if (hardware_interface::SystemInterface::on_init(params) !=
        hardware_interface::CallbackReturn::SUCCESS)
    {
        return hardware_interface::CallbackReturn::ERROR;
    }

    info_ = params.hardware_info;

    joint_names_.clear();
    joint_motor_ids_.clear();
    joint_names_.reserve(info_.joints.size());
    joint_motor_ids_.reserve(info_.joints.size());

    for (const auto & joint : info_.joints) {
        const std::string param_name = joint.name + "_motor_id";
        const auto param_it = info_.hardware_parameters.find(param_name);

        if (param_it == info_.hardware_parameters.end() || param_it->second.empty()) {
            RCLCPP_ERROR(
                get_logger(),
                "Missing hardware parameter '%s' for joint '%s'",
                param_name.c_str(),
                joint.name.c_str());
            return hardware_interface::CallbackReturn::ERROR;
        }

        try {
            joint_names_.push_back(joint.name);
            joint_motor_ids_.push_back(std::stoi(param_it->second));
        } catch (const std::exception & e) {
            RCLCPP_ERROR(
                get_logger(),
                "Invalid motor ID in parameter '%s' (value='%s'): %s",
                param_name.c_str(),
                param_it->second.c_str(),
                e.what());
            return hardware_interface::CallbackReturn::ERROR;
        }
    }

    const auto port_it = info_.hardware_parameters.find("dynamixel_port");
    if (port_it == info_.hardware_parameters.end() || port_it->second.empty()) {
        RCLCPP_ERROR(get_logger(), "Missing hardware parameter 'dynamixel_port'");
        return hardware_interface::CallbackReturn::ERROR;
    }
    port_ = port_it->second;

    const auto torque_constant_it =
        info_.hardware_parameters.find("torque_constant_nm_per_amp");
    if (torque_constant_it != info_.hardware_parameters.end()) {
        try {
            torque_constant_nm_per_amp_ =
                std::stod(torque_constant_it->second);
        } catch (const std::exception & e) {
            RCLCPP_ERROR(
                get_logger(),
                "Invalid torque_constant_nm_per_amp '%s': %s",
                torque_constant_it->second.c_str(),
                e.what());
            return hardware_interface::CallbackReturn::ERROR;
        }
    }

    driver_ = std::make_shared<XSeriesDriver>(port_);

    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn ArmHardwareInterface::on_configure
    (const rclcpp_lifecycle::State & previous_state)
{
    (void)previous_state;
    if (driver_->init() !=0) {
        return hardware_interface::CallbackReturn::ERROR;
    }
    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn ArmHardwareInterface::on_activate
    (const rclcpp_lifecycle::State & previous_state)
{
    (void)previous_state;

    for (size_t i = 0; i < joint_names_.size(); ++i) {
        set_state(
            joint_names_[i] + "/position",
            driver_->getPositionRadian(joint_motor_ids_[i]));
        driver_->activateWithPositionMode(joint_motor_ids_[i]);
    }

    // Inicializa SyncRead/SyncWrite para todos os motores de uma vez
    driver_->setupSync(joint_motor_ids_);

    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn ArmHardwareInterface::on_deactivate
    (const rclcpp_lifecycle::State & previous_state)
{
    (void)previous_state;

    for (const int motor_id : joint_motor_ids_) {
        driver_->deactivate(motor_id);
    }

    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type ArmHardwareInterface::read
    (const rclcpp::Time & time, const rclcpp::Duration & period)
{
    (void)time;
    (void)period;

    // Posição, corrente, tensão e temperatura em uma única SyncRead.
    const auto telemetry = driver_->syncReadTelemetry(joint_motor_ids_);
    for (size_t i = 0; i < joint_names_.size(); ++i) {
        if (!telemetry[i].valid) {
            continue;
        }

        const double direction = joint_motor_ids_[i] >= 6 ? -1.0 : 1.0;
        const double pos = direction * telemetry[i].position_rad;
        const double current = direction * telemetry[i].current_amp;
        const double effort = current * torque_constant_nm_per_amp_;

        set_state(joint_names_[i] + "/position", pos);
        set_state(joint_names_[i] + "/current", current);
        set_state(joint_names_[i] + "/voltage", telemetry[i].voltage_volt);
        set_state(joint_names_[i] + "/effort", effort);
        set_state(
            joint_names_[i] + "/temperature",
            telemetry[i].temperature_celsius);
    }

    return hardware_interface::return_type::OK;
}

hardware_interface::return_type ArmHardwareInterface::write
    (const rclcpp::Time & time, const rclcpp::Duration & period)
{
    (void)time;
    (void)period;

    std::vector<double> commands;
    commands.reserve(joint_names_.size());

    for (const auto & joint_name : joint_names_) {
        const double command = get_command(joint_name + "/position");
        if (std::isnan(command)) {
            return hardware_interface::return_type::OK;
        }
        commands.push_back(command);
    }

    // Aplica convenção de sinal e escreve todos os motores em uma única transação SyncWrite
    std::vector<double> adjusted_cmds(commands.size());
    for (size_t i = 0; i < joint_motor_ids_.size(); ++i) {
        adjusted_cmds[i] = (joint_motor_ids_[i] >= 6) ? -commands[i] : commands[i];
    }
    driver_->syncWritePositions(joint_motor_ids_, adjusted_cmds);

    return hardware_interface::return_type::OK;
}

} // namespace arm_hardware

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(arm_hardware::ArmHardwareInterface, hardware_interface::SystemInterface)

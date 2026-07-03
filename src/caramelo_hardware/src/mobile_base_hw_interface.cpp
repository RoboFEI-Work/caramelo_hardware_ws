#include "caramelo_hardware/mobile_base_hw_interface.hpp"

#include <array>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace mobile_base_hardware {

    namespace {
        constexpr int kPwmFrontLeft = 17;
        constexpr int kEncAFrontLeft = 5;
        constexpr int kEncBFrontLeft = 6;

        constexpr int kPwmFrontRight = 23;
        constexpr int kEncAFrontRight = 27;
        constexpr int kEncBFrontRight = 22;

        constexpr int kPwmBackLeft = 24;
        constexpr int kEncABackLeft = 16;
        constexpr int kEncBBackLeft = 26;

        constexpr int kPwmBackRight = 25;
        constexpr int kEncABackRight = 20;
        constexpr int kEncBBackRight = 21;
    } // namespace

    // Esse seria o construtor da Classe.
    // Ele é necessário para criar o objeto do tipo MobileBaseHWInterface, que é o hardware interface do ros2_control.
    hardware_interface::CallbackReturn MobileBaseHWInterface::on_init
        (const hardware_interface::HardwareComponentInterfaceParams & params)
        {
            // Igual para todos os futuros hardwares.
            if (hardware_interface::SystemInterface::on_init(params) != hardware_interface::CallbackReturn::SUCCESS) {
                return hardware_interface::CallbackReturn::ERROR;
            }

            // atributo privado para guardar as informações do hardware, que vem do arquivo de configuração (URDF)
            // Inerente do ros2_control
            info_ = params.hardware_info;

            if (info_.joints.empty()) {
                RCLCPP_ERROR(get_logger(), "Nenhuma junta foi declarada no ros2_control.");
                return hardware_interface::CallbackReturn::ERROR;
            }

            // Parametros gerais do node/driver (fixos no código)
            driver_config_ = MaxonDriverConfig{};
            const auto host_it = info_.hardware_parameters.find("pigpio_host");
            if (host_it != info_.hardware_parameters.end()) {
                driver_config_.pigpio_host = host_it->second;
            }
            const auto port_it = info_.hardware_parameters.find("pigpio_port");
            if (port_it != info_.hardware_parameters.end()) {
                driver_config_.pigpio_port = port_it->second;
            }
            // Encoder: 1024 sinais por volta do motor, gearbox 1:28 e decodificacao em quadratura x4.
            driver_config_.encoder_counts_per_wheel_rev = 1024.0 * 28.0 * 4.0;

            joint_names_.clear();
            motor_configs_.clear();
            joint_names_.reserve(info_.joints.size());
            motor_configs_.reserve(info_.joints.size());

            for (std::size_t i = 0; i < info_.joints.size(); ++i) {
                const auto & joint = info_.joints[i];
                joint_names_.push_back(joint.name);

                MaxonMotorConfig cfg;
                if (joint.name == "front_left_wheel_joint") {
                    cfg.pwm_gpio = kPwmFrontLeft;
                    cfg.enc_a_gpio = kEncAFrontLeft;
                    cfg.enc_b_gpio = kEncBFrontLeft;
                } else if (joint.name == "front_right_wheel_joint") {
                    cfg.pwm_gpio = kPwmFrontRight;
                    cfg.enc_a_gpio = kEncAFrontRight;
                    cfg.enc_b_gpio = kEncBFrontRight;
                } else if (joint.name == "back_left_wheel_joint") {
                    cfg.pwm_gpio = kPwmBackLeft;
                    cfg.enc_a_gpio = kEncABackLeft;
                    cfg.enc_b_gpio = kEncBBackLeft;
                } else if (joint.name == "back_right_wheel_joint") {
                    cfg.pwm_gpio = kPwmBackRight;
                    cfg.enc_a_gpio = kEncABackRight;
                    cfg.enc_b_gpio = kEncBBackRight;
                }

                if (cfg.pwm_gpio < 0 || cfg.enc_a_gpio < 0 || cfg.enc_b_gpio < 0) {
                    RCLCPP_ERROR(get_logger(), "Pinos invalidos para a junta '%s'.", joint.name.c_str());
                    return hardware_interface::CallbackReturn::ERROR;
                }

                motor_configs_.push_back(cfg);
            }

            front_left_motor_id_ = 0;
            front_right_motor_id_ = 1;
            back_left_motor_id_ = 2;
            back_right_motor_id_ = 3;

            //obrigatório retornar SUCCESS ou ERROR, para o ros2_control saber se a inicialização foi bem sucedida ou não.
            return hardware_interface::CallbackReturn::SUCCESS;
        }

    hardware_interface::CallbackReturn MobileBaseHWInterface::on_configure
        (const rclcpp_lifecycle::State & previous_state)
        {
            (void)previous_state; // para evitar warning de variável não utilizada

            // Aqui é onde você pode configurar o hardware e abrir comunicação.
            driver_ = std::make_shared<MaxonMotorsNode>();
            if (!driver_->initialize(driver_config_, motor_configs_)) {
                RCLCPP_ERROR(
                    get_logger(),
                    "Falha ao inicializar MaxonMotorsNode. Verifique pigpiod, GPIOs, permissao e alimentacao da base.");
                driver_->shutdown_hardware();
                driver_.reset();
                return hardware_interface::CallbackReturn::ERROR;
            }
            
            /** Adiciona o node do driver ao executor dedicado e inicia a thread de spin
             *  para processar os callbacks do driver em paralelo. */
            node_executor_.add_node(driver_);
            node_spin_thread_ = std::thread([this]() {node_executor_.spin();});

            return hardware_interface::CallbackReturn::SUCCESS;
        }

    hardware_interface::CallbackReturn MobileBaseHWInterface::on_activate
        (const rclcpp_lifecycle::State & previous_state)
        {
            (void)previous_state;

            set_state("front_left_wheel_joint/velocity", 0.0);
            set_state("front_right_wheel_joint/velocity", 0.0);
            set_state("back_left_wheel_joint/velocity", 0.0);
            set_state("back_right_wheel_joint/velocity", 0.0);

            set_state("front_left_wheel_joint/position", 0.0);
            set_state("front_right_wheel_joint/position", 0.0);
            set_state("back_left_wheel_joint/position", 0.0);
            set_state("back_right_wheel_joint/position", 0.0);

            driver_->stop_all_motors();
            return hardware_interface::CallbackReturn::SUCCESS;
        }

    hardware_interface::CallbackReturn MobileBaseHWInterface::on_deactivate
        (const rclcpp_lifecycle::State & previous_state)
        {
            (void)previous_state;

            node_executor_.cancel();
            if (node_spin_thread_.joinable()) {
                node_spin_thread_.join();
            }

            if (driver_) {
                driver_->stop_all_motors();
                driver_->shutdown_hardware();
                node_executor_.remove_node(driver_);
                driver_.reset();
            }

            return hardware_interface::CallbackReturn::SUCCESS;
        }

    hardware_interface::return_type MobileBaseHWInterface::read
        (const rclcpp::Time & time, const rclcpp::Duration & period)
        {
            (void)time;

            if (!driver_ || !driver_->is_initialized()) {
                return hardware_interface::return_type::OK;
            }

            double front_left_velocity = 0.0;
            driver_->get_velocity(front_left_motor_id_, front_left_velocity);

            double front_right_velocity = 0.0;
            driver_->get_velocity(front_right_motor_id_, front_right_velocity);

            double back_left_velocity = 0.0;
            driver_->get_velocity(back_left_motor_id_, back_left_velocity);

            double back_right_velocity = 0.0;
            driver_->get_velocity(back_right_motor_id_, back_right_velocity);

            if (std::abs(front_left_velocity) < 0.03) { front_left_velocity = 0.0; }
            if (std::abs(front_right_velocity) < 0.03) { front_right_velocity = 0.0; }
            if (std::abs(back_left_velocity) < 0.03) { back_left_velocity = 0.0; }
            if (std::abs(back_right_velocity) < 0.03) { back_right_velocity = 0.0; }

            set_state("front_left_wheel_joint/velocity", front_left_velocity);
            set_state("front_right_wheel_joint/velocity", front_right_velocity);
            set_state("back_left_wheel_joint/velocity", back_left_velocity);
            set_state("back_right_wheel_joint/velocity", back_right_velocity);

            // Para calcular a posição, basta integrar a velocidade ao longo do tempo.
            set_state("front_left_wheel_joint/position", get_state("front_left_wheel_joint/position") + front_left_velocity * period.seconds());
            set_state("front_right_wheel_joint/position", get_state("front_right_wheel_joint/position") + front_right_velocity * period.seconds());
            set_state("back_left_wheel_joint/position", get_state("back_left_wheel_joint/position") + back_left_velocity * period.seconds());
            set_state("back_right_wheel_joint/position", get_state("back_right_wheel_joint/position") + back_right_velocity * period.seconds());

            // RCLCPP_INFO(get_logger(),
            //             "front_left vel: %lf, front_right vel: %lf, back_left vel: %lf, back_right vel: %lf, ",
            //             front_left_velocity, front_right_velocity, back_left_velocity, back_right_velocity);

            return hardware_interface::return_type::OK;
        }

    hardware_interface::return_type MobileBaseHWInterface::write
        (const rclcpp::Time & time, const rclcpp::Duration & period)
        {
            (void)time;
            (void)period;

            if (!driver_ || !driver_->is_initialized()) {
                return hardware_interface::return_type::OK;
            }

            driver_->set_command_velocity(front_left_motor_id_, get_command("front_left_wheel_joint/velocity"));
            driver_->set_command_velocity(front_right_motor_id_, get_command("front_right_wheel_joint/velocity"));
            driver_->set_command_velocity(back_left_motor_id_, get_command("back_left_wheel_joint/velocity"));
            driver_->set_command_velocity(back_right_motor_id_, get_command("back_right_wheel_joint/velocity"));
            
            // RCLCPP_INFO(get_logger(),
            //             "front_left vel cmd: %lf, front_right vel cmd: %lf, back_left vel cmd: %lf, back_right vel cmd: %lf",
            //             get_command("front_left_wheel_joint/velocity"),
            //             get_command("front_right_wheel_joint/velocity"),
            //             get_command("back_left_wheel_joint/velocity"),
            //             get_command("back_right_wheel_joint/velocity"));

            return hardware_interface::return_type::OK;
        }

} // namespace mobile_base_hardware

// Essa macro é necessária para registrar a classe MobileBaseHWInterface como um plugin do tipo hardware_interface::SystemInterface, para que o ros2_control possa carregá-la dinamicamente.
#include "pluginlib/class_list_macros.hpp"
// Essa macro é necessária para registrar a classe MobileBaseHWInterface como um plugin do tipo hardware_interface::SystemInterface, para que o ros2_control possa carregá-la dinamicamente.
// provide -> namespace :: nome da classe, nome da classe pai (interface) :: tipo do plugin (hardware_interface::SystemInterface)
PLUGINLIB_EXPORT_CLASS(mobile_base_hardware::MobileBaseHWInterface, hardware_interface::SystemInterface)

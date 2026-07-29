#include "caramelo_hardware/mobile_base_hw_interface.hpp"

#include <array>
#include <string>
#include <utility>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace mobile_base_hardware {

    namespace {
        // 2026-07-29: canais A e B TROCADOS de proposito (contamos pelo B
        // fisico). Captura com gpiomon provou: o A fisico tem quique na borda
        // de DESCIDA (15304 descidas duplas em 15516 ciclos na BR); com o
        // motor energizado o ruido das fases transforma o quique em SUBIDA
        // fantasma -> contagem 2x (marca 6,1 voltas vs encoder 11,9). O B
        // fisico e' limpo (1 anomalia em 15k ciclos). Como o sentido vem do
        // COMANDO (enc_dir_), so o pino "A" (contado) importa — trocar aqui
        // e' o conserto inteiro. Fiacao fisica: FL A5/B6, FR A27/B22,
        // BL A16/B26, BR A20/B21.
        constexpr int kPwmFrontLeft = 17;
        constexpr int kEncAFrontLeft = 6;
        constexpr int kEncBFrontLeft = 5;

        constexpr int kPwmFrontRight = 23;
        constexpr int kEncAFrontRight = 22;
        constexpr int kEncBFrontRight = 27;

        constexpr int kPwmBackLeft = 24;
        constexpr int kEncABackLeft = 26;
        constexpr int kEncBBackLeft = 16;

        constexpr int kPwmBackRight = 25;
        constexpr int kEncABackRight = 21;
        constexpr int kEncBBackRight = 20;
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
            // 2026-07-27: gpiochip_device agora e' LIDO DE VERDADE do URDF (antes
            // era so' documentado e ignorado — auditoria do port lgpio, bug #5).
            // -1 = auto-detecta pelo label "rp1" (Pi 5) com fallback gpiochip0.
            const auto chip_it = info_.hardware_parameters.find("gpiochip_device");
            if (chip_it != info_.hardware_parameters.end()) {
                try {
                    driver_config_.gpiochip_device = std::stoi(chip_it->second);
                } catch (const std::exception &) {
                    RCLCPP_WARN(
                        get_logger(),
                        "Parametro 'gpiochip_device' invalido ('%s'); usando auto-deteccao.",
                        chip_it->second.c_str());
                }
            }

            // Mapa afim do ESC em rad/s de RODA (piso e escala cheia), configuravel
            // pelo URDF para permitir recalibracao sem recompilar. Ver
            // docs/esc_stm32_comportamento_e_riscos.md para a origem dos valores.
            const auto read_double_param =
                [this](const char * name, double & target) {
                    const auto it = info_.hardware_parameters.find(name);
                    if (it == info_.hardware_parameters.end()) {
                        return;
                    }
                    try {
                        target = std::stod(it->second);
                    } catch (const std::exception &) {
                        RCLCPP_WARN(
                            get_logger(),
                            "Parametro '%s' invalido ('%s'); mantendo %.3f.",
                            name, it->second.c_str(), target);
                    }
                };
            read_double_param("min_wheel_rad_per_sec", driver_config_.min_wheel_rad_per_sec);
            read_double_param("max_wheel_rad_per_sec", driver_config_.max_wheel_rad_per_sec);
            read_double_param("command_timeout_s", driver_config_.command_timeout_s);
            // Encoder: 1024 sinais por volta do motor x gearbox 1:28, decodificacao
            // 1x (SO borda de subida do canal A; sentido pelo nivel de B).
            // 2026-07-27: era x4 (114688) com alerts em TODAS as bordas de A e B —
            // ~730k eventos/s no total, inviavel em userspace (perdia bordas e
            // matava a thread de PWM do lgpio). 28672/volta sobra para odometria a
            // 100 Hz. Solucao plena (PIO/halls) registrada como pendencia.
            driver_config_.encoder_counts_per_wheel_rev = 1024.0 * 28.0;

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
                    "Falha ao inicializar MaxonMotorsNode. Verifique /dev/gpiochip* (grupo gpio), liblgpio, GPIOs e alimentacao da base.");
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

            // Semente das posicoes: captura a posicao atual acumulada do encoder para que
            // o primeiro read() nao produza salto de posicao nem pico de velocidade.
            for (std::size_t motor_id = 0; motor_id < last_wheel_position_rad_.size(); ++motor_id) {
                double position_rad = 0.0;
                double velocity_rad_s = 0.0;
                if (driver_ && driver_->get_feedback(motor_id, position_rad, velocity_rad_s)) {
                    last_wheel_position_rad_[motor_id] = position_rad;
                }
            }

            // Comandos comecam como NaN no ros2_control ate o controlador ativar;
            // zera-os aqui para o write() nunca propagar NaN ao driver (NaN
            // virava pulso de 1000us = re maxima nos 4 ESCs durante o bringup).
            set_command("front_left_wheel_joint/velocity", 0.0);
            set_command("front_right_wheel_joint/velocity", 0.0);
            set_command("back_left_wheel_joint/velocity", 0.0);
            set_command("back_right_wheel_joint/velocity", 0.0);

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

            // Pares (junta, motor_id) na MESMA ordem definida em on_init()
            // (front_left=0, front_right=1, back_left=2, back_right=3).
            static const std::array<std::pair<const char *, std::size_t>, 4> kWheels = {{
                {"front_left_wheel_joint", 0},
                {"front_right_wheel_joint", 1},
                {"back_left_wheel_joint", 2},
                {"back_right_wheel_joint", 3},
            }};

            const double dt = period.seconds();

            for (const auto & [joint, motor_id] : kWheels) {
                double position_rad = 0.0;
                double velocity_rad_s = 0.0;
                // Posicao REAL acumulada do encoder (calculada a ~100Hz no driver),
                // em vez de reintegrar a velocidade no ciclo do controller_manager.
                if (!driver_->get_feedback(motor_id, position_rad, velocity_rad_s)) {
                    continue;
                }

                // Delta real do encoder desde o ultimo read().
                const double delta_rad = position_rad - last_wheel_position_rad_[motor_id];
                last_wheel_position_rad_[motor_id] = position_rad;

                // Velocidade = media exata no periodo do controller, derivada da posicao
                // acumulada do encoder. Mais suave que a velocidade diferenciada a 100Hz
                // no driver e consistente com a posicao exportada: a odometria integrada
                // pelo mecanum_drive_controller reproduz o deslocamento real das rodas.
                // Fallback para a velocidade do driver se o periodo for degenerado.
                const double velocity_state = (dt > 1e-6) ? (delta_rad / dt) : velocity_rad_s;

                // IMPORTANTE: sem deadband no feedback. O controlador integra a velocidade
                // das rodas para gerar a odometria; zerar velocidades pequenas acumula erro
                // sistematico no /odom/wheel (deadband so faria sentido no comando, nunca
                // na leitura do encoder).
                const std::string joint_name(joint);
                set_state(joint_name + "/velocity", velocity_state);

                // Posicao continua: acumula o delta real do encoder (nao a integral da
                // velocidade), entao a posicao exportada segue exatamente o encoder.
                set_state(joint_name + "/position", get_state(joint_name + "/position") + delta_rad);
            }

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

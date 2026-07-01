#ifndef XM540_DRIVER_HPP
#define XM540_DRIVER_HPP

#define PROTOCOL_VERSION 2.0
#define BAUDRATE 1000000

#define ADDR_OPERATING_MODE 11
#define ADDR_TORQUE_ENABLE 64
#define ADDR_GOAL_VELOCITY 104
#define ADDR_GOAL_POSITION 116
#define ADDR_PRESENT_PWM 124
#define ADDR_PRESENT_CURRENT 126
#define ADDR_PRESENT_VELOCITY 128
#define ADDR_PRESENT_POSITION 132
#define ADDR_PRESENT_INPUT_VOLTAGE 144
#define ADDR_PRESENT_TEMPERATURE 146

#define LEN_GOAL_POSITION    4
#define LEN_PRESENT_POSITION 4
#define ADDR_TELEMETRY_START ADDR_PRESENT_PWM
#define LEN_TELEMETRY_BLOCK 23

#define RAD_TO_DXL_POSITION 651.088636364 // 1 / 0.0174533 / 0.088
#define RAD_S_TO_RPM 9.549 // 60 / (2*PI)
#define RPM_TO_DXL_VELOCITY 4.366812227 // X-series: 1 / 0.229 RPM per unit
#define DXL_CURRENT_TO_AMP 0.00269
#define DXL_VOLTAGE_TO_VOLT 0.1

#define OPERATING_MODE_VELOCITY 1
#define OPERATING_MODE_POSITION 3

#include <dynamixel_sdk/dynamixel_sdk.h>
#include <iostream>
#include <vector>
#include <cstdint>

struct XSeriesTelemetry
{
    bool valid{false};
    double position_rad{0.0};
    double current_amp{0.0};
    double voltage_volt{0.0};
    double temperature_celsius{0.0};
};

class XSeriesDriver {
    public:
        XSeriesDriver(std::string device_name)
        : group_sync_read_(nullptr)
        , group_sync_write_(nullptr)
        {
            portHandler_ = dynamixel::PortHandler::getPortHandler(device_name.c_str());
            packetHandler_ = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);
        }

        ~XSeriesDriver()
        {
            delete group_sync_read_;
            delete group_sync_write_;
        }
    
        int init() {
            std::cout << "Initializing connection with robot." << std::endl;
    
            // Open port
            if (portHandler_->openPort()) {
                std::cout << "Succeeded to open the port!" << std::endl;
            }
            else {
                std::cout << "Failed to open the port!" << std::endl;
                return -1;
            }
    
            // Set port baudrate
            if (portHandler_->setBaudRate(BAUDRATE)) {
                std::cout << "Succeeded to change the baudrate!" << std::endl;
            }
            else {
                std::cout << "Failed to change the baudrate!" << std::endl;
                return -1;
            } 
            return 0;
        }

        /// Inicializa GroupSyncRead e GroupSyncWrite para todos os IDs de uma vez.
        /// Deve ser chamado uma vez após ativar todos os motores.
        void setupSync(const std::vector<int> & ids)
        {
            delete group_sync_read_;
            delete group_sync_write_;

            group_sync_read_ = new dynamixel::GroupSyncRead(
                portHandler_,
                packetHandler_,
                ADDR_TELEMETRY_START,
                LEN_TELEMETRY_BLOCK);
            group_sync_write_ = new dynamixel::GroupSyncWrite(
                portHandler_, packetHandler_, ADDR_GOAL_POSITION, LEN_GOAL_POSITION);

            for (int id : ids) {
                group_sync_read_->addParam(static_cast<uint8_t>(id));
            }
        }

        /// Lê posição e telemetria de todos os motores em uma transação serial.
        std::vector<XSeriesTelemetry> syncReadTelemetry(
            const std::vector<int> & ids)
        {
            std::vector<XSeriesTelemetry> telemetry(ids.size());
            const int communication_result = group_sync_read_->txRxPacket();
            if (communication_result != COMM_SUCCESS) {
                std::cerr
                    << "Dynamixel SyncRead failed: "
                    << packetHandler_->getTxRxResult(communication_result)
                    << std::endl;
                return telemetry;
            }

            for (size_t i = 0; i < ids.size(); ++i) {
                const auto id = static_cast<uint8_t>(ids[i]);
                if (!group_sync_read_->isAvailable(
                        id,
                        ADDR_TELEMETRY_START,
                        LEN_TELEMETRY_BLOCK)) {
                    continue;
                }

                const auto raw_position = static_cast<int32_t>(
                    group_sync_read_->getData(
                        id,
                        ADDR_PRESENT_POSITION,
                        LEN_PRESENT_POSITION));
                const auto raw_current = static_cast<int16_t>(
                    group_sync_read_->getData(
                        id,
                        ADDR_PRESENT_CURRENT,
                        2));
                const auto raw_voltage = static_cast<uint16_t>(
                    group_sync_read_->getData(
                        id,
                        ADDR_PRESENT_INPUT_VOLTAGE,
                        2));
                const auto raw_temperature = static_cast<uint8_t>(
                    group_sync_read_->getData(
                        id,
                        ADDR_PRESENT_TEMPERATURE,
                        1));

                telemetry[i].position_rad =
                    static_cast<double>(raw_position - 2048) /
                    RAD_TO_DXL_POSITION;
                telemetry[i].current_amp =
                    static_cast<double>(raw_current) *
                    DXL_CURRENT_TO_AMP;
                telemetry[i].voltage_volt =
                    static_cast<double>(raw_voltage) *
                    DXL_VOLTAGE_TO_VOLT;
                telemetry[i].temperature_celsius =
                    static_cast<double>(raw_temperature);
                telemetry[i].valid = true;
            }
            return telemetry;
        }

        /// Escreve posições em todos os motores em uma única transação serial.
        /// 'cmds_rad' deve estar na mesma ordem de 'ids' e já com sinal correto aplicado.
        void syncWritePositions(const std::vector<int> & ids, const std::vector<double> & cmds_rad)
        {
            group_sync_write_->clearParam();
            for (size_t i = 0; i < ids.size(); ++i) {
                int32_t dxl_cmd = static_cast<int32_t>(cmds_rad[i] * RAD_TO_DXL_POSITION + 2048);
                uint8_t param[4];
                param[0] = DXL_LOBYTE(DXL_LOWORD(dxl_cmd));
                param[1] = DXL_HIBYTE(DXL_LOWORD(dxl_cmd));
                param[2] = DXL_LOBYTE(DXL_HIWORD(dxl_cmd));
                param[3] = DXL_HIBYTE(DXL_HIWORD(dxl_cmd));
                group_sync_write_->addParam(static_cast<uint8_t>(ids[i]), param);
            }
            group_sync_write_->txPacket();
        }
    
        void activateWithPositionMode(int dxl_id)
        {
            std::cout << "Activate motor" << std::endl;
    
            // Set Position Control Mode
            packetHandler_->write1ByteTxRx(portHandler_, dxl_id, ADDR_OPERATING_MODE, OPERATING_MODE_POSITION);
            
            // Enable Torque
            packetHandler_->write1ByteTxRx(portHandler_, dxl_id, ADDR_TORQUE_ENABLE, 1);
        }

        void activateWithVelocityMode(int dxl_id)
        {
            std::cout << "Activate motor" << std::endl;
    
            // Set Velocity Control Mode
            packetHandler_->write1ByteTxRx(portHandler_, dxl_id, ADDR_OPERATING_MODE, OPERATING_MODE_VELOCITY);
            
            // Enable Torque
            packetHandler_->write1ByteTxRx(portHandler_, dxl_id, ADDR_TORQUE_ENABLE, 1);
        }
    
        void deactivate(int dxl_id)
        {
            std::cout << "Deactivate motor" << std::endl;
    
            // Disable Torque
            packetHandler_->write1ByteTxRx(portHandler_, dxl_id, ADDR_TORQUE_ENABLE, 0);
        }
    
        // --- Métodos individuais mantidos para uso pontual (on_activate, etc.) ---

        void setTargetPositionRadian(int dxl_id, double command) 
        {
            int dxl_cmd = command * RAD_TO_DXL_POSITION + 2048;
            packetHandler_->write4ByteTxRx(portHandler_, dxl_id, ADDR_GOAL_POSITION, dxl_cmd);
        }
    
        void setTargetVelocityRadianPerSec(int dxl_id, double command)
        {
            int dxl_cmd = command * RAD_S_TO_RPM * RPM_TO_DXL_VELOCITY;
            packetHandler_->write4ByteTxRx(portHandler_, dxl_id, ADDR_GOAL_VELOCITY, dxl_cmd);
        }
    
        double getPositionRadian(int dxl_id) 
        {
            int32_t dxl_present_position = 0;
            packetHandler_->read4ByteTxRx(portHandler_, dxl_id, ADDR_PRESENT_POSITION, (uint32_t*)&dxl_present_position);
            return (double)(dxl_present_position - 2048) / RAD_TO_DXL_POSITION;
        }
    
        double getVelocityRadianPerSec(int dxl_id)
        {
            int32_t dxl_present_velocity = 0;
            packetHandler_->read4ByteTxRx(portHandler_, dxl_id, ADDR_PRESENT_VELOCITY, (uint32_t*)&dxl_present_velocity);
            double velocity = (double)dxl_present_velocity / RPM_TO_DXL_VELOCITY / RAD_S_TO_RPM ;
            return velocity;
        }
    
    private:
        dynamixel::PortHandler *portHandler_;
        dynamixel::PacketHandler *packetHandler_;

        dynamixel::GroupSyncRead  *group_sync_read_;
        dynamixel::GroupSyncWrite *group_sync_write_;
};

using XM540Driver = XSeriesDriver;

#endif

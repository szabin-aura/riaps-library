

#ifndef MODBUSUART_H
#define MODBUSUART_H
#include <base/ModbusUARTBase.h>
// riaps:keep_header:begin
//#include "time_macros.h"
#include <memory>
#include <modbus/modbus.h>
#include <modbus/modbus-rtu.h>
#include <tuple>
// riaps:keep_header:end>>

namespace riapsmodbuscreqrepuart {
    namespace components {
        class ModbusUART : public ModbusUARTBase {
        public:
            ModbusUART(const py::object*  parent_actor     ,
                          const py::dict     actor_spec       ,
                          const py::dict     type_spec        ,
                          const std::string& name             ,
                          const std::string& type_name        ,
                          const py::dict     args             ,
                          const std::string& application_name ,
                          const std::string& actor_name       );


            virtual void OnModbusrepport() override;
            virtual void OnClock() override;

            virtual ~ModbusUART();

            // riaps:keep_decl:begin
        private:
            struct SerialPortConfig {
                std::string portname;
                int baudrate;
                int bytesize;
                char parity;
                int stopbits;
            };

            pid_t PID_;
            bool debug_mode_;
            struct SerialPortConfig port_config_;
            int port_slave_address_;
            int port_serial_mode_;
            modbus_t* ctx_;
            int nb_holding_regs_;
            int nb_input_regs_;
            int nb_coil_bits_;
            int nb_discrete_bits_;
            std::unique_ptr<uint16_t[]> holding_regs_;
            std::unique_ptr<uint16_t[]> input_regs_;
            std::unique_ptr<uint8_t[]> coil_bits_;
            std::unique_ptr<uint8_t[]> discrete_bits_;
            bool port_open_;

            // time testing
            //struct timespec resolution_ = {0,0};
            //struct timespec preobservations_ = {0,0};
            //struct timespec postobservations_ = {0,0};
            //struct timespec postpostobservations_ = {0,0};
            //system_timespec result_;

            int SendModbusCommand(riapsmodbuscreqrepuart::messages::CommandFormat::Reader& message);
            bool StartRTUModbus(int serialmode);
            void CloseModbus(void);
            bool IsModbusReady(void);
            int ReadCoilBits(int addr, int nb);
            int ReadInputBits(int addr, int nb);
            int ReadHoldingRegs(int addr, int nb);
            int ReadInputRegs(int addr, int nb);
            int WriteCoilBit(int addr, int status);
            int WriteHoldingReg(int addr, int value);
            int WriteCoilBits(int addr, int nb);
            int WriteHoldingRegs(int addr, int nb);
            int WriteReadHoldingRegs(int write_addr, int write_nb, int read_addr, int read_nb);
            // riaps:keep_decl:end
        };
    }
}

std::unique_ptr<riapsmodbuscreqrepuart::components::ModbusUART>
create_component_py(const py::object *parent_actor,
                    const py::dict actor_spec,
                    const py::dict type_spec,
                    const std::string &name,
                    const std::string &type_name,
                    const py::dict args,
                    const std::string &application_name,
const std::string &actor_name);

#endif // MODBUSUART_H





#include <componentmodel/r_pyconfigconverter.h>
#include <base/ModbusUARTBase.h>

using namespace std;
using namespace riaps::ports;

namespace riapsmodbuscreqrepuart {
    namespace components {
        ModbusUARTBase::ModbusUARTBase(const py::object*  parent_actor     ,
                          const py::dict     actor_spec       ,
                          const py::dict     type_spec        ,
                          const std::string& name             ,
                          const std::string& type_name        ,
                          const py::dict     args             ,
                          const std::string& application_name ,
                          const std::string& actor_name       ) : ComponentBase(application_name, actor_name){
            auto conf = PyConfigConverter::convert(type_spec, actor_spec, args);
            conf.component_name = name;
            conf.component_type = type_name;
            conf.is_device=false;
            set_config(conf);
        }

        tuple<MessageReader<messages::CommandFormat>, PortError> ModbusUARTBase::RecvModbusrepport() {
            auto port = GetPortAs<riaps::ports::ResponsePort>(PORT_REP_MODBUSREPPORT);
            auto [msg_bytes, error] = port->Recv();
            MessageReader<messages::CommandFormat> reader(msg_bytes);
            return make_tuple(reader, error);
        }

        timespec ModbusUARTBase::RecvClock() {
            auto port = GetPortAs<riaps::ports::PeriodicTimer>(PORT_TIMER_CLOCK);
            return port->Recv();
        }

        riaps::ports::PortError ModbusUARTBase::SendModbusrepport(MessageBuilder<messages::ResponseFormat>& message) {
            return SendMessageOnPort(message.capnp_builder(), PORT_REP_MODBUSREPPORT);
        }


        void ModbusUARTBase::DispatchMessage(riaps::ports::PortBase* port) {
            auto port_name = port->port_name();
            if (port_name == PORT_REP_MODBUSREPPORT) {
                OnModbusrepport();
            }
            if (port_name == PORT_TIMER_CLOCK) {
                OnClock();
            }
        }

        void ModbusUARTBase::DispatchInsideMessage(zmsg_t *zmsg, riaps::ports::PortBase *port) { }
    }
}

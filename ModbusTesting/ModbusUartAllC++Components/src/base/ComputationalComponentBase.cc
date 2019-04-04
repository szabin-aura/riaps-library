



#include <componentmodel/r_pyconfigconverter.h>
#include <base/ComputationalComponentBase.h>

using namespace std;
using namespace riaps::ports;

namespace riapsmodbuscreqrepuart {
    namespace components {
        ComputationalComponentBase::ComputationalComponentBase(const py::object*  parent_actor     ,
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

        tuple<MessageReader<messages::ResponseFormat>, PortError> ComputationalComponentBase::RecvModbusreqport() {
            auto port = GetPortAs<riaps::ports::RequestPort>(PORT_REQ_MODBUSREQPORT);
            auto [msg_bytes, error] = port->Recv();
            MessageReader<messages::ResponseFormat> reader(msg_bytes);
            return make_tuple(reader, error);
        }

        timespec ComputationalComponentBase::RecvClock() {
            auto port = GetPortAs<riaps::ports::PeriodicTimer>(PORT_TIMER_CLOCK);
            return port->Recv();
        }

        riaps::ports::PortError ComputationalComponentBase::SendModbusreqport(MessageBuilder<messages::CommandFormat>& message) {
            return SendMessageOnPort(message.capnp_builder(), PORT_REQ_MODBUSREQPORT);
        }

        riaps::ports::PortError ComputationalComponentBase::SendTx_modbusdata(MessageBuilder<messages::LogData>& message) {
            return SendMessageOnPort(message.capnp_builder(), PORT_PUB_TX_MODBUSDATA);
        }


        void ComputationalComponentBase::DispatchMessage(riaps::ports::PortBase* port) {
            auto port_name = port->port_name();
            if (port_name == PORT_REQ_MODBUSREQPORT) {
                OnModbusreqport();
            }
            if (port_name == PORT_TIMER_CLOCK) {
                OnClock();
            }
        }

        void ComputationalComponentBase::DispatchInsideMessage(zmsg_t *zmsg, riaps::ports::PortBase *port) { }
    }
}

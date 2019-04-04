



#include <componentmodel/r_pyconfigconverter.h>
#include <base/LoggerBase.h>

using namespace std;
using namespace riaps::ports;

namespace riapsmodbuscreqrepuart {
    namespace components {
        LoggerBase::LoggerBase(const py::object*  parent_actor     ,
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

        tuple<MessageReader<messages::LogData>, PortError> LoggerBase::RecvRx_modbusdata() {
            auto port = GetPortAs<riaps::ports::SubscriberPort>(PORT_SUB_RX_MODBUSDATA);
            auto [msg_bytes, error] = port->Recv();
            MessageReader<messages::LogData> reader(msg_bytes);
            return make_tuple(reader, error);
        }



        void LoggerBase::DispatchMessage(riaps::ports::PortBase* port) {
            auto port_name = port->port_name();
            if (port_name == PORT_SUB_RX_MODBUSDATA) {
                OnRx_modbusdata();
            }
        }

        void LoggerBase::DispatchInsideMessage(zmsg_t *zmsg, riaps::ports::PortBase *port) { }
    }
}

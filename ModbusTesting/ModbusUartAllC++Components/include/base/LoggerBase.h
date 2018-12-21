

#ifndef LOGGERBASE_H
#define LOGGERBASE_H

#include <pybind11/stl.h>
#include <pybind11/pybind11.h>
#include <componentmodel/r_componentbase.h>
#include <componentmodel/r_messagebuilder.h>
#include <componentmodel/r_messagereader.h>
#include <messages/riapsmodbuscreqrepuart.capnp.h>

namespace py = pybind11;
constexpr auto PORT_SUB_RX_MODBUSDATA = "rx_modbusData";


namespace riapsmodbuscreqrepuart {
    namespace components {
        class LoggerBase : public riaps::ComponentBase {
        public:
            LoggerBase(const py::object*  parent_actor     ,
                          const py::dict     actor_spec       ,
                          const py::dict     type_spec        ,
                          const std::string& name             ,
                          const std::string& type_name        ,
                          const py::dict     args             ,
                          const std::string& application_name ,
                          const std::string& actor_name       );

            virtual void OnRx_modbusdata()=0;


            virtual std::tuple<MessageReader<messages::LogData>, riaps::ports::PortError> RecvRx_modbusdata() final;


            virtual ~LoggerBase() = default;
        protected:
            virtual void DispatchMessage(riaps::ports::PortBase* port) final;

            virtual void DispatchInsideMessage(zmsg_t* zmsg, riaps::ports::PortBase* port) final;
        };
    }
}


#endif // LOGGERBASE_H

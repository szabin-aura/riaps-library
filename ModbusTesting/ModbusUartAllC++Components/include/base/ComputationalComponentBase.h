

#ifndef COMPUTATIONALCOMPONENTBASE_H
#define COMPUTATIONALCOMPONENTBASE_H

#include <pybind11/stl.h>
#include <pybind11/pybind11.h>
#include <componentmodel/r_componentbase.h>
#include <componentmodel/r_messagebuilder.h>
#include <componentmodel/r_messagereader.h>
#include <messages/riapsmodbuscreqrepuart.capnp.h>

namespace py = pybind11;
constexpr auto PORT_REQ_MODBUSREQPORT = "modbusReqPort";
constexpr auto PORT_PUB_TX_MODBUSDATA = "tx_modbusData";
constexpr auto PORT_TIMER_CLOCK = "clock";


namespace riapsmodbuscreqrepuart {
    namespace components {
        class ComputationalComponentBase : public riaps::ComponentBase {
        public:
            ComputationalComponentBase(const py::object*  parent_actor     ,
                          const py::dict     actor_spec       ,
                          const py::dict     type_spec        ,
                          const std::string& name             ,
                          const std::string& type_name        ,
                          const py::dict     args             ,
                          const std::string& application_name ,
                          const std::string& actor_name       );

            virtual void OnModbusreqport()=0;
            virtual void OnClock()=0;


            virtual std::tuple<MessageReader<messages::ResponseFormat>, riaps::ports::PortError> RecvModbusreqport() final;
            virtual timespec RecvClock() final;

            virtual riaps::ports::PortError SendModbusreqport(MessageBuilder<messages::CommandFormat>& message) final;
            virtual riaps::ports::PortError SendTx_modbusdata(MessageBuilder<messages::LogData>& message) final;

            virtual ~ComputationalComponentBase() = default;
        protected:
            virtual void DispatchMessage(riaps::ports::PortBase* port) final;

            virtual void DispatchInsideMessage(zmsg_t* zmsg, riaps::ports::PortBase* port) final;
        };
    }
}


#endif // COMPUTATIONALCOMPONENTBASE_H

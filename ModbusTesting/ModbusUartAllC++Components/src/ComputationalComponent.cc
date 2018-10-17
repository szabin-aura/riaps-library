#include <ComputationalComponent.h>

namespace riapsmodbuscreqrepuart {
   namespace components {
      
      ComputationalComponent::ComputationalComponent(const py::object *parent_actor,
                                                     const py::dict actor_spec,
                                                     const py::dict type_spec,
                                                     const std::string &name,
                                                     const std::string &type_name,
                                                     const py::dict args,
                                                     const std::string &application_name,
                                                     const std::string &actor_name) :
      ComputationalComponentBase(parent_actor, actor_spec, type_spec, name, type_name, args, application_name,
                                 actor_name) {
          PID = getpid();
          debugMode = true;

          if (debugMode) {
              component_logger()->set_level(spdlog::level::level_enum::debug);
          } else {
              component_logger()->set_level(spdlog::level::level_enum::info);
          }

          // Setup Modbus Data Registers
          nb_holdingRegs = 3;
          nb_inputRegs = 4;
          holdingRegs = std::unique_ptr<uint16_t[]>(new uint16_t[nb_holdingRegs]);
          inputRegs = std::unique_ptr<uint16_t[]>(new uint16_t[nb_inputRegs]);

          inputRegs[0] = 45;
          for (int idx=1; idx < nb_inputRegs; idx++) {
              inputRegs[idx] = inputRegs[idx-1] + 5;
          }

          holdingRegs[0] = 54;
          for (int idx=1; idx < nb_holdingRegs; idx++) {
              holdingRegs[idx] = holdingRegs[idx-1] + 5;
          }

          // Setup Commands
          openReq = false;

          int resResult = clock_getres(CLOCK_MONOTONIC, &resolution);

          if (resResult != 0) {
              component_logger()->debug("Error occurred {}", strerror(errno));
              exit(errno);
          }

          component_logger()->info("ComputationalComponent: {} - starting",PID);
      }
      
      void ComputationalComponent::OnClock(riaps::ports::PortBase *port) {
          component_logger()->debug("{}: ComputationalComponent::OnClock(): port={}", PID, port->GetPortName());

          if (!openReq) {
              clock_gettime(CLOCK_MONOTONIC, &preobservations);
              capnp::MallocMessageBuilder messageRepBuilder;
              auto msgModbusCommand = messageRepBuilder.initRoot<modbusuart::messages::CommandFormat>();
              msgModbusCommand.setCommandType(modbusuart::messages::ModbusCommands::READ_HOLDING_REGS);
              msgModbusCommand.setRegisterAddress(0);
              msgModbusCommand.setNumberOfRegs(
                      nb_holdingRegs);  // for writes, this value is 1 if successful, -1 if failed

              auto values = msgModbusCommand.initValues(nb_holdingRegs);

              for (int i = 0; i <= nb_holdingRegs; i++) {
                  values.set(i, (uint16_t) holdingRegs.get()[i]);
              }

              // Send command back to modbus device component
              component_logger()->warn_if(!SendModbusReqPort(messageRepBuilder, msgModbusCommand),
                               "{}: Couldn't send command message", PID);
              openReq = true;
          } else {
              component_logger()->debug("There is an open request, did not send a new request this clock cycle");
          }
      }
      
      bool ComputationalComponent::RecvModbusReqPort(modbusuart::messages::ResponseFormat::Reader &message) {

          // Receive Response
          component_logger()->debug("Begin on_modbusReqPort()");

          openReq = false;

          if (debugMode) {
              clock_gettime(CLOCK_MONOTONIC, &postobservations);
              system_time_sub(&postobservations, &preobservations, &result);
              component_logger()->debug("Command to data: tv.sec={} tv_nsec={}", result.tv_sec,
                             ((double) result.tv_nsec) / NSEC_PER_SEC);
          }

          component_logger()->debug("End on_modbusReqPort()");
      }

   }
}

std::unique_ptr<riapsmodbuscreqrepuart::components::ComputationalComponent>
create_component_py(const py::object *parent_actor,
                    const py::dict actor_spec,
                    const py::dict type_spec,
                    const std::string &name,
                    const std::string &type_name,
                    const py::dict args,
                    const std::string &application_name,
                    const std::string &actor_name) {
    auto ptr = new riapsmodbuscreqrepuart::components::ComputationalComponent(parent_actor, actor_spec, type_spec, name, type_name, args,
                                                   application_name,
                                                   actor_name);
    return std::move(std::unique_ptr<riapsmodbuscreqrepuart::components::ComputationalComponent>(ptr));
}

PYBIND11_MODULE(libcomputationalcomponent, m) {
    py::class_<riapsmodbuscreqrepuart::components::ComputationalComponent> testClass(m, "ComputationalComponent");
    testClass.def(py::init<const py::object*, const py::dict, const py::dict, const std::string&, const std::string&, const py::dict, const std::string&, const std::string&>());

    testClass.def("setup"                 , &riapsmodbuscreqrepuart::components::ComputationalComponent::Setup);
    testClass.def("activate"              , &riapsmodbuscreqrepuart::components::ComputationalComponent::Activate);
    testClass.def("terminate"             , &riapsmodbuscreqrepuart::components::ComputationalComponent::Terminate);
    testClass.def("handlePortUpdate"      , &riapsmodbuscreqrepuart::components::ComputationalComponent::HandlePortUpdate);
    testClass.def("handleCPULimit"        , &riapsmodbuscreqrepuart::components::ComputationalComponent::HandleCPULimit);
    testClass.def("handleMemLimit"        , &riapsmodbuscreqrepuart::components::ComputationalComponent::HandleMemLimit);
    testClass.def("handleSpcLimit"        , &riapsmodbuscreqrepuart::components::ComputationalComponent::HandleSpcLimit);
    testClass.def("handleNetLimit"        , &riapsmodbuscreqrepuart::components::ComputationalComponent::HandleNetLimit);
    testClass.def("handleNICStateChange"  , &riapsmodbuscreqrepuart::components::ComputationalComponent::HandleNICStateChange);
    testClass.def("handlePeerStateChange" , &riapsmodbuscreqrepuart::components::ComputationalComponent::HandlePeerStateChange);

    m.def("create_component_py", &create_component_py, "Instantiates the component from python configuration");
}

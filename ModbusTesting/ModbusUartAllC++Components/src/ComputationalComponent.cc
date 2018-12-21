


#include <ComputationalComponent.h>
// riaps:keep_header:begin

// riaps:keep_header:end

namespace riapsmodbuscreqrepuart {
    namespace components {

        // riaps:keep_construct:begin
        ComputationalComponent::ComputationalComponent(const py::object*  parent_actor     ,
                      const py::dict     actor_spec       ,
                      const py::dict     type_spec        ,
                      const std::string& name             ,
                      const std::string& type_name        ,
                      const py::dict     args             ,
                      const std::string& application_name ,
                      const std::string& actor_name       )
            : ComputationalComponentBase(parent_actor, actor_spec, type_spec, name, type_name, args, application_name, actor_name) {
            PID_ = getpid();
            debug_mode_ = true;

            if (debug_mode_) {
                component_logger()->set_level(spdlog::level::level_enum::debug);
            } else {
                component_logger()->set_level(spdlog::level::level_enum::info);
            }

            // Setup Modbus Data Registers
            nb_holding_regs_ = 3;
            nb_input_regs_ = 4;
            holding_regs_ = std::unique_ptr<uint16_t[]>(new uint16_t[nb_holding_regs_]);
            input_regs_ = std::unique_ptr<uint16_t[]>(new uint16_t[nb_input_regs_]);

            input_regs_[0] = 45;
            for (int idx=1; idx < nb_input_regs_; idx++) {
                input_regs_[idx] = input_regs_[idx-1] + 5;
            }

            holding_regs_[0] = 54;
            for (int idx=1; idx < nb_holding_regs_; idx++) {
                holding_regs_[idx] = holding_regs_[idx-1] + 5;
            }

            // Setup Commands
            open_req_ = false;

            int res_result = clock_getres(CLOCK_MONOTONIC, &resolution_);

            if (res_result != 0) {
                component_logger()->debug("Error occurred {}", strerror(errno));
                exit(errno);
            }

            component_logger()->info("ComputationalComponent: {} - starting",PID_);
        }
        // riaps:keep_construct:end

        void ComputationalComponent::OnModbusreqport() {
            // riaps:keep_onmodbusreqport:begin
            auto [msg, err] = RecvModbusreqport();
            // Receive Response
            component_logger()->debug("Begin on_modbusReqPort()");

            open_req_ = false;

            if (debug_mode_) {
                clock_gettime(CLOCK_MONOTONIC, &postobservations_);
                system_time_sub(&postobservations_, &preobservations_, &result_);
                component_logger()->debug("Command to data: tv.sec={} tv_nsec={}", result_.tv_sec,
                               ((double) result_.tv_nsec) / NSEC_PER_SEC);
            }

            component_logger()->debug("End on_modbusReqPort()");
            // riaps:keep_onmodbusreqport:end
        }

        void ComputationalComponent::OnClock() {
            // riaps:keep_onclock:begin
            auto msg = RecvClock();
            component_logger()->debug("{}: ComputationalComponent::OnClock(): port={}", PID_, port->GetPortName());

            if (!openReq) {
                clock_gettime(CLOCK_MONOTONIC, &preobservations_);
                capnp::MallocMessageBuilder messageRepBuilder;
                auto msgModbusCommand = messageRepBuilder.initRoot<modbusuart::messages::CommandFormat>();
                msgModbusCommand.setCommandType(modbusuart::messages::ModbusCommands::READ_HOLDING_REGS);
                msgModbusCommand.setRegisterAddress(0);
                msgModbusCommand.setNumberOfRegs(
                        nb_holdingRegs);  // for writes, this value is 1 if successful, -1 if failed

                auto values = msgModbusCommand.initValues(nb_holding_regs_);

                for (int i = 0; i <= nb_holding_regs_; i++) {
                    values.set(i, (uint16_t) holding_regs_.get()[i]);
                }

                // Send command back to modbus device component
                component_logger()->warn_if(!SendModbusReqPort(messageRepBuilder, msgModbusCommand),
                                 "{}: Couldn't send command message", PID_);
                open_req_ = true;
            } else {
                component_logger()->debug("There is an open request, did not send a new request this clock cycle");
            }
            // riaps:keep_onclock:end
        }

        // riaps:keep_impl:begin

        // riaps:keep_impl:end

        // riaps:keep_destruct:begin
        ComputationalComponent::~ComputationalComponent() {

        }
        // riaps:keep_destruct:end

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
    testClass.def("handleReinstate"       , &riapsmodbuscreqrepuart::components::ComputationalComponent::HandleReinstate);

    m.def("create_component_py", &create_component_py, "Instantiates the component from python configuration");
}

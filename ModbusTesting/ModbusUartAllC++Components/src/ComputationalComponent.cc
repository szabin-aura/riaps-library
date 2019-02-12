


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

            debug_mode_ = false;
            if (debug_mode_)
                set_debug_level(spdlog::level::debug);

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
            auto [msg_request, request_err] = RecvModbusreqport();
            // Receive Response
            component_logger()->debug("Begin on_modbusReqPort()");

            if (!request_err) {
                open_req_ = false;

                if (debug_mode_) {
                    clock_gettime(CLOCK_MONOTONIC, &postobservations_);
                    system_time_sub(&postobservations_, &preobservations_, &result_);
                    component_logger()->debug("Command to data: tv.sec={} tv_nsec={}", result_.tv_sec,
                                   ((double) result_.tv_nsec) / NSEC_PER_SEC);
                }

                // MM TODO:  Rework output message to match what is really happening
                int16_t reg_address = msg_request->getRegisterAddress();
                int16_t num_regs = msg_request->getNumberOfRegs();
                std::string value_msg;

                for (int i = 0; i < num_regs; i++) {
                    value_msg.append(std::to_string(msg_request->getValues()[i]) + " ");
                }

                MessageBuilder<messages::LogData> log_builder;
                if (msg_request->getCommandType() == riapsmodbuscreqrepuart::messages::ModbusCommands::READ_COIL_BITS) {
                    log_builder->setMsg(fmt::format("Received {} Coil Bits at {}: values are {}", num_regs, reg_address, value_msg));
                } else if (msg_request->getCommandType() == riapsmodbuscreqrepuart::messages::ModbusCommands::READ_INPUT_BITS) {
                    log_builder->setMsg(fmt::format("Received {} Input Bits at {}: values are {}", num_regs, reg_address, value_msg));
                } else if (msg_request->getCommandType() == riapsmodbuscreqrepuart::messages::ModbusCommands::READ_INPUT_REGS) {
                    log_builder->setMsg(fmt::format("Received {} Input Registers at {}: values are {}", num_regs, reg_address, value_msg));
                } else if (msg_request->getCommandType() == riapsmodbuscreqrepuart::messages::ModbusCommands::READ_HOLDING_REGS) {
                    log_builder->setMsg(fmt::format("Received {} Holding Registers at {}: values are {}", num_regs, reg_address, value_msg));
                } else if ((msg_request->getCommandType() == riapsmodbuscreqrepuart::messages::ModbusCommands::WRITE_COIL_BIT) || (msg_request->getCommandType() == riapsmodbuscreqrepuart::messages::ModbusCommands::WRITE_COIL_BITS)) {
                    log_builder->setMsg(fmt::format("Wrote {} Coil Bits {}: values are {}", num_regs, reg_address, value_msg));
                } else if ((msg_request->getCommandType() == riapsmodbuscreqrepuart::messages::ModbusCommands::WRITE_HOLDING_REG) || (msg_request->getCommandType() == riapsmodbuscreqrepuart::messages::ModbusCommands::WRITE_MULTI_HOLDING_REGS)) {
                    log_builder->setMsg(fmt::format("Wrote {} Holding Registers {}: values are {}", num_regs, reg_address, value_msg));
                } else if (msg_request->getCommandType() == riapsmodbuscreqrepuart::messages::ModbusCommands::WRITE_READ_HOLDING_REGS) {
                    log_builder->setMsg(fmt::format("Wrote and Read {} Holding Registers {}: values are {}", num_regs, reg_address, value_msg));
                }

                auto logdata_error = SendTx_modbusdata(log_builder);
                if (!logdata_error) {
                    component_logger()->info("Log Message Sent");
                } else {
                    component_logger()->warn("Error sending Log Message: {}, errorcode: {}", __func__, logdata_error.error_code());
                }
            }
            else {
                component_logger()->warn("Recv() error in {}, errorcode: {}", __func__, request_err.error_code());
            }

            component_logger()->debug("End on_modbusReqPort()");
            // riaps:keep_onmodbusreqport:end
        }

        void ComputationalComponent::OnClock() {
            // riaps:keep_onclock:begin
            auto msg = RecvClock();
            component_logger()->debug("{}: ComputationalComponent::OnClock():", PID_);

            if (!open_req_) {
                clock_gettime(CLOCK_MONOTONIC, &preobservations_);

                MessageBuilder<messages::CommandFormat> command_builder;
                command_builder->setCommandType(riapsmodbuscreqrepuart::messages::ModbusCommands::READ_HOLDING_REGS);
                command_builder->setRegisterAddress(0);
                command_builder->setNumberOfRegs(nb_holding_regs_);  // for writes, this value is 1 if successful, -1 if failed

                auto values = command_builder->initValues(nb_holding_regs_);

                for (int i = 0; i < nb_holding_regs_; i++) {
                    values.set(i, (uint16_t) holding_regs_.get()[i]);
                }

                // Send command back to modbus device component
                auto command_error = SendModbusreqport(command_builder);
                if (!command_error) {
                    open_req_ = true;
                } else {
                    component_logger()->warn("Error sending command message: {}, errorcode: {}", PID_, command_error.error_code());
                }

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

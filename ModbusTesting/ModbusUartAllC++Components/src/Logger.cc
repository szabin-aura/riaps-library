


#include <Logger.h>
// riaps:keep_header:begin

// riaps:keep_header:end

namespace riapsmodbuscreqrepuart {
    namespace components {

        // riaps:keep_construct:begin
        Logger::Logger(const py::object*  parent_actor     ,
                      const py::dict     actor_spec       ,
                      const py::dict     type_spec        ,
                      const std::string& name             ,
                      const std::string& type_name        ,
                      const py::dict     args             ,
                      const std::string& application_name ,
                      const std::string& actor_name       )
            : LoggerBase(parent_actor, actor_spec, type_spec, name, type_name, args, application_name, actor_name) {

        }
        // riaps:keep_construct:end

        void Logger::OnRx_modbusdata() {
            // riaps:keep_onrx_modbusdata:begin
            auto [msg, err] = RecvRx_modbusdata();

            if (!err)
                component_logger()->info("{}: [{}]", msg->getMsg().cStr(), ::getpid());
            else
                component_logger()->warn("Recv() error in {}, errorcode: {}", __func__, err.error_code());

            // riaps:keep_onrx_modbusdata:end
        }

        // riaps:keep_impl:begin

        // riaps:keep_impl:end

        // riaps:keep_destruct:begin
        Logger::~Logger() {

        }
        // riaps:keep_destruct:end

    }
}

std::unique_ptr<riapsmodbuscreqrepuart::components::Logger>
create_component_py(const py::object *parent_actor,
                    const py::dict actor_spec,
                    const py::dict type_spec,
                    const std::string &name,
                    const std::string &type_name,
                    const py::dict args,
                    const std::string &application_name,
                    const std::string &actor_name) {
    auto ptr = new riapsmodbuscreqrepuart::components::Logger(parent_actor, actor_spec, type_spec, name, type_name, args,
                                                                     application_name,
                                                                     actor_name);
    return std::move(std::unique_ptr<riapsmodbuscreqrepuart::components::Logger>(ptr));
}

PYBIND11_MODULE(liblogger, m) {
    py::class_<riapsmodbuscreqrepuart::components::Logger> testClass(m, "Logger");
    testClass.def(py::init<const py::object*, const py::dict, const py::dict, const std::string&, const std::string&, const py::dict, const std::string&, const std::string&>());

    testClass.def("setup"                 , &riapsmodbuscreqrepuart::components::Logger::Setup);
    testClass.def("activate"              , &riapsmodbuscreqrepuart::components::Logger::Activate);
    testClass.def("terminate"             , &riapsmodbuscreqrepuart::components::Logger::Terminate);
    testClass.def("handlePortUpdate"      , &riapsmodbuscreqrepuart::components::Logger::HandlePortUpdate);
    testClass.def("handleCPULimit"        , &riapsmodbuscreqrepuart::components::Logger::HandleCPULimit);
    testClass.def("handleMemLimit"        , &riapsmodbuscreqrepuart::components::Logger::HandleMemLimit);
    testClass.def("handleSpcLimit"        , &riapsmodbuscreqrepuart::components::Logger::HandleSpcLimit);
    testClass.def("handleNetLimit"        , &riapsmodbuscreqrepuart::components::Logger::HandleNetLimit);
    testClass.def("handleNICStateChange"  , &riapsmodbuscreqrepuart::components::Logger::HandleNICStateChange);
    testClass.def("handlePeerStateChange" , &riapsmodbuscreqrepuart::components::Logger::HandlePeerStateChange);
    testClass.def("handleReinstate"       , &riapsmodbuscreqrepuart::components::Logger::HandleReinstate);

    m.def("create_component_py", &create_component_py, "Instantiates the component from python configuration");
}



#ifndef LOGGER_H
#define LOGGER_H
#include <base/LoggerBase.h>
// riaps:keep_header:begin

// riaps:keep_header:end>>

namespace riapsmodbuscreqrepuart {
    namespace components {
        class Logger : public LoggerBase {
        public:
            Logger(const py::object*  parent_actor     ,
                          const py::dict     actor_spec       ,
                          const py::dict     type_spec        ,
                          const std::string& name             ,
                          const std::string& type_name        ,
                          const py::dict     args             ,
                          const std::string& application_name ,
                          const std::string& actor_name       );


            virtual void OnRx_modbusdata() override;

            virtual ~Logger();

            // riaps:keep_decl:begin

            // riaps:keep_decl:end
        };
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
const std::string &actor_name);

#endif // LOGGER_H



#ifndef COMPUTATIONALCOMPONENT_H
#define COMPUTATIONALCOMPONENT_H
#include <base/ComputationalComponentBase.h>
// riaps:keep_header:begin
#include "time_macros.h"
// riaps:keep_header:end>>

namespace riapsmodbuscreqrepuart {
    namespace components {
        class ComputationalComponent : public ComputationalComponentBase {
        public:
            ComputationalComponent(const py::object*  parent_actor     ,
                          const py::dict     actor_spec       ,
                          const py::dict     type_spec        ,
                          const std::string& name             ,
                          const std::string& type_name        ,
                          const py::dict     args             ,
                          const std::string& application_name ,
                          const std::string& actor_name       );


            virtual void OnModbusreqport() override;
            virtual void OnClock() override;

            virtual ~ComputationalComponent();

            // riaps:keep_decl:begin
    	private:
            pid_t PID_;
            bool debug_mode_;
            bool open_req_;
            int nb_holding_regs_;
            int nb_input_regs_;
            std::unique_ptr<uint16_t[]> holding_regs_;
            std::unique_ptr<uint16_t[]> input_regs_;

            // time testing
            struct timespec resolution_ = {0,0};
            struct timespec preobservations_ = {0,0};
            struct timespec postobservations_ = {0,0};
            system_timespec result_;
            // riaps:keep_decl:end
        };
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
const std::string &actor_name);

#endif // COMPUTATIONALCOMPONENT_H

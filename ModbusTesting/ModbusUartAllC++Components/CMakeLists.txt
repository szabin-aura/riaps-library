cmake_minimum_required(VERSION 3.8)
project(modbusuartdevice)

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR})
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_RUNTIME_OUTPUT_DIRECTORY})
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY  ${CMAKE_RUNTIME_OUTPUT_DIRECTORY})

include_directories(/usr/include/python3.6m/)
include_directories(${RIAPS_PREFIX}/include)
include_directories(${RIAPS_PREFIX}/include/pybind11/include/)
include_directories(include)
link_directories(${RIAPS_PREFIX}/lib)


add_custom_command(
        OUTPUT  ${CMAKE_CURRENT_SOURCE_DIR}/include/messages/modbusuart.capnp.c++ ${CMAKE_CURRENT_SOURCE_DIR}/include/messages/modbusuart.capnp.h
        DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/include/messages/modbusuart.capnp
        WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/include/messages
        COMMAND /opt/riaps/amd64/bin/capnp compile modbusuart.capnp -oc++:./
        COMMENT "=== Generating modbusuart capnp ==="
)

add_custom_command(
        OUTPUT  ${CMAKE_CURRENT_SOURCE_DIR}/include/messages/LogData.capnp.c++ ${CMAKE_CURRENT_SOURCE_DIR}/include/messages/LogData.capnp.h
        DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/include/messages/LogData.capnp
        WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/include/messages
        COMMAND /opt/riaps/amd64/bin/capnp compile LogData.capnp -oc++:./
        COMMENT "=== Generating LogData capnp ==="
)

add_library(modbusuart SHARED src/ModbusUART.cc
                              src/base/ModbusUARTBase.cc
                              ${CMAKE_CURRENT_SOURCE_DIR}/include/messages/modbusuart.capnp.c++
                              include/base/ModbusUARTBase.h
                              include/ModbusUART.h
           )
target_link_libraries(modbusuart PRIVATE czmq riaps dl capnp kj modbus)
set_target_properties(modbusuart PROPERTIES PREFIX lib SUFFIX .so)

add_library(computationalcomponent SHARED src/ComputationalComponent.cc
                                          src/base/ComputationalComponentBase.cc
                                          ${CMAKE_CURRENT_SOURCE_DIR}/include/messages/modbusuart.capnp.c++
                                          ${CMAKE_CURRENT_SOURCE_DIR}/include/messages/LogData.capnp.c++
                                          include/base/ComputationalComponentBase.h
                                          include/ComputationalComponent.h
            )
target_link_libraries(computationalcomponent PRIVATE czmq riaps dl capnp kj modbusuart)
set_target_properties(computationalcomponent PROPERTIES PREFIX lib SUFFIX .so)

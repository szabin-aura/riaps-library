


#include <ModbusUART.h>
// riaps:keep_header:begin

// riaps:keep_header:end

namespace riapsmodbuscreqrepuart {
    namespace components {

        // riaps:keep_construct:begin
        ModbusUART::ModbusUART(const py::object*  parent_actor     ,
                      const py::dict     actor_spec       ,
                      const py::dict     type_spec        ,
                      const std::string& name             ,
                      const std::string& type_name        ,
                      const py::dict     args             ,
                      const std::string& application_name ,
                      const std::string& actor_name       )
            : ModbusUARTBase(parent_actor, actor_spec, type_spec, name, type_name, args, application_name, actor_name) {
            PID_ = getpid();

            debug_mode_ = false;

            // Configure Modbus UART port parameters
            auto iport = component_config().component_parameters.getParam("port");
            if (iport != nullptr) {
                std::string iportStr = iport->getValueAsString();
                if (iportStr == "UART1") {
                    port_config_.portname = "/dev/ttyO1";
                } else if (iportStr == "UART2") {
                    port_config_.portname = "/dev/ttyO2";
                } else if (iportStr == "UART3") {
                    port_config_.portname = "/dev/ttyO3";
                } else if (iportStr == "UART4") {
                    port_config_.portname = "/dev/ttyO4";
                } else if (iportStr == "UART5") {
                    port_config_.portname = "/dev/ttyO5";
                } else {
                    component_logger()->error("{}: Invalid UART argument (port={}), use port = UART1..5", PID_,
                                   iport->getValueAsString());
                }
            } else {
                // Declare a default value
                port_config_.portname = "/dev/ttyO2";
            }

            auto ibaudrate = component_config().component_parameters.getParam("baudrate");
            if (ibaudrate != nullptr) {
                if(!ibaudrate->getValueAsInt(&(port_config_.baudrate))) {
                    component_logger()->error("{}: Invalid Baud Rate argument - {}, should be integer value", PID_, port_config_.baudrate);
                }
            } else {
                // Declare a default value
                port_config_.baudrate = 115200;
            }

            auto ibytesize = component_config().component_parameters.getParam("bytesize");
            if (ibytesize != nullptr) {
                if(!ibytesize->getValueAsInt(&(port_config_.bytesize))) {
                    component_logger()->error("{}: Invalid Byte Size argument - {}, should be integer value", PID_, port_config_.bytesize);
                }
            } else {
                // Declare a default value
                port_config_.bytesize = 8;
            }

            auto iparity = component_config().component_parameters.getParam("parity");
            if (iparity != nullptr) {
                port_config_.parity = iparity->getValueAsString()[0];
            } else {
                // Declare a default value
                port_config_.parity = 'N';
            }

            auto istopbits = component_config().component_parameters.getParam("stopbits");
            if (istopbits != nullptr) {
                if(!istopbits->getValueAsInt(&(port_config_.stopbits))) {
                    component_logger()->error("{}: Invalid Stop Bits argument - {}, should be integer value", PID_, port_config_.stopbits);
                }
            } else {
                // Declare a default value
                port_config_.stopbits = 1;
            }

            auto islaveaddress = component_config().component_parameters.getParam("slaveaddress");
            if (islaveaddress != nullptr) {
                if(!islaveaddress->getValueAsInt(&(port_slave_address_))) {
                    component_logger()->error("{}: Invalid Port Address argument - {}, should be integer value", PID_, port_slave_address_);
                }
            } else {
                // Declare a default value
                port_slave_address_ = 10;
            }

            // Configure number of Modbus data registers used
            auto inumholdreg = component_config().component_parameters.getParam("numholdreg");
            if (inumholdreg != nullptr) {
                if(!inumholdreg->getValueAsInt(&(nb_holding_regs_))) {
                    component_logger()->error("{}: Invalid Number of Holding Registers argument - {}, should be integer value", PID_, nb_holding_regs_);
                }
            } else {
                // Set a default of 3 (an arbritary number based on the first example created)
                nb_holding_regs_ = 3;
            }

            auto inuminputreg = component_config().component_parameters.getParam("numinputreg");
            if (inuminputreg != nullptr) {
                if(!inuminputreg->getValueAsInt(&(nb_input_regs_))) {
                    component_logger()->error("{}: Invalid Number of Input Registers argument - {}, should be integer value", PID_, nb_input_regs_);
                }
            } else {
                // Set a default of 4 (an arbritary number based on the first example created)
                nb_input_regs_ = 4;
            }

            auto inumcoilbits = component_config().component_parameters.getParam("numcoilbits");
            if (inumcoilbits != nullptr) {
                if(!inumcoilbits->getValueAsInt(&(nb_coil_bits_))) {
                    component_logger()->error("{}: Invalid Number of Coil Bits argument - {}, should be integer value", PID_, nb_coil_bits_);
                }
            } else {
                // Set a default of 8 (an arbritary number based on the first example created)
                nb_coil_bits_ = 8;
            }

            auto inumdiscretebits = component_config().component_parameters.getParam("numdiscretebits");
            if (inumdiscretebits != nullptr) {
                if(!inumdiscretebits->getValueAsInt(&(nb_discrete_bits_))) {
                    component_logger()->error("{}: Invalid Number of Discreet Bits argument - {}, should be integer value", PID_, nb_discrete_bits_);
                }
            } else {
                // Set a default of 8 (an arbritary number based on the first example created)
                nb_discrete_bits_ = 8;
            }

            // Modbus port initially closed
            port_open_ = false;
            port_serial_mode_ = MODBUS_RTU_RS232;

            // Setup Modbus Data Registers
            holding_regs_ = std::unique_ptr<uint16_t[]>(new uint16_t[nb_holding_regs_]);
            input_regs_ = std::unique_ptr<uint16_t[]>(new uint16_t[nb_input_regs_]);
            coil_bits_ = std::unique_ptr<uint8_t[]>(new uint8_t[nb_coil_bits_]);
            discrete_bits_ = std::unique_ptr<uint8_t[]>(new uint8_t[nb_discrete_bits_]);

            component_logger()->info("{}: Modbus UART settings {} @{}:{} {}{}{}", PID_, port_slave_address_, port_config_.portname,
                          port_config_.baudrate, port_config_.bytesize, port_config_.parity, port_config_.stopbits);
            component_logger()->info(
                    "{}: Modbus Reg settings: numCoilBits={}, numDiscreteBits={}, numInputRegs={}, numHoldingRegs={}",
                    PID_, nb_coil_bits_, nb_discrete_bits_, nb_input_regs_, nb_holding_regs_);

            /* Initialize the libmodbus context
            * Returns a pointer to a modbus_t structure if successful. Otherwise it shall return NULL.
            *   where, baudrate: 9600, 19200, 57600, 115200, etc
            *          Parity: 'N', 'O', 'E'
            *          Slave Address is a decimal value
            *          Serial Mode is either RS232 (0) or RS485 (1) -  RS232 mode is default
            */
            ctx_ = modbus_new_rtu(port_config_.portname.c_str(), port_config_.baudrate, port_config_.parity,
                                 port_config_.bytesize, port_config_.stopbits);
            if (ctx_ == NULL) {
                component_logger()->error("Unable to create the libmodbus context");
            }

            //_logger->debug("Init vars");
            //int resResult = clock_getres(CLOCK_MONOTONIC, &resolution_);

            //if (resResult != 0) {
            //    _logger->debug("Error occurred {}", strerror(errno));
            //    exit(errno);
            //}

        }
        // riaps:keep_construct:end

        void ModbusUART::OnModbusrepport() {
            // riaps:keep_onmodbusrepport:begin
            auto [msg, err] = RecvModbusrepport();
            std::cout << "ModbusUART::OnModbusRepPort()" << std::endl;

            int response_value = -1;  // invalid response

            if (IsModbusReady()) {
                //clock_gettime(CLOCK_MONOTONIC, &preobservations);
                response_value = SendModbusCommand(message);

                capnp::MallocMessageBuilder messageRepBuilder;
                modbusuart::messages::ResponseFormat::Builder msgModbusResponse = messageRepBuilder.initRoot<modbusuart::messages::ResponseFormat>();
                msgModbusResponse.setCommandType(message.getCommandType());
                msgModbusResponse.setRegisterAddress(message.getRegisterAddress());
                msgModbusResponse.setNumberOfRegs(response_value);  // for writes, this value is 1 if successful, -1 if failed

                if (response_value > 0) {
                    auto values = msgModbusResponse.initValues(response_value);

                    if (message.getCommandType() == modbusuart::messages::ModbusCommands::READ_COIL_BITS) {
                        for (int i = 0; i <= response_value; i++) {
                            values.set(i, (uint16_t) coil_bits_.get()[i]);
                        }
                    } else if (message.getCommandType() == modbusuart::messages::ModbusCommands::READ_INPUT_BITS) {
                        for (int i = 0; i <= response_value; i++) {
                            values.set(i, (uint16_t) discrete_bits_.get()[i]);
                        }
                    } else if (message.getCommandType() == modbusuart::messages::ModbusCommands::READ_INPUT_REGS) {
                        for (int i = 0; i <= response_value; i++) {
                            values.set(i, (uint16_t) input_regs_.get()[i]);
                        }
                    } else if ((message.getCommandType() == modbusuart::messages::ModbusCommands::READ_HOLDING_REGS) ||
                               (message.getCommandType() == modbusuart::messages::ModbusCommands::WRITE_READ_HOLDING_REGS)) {
                        for (int i = 0; i <= response_value; i++) {
                            values.set(i, (uint16_t) holding_regs_.get()[i]);
                        }
                    }
                }

                // Send response back to requester
                component_logger()->warn_if(!SendModbusRepPort(messageRepBuilder, msgModbusResponse),
                                 "{}: Couldn't send response message", PID_);
            } else {
                component_logger()->warn("{}: Modbus is not ready for commands", PID_);
            }

            //if (debug_mode_) {
                //clock_gettime(CLOCK_MONOTONIC, &postobservations_);
                //system_time_sub(&postobservations_, &preobservations_, &result_);
                //_logger->debug("After Response sent: tv.sec={} tv_nsec={}",result_.tv_sec,((double)result_.tv_nsec)/NSEC_PER_SEC);
            //}
            // riaps:keep_onmodbusrepport:end
        }

        void ModbusUART::OnClock() {
            // riaps:keep_onclock:begin
            auto msg = RecvClock();
            // if missing modbus commands, consider commenting this message out
            std::cout << "ModbusUART::OnClock(): " << port->GetPortName()
                      << std::endl;
            if (!IsModbusReady() && (ctx_ != NULL)) {
                // Start Modbus
                if (startRTUModbus(port_serial_mode_)) {
                    component_logger()->debug("{}: Started Modbus: port={}, slaveAddress={}", PID_, port_config_.portname, port_slave_address_);
                }
            }

            // Halt clock port
            auto timerPort = GetPortByName(PORT_TIMER_CLOCK);
            if (timerPort != nullptr) {
                auto t = timerPort->AsTimerPort();
                if (t != nullptr) {
                    t->stop();
                    component_logger()->info("{}: Stopped Clock port", PID_);
                }
            }
          // riaps:keep_onclock:end
        }

        // riaps:keep_impl:begin
        int ModbusUART::SendModbusCommand(const modbusuart::messages::CommandFormat::Reader &message) {
            int value = 999;   // Return from the commands - read return number of items read, write provides success (1) / fail (0)
            int write_value;

            int16_t reg_address = message.getRegisterAddress();
            int16_t num_regs = message.getNumberOfRegs();
            std::string value_log_msg;


            switch (message.getCommandType()) {
                case modbusuart::messages::ModbusCommands::READ_COIL_BITS:
                    value = ReadCoilBits(reg_address, num_regs);
                    component_logger()->debug("{}: Sent command {}, register={}, numOfDecimals={}", PID_, "READ_COILBITS",
                                   reg_address, num_regs);
                    break;
                case modbusuart::messages::ModbusCommands::READ_INPUT_BITS:
                    value = ReadInputBits(reg_address, num_regs);
                    component_logger()->debug("{}: Sent command {}, register={}, numOfDecimals={}", PID_, "READ_INPUTBITS",
                                   reg_address, num_regs);
                    break;
                case modbusuart::messages::ModbusCommands::READ_INPUT_REGS:
                    value = ReadInputRegs(reg_address, num_regs);
                    component_logger()->debug("{}: Sent command {}, register={}, numOfRegs={}", PID_, "READ_INPUTREGS",
                                   reg_address, num_regs);
                    break;
                case modbusuart::messages::ModbusCommands::READ_HOLDING_REGS:
                    value = ReadHoldingRegs(reg_address, num_regs);
                    component_logger()->debug("{}: Sent command {}, register={}, numOfRegs={}", PID_, "READ_HOLDINGREGS",
                                   reg_address, num_regs);
                    break;
                case modbusuart::messages::ModbusCommands::WRITE_COIL_BIT:

                    write_value = (int) message.getValues()[0];
                    value = WriteCoilBit(reg_address, write_value);
                    component_logger()->debug("{}: Sent command {}, register={}, value={}", PID_, "WRITE_COILBIT", reg_address,
                                   write_value);
                    break;
                case modbusuart::messages::ModbusCommands::WRITE_HOLDING_REG:
                    write_value = (int) message.getValues()[0];
                    value = WriteHoldingReg(reg_address, write_value);
                    component_logger()->debug("{}: Sent command {}, register={}, value={}", PID_, "WRITE_HOLDINGREG", reg_address,
                                   write_value);
                    break;
                case modbusuart::messages::ModbusCommands::WRITE_COIL_BITS:
                    for (int i = 0; i <= num_regs; i++) {
                        coil_bits_[i] = (uint8_t) message.getValues()[i];
                        value_log_msg.append(std::to_string(coil_bits_[i]) + " ");
                    }
                    value = WriteCoilBits(reg_address, num_regs);
                    component_logger()->debug("{}: Sent command {}, register={}, numberOfDecimals={}", PID_, "WRITE_COILBITS",
                                   reg_address, num_regs);
                    component_logger()->debug("{}: Values:{}", PID_, value_log_msg);
                    break;
                case modbusuart::messages::ModbusCommands::WRITE_MULTI_HOLDING_REGS:
                    for (int i = 0; i <= num_regs; i++) {
                        holding_regs_[i] = (uint16_t) message.getValues()[i];
                        value_log_msg.append(std::to_string(holding_regs_[i]) + " ");
                    }
                    value = WriteHoldingRegs(reg_address, num_regs);
                    component_logger()->debug("{}: Sent command {}, register={}, numOfRegs={}", PID_, "WRITEMULTI_HOLDINGREGS",
                                   reg_address, num_regs);
                    component_logger()->debug("{}: Values:{}", PID_, value_log_msg);
                    break;
                case modbusuart::messages::ModbusCommands::WRITE_READ_HOLDING_REGS:
                    for (int i = 0; i <= num_regs; i++) {
                        holding_regs_[i] = (uint16_t) message.getValues()[i];
                        value_log_msg.append(std::to_string(holding_regs_[i]) + " ");
                    }
                    value = WriteReadHoldingRegs(reg_address, num_regs, message.getWreadRegAddress(),
                                                 message.getWreadNumOfRegs());
                    component_logger()->debug("{}: Sent command {}, writeReg={}, numOfRegsWrite={}, readReg={}, numOfRegsRead={}",
                                   PID_, "WRITEREAD_HOLDINGREGS", reg_address, num_regs, message.getWreadRegAddress(),
                                   message.getWreadNumOfRegs());
                    component_logger()->debug("{}: Values:{}", PID_, value_log_msg);
                    break;
                default:
                    component_logger()->warn("{}: Invalid Command - {}", PID_, (int16_t) message.getCommandType());
            }

            return value;
        }


        /* The following functions utilize the libmodbus library - https://github.com/stephane/libmodbus.
        * Both RTU and TCP are available from libmodbus, but only RTU is implemented with this interface
        * at this time since HW tools available at the moment are serial I/F only.  This could easily be
        * expanded in the future to include TCP interface.
        *
        * MM TODO:  1) Definitely consider adding byte and response timeout setup in the future
        *           2) Add TCP capability later
        *           3) Error recovery available - reconnect and a sleep/flush sequence options
        *           4) Untested functions:  readInputBits, readCoilBits, writeCoilBit, writeCoilBits
        */


        /* Start RTU Modbus
         */
        bool ModbusUART::StartRTUModbus(int serialmode) {

            port_open_ = false;

            // Enable debugging of libmodbus
            modbus_set_debug(ctx_, true);

            modbus_set_slave(ctx_, port_slave_address_);

            if (serialmode == MODBUS_RTU_RS485) {
                if (modbus_rtu_set_serial_mode(ctx_, serialmode) == -1) {
                    component_logger()->error("Invalid Serial Mode: {}.  SerialMode={}", modbus_strerror(errno), serialmode);
                    modbus_free(ctx_);
                    return port_open_ //false;
                } else {
                    component_logger()->debug("Modbus serial mode set to RS485");
                }
            }

            if (modbus_connect(ctx_) == -1) {
                component_logger()->error("Connection failed: {}", modbus_strerror(errno));
                modbus_free(ctx_);
                return port_open_ //false;
            } else {
                port_open_ = true;
                component_logger()->info("Modbus port open. Port={}, Baud={}, {}{}{}", port_config_.portname, port_config_.baudrate,
                              port_config_.bytesize, port_config_.parity, port_config_.stopbits);
            }

            return port_open_;
        }


        /* Close Modbus */
        void ModbusUART::CloseModbus(void) {
            modbus_close(ctx_);
            modbus_free(ctx)_;
            port_open_ = false;
            component_logger()->info("Modbus closed");
        }


        /* Is Modbus Ready?
         * User should make sure modbus is open before reading or writing bits or registers.
         */
        bool ModbusUART::IsModbusReady(void) {
            return port_open_;
        }

        /*  Read Coil Bit Status (Modbus Function Code = 0x01)
         *
         *  Reads the status of the 'nb' bits (coils) to the address 'addr' of the remote device. The result of reading
         *  is stored in 'coil_bits_' array as unsigned bytes (8 bits) set to TRUE or FALSE.
         *  User must take care to allocate enough memory to store the results in 'coil_bits_' (at least nb * sizeof(uint8_t)).
         *
         *  Returns the number of bits read if successful. Otherwise it shall return -1.
         */
        int ModbusUART::ReadCoilBits(int addr, int nb) {
            int bits_read = 0;

            bits_read = modbus_read_bits(ctx_, addr, nb, coil_bits_.get());
            if (bits_read == -1) {
                component_logger()->error("Failed to read coil bits: {}.  Address={}, #Bits={}", modbus_strerror(errno), addr, nb);
            }

            return bits_read;
        }


        /*  Read Input Bits or Status (Modbus Function Code = 0x02)
         *
         *  Reads the content of the 'nb' input bits to the address 'addr' of the remote device. The result of reading
         *  is stored in 'discrete_bits_' array as unsigned bytes (8 bits) set to TRUE or FALSE.
         *
         *  User must take care to allocate enough memory to store the results in 'discrete_bits_' (at least nb * sizeof(uint8_t)).
         *
         *  Returns the number of bits read if successful. Otherwise it shall return -1.
         */
        int ModbusUART::ReadInputBits(int addr, int nb) {
            int bits_read = 0;

            bits_read = modbus_read_input_bits(ctx_, addr, nb, discrete_bits_.get());
            if (bits_read == -1) {
                component_logger()->error("Failed to read input bits: {}.  Address={}, #Bits={}", modbus_strerror(errno), addr, nb);
            }

            return bits_read;
        }


        /*  Read Holding Registers (Modbus Function Code = 0x03)
         *
         *  Reads the content of the 'nb' holding registers to the address 'addr' of the remote device.
         *  The result of reading is stored in 'holding_regs_' array as word values (16 bits).
         *  User must take care to allocate enough memory to store the results in 'holding_regs_' (at least nb * sizeof(uint16_t)).
         *
         *  Returns the number of holding registers read if successful. Otherwise it shall return -1.
         */
        int ModbusUART::ReadHoldingRegs(int addr, int nb) {
            int regs_read = 0;

            regs_read = modbus_read_registers(ctx_, addr, nb, holding_regs_.get());
            if (regs_read == -1) {
                component_logger()->error("Failed to read holding registers: {}.  Address={}, #Reg={}", modbus_strerror(errno), addr, nb);
            }

            return regs_read;
        }


        /*  Read Input Registers (Modbus Function Code = 0x04)
         *
         *  Reads the content of the 'nb' input registers to address 'addr' of the remote device.
         *  The result of the reading is stored in 'input_regs_' array as word values (16 bits).
         *  User must take care to allocate enough memory to store the results in 'input_regs_' (at least nb * sizeof(uint16_t)).
         *
         *  Returns the number of input registers read if successful. Otherwise it shall return -1.
         */
        int ModbusUART::ReadInputRegs(int addr, int nb) {
            int regs_read = 0;

            regs_read = modbus_read_input_registers(ctx_, addr, nb, input_regs_.get());
            if (regs_read == -1) {
                component_logger()->error("Failed to read input registers: {}.  Address={}, #Reg={}", modbus_strerror(errno), addr, nb);
            }

            return regs_read;
        }


        /*  Write a single coil bit (Modbus Function Code = 0x05)
         *
         *  Writes the status of 'status' at the address 'addr' of the remote device.
         *  The value must be set to TRUE or FALSE.
         *
         *  Returns return 1 if successful. Otherwise it shall return -1.
         */
        int ModbusUART::WriteCoilBit(int addr, int status) {
            int bit_written = 0;

            bit_written = modbus_write_bit(ctx_, addr, status);
            if (bit_written == -1) {
                component_logger()->error("Failed to write coil bit: {}.  Address={}, Status={}", modbus_strerror(errno), addr, status);
            }

            return bit_written;
        }


        /*  Write a single register (Modbus Function Code = 0x06)
         *
         *  Writes the value of 'value' holding registers at the address 'addr' of the remote device.
         *
         *  Returns return 1 if successful. Otherwise it shall return -1.
         */
        int ModbusUART::WriteHoldingReg(int addr, int value) {
            int value_written = 0;

            value_written = modbus_write_register(ctx_, addr, value);
            if (value_written == -1) {
                component_logger()->error("Failed to write holding register: {}.  Address={},Value={}", modbus_strerror(errno), addr, value);
            }

            return value_written;
        }


        /*  Write multiple coil bits (Modbus Function Code = 0x0F)
         *
         *  Writes the status of the 'nb' bits (coils) from 'coil_bits_' at the address 'addr' of the remote device.
         *  The coil_bits array must contains bytes set to TRUE or FALSE.
         *
         *  Returns return return the number of written bits if successful. Otherwise it shall return -1.
         */
        int ModbusUART::WriteCoilBits(int addr, int nb) {
            int bits_written = 0;

            bits_written = modbus_write_bits(ctx_, addr, nb, coil_bits_.get());
            if (bits_written == -1) {
                component_logger()->error("Failed to write coil bits: {}.  Address={}, #Bits={}", modbus_strerror(errno), addr, nb);
            }

            return bits_written;
        }


        /*  Write holding registers (Modbus Function Code = 0x10)
         *
         *  Writes the content of the 'nb' holding registers from the array 'holding_regs_' at address 'addr' of the remote device.
         *
         *  Returns return the number of written registers if successful. Otherwise it shall return -1.
         */
        int ModbusUART::WriteHoldingRegs(int addr, int nb) {
            int values_written = 0;

            values_written = modbus_write_registers(ctx_, addr, nb, holding_regs_.get());
            if (values_written == -1) {
                component_logger()->error("Failed to write holding registers: {}.  Address={}, #Regs={}", modbus_strerror(errno), addr, nb);
            }

            return values_written;
        }


        /*  Write and read many holding registers in a single transaction (Modbus Function Code = 0x17)
         *
         *  Writes the content of the 'write_nb' holding registers from the array 'src' to the address 'write_addr'
         *  of the remote device, then reads the content of the 'read_nb' holding registers to the address 'read_addr'
         *  of the remote device. The result of reading is stored in 'dest' array as word values (16 bits).
         *
         *  User must take care to allocate enough memory to store the results in 'dest' (at least nb * sizeof(uint16_t)).
         *
         *  Returns return the number of read registers if successful. Otherwise it shall return -1.
         */
        int ModbusUART::WriteReadHoldingRegs(int write_addr, int write_nb, int read_addr, int read_nb) {
            int regs_read = 0;

            regs_read = modbus_write_and_read_registers(ctx_, write_addr, write_nb, holding_regs_.get(), read_addr, read_nb, holding_regs_.get());
            if (regs_read == -1) {
                component_logger()->error("Failed to write/read holding registers: {}.  WriteAddr={}, #WriteRegs={}, ReadAddr={}, #ReadRegs={}",
                        modbus_strerror(errno), write_addr, write_nb, read_addr, read_nb);
            }

            return regs_read;
        }

        // riaps:keep_impl:end

        // riaps:keep_destruct:begin
        ModbusUART::~ModbusUART() {

        }
        // riaps:keep_destruct:end

    }
}

std::unique_ptr<riapsmodbuscreqrepuart::components::ModbusUART>
create_component_py(const py::object *parent_actor,
                    const py::dict actor_spec,
                    const py::dict type_spec,
                    const std::string &name,
                    const std::string &type_name,
                    const py::dict args,
                    const std::string &application_name,
                    const std::string &actor_name) {
    auto ptr = new riapsmodbuscreqrepuart::components::ModbusUART(parent_actor, actor_spec, type_spec, name, type_name, args,
                                                                     application_name,
                                                                     actor_name);
    return std::move(std::unique_ptr<riapsmodbuscreqrepuart::components::ModbusUART>(ptr));
}

PYBIND11_MODULE(libmodbusuart, m) {
    py::class_<riapsmodbuscreqrepuart::components::ModbusUART> testClass(m, "ModbusUART");
    testClass.def(py::init<const py::object*, const py::dict, const py::dict, const std::string&, const std::string&, const py::dict, const std::string&, const std::string&>());

    testClass.def("setup"                 , &riapsmodbuscreqrepuart::components::ModbusUART::Setup);
    testClass.def("activate"              , &riapsmodbuscreqrepuart::components::ModbusUART::Activate);
    testClass.def("terminate"             , &riapsmodbuscreqrepuart::components::ModbusUART::Terminate);
    testClass.def("handlePortUpdate"      , &riapsmodbuscreqrepuart::components::ModbusUART::HandlePortUpdate);
    testClass.def("handleCPULimit"        , &riapsmodbuscreqrepuart::components::ModbusUART::HandleCPULimit);
    testClass.def("handleMemLimit"        , &riapsmodbuscreqrepuart::components::ModbusUART::HandleMemLimit);
    testClass.def("handleSpcLimit"        , &riapsmodbuscreqrepuart::components::ModbusUART::HandleSpcLimit);
    testClass.def("handleNetLimit"        , &riapsmodbuscreqrepuart::components::ModbusUART::HandleNetLimit);
    testClass.def("handleNICStateChange"  , &riapsmodbuscreqrepuart::components::ModbusUART::HandleNICStateChange);
    testClass.def("handlePeerStateChange" , &riapsmodbuscreqrepuart::components::ModbusUART::HandlePeerStateChange);
    testClass.def("handleReinstate"       , &riapsmodbuscreqrepuart::components::ModbusUART::HandleReinstate);

    m.def("create_component_py", &create_component_py, "Instantiates the component from python configuration");
}

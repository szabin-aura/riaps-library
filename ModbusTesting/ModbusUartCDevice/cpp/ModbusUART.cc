#include <unistd.h>
#include <iostream>
#include <ModbusUART.h>

namespace globals {
  namespace components {

    ModbusUART::ModbusUART(_component_conf &config, riaps::Actor &actor)
        : ModbusUARTBase(config, actor) {
        PID = getpid();

        debugMode = false;
        SetDebugLevel(_logger, spdlog::level::level_enum::info);

        // Configure Modbus UART port parameters
        auto iport = GetConfig().component_parameters.GetParam("port");
        if (iport != nullptr) {
            std::string iportStr = iport->GetValueAsString();
            if (iportStr == "UART1") {
                portConfig.portname = "/dev/ttyO1";
            } else if (iportStr == "UART2") {
                portConfig.portname = "/dev/ttyO2";
            } else if (iportStr == "UART3") {
                portConfig.portname = "/dev/ttyO3";
            } else if (iportStr == "UART4") {
                portConfig.portname = "/dev/ttyO4";
            } else if (iportStr == "UART5") {
                portConfig.portname = "/dev/ttyO5";
            } else {
                _logger->error("{}: Invalid UART argument (port={}), use port = UART1..5", PID,
                               iport->GetValueAsString());
            }
        } else {
            // Declare a default value
            portConfig.portname = "/dev/ttyO2";
        }

        auto ibaudrate = GetConfig().component_parameters.GetParam("baudrate");
        if (ibaudrate != nullptr) {
            if(!ibaudrate->GetValueAsInt(&(portConfig.baudrate))) {
            	_logger->error("{}: Invalid Baud Rate argument - {}, should be integer value", PID, portConfig.baudrate);
            }
        } else {
            // Declare a default value
            portConfig.baudrate = 115200;
        }

        auto ibytesize = GetConfig().component_parameters.GetParam("bytesize");
        if (ibytesize != nullptr) {
            if(!ibytesize->GetValueAsInt(&(portConfig.bytesize))) {
            	_logger->error("{}: Invalid Byte Size argument - {}, should be integer value", PID, portConfig.bytesize);
            }
        } else {
            // Declare a default value
            portConfig.bytesize = 8;
        }

        auto iparity = GetConfig().component_parameters.GetParam("parity");
        if (iparity != nullptr) {
            portConfig.parity = iparity->GetValueAsString()[0];
        } else {
            // Declare a default value
            portConfig.parity = 'N';
        }

        auto istopbits = GetConfig().component_parameters.GetParam("stopbits");
        if (istopbits != nullptr) {
            if(!istopbits->GetValueAsInt(&(portConfig.stopbits))) {
            	_logger->error("{}: Invalid Stop Bits argument - {}, should be integer value", PID, portConfig.stopbits);
            }
        } else {
            // Declare a default value
            portConfig.stopbits = 1;
        }

        auto islaveaddress = GetConfig().component_parameters.GetParam("slaveaddress");
        if (islaveaddress != nullptr) {
            if(!islaveaddress->GetValueAsInt(&(portSlaveAddress))) {
            	_logger->error("{}: Invalid Port Address argument - {}, should be integer value", PID, portSlaveAddress);
            }
        } else {
            // Declare a default value
            portSlaveAddress = 10;
        }

        // Configure number of Modbus data registers used
        auto inumholdreg = GetConfig().component_parameters.GetParam("numholdreg");
        if (inumholdreg != nullptr) {
            if(!inumholdreg->GetValueAsInt(&(nb_holdingRegs))) {
            	_logger->error("{}: Invalid Number of Holding Registers argument - {}, should be integer value", PID, nb_holdingRegs);
            }
        } else {
            // Set a default of 3 (an arbritary number based on the first example created)
            nb_holdingRegs = 3;
        }

        auto inuminputreg = GetConfig().component_parameters.GetParam("numinputreg");
        if (inuminputreg != nullptr) {
            if(!inuminputreg->GetValueAsInt(&(nb_inputRegs))) {
            	_logger->error("{}: Invalid Number of Input Registers argument - {}, should be integer value", PID, nb_inputRegs);
            }
        } else {
            // Set a default of 4 (an arbritary number based on the first example created)
            nb_inputRegs = 4;
        }

        auto inumcoilbits = GetConfig().component_parameters.GetParam("numcoilbits");
        if (inumcoilbits != nullptr) {
            if(!inumcoilbits->GetValueAsInt(&(nb_coilBits))) {
            	_logger->error("{}: Invalid Number of Coil Bits argument - {}, should be integer value", PID, nb_coilBits);
            }
        } else {
            // Set a default of 8 (an arbritary number based on the first example created)
            nb_coilBits = 8;
        }

        auto inumdiscretebits = GetConfig().component_parameters.GetParam("numdiscretebits");
        if (inumdiscretebits != nullptr) {
            if(!inumdiscretebits->GetValueAsInt(&(nb_discreteBits))) {
            	_logger->error("{}: Invalid Number of Discreet Bits argument - {}, should be integer value", PID, nb_discreteBits);
            }
        } else {
            // Set a default of 8 (an arbritary number based on the first example created)
            nb_discreteBits = 8;
        }

        // Modbus port initially closed
        portOpen = false;
        portSerialMode = MODBUS_RTU_RS232;

        // Setup Modbus Data Registers
        holding_regs = std::unique_ptr<uint16_t[]>(new uint16_t[nb_holdingRegs]);
        input_regs = std::unique_ptr<uint16_t[]>(new uint16_t[nb_inputRegs]);
        coil_bits = std::unique_ptr<uint8_t[]>(new uint8_t[nb_coilBits]);
        discrete_bits = std::unique_ptr<uint8_t[]>(new uint8_t[nb_discreteBits]);

        _logger->info("{}: Modbus UART settings {} @{}:{} {}{}{}", PID, portSlaveAddress, portConfig.portname,
                      portConfig.baudrate, portConfig.bytesize, portConfig.parity, portConfig.stopbits);
        _logger->info(
                "{}: Modbus Reg settings: numCoilBits={}, numDiscreteBits={}, numInputRegs={}, numHoldingRegs={}",
                PID, nb_coilBits, nb_discreteBits, nb_inputRegs, nb_holdingRegs);

        /* Initialize the libmodbus context
        * Returns a pointer to a modbus_t structure if successful. Otherwise it shall return NULL.
        *   where, baudrate: 9600, 19200, 57600, 115200, etc
        *          Parity: 'N', 'O', 'E'
        *          Slave Address is a decimal value
        *          Serial Mode is either RS232 (0) or RS485 (1) -  RS232 mode is default
        */
        ctx = modbus_new_rtu(portConfig.portname.c_str(), portConfig.baudrate, portConfig.parity,
                             portConfig.bytesize, portConfig.stopbits);
        if (ctx == NULL) {
            _logger->error("Unable to create the libmodbus context");
        }

        //_logger->debug("Init vars");
        //int resResult = clock_getres(CLOCK_MONOTONIC, &resolution);

        //if (resResult != 0) {
        //    _logger->debug("Error occurred {}", strerror(errno));
        //    exit(errno);
        //}
    }

    void ModbusUART::OnModbusRepPort(const CommandFormat::Reader &message,
                                     riaps::ports::PortBase *port) {
      std::cout << "ModbusUART::OnModbusRepPort()" << std::endl;

      int responseValue = -1;  // invalid response

      if (isModbusReady()) {
          //clock_gettime(CLOCK_MONOTONIC, &preobservations);
          responseValue = sendModbusCommand(message);

          capnp::MallocMessageBuilder messageRepBuilder;
          ResponseFormat::Builder msgModbusResponse = messageRepBuilder.initRoot<ResponseFormat>();
          msgModbusResponse.setCommandType(message.getCommandType());
          msgModbusResponse.setRegisterAddress(message.getRegisterAddress());
          msgModbusResponse.setNumberOfRegs(responseValue);  // for writes, this value is 1 if successful, -1 if failed

          if (responseValue > 0) {
              auto values = msgModbusResponse.initValues(responseValue);

              if (message.getCommandType() == ModbusCommands::READ_COIL_BITS) {
                  for (int i = 0; i <= responseValue; i++) {
                      values.set(i, (uint16_t) coil_bits.get()[i]);
                  }
              } else if (message.getCommandType() == ModbusCommands::READ_INPUT_BITS) {
                  for (int i = 0; i <= responseValue; i++) {
                      values.set(i, (uint16_t) discrete_bits.get()[i]);
                  }
              } else if (message.getCommandType() == ModbusCommands::READ_INPUT_REGS) {
                  for (int i = 0; i <= responseValue; i++) {
                      values.set(i, (uint16_t) input_regs.get()[i]);
                  }
              } else if ((message.getCommandType() == ModbusCommands::READ_HOLDING_REGS) ||
                         (message.getCommandType() == ModbusCommands::WRITE_READ_HOLDING_REGS)) {
                  for (int i = 0; i <= responseValue; i++) {
                      values.set(i, (uint16_t) holding_regs.get()[i]);
                  }
              }
          }

          // Send response back to requester
          _logger->warn_if(!SendModbusRepPort(messageRepBuilder, msgModbusResponse),
                           "{}: Couldn't send response message", PID);
      } else {
          _logger->warn("{}: Modbus is not ready for commands", PID);
      }

      //if (debugMode) {
          //clock_gettime(CLOCK_MONOTONIC, &postobservations);
          //system_time_sub(&postobservations, &preobservations, &result);
          //_logger->debug("After Response sent: tv.sec={} tv_nsec={}",result.tv_sec,((double)result.tv_nsec)/NSEC_PER_SEC);
      //}
    }

    void ModbusUART::OnClock(riaps::ports::PortBase *port) {
      // if missing modbus commands, consider commenting this message out
      std::cout << "ModbusUART::OnClock(): " << port->GetPortName()
                << std::endl;
      if (!isModbusReady() && (ctx != NULL)) {
          // Start Modbus
          if (startRTUModbus(portSerialMode)) {
        	  _logger->debug("{}: Started Modbus: port={}, slaveAddress={}", PID, portConfig.portname, portSlaveAddress);
          }
      }

      // Halt clock port
      auto timerPort = GetPortByName(PORT_TIMER_CLOCK);
      if (timerPort != nullptr) {
          auto t = timerPort->AsTimerPort();
          if (t != nullptr) {
              t->stop();
              _logger->info("{}: Stopped Clock port", PID);
          }
      }
    }

    int ModbusUART::sendModbusCommand(const CommandFormat::Reader &message) {
        int value = 999;   // Return from the commands - read return number of items read, write provides success (1) / fail (0)
        int writeValue;

        int16_t regAddress = message.getRegisterAddress();
        int16_t numRegs = message.getNumberOfRegs();
        std::string valueLogMsg;


        switch (message.getCommandType()) {
            case ModbusCommands::READ_COIL_BITS:
                value = readCoilBits(regAddress, numRegs);
                _logger->debug("{}: Sent command {}, register={}, numOfDecimals={}", PID, "READ_COILBITS",
                               regAddress, numRegs);
                break;
            case ModbusCommands::READ_INPUT_BITS:
                value = readInputBits(regAddress, numRegs);
                _logger->debug("{}: Sent command {}, register={}, numOfDecimals={}", PID, "READ_INPUTBITS",
                               regAddress, numRegs);
                break;
            case ModbusCommands::READ_INPUT_REGS:
                value = readInputRegs(regAddress, numRegs);
                _logger->debug("{}: Sent command {}, register={}, numOfRegs={}", PID, "READ_INPUTREGS",
                               regAddress, numRegs);
                break;
            case ModbusCommands::READ_HOLDING_REGS:
                value = readHoldingRegs(regAddress, numRegs);
                _logger->debug("{}: Sent command {}, register={}, numOfRegs={}", PID, "READ_HOLDINGREGS",
                               regAddress, numRegs);
                break;
            case ModbusCommands::WRITE_COIL_BIT:

                writeValue = (int) message.getValues()[0];
                value = writeCoilBit(regAddress, writeValue);
                _logger->debug("{}: Sent command {}, register={}, value={}", PID, "WRITE_COILBIT", regAddress,
                               writeValue);
                break;
            case ModbusCommands::WRITE_HOLDING_REG:
                writeValue = (int) message.getValues()[0];
                value = writeHoldingReg(regAddress, writeValue);
                _logger->debug("{}: Sent command {}, register={}, value={}", PID, "WRITE_HOLDINGREG", regAddress,
                               writeValue);
                break;
            case ModbusCommands::WRITE_COIL_BITS:
                for (int i = 0; i <= numRegs; i++) {
                    coil_bits[i] = (uint8_t) message.getValues()[i];
                    valueLogMsg.append(std::to_string(coil_bits[i]) + " ");
                }
                value = writeCoilBits(regAddress, numRegs);
                _logger->debug("{}: Sent command {}, register={}, numberOfDecimals={}", PID, "WRITE_COILBITS",
                               regAddress, numRegs);
                _logger->debug("{}: Values:{}", PID, valueLogMsg);
                break;
            case ModbusCommands::WRITE_MULTI_HOLDING_REGS:
                for (int i = 0; i <= numRegs; i++) {
                    holding_regs[i] = (uint16_t) message.getValues()[i];
                    valueLogMsg.append(std::to_string(holding_regs[i]) + " ");
                }
                value = writeHoldingRegs(regAddress, numRegs);
                _logger->debug("{}: Sent command {}, register={}, numOfRegs={}", PID, "WRITEMULTI_HOLDINGREGS",
                               regAddress, numRegs);
                _logger->debug("{}: Values:{}", PID, valueLogMsg);
                break;
            case ModbusCommands::WRITE_READ_HOLDING_REGS:
                for (int i = 0; i <= numRegs; i++) {
                    holding_regs[i] = (uint16_t) message.getValues()[i];
                    valueLogMsg.append(std::to_string(holding_regs[i]) + " ");
                }
                value = writeReadHoldingRegs(regAddress, numRegs, message.getWreadRegAddress(),
                                             message.getWreadNumOfRegs());
                _logger->debug("{}: Sent command {}, writeReg={}, numOfRegsWrite={}, readReg={}, numOfRegsRead={}",
                               PID, "WRITEREAD_HOLDINGREGS", regAddress, numRegs, message.getWreadRegAddress(),
                               message.getWreadNumOfRegs());
                _logger->debug("{}: Values:{}", PID, valueLogMsg);
                break;
            default:
                _logger->warn("{}: Invalid Command - {}", PID, (int16_t) message.getCommandType());
        }

        return value;
    }

    void ModbusUART::OnGroupMessage(const riaps::groups::GroupId &groupId,
                                    capnp::FlatArrayMessageReader &capnpreader,
                                    riaps::ports::PortBase *port) {}

    ModbusUART::~ModbusUART() {
        if (isModbusReady()) {
            closeModbus();
        }
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
    bool ModbusUART::startRTUModbus(int serialMode) {

        // Enable debugging of libmodbus
        modbus_set_debug(ctx, true);

        modbus_set_slave(ctx, portSlaveAddress);

        if (serialMode == MODBUS_RTU_RS485) {
            if (modbus_rtu_set_serial_mode(ctx, serialMode) == -1) {
                _logger->error("Invalid Serial Mode: {}.  SerialMode={}", modbus_strerror(errno), serialMode);
                modbus_free(ctx);
                return false;
            } else {
                _logger->debug("Modbus serial mode set to RS485");
            }
        }

        if (modbus_connect(ctx) == -1) {
            _logger->error("Connection failed: {}", modbus_strerror(errno));
            modbus_free(ctx);
            return false;
        } else {
            portOpen = true;
            _logger->info("Modbus port open. Port={}, Baud={}, {}{}{}", portConfig.portname, portConfig.baudrate,
                          portConfig.bytesize, portConfig.parity, portConfig.stopbits);
        }

        return portOpen;
    }


    /* Close Modbus */
    void ModbusUART::closeModbus(void) {
        modbus_close(ctx);
        modbus_free(ctx);
        portOpen = false;
        _logger->info("Modbus closed");
    }


    /* Is Modbus Ready?
     * User should make sure modbus is open before reading or writing bits or registers.
     */
    bool ModbusUART::isModbusReady(void) {
        return portOpen;
    }

    /*  Read Coil Bit Status (Modbus Function Code = 0x01)
     *
     *  Reads the status of the 'nb' bits (coils) to the address 'addr' of the remote device. The result of reading
     *  is stored in 'coil_bits' array as unsigned bytes (8 bits) set to TRUE or FALSE.
     *  User must take care to allocate enough memory to store the results in 'coil_bits' (at least nb * sizeof(uint8_t)).
     *
     *  Returns the number of bits read if successful. Otherwise it shall return -1.
     */
    int ModbusUART::readCoilBits(int addr, int nb) {
        int bitsRead = 0;

        bitsRead = modbus_read_bits(ctx, addr, nb, coil_bits.get());
        if (bitsRead == -1) {
            _logger->error("Failed to read coil bits: {}.  Address={}, #Bits={}", modbus_strerror(errno), addr, nb);
        }

        return bitsRead;
    }


    /*  Read Input Bits or Status (Modbus Function Code = 0x02)
     *
     *  Reads the content of the 'nb' input bits to the address 'addr' of the remote device. The result of reading
     *  is stored in 'discrete_bits' array as unsigned bytes (8 bits) set to TRUE or FALSE.
     *
     *  User must take care to allocate enough memory to store the results in 'discrete_bits' (at least nb * sizeof(uint8_t)).
     *
     *  Returns the number of bits read if successful. Otherwise it shall return -1.
     */
    int ModbusUART::readInputBits(int addr, int nb) {
        int bitsRead = 0;

        bitsRead = modbus_read_input_bits(ctx, addr, nb, discrete_bits.get());
        if (bitsRead == -1) {
            _logger->error("Failed to read input bits: {}.  Address={}, #Bits={}", modbus_strerror(errno), addr, nb);
        }

        return bitsRead;
    }


    /*  Read Holding Registers (Modbus Function Code = 0x03)
     *
     *  Reads the content of the 'nb' holding registers to the address 'addr' of the remote device.
     *  The result of reading is stored in 'holding_regs' array as word values (16 bits).
     *  User must take care to allocate enough memory to store the results in 'holding_regs' (at least nb * sizeof(uint16_t)).
     *
     *  Returns the number of holding registers read if successful. Otherwise it shall return -1.
     */
    int ModbusUART::readHoldingRegs(int addr, int nb) {
        int regsRead = 0;

        regsRead = modbus_read_registers(ctx, addr, nb, holding_regs.get());
        if (regsRead == -1) {
            _logger->error("Failed to read holding registers: {}.  Address={}, #Reg={}", modbus_strerror(errno), addr, nb);
        }

        return regsRead;
    }


    /*  Read Input Registers (Modbus Function Code = 0x04)
     *
     *  Reads the content of the 'nb' input registers to address 'addr' of the remote device.
     *  The result of the reading is stored in 'input_regs' array as word values (16 bits).
     *  User must take care to allocate enough memory to store the results in 'input_regs' (at least nb * sizeof(uint16_t)).
     *
     *  Returns the number of input registers read if successful. Otherwise it shall return -1.
     */
    int ModbusUART::readInputRegs(int addr, int nb) {
        int regsRead = 0;

        regsRead = modbus_read_input_registers(ctx, addr, nb, input_regs.get());
        if (regsRead == -1) {
            _logger->error("Failed to read input registers: {}.  Address={}, #Reg={}", modbus_strerror(errno), addr, nb);
        }

        return regsRead;
    }


    /*  Write a single coil bit (Modbus Function Code = 0x05)
     *
     *  Writes the status of 'status' at the address 'addr' of the remote device.
     *  The value must be set to TRUE or FALSE.
     *
     *  Returns return 1 if successful. Otherwise it shall return -1.
     */
    int ModbusUART::writeCoilBit(int addr, int status) {
        int bitWritten = 0;

        bitWritten = modbus_write_bit(ctx, addr, status);
        if (bitWritten == -1) {
            _logger->error("Failed to write coil bit: {}.  Address={}, Status={}", modbus_strerror(errno), addr, status);
        }

        return bitWritten;
    }


    /*  Write a single register (Modbus Function Code = 0x06)
     *
     *  Writes the value of 'value' holding registers at the address 'addr' of the remote device.
     *
     *  Returns return 1 if successful. Otherwise it shall return -1.
     */
    int ModbusUART::writeHoldingReg(int addr, int value) {
        int valueWritten = 0;

        valueWritten = modbus_write_register(ctx, addr, value);
        if (valueWritten == -1) {
            _logger->error("Failed to write holding register: {}.  Address={},Value={}", modbus_strerror(errno), addr, value);
        }

        return valueWritten;
    }


    /*  Write multiple coil bits (Modbus Function Code = 0x0F)
     *
     *  Writes the status of the 'nb' bits (coils) from 'coil_bits' at the address 'addr' of the remote device.
     *  The coil_bits array must contains bytes set to TRUE or FALSE.
     *
     *  Returns return return the number of written bits if successful. Otherwise it shall return -1.
     */
    int ModbusUART::writeCoilBits(int addr, int nb) {
        int bitsWritten = 0;

        bitsWritten = modbus_write_bits(ctx, addr, nb, coil_bits.get());
        if (bitsWritten == -1) {
            _logger->error("Failed to write coil bits: {}.  Address={}, #Bits={}", modbus_strerror(errno), addr, nb);
        }

        return bitsWritten;
    }


    /*  Write holding registers (Modbus Function Code = 0x10)
     *
     *  Writes the content of the 'nb' holding registers from the array 'holding_regs' at address 'addr' of the remote device.
     *
     *  Returns return the number of written registers if successful. Otherwise it shall return -1.
     */
    int ModbusUART::writeHoldingRegs(int addr, int nb) {
        int valuesWritten = 0;

        valuesWritten = modbus_write_registers(ctx, addr, nb, holding_regs.get());
        if (valuesWritten == -1) {
            _logger->error("Failed to write holding registers: {}.  Address={}, #Regs={}", modbus_strerror(errno), addr, nb);
        }

        return valuesWritten;
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
    int ModbusUART::writeReadHoldingRegs(int write_addr, int write_nb, int read_addr, int read_nb) {
        int regsRead = 0;

        regsRead = modbus_write_and_read_registers(ctx, write_addr, write_nb, holding_regs.get(), read_addr, read_nb, holding_regs.get());
        if (regsRead == -1) {
            _logger->error("Failed to write/read holding registers: {}.  WriteAddr={}, #WriteRegs={}, ReadAddr={}, #ReadRegs={}",
                    modbus_strerror(errno), write_addr, write_nb, read_addr, read_nb);
        }

        return regsRead;
    }

  }
}

riaps::ComponentBase *create_component(_component_conf &config,
                                       riaps::Actor &actor) {
  auto result = new globals::components::ModbusUART(config, actor);
  return result;
}

void destroy_component(riaps::ComponentBase *comp) { delete comp; }

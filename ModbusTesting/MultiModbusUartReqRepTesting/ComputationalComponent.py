'''
This example shows setting up two Modbus connections utilizing a Modbus Device Manager.
For this setup, the serial port configurations will be:
    1)  Modbus1 = slaveaddress=10,port="UART2",baudrate=19200,bytesize=serial.EIGHTBITS,parity=serial.PARITY_NONE,stopbits=serial.STOPBITS_ONE,serialTimeout=0.05
    2)  Modbus2 = slaveaddress=12,port="UART2",baudrate=19200,bytesize=serial.EIGHTBITS,parity=serial.PARITY_NONE,stopbits=serial.STOPBITS_ONE,serialTimeout=0.05

Any of the provided parameters can be adjusted for each modbus device, they are separate modbus instances at the device level.
Assume serial port setup parameters are valid (no error checking)

Created on Apr 9, 2019
@author: riaps
'''

import zmq
from riaps.run.comp import Component
from riaps.run.exc import PortError
import uuid
import os
from collections import namedtuple
import serial.serialutil
from ModbusUartDeviceMgr import CommandFormat,StatusResponse,ModbusCommands
from serialModbusLib.serialModbusComm import PortConfig
#import pydevd
import time
import logging


# Enable debugging to gather timing information on the code execution or see command feedback
debugMode = False

RegSet = namedtuple('RegSet', ['idx', 'value'])
InputRegs = namedtuple('InputRegs', ['outputCurrent','outputVolt','voltPhase','time'])

# For Inverter control
HoldingRegs = namedtuple('HoldingRegs',['unused', 'startStopCmd', 'power'])

''' For RIAPS future
1.start/stop, 2.power command, 3. frequency shift from secondary control, 4. voltage magnitude shift from secondary control.
HoldingRegs = namedtuple('HoldingRegs',['startStopCmd', 'powerCmd', 'freqShift', 'voltMagShift'])
'''

class AppModbusConfig:
    def __init__(self, aSlaveAddr, aPortConfig):
        self.slaveAddr = aSlaveAddr
        self.portConfig = aPortConfig
        self._modbusReady = False
        self._modbusUnusable = False

    def setModbusReady(self):
        self._modbusReady = True

    def setUnusableIndication(self):
        self._modbusUnusable = True

    def isModbusNonfunctional(self):
        return self._modbusUnusable

    def isModbusOperational(self):
        operational = False
        if self._modbusReady and not self._modbusUnusable:
            operational = True
        return operational


class ComputationalComponent(Component):
    def __init__(self):
        super().__init__()
        #pydevd.settrace(host='192.168.1.103',port=5678)
        self.uuid = uuid.uuid4().int
        self.pid = os.getpid()
        self.inputRegs = InputRegs(RegSet(0,45),RegSet(1,56),RegSet(2,78),RegSet(3,91))
        self.holdingRegs = HoldingRegs(RegSet(0,0),RegSet(1,55),RegSet(2,66))

        self.message_sent = False

        # Setup Commands
        self.defaultNumOfRegs = 1
        self.dummyValue = [0]
        self.defaultNumOfDecimals = 0
        self.signedDefault = False
        self.ModbusPending = 0
        self.allModbusReady = False

        # Used list to cycle through commands, expect user to use keywords in their state machine
        self.cmdList = ['sensor','breaker']
        self.currentCmd = 0

        # Multiple Modbus Configuration Information
        self.modbusInstances = dict()
        self.sensorSlaveAddr = 10
        self.breakerSlaveAddr = 12
        # PortConfig = portname, baudrate, bytesize, parity, stopbits, serial timeout
        #    using user defined portname, the ModbusUartDeviceMgr will translate to the value needed by pyserial (/dev/ttyOx)
        self.sensorPortConfig = PortConfig('UART2', 115200, serial.EIGHTBITS, serial.PARITY_NONE, serial.STOPBITS_ONE, 0.05)
        self.breakerPortConfig = PortConfig('UART2', 115200, serial.EIGHTBITS, serial.PARITY_NONE, serial.STOPBITS_ONE, 0.05)

        # Create Modbus configuration for specific Modbus Instances
        self.sensorModbusConfig = AppModbusConfig(self.sensorSlaveAddr,self.sensorPortConfig)
        self.breakerModbusConfig = AppModbusConfig(self.breakerSlaveAddr,self.breakerPortConfig)

        # Modbus Instances Dictionary Values:  slave address, serial port configuration, ready for commands (started up)
        self.modbusInstances = {'sensor':self.sensorModbusConfig, 'breaker':self.breakerModbusConfig}
        self.logger.info("ComputationalComponent: %s - starting" % str(self.pid))


    def on_clock(self):
        now = self.clock.recv_pyobj()
        self.logger.info("on_clock()[%s]: %s" % (str(self.pid),str(now)))

        '''
        Status specific modbus using slave address and serial port configuration
        First status call with a specific modbus setup will activate a new modbus instance.
        Subsequent calls will obtain status.
        '''
        if not self.allModbusReady:
            numModbus = len(self.modbusInstances)
            numReady = 0
            for x in self.modbusInstances.keys():
                # See if modbus instance is ready for operation, if so count as ready and go to next instance
                if self.modbusInstances[x].isModbusOperational():
                    numReady += 1
                # If not ready because it has not been tried yet
                elif not self.modbusInstances[x].isModbusNonfunctional():
                    # Halt clock while waiting for the status
                    self.clock.halt()
                    self.logger.info("clock halted")

                    # If status=true, set flag and restart clock
                    try:
                        if not self.message_sent:
                            # Save pending status information (using response format since it has most elements desired to save)
                            self.pStatusCmd = StatusResponse(self.modbusInstances[x].slaveAddr, self.modbusInstances[x].portConfig.portname,False)
                            msg = (self.modbusInstances[x].slaveAddr, self.modbusInstances[x].portConfig)
                            self.modbusStatusReqPort.send_pyobj(msg)
                            self.logger.debug("Modbus status requested - (slaveAddr=%d,port=%s)" % (self.pStatusCmd.slaveAddr,self.pStatusCmd.portName))
                            self.message_sent = True
                        if self.message_sent:
                            modStatus = self.modbusStatusReqPort.recv_pyobj()
                            self.logger.debug("Modbus status received - (slaveAddr=%d,port=%s)" % (modStatus.slaveAddr,modStatus.portName))
                            self.message_sent = False
                            if modStatus.modbusReady:
                                if (modStatus.slaveAddr == self.pStatusCmd.slaveAddr) and (modStatus.portName == self.pStatusCmd.portName):
                                    self.modbusInstances[x].setModbusReady()
                                    numReady += 1
                                    self.logger.info("Modbus is ready, clock is restarted - (slaveAddr=%d,port=%s)" % (modStatus.slaveAddr,modStatus.portName))
                                else:
                                    self.logger.info("Received status for different modbus instance than expected - (received: slaveAddr=%d,port=%s)" % (modStatus.slaveAddr,modStatus.portName))
                            else:
                                self.logger.info("Modbus is not ready yet (received: slaveAddr=%d,port=%s)" % (modStatus.slaveAddr,modStatus.portName))
                    except PortError as e:
                        self.logger.error("on_clock-modbusStatusReq:Modbus port exception = %d - (slaveAddr=%d,port=%s)" % (e.errno,self.modbusInstances[x].slaveAddr,self.modbusInstances[x].portConfig.portname))
                        if e.errno in (PortError.EAGAIN,PortError.EPROTO):
                            self.logger.error("on_clock-modbusStatusReq: port error received")
                        #self.logger.error("Modbus is considered not usable")
                        #self.modbusInstances[x].setUnusableIndication()

                    self.clock.launch()
                    self.logger.info("clock restarted")
                else:
                    self.logger.info("Modbus instance known to be none functional (previous error provided) - (slaveAddr=%d,port=%s)" % (self.modbusInstances[x].slaveAddr,self.modbusInstances[x].portConfig.portname))

            # Looking for all Modbus instances to be ready before sending commands
            if numReady == numModbus:
                self.allModbusReady = True
                self.logger.info("All Modbus Devices are now ready and available for commands")

        # Ready to send Modbus Commands, cycle through available commands
        else:
            # Send Command
            if self.ModbusPending == 0:
                #if debugMode:
                #    self.cmdSendStartTime = time.perf_counter()
                #    self.logger.debug("on_clock()[%s]: Send command to ModbusUartDevice at %f" % (str(self.pid),self.cmdSendStartTime))

                # Get current command's modbus instance
                cModInst = self.modbusInstances[self.cmdList[self.currentCmd]]

                # Request:  Commands to send over Modbus - one command used at a time
                '''
                Read/Write (holding only) a single register
                self.command = CommandFormat(cModInst.slaveAddr,
                                             cModInst.portConfig.portname,
                                             ModbusCommands.READ_INPUTREG,
                                             self.inputRegs.time.idx,
                                             self.defaultNumOfRegs,
                                             self.dummyValue,
                                             self.defaultNumOfDecimals,
                                             self.signedDefault)

                self.command = CommandFormat(cModInst.slaveAddr,
                                             cModInst.portConfig.portname,
                                             ModbusCommands.READ_HOLDINGREG,
                                             self.holdingRegs.startStopCmd.idx,
                                             self.defaultNumOfRegs,
                                             self.dummyValue,
                                             self.defaultNumOfDecimals,
                                             self.signedDefault)
                self.values = [83]
                self.command = CommandFormat(cModInst.slaveAddr,
                                             cModInst.portConfig.portname,
                                             ModbusCommands.WRITE_HOLDINGREG,
                                             self.holdingRegs.power.idx,
                                             self.defaultNumOfRegs,
                                             self.values,self.defaultNumOfDecimals,
                                             self.signedDefault)
                '''

                # Read/Write all input registers
                numRegsToRead = len(self.inputRegs)
                self.command = CommandFormat(cModInst.slaveAddr,
                                             cModInst.portConfig.portname,
                                             ModbusCommands.READMULTI_INPUTREGS,
                                             self.inputRegs.outputCurrent.idx,
                                             numRegsToRead,
                                             self.dummyValue,
                                             self.defaultNumOfDecimals,
                                             self.signedDefault)

                '''
                Read/Write all holding registers
                self.command = CommandFormat(cModInst.slaveAddr,
                                             cModInst.portConfig.portname,
                                             ModbusCommands.READMULTI_HOLDINGREGS,
                                             self.holdingRegs.unused.idx,
                                             len(self.holdingRegs),
                                             self.dummyValue,
                                             self.defaultNumOfDecimals,
                                             self.signedDefault)
                self.values = [75,67]
                self.command = CommandFormat(cModInst.slaveAddr,
                                             cModInst.portConfig.portname,
                                             ModbusCommands.WRITEMULTI_HOLDINGREGS,
                                             self.holdingRegs.startStopCmd.idx,
                                             2,
                                             self.values,
                                             self.defaultNumOfDecimals,
                                             self.signedDefault)
                '''

                msg = self.command

                try:
                    self.modbusCommandReqPort.send_pyobj(msg)
                    self.ModbusPending += 1
                    self.logger.info("Modbus command sent for (slaveAddr=%d,port=%s)" % (cModInst.slaveAddr,cModInst.portConfig.portname))

                    # Prepare for next command
                    self.currentCmd += 1
                    if self.currentCmd == len(self.cmdList):
                        self.currentCmd = 0
                        self.logger.debug("Reset to beginning of command list")
                except PortError as e:
                    self.logger.error("on_clock-modbusCommandReqPort:send exception = %d - (slaveAddr=%d,port=%s)" % (e.errno,cModInst.slaveAddr,cModInst.portConfig.portname))
                    if e.errno in (PortError.EAGAIN,PortError.EPROTO):
                        self.logger.error("on_clock-modbusCommandReqPort: port error received")
            else:
                self.logger.info("Waiting for previous Modbus command reply")


    def on_modbusCommandReqPort(self):
        # Receive Response
        try:
            msg = self.modbusCommandReqPort.recv_pyobj()
            self.ModbusPending -= 1
            self.logger.info("Modbus command response received")
        except PortError as e:
            self.logger.error("on_modbusCommandReqPort:receive exception = %d" % e.errno)
            if e.errno in (PortError.EAGAIN,PortError.EPROTO):
                self.logger.error("on_modbusCommandReqPort: port error received")

        #if debugMode:
        #    self.cmdResultsRxTime = time.perf_counter()
        #    self.logger.debug("on_modbusCommandReqPort()[%s]: Received Modbus data=%s from ModbusUartDevice at %f, time from cmd to data is %f ms" % (str(self.pid),repr(msg),self.cmdResultsRxTime,(self.cmdResultsRxTime-self.cmdSendStartTime)*1000))

        if self.command.commandType == ModbusCommands.READ_INPUTREG or self.command.commandType == ModbusCommands.READ_HOLDINGREG:
            logMsg = "Register " + str(self.command.registerAddress) + " value is " + str(msg)
        elif self.command.commandType == ModbusCommands.READMULTI_INPUTREGS or self.command.commandType == ModbusCommands.READMULTI_HOLDINGREGS:
            logMsg = "Register " + str(self.command.registerAddress) + " values are " + str(msg)
        elif self.command.commandType == ModbusCommands.WRITE_HOLDINGREG:
            logMsg = "Wrote Register " + str(self.command.registerAddress)
        elif self.command.commandType == ModbusCommands.WRITEMULTI_HOLDINGREGS:
            logMsg = "Wrote Registers " + str(self.command.registerAddress) + " to " + str(self.command.registerAddress + self.command.numberOfRegs - 1)

        self.tx_modbusData.send_pyobj(logMsg)  # Send log data


    def __destroy__(self):
        self.logger.info("[%d] destroyed" % self.pid)

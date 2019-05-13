'''
Created on Mar 14, 2017

@author: riaps

This module utilizes the MinimalModbus (which utilizes pySerial).
Both need to be installed in the development environment.
    $ sudo pip3 install minimalmodbus  (which should install pySerial)
'''

from riaps.run.comp import Component
import os
import serial
from serialModbusLib.serialModbusComm import SerialModbusComm,PortConfig
from collections import namedtuple
from enum import Enum
#import pydevd
import time
import sys


# Enable debugging to gather timing information on the code execution or see command feedback
debugMode = False

class ModbusCommands(Enum):
    READ_BIT = 1
    READ_INPUTREG = 2
    READ_HOLDINGREG = 3
    READMULTI_INPUTREGS = 4
    READMULTI_HOLDINGREGS = 5
    WRITE_BIT = 6
    WRITE_HOLDINGREG = 7
    WRITEMULTI_HOLDINGREGS = 8

# Slave Address and Serial Port Name will be used to index the modbus instance desired for the command
CommandFormat = namedtuple('CommandFormat', ['slaveAddr','serialPort','commandType','registerAddress','numberOfRegs','values','numberOfDecimals','signedValue'])
StatusResponse = namedtuple('StatusResponse', ['slaveAddr','portName','modbusReady'])


class ModbusUartDeviceMgr(Component):
    def __init__(self): # defaults for Modbus spec
        super().__init__()
        self.pid = os.getpid()
        self.statusRequestPending = 0
        self.commandRequestPending = 0

        # Dictionary of open modbus instances
        self.modbusInstances = dict()
        self.logger.info("Modbus device component initialized")
        #pydevd.settrace(host='192.168.1.103',port=5678)

    def createModbusInstance(self):
        validSerialPort = True

        if self.statusSerialPortConfig.portname == 'UART1':
            port = '/dev/ttyO1'
        elif self.statusSerialPortConfig.portname == 'UART2':
            port = '/dev/ttyO2'
        elif self.statusSerialPortConfig.portname == 'UART3':
            port = '/dev/ttyO3'
        elif self.statusSerialPortConfig.portname == 'UART4':
            port = '/dev/ttyO4'
        elif self.statusSerialPortConfig.portname == 'UART5':
            port = '/dev/ttyO5'
        else:
            self.logger.error("__init__[%s]: Invalid UART argument, use UART1..5" % self.pid)
            validSerialPort = False

        # invalid port configuration will be caught in pyserial initialization
        if validSerialPort:
            nPortConfig = PortConfig(port,
                                     self.statusSerialPortConfig.baudrate,
                                     self.statusSerialPortConfig.bytesize,
                                     self.statusSerialPortConfig.parity,
                                     self.statusSerialPortConfig.stopbits,
                                     self.statusSerialPortConfig.serialTimeout)
            nModbusInst = SerialModbusComm(self.statusSlaveAddr, nPortConfig)
            self.logger.info("Modbus settings %d @%s:%d %d%s%d [%d]" % (self.statusSlaveAddr,
                                                                        nPortConfig.portname,
                                                                        nPortConfig.baudrate,
                                                                        nPortConfig.bytesize,
                                                                        nPortConfig.parity,
                                                                        nPortConfig.stopbits,
                                                                        self.pid))
            nModbusInst.startModbus()
            self.logger.info("Modbus instance started")

            # Add new modbus instance to Dictionary
            self.modbusInstances[(self.statusSlaveAddr,self.statusSerialPortConfig.portname)] = nModbusInst
            self.logger.info("Current Modbus Instances: ")
            for x in self.modbusInstances.keys():
                self.logger.info("(%d, %s)" % (x[0],x[1]))

    def __destroy__(self):
        for x in self.modbusInstances.keys():
            self.modbusInstances[x].stopModbus()
            self.logger.info("Modbus Instance Stopped:")
            self.logger.info("(%d, %s)" % (x[0],x[1]))

        self.logger.info("__destroy__")

    '''
    Receive a Modbus status request.  Send back state of the modbusReady flag
    '''
    def on_modbusStatusRepPort(self):
        self.logger.debug("start on_modbusStatusReqPort")
        try:
            (self.statusSlaveAddr,self.statusSerialPortConfig) = self.modbusStatusRepPort.recv_pyobj()
            self.statusRequestPending += 1
            self.logger.debug("Request for Modbus device status received")
        except PortError as e:
            self.logger.error("on_modbusStatusRepPort:receive exception = %d" % e.errno)
            if e.errno in (PortError.EAGAIN,PortError.EPROTO):
                self.logger.error("on_modbusStatusRepPort: port error received")

        if self.statusRequestPending == 1:
            # If modbus instance not in dictionary, start a new modbus instance with the given configuration
            if (self.statusSlaveAddr,self.statusSerialPortConfig.portname) not in self.modbusInstances:
                self.createModbusInstance()
            # Request status from modbus instance
            try:
                modbusToStatus = self.modbusInstances[(self.statusSlaveAddr,self.statusSerialPortConfig.portname)]
                modbusAvail = modbusToStatus.isModbusAvailable()
                if modbusAvail:
                    self.logger.info("Modbus is available")
                else:
                    self.logger.info("Modbus is not available")

                cStatusResponse = StatusResponse(self.statusSlaveAddr,self.statusSerialPortConfig.portname,modbusAvail)
                self.modbusStatusRepPort.send_pyobj(cStatusResponse)
                self.statusRequestPending -= 1
                self.logger.info("Response for Modbus device status sent")
            except PortError as e:
                self.logger.error("on_modbusStatusRepPort:send exception = %d" % e.errno)
                if e.errno in (PortError.EAGAIN,PortError.EPROTO):
                    self.logger.error("on_modbusStatusRepPort: port error received")
        elif self.statusRequestPending > 1:
            # This should not happen if the requesting component has the appropriate error handling for request/reply
            self.logger.error("A status request is pending, no additional request was made")


    '''
    Receive a Modbus command request.  Process command and send back response.
    '''
    def on_modbusCommandRepPort(self):
        '''Request Received'''
        try:
            commandRequest = self.modbusCommandRepPort.recv_pyobj()
            self.commandRequestPending += 1
            self.logger.debug("Request for Modbus device command received")
            #if debugMode:
            #    self.modbusReqRxTime = time.perf_counter()
            #    self.logger.debug("modbusCommandRepPort()[%s]: Request=%s Received at %f" % (str(self.pid),commandRequest,self.modbusReqRxTime))

        except PortError as e:
            self.logger.error("on_modbusCommandRepPort:receive exception = %d" % e.errno)
            if e.errno in (PortError.EAGAIN,PortError.EPROTO):
                self.logger.error("on_modbusCommandRepPort: port error received")

        if self.commandRequestPending == 1:
            self.unpackCommand(commandRequest)
            responseValue = -1  # invalid response
            if self.cModInst.isModbusAvailable() == True:
                self.logger.debug("Sending command to Modbus interface for (slaveAddr=%d,port=%s)" % (self.cSlaveAddr,self.cPortName))
                responseValue = self.sendModbusCommand()
                self.logger.debug("Modbus Response received by device for (slaveAddr=%d,port=%s)" % (self.cSlaveAddr,self.cPortName))
                #if debugMode:
                #    t1 = time.perf_counter()
                #    self.logger.debug("modbusCommandRepPort()[%s]: Send Modbus response=%s back to requester at %f" % (str(self.pid),responseValue,t1))
            else:
                self.logger.info("Modbus is not available - (slaveAddr=%d,port=%s)" % (self.cSlaveAddr,self.cPortName))

            '''Send Results'''
            try:
                self.modbusCommandRepPort.send_pyobj(responseValue)
                self.commandRequestPending -= 1
                self.logger.debug("Response for Modbus device command sent for (slaveAddr=%d,port=%s)" % (self.cSlaveAddr,self.cPortName))
            except PortError as e:
                self.logger.error("on_modbusCommandRepPort:send exception = %d - (slaveAddr=%d,port=%s)" % (e.errno,self.cSlaveAddr,self.cPortName))
                if e.errno in (PortError.EAGAIN,PortError.EPROTO):
                    self.logger.error("on_modbusCommandRepPort: port error received")

        elif self.commandRequestPending > 1:
            # This should not happen if the requesting component has the appropriate error handling for request/reply
            self.logger.info("A command request is pending, no additional request was made")


    def unpackCommand(self,rxCommand):
        self.cSlaveAddr = rxCommand.slaveAddr
        self.cPortName = rxCommand.serialPort
        self.commmandRequested = rxCommand.commandType
        self.registerAddress = rxCommand.registerAddress
        self.numberOfRegs = rxCommand.numberOfRegs
        self.numberOfDecimals = rxCommand.numberOfDecimals
        self.signedValue = rxCommand.signedValue
        self.values = rxCommand.values
        try:
            self.cModInst = self.modbusInstances[(self.cSlaveAddr, self.cPortName)]
        except KeyError as ke:
            self.logger.error("Modbus Configuration not available = (slaveAddr=%d,port=%s)" % (self.cSlaveAddr,self.cPortName))


    def sendModbusCommand(self):
        value = 999  # large invalid value

        #if debugMode:
        #    t0 = time.perf_counter()
        #    self.logger.debug("sendModbusCommand()[%s]: Sending command to Modbus library at %f" % (str(self.pid),t0))
        self.logger.debug("sendModbusCommand(): Sending command to Modbus library")

        try:
            if self.commmandRequested == ModbusCommands.READ_INPUTREG:
                value = self.cModInst.readInputRegValue(self.registerAddress, self.numberOfDecimals, self.signedValue)
                self.logger.debug("ModbusUartDevice: sent command %s, register=%d, numOfDecimals=%d, signed=%s" % (ModbusCommands.READ_INPUTREG.name,self.registerAddress,self.numberOfDecimals,str(self.signedValue)))
            elif self.commmandRequested == ModbusCommands.READ_HOLDINGREG:
                value = self.cModInst.readHoldingRegValue(self.registerAddress, self.numberOfDecimals, self.signedValue)
                self.logger.debug("ModbusUartDevice: sent command %s, register=%d, numOfDecimals=%d, signed=%s" % (ModbusCommands.READ_HOLDINGREG.name,self.registerAddress,self.numberOfDecimals,str(self.signedValue)))
            elif self.commmandRequested == ModbusCommands.READMULTI_INPUTREGS:
                value = self.cModInst.readMultiInputRegValues(self.registerAddress, self.numberOfRegs)
                self.logger.debug("ModbusUartDevice: sent command %s, register=%d, numOfRegs=%d" % (ModbusCommands.READMULTI_INPUTREGS.name,self.registerAddress,self.numberOfRegs))
            elif self.commmandRequested == ModbusCommands.READMULTI_HOLDINGREGS:
                value = self.cModInst.readMultiHoldingRegValues(self.registerAddress, self.numberOfRegs)
                self.logger.debug("ModbusUartDevice: sent command %s, register=%d, numOfRegs=%d" % (ModbusCommands.READMULTI_HOLDINGREGS.name,self.registerAddress,self.numberOfRegs))
            elif self.commmandRequested == ModbusCommands.WRITE_HOLDINGREG:
                self.cModInst.writeHoldingRegister(self.registerAddress, self.values[0], self.numberOfDecimals, self.signedValue)
                self.logger.debug("ModbusUartDevice: sent command %s, register=%d, value=%d, numberOfDecimals=%d, signed=%s" % (ModbusCommands.WRITE_HOLDINGREG.name,self.registerAddress,self.values[0],self.numberOfDecimals,str(self.signedValue)))
            elif self.commmandRequested == ModbusCommands.WRITEMULTI_HOLDINGREGS:
                self.cModInst.writeHoldingRegisters(self.registerAddress, self.values)
                self.logger.debug("ModbusUartDevice: sent command %s, register=%d" % (ModbusCommands.WRITEMULTI_HOLDINGREGS.name,self.registerAddress))
                self.logger.debug("ModbusUartDevice: Values - %s" % str(self.values).strip('[]'))
        except Exception as e:
            self.logger.error("ModbusUartDevice: Exception thrown during Modbus command action - %s" % e)

        #if debugMode:
        #    t1 = time.perf_counter()
        #    self.logger.debug("sendModbusCommand()[%s]: Modbus library command complete at %f, time to interact with Modbus library is %f ms" % (str(self.pid),t1,(t1-t0)*1000))

        return value

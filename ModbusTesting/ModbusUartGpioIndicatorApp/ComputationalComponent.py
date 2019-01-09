#ComputationalComponent.py
from riaps.run.comp import Component
import os
import logging
from collections import namedtuple
import pydevd
import time
import capnp
import CommandFormat_capnp
import ResponseFormat_capnp
import LogData_capnp
import ModbusCommands_capnp

''' Enable debugging to gather timing information on the code execution'''
debugMode = False

RegSet = namedtuple('RegSet', ['idx', 'value'])
InputRegs = namedtuple('InputRegs', ['outputCurrent','outputVolt','voltPhase','time'])

'''For Inverter control'''
HoldingRegs = namedtuple('HoldingRegs',['unused', 'startStopCmd', 'power'])
'''For RIAPS future
1.start/stop, 2.power command, 3. frequency shift from secondary control, 4. voltage magnitude shift from secondary control.
HoldingRegs = namedtuple('HoldingRegs',['startStopCmd', 'powerCmd', 'freqShift', 'voltMagShift']) 
'''

class ComputationalComponent(Component):
    def __init__(self):
        super(ComputationalComponent, self).__init__()	        
        self.pid = os.getpid()
        self.logger.info("(PID %s) - starting ComputationalComponent" % str(self.pid))
        self.inputRegs = InputRegs(RegSet(0,45),RegSet(1,56),RegSet(2,78),RegSet(3,91))
        self.holdingRegs = HoldingRegs(RegSet(0,0),RegSet(1,55),RegSet(2,66))

        if debugMode:
            self.logger.setLevel(logging.DEBUG)
            self.logger.handlers[0].setLevel(logging.DEBUG) # a workaround for hardcoded INFO level of StreamHandler logger
        else:
            self.logger.setLevel(logging.INFO)

        '''Setup Commands'''
        self.defaultNumOfRegs = 1
        self.dummyValue = [0]
        self.dummyInputRegValues = [0,1,2,3]
        self.dummyHoldingRegValues = [0,1,2]
        self.successfulWrite = 1
        self.openReq = False

    def on_clock(self):
        now = self.clock.recv_pyobj()
        self.logger.info("PID(%s) - on_clock(): %s" % (str(self.pid),str(now)))

        '''Setup a Capnp Message'''
        command = CommandFormat_capnp.CommandFormat.new_message()

        '''Request:  Commands to send over Modbus - one command used at a time'''

        '''Read all input registers'''
        #command.commandType = ModbusCommands_capnp.ModbusCommands.readInputRegs
        #command.registerAddress = self.inputRegs.outputCurrent.idx
        #command.numberOfRegs = len(self.inputRegs)

        '''Read a single holding register'''
        #command.commandType = ModbusCommands_capnp.ModbusCommands.readHoldingRegs
        #command.registerAddress = self.holdingRegs.startStopCmd.idx
        #command.numberOfRegs = self.defaultNumOfRegs

        '''Write a single holding register'''
        #self.values = [83]
        #command.commandType = ModbusCommands_capnp.ModbusCommands.writeHoldingReg
        #command.registerAddress = self.holdingRegs.startStopCmd.idx
        #command.numberOfRegs = self.defaultNumOfRegs
        #command.values = self.values

        '''Read all holding registers'''
        command.commandType = ModbusCommands_capnp.ModbusCommands.readHoldingRegs
        command.registerAddress = self.holdingRegs.unused.idx
        command.numberOfRegs = len(self.holdingRegs)


        '''Write multiple holding registers'''
        #self.values = [75,67]
        #command.commandType = ModbusCommands_capnp.ModbusCommands.writeMultiHoldingRegs
        #command.registerAddress = self.holdingRegs.startStopCmd.idx
        #command.numberOfRegs = len(self.values)
        #command.values = self.values

        '''Write/Read multiple holding registers'''
        #self.values = [57,76]
        #command.commandType = ModbusCommands_capnp.ModbusCommands.writeReadHoldingRegs
        #command.registerAddress = self.holdingRegs.startStopCmd.idx
        #command.numberOfRegs = len(self.values)
        #command.values = self.values
        #command.wreadRegAddress = self.holdingRegs.unused.idx
        #command.wreadNumOfRegs = len(self.holdingRegs)

        '''Send Command'''
        cmdBytes = command.to_bytes()

        if not self.openReq:
            #self.cmdSendStartTime = time.perf_counter()
            #self.logger.debug("on_clock()[%s]: Send command to ModbusUartDevice at %f" % (str(self.pid),self.cmdSendStartTime))
            self.logger.debug("on_clock()[%s]: Send command to ModbusUartDevice" % str(self.pid))

            if self.modbusReqPort.send_capnp(cmdBytes):
                #self.logger.debug("Sent Request")
                self.openReq = True
            else:
                self.logger.info("Failed to send")
        else:
            self.logger.info("There is an open request, did not send a new request this clock cycle")

    def on_modbusReqPort(self):
        bytes = self.modbusReqPort.recv_capnp()
        self.logger.info("PID (%s) - on_modbusReqPort():%s" % (str(self.pid),str(bytes)))
        self.openReq = False

        response = ResponseFormat_capnp.ResponseFormat.from_bytes(bytes)
        #pydevd.settrace(host='192.168.1.103',port=5678)

        #if debugMode:
        #self.cmdResultsRxTime = time.perf_counter()
        #self.logger.info("on_modbusReqPort()[%s]: Received Modbus data=%s, time from cmd to data is %f ms" % (str(self.pid),repr(response),(self.cmdResultsRxTime-self.cmdSendStartTime)*1000))
        self.logger.debug("response:%s" % response)

        if response.commandType == ModbusCommands_capnp.ModbusCommands.readInputRegs or response.commandType == ModbusCommands_capnp.ModbusCommands.readHoldingRegs:
            strList = []
            for idx in range (0,response.numberOfRegs):
                strList.append(str(response.values[idx]))
            valuesStr = ' '.join(strList)
            logMsg = "Register starting at " + str(response.registerAddress) + " has values of " + valuesStr
        elif response.commandType == ModbusCommands_capnp.ModbusCommands.writeHoldingReg:
            if response.numberOfRegs == self.successfulWrite:
                logMsg = "Successfully wrote Register " + str(response.registerAddress)
            else:
                logMsg = "Failed to write Register " + str(response.registerAddress)
        elif response.commandType == ModbusCommands_capnp.ModbusCommands.writeMultiHoldingRegs:
            if response.numberOfRegs == self.successfulWrite:
                logMsg = "Successfully wrote Registers starting at " + str(response.registerAddress)
            else:
                logMsg = "Failed to write Registers starting at " + str(response.registerAddress)
        elif response.commandType == ModbusCommands_capnp.ModbusCommands.writeReadHoldingRegs:
            strList = []
            for idx in range (0,response.numberOfRegs):
                strList.append(str(response.values[idx]))
            valuesStr = ' '.join(strList)
            logMsg = "Wrote Registers, then read " + response.numberOfRegs + " registers starting at Register " + str(response.registerAddress) + " which had values of " + valuesStr

        self.logger.debug("logMsg: %s" % logMsg)
        self.tx_modbusData.send_pyobj(logMsg)  # Send log data

        self.logger.debug("End on_modbusReqPort()")

    def __destroy__(self):
        self.logger.info("(PID %s) - stopping ComputationalComponent" % str(self.pid))
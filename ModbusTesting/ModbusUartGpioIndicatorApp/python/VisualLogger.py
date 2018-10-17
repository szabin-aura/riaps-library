#VisualLogger.py
from riaps.run.comp import Component
import os
import logging
import time

class VisualLogger(Component):
    def __init__(self):
        super(VisualLogger, self).__init__()	        
        self.pid = os.getpid()
        self.logger.info("(PID %s) - starting VisualLogger",str(self.pid))
        self.gpioOn = False        

    def on_offTimer(self):
        now = self.offTimer.recv_pyobj()
        self.logger.info('PID(%s) - on_offTimer(): %s',str(self.pid),str(now))
                                
        if self.gpioOn:  # If answer comes back and LED is on, turn off LED
            gpioMsg = ('write', 0)
            self.gpioReqPort.send_pyobj(gpioMsg)
            self.gpioOn = False
            self.logger.info("Turn off LED")

    def on_rx_modbusData(self):
        msg = self.rx_modbusData.recv_pyobj()
        self.logger.info("PID (%s) - on_rx_modbusData():%s",str(self.pid),str(msg))
        # Send request to turn on an LED to signal command received (if already on, skip it)
        if not self.gpioOn:
            gpioMsg = ('write', 1)
            self.gpioReqPort.send_pyobj(gpioMsg)
            self.gpioOn = True
            self.logger.info("Modbus Data returned, turn on LED")

    def on_gpioReqPort(self):
        req = self.gpioReqPort.recv_pyobj()
        self.logger.info("PID (%s) - on_gpioReqPort():%s",str(self.pid),str(req))
    
    def __destroy__(self):			
        self.logger.info("(PID %s) - stopping VisualLogger",str(self.pid))   	        	        

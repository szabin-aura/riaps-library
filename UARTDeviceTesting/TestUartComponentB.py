'''
Created on Aug 2, 2017

@author: Tim Krentz
'''
# riaps:keep_import:begin
from riaps.run.comp import Component
import os
import threading
# riaps:keep_import:end

class TestUartComponentB(Component):
# riaps:keep_constr:begin
    def __init__(self):
        super().__init__()
        self.pid = os.getpid()
        self.setValue = 0  # default off
        self.logger.info("TestUartComponent: %s - starting" % str(self.pid))
        self.protectReq = threading.Event()
        self.protectReq.set()
        self.protectRead = threading.Event()
        self.protectRead.set()
        self.closeState = 0
# riaps:keep_constr:end

# riaps:keep_activity:begin
    def on_activity(self):
        msg = self.activity.recv_pyobj()

        if self.protectReq.isSet():
            self.protectReq.clear()
            msg = ('read',10)
            self.uartReqPort.send_pyobj(msg)
            self.logger.info("on_activity()[%s]: requested to read: %s" %
                (str(self.pid),repr(msg)))
# riaps:keep_activity:end

# riaps:keep_uartReqPort:begin
    def on_uartReqPort(self):
        msg = self.uartReqPort.recv_pyobj()
        self.logger.info("on_uartReqPort()[%s]: got reply : %s " %
                        (str(self.pid),repr(msg)))
        self.protectReq.set()
# riaps:keep_uartReqPort:end

# riaps:keep_uartReadSub:begin
    def on_uartReadSub(self):
        msg = self.uartReadSub.recv_pyobj()
        self.logger.info("on_uartReadSub()[%s]: got bytes : %s " %
                        (str(self.pid),repr(msg)))
# riaps:keep_uartReadSub:end

# riaps:keep_impl:begin

# riaps:keep_impl:end

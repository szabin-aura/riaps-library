'''
Created on July 31, 2017

@author: Tim Krentz
'''
# riaps:keep_import:begin
from riaps.run.comp import Component
import os
# riaps:keep_import:end

class TestUartComponentA(Component):
# riaps:keep_constr:begin
    def __init__(self):
        super().__init__()
        self.pid = os.getpid()
        self.setValue = 0  # default off
        self.logger.info("TestUartComponent: %s - starting" % str(self.pid))
        self.count = 0
# riaps:keep_constr:end

# riaps:keep_activity:begin
    def on_activity(self):
        msg = self.activity.recv_pyobj()

        msg = ('write',str.encode(str(self.count)))
        # msg = ('write',str.encode('RIAPS'))
        self.uartReqPort.send_pyobj(msg)
        self.count = self.count + 1

        self.logger.info("on_activity()[%s]: requested to write: %s" %
            (str(self.pid),repr(msg)))
# riaps:keep_activity:end

# riaps:keep_uartReqPort:begin
    def on_uartReqPort(self):
        msg = self.uartReqPort.recv_pyobj()
        self.logger.info("on_uartReqPort()[%s]: got reply : %s " %
                        (str(self.pid),repr(msg)))
# riaps:keep_uartReqPort:end

# riaps:keep_impl:begin

# riaps:keep_impl:end

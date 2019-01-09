'''
Created on July 27, 2017

@author: Tim Krentz
'''
# riaps:keep_import:begin
from riaps.run.comp import Component
import os
import threading
# riaps:keep_import:end

class ToggleGpioComponent(Component):
# riaps:keep_constr:begin
    def __init__(self):
        super().__init__()
        self.pid = os.getpid()
        self.setValue = 0  # default off
        self.logger.info("ToggleGpioComponent: %s - starting",str(self.pid))
        self.waiting = 0
        self.protectReq = threading.Semaphore()
        self.deviceActive = True
        self.uuid = False
# riaps:keep_constr:end

# riaps:keep_toggle:begin
    def on_toggle(self):
        msg = self.toggle.recv_pyobj()
        if self.deviceActive == False:
            return
        self.toggle.setPeriod(1.0)
        if self.protectReq.acquire(blocking = False):
            if self.setValue == 0:
                self.setValue = 1
            else:
                self.setValue = 0
            msg = ('write',self.setValue)
            self.gpioReqPort.send_pyobj(msg)
            self.logger.info("on_toggle()[%s]: Send write request, setValue=%d",
                            str(self.pid), self.setValue)
# riaps:keep_toggle:end

# riaps:keep_readValue:begin
    def on_readValue(self):
        msg = self.readValue.recv_pyobj()
        if self.deviceActive == False:
            return
        if self.protectReq.acquire(blocking = False):
            self.logger.info("on_readValue()[%s]: %s",str(self.pid),repr(msg))
            msg = ('read',0)
            self.gpioReqPort.send_pyobj(msg)
# riaps:keep_readValue:end

# riaps:keep_gpioReqPort:begin
    def on_gpioReqPort(self):
        msg = self.gpioReqPort.recv_pyobj()
        self.logger.info("on_gpioReqPort()[%s]: got reply : %s ",
                        str(self.pid),repr(msg))
        self.protectReq.release()
# riaps:keep_gpioReqPort:end

# riaps:keep_impl:begin
    def handleActivate(self):
        self.uuid = self.getUUID()
        self.logger.info("My UUID is: %s" % self.uuid)

    def handlePeerStateChange(self,state,uuid):
        self.logger.info("peer %s is %s" % (uuid,state))
        if uuid is not self.uuid and 'on' in str(state):
            self.deviceActive = True
# riaps:keep_impl:end

# riaps:keep_import:begin
from riaps.run.comp import Component
import logging
import uuid
import time
import os
# riaps:keep_import:end

class Concentrator(Component):
# riaps:keep_constr:begin
    def __init__(self):
        super().__init__()
        self.uuid = uuid.uuid4().int
        self.pid = os.getpid()
        self.logger.info("%s - starting" % str(self.pid))
# riaps:keep_constr:end

# riaps:keep_pmuDataReady:begin
    def on_pmuDataReady(self):
        msg = self.pmuDataReady.recv_pyobj() # Receive DataFrame
        self.logger.info("on_pmuDataReady()[%s]: %s" % (str(self.pid),repr(msg)))
# riaps:keep_pmuDataReady:end

# riaps:keep_display:begin
    def on_display(self):
        msg = self.display.recv_pyobj()
        self.logger.info("on_display()[%s]" % str(self.pid))
# riaps:keep_display:end

# riaps:keep_impl:begin

# riaps:keep_impl:end

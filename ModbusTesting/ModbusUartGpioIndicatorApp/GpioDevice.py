'''
Created on Apr 4, 2017
@author: Mary Metelko
Edited on Jul 27, 2017
@author: Tim Krentz
'''

'''
The GPIO device component utilizes Adafruit_BBIO for control of the GPIO pins
Need to install this on the BBBs
    $ sudo pip3 install Adafruit_BBIO
'''
from riaps.run.comp import Component
import os
import logging
import threading
import Adafruit_BBIO.GPIO as GPIO
#import pydevd
import zmq

''' Internal thread for GPIO hardware interaction '''
class GpioDeviceThread(threading.Thread):
    def __init__(self, component, trigger):
        threading.Thread.__init__(self)
        self.logger = logger
        self.terminated = threading.Event()
        self.terminated.clear()
        self.active = threading.Event()
        self.active.clear()
        self.waiting = threading.Event()
        self.component = component
        self.trigger = trigger
        self.gpioAvailable = False
        self.pid = os.getpid()

        # Convert input configurations into enums that represent the requests
        if self.component.direction == 'IN':
            self.direction = GPIO.IN
        elif self.component.direction == 'OUT':
            self.direction = GPIO.OUT
        else:
            self.direction = GPIO.ALT0

        if self.component.pull_up_down == "PUD_UP":
            self.pull_up_down = GPIO.PUD_UP
        elif self.component.pull_up_down == "PUD_DOWN":
            self.pull_up_down = GPIO.PUD_DOWN
        else:
            self.pull_up_down = GPIO.PUD_OFF

        if self.component.trigger_edge == "RISING":
            self.trigger_edge = GPIO.RISING
        elif self.component.trigger_edge == "FALLING":
            self.trigger_edge = GPIO.FALLING
        else:
            self.trigger_edge = GPIO.BOTH

        self.logger.info("GpioDeviceThread [%s]: initialized",self.pid)

    ''' Main loop interacting with hardware pin'''
    def run(self):
        self.logger.info('GpioDeviceThread starting')
        # Ask parent port to make a plug for this end
        self.plug = self.trigger.setupPlug(self)
        self.poller = zmq.Poller()
        self.poller.register(self.plug, zmq.POLLIN)
        if self.terminated.is_set(): return
        self.enableGpio()  # setup the requested GPIO

        while True:
            self.active.wait(None)
            if self.terminated.is_set():
                self.disableGpio()
                break
            if self.active.is_set():
                socks = dict(self.poller.poll())
                if len(socks) == 0:
                    self.logger.info('GpioDeviceThread timeout')
                if self.terminated.is_set(): break
                if self.plug in socks and socks[self.plug] == zmq.POLLIN:
                    message = self.plug.recv_pyobj()

                    if 'write' in message:
                        GPIO.output(self.component.bbb_pin_name, message[1])
                        self.logger.info("GpioDeviceThread - value written to GPIO %s: %s",
                                                    self.component.bbb_pin_name, str(message[1]))
                        self.plug.send_pyobj(('write',message[1]))

                    elif 'read' in message:
                        val = GPIO.input(self.component.bbb_pin_name)
                        self.logger.info("GpioDeviceThread - value read from GPIO %s: %s",
                                                    self.component.bbb_pin_name, val)
                        self.plug.send_pyobj(('read',val))

        self.logger.info('GpioDeviceThread ended')


    def isGpioAvailable(self):
        return self.gpioAvailable

    ''' Setup GPIO pin indicated '''
    def enableGpio(self):
        self.component.logger.info("GpioDeviceThread setting up GPIO=%s: direction=%s resistor=%s trigger=%s ivalue=%d delay=%d [%d]",
                                    self.component.bbb_pin_name, self.component.direction, self.component.pull_up_down,
                                    self.component.trigger_edge, self.component.initial_value, self.component.setup_delay, self.pid)
        GPIO.setup(self.component.bbb_pin_name, self.direction, self.pull_up_down,
                    self.component.initial_value, self.component.setup_delay)
        self.gpioAvailable = True
        self.logger.info("GpioDeviceThread GPIO=%s setup and available for use", self.component.bbb_pin_name)

    def disableGpio(self):
        GPIO.cleanup(self.component.bbb_pin_name)
        self.gpioAvailable = False
        self.logger.info("GpioDeviceThread - disabled GPIO: %s",self.component.bbb_pin_name)

    def activate(self):
        self.active.set()
        self.logger.info('GpioDeviceThread activated')

    def deactivate(self):
        self.active.clear()
        self.logger.info('GpioDeviceThread deactivated')

    def terminate(self):
        self.terminated.set()
        self.logger.info('GpioDeviceThread terminating')


'''
GPIO Device Component Options:
  Pin name:      connector and pin used on the BBB - P8_pin# or P9_pin# (required input)
  direction:     pin direction - IN or OUT (required input)
  pull_up_down:  BBB pin resistor configuration - PUD_OFF (default), PUD_UP or PUD_DOWN
  trigger_edge:  which edge to trigger on - RISING, FALLING, BOTH
  initial_value: if GPIO is OUT direction, then this initial value will be set - default = 0
  setup_delay:   time in milliseconds to wait after exporting GPIO pin to give udev some time to set file permissions - default = 60 ms, should not use over 1000 ms

  Note:  edge triggering will not be implemented in the initial release of this device component (MM)
'''
class GpioDevice(Component):
    def __init__(self,bbb_pin_name='P8_11',direction='OUT',pull_up_down='PUD_OFF',trigger_edge='RISING',initial_value=0,setup_delay=60):
        super().__init__()
        #super(GpioDevice, self).__init__()
        self.pid = os.getpid()
        self.logger.info("(PID %s) - starting GpioDevice",str(self.pid))
        self.logger.setLevel(logging.DEBUG)
        self.pid = os.getpid()
        self.bbb_pin_name = bbb_pin_name
        self.direction = direction
        self.pull_up_down = pull_up_down
        self.trigger_edge = trigger_edge
        self.initial_value = initial_value
        self.setup_delay = setup_delay
#        pydevd.settrace(host='192.168.1.102',port=5678)
        self.logger.info("@%s: %s %s %s ivalue=%d delay=%d [%d]", self.bbb_pin_name, self.direction, self.pull_up_down, self.trigger_edge, self.initial_value, self.setup_delay, self.pid)
        self.gpioDeviceThread = None                    # Cannot manipulate GPIOs in constructor or start threads

    ''' Clock used to start internal thread to talk with the device.
        Fires once and then is halted '''
    def on_clock(self):
        now = self.clock.recv_pyobj()
        self.logger.info('PID(%s) - on_clock(): %s',str(self.pid),str(now))
        if self.gpioDeviceThread == None:
            self.logger.info("No thread, init the thread")
            self.gpioDeviceThread = GpioDeviceThread(self,self.trigger)
            self.gpioDeviceThread.start()
            self.trigger.activate()
            self.logger.info("GpioDeviceThread activated")

        self.clock.halt()

    ''' Internal thread response available, message sent back to requesting component using reply port '''
    def on_trigger(self):
        msg = self.trigger.recv_pyobj()
        self.logger.info("Received GpioDeviceThread response - %s",msg)
        self.gpioRepPort.send_pyobj(msg)
        self.logger.info("Sent response back from GPIO request")

    ''' Received a request from a component to read or write to the device.
        Calls into internal thread with request. '''
    def on_gpioRepPort(self):
        msg = self.gpioRepPort.recv_pyobj()
        self.logger.info("PID (%s) - on_gpioRepPort():%s",str(self.pid),str(msg))
        if self.gpioDeviceThread == None:
            self.logger.info("on_gpioRepPort()[%s]: gpioDeviceThread not available yet",str(self.pid))
            msg = ('ERROR',-1)
            self.gpioRepPort.send_pyobj(msg)
        else:
            if self.gpioDeviceThread.isGpioAvailable() == True:
                msgType, msgVal = msg
                if msgType == 'read':
                    self.trigger.send_pyobj('read')
                    self.logger.info("on_gpioRepPort()[%s]: %s",str(self.pid),repr(msg))

                elif msgType == 'write':
                    if msgVal == 1 or msgVal == 0:
                        self.trigger.send_pyobj((msgType,msgVal))
                        self.logger.info("on_gpioRepPort()[%s]: %s",str(self.pid),repr(msg))
                    else:
                        self.logger.info("on_gpioRepPort()[%s]: invalid write value",str(self.pid))
                        msg = ('ERROR',-1)
                        self.gpioRepPort.send_pyobj(msg)
                else:
                    self.logger.info("on_gpioRepPort()[%s]: unsupported request=%s",str(self.pid),repr(msg))
                    msg = ('ERROR',-1)
                    self.gpioRepPort.send_pyobj(msg)
            else:
                self.logger.info("on_gpioRepPort()[%s]: GPIO not available yet",str(self.pid))
                msg = ('ERROR',-1)
                self.gpioRepPort.send_pyobj(msg)

    def __destroy__(self):
        self.logger.info("(PID %s) - stopping GpioDevice",str(self.pid))
        if self.gpioDeviceThread != None:
            self.gpioDeviceThread.deactivate()
            self.gpioDeviceThread.terminate()
            self.gpioDeviceThread.join()
            self.logger.info("__destroy__ed")

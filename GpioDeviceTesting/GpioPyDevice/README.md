# GPIO Python Device Component 

This is a reusable GPIO Device component.  The component is able to read or write to GPIO pin that have been setup 
when configuring the GPIO Device component.  Application developers can create a project for the component and then 
link that device project to their power applications.  

# Building the Shared Library

The best way is to use Eclipse and setup a RIAPS Apps project using the model file (.riaps).  These device files can be copied over the generated files.   This project can then be linked to another RIAPS Apps project that uses this device.  

Resulting shared library file (for packaging): GpioDevice.py
  
# GPIO Request Commands

This device is setup to be the reply port what waits for request from an application component.  A request command consists of a command type and value.  

## Command Types
    * read
    * write

## Value
    Either 0 or 1 for the brightness value of the LED

# Setting up an Application to Use GPIO Device Component

In the application model (.riaps file), create a GPIO device component with the configuration desired.  

```
    device GpioDeviceComponent(bbb_pin_name='P8_11', direction='OUT', pull_up_down='PUD_OFF', setup_delay=60) 
```

where,
- Pin name: connector and pin used on the BBB - P8_pin# or P9_pin# (***required input***), there are also USR0-USR3 available on the BBB (USR0 is in use for network activity indication)
- direction: pin direction - IN or OUT (***required input***)
- pull_up_down: BBB pin resistor configuration - ***PUD_OFF*** (***default***), PUD_UP or PUD_DOWN
- setup_delay: time in milliseconds to wait after exporting GPIO pin to give udev some time to set file permissions - ***default = 60 ms***, should not use over 1000 ms
 
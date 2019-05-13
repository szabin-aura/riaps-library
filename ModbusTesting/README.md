Welcome to the RIAPS Modbus Device Testing Page

The current testing applications are:
* ModbusUartReqRepTesting
    - Application makes a command request to the device application and will block until a response is returned.
    - Application and Device Components are both Python code
    - Works for 20 ms update rates or greater using a 115.2 kbps UART setup
    - UART Physical Device only (single connection)
* MultiModbusUartReqRepTesting
    - Allows configuration of multiple modbus instances controlled by a Modbus device component
    - This is an extension of the ModbusUartReqRepTesting example
    - Application and Device Components are both Python code
    - Works for 20 ms update rates or greater using a 115.2 kbps UART setup
    - Two UART Physical Connections controlled by a Modbus Manager Device
* ModbusUartAllC++Components
    - This is the same as ModbusUartReqRepTesting, except the device and application components are developing using C++ to get the fastest combination of execution speed
    - The Logger component is intentionally missing since it did not add any value to this example at the moment
    - Works for 10-11 ms update rates or greater using a 115.2 kbps UART setup
    - UART Physical Device only (single connection)
* ModbusUartGpioIndicatorApp
    - This is the same as ModbusUartReqRepTesting, except that the Device Component is written as C++ component and the application components are in Python.  This also includes a GPIO device to blink a user LED when a Modbus command response is available.
    - Works for 12-13 ms update rates or greater using a 115.2 kbps UART setup
    - UART (single connection) and GPIO Physical Devices


The Modbus slave device is tested on a DSP.  The code for this is located in DSP_Code.

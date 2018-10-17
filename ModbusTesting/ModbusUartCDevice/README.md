# RIAPS Modbus UART Device Component Library

This library code will create one shared library:  
* libmodbusuart.so - for use as the C++ device component

## Developer
* Mary Metelko, Vanderbilt University

## Required Software Installation (for compiling and use)

* **libmodbus**

    This library should be installed on the machine where the library will be built and on the BBBs where the library will be call from the RIAPS Modbus UART shared library.

    - To build on the BBBs:

    ```
    git clone https://github.com/cmjones01/libmodbus.git
    sudo apt-get install autoconf libtool pkg-config
    cd libmodbus
    ./autogen.sh
    ./configure
    make

        libmodbus 3.1.2
        ===============
        prefix:                 /usr/local
        sysconfdir:             ${prefix}/etc
        libdir:                 ${exec_prefix}/lib
        includedir:             ${prefix}/include

    sudo make install

    `----------------------------------------------------------------
    Libraries have been installed in:
        /usr/local/lib
    `----------------------------------------------------------------
    ```

    - To cross-compile on the development machine, libmodbus needs to be cross compiled on the VM.  The above method applies,
    but use the following configuration statement.

    ```
    ac_cv_func_malloc_0_nonnull=yes ./configure --host=arm-linux-gnueabihf --prefix=/opt/riaps/armhf
    ```

    which setups the following:

    ```
    libmodbus 3.1.2
        ===============
        prefix:                 /opt/riaps/armhf
        sysconfdir:             ${prefix}/etc
        libdir:                 ${exec_prefix}/lib
        includedir:             ${prefix}/include

    ```


> Note: cmjones01 fork was used to allow option for asynchronous operation in the future
- See https://martin-jones.com/2015/12/16/modifying-libmodbus-for-asynchronous-operation/

## Hardware Setup

Refer to https://github.com/RIAPS/riaps-library/tree/master/UARTDeviceTesting for instructions on setting up the UART port (Setup UART2 on BBBs).

## Building the Shared Library

The best way is to use Eclipse and setup a RIAPS Apps project using the model file (.riaps).  These device files can be copied over the generated files.  
This project can then be linked to another RIAPS Apps project that uses this device.  

Resulting shared library: libmodbusuart.so

## Device Interface Available to Application Components

* Cap'n Proto Messages:  defined in 'messages/riapsModbusUART/modbusuart.capnp'
    - ***CommandFormat***
        - **commandType** (UInt16): ModbusCommands enum value for desired command
        - **registerAddress** (UInt16): address of the remote device
        >  Note: for writeReadHoldingRegs, this is the write holding address 
        - **numberOfRegs** (UInt16): number of bits or registers
        - **values** (List(UInt16)): values to write
        - **wreadRegAddress** (UInt16): used for writeReadHoldingRegs to specify the read holding address
        - **wreadNumOfRegs** (UInt16): used for writeReadHoldingRegs to specify the number of registers to read
    - ***ResponseFormat***
        - **commandType** (UInt16): ModbusCommands enum value for command used
        - **registerAddress** (UInt16): address of the remote device
        - **numberOfRegs** (UInt16): number of registers read or written
        - **values** (List(UInt16)): values read (not used in writes)
    - ***ModbusCommands*** is an enum describing the types of Modbus Commands that are available to the user in the capnp file, the values are defined as follows:
        - **readCoilBits / writeCoilBit / writeCoilBits**: Used to read and write the Coil bits.  The number of coil bits is set when configuring the Modbus in the RIAPS model file by using "numcoilbits=xxxx", where xxxx is a value between 1 to 2000.
        > Note:  This functionality has not yet been tested        
        - **readInputBits**: Used to read the discrete input bits.  The number of discrete input bits is set when configuring the Modbus in the RIAPS model file by using  "numdiscretebits=xxxx", where xxxx is a value between 1 to 2000.
        > Note:  This functionality has not yet been tested
        - **readInputRegs**: Used to read the input registers.  The number of input registers is set when configuring the Modbus in the RIAPS model file by using "numinputreg=xxx", where xxx is a value between 1 to 125.
        - **readHoldingRegs / writeHoldingReg / writeMultiHoldingRegs / writeReadHoldingRegs**: Used to read/write the holding registers.  The number of holding registers is set when configuring the Modbus in the RIAPS model file by using "numholdreg=xxx", where xxx is a value between 1 to 125.


* Usage of Cap'n Proto Message Enums
    - The Cap'n Proto syntax requires that the enum name be camelback with a lowercase first letter.  The translation into the C++ sets each of these names to all uppercase words with '_' between camelback parts of the Cap'n Proto message defined name.
      - C++ Components example

      ```
      ModbusCommands::READ_HOLDING_REGS
      ```

      - Python Components

      ```
      import capnp
      import ModbusCommands_capnp

      commandType = ModbusCommands_capnp.ModbusCommands.readHoldingRegs
      ```

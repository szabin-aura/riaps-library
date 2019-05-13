# RIAPS Application Description

This application example shows how to create components using C++ and interface with a C++
device component (libmodbusuart.so).  

To begin, the user defines the model (.riaps) and deployment files (.depl). This will generate
a shell for the user edited C++ component files.

The C++ Components in this project are:
* ComputationalComponent (.h/.cc) - this is the application code that send and reads data to the Modbus Uart device component.  The application developer is expected to place the computational logic in this code.
  - ComputationalComponentBase (.h/.cc) - these files are auto generated based on the model file input.
    The application developer should not edit these files.  Any changes to the model file will
    override any edits to these files.

* ModbusUART (.h/.cc) - this is the modbus device component.  This device receives requests for Modbus communication,
performs the communication, and then provides the response back to the requestor.

* Logger (.h/cc) - this component subscribes to the results of the Modbus communication which is published by the
ComputationalComponent.

## Developer
* Mary Metelko, Vanderbilt University

## BBB Software Setup Requirements

### Install libmodbus

This library should be installed on the machine where the library will be built and on the BBBs where the
library will be call from the RIAPS Modbus UART shared library.

- To build on the BBBs:

    ```
    git clone https://github.com/cmjones01/libmodbus.git
    sudo apt-get install autoconf libtool pkg-config
    cd libmodbus
    ./autogen.sh
    ./configure --libdir=/usr/lib/arm-linux-gnueabihf --includedir=/usr/include/arm-linux-gnueabihf
    make

        libmodbus 3.1.2
        ===============
        prefix:                 /usr/local
        sysconfdir:             ${prefix}/etc
        libdir:                 /usr/lib/arm-linux-gnueabihf
        includedir:             /usr/include/arm-linux-gnueabihf

    sudo make install

    `----------------------------------------------------------------
    Libraries have been installed in:
        /usr/lib/arm-linux-gnueabihf
    `----------------------------------------------------------------
    ```

- To cross-compile on the development machine, libmodbus needs to be cross compiled on the VM.  
    The above method applies, but use the following configuration statement.

    ```
    ac_cv_func_malloc_0_nonnull=yes ./configure --host=arm-linux-gnueabihf --prefix=/usr/arm-linux-gnueabihf
    ```

> Note: cmjones01 fork was used to allow option for asynchronous operation in the future
- See https://martin-jones.com/2015/12/16/modifying-libmodbus-for-asynchronous-operation/

### Install InfluxDB for Logger

   Install on BBB or VM (where logging is happening)

```
curl -sL https://repos.influxdata.com/influxdb.key | sudo apt-key add -      
source /etc/lsb-release     
echo "deb https://repos.influxdata.com/${DISTRIB_ID,,} ${DISTRIB_CODENAME} stable" | sudo tee /etc/apt/sources.list.d/influxdb.list     
sudo apt-get update -y && sudo apt-get install influxdb -y      
sudo systemctl start influxdb
```

   On BBB, configure python library

```
sudo pip3 install influxdb
```

### Eclipse Project Properties Additions

* Include **/usr/arm-linux-gnueabihf/include/modbus** in the project properties include paths (C/C++ General --> Path and Symbols)
* In the CMakeLists.txt file, add "modbus" in the target link libraries for the libmodbusuart.so.

```
target_link_libraries(modbusuart PRIVATE czmq riaps dl capnp kj modbus)
```

## UART Configuration
* port = 'UART2'
* baud rate = 115200
* 8 bit, parity = None, 1 stopbit (by default)

## Modbus Configuration
* Slave Address:  10 or (0x0A)

* InputRegs (read only)
  - [0]=outputCurrent,
  - [1]=outputVolt,
  - [2]=voltPhase,
  - [3]=time

* For Inverter control:  HoldingRegs (read/write)
  - [0]=unused
  - [1]=startStopCmd
  - [2]=power

## HW Configuration

### Enable UART on the BBB
* Tools used to test UART2:  
  - Terminal tool on the host
  - USB to 3.3 V TTL Cable (TTL-232R-3V3 by FTDI Chip)
    - How to connect with BBB (P9 connector)
      - White (RX) to BBB TX (pin 21),
      - Green (TX) to BBB RX (pin 22),
      - GND on BBB pins 1, 2, 45, 46
    - Cable information from https://www.adafruit.com/product/954?gclid=EAIaIQobChMIlIWZzJvX1QIVlyOBCh3obgJjEAQYASABEgImJfD_BwE

* To turn on the UART2, on the beaglebone, modify /boot/uEnv.txt by uncommenting the following line and adding BB-UART2
(which points to an overlay in /lib/firmware)

```
   ###Master Enable
   enable_uboot_overlays=1
   ...
   ###Additional custom capes
   uboot_overlay_addr4=/lib/firmware/BB-UART2-00A0.dtbo
```

* Reboot the beaglebone to see the UART2 enabled. UART2 device is setup as ttyO2 (where the fourth letter
is the letter 'O', not zero) that references ttyS2 (a special character files)

* To verify that UART2 is enabled, do the following

```
   ls -l /dev/ttyO*

   lrwxrwxrwx 1 root root 5 Mar  6 22:54 /dev/ttyO0 -> ttyS0
   lrwxrwxrwx 1 root root 5 Mar  6 22:54 /dev/ttyO2 -> ttyS2
```

If the enabled UART does not show up in the verification step, check that the eMMC bootloader is not blocking the uboot overlay.  

```
   sudo /opt/scripts/tools/version.sh | grep bootloader
```

You might find that the eMMC bootloader is older than 2018 (as shown below).  

```
bootloader:[microSD-(push-button)]:[/dev/mmcblk0]:[U-Boot 2018.09-00002-g0b54a51eee]:[location: dd MBR]
bootloader:[eMMC-(default)]:[/dev/mmcblk1]:[U-Boot 2016.01-00001-g4eb802e]:[location: dd MBR]
```

Then go to [Flasher - eMMC: All BeagleBone Varients with eMMC section of the Ubuntu BeagleBone website](https://elinux.org/BeagleBoardUbuntu) and follow the instructions to update the eMMC bootloader.

### Connecting the BBB to the DSP Modbus UART Connection
* DSP:  C2000(TM) Microcontrollers (TMS320F28377S) LaunchPad Development Kit (LAUNCHXL-F28377S)
* Can buy from Newark (http://www.newark.com/texas-instruments/launchxl-f28377s/dev-board-tms320f28377s-c2000/dp/49Y4795)
  - powered by USB connection
* Code Composer Studio (CCS) by TI used to download software provide by NCSU (source code in library)
* Use CCS to start DSP application

### BBB UART to DSP Modbus Connection
* BBB TX (P9, pin 21) --> DSP RX (J4, pin 37, SCITXDB, GPIO15)
* BBB RX (P9, pin 22) --> DSP TX (J4, pin 38, SCIRXDB, GPIO14)

## Tools used for Debugging Modbus Interface  
* BBB is master, so to debug this interface a slave simulator was used: MODBUS RTU RS-232 PLC
  - Simulator found at www.plcsimulator.org
* DSP is slave, so to debug this interface a master simulator was used: QModMaster 0.4.7
  - libmodbus 3.1.4 found at https://sourceforge.net/projects/qmodmaster
* Modbus Message Parser
  - http://modbus.rapidscada.net/

## Device Interface Available to Application Components

* Cap'n Proto Messages:  defined in 'messages/riapsModbusUART/modbusuart.capnp'
    - ***CommandFormat***
        - **commandType** (UInt16): ModbusCommands enum value for desired command
        - **registerAddress** (UInt16): address of the remote device
        >  Note: for writeReadHoldingRegs, this is the holding address to write
        - **numberOfRegs** (UInt16): number of bits or registers
        - **values** (List(UInt16)): values to write
        - **wreadRegAddress** (UInt16): used for writeReadHoldingRegs to specify the holding address to read
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
      riapsmodbuscreqrepuart::messages::ModbusCommands::READ_HOLDING_REGS
      ```

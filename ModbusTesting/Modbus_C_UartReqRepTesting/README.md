# RIAPS Application Description

> ***Note:  THIS PROJECT NEEDS UPDATING TO THE LATEST METHODS USING PYBIND***

This application example shows how to create components using Python and interface with a C++
shared library device (libmodbusuart.so that is available in the ModbusUartCDevice directory).  

The Python Components in this project are:
* ComputationalComponent.py - this is the application code that send and reads data to the Modbus Uart device component.
* Logger.py - this is the application code that publishes a log data created by the ComputationalComponent.  
The commented out code show an example of interfacing with InfluxDB to gather graphing data.

The Device Component is a C++ Component:
Pybind is utilized to allow the python components to interface with the C++ device through Capnp messages.

## Developer
- Mary Metelko, Vanderbilt University

## BBB Software Setup Requirements

### Install libmodbus

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
-
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

## Debugging Hints
* If serial port is busy, make sure to stop the gpsd service using the following command
```
sudo systemctl stop gpsd
```

## Deployment Package
Once the application is ready to deploy, the following files are expected to be available.

***User Edited Files:***
* modbus.riaps
* modbus.depl
* ComputationalComponent.py
* Logger.py

***RIAPS Library File:***
* libmodbusuart.so (the library device component)

***Development Tool Generated Files:***
* ModbusCommands.capnp
* ResponseFormat.capnp
* CommandFormat.capnp
* LogData.capnp
* RIAPSModbusCReqRepUART.json

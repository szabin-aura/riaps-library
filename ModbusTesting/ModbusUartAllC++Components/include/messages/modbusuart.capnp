@0x882e1c0f1af44d05;
using Cxx = import "/capnp/c++.capnp";
$Cxx.namespace("modbusuart::messages");

enum ModbusCommands {
    noCmd @0;
    readCoilBits @1;
    readInputBits @2;
    readInputRegs @3;
    readHoldingRegs @4;
    writeCoilBit @5;
    writeHoldingReg @6;
    writeCoilBits @7;
    writeMultiHoldingRegs @8;
    writeReadHoldingRegs @9;
}

struct CommandFormat
{
    commandType @0: ModbusCommands;
    registerAddress @1: UInt16;
    numberOfRegs @2: UInt16;
    values @3: List(UInt16);
    wreadRegAddress @4: UInt16;
    wreadNumOfRegs @5: UInt16;
}

struct ResponseFormat
{
    commandType @0: ModbusCommands;
    registerAddress @1: UInt16;
    numberOfRegs @2: UInt16;
    values @3: List(UInt16);
}

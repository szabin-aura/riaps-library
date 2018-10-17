@0xec458ab45ebe855b;
using Cxx = import "/capnp/c++.capnp";
$Cxx.namespace("logdata::messages");

struct LogData
{
    msg @0: Text;
}

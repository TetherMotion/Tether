/**
 * @file DebugFlags.cpp
 * @brief Backing storage for EtherCAT debug flag accessor functions.
 */

#include "tether/ethercat/DebugFlags.hpp"
#include "tether/ethercat/Slave.hpp"

namespace EtherCAT {
namespace debug {

bool& rxPDO()       { static bool value = false; return value; }
bool& txPDO()       { static bool value = false; return value; }
bool& stateMachine() { static bool value = false; return value; }
bool& txPackets()   { static bool value = false; return value; }
bool& rxPackets()   { static bool value = false; return value; }
bool& fmmu()        { static bool value = false; return value; }
bool& siiEeprom()   { static bool value = false; return value; }
bool& coeReads()    { static bool value = false; return value; }
bool& coeWrites()   { static bool value = false; return value; }
bool& coeRxPackets(){ static bool value = false; return value; }
bool& coeTxPackets(){ static bool value = false; return value; }

const std::vector<DebugFlagInfo>& allDebugFlags() {
    static const std::vector<DebugFlagInfo> kFlags = {
        {"sii-derivation",
         "Dump SII mailbox derivation after slave discovery",
         nullptr},
        {"mailbox-configuration",
         "Dump mailbox hardware configuration after slave discovery",
         nullptr},
        {"al-state",
         "Log detailed EtherCAT state-machine transitions",
         &enableStateMachineDebug},
        {"tx-ethercat-packets",
         "Log all transmitted EtherCAT packets",
         &enableTxPacketDebug},
        {"rx-ethercat-packets",
         "Log all received EtherCAT packets",
         &enableRxPacketDebug},
        {"rx-pdo",
         "Log received PDO (master->slave) data",
         &enableRxPDODebug},
        {"tx-pdo",
         "Log transmitted PDO (slave->master) data",
         &enableTxPDODebug},
        {"dc",
         "Log distributed-clock (DC) debug information",
         nullptr},
        {"fmmu",
         "Log FMMU register writes during configuration",
         &enableFmmuDebug},
        {"sii-eeprom",
         "Log SII / EEPROM access details",
         &enableSIIEEPROMDebug},
        {"coe-reads",
         "Log CoE (SDO) read operations",
         &enableCoEReadsDebug},
        {"coe-writes",
         "Log CoE (SDO) write operations",
         &enableCoEWritesDebug},
        {"coe-rx-packets",
         "Log CoE received packets",
         &enableCoERxPacketsDebug},
        {"coe-tx-packets",
         "Log CoE transmitted packets",
         &enableCoETxPacketsDebug},
    };
    return kFlags;
}

} // namespace debug
} // namespace EtherCAT

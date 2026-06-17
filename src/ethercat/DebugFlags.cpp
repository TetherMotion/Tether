/**
 * @file DebugFlags.cpp
 * @brief Backing storage for EtherCAT debug flag accessor functions.
 */

#include "tether/ethercat/DebugFlags.hpp"

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

} // namespace debug
} // namespace EtherCAT

/**
 * @file Master_sdo.cpp
 * @brief Master — CoE/SDO transport and test hooks
 */

#include "tether/ethercat/Master.hpp"
#include "tether/ethercat/Slave.hpp"
#include "tether/ethercat/DC.hpp"
#include "tether/ethercat/PDOManager.hpp"
#include "tether/ethercat/SDOManager.hpp"
#include "tether/ethercat/FoE.hpp"
#include "tether/ethercat/VoE.hpp"
#include "tether/ethercat/EoE.hpp"
#include "tether/ethercat/FaultDetection.hpp"
#include "tether/ethercat/RealtimeLoop.hpp"
#include "tether/ethercat/SyncManagerValidation.hpp"
#include "tether/sii/SIIParser.hpp"
#include "tether/fmmu/FMMUConfiguration.hpp"
#include "raw/internal.hpp"
#include "tether/platform/Platform.hpp"

#include <thread>
#include <chrono>
#include <algorithm>
#include <cstdio>
#include <cstring>
#include <cerrno>
#include "sii/SIIReader.hpp"
#include <inttypes.h>

namespace EtherCAT {

static const char* TAG = "ethercat";

// Global debug flag for al-state (shared with Slave)
extern bool g_debug_statemachine;

// Global debug flags for tx/rx packet logging (shared with Slave)
extern bool g_debug_tx_packets;
extern bool g_debug_rx_packets;

// Global debug flags for PDO logging (shared with PDOManager)
extern bool g_debug_rx_pdo;
extern bool g_debug_tx_pdo;

} // namespace EtherCAT

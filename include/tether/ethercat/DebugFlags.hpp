/**
 * @file DebugFlags.hpp
 * @brief Accessor functions for EtherCAT global debug flags.
 *
 * These functions replace raw extern bool globals so that debug state
 * works correctly when Tether is built as shared libraries (including
 * on Windows, where CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS auto-exports
 * functions but not data variables).
 */

#pragma once

namespace EtherCAT {
namespace debug {

bool& rxPDO();
bool& txPDO();
bool& stateMachine();
bool& txPackets();
bool& rxPackets();
bool& fmmu();
bool& siiEeprom();
bool& coeReads();
bool& coeWrites();
bool& coeRxPackets();
bool& coeTxPackets();

} // namespace debug
} // namespace EtherCAT

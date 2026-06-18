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

#include <string>
#include <vector>

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

/**
 * @brief Metadata for a single debug flag.
 */
struct DebugFlagInfo {
    std::string name;           ///< Flag name used on the command line.
    std::string description;    ///< Human-readable description.
    void (*setter)(bool);       ///< Function to enable the flag (may be nullptr).
};

/**
 * @brief Registry of all available debug flags.
 *
 * Contains every known flag name, its description, and the setter that
 * enables the corresponding debug logging.  Flags that have no global
 * setter (e.g. "sii-derivation") store nullptr in @p setter.
 */
const std::vector<DebugFlagInfo>& allDebugFlags();

} // namespace debug
} // namespace EtherCAT

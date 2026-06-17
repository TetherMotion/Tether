#pragma once

#include <cstdint>
#include <string>
#include <magic_enum/magic_enum.hpp>

#include "tether/ethercat/SyncManager.hpp"
#include "tether/profiles/cia402/1Cxx-SyncManagerParameters.hpp"
#include "tether/ethercat/Master.hpp"  // for logging context
#include "tether/ethercat/SDO.hpp"
#include "tether/ethercat/CoEManager.hpp"  // for CoEManager
#include "logging/Logger.hpp"  // for TETHER_LOGI/W

namespace EtherCAT {
namespace SyncManager {
namespace Utils {

/**
 * @brief Read the synchronization mode for a given SyncManager (SM2/SM3).
 *
 * @param sdo        Reference to an SDO manager (usual master.sdoManager()).
 * @param slave_idx  EtherCAT slave index (0-based)
 * @param sm_index   Sync manager index (2 for SM2 / 3 for SM3)
 * @param out_mode   Output variable filled with the raw mode value.
 * @param timeout_ms SDO timeout, defaults to 2000 ms.
 * @return true if the read succeeded, false otherwise.
 */
// ---------------------------------------------------------------------------
// Generic helpers that work with ObjectDictionaryEntry
// ---------------------------------------------------------------------------

/**
 * @brief Low-level read that uses an ObjectDictionaryEntry definition.
 *
 * This simply forwards to `SDOManager::readSync` using the index/subindex
 * stored in @p entry.  The caller must supply a buffer of appropriate size.
 */
inline bool readSyncEntry(CoE::CoEManager& coe,
                          const ::EtherCAT::ObjectDictionary::ObjectDictionaryEntry& entry,
                          void* buffer,
                          size_t len,
                          uint32_t timeout_ms = 2000)
{
    return coe.readSync(entry.index, entry.subindex, buffer, len, timeout_ms);
}

/**
 * @brief Type-safe wrapper around `readSyncEntry`.
 *
 * Automatically infers the buffer size from the template parameter.
 */
template<typename T>
inline bool readSync(CoE::CoEManager& coe,
                     const ::EtherCAT::ObjectDictionary::ObjectDictionaryEntry& entry,
                     T& out,
                     uint32_t timeout_ms = 2000)
{
    static_assert(std::is_trivially_copyable_v<T>, "T must be trivially copyable");
    return readSyncEntry(coe, entry, &out, sizeof(out), timeout_ms);
}

/**
 * @brief Read an entry and log its value (decimal/hex depending on type).
 *
 * For simple integer types the value is logged numerically; for others the
 * raw bytes are printed in hex.
 */
inline void printSyncEntry(CoE::CoEManager& coe,
                           const ::EtherCAT::ObjectDictionary::ObjectDictionaryEntry& entry,
                           const char* tag = "SyncMgr",
                           uint32_t timeout_ms = 2000)
{
    char buf[8] = {0};
    if (!readSyncEntry(coe, entry, buf, sizeof(buf), timeout_ms)) {
        TETHER_LOGW(tag, "0x%04X:%02X (%s) read FAILED", entry.index, entry.subindex, entry.name);
        return;
    }

    switch (entry.data_type) {
    case ::EtherCAT::ObjectDictionary::ObjectDictionaryDataType::Integer8:
    case ::EtherCAT::ObjectDictionary::ObjectDictionaryDataType::Unsigned8:
        TETHER_LOGI(tag, "0x%04X:%02X (%s) = %u", entry.index, entry.subindex, entry.name, static_cast<unsigned>(buf[0]));
        break;
    case ::EtherCAT::ObjectDictionary::ObjectDictionaryDataType::Integer16:
    case ::EtherCAT::ObjectDictionary::ObjectDictionaryDataType::Unsigned16: {
        uint16_t v = buf[0] | (buf[1] << 8);
        TETHER_LOGI(tag, "0x%04X:%02X (%s) = 0x%04X", entry.index, entry.subindex, entry.name, v);
        break;
    }
    case ::EtherCAT::ObjectDictionary::ObjectDictionaryDataType::Integer32:
    case ::EtherCAT::ObjectDictionary::ObjectDictionaryDataType::Unsigned32: {
        uint32_t v = buf[0] | (buf[1] << 8) | (buf[2] << 16) | (buf[3] << 24);
        TETHER_LOGI(tag, "0x%04X:%02X (%s) = %u", entry.index, entry.subindex, entry.name, v);
        break;
    }
    default:
        // fallback: hex dump first four bytes
        TETHER_LOGI(tag, "0x%04X:%02X (%s) = [%02X %02X %02X %02X]",
                 entry.index, entry.subindex, entry.name,
                 static_cast<unsigned>(buf[0]), static_cast<unsigned>(buf[1]),
                 static_cast<unsigned>(buf[2]), static_cast<unsigned>(buf[3]));
        break;
    }
}


/**
 * @brief Read the synchronization mode for a given SyncManager (SM2/SM3).
 *
 * @param sdo        Reference to an SDO manager (usual master.sdoManager()).
 * @param slave_idx  EtherCAT slave index (0-based)
 * @param sm_index   Sync manager index (2 for SM2 / 3 for SM3)
 * @param out_mode   Output variable filled with the raw mode value.
 * @param timeout_ms SDO timeout, defaults to 2000 ms.
 * @return true if the read succeeded, false otherwise.
 */
inline bool readSyncMode(CoE::CoEManager& coe,
                         uint8_t sm_index,
                         uint16_t& out_mode,
                         uint32_t timeout_ms = 2000)
{
    // delegate to generic entry-based reader
    using namespace CiA301::Parameters1Cxx;
    auto entry = (sm_index == 2) ? SM2SyncMode : SM3SyncMode;
    return readSync(coe, entry, out_mode, timeout_ms);
}

/**
 * @brief Read the cycle time (nanoseconds) for the given SyncManager.
 */
inline bool readCycleTime(CoE::CoEManager& coe,
                          uint8_t sm_index,
                          uint32_t& out_time,
                          uint32_t timeout_ms = 2000)
{
    using namespace CiA301::Parameters1Cxx;
    auto entry = (sm_index == 2) ? SM2CycleTime : SM3CycleTime;
    return readSync(coe, entry, out_time, timeout_ms);
}

/**
 * @brief Read the supported sync-types bitmask for the given SyncManager.
 */
inline bool readSupportedSyncTypes(CoE::CoEManager& coe,
                                   uint8_t sm_index,
                                   uint16_t& out_mask,
                                   uint32_t timeout_ms = 2000)
{
    using namespace CiA301::Parameters1Cxx;
    auto entry = (sm_index == 2) ? SM2SupportedSyncTypes : SM3SupportedSyncTypes;
    return readSync(coe, entry, out_mask, timeout_ms);
}

// ---------------------------------------------------------------------------
// Stringification helpers
// ---------------------------------------------------------------------------

inline std::string syncModeToString(uint16_t raw) {
    auto mode = static_cast<CiA301::Parameters1Cxx::SyncMode>(raw);
    auto name = magic_enum::enum_name(mode);
    return std::string(name);
}

inline std::string supportedSyncTypesToString(uint16_t mask) {
    return CiA301::Parameters1Cxx::supportedSyncTypesToString(mask);
}


// ---------------------------------------------------------------------------
// ESC register helpers
// ---------------------------------------------------------------------------

/**
 * @brief Dump the 8-byte ESC register block for a single Sync Manager.
 *
 * Reads ESC address 0x0800 + sm*8 and logs address/length/control/status.
 * If the APRD fails, a warning is printed.
 */
inline void printSMEscRegisters(EtherCAT::Master& master,
                                uint16_t slave_idx,
                                uint8_t sm,
                                const char* tag = "SyncMgr")
{
    uint16_t adp = EtherCAT::Master::adpForSlaveIndex(slave_idx);
    uint8_t regs[8] = {0};
    uint16_t base = static_cast<uint16_t>(0x0800 + sm * 8);
    if (master.readRegister(adp, base, regs, sizeof(regs), 200)) {
        uint16_t addr = regs[0] | (regs[1] << 8);
        uint16_t len  = regs[2] | (regs[3] << 8);
        TETHER_LOGI(tag, "SM%u ESC: Addr=0x%04X Len=%u Ctrl=0x%02X Status=0x%02X Act=0x%02X",
                 sm, addr, len, regs[4], regs[5], regs[6]);
    } else {
        TETHER_LOGW(tag, "SM%u ESC read FAILED", sm);
    }
}

/**
 * @brief Convenience wrapper to dump all eight Sync Manager ESC blocks.
 */
inline void printAllSMEscRegisters(EtherCAT::Master& master,
                                   uint16_t slave_idx,
                                   const char* tag = "SyncMgr")
{
    for (uint8_t sm = 0; sm < 8; ++sm) {
        printSMEscRegisters(master, slave_idx, sm, tag);
    }
}

// ---------------------------------------------------------------------------
// Printing helpers
// ---------------------------------------------------------------------------

inline void printSyncMode(CoE::CoEManager& coe,
                          uint8_t sm_index,
                          const char* tag = "SyncMgr")
{
    using namespace CiA301::Parameters1Cxx;
    auto entry = (sm_index == 2) ? SM2SyncMode : SM3SyncMode;
    printSyncEntry(coe, entry, tag);
}

inline void printCycleTime(CoE::CoEManager& coe,
                           uint8_t sm_index,
                           const char* tag = "SyncMgr")
{
    using namespace CiA301::Parameters1Cxx;
    auto entry = (sm_index == 2) ? SM2CycleTime : SM3CycleTime;
    printSyncEntry(coe, entry, tag);
}

inline void printSupportedSyncTypes(CoE::CoEManager& coe,
                                    uint8_t sm_index,
                                    const char* tag = "SyncMgr")
{
    using namespace CiA301::Parameters1Cxx;
    auto entry = (sm_index == 2) ? SM2SupportedSyncTypes : SM3SupportedSyncTypes;
    printSyncEntry(coe, entry, tag);
}

/**
 * @brief Convenience function that reads and prints the most commonly
 *        inspected synchronization parameters for SM2 and SM3.
 *
 * This replicates the diagnostic block previously located in the example.
 */
inline void printSyncDiagnostics(EtherCAT::Master& master,
                                 uint16_t slave_idx,
                                 const char* tag = "SyncMgr")
{
    using namespace CiA301::Parameters1Cxx;
    auto& coe = master.sdoManager(slave_idx);

    TETHER_LOGI(tag, "  ----- SM Sync Mode (0x%04X / 0x%04X) -----",
             kIdxSM2Sync, kIdxSM3Sync);

    printSyncMode(coe, 2, tag);
    printSyncMode(coe, 3, tag);

    printCycleTime(coe, 2, tag);
    printSupportedSyncTypes(coe, 2, tag);

    TETHER_LOGI(tag, "  -------------------------------------------");
}

} // namespace Utils
} // namespace SyncManager
} // namespace EtherCAT

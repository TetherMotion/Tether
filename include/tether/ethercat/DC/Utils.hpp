#pragma once

#include <cstdint>
#include <string>

#include "tether/ethercat/EtherCATMaster.hpp"
#include "tether/ethercat/EtherCATDC.hpp"  // for DCRegisters & DC class
#include "logging/Logger.hpp"     // for TETHER_LOGI/W

namespace EtherCAT {
namespace DC {
namespace Utils {

// ---------------------------------------------------------------------------
// Low-level reads
// ---------------------------------------------------------------------------

inline bool readSyncActivationStatus(EtherCAT::Master& master,
                                    uint16_t slave_idx,
                                    uint8_t& out_status,
                                    uint32_t timeout_ms = 200)
{
    auto* dc = master.dc().get();
    if (!dc) return false;
    return dc->readRegister(slave_idx, DCRegisters::DCSyncAct, &out_status, 1, timeout_ms);
}

inline bool readSyncActivationMask(EtherCAT::Master& master,
                                  uint16_t slave_idx,
                                  uint16_t& out_mask,
                                  uint32_t timeout_ms = 200)
{
    auto* dc = master.dc().get();
    if (!dc) return false;
    uint8_t buf[2] = {0};
    if (!dc->readRegister(slave_idx, DCRegisters::DCCuc, buf, 2, timeout_ms))
        return false;
    out_mask = static_cast<uint16_t>(buf[0] | (buf[1] << 8));
    return true;
}

inline bool readSync0CycleTime(EtherCAT::Master& master,
                               uint16_t slave_idx,
                               uint32_t& out_ns,
                               uint32_t timeout_ms = 200)
{
    auto* dc = master.dc().get();
    if (!dc) return false;
    uint8_t buf[4] = {0};
    if (!dc->readRegister(slave_idx, DCRegisters::DCCycle0, buf, 4, timeout_ms))
        return false;
    out_ns = static_cast<uint32_t>(buf[0] | (buf[1] << 8) | (buf[2] << 16) | (buf[3] << 24));
    return true;
}

inline bool readSystemTimeDiff(EtherCAT::Master& master,
                               uint16_t slave_idx,
                               int32_t& out_diff,
                               uint32_t timeout_ms = 200)
{
    auto* dc = master.dc().get();
    if (!dc) return false;
    uint8_t buf[4] = {0};
    if (!master.dc().readRegister(slave_idx, DCRegisters::DCSysDiff, buf, 4, timeout_ms))
        return false;
    out_diff = static_cast<int32_t>(buf[0] | (buf[1] << 8) | (buf[2] << 16) | (buf[3] << 24));
    return true;
}

inline bool readSyncImpulseCounter(EtherCAT::Master& master,
                                   uint16_t slave_idx,
                                   uint16_t& out_count,
                                   uint32_t timeout_ms = 200)
{
    auto* dc = master.dc().get();
    if (!dc) return false;
    uint8_t buf[2] = {0};
    if (!master.dc().readRegister(slave_idx, DCRegisters::DCTimeFilter, buf, 2, timeout_ms))
        return false;
    out_count = static_cast<uint16_t>(buf[0] | (buf[1] << 8));
    return true;
}

// ---------------------------------------------------------------------------
// Formatting helpers
// ---------------------------------------------------------------------------

inline std::string syncActivationMaskToString(uint16_t mask) {
    std::string ret;
    if (mask & 0x0001u) ret += "SYNC0_EN";
    else ret += "SYNC0_DIS";
    if (mask & 0x0002u) ret += " SYNC1_EN";
    else ret += " SYNC1_DIS";
    return ret;
}

// ---------------------------------------------------------------------------
// Print helpers
// ---------------------------------------------------------------------------

inline void printSyncActivationStatus(EtherCAT::Master& master,
                                      uint16_t slave_idx,
                                      const char* tag = "DC")
{
    uint8_t status = 0;
    if (readSyncActivationStatus(master, slave_idx, status)) {
        TETHER_LOGI(tag, "0x0981 (DC Activation Status) = 0x%02X %s",
                 status,
                 (status & 0x01) ? "[SYNC0 active]" : "[SYNC0 NOT active]");
    } else {
        TETHER_LOGW(tag, "0x0981 read FAILED");
    }
}

inline void printSyncActivationMask(EtherCAT::Master& master,
                                    uint16_t slave_idx,
                                    const char* tag = "DC")
{
    uint16_t mask = 0;
    if (readSyncActivationMask(master, slave_idx, mask)) {
        TETHER_LOGI(tag, "0x0980 (DC Sync Activation) = 0x%04X %s",
                 mask, syncActivationMaskToString(mask).c_str());
    } else {
        TETHER_LOGW(tag, "0x0980 read FAILED");
    }
}

inline void printSync0CycleTime(EtherCAT::Master& master,
                                uint16_t slave_idx,
                                const char* tag = "DC")
{
    uint32_t ct = 0;
    if (readSync0CycleTime(master, slave_idx, ct)) {
        TETHER_LOGI(tag, "0x09A0 (SYNC0 Cycle Time) = %u ns (%.1f us)", ct, ct / 1000.0);
    } else {
        TETHER_LOGW(tag, "0x09A0 read FAILED");
    }
}

inline void printSystemTimeDiff(EtherCAT::Master& master,
                                uint16_t slave_idx,
                                const char* tag = "DC")
{
    int32_t diff = 0;
    if (readSystemTimeDiff(master, slave_idx, diff)) {
        TETHER_LOGI(tag, "0x092C (System Time Diff) = %ld ns", (long)diff);
    } else {
        TETHER_LOGW(tag, "0x092C read FAILED");
    }
}

inline void printSyncImpulseCounter(EtherCAT::Master& master,
                                    uint16_t slave_idx,
                                    const char* tag = "DC")
{
    uint16_t wc = 0;
    if (readSyncImpulseCounter(master, slave_idx, wc)) {
        TETHER_LOGI(tag, "0x0934 (SyncImpulse Counter) = %u", wc);
    } else {
        TETHER_LOGW(tag, "0x0934 read FAILED");
    }
}

/**
 * @brief Top-level helper that echoes the full DC diagnostic block shown in
 *        examples.
 */
inline void printDCDiagnostics(EtherCAT::Master& master,
                               uint16_t slave_idx,
                               const char* tag = "DC")
{
    TETHER_LOGI(tag, "  ----- DC SYNC Configuration -----");
    printSyncActivationStatus(master, slave_idx, tag);
    printSyncActivationMask(master, slave_idx, tag);
    printSync0CycleTime(master, slave_idx, tag);
    printSystemTimeDiff(master, slave_idx, tag);
    printSyncImpulseCounter(master, slave_idx, tag);
    TETHER_LOGI(tag, "  -----------------------------------");
}

} // namespace Utils
} // namespace DC
} // namespace EtherCAT

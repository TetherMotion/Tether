/**
 * @file dc_sync_characterize.cpp
 * @brief Characterize a single slave's DC SYNC0/SYNC1 support and SYNC0
 *        cycle-time limits.
 *
 * The tool brings up a single EtherCAT slave, checks whether it advertises
 * SYNC0 and SYNC1 signal support (from the SII DC category and the ESC
 * features register), then sweeps SYNC0 cycle times from large to small
 * until the slave reports an AL Status error.
 *
 * Flow:
 *   1. Discover slaves, pick the target slave.
 *   2. Configure mailbox from SII, transition to PRE-OP.
 *   3. Read SII DC config + ESC features register for SYNC0/SYNC1 support.
 *   4. Configure PDO sync managers from SII.
 *   5. Initialize Distributed Clocks (time sync only, no SYNC signals).
 *   6. Transition to SAFE-OP.
 *   7. Probe SYNC0 support: activate SYNC0, check AL_STATUS / AL_STATUS_CODE.
 *   8. Probe SYNC1 support: activate SYNC1, check AL_STATUS / AL_STATUS_CODE.
 *   9. Sweep SYNC0 cycle times (geometric sweep from --start-ns to --stop-ns),
 *      reading AL_STATUS_CODE after each step.  Stop at the first AL error
 *      and report the offending cycle time + error code.
 *
 * Usage (Linux, requires root or CAP_NET_RAW):
 *   ./dc_sync_characterize                  # uses eth0, slave 0
 *   ./dc_sync_characterize -i enp3s0        # specify interface
 *   ./dc_sync_characterize -s 1             # specify slave index
 *   ./dc_sync_characterize --start-ns 1000000 --stop-ns 100 --steps 30
 */

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <iostream>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "tether/ethercat/Master.hpp"
#include "tether/ethercat/Slave.hpp"
#include "tether/ethercat/Types.hpp"
#include "tether/ethercat/SyncManager.hpp"
#include "tether/ethercat/ESIFile.hpp"
#include "tether/ethercat/ESCRegisterMap.hpp"
#include "tether/ethercat/ESCFeatureReg.hpp"
#include "tether/ethercat/FaultDetection.hpp"
#include "tether/ethercat/DC.hpp"
#include "tether/ethercat/DCTypes.hpp"
#include "tether/ethercat/DCRegisters.hpp"
#include "tether/platform/EspCompat.hpp"
#include "tether/sii/SIIReader.hpp"
#include "tether/sii/SIIParser.hpp"

#include "common/ExampleHelpers.hpp"
#include "common/EtherCATHostSetup.hpp"

static const char* TAG = "dc_sync_characterize";

// ---------------------------------------------------------------------------
// DC register addresses (from DCTypes.hpp / ESCRegisterMap.hpp)
// ---------------------------------------------------------------------------
static constexpr uint16_t REG_AL_STATUS       = EtherCAT::ESCReg::ALStatus;       // 0x0130
static constexpr uint16_t REG_AL_STATUS_CODE  = EtherCAT::ESCReg::ALStatusCode;   // 0x0134
static constexpr uint16_t REG_DC_SYNC_ACT     = EtherCAT::toUInt16(EtherCAT::DCRegisters::DCSyncAct);  // 0x0981
static constexpr uint16_t REG_DC_CYCLE0       = EtherCAT::toUInt16(EtherCAT::DCRegisters::DCCycle0);   // 0x09A0
static constexpr uint16_t REG_DC_CYCLE1       = EtherCAT::toUInt16(EtherCAT::DCRegisters::DCCycle1);   // 0x09A4
static constexpr uint16_t REG_DC_START0       = EtherCAT::toUInt16(EtherCAT::DCRegisters::DCStart0);   // 0x0990

// ---------------------------------------------------------------------------
// Helpers — direct ESC register access via position-addressed datagrams
// ---------------------------------------------------------------------------

/// Read a 1-byte ESC register from a slave by position.
static bool readRegU8(EtherCAT::Master& m, uint16_t slave_idx,
                      uint16_t addr, uint8_t& out) {
    return m.readRegister(EtherCAT::SlaveAddress(slave_idx),
                          EtherCAT::RegisterAddress(addr),
                          &out, 1, 200);
}

/// Read a 2-byte ESC register (little-endian) from a slave by position.
static bool readRegU16(EtherCAT::Master& m, uint16_t slave_idx,
                       uint16_t addr, uint16_t& out) {
    uint8_t buf[2] = {0};
    if (!m.readRegister(EtherCAT::SlaveAddress(slave_idx),
                        EtherCAT::RegisterAddress(addr),
                        buf, 2, 200))
        return false;
    out = static_cast<uint16_t>(buf[0] | (buf[1] << 8));
    return true;
}

/// Read a 4-byte ESC register (little-endian) from a slave by position.
static bool readRegU32(EtherCAT::Master& m, uint16_t slave_idx,
                       uint16_t addr, uint32_t& out) {
    uint8_t buf[4] = {0};
    if (!m.readRegister(EtherCAT::SlaveAddress(slave_idx),
                        EtherCAT::RegisterAddress(addr),
                        buf, 4, 200))
        return false;
    out = static_cast<uint32_t>(buf[0] | (buf[1] << 8) |
                                (buf[2] << 16) | (buf[3] << 24));
    return true;
}

/// Write a 1-byte ESC register to a slave by position.
static bool writeRegU8(EtherCAT::Master& m, uint16_t slave_idx,
                       uint16_t addr, uint8_t val) {
    return m.writeRegister(EtherCAT::SlaveAddress(slave_idx),
                           EtherCAT::RegisterAddress(addr),
                           &val, 1, 200);
}

/// Write a 4-byte ESC register (little-endian) to a slave by position.
static bool writeRegU32(EtherCAT::Master& m, uint16_t slave_idx,
                        uint16_t addr, uint32_t val) {
    uint8_t buf[4] = {
        static_cast<uint8_t>(val & 0xFF),
        static_cast<uint8_t>((val >> 8) & 0xFF),
        static_cast<uint8_t>((val >> 16) & 0xFF),
        static_cast<uint8_t>((val >> 24) & 0xFF),
    };
    return m.writeRegister(EtherCAT::SlaveAddress(slave_idx),
                           EtherCAT::RegisterAddress(addr),
                           buf, 4, 200);
}

/// Write an 8-byte ESC register (little-endian) to a slave by position.
static bool writeRegU64(EtherCAT::Master& m, uint16_t slave_idx,
                        uint16_t addr, uint64_t val) {
    uint8_t buf[8];
    for (int i = 0; i < 8; ++i)
        buf[i] = static_cast<uint8_t>((val >> (i * 8)) & 0xFF);
    return m.writeRegister(EtherCAT::SlaveAddress(slave_idx),
                           EtherCAT::RegisterAddress(addr),
                           buf, 8, 200);
}

// ---------------------------------------------------------------------------
// AL status helpers
// ---------------------------------------------------------------------------

struct ALStatusSnapshot {
    uint16_t al_status = 0;       // 0x0130
    uint16_t al_status_code = 0;  // 0x0134
    bool read_ok = false;
};

static ALStatusSnapshot readALStatus(EtherCAT::Master& m, uint16_t slave_idx) {
    ALStatusSnapshot s;
    s.read_ok = readRegU16(m, slave_idx, REG_AL_STATUS, s.al_status) &&
                readRegU16(m, slave_idx, REG_AL_STATUS_CODE, s.al_status_code);
    return s;
}

static bool alHasError(const ALStatusSnapshot& s) {
    return EtherCAT::al_status_has_error(s.al_status) || s.al_status_code != 0;
}

// ---------------------------------------------------------------------------
// SYNC activation register helpers
// ---------------------------------------------------------------------------

/// Write the DC Sync Activation register (0x0981).
/// @param enable      DC unit enable (bit 0)
/// @param sync0       SYNC0 enable (bit 1)
/// @param sync1       SYNC1 enable (bit 2)
/// @param auto_act    Auto-activation (bit 3)
static bool writeSyncActivation(EtherCAT::Master& m, uint16_t slave_idx,
                                bool enable, bool sync0, bool sync1,
                                bool auto_act) {
    uint8_t val = 0;
    if (enable)   val |= EtherCAT::DC_SYNCACT_ENA;
    if (sync0)    val |= EtherCAT::DC_SYNCACT_SYNC0_ENA;
    if (sync1)    val |= EtherCAT::DC_SYNCACT_SYNC1_ENA;
    if (auto_act) val |= EtherCAT::DC_SYNCACT_AUTO_ACT;
    return writeRegU8(m, slave_idx, REG_DC_SYNC_ACT, val);
}

// ---------------------------------------------------------------------------
// SYNC0 cycle-time sweep
// ---------------------------------------------------------------------------

/// Generate a geometric sweep of cycle times from @p start_ns down to @p stop_ns.
static std::vector<uint32_t> generateSweep(uint32_t start_ns, uint32_t stop_ns,
                                           int steps) {
    std::vector<uint32_t> times;
    if (steps < 1) steps = 1;
    if (start_ns <= stop_ns) {
        times.push_back(start_ns);
        return times;
    }
    const double ratio = std::pow(static_cast<double>(stop_ns) /
                                  static_cast<double>(start_ns),
                                  1.0 / static_cast<double>(steps));
    double cur = static_cast<double>(start_ns);
    for (int i = 0; i <= steps; ++i) {
        uint32_t ns = static_cast<uint32_t>(std::round(cur));
        if (ns < stop_ns) ns = stop_ns;
        // Avoid duplicates from rounding
        if (times.empty() || times.back() != ns)
            times.push_back(ns);
        cur *= ratio;
        if (ns <= stop_ns) break;
    }
    // Ensure the final stop value is included
    if (times.empty() || times.back() != stop_ns)
        times.push_back(stop_ns);
    return times;
}

// ---------------------------------------------------------------------------
// Main
// ---------------------------------------------------------------------------

int main(int argc, char** argv) {
    argparse::ArgumentParser program("dc_sync_characterize");
    Tether::Examples::addInterfaceArg(program);
    Tether::Examples::addSlaveArg(program);
    Tether::Examples::addDebugArg(program);
    Tether::Examples::addVlanArgs(program);
    Tether::Examples::addMailboxSizeArg(program);
    Tether::Examples::addMailboxAddressArg(program);
    Tether::Examples::addEsiXmlArg(program);

    program.add_argument("--start-ns")
        .default_value(std::string("10000000"))
        .help("SYNC0 cycle time to start the sweep at, in ns (default 10000000 = 10 ms).");
    program.add_argument("--stop-ns")
        .default_value(std::string("100"))
        .help("Minimum SYNC0 cycle time to test, in ns (default 100). The sweep stops "
              "when the slave reports an AL error or this value is reached.");
    program.add_argument("--steps")
        .default_value(std::string("25"))
        .help("Number of geometric steps in the SYNC0 cycle-time sweep (default 25).");
    program.add_argument("--dwell-ms")
        .default_value(std::string("150"))
        .help("Time in ms to wait after applying each SYNC0 setting before reading "
              "AL_STATUS_CODE (default 150).");
    program.add_argument("--sync1-cycle-ns")
        .default_value(std::string("0"))
        .help("SYNC1 cycle time in ns to use when probing SYNC1 support (default 0 = "
              "skip SYNC1 cycle-time configuration, just toggle the enable bit).");

    try { program.parse_args(argc, argv); }
    catch (const std::runtime_error& err) {
        std::cerr << err.what() << "\n" << program;
        return 1;
    }

    const std::string iface     = Tether::Examples::resolveInterface(program.get<std::string>("--interface"), TAG);
    const int slave_idx_signed  = program.get<int>("--slave");
    const std::string debug_str = program.get<std::string>("--debug");

    // Sweep parameters
    uint32_t start_ns = 10000000;
    uint32_t stop_ns  = 100;
    int      steps    = 25;
    uint32_t dwell_ms = 150;
    uint32_t sync1_cycle_ns = 0;
    try {
        start_ns = static_cast<uint32_t>(std::stoul(program.get<std::string>("--start-ns")));
        stop_ns  = static_cast<uint32_t>(std::stoul(program.get<std::string>("--stop-ns")));
        steps    = std::stoi(program.get<std::string>("--steps"));
        dwell_ms = static_cast<uint32_t>(std::stoul(program.get<std::string>("--dwell-ms")));
        sync1_cycle_ns = static_cast<uint32_t>(std::stoul(program.get<std::string>("--sync1-cycle-ns")));
    } catch (...) {
        std::cerr << "Invalid numeric argument\n";
        return 1;
    }

    if (slave_idx_signed < 0 || slave_idx_signed > 65535) {
        std::cerr << "Invalid slave index\n";
        return 1;
    }
    const uint16_t slave_idx = static_cast<uint16_t>(slave_idx_signed);

    Tether::Examples::VlanConfig vlan;
    if (!Tether::Examples::parseVlanArgs(
            program.get<std::string>("--rx-vlan"),
            program.get<std::string>("--tx-vlan"),
            vlan, TAG)) {
        return 1;
    }

    Tether::Examples::MailboxSizeConfig mbSize;
    if (!Tether::Examples::parseMailboxSize(program.get<std::string>("--mailbox-size"), mbSize)) {
        return 1;
    }
    Tether::Examples::MailboxAddressConfig mbAddr;
    if (!Tether::Examples::parseMailboxAddress(program.get<std::string>("--mailbox-address"), mbAddr)) {
        return 1;
    }

    if (Tether::Examples::printDebugHelpIfRequested(debug_str)) return 0;
    auto debug_flags = Tether::Examples::parseDebugFlags(debug_str);

    TETHER_LOGI(TAG, "dc_sync_characterize — interface: %s, slave: %u", iface.c_str(), slave_idx);
    TETHER_LOGI(TAG, "Sweep: start=%u ns, stop=%u ns, steps=%d, dwell=%u ms",
                start_ns, stop_ns, steps, dwell_ms);

    // ---- Host Ethernet + Master startup ----
    Tether::Examples::HostEtherNetSession session;
    if (!Tether::Examples::initHostEthernet(session, iface, TAG)) {
        return 2;
    }

    EtherCAT::Master master;
    Tether::Examples::applyDebugFlags(debug_flags, master, TAG);

    if (!Tether::Examples::setupVlanAndRxCallback(session, master, vlan, TAG)) {
        Tether::Examples::shutdownHostEthernet(session);
        return 5;
    }
    Tether::Examples::startHostPollThread(session, TAG);

    if (!Tether::Examples::startHostMaster(session, master, vlan, TAG)) {
        Tether::Examples::shutdownHostEthernet(session);
        return 5;
    }

    if (!master.discoverSlaves()) {
        TETHER_LOGE(TAG, "No slaves discovered");
        master.stop();
        Tether::Examples::shutdownHostEthernet(session);
        return 4;
    }

    const uint16_t slaves = master.getDiscoveredSlaveCount();
    TETHER_LOGI(TAG, "Discovered %u slave(s)", slaves);

    if (slave_idx >= slaves) {
        TETHER_LOGE(TAG, "Slave index %u out of range (only %u slave(s) found)",
                    slave_idx, slaves);
        master.stop();
        Tether::Examples::shutdownHostEthernet(session);
        return 5;
    }

    auto& sl = master.slave(slave_idx);

    // ---- Configure mailbox + PRE-OP ----
    TETHER_LOGI(TAG, "Configuring mailbox for slave %u ...", slave_idx);
    EtherCAT::SlaveError mb_err = sl.configureMailbox(
        {.address = mbAddr.outAddress, .length = mbSize.outSize},
        {.address = mbAddr.inAddress,  .length = mbSize.inSize},
        0x0004);
    if (mb_err != EtherCAT::SlaveError::Ok) {
        TETHER_LOGE(TAG, "Mailbox configuration failed: %s",
                    EtherCAT::slaveErrorToString(mb_err));
        master.stop();
        Tether::Examples::shutdownHostEthernet(session);
        return 7;
    }

    auto pre_err = sl.transitionToPreOp();
    if (pre_err != EtherCAT::SlaveError::Ok) {
        TETHER_LOGE(TAG, "PRE-OP transition failed: %s",
                    EtherCAT::slaveErrorToString(pre_err));
        master.stop();
        Tether::Examples::shutdownHostEthernet(session);
        return 8;
    }
    TETHER_LOGI(TAG, "Slave %u is in PRE-OP", slave_idx);

    // ---- Read SII for DC config + ESC features ----
    EtherCAT::SII::SIIData sii_data;
    bool sii_ok = (sl.readSII(sii_data) == EtherCAT::SlaveError::Ok);

    bool sii_sync0 = false;
    bool sii_sync1 = false;
    if (sii_ok) {
        for (const auto& dc : sii_data.dc_configs) {
            if (dc.sync0Enabled()) sii_sync0 = true;
            if (dc.sync1Enabled()) sii_sync1 = true;
        }
        TETHER_LOGI(TAG, "SII DC category: %zu config(s), SYNC0=%s, SYNC1=%s",
                    sii_data.dc_configs.size(),
                    sii_sync0 ? "advertised" : "no",
                    sii_sync1 ? "advertised" : "no");
    } else {
        TETHER_LOGW(TAG, "SII read failed — SYNC0/SYNC1 support will be probed directly");
    }

    // Read ESC Features register (0x0008) to check DC capability
    uint16_t features_raw = 0;
    bool dc_hardware = false;
    if (readRegU16(master, slave_idx, EtherCAT::ESCReg::Features, features_raw)) {
        auto feat = std::bit_cast<EtherCAT::ESC::ESCFeatureReg>(features_raw);
        dc_hardware = feat.dc;
        TETHER_LOGI(TAG, "ESC Features (0x0008) = 0x%04X  DC=%s  DC64=%s  DC_enhanced=%s",
                    features_raw,
                    feat.dc ? "yes" : "no",
                    feat.dc_width64 ? "yes" : "no",
                    feat.dc_enhanced ? "yes" : "no");
    } else {
        TETHER_LOGW(TAG, "Failed to read ESC Features register (0x0008)");
    }

    if (!dc_hardware) {
        TETHER_LOGW(TAG, "Slave %u does not advertise DC hardware support "
                         "(ESC Features DC bit = 0). SYNC0/SYNC1 unavailable.",
                    slave_idx);
        // Still continue to probe — some slaves have the bit cleared but
        // still accept sync register writes.
    }

    // ---- Configure PDO sync managers from SII ----
    TETHER_LOGI(TAG, "Configuring PDO sync managers from SII ...");
    auto pdo_err = sl.configurePDOSyncManagers();
    if (pdo_err != EtherCAT::SlaveError::Ok) {
        TETHER_LOGE(TAG, "PDO sync-manager configuration from SII failed: %s — "
                         "cannot proceed to SAFE-OP.",
                    EtherCAT::slaveErrorToString(pdo_err));
        master.stop();
        Tether::Examples::shutdownHostEthernet(session);
        return 9;
    }

    // ---- Initialize Distributed Clocks ----
    // The DC realtime loop runs in a dedicated RT thread at 1 ms cycle time
    // (1 kHz).  It sends system-time reference frames to the slave so the
    // slave's DC hardware stays synchronized with the master throughout the
    // SYNC0/SYNC1 probe and cycle-time sweep.
    //
    // SYNC0/SYNC1 signal generation is left disabled in the initial config;
    // the probe/sweep phases activate them directly via ESC register writes
    // (0x0981 / 0x09A0 / 0x0990).  The DC RT loop continues running
    // independently, sending time-reference datagrams every 1 ms.
    EtherCAT::DC::DCConfig dc_config = EtherCAT::DC::DCConfig::defaults();
    dc_config.enable_sync0 = false;
    dc_config.enable_sync1 = false;
    dc_config.sync0_cycle_time_ns = 0;
    dc_config.sync1_cycle_time_ns = 0;
    dc_config.cycle_period_us = 1000;  // 1 ms RT cycle (1 kHz)

    if (!master.dc().init(dc_config, slaves)) {
        TETHER_LOGE(TAG, "Failed to initialize Distributed Clocks");
        master.stop();
        Tether::Examples::shutdownHostEthernet(session);
        return 10;
    }
    TETHER_LOGI(TAG, "Distributed Clocks initialized (time sync, no SYNC signals yet)");

    if (!master.dc().start()) {
        TETHER_LOGE(TAG, "Failed to start Distributed Clocks RT loop");
        master.stop();
        Tether::Examples::shutdownHostEthernet(session);
        return 10;
    }

    // Verify the DC realtime loop is actually running
    auto dc_state = master.dc().getState();
    if (dc_state != EtherCAT::DC::DCState::Running) {
        TETHER_LOGE(TAG, "DC realtime loop is not running (state=%s) — aborting",
                    EtherCAT::DC::dc_state_name(dc_state));
        master.dc().stop();
        master.stop();
        Tether::Examples::shutdownHostEthernet(session);
        return 10;
    }
    TETHER_LOGI(TAG, "DC realtime loop RUNNING — 1 ms cycle (1 kHz), sync frames active");

    // ---- Transition to SAFE-OP ----
    auto safe_err = sl.transitionToSafeOp();
    if (safe_err != EtherCAT::SlaveError::Ok) {
        TETHER_LOGE(TAG, "SAFE-OP transition failed: %s",
                    EtherCAT::slaveErrorToString(safe_err));
        master.dc().stop();
        master.stop();
        Tether::Examples::shutdownHostEthernet(session);
        return 11;
    }
    TETHER_LOGI(TAG, "Slave %u is in SAFE-OP", slave_idx);

    // Give the DC loop a moment to stabilize
    Tether::Platform::Clock::instance().delayMilliseconds(200);

    // ---- Baseline AL status ----
    auto baseline = readALStatus(master, slave_idx);
    if (!baseline.read_ok) {
        TETHER_LOGE(TAG, "Failed to read baseline AL_STATUS");
        master.dc().stop();
        master.stop();
        Tether::Examples::shutdownHostEthernet(session);
        return 12;
    }
    TETHER_LOGI(TAG, "Baseline AL_STATUS=0x%04X (%s), AL_STATUS_CODE=0x%04X (%s)",
                baseline.al_status,
                EtherCAT::al_status_get_state_name(baseline.al_status),
                baseline.al_status_code,
                EtherCAT::getALStatusCodeName(baseline.al_status_code));

    if (alHasError(baseline)) {
        TETHER_LOGE(TAG, "Slave already in error state before SYNC probing — aborting");
        master.dc().stop();
        master.stop();
        Tether::Examples::shutdownHostEthernet(session);
        return 12;
    }

    // ====================================================================
    // Phase 1: Probe SYNC0 support
    // ====================================================================
    TETHER_LOGI(TAG, "");
    TETHER_LOGI(TAG, "=== Phase 1: Probe SYNC0 support ===");

    // Use a conservative initial cycle time (10 ms) for the support probe
    const uint32_t probe_cycle_ns = 10000000;
    TETHER_LOGI(TAG, "  Writing SYNC0 cycle time = %u ns", probe_cycle_ns);
    writeRegU32(master, slave_idx, REG_DC_CYCLE0, probe_cycle_ns);

    TETHER_LOGI(TAG, "  Activating SYNC0 (DCSyncAct = ENA | SYNC0_ENA | AUTO_ACT)");
    writeSyncActivation(master, slave_idx, true, true, false, true);

    Tether::Platform::Clock::instance().delayMilliseconds(dwell_ms);

    auto s0_status = readALStatus(master, slave_idx);
    bool sync0_supported = false;
    if (!s0_status.read_ok) {
        TETHER_LOGW(TAG, "  Failed to read AL_STATUS after SYNC0 activation");
    } else {
        TETHER_LOGI(TAG, "  AL_STATUS=0x%04X (%s), AL_STATUS_CODE=0x%04X (%s)",
                    s0_status.al_status,
                    EtherCAT::al_status_get_state_name(s0_status.al_status),
                    s0_status.al_status_code,
                    EtherCAT::getALStatusCodeName(s0_status.al_status_code));
        sync0_supported = !alHasError(s0_status);
    }
    TETHER_LOGI(TAG, "  => SYNC0 %s", sync0_supported ? "SUPPORTED" : "NOT SUPPORTED");

    // Deactivate SYNC0 before probing SYNC1
    writeSyncActivation(master, slave_idx, true, false, false, true);
    Tether::Platform::Clock::instance().delayMilliseconds(dwell_ms);

    // ====================================================================
    // Phase 2: Probe SYNC1 support
    // ====================================================================
    TETHER_LOGI(TAG, "");
    TETHER_LOGI(TAG, "=== Phase 2: Probe SYNC1 support ===");

    if (sync1_cycle_ns > 0) {
        TETHER_LOGI(TAG, "  Writing SYNC1 cycle time = %u ns", sync1_cycle_ns);
        writeRegU32(master, slave_idx, REG_DC_CYCLE1, sync1_cycle_ns);
    }
    TETHER_LOGI(TAG, "  Activating SYNC1 (DCSyncAct = ENA | SYNC1_ENA | AUTO_ACT)");
    writeSyncActivation(master, slave_idx, true, false, true, true);

    Tether::Platform::Clock::instance().delayMilliseconds(dwell_ms);

    auto s1_status = readALStatus(master, slave_idx);
    bool sync1_supported = false;
    if (!s1_status.read_ok) {
        TETHER_LOGW(TAG, "  Failed to read AL_STATUS after SYNC1 activation");
    } else {
        TETHER_LOGI(TAG, "  AL_STATUS=0x%04X (%s), AL_STATUS_CODE=0x%04X (%s)",
                    s1_status.al_status,
                    EtherCAT::al_status_get_state_name(s1_status.al_status),
                    s1_status.al_status_code,
                    EtherCAT::getALStatusCodeName(s1_status.al_status_code));
        sync1_supported = !alHasError(s1_status);
    }
    TETHER_LOGI(TAG, "  => SYNC1 %s", sync1_supported ? "SUPPORTED" : "NOT SUPPORTED");

    // Deactivate SYNC1
    writeSyncActivation(master, slave_idx, true, false, false, true);
    Tether::Platform::Clock::instance().delayMilliseconds(dwell_ms);

    // ====================================================================
    // Phase 3: Sweep SYNC0 cycle times
    // ====================================================================
    TETHER_LOGI(TAG, "");
    TETHER_LOGI(TAG, "=== Phase 3: Sweep SYNC0 cycle times ===");

    if (!sync0_supported) {
        TETHER_LOGW(TAG, "SYNC0 not supported — skipping cycle-time sweep.");
    }

    const auto sweep = generateSweep(start_ns, stop_ns, steps);
    TETHER_LOGI(TAG, "  Sweep has %zu steps: %u ns -> %u ns",
                sweep.size(), sweep.front(), sweep.back());

    struct SweepResult {
        uint32_t cycle_ns;
        uint16_t al_status;
        uint16_t al_status_code;
        bool error;
    };

    std::vector<SweepResult> results;
    uint32_t first_error_ns = 0;
    uint16_t first_error_code = 0;
    uint32_t last_ok_ns = 0;
    bool hit_error = false;

    for (uint32_t cycle_ns : sweep) {
        // Write cycle time first, then activate SYNC0
        writeRegU32(master, slave_idx, REG_DC_CYCLE0, cycle_ns);

        // Read current system time and set SYNC0 start time to the next
        // cycle boundary (slave DC time + a few cycles).
        uint64_t sys_time = 0;
        {
            uint8_t buf[8] = {0};
            if (master.readRegister(EtherCAT::SlaveAddress(slave_idx),
                                    EtherCAT::RegisterAddress(0x0910),
                                    buf, 8, 200)) {
                for (int i = 7; i >= 0; --i)
                    sys_time = (sys_time << 8) | buf[i];
            }
        }
        if (sys_time > 0 && cycle_ns > 0) {
            uint64_t start_time = sys_time + cycle_ns * 5;
            start_time = ((start_time / cycle_ns) + 1) * cycle_ns;
            writeRegU64(master, slave_idx, REG_DC_START0, start_time);
        }

        writeSyncActivation(master, slave_idx, true, true, false, true);

        Tether::Platform::Clock::instance().delayMilliseconds(dwell_ms);

        auto st = readALStatus(master, slave_idx);
        SweepResult r{cycle_ns, st.al_status, st.al_status_code, false};
        if (!st.read_ok) {
            TETHER_LOGW(TAG, "  %7u ns: AL_STATUS read FAILED", cycle_ns);
            r.error = true;
            results.push_back(r);
            // Treat read failure as a hard error — stop the sweep
            hit_error = true;
            first_error_ns = cycle_ns;
            first_error_code = 0xFFFF;
            break;
        }

        r.error = alHasError(st);
        const char* verdict = r.error ? "ERROR" : "ok";
        TETHER_LOGI(TAG, "  %7u ns (%7.1f us): AL_STATUS=0x%04X %-8s  code=0x%04X (%s)  [%s]",
                    cycle_ns, cycle_ns / 1000.0,
                    st.al_status,
                    EtherCAT::al_status_get_state_name(st.al_status),
                    st.al_status_code,
                    EtherCAT::getALStatusCodeName(st.al_status_code),
                    verdict);

        results.push_back(r);

        if (r.error) {
            hit_error = true;
            first_error_ns = cycle_ns;
            first_error_code = st.al_status_code;
            break;  // stop at first AL error
        }
        last_ok_ns = cycle_ns;

        // Brief pause between steps
        Tether::Platform::Clock::instance().delayMilliseconds(50);
    }

    // Deactivate SYNC0 after the sweep
    writeSyncActivation(master, slave_idx, true, false, false, true);

    // ====================================================================
    // Summary
    // ====================================================================
    TETHER_LOGI(TAG, "");
    TETHER_LOGI(TAG, "=== Characterization Summary (slave %u) ===", slave_idx);
    TETHER_LOGI(TAG, "  DC hardware (ESC Features): %s", dc_hardware ? "yes" : "no");
    TETHER_LOGI(TAG, "  SII SYNC0 advertised:       %s", sii_sync0 ? "yes" : "no");
    TETHER_LOGI(TAG, "  SII SYNC1 advertised:       %s", sii_sync1 ? "yes" : "no");
    TETHER_LOGI(TAG, "  SYNC0 probe result:         %s", sync0_supported ? "SUPPORTED" : "NOT SUPPORTED");
    TETHER_LOGI(TAG, "  SYNC1 probe result:         %s", sync1_supported ? "SUPPORTED" : "NOT SUPPORTED");

    if (sync0_supported && !results.empty()) {
        if (hit_error) {
            TETHER_LOGI(TAG, "  SYNC0 sweep: first AL error at %u ns (%.1f us)",
                        first_error_ns, first_error_ns / 1000.0);
            TETHER_LOGI(TAG, "    AL_STATUS_CODE = 0x%04X (%s)",
                        first_error_code, EtherCAT::getALStatusCodeName(first_error_code));
            if (last_ok_ns > 0) {
                TETHER_LOGI(TAG, "  Last OK SYNC0 cycle time:   %u ns (%.1f us)",
                            last_ok_ns, last_ok_ns / 1000.0);
            }
        } else {
            TETHER_LOGI(TAG, "  SYNC0 sweep: no AL error encountered down to %u ns (%.1f us)",
                        sweep.back(), sweep.back() / 1000.0);
        }
    }

    // Print the full sweep table
    if (!results.empty()) {
        TETHER_LOGI(TAG, "");
        TETHER_LOGI(TAG, "  SYNC0 cycle-time sweep results:");
        TETHER_LOGI(TAG, "    %10s  %10s  %-10s  %s",
                    "ns", "us", "AL state", "AL_STATUS_CODE");
        for (const auto& r : results) {
            TETHER_LOGI(TAG, "    %10u  %10.1f  %-10s  0x%04X (%s) %s",
                        r.cycle_ns, r.cycle_ns / 1000.0,
                        EtherCAT::al_status_get_state_name(r.al_status),
                        r.al_status_code,
                        EtherCAT::getALStatusCodeName(r.al_status_code),
                        r.error ? " <-- ERROR" : "");
        }
    }

    // ---- DC realtime loop stats ----
    // Report how many 1 ms cycles the DC RT thread executed during the
    // characterization, plus jitter statistics.
    auto dc_stats = master.dc().getStats();
    TETHER_LOGI(TAG, "");
    TETHER_LOGI(TAG, "=== DC Realtime Loop Statistics ===");
    TETHER_LOGI(TAG, "  DC state:          %s",
                EtherCAT::DC::dc_state_name(master.dc().getState()));
    TETHER_LOGI(TAG, "  Total cycles:      %llu  (%.1f s at 1 ms)",
                (unsigned long long)dc_stats.cycle_count,
                dc_stats.cycle_count / 1000.0);
    TETHER_LOGI(TAG, "  Sync frames sent:  %llu",
                (unsigned long long)dc_stats.sync_count);
    TETHER_LOGI(TAG, "  PDO errors:        %llu",
                (unsigned long long)dc_stats.pdo_error_count);
    TETHER_LOGI(TAG, "  Max jitter:        %u us", dc_stats.max_jitter_us);
    TETHER_LOGI(TAG, "  Avg jitter:        %u us", dc_stats.avg_jitter_us);
    TETHER_LOGI(TAG, "  Last drift:        %ld ns", (long)dc_stats.last_drift_ns);

    // ---- Shutdown ----
    master.dc().stop();
    master.stop();
    Tether::Examples::shutdownHostEthernet(session);

    // Exit code: 0 = success, 1 = slave doesn't support SYNC0 at all,
    //            2 = sweep found an AL error (expected outcome)
    if (!sync0_supported) {
        return 1;
    }
    return hit_error ? 2 : 0;
}

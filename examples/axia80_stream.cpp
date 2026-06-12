/**
 * @file axia80_stream.cpp
 * @brief ATI Axia80 Force/Torque Sensor Streaming Example
 *
 * Discovers an Axia80 sensor on the EtherCAT bus, configures PDOs,
 * reads calibration data via SDO, and streams 6-DOF force/torque data.
 *
 * Usage (Linux, requires root or CAP_NET_RAW):
 *   ./axia80_stream              # uses eth0, streams engineering units
 *   ./axia80_stream -i enp3s0  # specify interface
 *   ./axia80_stream --raw      # stream raw sensor counts
 *   ./axia80_stream --dc off   # disable DC/Sync0
 */

#include <atomic>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <csignal>
#include <cstring>
#include <iomanip>
#include <iostream>
#include <memory>
#include <mutex>
#include <queue>
#include <thread>
#include <magic_enum/magic_enum.hpp>

#include "tether/ethercat/EtherCATMaster.hpp"
#include "tether/ethercat/EtherCATTypes.hpp"
#include "tether/ethercat/DC/Utils.hpp"
#include "tether/ethercat/VLANRouter.hpp"
#include "tether/ethercat/ESIParser.hpp"
#include "tether/ethercat/SyncManager.hpp"
#include "tether/hal/IEthernet.hpp"
#include "tether/platform/Platform.hpp"
#include "tether/sensors/Axia80.hpp"
#include "tether/fmmu/FMMUConfiguration.hpp"

#ifdef UNIT_TEST_HOST
#include <argparse/argparse.hpp>
#endif

// Forward-declare host transport helpers
namespace EtherCAT {
namespace Raw {
    void set_network_interface(const ::EtherCAT::NetworkInterface* iface);
    const ::EtherCAT::NetworkInterface* network_interface();
    void set_src_mac(const uint8_t src_mac[6]);
}
}

static const char* TAG = "axia80_stream";
static EtherCAT::EtherCATMaster* g_master = nullptr;

void signalHandler(int) {
    if (g_master) {
        g_master->requestCancel();
    }
}

#ifndef UNIT_TEST_HOST
// ----- ESP-IDF / embedded entry point -----
extern "C" void axia80_stream_main(const EtherCAT::NetworkInterface* iface,
                                     const uint8_t src_mac[6]) {
    TETHER_LOGI(TAG, "axia80_stream (embedded) — not yet implemented");
}

#else // UNIT_TEST_HOST — host/Linux build

// ============================================================================
// ESI Verification Helpers
// ============================================================================

namespace ESI = EtherCAT::ESI;
namespace Axia80 = EtherCAT::Sensors::Axia80;
namespace Axia80_pdo = EtherCAT::Sensors::Axia80_pdo;

static void logMismatch(const char* field, uint32_t expected, uint32_t actual) {
    TETHER_LOGW(TAG, "ESI mismatch: %s expected=0x%08X actual=0x%08X", field, expected, actual);
}

static void logMismatch(const char* field, uint16_t expected, uint16_t actual) {
    TETHER_LOGW(TAG, "ESI mismatch: %s expected=0x%04X actual=0x%04X", field, expected, actual);
}

static void logMismatch(const char* field, uint8_t expected, uint8_t actual) {
    TETHER_LOGW(TAG, "ESI mismatch: %s expected=0x%02X actual=0x%02X", field, expected, actual);
}

static void logMismatch(const char* field, int expected, int actual) {
    TETHER_LOGW(TAG, "ESI mismatch: %s expected=%d actual=%d", field, expected, actual);
}

static void logMismatchStr(const char* field, const char* expected, const char* actual) {
    TETHER_LOGW(TAG, "ESI mismatch: %s expected='%s' actual='%s'", field, expected, actual);
}

// ---- Phase 1: Verify intended (hardcoded) settings against ESI ----

static void verifyIntendedAgainstESI(const ESI::DeviceInfo& esi) {
    TETHER_LOGI(TAG, "=== ESI Intended-Settings Verification ===");

    // Identity
    if (esi.vendorId != 0 && esi.vendorId != Axia80::kVendorId) {
        logMismatch("VendorId", esi.vendorId, Axia80::kVendorId);
    }
    if (esi.productCode != 0 && esi.productCode != Axia80::kProductCode) {
        logMismatch("ProductCode", esi.productCode, Axia80::kProductCode);
    }

    // Mailbox timeouts
    if (esi.mailbox_request_timeout_ms.has_value()) {
        TETHER_LOGI(TAG, "ESI Mailbox RequestTimeout=%u ms", *esi.mailbox_request_timeout_ms);
    }
    if (esi.mailbox_response_timeout_ms.has_value()) {
        TETHER_LOGI(TAG, "ESI Mailbox ResponseTimeout=%u ms", *esi.mailbox_response_timeout_ms);
    }

    // Sync Managers
    for (size_t i = 0; i < esi.syncManagers.size() && i < 4; ++i) {
        const auto& sm = esi.syncManagers[i];
        TETHER_LOGI(TAG, "ESI SM%zu: addr=0x%04X len=%u ctrl=0x%02X enable=%u name=%s",
                    i, sm.startAddress, sm.defaultSize, sm.control, sm.enable, sm.name.c_str());
    }

    // FMMUs
    if (!esi.fmmus.empty()) {
        std::string fmmu_names;
        for (const auto& f : esi.fmmus) {
            if (!fmmu_names.empty()) fmmu_names += ", ";
            fmmu_names += f;
        }
        TETHER_LOGI(TAG, "ESI FMMUs (%zu): %s", esi.fmmus.size(), fmmu_names.c_str());
    }

    // Mailbox protocols
    if (esi.mailbox.protocols.has_value()) {
        TETHER_LOGI(TAG, "ESI Mailbox Protocols=0x%04X", *esi.mailbox.protocols);
    }

    // RxPDO intended check
    bool found_rxpdo = false;
    for (const auto& pdo : esi.rxPdos) {
        if (pdo.index == Axia80_pdo::RxPDO_1601.index) {
            found_rxpdo = true;
            if (!pdo.fixed) {
                TETHER_LOGW(TAG, "ESI RxPDO 0x%04X fixed=false (code expects true)", pdo.index);
            }
            if (pdo.sm != 2) {
                logMismatch("RxPDO SM", 2, pdo.sm);
            }
            size_t expected_entries = 2; // control1, control2
            if (pdo.entries.size() != expected_entries) {
                logMismatch("RxPDO entry count", static_cast<int>(expected_entries), static_cast<int>(pdo.entries.size()));
            }
            break;
        }
    }
    if (!found_rxpdo) {
        TETHER_LOGW(TAG, "ESI does not contain RxPDO 0x%04X", Axia80_pdo::RxPDO_1601.index);
    }

    // TxPDO intended check
    bool found_txpdo = false;
    for (const auto& pdo : esi.txPdos) {
        if (pdo.index == Axia80_pdo::TxPDO_1A00.index) {
            found_txpdo = true;
            if (!pdo.fixed) {
                TETHER_LOGW(TAG, "ESI TxPDO 0x%04X fixed=false (code expects true)", pdo.index);
            }
            if (pdo.sm != 3) {
                logMismatch("TxPDO SM", 3, pdo.sm);
            }
            size_t expected_entries = 8; // fx, fy, fz, tx, ty, tz, status, counter
            if (pdo.entries.size() != expected_entries) {
                logMismatch("TxPDO entry count", static_cast<int>(expected_entries), static_cast<int>(pdo.entries.size()));
            }
            break;
        }
    }
    if (!found_txpdo) {
        TETHER_LOGW(TAG, "ESI does not contain TxPDO 0x%04X", Axia80_pdo::TxPDO_1A00.index);
    }

    // Verify RxPDO entry details against hardcoded struct layout
    for (const auto& pdo : esi.rxPdos) {
        if (pdo.index != Axia80_pdo::RxPDO_1601.index) continue;
        const struct EntryCheck { uint16_t index; uint8_t sub; uint16_t bits; const char* name; } expected[] = {
            { Axia80::OD_CONTROL_CODES, 1, 32, "Control 1" },
            { Axia80::OD_CONTROL_CODES, 2, 32, "Control 2" },
        };
        for (size_t i = 0; i < pdo.entries.size() && i < 2; ++i) {
            const auto& e = pdo.entries[i];
            if (e.index != expected[i].index) logMismatch("RxPDO entry index", expected[i].index, e.index);
            if (e.subindex != expected[i].sub) logMismatch("RxPDO entry subindex", expected[i].sub, e.subindex);
            if (e.bitLen != expected[i].bits) logMismatch("RxPDO entry bits", expected[i].bits, e.bitLen);
        }
    }

    // Verify TxPDO entry details against hardcoded struct layout
    for (const auto& pdo : esi.txPdos) {
        if (pdo.index != Axia80_pdo::TxPDO_1A00.index) continue;
        const struct EntryCheck { uint16_t index; uint8_t sub; uint16_t bits; const char* name; } expected[] = {
            { Axia80::OD_READING_DATA, 1, 32, "Fx" },
            { Axia80::OD_READING_DATA, 2, 32, "Fy" },
            { Axia80::OD_READING_DATA, 3, 32, "Fz" },
            { Axia80::OD_READING_DATA, 4, 32, "Tx" },
            { Axia80::OD_READING_DATA, 5, 32, "Ty" },
            { Axia80::OD_READING_DATA, 6, 32, "Tz" },
            { Axia80::OD_STATUS_CODE, 0, 32, "Status Code" },
            { Axia80::OD_SAMPLE_COUNTER, 0, 32, "Sample Counter" },
        };
        for (size_t i = 0; i < pdo.entries.size() && i < 8; ++i) {
            const auto& e = pdo.entries[i];
            if (e.index != expected[i].index) logMismatch("TxPDO entry index", expected[i].index, e.index);
            if (e.subindex != expected[i].sub) logMismatch("TxPDO entry subindex", expected[i].sub, e.subindex);
            if (e.bitLen != expected[i].bits) logMismatch("TxPDO entry bits", expected[i].bits, e.bitLen);
        }
    }

    TETHER_LOGI(TAG, "=== End Intended-Settings Verification ===");
}

// ---- Phase 2: Read back actual slave registers and compare against ESI ----

static void verifyReadbackAgainstESI(const ESI::DeviceInfo& esi,
                                        EtherCAT::EtherCATMaster& master,
                                        EtherCAT::Sensors::Axia80Sensor& sensor,
                                        uint16_t slave_index)
{
    TETHER_LOGI(TAG, "=== ESI Readback Verification ===");

    auto& sl = sensor.slave();

    // 1. Identity (SII)
    EtherCAT::SII::SIIIdentity id;
    if (EtherCAT::SII::readSIIIdentity(master, slave_index, id)) {
        if (esi.vendorId != 0 && id.vendor_id != esi.vendorId) {
            logMismatch("Readback VendorId", esi.vendorId, id.vendor_id);
        }
        if (esi.productCode != 0 && id.product_code != esi.productCode) {
            logMismatch("Readback ProductCode", esi.productCode, id.product_code);
        }
        if (esi.revision != 0 && id.revision_number != esi.revision) {
            logMismatch("Readback Revision", esi.revision, id.revision_number);
        }
    } else {
        TETHER_LOGW(TAG, "Readback: failed to read SII identity");
    }

    // 2. Sync Manager hardware registers (SM0..SM3)
    for (uint8_t smIdx = 0; smIdx < 4 && smIdx < esi.syncManagers.size(); ++smIdx) {
        auto hw = sl.sm(smIdx).readHardwareConfig();
        if (!hw.read_ok) {
            TETHER_LOGW(TAG, "Readback SM%u: failed to read hardware config", smIdx);
            continue;
        }
        const auto& expected = esi.syncManagers[smIdx];
        if (hw.start_addr != expected.startAddress) {
            logMismatch("SM start_addr", expected.startAddress, hw.start_addr);
        }
        if (hw.length != expected.defaultSize) {
            logMismatch("SM length", expected.defaultSize, hw.length);
        }
        if (hw.control != expected.control) {
            logMismatch("SM control", expected.control, hw.control);
        }
        bool expected_enable = (expected.enable != 0);
        if (hw.isEnabled() != expected_enable) {
            TETHER_LOGW(TAG, "SM%u enable mismatch: expected=%s actual=%s",
                        smIdx, expected_enable ? "yes" : "no", hw.isEnabled() ? "yes" : "no");
        }
    }

    // 3. PDO Assignments via SDO (0x1C12 = SM2, 0x1C13 = SM3)
    for (int smAssign = 2; smAssign <= 3; ++smAssign) {
        uint16_t assignIndex = EtherCAT::SyncManager::pdoAssignIndex(smAssign);
        uint8_t count = 0;
        if (sl.sdoReadU8(assignIndex, 0, count) == EtherCAT::SlaveError::Ok) {
            uint16_t expectedCount = 1;
            if (count != expectedCount) {
                logMismatch("PDO assign count", expectedCount, static_cast<uint16_t>(count));
            }
            uint16_t assignedPdo = 0;
            if (sl.sdoReadU16(assignIndex, 1, assignedPdo) == EtherCAT::SlaveError::Ok) {
                uint16_t expectedPdo = (smAssign == 2) ? Axia80_pdo::RxPDO_1601.index : Axia80_pdo::TxPDO_1A00.index;
                if (assignedPdo != expectedPdo) {
                    logMismatch("Assigned PDO index", expectedPdo, assignedPdo);
                }
            }
        } else {
            TETHER_LOGW(TAG, "Readback: failed to read PDO assignment 0x%04X", assignIndex);
        }
    }

    // 4. PDO Mapping entries via SDO
    // RxPDO mapping (0x1601)
    {
        uint8_t mapCount = 0;
        if (sl.sdoReadU8(Axia80_pdo::RxPDO_1601.index, 0, mapCount) == EtherCAT::SlaveError::Ok) {
            const auto& expectedPdo = [&]() -> const ESI::PDO* {
                for (const auto& p : esi.rxPdos) {
                    if (p.index == Axia80_pdo::RxPDO_1601.index) return &p;
                }
                return nullptr;
            }();
            if (expectedPdo && mapCount != expectedPdo->entries.size()) {
                logMismatch("RxPDO map count", static_cast<uint16_t>(expectedPdo->entries.size()), static_cast<uint16_t>(mapCount));
            }
            for (uint8_t i = 1; i <= mapCount; ++i) {
                uint32_t entry = 0;
                if (sl.sdoReadU32(Axia80_pdo::RxPDO_1601.index, i, entry) == EtherCAT::SlaveError::Ok) {
                    uint16_t idx = EtherCAT::SyncManager::mappingIndex(entry);
                    uint8_t  sub = EtherCAT::SyncManager::mappingSubindex(entry);
                    uint8_t  bits = EtherCAT::SyncManager::mappingBits(entry);
                    if (expectedPdo && (i - 1) < expectedPdo->entries.size()) {
                        const auto& e = expectedPdo->entries[i - 1];
                        if (idx != e.index) logMismatch("RxPDO map index", e.index, idx);
                        if (sub != e.subindex) logMismatch("RxPDO map subindex", e.subindex, sub);
                        if (bits != e.bitLen) logMismatch("RxPDO map bits", e.bitLen, static_cast<uint16_t>(bits));
                    }
                }
            }
        } else {
            TETHER_LOGW(TAG, "Readback: failed to read RxPDO mapping 0x%04X", Axia80_pdo::RxPDO_1601.index);
        }
    }

    // TxPDO mapping (0x1A00)
    {
        uint8_t mapCount = 0;
        if (sl.sdoReadU8(Axia80_pdo::TxPDO_1A00.index, 0, mapCount) == EtherCAT::SlaveError::Ok) {
            const auto& expectedPdo = [&]() -> const ESI::PDO* {
                for (const auto& p : esi.txPdos) {
                    if (p.index == Axia80_pdo::TxPDO_1A00.index) return &p;
                }
                return nullptr;
            }();
            if (expectedPdo && mapCount != expectedPdo->entries.size()) {
                logMismatch("TxPDO map count", static_cast<uint16_t>(expectedPdo->entries.size()), static_cast<uint16_t>(mapCount));
            }
            for (uint8_t i = 1; i <= mapCount; ++i) {
                uint32_t entry = 0;
                if (sl.sdoReadU32(Axia80_pdo::TxPDO_1A00.index, i, entry) == EtherCAT::SlaveError::Ok) {
                    uint16_t idx = EtherCAT::SyncManager::mappingIndex(entry);
                    uint8_t  sub = EtherCAT::SyncManager::mappingSubindex(entry);
                    uint8_t  bits = EtherCAT::SyncManager::mappingBits(entry);
                    if (expectedPdo && (i - 1) < expectedPdo->entries.size()) {
                        const auto& e = expectedPdo->entries[i - 1];
                        if (idx != e.index) logMismatch("TxPDO map index", e.index, idx);
                        if (sub != e.subindex) logMismatch("TxPDO map subindex", e.subindex, sub);
                        if (bits != e.bitLen) logMismatch("TxPDO map bits", e.bitLen, static_cast<uint16_t>(bits));
                    }
                }
            }
        } else {
            TETHER_LOGW(TAG, "Readback: failed to read TxPDO mapping 0x%04X", Axia80_pdo::TxPDO_1A00.index);
        }
    }

    TETHER_LOGI(TAG, "=== End Readback Verification ===");
}

// ---- Realtime data queue ---------------------------------------------------

struct SensorFrame {
    double fx, fy, fz, tx_val, ty, tz;
    uint32_t status;
    uint32_t counter;
};

static std::queue<SensorFrame> data_queue;
static std::mutex queue_mtx;
static std::condition_variable queue_cv;

int main(int argc, char** argv) {
    // ---- Argument parsing ----
    argparse::ArgumentParser program("axia80_stream");
    program.add_argument("-i", "--interface")
        .default_value(std::string("eth0"))
        .help("Network interface name (e.g. eth0, enp3s0)");
    program.add_argument("--raw")
        .default_value<bool>(false)
        .implicit_value(true)
        .help("Stream raw sensor counts instead of engineering units");
    program.add_argument("-s", "--slave")
        .scan<'i', int>()
        .default_value(0)
        .help("Slave index on the bus (0-based)");
    program.add_argument("-t", "--time")
        .scan<'g', double>()
        .default_value(0.0)
        .help("Stream duration in seconds (0 = infinite until Ctrl-C)");
    program.add_argument("--debug")
        .default_value(std::string(""))
        .help("Comma-separated debug flags. Known flags: sii-derivation, mailbox-configuration, al-state, tx-ethercat-packets, rx-ethercat-packets, rx-pdo, tx-pdo");
    program.add_argument("--rx-vlan")
        .default_value(std::string(""))
        .help("RX VLAN filter: single VID (e.g. 100), range (e.g. 100-200), or 'any' for catch-all undefined target");
    program.add_argument("--tx-vlan")
        .default_value(std::string(""))
        .help("TX VLAN encapsulation: single VID (e.g. 100)");
    program.add_argument("--dc")
        .default_value(std::string("on"))
        .help("DC/Sync0 distributed clocks: 'on' or 'off' (default: on)");
    program.add_argument("--esi-xml")
        .default_value(std::string(""))
        .help("Path to ESI XML file for register verification");
    program.add_argument("--columns")
        .default_value(std::string(""))
        .help("Comma-separated TxPDO columns to display (e.g. fx,fy,fz or fx,fy,fz,tx,ty,tz,status,counter)");

    try { program.parse_args(argc, argv); }
    catch (const std::runtime_error& err) {
        std::cerr << err.what() << "\n" << program;
        return 1;
    }

    std::string iface = program.get<std::string>("--interface");
    bool raw_mode = program.get<bool>("--raw");
    int slave_idx = program.get<int>("--slave");
    double duration_sec = program.get<double>("--time");
    std::string debug_str = program.get<std::string>("--debug");
    std::string rx_vlan_str = program.get<std::string>("--rx-vlan");
    std::string tx_vlan_str = program.get<std::string>("--tx-vlan");
    std::string dc_str = program.get<std::string>("--dc");
    bool dc_enabled = (dc_str == "on" || dc_str == "true" || dc_str == "1");
    std::string esi_xml_path = program.get<std::string>("--esi-xml");
    std::string columns_str = program.get<std::string>("--columns");

    // Known debug flags
    const std::set<std::string> known_debug_flags = {
        "sii-derivation",
        "mailbox-configuration",
        "al-state",
        "tx-ethercat-packets",
        "rx-ethercat-packets",
        "rx-pdo",
        "tx-pdo",
        "dc",
        "fmmu"
    };

    // Parse debug flags
    std::set<std::string> debug_flags;
    std::set<std::string> unknown_flags;
    if (!debug_str.empty()) {
        std::stringstream ss(debug_str);
        std::string flag;
        while (std::getline(ss, flag, ',')) {
            // Trim whitespace
            flag.erase(0, flag.find_first_not_of(" \t"));
            flag.erase(flag.find_last_not_of(" \t") + 1);
            if (!flag.empty()) {
                debug_flags.insert(flag);
                if (known_debug_flags.find(flag) == known_debug_flags.end()) {
                    unknown_flags.insert(flag);
                }
            }
        }
    }

    // Warn about unknown debug flags
    if (!unknown_flags.empty()) {
        TETHER_LOGW(TAG, "Unknown debug flags:");
        for (const auto& flag : unknown_flags) {
            TETHER_LOGW(TAG, "  - %s", flag.c_str());
        }
        TETHER_LOGW(TAG, "Known debug flags:");
        for (const auto& flag : known_debug_flags) {
            TETHER_LOGW(TAG, "  - %s", flag.c_str());
        }
    }

    // Parse column selection
    std::vector<std::string> selected_columns;
    const std::set<std::string> valid_columns = {"fx", "fy", "fz", "tx", "ty", "tz", "status", "counter"};
    if (!columns_str.empty()) {
        std::stringstream ss(columns_str);
        std::string col;
        while (std::getline(ss, col, ',')) {
            // Trim whitespace
            col.erase(0, col.find_first_not_of(" \t"));
            col.erase(col.find_last_not_of(" \t") + 1);
            if (!col.empty()) {
                if (valid_columns.find(col) == valid_columns.end()) {
                    std::cerr << "Invalid column: " << col << "\n";
                    std::cerr << "Valid columns: fx, fy, fz, tx, ty, tz, status, counter\n";
                    return 1;
                }
                selected_columns.push_back(col);
            }
        }
    } else {
        // Default: all columns
        selected_columns = {"fx", "fy", "fz", "tx", "ty", "tz", "status", "counter"};
    }

    // Enable statemachine debug if requested
    if (debug_flags.count("al-state")) {
        EtherCAT::enableStateMachineDebug(true);
        TETHER_LOGI(TAG, "EtherCAT state machine debug logging enabled");
    }

    // Enable TX packet debug if requested
    if (debug_flags.count("tx-ethercat-packets")) {
        EtherCAT::enableTxPacketDebug(true);
        TETHER_LOGI(TAG, "TX EtherCAT packet debug logging enabled");
    }

    // Enable RX packet debug if requested
    if (debug_flags.count("rx-ethercat-packets")) {
        EtherCAT::enableRxPacketDebug(true);
        TETHER_LOGI(TAG, "RX EtherCAT packet debug logging enabled");
    }

    // Enable RxPDO debug if requested
    if (debug_flags.count("rx-pdo")) {
        EtherCAT::enableRxPDODebug(true);
        TETHER_LOGI(TAG, "RxPDO debug logging enabled");
    }

    // Enable TxPDO debug if requested
    if (debug_flags.count("tx-pdo")) {
        EtherCAT::enableTxPDODebug(true);
        TETHER_LOGI(TAG, "TxPDO debug logging enabled");
    }

    // Enable FMMU debug if requested
    bool fmmu_debug = debug_flags.count("fmmu");
    if (fmmu_debug) {
        EtherCAT::enableFmmuDebug(true);
        TETHER_LOGI(TAG, "FMMU debug logging enabled");
    }

    // ---- Parse VLAN arguments ----
    bool vlan_mode = !rx_vlan_str.empty() || !tx_vlan_str.empty();
    std::optional<uint16_t> tx_vlan;
    bool rx_any = false;
    std::optional<EtherCAT::VLANRouter::VLANRange> rx_range;

    if (vlan_mode) {
        if (!tx_vlan_str.empty()) {
            try {
                int v = std::stoi(tx_vlan_str);
                if (v < 1 || v > 4095) {
                    std::cerr << "--tx-vlan must be in range 1–4095\n";
                    return 1;
                }
                tx_vlan = static_cast<uint16_t>(v);
            } catch (...) {
                std::cerr << "Invalid --tx-vlan value: " << tx_vlan_str << "\n";
                return 1;
            }
        }

        if (!rx_vlan_str.empty()) {
            if (rx_vlan_str == "any") {
                rx_any = true;
            } else {
                // Parse single VID or range "start-end"
                size_t dash = rx_vlan_str.find('-');
                try {
                    if (dash == std::string::npos) {
                        int v = std::stoi(rx_vlan_str);
                        if (v < 1 || v > 4095) {
                            std::cerr << "--rx-vlan must be in range 1–4095\n";
                            return 1;
                        }
                        rx_range = EtherCAT::VLANRouter::VLANRange{
                            static_cast<uint16_t>(v), static_cast<uint16_t>(v)};
                    } else {
                        int start = std::stoi(rx_vlan_str.substr(0, dash));
                        int end   = std::stoi(rx_vlan_str.substr(dash + 1));
                        if (start < 1 || end > 4095 || start > end) {
                            std::cerr << "--rx-vlan range must be 1–4095 with start <= end\n";
                            return 1;
                        }
                        rx_range = EtherCAT::VLANRouter::VLANRange{
                            static_cast<uint16_t>(start), static_cast<uint16_t>(end)};
                    }
                } catch (...) {
                    std::cerr << "Invalid --rx-vlan value: " << rx_vlan_str << "\n";
                    return 1;
                }
            }
        }
    }

    TETHER_LOGI(TAG, "axia80_stream (host) — interface: %s, raw: %s, slave: %d",
                iface.c_str(), raw_mode ? "yes" : "no", slave_idx);
    if (!debug_flags.empty()) {
        TETHER_LOGI(TAG, "Debug flags: %s", debug_str.c_str());
    }
    if (vlan_mode) {
        if (rx_any) {
            TETHER_LOGI(TAG, "VLAN mode: RX=any (undefined target), TX=%s",
                        tx_vlan ? std::to_string(*tx_vlan).c_str() : "none");
        } else if (rx_range) {
            TETHER_LOGI(TAG, "VLAN mode: RX=%u-%u, TX=%s",
                        rx_range->start, rx_range->end,
                        tx_vlan ? std::to_string(*tx_vlan).c_str() : "none");
        } else {
            TETHER_LOGI(TAG, "VLAN mode: RX=untagged, TX=%s",
                        tx_vlan ? std::to_string(*tx_vlan).c_str() : "none");
        }
    }

    // ---- Parse and verify against ESI XML (intended settings) ----
    std::optional<ESI::DeviceInfo> esi_device;
    if (!esi_xml_path.empty()) {
        std::vector<ESI::DeviceInfo> devices;
        std::string err;
        if (ESI::parseESIFile(esi_xml_path, devices, err)) {
            if (!devices.empty()) {
                esi_device = devices[0];
                TETHER_LOGI(TAG, "Parsed ESI XML: %s", esi_xml_path.c_str());
                verifyIntendedAgainstESI(*esi_device);
            } else {
                TETHER_LOGW(TAG, "ESI XML parsed but no devices found: %s", esi_xml_path.c_str());
            }
        } else {
            TETHER_LOGW(TAG, "Failed to parse ESI XML '%s': %s", esi_xml_path.c_str(), err.c_str());
        }
    }

    // ---- Install signal handler ----
    std::signal(SIGINT, signalHandler);
    std::signal(SIGTERM, signalHandler);

    // ---- Open raw socket ----
    auto eth = EtherCAT::HAL::createDefaultEthernet();
    if (!eth) { TETHER_LOGE(TAG, "No Ethernet HAL available"); return 1; }

    EtherCAT::HAL::EthernetConfig cfg;
    cfg.interfaceName = iface.c_str();
    cfg.promiscuous   = true;
    cfg.ethertypeFilter = static_cast<uint16_t>(EtherCAT::kEtherTypeEtherCAT);

    {
        auto err = eth->init(cfg);
        if (err != EtherCAT::HAL::Error::OK) {
            if (err == EtherCAT::HAL::Error::InterfaceNotFound)
                TETHER_LOGE(TAG, "Interface '%s' not found", iface.c_str());
            else if (err == EtherCAT::HAL::Error::PermissionDenied)
                TETHER_LOGE(TAG, "Permission denied — run as root or with CAP_NET_RAW");
            else
                TETHER_LOGE(TAG, "Failed to init '%s' (%s)", iface.c_str(),
                            magic_enum::enum_name(err).data());
            return 2;
        }

        auto ls = eth->getLinkStatus();
        if (!ls.up) {
            TETHER_LOGE(TAG, "Link DOWN on '%s'", iface.c_str());
            return 6;
        }
    }

    // ---- MAC address ----
    EtherCAT::HAL::MacAddress mac;
    if (eth->getMacAddress(mac) != EtherCAT::HAL::Error::OK) {
        TETHER_LOGE(TAG, "Failed to read MAC address");
        return 3;
    }
    uint8_t src_mac[6];
    std::memcpy(src_mac, mac.bytes, 6);

    // ---- NetworkInterface wrapper ----
    auto ni_ptr = std::make_unique<EtherCAT::NetworkInterface>();
    ni_ptr->send = [eth_raw = eth.get()](const uint8_t* data, size_t len) -> bool {
        return eth_raw->transmit(data, len) == EtherCAT::HAL::Error::OK;
    };
    EtherCAT::Raw::set_network_interface(ni_ptr.get());
    EtherCAT::Raw::set_src_mac(src_mac);

    // ---- Create master ----
    EtherCAT::EtherCATMaster master;
    g_master = &master;

    // ---- Optional VLAN router ----
    std::unique_ptr<EtherCAT::VLANRouter> router;
    if (vlan_mode) {
        router = std::make_unique<EtherCAT::VLANRouter>();
        router->setBackend(ni_ptr.get());
        if (rx_any) {
            router->setUndefinedTarget(
                std::shared_ptr<EtherCAT::EtherCATMaster>(&master, [](auto*){}),
                tx_vlan, true);
        } else if (rx_range) {
            router->addMaster(
                std::shared_ptr<EtherCAT::EtherCATMaster>(&master, [](auto*){}),
                *rx_range, tx_vlan);
        } else {
            router->addMaster(
                std::shared_ptr<EtherCAT::EtherCATMaster>(&master, [](auto*){}),
                std::nullopt, tx_vlan);
        }
    }

    // Route RX frames
    if (router) {
        eth->setRxCallback([&router](const uint8_t* frame, size_t len,
                                      const EtherCAT::HAL::RxFrameInfo&, void*) {
            router->processRxFrame(frame, len);
        }, nullptr);
    } else {
        eth->setRxCallback([&master](const uint8_t* frame, size_t len,
                                      const EtherCAT::HAL::RxFrameInfo&, void*) {
            master.handleRxFrame(frame, len);
        }, nullptr);
    }

    // ---- Poll thread ----
    std::thread poll_thread([&]() {
        if (!Tether::Platform::setCurrentThreadRealtime(-1)) {
            TETHER_LOGW(TAG, "poll_thread: could not set realtime scheduling");
        }
        while (!master.isCancelRequested()) { eth->poll(1); }
    });

    // ---- Start master ----
    if (router) {
        EtherCAT::NetworkInterface* master_iface = rx_any
            ? router->undefinedNetworkInterface()
            : router->networkInterfaceFor(&master);
        if (!master_iface) {
            TETHER_LOGE(TAG, "Failed to obtain per-master NetworkInterface from VLAN router");
            return 5;
        }
        master.start(*master_iface, src_mac);
    } else {
        master.start(*EtherCAT::Raw::network_interface(), src_mac);
    }

    // ---- Discover slaves ----
    if (!master.discoverSlaves()) {
        TETHER_LOGE(TAG, "No slaves found — check wiring and power");
        master.stop();
        master.requestCancel();
        poll_thread.join();
        eth->shutdown();
        return 4;
    }

    uint16_t slaves = master.getDiscoveredSlaveCount();
    TETHER_LOGI(TAG, "Discovered %u slave(s)", slaves);
    master.logDiscoveredSlavesSummary(TAG);

    if (static_cast<uint16_t>(slave_idx) >= slaves) {
        TETHER_LOGE(TAG, "Slave index %d out of range (max %u)", slave_idx, slaves - 1);
        master.stop();
        master.requestCancel();
        poll_thread.join();
        eth->shutdown();
        return 5;
    }

    // ---- Declare calibration data early (needed by motion callback) ----
    EtherCAT::Sensors::Axia80::CalibrationData cal;
    std::atomic<bool> have_cal{false};

    // ---- Initialise Axia80 sensor ----
    EtherCAT::Sensors::Axia80Sensor sensor(master, static_cast<uint16_t>(slave_idx));

    if (!sensor.isAxia80Device()) {
        TETHER_LOGW(TAG, "Slave %d does not appear to be an Axia80 (wrong VID/PID)", slave_idx);
    }

    // Initialise up to SAFE-OP (do not transition to OP yet)
    if (!sensor.init(Tether::Platform::LogLevel::Info, false)) {
        TETHER_LOGE(TAG, "Failed to initialise Axia80 sensor");
        master.stop();
        master.requestCancel();
        poll_thread.join();
        eth->shutdown();
        return 7;
    }

    // ---- Read calibration data via SDO (before RT loop so first frames are converted) ----
    if (sensor.readCalibrationData(cal)) {
        have_cal.store(true, std::memory_order_release);
        TETHER_LOGI(TAG, "Calibration loaded: serial=%s, force=%s, torque=%s, cpf=%u, cpt=%u",
                    cal.ft_serial,
                    Axia80::forceUnitsToString(cal.force_units),
                    Axia80::torqueUnitsToString(cal.torque_units),
                    cal.counts_per_force, cal.counts_per_torque);
    } else {
        TETHER_LOGW(TAG, "Could not read calibration data — using raw counts");
        raw_mode = true;
    }

    // ---- Set up RT motion loop callback ----
    master.setMotionControlCallback(
        [&master, &sensor, &cal, &have_cal, raw_mode](double /*dt_seconds*/) -> bool {
            if (!master.pdo().exchangeAll()) {
                return true; // keep loop running even on transient errors
            }

            auto* tx = sensor.txPDO();
            if (!tx) return true;

            double fx = 0, fy = 0, fz = 0, tx_val = 0, ty = 0, tz = 0;
            bool cal_ok = have_cal.load(std::memory_order_acquire);

            if (raw_mode || !cal_ok || cal.counts_per_force == 0 || cal.counts_per_torque == 0) {
                fx = static_cast<double>(tx->fx);
                fy = static_cast<double>(tx->fy);
                fz = static_cast<double>(tx->fz);
                tx_val = static_cast<double>(tx->tx);
                ty = static_cast<double>(tx->ty);
                tz = static_cast<double>(tx->tz);
            } else {
                int32_t raw[6] = { tx->fx, tx->fy, tx->fz, tx->tx, tx->ty, tx->tz };
                double out[6] = {};
                EtherCAT::Sensors::Axia80Sensor::convertWrench(
                    raw, out, cal.counts_per_force, cal.counts_per_torque);
                fx = out[0]; fy = out[1]; fz = out[2];
                tx_val = out[3]; ty = out[4]; tz = out[5];
            }

            {
                std::lock_guard<std::mutex> lock(queue_mtx);
                data_queue.push({fx, fy, fz, tx_val, ty, tz, tx->status, tx->counter});
            }
            queue_cv.notify_one();
            return true;
        });

    EtherCAT::EtherCATMaster::RealtimeMotionLoopConfig loop_config;
    loop_config.cycle_period_us = 1000;          // 1 kHz
    loop_config.sync_interval_cycles = 10;
    loop_config.enable_dc_synchronization = false; // Axia80 does not use DC

    // Start motion loop while still in SAFE-OP so PDO exchange is active
    // during the OP transition
    if (!master.startRealtimeMotionControlLoop(loop_config)) {
        TETHER_LOGE(TAG, "Failed to start realtime motion loop");
        master.stop();
        master.requestCancel();
        poll_thread.join();
        eth->shutdown();
        return 8;
    }

    // ---- Transition to OP while PDO exchange is already running ----
    if (sensor.slave().transitionToOp() != EtherCAT::SlaveError::Ok) {
        TETHER_LOGE(TAG, "Slave %d: OP transition failed", slave_idx);
        master.stopMotionControlLoop();
        master.stop();
        master.requestCancel();
        poll_thread.join();
        eth->shutdown();
        return 7;
    }

    // Verify slave actually reached OP state
    EtherCAT::SlaveState actual_state;
    if (sensor.slave().readState(actual_state) != EtherCAT::SlaveError::Ok) {
        TETHER_LOGE(TAG, "Failed to read slave state after OP transition");
        master.stopMotionControlLoop();
        master.stop();
        master.requestCancel();
        poll_thread.join();
        eth->shutdown();
        return 7;
    }
    if (actual_state != EtherCAT::SlaveState::OP) {
        TETHER_LOGE(TAG, "Slave %d is not in OP (actual: %s)", slave_idx,
                    magic_enum::enum_name(actual_state).data());
        master.stopMotionControlLoop();
        master.stop();
        master.requestCancel();
        poll_thread.join();
        eth->shutdown();
        return 7;
    }

    // ---- FMMU debug output ----
    if (fmmu_debug) {
        sensor.slave().fmmuManager().logConfig(TAG);
        sensor.slave().fmmuManager().logHardware(TAG);
    }

    // ---- Readback verification against ESI XML ----
    if (esi_device.has_value()) {
        verifyReadbackAgainstESI(*esi_device, master, sensor, static_cast<uint16_t>(slave_idx));
    }

    // ---- Read product info ----
    EtherCAT::Sensors::Axia80::ProductDescription desc;
    if (sensor.readProductDescription(desc)) {
        TETHER_LOGI(TAG, "Device: %s (SN: %u)", desc.product_name, desc.product_serial_number);
    }

    // ---- Configure sensor ----
    sensor.setConfiguration(
        EtherCAT::Sensors::Axia80::FilterType::FILTER_3,
        EtherCAT::Sensors::Axia80::CalibrationSlot::SLOT_0,
        EtherCAT::Sensors::Axia80::SampleRate::RATE_1953_HZ);
    // Motion loop will transmit RxPDO values automatically

    // ---- Set bias (tare) ----
    TETHER_LOGI(TAG, "Setting bias (tare)...");
    sensor.setBias();
    // Allow the motion loop to send the bias bit for several cycles
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    // Clear bias bit so it doesn't stay set
    if (auto* pdo = sensor.rxPDO()) {
        pdo->control1 &= ~EtherCAT::Sensors::Axia80::CTRL_BIAS_BIT;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    // ---- Consumer thread: print from queue ----
    std::atomic<uint64_t> cycle_count{0};

    std::string unit_label;
    if (raw_mode) {
        unit_label = "raw (counts)";
    } else {
        unit_label = std::string(Axia80::forceUnitsToString(cal.force_units))
                     + " / " + Axia80::torqueUnitsToString(cal.torque_units);
    }
    TETHER_LOGI(TAG, "Streaming %s data... Press Ctrl-C to stop", unit_label.c_str());

    std::cout << std::fixed << std::setprecision(4);
    const char* force_unit = raw_mode ? "counts" : Axia80::forceUnitsToString(cal.force_units);
    const char* torque_unit = raw_mode ? "counts" : Axia80::torqueUnitsToString(cal.torque_units);
    // Print header based on selected columns
    for (const auto& col : selected_columns) {
        std::string header;
        if (col == "fx" || col == "fy" || col == "fz") {
            header = std::string(1, std::toupper(col[0])) + col.substr(1) + "(" + force_unit + ")";
        } else if (col == "tx" || col == "ty" || col == "tz") {
            header = std::string(1, std::toupper(col[0])) + col.substr(1) + "(" + torque_unit + ")";
        } else {
            header = col;
            header[0] = std::toupper(header[0]);
        }
        std::cout << std::setw(18) << header;
    }
    std::cout << "\n";
    std::cout << std::string(selected_columns.size() * 18, '-') << "\n";

    std::thread consumer_thread([&]() {
        auto start_time = std::chrono::steady_clock::now();
        auto end_time = start_time + std::chrono::seconds(static_cast<int>(duration_sec));

        while (!master.isCancelRequested()) {
            if (duration_sec > 0 && std::chrono::steady_clock::now() >= end_time) {
                master.requestCancel();
                break;
            }

            std::unique_lock<std::mutex> lock(queue_mtx);
            if (!queue_cv.wait_for(lock, std::chrono::milliseconds(100),
                                   [&master]() { return !data_queue.empty() || master.isCancelRequested(); })) {
                continue;
            }
            if (data_queue.empty()) continue;

            auto frame = data_queue.front();
            data_queue.pop();
            lock.unlock();

            // Print only selected columns
            for (const auto& col : selected_columns) {
                char cell[32];
                if (col == "fx") {
                    snprintf(cell, sizeof(cell), "%.4f%s", frame.fx, force_unit);
                    std::cout << std::setw(18) << cell;
                } else if (col == "fy") {
                    snprintf(cell, sizeof(cell), "%.4f%s", frame.fy, force_unit);
                    std::cout << std::setw(18) << cell;
                } else if (col == "fz") {
                    snprintf(cell, sizeof(cell), "%.4f%s", frame.fz, force_unit);
                    std::cout << std::setw(18) << cell;
                } else if (col == "tx") {
                    snprintf(cell, sizeof(cell), "%.4f%s", frame.tx_val, torque_unit);
                    std::cout << std::setw(18) << cell;
                } else if (col == "ty") {
                    snprintf(cell, sizeof(cell), "%.4f%s", frame.ty, torque_unit);
                    std::cout << std::setw(18) << cell;
                } else if (col == "tz") {
                    snprintf(cell, sizeof(cell), "%.4f%s", frame.tz, torque_unit);
                    std::cout << std::setw(18) << cell;
                } else if (col == "status") {
                    snprintf(cell, sizeof(cell), "0x%08X", frame.status);
                    std::cout << std::setw(18) << cell;
                } else if (col == "counter") {
                    std::cout << std::setw(18) << frame.counter;
                }
            }
            std::cout << "\n";

            if (EtherCAT::Sensors::Axia80Sensor::hasError(frame.status)) {
                std::cout << "  [ERROR] status=0x" << std::hex << frame.status << std::dec << "\n";
            }

            cycle_count++;
        }
    });

    // Wait for Ctrl-C or timeout
    while (!master.isCancelRequested()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    queue_cv.notify_all();
    consumer_thread.join();

    TETHER_LOGI(TAG, "Streamed %lu cycles", cycle_count.load());

    // ---- Cleanup ----
    master.stopMotionControlLoop();
    master.stop();
    master.requestCancel();
    poll_thread.join();
    eth->shutdown();
    g_master = nullptr;

    return 0;
}

#endif // UNIT_TEST_HOST

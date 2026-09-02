/**
 * @file kinco_rp20_io.cpp
 * @brief Kinco RP20 I/O module initialization and live I/O demo
 *
 * Auto-discovers RP20 I/O modules across all discovered EtherCAT slaves
 * and slots 0–15, initializes them via CoE init commands, configures PDOs,
 * transitions to OP, and runs a cyclic loop reading/writing PDO data.
 *
 * Usage (Linux, requires root or CAP_NET_RAW):
 *   ./kinco_rp20_io              # uses eth0
 *   ./kinco_rp20_io -i enp3s0  # specify interface
 *   ./kinco_rp20_io -t 10       # run for 10 seconds
 *   ./kinco_rp20_io --interactive  # ncurses UI for toggling outputs
 *   ./kinco_rp20_io --tc-type K    # set thermocouple type to K for all TC channels
 *   ./kinco_rp20_io --tc-cjc internal  # set cold junction compensation to internal
 *   ./kinco_rp20_io --tc-filter none    # disable TC filtering
 *   ./kinco_rp20_io --ai-type 4-20mA   # set AI signal form to 4-20mA
 *   ./kinco_rp20_io --rd-type PT100    # set RTD type to PT100
 */

#include <atomic>
#include <chrono>
#include <cstring>
#include <iomanip>
#include <iostream>
#include <optional>
#include <string>
#include <unordered_map>
#include <vector>

#include <magic_enum/magic_enum.hpp>

#ifdef HAVE_NCURSES
#include <ncurses.h>
#undef OK
#undef ERR
#endif

#include "tether/ethercat/Master.hpp"
#include "tether/ethercat/Slave.hpp"
#include "tether/ethercat/Types.hpp"
#include "tether/ethercat/SyncManager.hpp"
#include "tether/ethercat/PDOManager.hpp"
#include "tether/ethercat/CoEManager.hpp"
#include "tether/ethercat/ESIFile.hpp"
#include "tether/platform/EspCompat.hpp"
#include "tether/platform/Platform.hpp"

#include "tether/drives/RP20/RP20Module.hpp"
#include "tether/drives/RP20/RP20ModuleConfig.hpp"
#include "tether/drives/RP20/RP20PDO.hpp"
#include "tether/drives/RP20/RP20Registers.hpp"
#include "tether/utils/SignalHandler.hpp"

#include "common/ExampleHelpers.hpp"
#include "common/EtherCATHostSetup.hpp"

static const char* TAG = "kinco_rp20_io";
static EtherCAT::Master* g_master = nullptr;
static std::atomic<bool> g_running{true};

namespace RP20Mod = ::EtherCAT::Drives::RP20Module;
namespace RP20Cfg = ::EtherCAT::Drives::RP20Config;
namespace RP20Reg = ::EtherCAT::Drives::Registers::RP20;
namespace RP20Pdo = ::EtherCAT::Drives::RP20_pdo;

// ============================================================================
// Discovered module info
// ============================================================================

struct DiscoveredModule {
    uint16_t slave_index;
    uint8_t  slot;
    const RP20Mod::ModuleDescriptor* descriptor;
    std::vector<uint8_t> tx_buffer;
    std::vector<uint8_t> rx_buffer;
    int tx_pdo_entry = -1;
    int rx_pdo_entry = -1;
};

// ============================================================================
// Slot scanning
// ============================================================================

static std::vector<DiscoveredModule> scanSlots(EtherCAT::Master& master,
                                               uint16_t slave_count) {
    std::vector<DiscoveredModule> modules;

    for (uint16_t s = 0; s < slave_count; ++s) {
        for (uint8_t slot = 0; slot < 16; ++slot) {
            uint16_t diag_idx = RP20Mod::diagnosisIndexForSlot(slot);
            // The RP20 diagnosis entry is a 4-byte OD object whose low byte
            // holds the module ID; readU8 returns the relevant byte and the
            // trailing 3 bytes are expected padding.
            EtherCAT::CoE::CoETransactionOptions opts;
            opts.allow_trailing_bytes = true;
            auto id_res = master.sdoManager(s).readU8(diag_idx, 0x01, opts);
            if (!id_res.has_value()) {
                continue;
            }
            uint8_t module_id = id_res.value();
            if (module_id == 0) {
                continue;
            }

            const RP20Mod::ModuleDescriptor* desc = RP20Mod::findByIdent(module_id);
            if (!desc) {
                TETHER_LOGW(TAG, "{} slot {}: unknown module ID 0x{:02X}, skipping",
                            master.slaveLogPrefix(s).c_str(), slot, module_id);
                continue;
            }

            TETHER_LOGI(TAG, "{} slot {}: found {} ({}, ident=0x{:02X})",
                        master.slaveLogPrefix(s).c_str(), slot, desc->name, desc->module_class, module_id);

            DiscoveredModule mod;
            mod.slave_index = s;
            mod.slot = slot;
            mod.descriptor = desc;

            if (desc->has_txpdo && desc->txpdo) {
                mod.tx_buffer.assign(desc->txpdo->size, 0);
            }
            if (desc->has_rxpdo && desc->rxpdo) {
                mod.rx_buffer.assign(desc->rxpdo->size, 0);
            }

            modules.push_back(std::move(mod));
        }
    }

    return modules;
}

// ============================================================================
// Module configuration (TC, AI, RD)
// ============================================================================

using RP20Reg::TCSignalForm;
using RP20Reg::AISignalForm;
using RP20Reg::RTDSignalForm;
using RP20Reg::FilteringMode;
using RP20Reg::ColdJunctionCompensation;

struct ModuleConfig {
    // TC options
    std::optional<TCSignalForm> tc_signal_form;
    std::optional<FilteringMode> tc_filter;
    std::optional<ColdJunctionCompensation> tc_cjc;
    // AI options (also used for Mixed_AIO AI channels)
    std::optional<AISignalForm> ai_signal_form;
    std::optional<FilteringMode> ai_filter;
    // RD options
    std::optional<RTDSignalForm> rd_signal_form;
    std::optional<FilteringMode> rd_filter;
};

// --- Parsing helpers ---

static std::optional<TCSignalForm> parseTcSignalForm(const std::string& s) {
    if (s.empty() || s == "-1")
        return std::nullopt;
    if (auto v = magic_enum::enum_cast<TCSignalForm>(s))
        return v;
    static const std::unordered_map<std::string, TCSignalForm> aliases = {
        {"J", TCSignalForm::Type_J}, {"K", TCSignalForm::Type_K},
        {"E", TCSignalForm::Type_E}, {"S", TCSignalForm::Type_S},
        {"T", TCSignalForm::Type_T}, {"100mV", TCSignalForm::Voltage_100mV},
    };
    auto it = aliases.find(s);
    if (it != aliases.end()) return it->second;
    try {
        int val = std::stoi(s);
        if (val >= 0 && val <= 5) return static_cast<TCSignalForm>(val);
    } catch (...) {}
    std::cerr << "Invalid --tc-type: " << s
              << " (expected J, K, E, S, T, 100mV, or 0-5)\n";
    return std::nullopt;
}

static std::optional<AISignalForm> parseAiSignalForm(const std::string& s) {
    if (s.empty() || s == "-1")
        return std::nullopt;
    if (auto v = magic_enum::enum_cast<AISignalForm>(s))
        return v;
    static const std::unordered_map<std::string, AISignalForm> aliases = {
        {"4-20mA", AISignalForm::Current_4_20mA},
        {"20mA", AISignalForm::Current_20mA},
        {"1-5V", AISignalForm::Voltage_1_5V},
        {"10V", AISignalForm::Voltage_10V},
    };
    auto it = aliases.find(s);
    if (it != aliases.end()) return it->second;
    try {
        int val = std::stoi(s);
        if (val >= 0 && val <= 3) return static_cast<AISignalForm>(val);
    } catch (...) {}
    std::cerr << "Invalid --ai-type: " << s
              << " (expected 4-20mA, 20mA, 1-5V, 10V, or 0-3)\n";
    return std::nullopt;
}

static std::optional<RTDSignalForm> parseRdSignalForm(const std::string& s) {
    if (s.empty() || s == "-1")
        return std::nullopt;
    if (auto v = magic_enum::enum_cast<RTDSignalForm>(s))
        return v;
    static const std::unordered_map<std::string, RTDSignalForm> aliases = {
        {"PT100", RTDSignalForm::PT100}, {"PT1000", RTDSignalForm::PT1000},
        {"Cu50", RTDSignalForm::Cu50}, {"Cu100", RTDSignalForm::Cu100},
    };
    auto it = aliases.find(s);
    if (it != aliases.end()) return it->second;
    try {
        int val = std::stoi(s);
        if (val == 0 || val == 1 || val == 4 || val == 5)
            return static_cast<RTDSignalForm>(val);
    } catch (...) {}
    std::cerr << "Invalid --rd-type: " << s
              << " (expected PT100, PT1000, Cu50, Cu100, or 0/1/4/5)\n";
    return std::nullopt;
}

static std::optional<FilteringMode> parseFilteringMode(const std::string& s) {
    if (s.empty() || s == "-1")
        return std::nullopt;
    if (auto v = magic_enum::enum_cast<FilteringMode>(s))
        return v;
    if (s == "none" || s == "0") return FilteringMode::None;
    if (s == "average" || s == "1") return FilteringMode::Average;
    std::cerr << "Invalid filter mode: " << s
              << " (expected none, average, or 0-1)\n";
    return std::nullopt;
}

static std::optional<ColdJunctionCompensation>
parseColdJunctionCompensation(const std::string& s) {
    if (s.empty() || s == "-1")
        return std::nullopt;
    if (auto v = magic_enum::enum_cast<ColdJunctionCompensation>(s))
        return v;
    if (s == "internal" || s == "0") return ColdJunctionCompensation::Internal;
    if (s == "external" || s == "1") return ColdJunctionCompensation::External;
    std::cerr << "Invalid --tc-cjc: " << s
              << " (expected internal, external, or 0-1)\n";
    return std::nullopt;
}

// --- Argument registration ---

static void addModuleArguments(argparse::ArgumentParser& program) {
    program.add_argument("--tc-type")
        .default_value(std::string("K"))
        .help("Thermocouple type for all TC channels: "
              "J, K, E, S, T, 100mV (or 0-5). Default: K");
    program.add_argument("--tc-filter")
        .default_value(std::string("none"))
        .help("Filtering mode for all TC channels: "
              "none, average (or 0-1). Default: none");
    program.add_argument("--tc-cjc")
        .default_value(std::string("internal"))
        .help("Cold junction compensation for all TC channels: "
              "internal, external (or 0-1). Default: internal");
    program.add_argument("--ai-type")
        .default_value(std::string("4-20mA"))
        .help("Signal form for all AI channels: "
              "4-20mA, 20mA, 1-5V, 10V (or 0-3). Default: 4-20mA");
    program.add_argument("--ai-filter")
        .default_value(std::string("none"))
        .help("Filtering mode for all AI channels: "
              "none, average (or 0-1). Default: none");
    program.add_argument("--rd-type")
        .default_value(std::string("PT100"))
        .help("RTD type for all RD channels: "
              "PT100, PT1000, Cu50, Cu100 (or 0/1/4/5). Default: PT100");
    program.add_argument("--rd-filter")
        .default_value(std::string("none"))
        .help("Filtering mode for all RD channels: "
              "none, average (or 0-1). Default: none");
}

static ModuleConfig parseModuleConfig(const argparse::ArgumentParser& program) {
    ModuleConfig config;
    config.tc_signal_form = parseTcSignalForm(program.get<std::string>("--tc-type"));
    config.tc_filter = parseFilteringMode(program.get<std::string>("--tc-filter"));
    config.tc_cjc = parseColdJunctionCompensation(program.get<std::string>("--tc-cjc"));
    config.ai_signal_form = parseAiSignalForm(program.get<std::string>("--ai-type"));
    config.ai_filter = parseFilteringMode(program.get<std::string>("--ai-filter"));
    config.rd_signal_form = parseRdSignalForm(program.get<std::string>("--rd-type"));
    config.rd_filter = parseFilteringMode(program.get<std::string>("--rd-filter"));
    return config;
}

// --- Config label formatting ---

static std::string formatTcConfigLabel(const ModuleConfig& config) {
    std::string label = "[";
    label += config.tc_signal_form
        ? std::string(magic_enum::enum_name(*config.tc_signal_form)) : "default";
    label += ", ";
    label += config.tc_filter
        ? std::string(magic_enum::enum_name(*config.tc_filter)) : "default";
    label += " filter, ";
    label += config.tc_cjc
        ? std::string(magic_enum::enum_name(*config.tc_cjc)) : "default";
    label += " CJC]";
    return label;
}

static std::string formatAiConfigLabel(const ModuleConfig& config) {
    std::string label = "[";
    label += config.ai_signal_form
        ? std::string(magic_enum::enum_name(*config.ai_signal_form)) : "default";
    label += ", ";
    label += config.ai_filter
        ? std::string(magic_enum::enum_name(*config.ai_filter)) : "default";
    label += " filter]";
    return label;
}

static std::string formatRdConfigLabel(const ModuleConfig& config) {
    std::string label = "[";
    label += config.rd_signal_form
        ? std::string(magic_enum::enum_name(*config.rd_signal_form)) : "default";
    label += ", ";
    label += config.rd_filter
        ? std::string(magic_enum::enum_name(*config.rd_filter)) : "default";
    label += " filter]";
    return label;
}

// ============================================================================
// Apply module configuration via SDO
// ============================================================================

static bool applyModuleConfig(EtherCAT::Master& master,
                              const DiscoveredModule& mod,
                              const ModuleConfig& config) {
    const auto type = mod.descriptor->type;
    const bool is_tc = (type == RP20Mod::ModuleType::TC_4);
    const bool is_ai = (type == RP20Mod::ModuleType::AI_4 ||
                        type == RP20Mod::ModuleType::Mixed_AIO);
    const bool is_rd = (type == RP20Mod::ModuleType::RD_4);
    if (!is_tc && !is_ai && !is_rd)
        return true;

    auto& sdo = master.sdoManager(mod.slave_index);
    uint16_t cfg_idx = RP20Mod::configIndexForSlot(mod.slot);
    bool ok = true;

    // Number of analog input channels (Mixed_AIO has 2, others have 4)
    uint8_t ai_ch_count = (type == RP20Mod::ModuleType::Mixed_AIO) ? 2 : 4;

    auto writeChannels = [&](uint8_t start_sub, uint8_t ch_count,
                             uint8_t val, const char* what,
                             std::string_view name_sv) {
        for (uint8_t ch = 0; ch < ch_count; ++ch) {
            uint8_t sub = static_cast<uint8_t>(start_sub + ch);
            if (!sdo.writeU8(cfg_idx, sub, val).has_value()) {
                TETHER_LOGW(TAG, "Slave {} slot {}: failed to set {} CH{}={}",
                            mod.slave_index, mod.slot, what, ch,
                            name_sv);
                ok = false;
            } else {
                TETHER_LOGI(TAG, "Slave {} slot {}: {} CH{}={}",
                            mod.slave_index, mod.slot, what, ch,
                            name_sv);
            }
        }
    };

    if (is_tc) {
        if (config.tc_signal_form) {
            uint8_t val = static_cast<uint8_t>(*config.tc_signal_form);
            writeChannels(0x01, 4, val, "TC type",
                          magic_enum::enum_name(*config.tc_signal_form));
        }
        if (config.tc_filter) {
            uint8_t val = static_cast<uint8_t>(*config.tc_filter);
            writeChannels(0x05, 4, val, "TC filter",
                          magic_enum::enum_name(*config.tc_filter));
        }
        if (config.tc_cjc) {
            uint8_t val = static_cast<uint8_t>(*config.tc_cjc);
            writeChannels(0x09, 4, val, "TC CJC",
                          magic_enum::enum_name(*config.tc_cjc));
        }
    } else if (is_ai) {
        if (config.ai_signal_form) {
            uint8_t val = static_cast<uint8_t>(*config.ai_signal_form);
            writeChannels(0x01, ai_ch_count, val, "AI type",
                          magic_enum::enum_name(*config.ai_signal_form));
        }
        if (config.ai_filter) {
            uint8_t val = static_cast<uint8_t>(*config.ai_filter);
            // AI filter subindices: 0x05-0x08 for AI_4, 0x03-0x04 for Mixed_AIO
            uint8_t filter_sub = (type == RP20Mod::ModuleType::Mixed_AIO) ? 0x03 : 0x05;
            writeChannels(filter_sub, ai_ch_count, val, "AI filter",
                          magic_enum::enum_name(*config.ai_filter));
        }
    } else if (is_rd) {
        if (config.rd_signal_form) {
            uint8_t val = static_cast<uint8_t>(*config.rd_signal_form);
            writeChannels(0x01, 4, val, "RD type",
                          magic_enum::enum_name(*config.rd_signal_form));
        }
        if (config.rd_filter) {
            uint8_t val = static_cast<uint8_t>(*config.rd_filter);
            writeChannels(0x05, 4, val, "RD filter",
                          magic_enum::enum_name(*config.rd_filter));
        }
    }

    return ok;
}

static void applyModuleConfigToAll(EtherCAT::Master& master,
                                   const std::vector<DiscoveredModule>& modules,
                                   const ModuleConfig& config) {
    for (const auto& mod : modules) {
        applyModuleConfig(master, mod, config);
    }
}

static bool sendInitCommands(EtherCAT::Master& master,
                             const DiscoveredModule& mod) {
    auto& sdo = master.sdoManager(mod.slave_index);
    uint16_t cfg_idx = RP20Mod::configIndexForSlot(mod.slot);

    for (size_t i = 0; i < mod.descriptor->init_cmd_count; ++i) {
        const RP20Cfg::CoEInitCmd& cmd = mod.descriptor->init_cmds[i];
        uint16_t idx = cfg_idx;  // All init cmds use kConfigBaseIndex, slot offset applied

        bool ok = false;
        if (cmd.data_size == 1) {
            ok = sdo.writeU8(idx, cmd.subindex,
                             static_cast<uint8_t>(cmd.data)).has_value();
        } else if (cmd.data_size == 2) {
            ok = sdo.writeU16(idx, cmd.subindex,
                              static_cast<uint16_t>(cmd.data)).has_value();
        } else if (cmd.data_size == 4) {
            ok = sdo.writeU32(idx, cmd.subindex, cmd.data).has_value();
        } else {
            TETHER_LOGW(TAG, "Slave {} slot {}: init cmd {} has unsupported size {}",
                        mod.slave_index, mod.slot, i, cmd.data_size);
            continue;
        }

        if (!ok) {
            TETHER_LOGW(TAG, "Slave {} slot {}: init cmd 0x{:04X}:0x{:02X} failed ({})",
                        mod.slave_index, mod.slot, idx, cmd.subindex, cmd.comment);
        } else {
            TETHER_LOGI(TAG, "Slave {} slot {}: init 0x{:04X}:0x{:02X} = 0x{:X} ({})",
                        mod.slave_index, mod.slot, idx, cmd.subindex,
                        cmd.data, cmd.comment);
        }
    }

    return true;
}

// ============================================================================
// Register PDO buffers on master side
// ============================================================================

static bool registerPDOBuffers(EtherCAT::Master& master,
                               std::vector<DiscoveredModule>& modules) {
    auto& mapping = master.pdo().mapping();

    for (auto& mod : modules) {
        const auto* desc = mod.descriptor;
        uint16_t txpdo_idx = RP20Mod::slotPDOIndex(RP20Reg::kTxPDOBaseIndex, mod.slot);
        uint16_t rxpdo_idx = RP20Mod::slotPDOIndex(RP20Reg::kRxPDOBaseIndex, mod.slot);

        if (desc->has_txpdo && desc->txpdo && !mod.tx_buffer.empty()) {
            mod.tx_pdo_entry = mapping.add_txpdo(
                mod.slave_index, mod.tx_buffer.data(),
                desc->txpdo->size, txpdo_idx,
                EtherCAT::PDO::PDOAddressMode::Position);
            if (mod.tx_pdo_entry < 0) {
                TETHER_LOGE(TAG, "Slave {} slot {}: failed to register TxPDO 0x{:04X}",
                            mod.slave_index, mod.slot, txpdo_idx);
                return false;
            }
            TETHER_LOGI(TAG, "Slave {} slot {}: registered TxPDO 0x{:04X} ({} bytes, entry {})",
                        mod.slave_index, mod.slot, txpdo_idx,
                        desc->txpdo->size, mod.tx_pdo_entry);
        }

        if (desc->has_rxpdo && desc->rxpdo && !mod.rx_buffer.empty()) {
            mod.rx_pdo_entry = mapping.add_rxpdo(
                mod.slave_index, mod.rx_buffer.data(),
                desc->rxpdo->size, rxpdo_idx,
                EtherCAT::PDO::PDOAddressMode::Position);
            if (mod.rx_pdo_entry < 0) {
                TETHER_LOGE(TAG, "Slave {} slot {}: failed to register RxPDO 0x{:04X}",
                            mod.slave_index, mod.slot, rxpdo_idx);
                return false;
            }
            TETHER_LOGI(TAG, "Slave {} slot {}: registered RxPDO 0x{:04X} ({} bytes, entry {})",
                        mod.slave_index, mod.slot, rxpdo_idx,
                        desc->rxpdo->size, mod.rx_pdo_entry);
        }
    }

    return true;
}

// ============================================================================
// Write PDO assignments to slave via SDO
// ============================================================================

static bool assignPDOsToSlave(EtherCAT::Master& master,
                              const std::vector<DiscoveredModule>& modules,
                              uint16_t slave_index) {
    // Collect RxPDO and TxPDO indices for this slave
    std::vector<uint16_t> rxpdos, txpdos;
    for (const auto& mod : modules) {
        if (mod.slave_index != slave_index) continue;
        if (mod.descriptor->has_rxpdo && mod.descriptor->rxpdo) {
            rxpdos.push_back(RP20Mod::slotPDOIndex(RP20Reg::kRxPDOBaseIndex, mod.slot));
        }
        if (mod.descriptor->has_txpdo && mod.descriptor->txpdo) {
            txpdos.push_back(RP20Mod::slotPDOIndex(RP20Reg::kTxPDOBaseIndex, mod.slot));
        }
    }

    auto& sdo = master.sdoManager(slave_index);

    // Assign RxPDOs to SM2 (0x1C12)
    if (!rxpdos.empty()) {
        if (!sdo.writeU8(CiA301::SyncManager2PDOAssign, 0, 0).has_value()) {
            TETHER_LOGW(TAG, "{}: failed to clear SM2 PDO count", master.slaveLogPrefix(slave_index).c_str());
        }
        for (size_t i = 0; i < rxpdos.size(); ++i) {
            if (!sdo.writeU16(CiA301::SyncManager2PDOAssign,
                              static_cast<uint8_t>(i + 1), rxpdos[i]).has_value()) {
                TETHER_LOGW(TAG, "{}: failed to assign RxPDO 0x{:04X} to SM2",
                            master.slaveLogPrefix(slave_index).c_str(), rxpdos[i]);
            }
        }
        if (!sdo.writeU8(CiA301::SyncManager2PDOAssign, 0,
                         static_cast<uint8_t>(rxpdos.size())).has_value()) {
            TETHER_LOGW(TAG, "{}: failed to set SM2 PDO count", master.slaveLogPrefix(slave_index).c_str());
        }
        TETHER_LOGI(TAG, "{}: assigned {} RxPDO(s) to SM2", master.slaveLogPrefix(slave_index).c_str(), rxpdos.size());
    }

    // Assign TxPDOs to SM3 (0x1C13)
    if (!txpdos.empty()) {
        if (!sdo.writeU8(CiA301::SyncManager3PDOAssign, 0, 0).has_value()) {
            TETHER_LOGW(TAG, "{}: failed to clear SM3 PDO count", master.slaveLogPrefix(slave_index).c_str());
        }
        for (size_t i = 0; i < txpdos.size(); ++i) {
            if (!sdo.writeU16(CiA301::SyncManager3PDOAssign,
                              static_cast<uint8_t>(i + 1), txpdos[i]).has_value()) {
                TETHER_LOGW(TAG, "{}: failed to assign TxPDO 0x{:04X} to SM3",
                            master.slaveLogPrefix(slave_index).c_str(), txpdos[i]);
            }
        }
        if (!sdo.writeU8(CiA301::SyncManager3PDOAssign, 0,
                         static_cast<uint8_t>(txpdos.size())).has_value()) {
            TETHER_LOGW(TAG, "{}: failed to set SM3 PDO count", master.slaveLogPrefix(slave_index).c_str());
        }
        TETHER_LOGI(TAG, "{}: assigned {} TxPDO(s) to SM3", master.slaveLogPrefix(slave_index).c_str(), txpdos.size());
    }

    return true;
}

// ============================================================================
// Update SM2/SM3 lengths for total PDO sizes
// ============================================================================

static void updateSMLengths(EtherCAT::Master& master,
                            const std::vector<DiscoveredModule>& modules,
                            uint16_t slave_index) {
    uint16_t total_rx = 0, total_tx = 0;
    for (const auto& mod : modules) {
        if (mod.slave_index != slave_index) continue;
        if (mod.descriptor->has_rxpdo && mod.descriptor->rxpdo) {
            total_rx += mod.descriptor->rxpdo->size;
        }
        if (mod.descriptor->has_txpdo && mod.descriptor->txpdo) {
            total_tx += mod.descriptor->txpdo->size;
        }
    }

    auto* cfgs = master.pdo().slaveConfigs();
    if (!cfgs) return;

    if (total_rx > 0) {
        TETHER_LOGI(TAG, "{}: SM2 length: {} -> {}",
                    master.slaveLogPrefix(slave_index).c_str(), cfgs[slave_index].sm[2].length, total_rx);
        cfgs[slave_index].sm[2].length = total_rx;
        cfgs[slave_index].rxpdo_size = total_rx;
    }
    if (total_tx > 0) {
        TETHER_LOGI(TAG, "{}: SM3 length: {} -> {}",
                    master.slaveLogPrefix(slave_index).c_str(), cfgs[slave_index].sm[3].length, total_tx);
        cfgs[slave_index].sm[3].length = total_tx;
        cfgs[slave_index].txpdo_size = total_tx;
    }
}

// ============================================================================
// Interactive ncurses UI
// ============================================================================

#ifdef HAVE_NCURSES

struct OutputBit {
    size_t module_idx;
    size_t field_idx;
    uint8_t bit;
    const char* label;
};

static void initColors() {
    if (has_colors()) {
        start_color();
        use_default_colors();
        init_pair(1, COLOR_GREEN, -1);   // active ON
        init_pair(2, COLOR_WHITE, -1);   // inactive
        init_pair(3, COLOR_RED, -1);     // error/stale
        init_pair(4, COLOR_CYAN, -1);    // header
        init_pair(5, COLOR_YELLOW, -1);  // cursor
    }
}

static void runInteractiveUI(EtherCAT::Master& master,
                             std::vector<DiscoveredModule>& modules,
                             std::atomic<uint64_t>& cycle_count,
                             std::atomic<bool>& pdo_ok,
                             const ModuleConfig& mod_config) {
    // Build list of toggleable output bits across all modules
    std::vector<OutputBit> output_bits;
    for (size_t mi = 0; mi < modules.size(); ++mi) {
        auto& mod = modules[mi];
        const auto* desc = mod.descriptor;
        if (!desc->has_rxpdo || !desc->rxpdo || mod.rx_buffer.empty()) continue;

        switch (desc->type) {
            case RP20Mod::ModuleType::DO_16_PNP:
            case RP20Mod::ModuleType::DO_16_NPN:
            case RP20Mod::ModuleType::Multi_DIO_8:
            case RP20Mod::ModuleType::DR_8: {
                for (size_t fi = 0; fi < desc->rxpdo->field_count; ++fi) {
                    const auto* f = RP20Mod::getFieldByChannel(*desc->rxpdo, fi);
                    if (!f) continue;
                    uint8_t nbits = f->size * 8;
                    for (uint8_t b = 0; b < nbits; ++b) {
                        char buf[64];
                        std::snprintf(buf, sizeof(buf), "%s DO.%zu.%u",
                                      desc->name, fi, b);
                        output_bits.push_back({mi, fi, b, ""});
                        output_bits.back().label = strdup(buf);
                    }
                }
                break;
            }
            default:
                break;
        }
    }

    if (output_bits.empty()) {
        TETHER_LOGW(TAG, "No toggleable digital outputs found for interactive mode");
        return;
    }

    // State: which output bits are ON
    std::vector<bool> bit_states(output_bits.size(), false);

    // Initialize ncurses
    setlocale(LC_ALL, "");
    initscr();
    noecho();
    cbreak();
    curs_set(0);
    nodelay(stdscr, TRUE);
    keypad(stdscr, TRUE);
    initColors();

    int cursor = 0;
    int scroll_offset = 0;
    const int max_rows = LINES - 8;

    auto applyOutputs = [&]() {
        for (size_t i = 0; i < output_bits.size(); ++i) {
            auto& ob = output_bits[i];
            auto& mod = modules[ob.module_idx];
            const auto* f = RP20Mod::getFieldByChannel(*mod.descriptor->rxpdo, ob.field_idx);
            if (!f) continue;
            std::span<uint8_t> rx(mod.rx_buffer.data(), mod.rx_buffer.size());
            RP20Mod::writeBit(rx, *f, ob.bit, bit_states[i]);
        }
    };

    applyOutputs();

    while (g_running.load()) {
        if (!pdo_ok.load()) {
            // Keep trying — the RT loop keeps exchanging
        }

        uint64_t cyc = cycle_count.load();

        clear();

        // Header
        attron(COLOR_PAIR(4) | A_BOLD);
        mvprintw(0, 0, "Kinco RP20 Interactive I/O  —  Cycle %llu  (%zu outputs)",
                 static_cast<unsigned long long>(cyc), output_bits.size());
        attroff(COLOR_PAIR(4) | A_BOLD);
        mvprintw(1, 0, "UP/DOWN: select  SPACE: toggle  a: all on  n: all off  q: quit");

        // Adjust scroll
        if (cursor < scroll_offset) scroll_offset = cursor;
        if (cursor >= scroll_offset + max_rows) scroll_offset = cursor - max_rows + 1;

        // Output section
        attron(A_BOLD);
        mvprintw(3, 0, "Outputs:");
        attroff(A_BOLD);

        int y = 4;
        int visible = 0;
        for (size_t i = 0; i < output_bits.size(); ++i) {
            if (static_cast<int>(i) < scroll_offset) continue;
            if (visible >= max_rows) break;
            visible++;

            auto& ob = output_bits[i];
            auto& mod = modules[ob.module_idx];

            bool is_cursor = static_cast<int>(i) == cursor;
            bool is_on = bit_states[i];

            if (is_cursor) attron(COLOR_PAIR(5) | A_BOLD);
            else if (is_on) attron(COLOR_PAIR(1));

            mvprintw(y, 2, "[%s] %s  (s%u/slot%u)",
                     is_on ? "ON " : "off",
                     ob.label,
                     mod.slave_index, mod.slot);

            if (is_cursor) attroff(COLOR_PAIR(5) | A_BOLD);
            else if (is_on) attroff(COLOR_PAIR(1));
            y++;
        }

        // Input section
        y++;
        attron(A_BOLD);
        mvprintw(y, 0, "Inputs:");
        attroff(A_BOLD);
        y++;

        for (auto& mod : modules) {
            const auto* desc = mod.descriptor;
            if (!desc->has_txpdo || !desc->txpdo || mod.tx_buffer.empty()) continue;

            std::span<const uint8_t> tx(mod.tx_buffer.data(), mod.tx_buffer.size());

            switch (desc->type) {
                case RP20Mod::ModuleType::DI_16:
                case RP20Mod::ModuleType::Multi_DIO_8: {
                    for (size_t fi = 0; fi < desc->txpdo->field_count; ++fi) {
                        const auto* f = RP20Mod::getFieldByChannel(*desc->txpdo, fi);
                        if (!f) continue;
                        uint8_t val = RP20Mod::readU8(tx, *f);
                        uint8_t nbits = f->size * 8;
                        mvprintw(y, 2, "%s DI.%zu: ", desc->name, fi);
                        int x = 2 + static_cast<int>(strlen(desc->name)) + 8;
                        for (uint8_t b = 0; b < nbits; ++b) {
                            bool on = (val >> b) & 1u;
                            if (on) attron(COLOR_PAIR(1));
                            mvprintw(y, x + b * 4, "%2d", b);
                            if (on) {
                                attron(A_BOLD);
                                mvprintw(y, x + b * 4 + 2, "1");
                                attroff(A_BOLD);
                                attroff(COLOR_PAIR(1));
                            } else {
                                mvprintw(y, x + b * 4 + 2, "0");
                            }
                        }
                        y++;
                    }
                    break;
                }
                case RP20Mod::ModuleType::TC_4: {
                    auto tc_label = formatTcConfigLabel(mod_config);
                    for (size_t fi = 0; fi < desc->txpdo->field_count; ++fi) {
                        const auto* f = RP20Mod::getFieldByChannel(*desc->txpdo, fi);
                        if (!f) continue;
                        int16_t val = RP20Mod::readI16(tx, *f);
                        mvprintw(y, 2, "%s %s TC.%zu: %6.1f C",
                                 desc->name, tc_label.c_str(), fi, val / 10.0);
                        y++;
                    }
                    break;
                }
                case RP20Mod::ModuleType::AI_4:
                case RP20Mod::ModuleType::RD_4:
                case RP20Mod::ModuleType::Mixed_AIO: {
                    const char* prefix = (desc->type == RP20Mod::ModuleType::RD_4) ? "RD" : "AI";
                    std::string label;
                    if (desc->type == RP20Mod::ModuleType::RD_4)
                        label = formatRdConfigLabel(mod_config);
                    else
                        label = formatAiConfigLabel(mod_config);
                    for (size_t fi = 0; fi < desc->txpdo->field_count; ++fi) {
                        const auto* f = RP20Mod::getFieldByChannel(*desc->txpdo, fi);
                        if (!f) continue;
                        int16_t val = RP20Mod::readI16(tx, *f);
                        mvprintw(y, 2, "%s %s %s.%zu: %d",
                                 desc->name, label.c_str(), prefix, fi, val);
                        y++;
                    }
                    break;
                }
                default:
                    break;
            }
        }

        // Footer
        if (!pdo_ok.load()) {
            attron(COLOR_PAIR(3));
            mvprintw(LINES - 1, 0, "[PDO EXCHANGE ERROR]");
            attroff(COLOR_PAIR(3));
        }

        refresh();

        // Handle input
        int ch = getch();
        if (ch == 'q' || ch == 27) {
            g_running.store(false);
            if (g_master) g_master->requestCancel();
            break;
        } else if (ch == KEY_UP || ch == 'k') {
            if (cursor > 0) cursor--;
        } else if (ch == KEY_DOWN || ch == 'j') {
            if (cursor < static_cast<int>(output_bits.size()) - 1) cursor++;
        } else if (ch == ' ' || ch == '\n') {
            bit_states[cursor] = !bit_states[cursor];
            applyOutputs();
        } else if (ch == 'a') {
            std::fill(bit_states.begin(), bit_states.end(), true);
            applyOutputs();
        } else if (ch == 'n') {
            std::fill(bit_states.begin(), bit_states.end(), false);
            applyOutputs();
        }

        Tether::Platform::Clock::instance().delayMilliseconds(20);
    }

    // Cleanup
    for (auto& ob : output_bits) {
        if (ob.label) free(const_cast<char*>(ob.label));
    }

    endwin();
}

#endif // HAVE_NCURSES

// ============================================================================
// Print module I/O data
// ============================================================================

static void printModuleIO(DiscoveredModule& mod, uint64_t cycle,
                          const ModuleConfig& mod_config) {
    const auto* desc = mod.descriptor;
    std::cout << "[" << desc->name << " s" << mod.slave_index
              << "/slot" << static_cast<int>(mod.slot) << "] ";

    if (desc->has_txpdo && desc->txpdo && !mod.tx_buffer.empty()) {
        std::span<const uint8_t> tx(mod.tx_buffer.data(), mod.tx_buffer.size());
        switch (desc->type) {
            case RP20Mod::ModuleType::DI_16:
            case RP20Mod::ModuleType::Multi_DIO_8: {
                // Digital inputs — print as hex
                std::cout << "DI:";
                for (size_t i = 0; i < desc->txpdo->field_count; ++i) {
                    const auto* f = RP20Mod::getFieldByChannel(*desc->txpdo, i);
                    if (f) {
                        uint8_t val = RP20Mod::readU8(tx, *f);
                        std::cout << " " << std::hex << std::setw(2)
                                  << std::setfill('0') << static_cast<int>(val);
                    }
                }
                std::cout << std::dec;
                break;
            }
            case RP20Mod::ModuleType::TC_4: {
                // Thermocouple inputs — value / 10 = °C
                auto tc_label = formatTcConfigLabel(mod_config);
                std::cout << tc_label << " TC:";
                for (size_t i = 0; i < desc->txpdo->field_count; ++i) {
                    const auto* f = RP20Mod::getFieldByChannel(*desc->txpdo, i);
                    if (f) {
                        int16_t val = RP20Mod::readI16(tx, *f);
                        std::cout << " " << std::fixed << std::setprecision(1)
                                  << std::setw(6) << (val / 10.0) << "C";
                    }
                }
                break;
            }
            case RP20Mod::ModuleType::AI_4:
            case RP20Mod::ModuleType::RD_4:
            case RP20Mod::ModuleType::Mixed_AIO: {
                const char* prefix = (desc->type == RP20Mod::ModuleType::RD_4) ? "RD" : "AI";
                std::string label;
                if (desc->type == RP20Mod::ModuleType::RD_4)
                    label = formatRdConfigLabel(mod_config);
                else
                    label = formatAiConfigLabel(mod_config);
                std::cout << label << " " << prefix << ":";
                for (size_t i = 0; i < desc->txpdo->field_count; ++i) {
                    const auto* f = RP20Mod::getFieldByChannel(*desc->txpdo, i);
                    if (f) {
                        int16_t val = RP20Mod::readI16(tx, *f);
                        std::cout << " " << val;
                    }
                }
                break;
            }
            default:
                break;
        }
    }

    if (desc->has_rxpdo && desc->rxpdo && !mod.rx_buffer.empty()) {
        std::span<uint8_t> rx(mod.rx_buffer.data(), mod.rx_buffer.size());
        switch (desc->type) {
            case RP20Mod::ModuleType::DO_16_PNP:
            case RP20Mod::ModuleType::DO_16_NPN:
            case RP20Mod::ModuleType::Multi_DIO_8:
            case RP20Mod::ModuleType::DR_8: {
                // Digital/relay outputs — display current state (no toggling)
                std::cout << " DO:";
                for (size_t i = 0; i < desc->rxpdo->field_count; ++i) {
                    const auto* f = RP20Mod::getFieldByChannel(*desc->rxpdo, i);
                    if (f) {
                        uint8_t val = RP20Mod::readU8(rx, *f);
                        std::cout << " " << std::hex << std::setw(2)
                                  << std::setfill('0') << static_cast<int>(val);
                    }
                }
                std::cout << std::dec;
                break;
            }
            case RP20Mod::ModuleType::AO_4:
            case RP20Mod::ModuleType::Mixed_AIO: {
                // Analog outputs — display current state (no sine wave)
                std::cout << " AO:";
                for (size_t i = 0; i < desc->rxpdo->field_count; ++i) {
                    const auto* f = RP20Mod::getFieldByChannel(*desc->rxpdo, i);
                    if (f) {
                        int16_t val = RP20Mod::readI16(rx, *f);
                        std::cout << " " << val;
                    }
                }
                break;
            }
            default:
                break;
        }
    }

    std::cout << "\n";
}

// ============================================================================
// Main
// ============================================================================

int main(int argc, char** argv) {
    argparse::ArgumentParser program("kinco_rp20_io");
    Tether::Examples::addInterfaceArg(program);
    Tether::Examples::addDebugArg(program);
    Tether::Examples::addVlanArgs(program);
    Tether::Examples::addMailboxSizeArg(program);
    Tether::Examples::addMailboxAddressArg(program);
    Tether::Examples::addEsiXmlArg(program);
    Tether::Examples::addDurationArg(program, 0.0);
    program.add_argument("--slot-scan-delay")
        .scan<'i', int>()
        .default_value(100)
        .help("Delay in ms between PRE-OP and slot scan (default: 100)");
    program.add_argument("--interactive")
        .default_value(false)
        .implicit_value(true)
        .help("Use ncurses interactive UI for toggling outputs");
    addModuleArguments(program);

    try { program.parse_args(argc, argv); }
    catch (const std::runtime_error& err) {
        std::cerr << err.what() << "\n" << program;
        return 1;
    }

    std::string iface = Tether::Examples::resolveInterface(program.get<std::string>("--interface"), TAG);
    std::string debug_str = program.get<std::string>("--debug");
    double duration_sec = program.get<double>("--time");
    int slot_scan_delay = program.get<int>("--slot-scan-delay");
    bool interactive = program.get<bool>("--interactive");

    auto mod_config = parseModuleConfig(program);
    if (program.is_used("--tc-type") && !mod_config.tc_signal_form) return 1;
    if (program.is_used("--tc-filter") && !mod_config.tc_filter) return 1;
    if (program.is_used("--tc-cjc") && !mod_config.tc_cjc) return 1;
    if (program.is_used("--ai-type") && !mod_config.ai_signal_form) return 1;
    if (program.is_used("--ai-filter") && !mod_config.ai_filter) return 1;
    if (program.is_used("--rd-type") && !mod_config.rd_signal_form) return 1;
    if (program.is_used("--rd-filter") && !mod_config.rd_filter) return 1;

#ifndef HAVE_NCURSES
    if (interactive) {
        std::cerr << "Interactive mode requires ncurses (not compiled in).\n";
        return 1;
    }
#endif

    if (Tether::Examples::printDebugHelpIfRequested(debug_str)) return 0;
    auto debug_flags = Tether::Examples::parseDebugFlags(debug_str);

    Tether::Platform::ensureRealtimeKernelOrExit();

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

    std::string esi_xml = program.get<std::string>("--esi-xml");
#if !TETHER_HAVE_ESI
    if (!esi_xml.empty()) {
        std::cerr << "ESI support not compiled in (TETHER_BUILD_EXTRACT_ESI=OFF). "
                     "Cannot use --esi-xml.\n";
        return 1;
    }
    const bool use_esi = false;
    (void)use_esi;
#else
    const bool use_esi = !esi_xml.empty();
    std::optional<EtherCAT::ESIFile> esi;
    if (use_esi) {
        esi.emplace(esi_xml);
        if (esi->empty()) {
            TETHER_LOGE(TAG, "Failed to parse ESI XML '{}': {}",
                        esi_xml.c_str(), esi->errorMessage().c_str());
            return 1;
        }
        TETHER_LOGI(TAG, "Loaded ESI XML '{}' ({} device(s))",
                    esi_xml.c_str(), esi->devices().size());
    }
#endif

    TETHER_LOGI(TAG, "kinco_rp20_io — interface: {}, duration: {:.1f} s",
                iface.c_str(), duration_sec);
    Tether::Examples::logVlanConfig(vlan, TAG);
    Tether::Examples::logMailboxConfig(mbSize, mbAddr, TAG);

    // ---- Signal handlers ----
    Tether::Utils::SignalHandler sig_handler(g_running, false);

    // ---- Host Ethernet setup ----
    Tether::Examples::HostEtherNetSession session;
    if (!Tether::Examples::initHostEthernet(session, iface, TAG)) {
        return 2;
    }

    EtherCAT::Master master;
    g_master = &master;
    sig_handler.setCancelCallback([&master]() { master.requestCancel(); });
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

    // ---- Discover slaves ----
    if (!master.discoverSlaves()) {
        TETHER_LOGE(TAG, "No slaves discovered");
        master.stop();
        Tether::Examples::shutdownHostEthernet(session);
        return 4;
    }

    uint16_t slave_count = master.getDiscoveredSlaveCount();
    TETHER_LOGI(TAG, "Discovered {} slave(s)", slave_count);
    master.logDiscoveredSlavesSummary(TAG);

    // ---- Per-slave: configure mailbox, transition to PRE-OP ----
    for (uint16_t s = 0; s < slave_count; ++s) {
        auto& sl = master.slave(s);

        TETHER_LOGI(TAG, "{}: configuring mailbox...", master.slaveLogPrefix(s).c_str());
        EtherCAT::SlaveError mb_err;
#if TETHER_HAVE_ESI
        if (use_esi) {
            mb_err = sl.configureMailbox(*esi);
        } else
#endif
        {
            mb_err = sl.configureMailbox(
                {.address = mbAddr.outAddress, .length = mbSize.outSize},
                {.address = mbAddr.inAddress, .length = mbSize.inSize},
                0x0004);
        }
        if (mb_err != EtherCAT::SlaveError::Ok) {
            TETHER_LOGW(TAG, "{}: explicit mailbox config failed ({}), trying SII auto-config",
                        master.slaveLogPrefix(s).c_str(), EtherCAT::slaveErrorToString(mb_err));
            if (!master.autoConfigureMailbox(s, Tether::Platform::LogLevel::Info)) {
                TETHER_LOGE(TAG, "{}: autoConfigureMailbox also failed", master.slaveLogPrefix(s).c_str());
                master.stop();
                Tether::Examples::shutdownHostEthernet(session);
                return 7;
            }
            sl.assumeMailboxAlreadyConfigured();
        }

        auto pre_err = sl.transitionToPreOp();
        if (pre_err != EtherCAT::SlaveError::Ok) {
            TETHER_LOGE(TAG, "{}: PRE-OP transition failed: {}",
                        master.slaveLogPrefix(s).c_str(), EtherCAT::slaveErrorToString(pre_err));
            master.stop();
            Tether::Examples::shutdownHostEthernet(session);
            return 8;
        }
        TETHER_LOGI(TAG, "{}: in PRE-OP", master.slaveLogPrefix(s).c_str());
    }

    // ---- Delay for slaves to settle ----
    if (slot_scan_delay > 0) {
        Tether::Platform::Clock::instance().delayMilliseconds(
            static_cast<uint32_t>(slot_scan_delay));
    }

    // ---- Scan all slaves/slots for RP20 modules ----
    TETHER_LOGI(TAG, "Scanning for RP20 modules (slots 0-15)...");
    auto modules = scanSlots(master, slave_count);

    if (modules.empty()) {
        TETHER_LOGE(TAG, "No RP20 modules found on any slave");
        master.stop();
        Tether::Examples::shutdownHostEthernet(session);
        return 6;
    }

    TETHER_LOGI(TAG, "Found {} RP20 module(s)", modules.size());

    // ---- Send CoE init commands for each module ----
    for (const auto& mod : modules) {
        sendInitCommands(master, mod);
    }
    applyModuleConfigToAll(master, modules, mod_config);

    // ---- Register PDO buffers on master side ----
    if (!registerPDOBuffers(master, modules)) {
        TETHER_LOGE(TAG, "Failed to register PDO buffers");
        master.stop();
        Tether::Examples::shutdownHostEthernet(session);
        return 9;
    }

    // ---- Write PDO assignments and configure SMs per slave ----
    for (uint16_t s = 0; s < slave_count; ++s) {
        // Check if this slave has any modules
        bool has_modules = false;
        for (const auto& mod : modules) {
            if (mod.slave_index == s) { has_modules = true; break; }
        }
        if (!has_modules) continue;

        // Assign PDOs via SDO
        assignPDOsToSlave(master, modules, s);

        // Configure SM2/SM3 from SII
        if (!master.configureProcessDataSyncManagersFromSii(s)) {
            TETHER_LOGW(TAG, "{}: SM2/SM3 config from SII failed", master.slaveLogPrefix(s).c_str());
        }

        // Update SM2/SM3 lengths to match total PDO sizes
        updateSMLengths(master, modules, s);

        // Finalize PDO mapping
        master.pdo().finalizeMapping(s);

        // Mark PDO as configured and transition to SAFE-OP
        auto& sl = master.slave(s);
        sl.assumePDOAlreadyConfigured();

        auto safe_err = sl.transitionToSafeOp();
        if (safe_err != EtherCAT::SlaveError::Ok) {
            TETHER_LOGE(TAG, "{}: SAFE-OP transition failed: {}",
                        master.slaveLogPrefix(s).c_str(), EtherCAT::slaveErrorToString(safe_err));
            master.stop();
            Tether::Examples::shutdownHostEthernet(session);
            return 10;
        }
        TETHER_LOGI(TAG, "{}: in SAFE-OP", master.slaveLogPrefix(s).c_str());
    }

    // ---- Start realtime motion loop ----
    std::atomic<uint64_t> cycle_count{0};
    std::atomic<bool> pdo_ok{true};

    master.setMotionControlCallback(
        [&master, &modules, &cycle_count, &pdo_ok](double /*dt*/) -> bool {
            if (!master.pdo().exchangeAll()) {
                pdo_ok.store(false);
                return true;
            }
            pdo_ok.store(true);
            cycle_count.fetch_add(1);
            return true;
        });

    EtherCAT::Master::RealtimeMotionLoopConfig loop_config;
    loop_config.cycle_period_us = 1000;
    loop_config.sync_interval_cycles = 10;
    loop_config.enable_dc_synchronization = false;

    if (!master.startRealtimeMotionControlLoop(loop_config)) {
        TETHER_LOGE(TAG, "Failed to start realtime motion loop");
        master.stop();
        Tether::Examples::shutdownHostEthernet(session);
        return 11;
    }

    // ---- Transition to OP ----
    for (uint16_t s = 0; s < slave_count; ++s) {
        bool has_modules = false;
        for (const auto& mod : modules) {
            if (mod.slave_index == s) { has_modules = true; break; }
        }
        if (!has_modules) continue;

        auto& sl = master.slave(s);
        auto op_err = sl.transitionToOp();
        if (op_err != EtherCAT::SlaveError::Ok) {
            TETHER_LOGE(TAG, "{}: OP transition failed: {}",
                        master.slaveLogPrefix(s).c_str(), EtherCAT::slaveErrorToString(op_err));
            master.stopMotionControlLoop();
            master.stop();
            Tether::Examples::shutdownHostEthernet(session);
            return 12;
        }
        TETHER_LOGI(TAG, "{}: in OP", master.slaveLogPrefix(s).c_str());
    }

    TETHER_LOGI(TAG, "All slaves in OP — starting cyclic I/O");

#ifdef HAVE_NCURSES
    if (interactive) {
        runInteractiveUI(master, modules, cycle_count, pdo_ok, mod_config);
    } else
#endif
    {
        // ---- Main display loop (non-interactive) ----
        auto start_time = std::chrono::steady_clock::now();
        uint64_t last_print_cycle = 0;

        while (g_running.load()) {
            if (duration_sec > 0.0) {
                auto elapsed = std::chrono::duration<double>(
                    std::chrono::steady_clock::now() - start_time).count();
                if (elapsed >= duration_sec) break;
            }

            uint64_t cyc = cycle_count.load();
            if (cyc != last_print_cycle && cyc % 100 == 0) {
                last_print_cycle = cyc;
                std::cout << "\n=== Cycle " << cyc << " ===\n";
                for (auto& mod : modules) {
                    printModuleIO(mod, cyc, mod_config);
                }
                std::cout.flush();
            }

            if (!pdo_ok.load()) {
                TETHER_LOGW(TAG, "PDO exchange error detected");
            }

            Tether::Platform::Clock::instance().delayMilliseconds(10);
        }
    }

    // ---- Shutdown ----
    TETHER_LOGI(TAG, "Shutting down...");
    master.stopMotionControlLoop();
    master.stop();
    g_master = nullptr;
    Tether::Examples::shutdownHostEthernet(session);

    TETHER_LOGI(TAG, "Done. Total cycles: {}",
                static_cast<unsigned long long>(cycle_count.load()));
    return 0;
}

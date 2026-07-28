/**
 * @file RP20Device.cpp
 * @brief Runtime driver implementation for Kinco RP20 modular EtherCAT I/O couplers
 */

#include "tether/drives/RP20/RP20Device.hpp"

#include "tether/ethercat/Master.hpp"
#include "tether/ethercat/Slave.hpp"
#include "tether/ethercat/CoEManager.hpp"
#include "tether/ethercat/PDOManager.hpp"
#include "tether/ethercat/SyncManager.hpp"
#include "tether/platform/EspCompat.hpp"

#include <magic_enum/magic_enum.hpp>
#include <span>
#include <sstream>
#include <string>
#include <string_view>
#include <unordered_map>

namespace EtherCAT {
namespace Drives {
namespace RP20Module {

static const char* TAG = "RP20Device";

// ---------------------------------------------------------------------------
// Constructor
// ---------------------------------------------------------------------------

RP20Device::RP20Device(EtherCAT::Master& master)
    : master_(master)
{
}

// ---------------------------------------------------------------------------
// scanModules — discover RP20 modules across all slaves and slots 0–15
// ---------------------------------------------------------------------------

bool RP20Device::scanModules() {
    modules_.clear();

    uint16_t slave_count = master_.getDiscoveredSlaveCount();

    for (uint16_t s = 0; s < slave_count; ++s) {
        for (uint8_t slot = 0; slot < 16; ++slot) {
            uint16_t diag_idx = diagnosisIndexForSlot(slot);
            // The RP20 diagnosis entry is a 4-byte OD object whose low byte
            // holds the module ID; readU8 returns the relevant byte and the
            // trailing 3 bytes are expected padding.
            CoE::CoETransactionOptions opts;
            opts.allow_trailing_bytes = true;
            auto id_res = master_.sdoManager(s).readU8(diag_idx, 0x01, opts);
            if (!id_res.has_value()) {
                continue;
            }
            uint8_t module_id = id_res.value();
            if (module_id == 0) {
                continue;
            }

            const ModuleDescriptor* desc = findByIdent(module_id);
            if (!desc) {
                TETHER_LOGW(TAG, "Slave %u slot %u: unknown module ID 0x%02X, skipping",
                            s, slot, module_id);
                continue;
            }

            TETHER_LOGI(TAG, "Slave %u slot %u: found %s (%s, ident=0x%02X)",
                        s, slot, desc->name, desc->module_class, module_id);

            ModuleInstance mod;
            mod.slave_index = s;
            mod.slot = slot;
            mod.descriptor = desc;

            if (desc->has_txpdo && desc->txpdo) {
                mod.tx_buffer.assign(desc->txpdo->size, 0);
            }
            if (desc->has_rxpdo && desc->rxpdo) {
                mod.rx_buffer.assign(desc->rxpdo->size, 0);
            }

            modules_.push_back(std::move(mod));
        }
    }

    return !modules_.empty();
}

// ---------------------------------------------------------------------------
// sendInitCommands — dispatch CoE init commands for one module
// ---------------------------------------------------------------------------

bool RP20Device::sendInitCommands(size_t module_index) {
    if (module_index >= modules_.size()) return false;

    const auto& mod = modules_[module_index];
    auto& sdo = master_.sdoManager(mod.slave_index);
    uint16_t cfg_idx = configIndexForSlot(mod.slot);

    bool ok = true;
    for (size_t i = 0; i < mod.descriptor->init_cmd_count; ++i) {
        const Cfg::CoEInitCmd& cmd = mod.descriptor->init_cmds[i];
        uint16_t idx = cfg_idx;

        bool cmd_ok = false;
        if (cmd.data_size == 1) {
            cmd_ok = sdo.writeU8(idx, cmd.subindex,
                                 static_cast<uint8_t>(cmd.data)).has_value();
        } else if (cmd.data_size == 2) {
            cmd_ok = sdo.writeU16(idx, cmd.subindex,
                                  static_cast<uint16_t>(cmd.data)).has_value();
        } else if (cmd.data_size == 4) {
            cmd_ok = sdo.writeU32(idx, cmd.subindex, cmd.data).has_value();
        } else {
            TETHER_LOGW(TAG, "Slave %u slot %u: init cmd %zu has unsupported size %u",
                        mod.slave_index, mod.slot, i, cmd.data_size);
            continue;
        }

        if (!cmd_ok) {
            TETHER_LOGW(TAG, "Slave %u slot %u: init cmd 0x%04X:0x%02X failed (%s)",
                        mod.slave_index, mod.slot, idx, cmd.subindex, cmd.comment);
            ok = false;
        } else {
            TETHER_LOGI(TAG, "Slave %u slot %u: init 0x%04X:0x%02X = 0x%X (%s)",
                        mod.slave_index, mod.slot, idx, cmd.subindex,
                        cmd.data, cmd.comment);
        }
    }

    return ok;
}

bool RP20Device::sendInitCommandsAll() {
    bool ok = true;
    for (size_t i = 0; i < modules_.size(); ++i) {
        if (!sendInitCommands(i)) ok = false;
    }
    return ok;
}

// ---------------------------------------------------------------------------
// configureModule — write TC/AI/RD signal form, filter, CJC via SDO
// ---------------------------------------------------------------------------

bool RP20Device::configureModule(size_t module_index, const ModuleConfig& config) {
    if (module_index >= modules_.size()) return false;

    const auto& mod = modules_[module_index];
    const auto type = mod.descriptor->type;
    const bool is_tc = (type == ModuleType::TC_4);
    const bool is_ai = (type == ModuleType::AI_4 ||
                        type == ModuleType::Mixed_AIO);
    const bool is_rd = (type == ModuleType::RD_4);
    if (!is_tc && !is_ai && !is_rd)
        return true;

    auto& sdo = master_.sdoManager(mod.slave_index);
    uint16_t cfg_idx = configIndexForSlot(mod.slot);
    bool ok = true;

    uint8_t ai_ch_count = (type == ModuleType::Mixed_AIO) ? 2 : 4;

    auto writeChannels = [&](uint8_t start_sub, uint8_t ch_count,
                             uint8_t val, const char* what,
                             std::string_view name_sv) {
        for (uint8_t ch = 0; ch < ch_count; ++ch) {
            uint8_t sub = static_cast<uint8_t>(start_sub + ch);
            if (!sdo.writeU8(cfg_idx, sub, val).has_value()) {
                TETHER_LOGW(TAG, "Slave %u slot %u: failed to set %s CH%u=%.*s",
                            mod.slave_index, mod.slot, what, ch,
                            static_cast<int>(name_sv.size()), name_sv.data());
                ok = false;
            } else {
                TETHER_LOGI(TAG, "Slave %u slot %u: %s CH%u=%.*s",
                            mod.slave_index, mod.slot, what, ch,
                            static_cast<int>(name_sv.size()), name_sv.data());
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
            uint8_t filter_sub = (type == ModuleType::Mixed_AIO) ? 0x03 : 0x05;
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

bool RP20Device::configureAllModules(const ModuleConfig& config) {
    bool ok = true;
    for (size_t i = 0; i < modules_.size(); ++i) {
        if (!configureModule(i, config)) ok = false;
    }
    return ok;
}

// ---------------------------------------------------------------------------
// registerPDOs — register TxPDO/RxPDO buffers with the master's PDO mapping
// ---------------------------------------------------------------------------

bool RP20Device::registerPDOs() {
    auto& mapping = master_.pdo().mapping();

    for (auto& mod : modules_) {
        const auto* desc = mod.descriptor;
        uint16_t txpdo_idx = slotPDOIndex(Reg::kTxPDOBaseIndex, mod.slot);
        uint16_t rxpdo_idx = slotPDOIndex(Reg::kRxPDOBaseIndex, mod.slot);

        if (desc->has_txpdo && desc->txpdo && !mod.tx_buffer.empty()) {
            mod.tx_pdo_entry = mapping.add_txpdo(
                mod.slave_index, mod.tx_buffer.data(),
                desc->txpdo->size, txpdo_idx,
                PDO::PDOAddressMode::Position);
            if (mod.tx_pdo_entry < 0) {
                TETHER_LOGE(TAG, "Slave %u slot %u: failed to register TxPDO 0x%04X",
                            mod.slave_index, mod.slot, txpdo_idx);
                return false;
            }
            TETHER_LOGI(TAG, "Slave %u slot %u: registered TxPDO 0x%04X (%u bytes, entry %d)",
                        mod.slave_index, mod.slot, txpdo_idx,
                        desc->txpdo->size, mod.tx_pdo_entry);
        }

        if (desc->has_rxpdo && desc->rxpdo && !mod.rx_buffer.empty()) {
            mod.rx_pdo_entry = mapping.add_rxpdo(
                mod.slave_index, mod.rx_buffer.data(),
                desc->rxpdo->size, rxpdo_idx,
                PDO::PDOAddressMode::Position);
            if (mod.rx_pdo_entry < 0) {
                TETHER_LOGE(TAG, "Slave %u slot %u: failed to register RxPDO 0x%04X",
                            mod.slave_index, mod.slot, rxpdo_idx);
                return false;
            }
            TETHER_LOGI(TAG, "Slave %u slot %u: registered RxPDO 0x%04X (%u bytes, entry %d)",
                        mod.slave_index, mod.slot, rxpdo_idx,
                        desc->rxpdo->size, mod.rx_pdo_entry);
        }
    }

    return true;
}

// ---------------------------------------------------------------------------
// assignPDOs — assign RxPDOs to SM2 (0x1C12) and TxPDOs to SM3 (0x1C13)
// ---------------------------------------------------------------------------

bool RP20Device::assignPDOs(uint16_t slave_index) {
    std::vector<uint16_t> rxpdos, txpdos;
    for (const auto& mod : modules_) {
        if (mod.slave_index != slave_index) continue;
        if (mod.descriptor->has_rxpdo && mod.descriptor->rxpdo) {
            rxpdos.push_back(slotPDOIndex(Reg::kRxPDOBaseIndex, mod.slot));
        }
        if (mod.descriptor->has_txpdo && mod.descriptor->txpdo) {
            txpdos.push_back(slotPDOIndex(Reg::kTxPDOBaseIndex, mod.slot));
        }
    }

    auto& sdo = master_.sdoManager(slave_index);

    if (!rxpdos.empty()) {
        if (!sdo.writeU8(CiA301::SyncManager2PDOAssign, 0, 0).has_value()) {
            TETHER_LOGW(TAG, "Slave %u: failed to clear SM2 PDO count", slave_index);
        }
        for (size_t i = 0; i < rxpdos.size(); ++i) {
            if (!sdo.writeU16(CiA301::SyncManager2PDOAssign,
                              static_cast<uint8_t>(i + 1), rxpdos[i]).has_value()) {
                TETHER_LOGW(TAG, "Slave %u: failed to assign RxPDO 0x%04X to SM2",
                            slave_index, rxpdos[i]);
            }
        }
        if (!sdo.writeU8(CiA301::SyncManager2PDOAssign, 0,
                         static_cast<uint8_t>(rxpdos.size())).has_value()) {
            TETHER_LOGW(TAG, "Slave %u: failed to set SM2 PDO count", slave_index);
        }
        TETHER_LOGI(TAG, "Slave %u: assigned %zu RxPDO(s) to SM2", slave_index, rxpdos.size());
    }

    if (!txpdos.empty()) {
        if (!sdo.writeU8(CiA301::SyncManager3PDOAssign, 0, 0).has_value()) {
            TETHER_LOGW(TAG, "Slave %u: failed to clear SM3 PDO count", slave_index);
        }
        for (size_t i = 0; i < txpdos.size(); ++i) {
            if (!sdo.writeU16(CiA301::SyncManager3PDOAssign,
                              static_cast<uint8_t>(i + 1), txpdos[i]).has_value()) {
                TETHER_LOGW(TAG, "Slave %u: failed to assign TxPDO 0x%04X to SM3",
                            slave_index, txpdos[i]);
            }
        }
        if (!sdo.writeU8(CiA301::SyncManager3PDOAssign, 0,
                         static_cast<uint8_t>(txpdos.size())).has_value()) {
            TETHER_LOGW(TAG, "Slave %u: failed to set SM3 PDO count", slave_index);
        }
        TETHER_LOGI(TAG, "Slave %u: assigned %zu TxPDO(s) to SM3", slave_index, txpdos.size());
    }

    return true;
}

bool RP20Device::assignPDOsAll() {
    uint16_t slave_count = master_.getDiscoveredSlaveCount();
    bool ok = true;
    for (uint16_t s = 0; s < slave_count; ++s) {
        bool has_modules = false;
        for (const auto& mod : modules_) {
            if (mod.slave_index == s) { has_modules = true; break; }
        }
        if (!has_modules) continue;
        if (!assignPDOs(s)) ok = false;
    }
    return ok;
}

// ---------------------------------------------------------------------------
// updateSyncManagerLengths — set SM2/SM3 lengths to match total PDO sizes
// ---------------------------------------------------------------------------

void RP20Device::updateSyncManagerLengths(uint16_t slave_index) {
    uint16_t total_rx = 0, total_tx = 0;
    for (const auto& mod : modules_) {
        if (mod.slave_index != slave_index) continue;
        if (mod.descriptor->has_rxpdo && mod.descriptor->rxpdo) {
            total_rx += mod.descriptor->rxpdo->size;
        }
        if (mod.descriptor->has_txpdo && mod.descriptor->txpdo) {
            total_tx += mod.descriptor->txpdo->size;
        }
    }

    auto* cfgs = master_.pdo().slaveConfigs();
    if (!cfgs) return;

    if (total_rx > 0) {
        TETHER_LOGI(TAG, "Slave %u: SM2 length: %u -> %u",
                    slave_index, cfgs[slave_index].sm[2].length, total_rx);
        cfgs[slave_index].sm[2].length = total_rx;
        cfgs[slave_index].rxpdo_size = total_rx;
    }
    if (total_tx > 0) {
        TETHER_LOGI(TAG, "Slave %u: SM3 length: %u -> %u",
                    slave_index, cfgs[slave_index].sm[3].length, total_tx);
        cfgs[slave_index].sm[3].length = total_tx;
        cfgs[slave_index].txpdo_size = total_tx;
    }
}

void RP20Device::updateSyncManagerLengthsAll() {
    uint16_t slave_count = master_.getDiscoveredSlaveCount();
    for (uint16_t s = 0; s < slave_count; ++s) {
        bool has_modules = false;
        for (const auto& mod : modules_) {
            if (mod.slave_index == s) { has_modules = true; break; }
        }
        if (!has_modules) continue;
        updateSyncManagerLengths(s);
    }
}

// ---------------------------------------------------------------------------
// P1: bringToSafeOp — full PDO config + SAFE-OP transition
// ---------------------------------------------------------------------------

bool RP20Device::bringToSafeOp(const ModuleConfig& config) {
    if (modules_.empty()) {
        TETHER_LOGE(TAG, "bringToSafeOp: no modules scanned");
        return false;
    }

    sendInitCommandsAll();
    configureAllModules(config);

    if (!registerPDOs()) {
        TETHER_LOGE(TAG, "bringToSafeOp: PDO registration failed");
        return false;
    }

    uint16_t slave_count = master_.getDiscoveredSlaveCount();
    for (uint16_t s = 0; s < slave_count; ++s) {
        bool has_modules = false;
        for (const auto& mod : modules_) {
            if (mod.slave_index == s) { has_modules = true; break; }
        }
        if (!has_modules) continue;

        assignPDOs(s);

        if (!master_.configureProcessDataSyncManagersFromSii(s)) {
            TETHER_LOGW(TAG, "bringToSafeOp: slave %u SM2/SM3 config from SII failed", s);
        }

        updateSyncManagerLengths(s);
        master_.pdo().finalizeMapping(s);

        auto& sl = master_.slave(s);
        sl.assumePDOAlreadyConfigured();

        auto err = sl.transitionToSafeOp();
        if (err != SlaveError::Ok) {
            TETHER_LOGE(TAG, "bringToSafeOp: slave %u SAFE-OP failed: %s",
                        s, slaveErrorToString(err));
            return false;
        }
        TETHER_LOGI(TAG, "bringToSafeOp: slave %u in SAFE-OP", s);
    }

    return true;
}

// ---------------------------------------------------------------------------
// P1: bringToOp — transition from SAFE-OP to OP
// ---------------------------------------------------------------------------

bool RP20Device::bringToOp() {
    uint16_t slave_count = master_.getDiscoveredSlaveCount();
    for (uint16_t s = 0; s < slave_count; ++s) {
        bool has_modules = false;
        for (const auto& mod : modules_) {
            if (mod.slave_index == s) { has_modules = true; break; }
        }
        if (!has_modules) continue;

        auto& sl = master_.slave(s);
        auto err = sl.transitionToOp();
        if (err != SlaveError::Ok) {
            TETHER_LOGE(TAG, "bringToOp: slave %u OP failed: %s",
                        s, slaveErrorToString(err));
            return false;
        }
        TETHER_LOGI(TAG, "bringToOp: slave %u in OP", s);
    }
    return true;
}

// ---------------------------------------------------------------------------
// P2: Typed I/O channel accessors
// ---------------------------------------------------------------------------

uint8_t RP20Device::readDigitalInputs(size_t module_index, size_t field_idx) const {
    if (module_index >= modules_.size()) return 0;
    const auto& mod = modules_[module_index];
    const auto* desc = mod.descriptor;
    if (!desc || !desc->has_txpdo || !desc->txpdo || mod.tx_buffer.empty()) return 0;

    const auto type = desc->type;
    if (type != ModuleType::DI_16 && type != ModuleType::Multi_DIO_8) return 0;

    const auto* f = getFieldByChannel(*desc->txpdo, field_idx);
    if (!f) return 0;

    std::span<const uint8_t> tx(mod.tx_buffer.data(), mod.tx_buffer.size());
    return readU8(tx, *f);
}

float RP20Device::readTemperature(size_t module_index, size_t field_idx) const {
    if (module_index >= modules_.size()) return 0.0f;
    const auto& mod = modules_[module_index];
    const auto* desc = mod.descriptor;
    if (!desc || !desc->has_txpdo || !desc->txpdo || mod.tx_buffer.empty()) return 0.0f;

    if (desc->type != ModuleType::TC_4) return 0.0f;

    const auto* f = getFieldByChannel(*desc->txpdo, field_idx);
    if (!f) return 0.0f;

    std::span<const uint8_t> tx(mod.tx_buffer.data(), mod.tx_buffer.size());
    return readI16(tx, *f) / 10.0f;
}

int16_t RP20Device::readAnalogInput(size_t module_index, size_t field_idx) const {
    if (module_index >= modules_.size()) return 0;
    const auto& mod = modules_[module_index];
    const auto* desc = mod.descriptor;
    if (!desc || !desc->has_txpdo || !desc->txpdo || mod.tx_buffer.empty()) return 0;

    const auto type = desc->type;
    if (type != ModuleType::AI_4 && type != ModuleType::RD_4 &&
        type != ModuleType::Mixed_AIO) return 0;

    const auto* f = getFieldByChannel(*desc->txpdo, field_idx);
    if (!f) return 0;

    std::span<const uint8_t> tx(mod.tx_buffer.data(), mod.tx_buffer.size());
    return readI16(tx, *f);
}

void RP20Device::writeDigitalOutputs(size_t module_index, size_t field_idx, uint8_t pattern) {
    if (module_index >= modules_.size()) return;
    auto& mod = modules_[module_index];
    const auto* desc = mod.descriptor;
    if (!desc || !desc->has_rxpdo || !desc->rxpdo || mod.rx_buffer.empty()) return;

    const auto type = desc->type;
    if (type != ModuleType::DO_16_PNP && type != ModuleType::DO_16_NPN &&
        type != ModuleType::Multi_DIO_8 && type != ModuleType::DR_8) return;

    const auto* f = getFieldByChannel(*desc->rxpdo, field_idx);
    if (!f) return;

    std::span<uint8_t> rx(mod.rx_buffer.data(), mod.rx_buffer.size());
    writeU8(rx, *f, pattern);
}

void RP20Device::writeAnalogOutput(size_t module_index, size_t field_idx, int16_t value) {
    if (module_index >= modules_.size()) return;
    auto& mod = modules_[module_index];
    const auto* desc = mod.descriptor;
    if (!desc || !desc->has_rxpdo || !desc->rxpdo || mod.rx_buffer.empty()) return;

    const auto type = desc->type;
    if (type != ModuleType::AO_4 && type != ModuleType::Mixed_AIO) return;

    const auto* f = getFieldByChannel(*desc->rxpdo, field_idx);
    if (!f) return;

    std::span<uint8_t> rx(mod.rx_buffer.data(), mod.rx_buffer.size());
    writeI16(rx, *f, value);
}

void RP20Device::writeOutputBit(size_t module_index, size_t field_idx, uint8_t bit, bool value) {
    if (module_index >= modules_.size()) return;
    auto& mod = modules_[module_index];
    const auto* desc = mod.descriptor;
    if (!desc || !desc->has_rxpdo || !desc->rxpdo || mod.rx_buffer.empty()) return;

    const auto type = desc->type;
    if (type != ModuleType::DO_16_PNP && type != ModuleType::DO_16_NPN &&
        type != ModuleType::Multi_DIO_8 && type != ModuleType::DR_8) return;

    const auto* f = getFieldByChannel(*desc->rxpdo, field_idx);
    if (!f) return;

    std::span<uint8_t> rx(mod.rx_buffer.data(), mod.rx_buffer.size());
    writeBit(rx, *f, bit, value);
}

// ---------------------------------------------------------------------------
// P3: ModuleConfig::formatLabel
// ---------------------------------------------------------------------------

std::string ModuleConfig::formatLabel(ModuleType type) const {
    std::string label = "[";

    auto optName = [](auto opt) -> std::string {
        return opt ? std::string(magic_enum::enum_name(*opt)) : "default";
    };

    switch (type) {
        case ModuleType::TC_4:
            label += optName(tc_signal_form) + ", ";
            label += optName(tc_filter) + " filter, ";
            label += optName(tc_cjc) + " CJC]";
            break;
        case ModuleType::AI_4:
        case ModuleType::Mixed_AIO:
            label += optName(ai_signal_form) + ", ";
            label += optName(ai_filter) + " filter]";
            break;
        case ModuleType::RD_4:
            label += optName(rd_signal_form) + ", ";
            label += optName(rd_filter) + " filter]";
            break;
        default:
            label = "[no config]";
            break;
    }
    return label;
}

// ---------------------------------------------------------------------------
// P4: String-to-enum parsing with vendor aliases
// ---------------------------------------------------------------------------

std::optional<Registers::RP20::TCSignalForm>
RP20Device::parseTCSignalForm(std::string_view sv) {
    std::string s(sv);
    if (s.empty() || s == "-1") return std::nullopt;
    if (auto v = magic_enum::enum_cast<Registers::RP20::TCSignalForm>(s)) return v;
    static const std::unordered_map<std::string, Registers::RP20::TCSignalForm> aliases = {
        {"J", Registers::RP20::TCSignalForm::Type_J},
        {"K", Registers::RP20::TCSignalForm::Type_K},
        {"E", Registers::RP20::TCSignalForm::Type_E},
        {"S", Registers::RP20::TCSignalForm::Type_S},
        {"T", Registers::RP20::TCSignalForm::Type_T},
        {"100mV", Registers::RP20::TCSignalForm::Voltage_100mV},
    };
    auto it = aliases.find(s);
    if (it != aliases.end()) return it->second;
    try {
        int val = std::stoi(s);
        if (val >= 0 && val <= 5) return static_cast<Registers::RP20::TCSignalForm>(val);
    } catch (...) {}
    return std::nullopt;
}

std::optional<Registers::RP20::AISignalForm>
RP20Device::parseAISignalForm(std::string_view sv) {
    std::string s(sv);
    if (s.empty() || s == "-1") return std::nullopt;
    if (auto v = magic_enum::enum_cast<Registers::RP20::AISignalForm>(s)) return v;
    static const std::unordered_map<std::string, Registers::RP20::AISignalForm> aliases = {
        {"4-20mA", Registers::RP20::AISignalForm::Current_4_20mA},
        {"20mA", Registers::RP20::AISignalForm::Current_20mA},
        {"1-5V", Registers::RP20::AISignalForm::Voltage_1_5V},
        {"10V", Registers::RP20::AISignalForm::Voltage_10V},
    };
    auto it = aliases.find(s);
    if (it != aliases.end()) return it->second;
    try {
        int val = std::stoi(s);
        if (val >= 0 && val <= 3) return static_cast<Registers::RP20::AISignalForm>(val);
    } catch (...) {}
    return std::nullopt;
}

std::optional<Registers::RP20::RTDSignalForm>
RP20Device::parseRTDSignalForm(std::string_view sv) {
    std::string s(sv);
    if (s.empty() || s == "-1") return std::nullopt;
    if (auto v = magic_enum::enum_cast<Registers::RP20::RTDSignalForm>(s)) return v;
    static const std::unordered_map<std::string, Registers::RP20::RTDSignalForm> aliases = {
        {"PT100", Registers::RP20::RTDSignalForm::PT100},
        {"PT1000", Registers::RP20::RTDSignalForm::PT1000},
        {"Cu50", Registers::RP20::RTDSignalForm::Cu50},
        {"Cu100", Registers::RP20::RTDSignalForm::Cu100},
    };
    auto it = aliases.find(s);
    if (it != aliases.end()) return it->second;
    try {
        int val = std::stoi(s);
        if (val == 0 || val == 1 || val == 4 || val == 5)
            return static_cast<Registers::RP20::RTDSignalForm>(val);
    } catch (...) {}
    return std::nullopt;
}

std::optional<Registers::RP20::FilteringMode>
RP20Device::parseFilteringMode(std::string_view sv) {
    std::string s(sv);
    if (s.empty() || s == "-1") return std::nullopt;
    if (auto v = magic_enum::enum_cast<Registers::RP20::FilteringMode>(s)) return v;
    if (s == "none" || s == "0") return Registers::RP20::FilteringMode::None;
    if (s == "average" || s == "1") return Registers::RP20::FilteringMode::Average;
    return std::nullopt;
}

std::optional<Registers::RP20::ColdJunctionCompensation>
RP20Device::parseColdJunctionCompensation(std::string_view sv) {
    std::string s(sv);
    if (s.empty() || s == "-1") return std::nullopt;
    if (auto v = magic_enum::enum_cast<Registers::RP20::ColdJunctionCompensation>(s)) return v;
    if (s == "internal" || s == "0") return Registers::RP20::ColdJunctionCompensation::Internal;
    if (s == "external" || s == "1") return Registers::RP20::ColdJunctionCompensation::External;
    return std::nullopt;
}

// ---------------------------------------------------------------------------
// P5: ModuleInstance::toString
// ---------------------------------------------------------------------------

std::string ModuleInstance::toString() const {
    std::ostringstream oss;
    oss << "Slave " << slave_index << " Slot " << static_cast<int>(slot);
    if (descriptor) {
        oss << ": " << descriptor->name
            << " (" << descriptor->module_class << ")";
    } else {
        oss << ": (no descriptor)";
    }
    return oss.str();
}

} // namespace RP20Module
} // namespace Drives
} // namespace EtherCAT

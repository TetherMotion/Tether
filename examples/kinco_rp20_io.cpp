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
 */

#include <atomic>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstring>
#include <iomanip>
#include <iostream>
#include <string>
#include <vector>

#include "tether/ethercat/Master.hpp"
#include "tether/ethercat/Slave.hpp"
#include "tether/ethercat/Types.hpp"
#include "tether/ethercat/SyncManager.hpp"
#include "tether/ethercat/PDOManager.hpp"
#include "tether/ethercat/CoEManager.hpp"
#include "tether/platform/EspCompat.hpp"
#include "tether/platform/Platform.hpp"

#include "tether/drives/RP20/RP20Module.hpp"
#include "tether/drives/RP20/RP20ModuleConfig.hpp"
#include "tether/drives/RP20/RP20PDO.hpp"
#include "tether/drives/RP20/RP20Registers.hpp"

#include "common/ExampleHelpers.hpp"
#include "common/EtherCATHostSetup.hpp"

static const char* TAG = "kinco_rp20_io";
static EtherCAT::Master* g_master = nullptr;

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
// Signal handler
// ============================================================================

static std::atomic<bool> g_running{true};

void signalHandler(int) {
    g_running.store(false);
    if (g_master) {
        g_master->requestCancel();
    }
}

// ============================================================================
// Slot scanning
// ============================================================================

static std::vector<DiscoveredModule> scanSlots(EtherCAT::Master& master,
                                               uint16_t slave_count) {
    std::vector<DiscoveredModule> modules;

    for (uint16_t s = 0; s < slave_count; ++s) {
        for (uint8_t slot = 0; slot < 16; ++slot) {
            uint16_t diag_idx = RP20Mod::diagnosisIndexForSlot(slot);
            auto id_res = master.sdoManager(s).readU8(diag_idx, 0x01);
            if (!id_res.has_value()) {
                continue;
            }
            uint8_t module_id = id_res.value();
            if (module_id == 0) {
                continue;
            }

            const RP20Mod::ModuleDescriptor* desc = RP20Mod::findByIdent(module_id);
            if (!desc) {
                TETHER_LOGW(TAG, "Slave %u slot %u: unknown module ID 0x%02X, skipping",
                            s, slot, module_id);
                continue;
            }

            TETHER_LOGI(TAG, "Slave %u slot %u: found %s (%s, ident=0x%02X)",
                        s, slot, desc->name, desc->module_class, module_id);

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
// Send CoE init commands for a module
// ============================================================================

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
            TETHER_LOGW(TAG, "Slave %u slot %u: init cmd %zu has unsupported size %u",
                        mod.slave_index, mod.slot, i, cmd.data_size);
            continue;
        }

        if (!ok) {
            TETHER_LOGW(TAG, "Slave %u slot %u: init cmd 0x%04X:0x%02X failed (%s)",
                        mod.slave_index, mod.slot, idx, cmd.subindex, cmd.comment);
        } else {
            TETHER_LOGI(TAG, "Slave %u slot %u: init 0x%04X:0x%02X = 0x%X (%s)",
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
                EtherCAT::PDO::PDOAddressMode::Position);
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

    // Assign TxPDOs to SM3 (0x1C13)
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

// ============================================================================
// Print module I/O data
// ============================================================================

static void printModuleIO(DiscoveredModule& mod, uint64_t cycle) {
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
            case RP20Mod::ModuleType::AI_4:
            case RP20Mod::ModuleType::RD_4:
            case RP20Mod::ModuleType::TC_4:
            case RP20Mod::ModuleType::Mixed_AIO: {
                // Analog inputs — print as signed 16-bit decimal
                std::cout << "AI:";
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
                // Digital/relay outputs — toggle a walking bit
                std::cout << " DO:";
                for (size_t i = 0; i < desc->rxpdo->field_count; ++i) {
                    const auto* f = RP20Mod::getFieldByChannel(*desc->rxpdo, i);
                    if (f) {
                        uint8_t pattern = static_cast<uint8_t>(
                            (cycle >> 4) & 0xFF);
                        if (desc->type == RP20Mod::ModuleType::DR_8) {
                            // For relay, toggle individual bits more slowly
                            pattern = static_cast<uint8_t>(1u << ((cycle / 500) % 8));
                        }
                        RP20Mod::writeU8(rx, *f, pattern);
                        std::cout << " " << std::hex << std::setw(2)
                                  << std::setfill('0') << static_cast<int>(pattern);
                    }
                }
                std::cout << std::dec;
                break;
            }
            case RP20Mod::ModuleType::AO_4:
            case RP20Mod::ModuleType::Mixed_AIO: {
                // Analog outputs — write a sine wave
                std::cout << " AO:";
                for (size_t i = 0; i < desc->rxpdo->field_count; ++i) {
                    const auto* f = RP20Mod::getFieldByChannel(*desc->rxpdo, i);
                    if (f) {
                        double phase = cycle * 0.01 + i * 1.5708;
                        int16_t val = static_cast<int16_t>(10000.0 * sin(phase));
                        RP20Mod::writeI16(rx, *f, val);
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
    Tether::Examples::addDurationArg(program, 0.0);
    program.add_argument("--slot-scan-delay")
        .scan<'i', int>()
        .default_value(100)
        .help("Delay in ms between PRE-OP and slot scan (default: 100)");

    try { program.parse_args(argc, argv); }
    catch (const std::runtime_error& err) {
        std::cerr << err.what() << "\n" << program;
        return 1;
    }

    std::string iface = program.get<std::string>("--interface");
    std::string debug_str = program.get<std::string>("--debug");
    double duration_sec = program.get<double>("--time");
    int slot_scan_delay = program.get<int>("--slot-scan-delay");

    if (Tether::Examples::printDebugHelpIfRequested(debug_str)) return 0;
    auto debug_flags = Tether::Examples::parseDebugFlags(debug_str);

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

    TETHER_LOGI(TAG, "kinco_rp20_io — interface: %s, duration: %.1f s",
                iface.c_str(), duration_sec);
    Tether::Examples::logVlanConfig(vlan, TAG);
    Tether::Examples::logMailboxConfig(mbSize, mbAddr, TAG);

    // ---- Signal handlers ----
    std::signal(SIGINT, signalHandler);
    std::signal(SIGTERM, signalHandler);

    // ---- Host Ethernet setup ----
    Tether::Examples::HostEtherNetSession session;
    if (!Tether::Examples::initHostEthernet(session, iface, TAG)) {
        return 2;
    }

    EtherCAT::Master master;
    g_master = &master;
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
    TETHER_LOGI(TAG, "Discovered %u slave(s)", slave_count);
    master.logDiscoveredSlavesSummary(TAG);

    // ---- Per-slave: configure mailbox, transition to PRE-OP ----
    for (uint16_t s = 0; s < slave_count; ++s) {
        auto& sl = master.slave(s);

        TETHER_LOGI(TAG, "Slave %u: configuring mailbox...", s);
        auto mb_err = sl.configureMailbox(
            {.address = mbAddr.outAddress, .length = mbSize.outSize},
            {.address = mbAddr.inAddress, .length = mbSize.inSize},
            0x0004);
        if (mb_err != EtherCAT::SlaveError::Ok) {
            TETHER_LOGW(TAG, "Slave %u: explicit mailbox config failed (%s), trying SII auto-config",
                        s, EtherCAT::slaveErrorToString(mb_err));
            if (!master.autoConfigureMailbox(s, Tether::Platform::LogLevel::Info)) {
                TETHER_LOGE(TAG, "Slave %u: autoConfigureMailbox also failed", s);
                master.stop();
                Tether::Examples::shutdownHostEthernet(session);
                return 7;
            }
            sl.assumeMailboxAlreadyConfigured();
        }

        auto pre_err = sl.transitionToPreOp();
        if (pre_err != EtherCAT::SlaveError::Ok) {
            TETHER_LOGE(TAG, "Slave %u: PRE-OP transition failed: %s",
                        s, EtherCAT::slaveErrorToString(pre_err));
            master.stop();
            Tether::Examples::shutdownHostEthernet(session);
            return 8;
        }
        TETHER_LOGI(TAG, "Slave %u: in PRE-OP", s);
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

    TETHER_LOGI(TAG, "Found %zu RP20 module(s)", modules.size());

    // ---- Send CoE init commands for each module ----
    for (const auto& mod : modules) {
        sendInitCommands(master, mod);
    }

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
            TETHER_LOGW(TAG, "Slave %u: SM2/SM3 config from SII failed", s);
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
            TETHER_LOGE(TAG, "Slave %u: SAFE-OP transition failed: %s",
                        s, EtherCAT::slaveErrorToString(safe_err));
            master.stop();
            Tether::Examples::shutdownHostEthernet(session);
            return 10;
        }
        TETHER_LOGI(TAG, "Slave %u: in SAFE-OP", s);
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
            TETHER_LOGE(TAG, "Slave %u: OP transition failed: %s",
                        s, EtherCAT::slaveErrorToString(op_err));
            master.stopMotionControlLoop();
            master.stop();
            Tether::Examples::shutdownHostEthernet(session);
            return 12;
        }
        TETHER_LOGI(TAG, "Slave %u: in OP", s);
    }

    TETHER_LOGI(TAG, "All slaves in OP — starting cyclic I/O");

    // ---- Main display loop ----
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
                printModuleIO(mod, cyc);
            }
            std::cout.flush();
        }

        if (!pdo_ok.load()) {
            TETHER_LOGW(TAG, "PDO exchange error detected");
        }

        Tether::Platform::Clock::instance().delayMilliseconds(10);
    }

    // ---- Shutdown ----
    TETHER_LOGI(TAG, "Shutting down...");
    master.stopMotionControlLoop();
    master.stop();
    g_master = nullptr;
    Tether::Examples::shutdownHostEthernet(session);

    TETHER_LOGI(TAG, "Done. Total cycles: %llu",
                static_cast<unsigned long long>(cycle_count.load()));
    return 0;
}

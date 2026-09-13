/**
 * @file el2004_toggle.cpp
 * @brief Beckhoff EL2004 (4ch digital output, 24V/0.5A) channel stepper demo
 *
 * Auto-selects the first EL2004 in the EtherCAT chain, brings it to OP, and
 * lights exactly one output channel at a time.  Pressing Enter advances to
 * the next channel (1 -> 2 -> 3 -> 4 -> 1 ...).
 *
 * EL2004 specifics (from Beckhoff ESI "EL2xxx.xml"):
 *   Vendor 0x00000002, ProductCode 0x07d43052.
 *   Four fixed RxPDOs 0x1600-0x1603, each mapping a single BOOL "Output"
 *   entry (0x3001:01..04).  All four channels live in ONE sync manager —
 *   SII SM channel 0 @ 0x0F00, len 1, ctrl 0x44 (buffered, ECAT-write,
 *   repeat-request, no watchdog).  The terminal has NO mailbox and no CoE —
 *   SM0 is the process-data SM, not a mailbox SM, so configureMailbox() must
 *   NOT be called (it would clobber the SM0 registers).
 *
 * Because the EL2004 is output-only, Slave::transitionToOp()'s
 * request+reply counter check cannot be satisfied (no TxPDO ever
 * increments the reply counter), so the OP request is issued directly via
 * requestSlaveApplicationLayerState().
 *
 * Usage (Linux, requires root or CAP_NET_RAW):
 *   ./el2004_toggle                # auto-detect interface
 *   ./el2004_toggle -i enp3s0      # specify interface
 *   ./el2004_toggle -t 30          # run for 30 s, then exit
 */

#include <atomic>
#include <bit>
#include <chrono>
#include <cstdint>
#include <iostream>
#include <string>

#include <poll.h>
#include <unistd.h>

#include "tether/ethercat/Master.hpp"
#include "tether/ethercat/Slave.hpp"
#include "tether/ethercat/SlaveDiscoveryManager.hpp"
#include "tether/ethercat/Types.hpp"
#include "tether/ethercat/PDOManager.hpp"
#include "tether/ethercat/FaultDetection.hpp"
#include "tether/platform/EspCompat.hpp"
#include "tether/platform/Platform.hpp"
#include "tether/sii/SIIParser.hpp"
#include "tether/utils/SignalHandler.hpp"

#include "common/ExampleHelpers.hpp"
#include "common/EtherCATHostSetup.hpp"

static const char* TAG = "el2004_toggle";

// ---- EL2004 identity / process-data layout --------------------------------
constexpr uint32_t kBeckhoffVendorId  = 0x00000002;
constexpr uint32_t kEL2004ProductCode = 0x07d43052;
constexpr size_t   kEL2004Channels    = 4;

// Fallbacks taken from the ESI, used when the SII sync-manager category
// could not be read for some reason.
constexpr uint8_t  kFallbackSmChannel = 0;
constexpr uint16_t kFallbackSmAddr    = 0x0F00;
constexpr uint16_t kFallbackSmLen     = 1;   // 4 x 1-bit outputs pack into 1 byte
constexpr uint8_t  kFallbackSmCtrl    = 0x44;

static std::atomic<bool> g_running{true};

static void printChannelState(size_t active) {
    std::cout << "Outputs: ";
    for (size_t ch = 0; ch < kEL2004Channels; ++ch) {
        std::cout << "  CH" << (ch + 1) << (ch == active ? " [ON] " : " [ --]");
    }
    std::cout << "   (press Enter for next channel)" << std::endl;
}

/// Poll AL status until @p target or timeout.  Returns true on match.
static bool waitAlState(EtherCAT::Master& master, uint16_t slave_index,
                        EtherCAT::SlaveState target, int timeout_ms) {
    for (int t = 0; t < timeout_ms; t += 10) {
        if (master.isCancelRequested()) return false;
        uint8_t state = 0;
        if (master.readSlaveApplicationLayerState(
                EtherCAT::SlaveAddress(slave_index), state) &&
            state == static_cast<uint8_t>(target)) {
            return true;
        }
        Tether::Platform::Clock::instance().delayMilliseconds(10);
    }
    return false;
}

int main(int argc, char** argv) {
    argparse::ArgumentParser program("el2004_toggle", "1.0",
                                     argparse::default_arguments::help);
    Tether::Examples::addInterfaceArg(program);
    Tether::Examples::addListInterfacesArg(program);
    Tether::Examples::addDebugArg(program);
    Tether::Examples::addVlanArgs(program);
    Tether::Examples::addDurationArg(program, 0.0);

    try { program.parse_args(argc, argv); }
    catch (const std::runtime_error& err) {
        std::cerr << err.what() << "\n" << program;
        return 1;
    }

    if (program.get<bool>("--list-interfaces")) {
        Tether::Examples::listPhysicalInterfaces(TAG);
        return 0;
    }

    std::string iface =
        Tether::Examples::resolveInterface(program.get<std::string>("--interface"), TAG);
    if (iface.empty()) return 1;

    std::string debug_str = program.get<std::string>("--debug");
    if (Tether::Examples::printDebugHelpIfRequested(debug_str)) return 0;
    auto debug_flags = Tether::Examples::parseDebugFlags(debug_str);

    Tether::Examples::VlanConfig vlan;
    if (!Tether::Examples::parseVlanArgs(
            program.get<std::string>("--rx-vlan"),
            program.get<std::string>("--tx-vlan"), vlan, TAG)) {
        return 1;
    }

    const double duration_sec = program.get<double>("--time");

    Tether::Platform::ensureRealtimeKernelOrExit();
    Tether::Utils::SignalHandler sig_handler(g_running, false);

    // ---- Host Ethernet + master bring-up ----
    Tether::Examples::HostEtherNetSession session;
    if (!Tether::Examples::initHostEthernet(session, iface, TAG)) {
        return 2;
    }

    EtherCAT::Master master;
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

    // ---- Discover and find the first EL2004 ----
    auto slaves = master.discovery().discover(EtherCAT::DiscoveryOption::All);
    if (slaves.empty()) {
        TETHER_LOGE(TAG, "No slaves discovered");
        master.stop();
        Tether::Examples::shutdownHostEthernet(session);
        return 4;
    }

    TETHER_LOGI(TAG, "=== Discovered {} slave(s) ===", slaves.size());
    const EtherCAT::DiscoveredSlave* el2004 = nullptr;
    for (const auto& s : slaves) {
        const char* name = s.device_name ? s.device_name->c_str() : "?";
        TETHER_LOGI(TAG, "Slave {}: {} (vendor=0x{:08X} product=0x{:08X})",
                    s.index, name,
                    s.vendor_id ? *s.vendor_id : 0,
                    s.product_code ? *s.product_code : 0);
        if (!el2004 && s.hasVendorAndProduct(kBeckhoffVendorId, kEL2004ProductCode)) {
            el2004 = &s;
        }
    }

    if (!el2004) {
        TETHER_LOGE(TAG, "No EL2004 found in the chain");
        master.stop();
        Tether::Examples::shutdownHostEthernet(session);
        return 6;
    }

    const uint16_t el_idx = el2004->index;
    TETHER_LOGI(TAG, "{}: selected EL2004{}",
                master.slaveLogPrefix(el_idx).c_str(),
                el2004->device_name
                    ? std::format(" ('{}')", *el2004->device_name).c_str()
                    : "");

    // ---- Determine the process-data output SM from SII ----
    // The SM category is positional: entry index == SM channel.  EL2004 has
    // exactly one SM (channel 0, outputs @ 0x0F00) and no mailbox SMs.
    uint8_t  sm_channel = kFallbackSmChannel;
    uint16_t sm_addr    = kFallbackSmAddr;
    uint16_t sm_len     = kFallbackSmLen;
    uint8_t  sm_ctrl    = kFallbackSmCtrl;

    if (el2004->sync_managers && !el2004->sync_managers->empty()) {
        bool found = false;
        for (size_t i = 0; i < el2004->sync_managers->size() && i < 4; ++i) {
            const auto& sm = (*el2004->sync_managers)[i];
            if (sm.sm_type == EtherCAT::SII::SM_TYPE_PROCESS_OUT) {
                sm_channel = static_cast<uint8_t>(i);
                sm_addr    = sm.phys_start_address;
                sm_len     = sm.length;
                sm_ctrl    = std::bit_cast<uint8_t>(sm.control_register);
                found = true;
                break;
            }
        }
        if (!found) {
            TETHER_LOGW(TAG, "{}: SII lists no process-output SM — using ESI defaults",
                        master.slaveLogPrefix(el_idx).c_str());
        }
    } else {
        TETHER_LOGW(TAG, "{}: SII SM data unavailable — using ESI defaults",
                    master.slaveLogPrefix(el_idx).c_str());
    }

    // EL terminals report SM length 0 in SII — the real process-data size is
    // the sum of the RxPDO entry bit lengths (4 x 1 bit = 1 byte for EL2004).
    if (sm_len == 0 && el2004->rx_pdos) {
        size_t total_bits = 0;
        for (const auto& pdo : *el2004->rx_pdos) {
            total_bits += pdo.totalBits();
        }
        sm_len = static_cast<uint16_t>((total_bits + 7) / 8);
    }
    if (sm_len == 0) sm_len = kFallbackSmLen;
    TETHER_LOGI(TAG, "{}: output SM channel {} @ 0x{:04X} len {} ctrl 0x{:02X}",
                master.slaveLogPrefix(el_idx).c_str(), sm_channel, sm_addr,
                sm_len, sm_ctrl);

    auto& sl = master.slave(el_idx);

    // EL2004 has no mailbox — satisfy the PRE_OP gate without touching SM0/SM1.
    sl.assumeMailboxAlreadyConfigured();

    // Register the real process-data SM under its actual channel index so
    // configureSlavesSMs() writes the matching ESC register block
    // (0x0800 + channel*8).  sm[2] receives only the physical base address as
    // bookkeeping for finalizeMapping()'s RxPDO offset; its type stays Unused
    // so no SM2 registers are written (the EL2004 has no SM2).
    auto* cfgs = master.pdo().slaveConfigs();
    if (el_idx >= EtherCAT::PDO::kMaxPDOSlaves) {
        TETHER_LOGE(TAG, "Slave index {} exceeds max {}", el_idx,
                    EtherCAT::PDO::kMaxPDOSlaves);
        master.stop();
        Tether::Examples::shutdownHostEthernet(session);
        return 7;
    }
    cfgs[el_idx].sm[sm_channel] =
        EtherCAT::PDO::SyncManagerConfig::process_output(sm_addr, sm_len);
    cfgs[el_idx].sm[sm_channel].control =
        std::bit_cast<EtherCAT::SyncManager::SMControlReg>(sm_ctrl);
    if (sm_channel != 2) {
        cfgs[el_idx].sm[2].phys_start_addr = sm_addr;
    }
    if (!master.pdo().configureSlavesSMs(el_idx)) {
        TETHER_LOGE(TAG, "{}: failed to write SM{} registers",
                    master.slaveLogPrefix(el_idx).c_str(), sm_channel);
        master.stop();
        Tether::Examples::shutdownHostEthernet(session);
        return 8;
    }

    auto pre_err = sl.transitionToPreOp();
    if (pre_err != EtherCAT::SlaveError::Ok) {
        TETHER_LOGE(TAG, "{}: PRE-OP transition failed: {}",
                    master.slaveLogPrefix(el_idx).c_str(),
                    EtherCAT::slaveErrorToString(pre_err));
        master.stop();
        Tether::Examples::shutdownHostEthernet(session);
        return 9;
    }
    TETHER_LOGI(TAG, "{}: in PRE-OP", master.slaveLogPrefix(el_idx).c_str());

    // ---- Register the output PDO buffer ----
    // All four channels share one byte in the SM buffer; bit N = channel N+1.
    // Position addressing (APWR straight into 0x0F00) needs no FMMU.
    static uint8_t rxpdo_buf = 0;
    int pdo_entry = master.pdo().mapping().add_rxpdo(
        el_idx, &rxpdo_buf, sm_len, 0x1600,
        EtherCAT::PDO::PDOAddressMode::Position);
    if (pdo_entry < 0) {
        TETHER_LOGE(TAG, "{}: failed to register RxPDO buffer",
                    master.slaveLogPrefix(el_idx).c_str());
        master.stop();
        Tether::Examples::shutdownHostEthernet(session);
        return 10;
    }
    master.pdo().finalizeMapping(el_idx);

    sl.assumePDOAlreadyConfigured();
    auto safe_err = sl.transitionToSafeOp();
    if (safe_err != EtherCAT::SlaveError::Ok) {
        TETHER_LOGE(TAG, "{}: SAFE-OP transition failed: {}",
                    master.slaveLogPrefix(el_idx).c_str(),
                    EtherCAT::slaveErrorToString(safe_err));
        master.stop();
        Tether::Examples::shutdownHostEthernet(session);
        return 11;
    }
    TETHER_LOGI(TAG, "{}: in SAFE-OP", master.slaveLogPrefix(el_idx).c_str());

    // ---- Start the cyclic PDO exchange ----
    // Bit pattern currently requested by the UI thread; the RT callback copies
    // it into the PDO buffer so the UI never touches RT-owned memory.
    std::atomic<uint8_t> desired_outputs{0x01};  // start with channel 1 ON
    std::atomic<uint64_t> cycle_count{0};
    std::atomic<bool> pdo_ok{true};

    master.setMotionControlCallback(
        [&master, &desired_outputs, &cycle_count, &pdo_ok](double) -> bool {
            rxpdo_buf = desired_outputs.load(std::memory_order_relaxed);
            if (!master.pdo().exchangeAll()) {
                pdo_ok.store(false, std::memory_order_relaxed);
                return true;
            }
            pdo_ok.store(true, std::memory_order_relaxed);
            cycle_count.fetch_add(1, std::memory_order_relaxed);
            return true;
        });

    EtherCAT::Master::RealtimeMotionLoopConfig loop_cfg;
    loop_cfg.cycle_period_us = 1000;
    loop_cfg.enable_dc_synchronization = false;
    if (!master.startRealtimeMotionControlLoop(loop_cfg)) {
        TETHER_LOGE(TAG, "Failed to start realtime loop");
        master.stop();
        Tether::Examples::shutdownHostEthernet(session);
        return 12;
    }

    // ---- Request OP ----
    // Bypass Slave::transitionToOp(): its PDO request+reply counter check can
    // never pass for an output-only slave (no TxPDO -> reply count stays 0).
    if (!master.requestSlaveApplicationLayerState(
            EtherCAT::SlaveAddress(el_idx),
            static_cast<uint8_t>(EtherCAT::SlaveState::OP) | 0x10)) {
        TETHER_LOGE(TAG, "{}: OP request failed (transport)",
                    master.slaveLogPrefix(el_idx).c_str());
        master.stopMotionControlLoop();
        master.stop();
        Tether::Examples::shutdownHostEthernet(session);
        return 13;
    }
    if (!waitAlState(master, el_idx, EtherCAT::SlaveState::OP, 5000)) {
        uint16_t al_code = 0;
        sl.readALStatusCode(al_code);
        TETHER_LOGE(TAG, "{}: OP not confirmed after 5 s (AL status: {} 0x{:04X})",
                    master.slaveLogPrefix(el_idx).c_str(),
                    EtherCAT::getALStatusCodeName(al_code), al_code);
        master.stopMotionControlLoop();
        master.stop();
        Tether::Examples::shutdownHostEthernet(session);
        return 14;
    }
    TETHER_LOGI(TAG, "{}: in OP — outputs live", master.slaveLogPrefix(el_idx).c_str());

    // ---- UI loop: Enter steps the active channel ----
    std::cout << "\nEL2004 @ slave " << el_idx
              << " — exactly one output is ON.\n";
    size_t active = 0;
    printChannelState(active);

    const auto start_time = std::chrono::steady_clock::now();
    bool stdin_open = true;

    while (g_running.load()) {
        if (duration_sec > 0.0) {
            double elapsed = std::chrono::duration<double>(
                std::chrono::steady_clock::now() - start_time).count();
            if (elapsed >= duration_sec) break;
        }

        if (!pdo_ok.load(std::memory_order_relaxed)) {
            TETHER_LOGW(TAG, "PDO exchange error");
        }

        if (!stdin_open) {
            Tether::Platform::Clock::instance().delayMilliseconds(100);
            continue;
        }

        pollfd pfd{STDIN_FILENO, POLLIN, 0};
        int pr = ::poll(&pfd, 1, 100);
        if (pr <= 0) continue;
        if (pfd.revents & POLLIN) {
            std::string line;
            if (!std::getline(std::cin, line)) {
                stdin_open = false;  // piped/closed stdin — keep running
                continue;
            }
            active = (active + 1) % kEL2004Channels;
            desired_outputs.store(static_cast<uint8_t>(1u << active),
                                  std::memory_order_relaxed);
            printChannelState(active);
        } else if (pfd.revents & (POLLERR | POLLHUP | POLLNVAL)) {
            stdin_open = false;
        }
    }

    // ---- Shutdown: all outputs off, then stop ----
    desired_outputs.store(0, std::memory_order_relaxed);
    Tether::Platform::Clock::instance().delayMilliseconds(20);  // a few cycles
    master.stopMotionControlLoop();
    master.stop();
    Tether::Examples::shutdownHostEthernet(session);

    TETHER_LOGI(TAG, "Done. {} PDO cycles, all outputs off.",
                static_cast<unsigned long long>(cycle_count.load()));
    return 0;
}

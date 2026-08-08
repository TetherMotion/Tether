/**
 * @file kinco_rp20_list_modules.cpp
 * @brief Kinco RP20 module discovery demo
 *
 * Auto-discovers RP20 I/O modules across all discovered EtherCAT slaves
 * and slots 0–15, then prints the list of found modules and exits.
 * No CoE init commands, PDO configuration, or cyclic I/O loop is run.
 *
 * Usage (Linux, requires root or CAP_NET_RAW):
 *   ./kinco_rp20_list_modules              # uses eth0
 *   ./kinco_rp20_list_modules -i enp3s0    # specify interface
 */

#include <atomic>
#include <csignal>
#include <cstdint>
#include <iomanip>
#include <iostream>
#include <optional>
#include <sstream>
#include <string>
#include <vector>

#include "tether/ethercat/Master.hpp"
#include "tether/ethercat/Slave.hpp"
#include "tether/ethercat/CoEManager.hpp"
#include "tether/ethercat/ESIFile.hpp"
#include "tether/platform/EspCompat.hpp"
#include "tether/platform/Platform.hpp"

#include "tether/drives/RP20/RP20Module.hpp"

#include "common/ExampleHelpers.hpp"
#include "common/EtherCATHostSetup.hpp"

static const char* TAG = "kinco_rp20_list_modules";
static EtherCAT::Master* g_master = nullptr;

namespace RP20Mod = ::EtherCAT::Drives::RP20Module;

// ============================================================================
// Discovered module info
// ============================================================================

struct DiscoveredModule {
    uint16_t slave_index;
    uint8_t  slot;
    uint8_t  module_id;
    const RP20Mod::ModuleDescriptor* descriptor;
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
                TETHER_LOGW(TAG, "Slave %u slot %u: unknown module ID 0x%02X, skipping",
                            s, slot, module_id);
                continue;
            }

            TETHER_LOGI(TAG, "Slave %u slot %u: found %s (%s, ident=0x%02X)",
                        s, slot, desc->name, desc->module_class, module_id);

            DiscoveredModule mod;
            mod.slave_index = s;
            mod.slot = slot;
            mod.module_id = module_id;
            mod.descriptor = desc;
            modules.push_back(std::move(mod));
        }
    }

    return modules;
}

// ============================================================================
// Print the discovered module list
// ============================================================================

static void printModuleList(const std::vector<DiscoveredModule>& modules) {
    std::cout << "\n";
    std::cout << "=== Discovered RP20 modules (" << modules.size() << ") ===\n";
    std::cout << std::left
              << std::setw(8)  << "Slave"
              << std::setw(8)  << "Slot"
              << std::setw(10) << "Ident"
              << std::setw(24) << "Name"
              << "Class\n";
    std::cout << std::string(70, '-') << "\n";

    for (const auto& mod : modules) {
        const auto* desc = mod.descriptor;
        std::ostringstream ident;
        ident << "0x" << std::hex << std::setw(2) << std::setfill('0')
              << static_cast<int>(mod.module_id);
        std::cout << std::left
                  << std::setw(8)  << mod.slave_index
                  << std::setw(8)  << static_cast<int>(mod.slot)
                  << std::setw(10) << ident.str()
                  << std::setw(24) << desc->name
                  << desc->module_class << "\n";
    }
    std::cout << "\n";
    std::cout.flush();
}

// ============================================================================
// Main
// ============================================================================

int main(int argc, char** argv) {
    argparse::ArgumentParser program("kinco_rp20_list_modules");
    Tether::Examples::addInterfaceArg(program);
    Tether::Examples::addDebugArg(program);
    Tether::Examples::addVlanArgs(program);
    Tether::Examples::addMailboxSizeArg(program);
    Tether::Examples::addMailboxAddressArg(program);
    Tether::Examples::addEsiXmlArg(program);
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
            TETHER_LOGE(TAG, "Failed to parse ESI XML '%s': %s",
                        esi_xml.c_str(), esi->errorMessage().c_str());
            return 1;
        }
        TETHER_LOGI(TAG, "Loaded ESI XML '%s' (%zu device(s))",
                    esi_xml.c_str(), esi->devices().size());
    }
#endif

    TETHER_LOGI(TAG, "kinco_rp20_list_modules — interface: %s", iface.c_str());
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

    // ---- Print the list of discovered modules ----
    printModuleList(modules);

    // ---- Shutdown ----
    TETHER_LOGI(TAG, "Shutting down...");
    master.stop();
    g_master = nullptr;
    Tether::Examples::shutdownHostEthernet(session);

    TETHER_LOGI(TAG, "Done.");
    return 0;
}

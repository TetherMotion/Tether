/**
 * @file detect_slaves.cpp
 * @brief Minimal EtherCAT slave detection example
 *
 * Scans the bus, reports the number of slaves found,
 * and prints a brief identity / SII summary for each.
 *
 * Usage (Linux, requires root or CAP_NET_RAW):
 *   ./detect_slaves              # uses eth0
 *   ./detect_slaves -i enp3s0    # or: ./detect_slaves --interface enp3s0
 */

#include <memory>
#include <optional>
#include <set>
#include <sstream>

#include "tether/ethercat/Master.hpp"
#include "tether/ethercat/Slave.hpp"
#include "tether/ethercat/Types.hpp"
#include "tether/ethercat/SyncManager.hpp"
#include "tether/ethercat/ESIFile.hpp"
#include "tether/ethercat/ESIParser.hpp"
#include "tether/platform/EspCompat.hpp"
#include "tether/sii/SIIReader.hpp"
#include "tether/sii/SIIParser.hpp"

#include "common/ExampleHelpers.hpp"
#include "common/EtherCATHostSetup.hpp"

static const char* TAG = "detect_slaves";

int main(int argc, char** argv) {
    argparse::ArgumentParser program("detect_slaves");
    Tether::Examples::addInterfaceArg(program);
    Tether::Examples::addDebugArg(program);
    Tether::Examples::addVlanArgs(program);
    Tether::Examples::addMailboxSizeArg(program);
    Tether::Examples::addMailboxAddressArg(program);
    Tether::Examples::addEsiXmlArg(program);

    try { program.parse_args(argc, argv); }
    catch (const std::runtime_error& err) {
        std::cerr << err.what() << "\n" << program;
        return 1;
    }

    std::string iface = Tether::Examples::resolveInterface(program.get<std::string>("--interface"), TAG);
    std::string debug_str = program.get<std::string>("--debug");
    std::string esi_xml = program.get<std::string>("--esi-xml");
#if !TETHER_HAVE_ESI
    if (!esi_xml.empty()) {
        std::cerr << "ESI support not compiled in (TETHER_BUILD_EXTRACT_ESI=OFF). "
                     "Cannot use --esi-xml.\n";
        return 1;
    }
#else
    std::optional<EtherCAT::ESIFile> esi;
    if (!esi_xml.empty()) {
        esi.emplace(esi_xml);
        if (esi->empty()) {
            TETHER_LOGE(TAG, "Failed to parse ESI XML '%s': %s",
                        esi_xml.c_str(), esi->errorMessage().c_str());
            return 1;
        }
        TETHER_LOGI(TAG, "Loaded ESI XML '%s' (%zu device(s)) for cross-reference",
                    esi_xml.c_str(), esi->devices().size());
    }
#endif

    if (Tether::Examples::printDebugHelpIfRequested(debug_str)) return 0;
    auto debug_flags = Tether::Examples::parseDebugFlags(debug_str);

    Tether::Examples::VlanConfig vlan;
    if (!Tether::Examples::parseVlanArgs(
            program.get<std::string>("--rx-vlan"),
            program.get<std::string>("--tx-vlan"),
            vlan, TAG)) {
        return 1;
    }

    TETHER_LOGI(TAG, "detect_slaves (host) — interface: %s", iface.c_str());
    if (!debug_flags.empty()) {
        TETHER_LOGI(TAG, "Debug flags: %s", debug_str.c_str());
    }
    Tether::Examples::logVlanConfig(vlan, TAG);

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
        TETHER_LOGW(TAG, "No slaves discovered");
    }

    uint16_t slaves = master.getDiscoveredSlaveCount();
    TETHER_LOGI(TAG, "=== Discovered %u slave(s) ===", slaves);
    master.logDiscoveredSlavesSummary(TAG);

    if (debug_flags.count("sii-derivation") && slaves > 0) {
        TETHER_LOGI(TAG, "\n=== SII Mailbox Derivation Debug ===");
        for (uint16_t i = 0; i < slaves; i++) {
            EtherCAT::SII::debugSIIMailboxDerivation(master, i, TAG);
        }
    }

    if (debug_flags.count("mailbox-configuration") && slaves > 0) {
        TETHER_LOGI(TAG, "\n=== Mailbox Hardware Configuration Debug ===");
        for (uint16_t i = 0; i < slaves; i++) {
            EtherCAT::debugMailboxConfiguration(master, i, TAG);
        }
    }

    if (debug_flags.count("pdo-sm") && slaves > 0) {
        TETHER_LOGI(TAG, "\n=== PDO Sync Manager Configuration Debug ===");
        for (uint16_t i = 0; i < slaves; i++) {
            EtherCAT::debugPDOSyncManagerConfiguration(master, i, TAG);
        }
    }

    if (slaves == 0) {
        TETHER_LOGW(TAG, "No slaves found — check wiring, power, and interface name");
    }

    // Print ESI device info for cross-reference if --esi-xml was provided
#if TETHER_HAVE_ESI
    if (esi && !esi->empty()) {
        TETHER_LOGI(TAG, "\n=== ESI XML Cross-Reference (%s) ===", esi_xml.c_str());
        for (const auto& dev : esi->devices()) {
            TETHER_LOGI(TAG, "%s",
                        EtherCAT::ESI::formatDeviceHumanReadable(dev, true).c_str());
        }
    }
#endif

    master.stop();
    Tether::Examples::shutdownHostEthernet(session);

    return (slaves > 0) ? 0 : 4;
}

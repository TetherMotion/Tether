/**
 * @file ethercat_dump_sii.cpp
 * @brief Example: dump parsed SII for a slave index (host + embedded)
 *
 * Usage (host build):
 *   ./ethercat_dump_sii -i eth0                      # dump slave 0
 *   ./ethercat_dump_sii -s 1 -i eth0                  # dump slave 1 (or use --interface)
 *   ./ethercat_dump_sii -i enp3s0.1999 --rx-vlan any  # VLAN catch-all mode
 *   ./ethercat_dump_sii -i eth0 --debug rx-ethercat-packets,tx-ethercat-packets
 */

#include <cstdint>
#include <cstring>
#include <optional>
#include <string>

#include "tether/ethercat/Diagnostics.hpp"
#include "tether/ethercat/Master.hpp"
#include "tether/ethercat/ESIFile.hpp"
#include "tether/ethercat/ESIParser.hpp"
#include "tether/platform/EspCompat.hpp"

#include "common/ExampleHelpers.hpp"
#include "common/EtherCATHostSetup.hpp"

static const char* TAG = "ethercat_dump_sii";

static int dumpSiiForSlave(EtherCAT::Master& master, uint16_t slave_idx) {
    uint16_t slaves = master.getDiscoveredSlaveCount();
    if (slave_idx >= slaves) {
        TETHER_LOGE(TAG, "Requested slave index %u >= discovered slaves (%u)", slave_idx, slaves);
        return 2;
    }

    return EtherCAT::Diagnostics::logParsedSlaveSII(master, slave_idx, TAG) ? 0 : 3;
}

int main(int argc, char** argv) {
    argparse::ArgumentParser program("ethercat_dump_sii");
    Tether::Examples::addInterfaceArg(program);
    Tether::Examples::addSlaveArg(program);
    Tether::Examples::addVlanArgs(program);
    Tether::Examples::addDebugArg(program);
    Tether::Examples::addMailboxSizeArg(program);
    Tether::Examples::addMailboxAddressArg(program);
    Tether::Examples::addEsiXmlArg(program);

    try { program.parse_args(argc, argv); }
    catch (const std::runtime_error& err) {
        std::cerr << err.what() << std::endl;
        std::cerr << program;
        return 1;
    }

    std::string iface = program.get<std::string>("--interface");
    int slave_idx = program.get<int>("--slave");
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

    TETHER_LOGI(TAG, "ethercat_dump_sii (host)\nNetwork interface: %s", iface.c_str());
    if (!debug_flags.empty()) {
        TETHER_LOGI(TAG, "Debug flags: %s", debug_str.c_str());
    }
    Tether::Examples::logVlanConfig(vlan, TAG);

    Tether::Examples::HostEtherNetSession session;
    if (!Tether::Examples::initHostEthernet(session, iface, TAG)) {
        return 2;
    }

    EtherCAT::Master::Config mcfg;
    mcfg.enable_mailbox_fallback = true;
    EtherCAT::Master master(mcfg);
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
    TETHER_LOGI(TAG, "Discovered %u slave(s)", slaves);
    if (slaves == 0) {
        Tether::Examples::shutdownHostEthernet(session);
        return 5;
    }

    int rc = dumpSiiForSlave(master, static_cast<uint16_t>(slave_idx));

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

    Tether::Examples::shutdownHostEthernet(session);

    return rc;
}

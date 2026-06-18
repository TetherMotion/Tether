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

#include "tether/ethercat/Diagnostics.hpp"
#include "tether/ethercat/Master.hpp"
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

    try { program.parse_args(argc, argv); }
    catch (const std::runtime_error& err) {
        std::cerr << err.what() << std::endl;
        std::cerr << program;
        return 1;
    }

    std::string iface = program.get<std::string>("--interface");
    int slave_idx = program.get<int>("--slave");
    std::string debug_str = program.get<std::string>("--debug");

    auto debug_flags = Tether::Examples::parseDebugFlags(debug_str);
    Tether::Examples::applyDebugFlags(debug_flags, &Tether::Examples::allKnownDebugFlags(), TAG);

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

    Tether::Examples::shutdownHostEthernet(session);

    return rc;
}

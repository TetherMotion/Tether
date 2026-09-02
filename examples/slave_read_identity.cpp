/**
 * @file slave_read_identity.cpp
 * @brief Read slave identity object (0x1018) via CoE/SDO
 *
 * Discovers an EtherCAT slave, configures the mailbox from SII,
 * transitions to PRE-OP, and reads all subindexes of the identity
 * object (0x1018), printing the results.
 *
 * Usage (Linux, requires root or CAP_NET_RAW):
 *   ./slave_read_identity              # uses eth0, slave 0
 *   ./slave_read_identity -i enp3s0  # specify interface
 *   ./slave_read_identity -s 1         # specify slave index
 */

#include <cstring>
#include <iomanip>
#include <iostream>
#include <optional>
#include <memory>
#include <string>

#include "tether/ethercat/Master.hpp"
#include "tether/ethercat/Slave.hpp"
#include "tether/ethercat/Types.hpp"
#include "tether/ethercat/SyncManager.hpp"
#include "tether/ethercat/ESIFile.hpp"
#include "tether/sii/SIIReader.hpp"
#include "tether/sii/SIIParser.hpp"

#include "common/ExampleHelpers.hpp"
#include "common/EtherCATHostSetup.hpp"

static const char* TAG = "slave_read_identity";

int main(int argc, char** argv) {
    argparse::ArgumentParser program("slave_read_identity");
    Tether::Examples::addInterfaceArg(program);
    Tether::Examples::addSlaveArg(program);
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
    int slave_idx = program.get<int>("--slave");

    if (slave_idx < 0 || slave_idx > 65535) {
        std::cerr << "Invalid slave index\n";
        return 1;
    }

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

    TETHER_LOGI(TAG, "slave_read_identity — interface: {}, target slave: {}",
                iface.c_str(), slave_idx);
    if (!use_esi) {
        Tether::Examples::logMailboxConfig(mbSize, mbAddr, TAG);
    }

    Tether::Examples::HostEtherNetSession session;
    if (!Tether::Examples::initHostEthernet(session, iface, TAG)) {
        return 2;
    }

    EtherCAT::Master master;
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

    uint16_t slaves = master.getDiscoveredSlaveCount();
    TETHER_LOGI(TAG, "Discovered {} slave(s)", slaves);

    if (static_cast<uint16_t>(slave_idx) >= slaves) {
        TETHER_LOGE(TAG, "Slave index {} out of range (only {} slave(s) found)",
                    slave_idx, slaves);
        master.stop();
        Tether::Examples::shutdownHostEthernet(session);
        return 5;
    }

    auto& sl = master.slave(static_cast<uint16_t>(slave_idx));

    TETHER_LOGI(TAG, "Configuring mailbox for slave {}...", slave_idx);
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
        TETHER_LOGE(TAG, "Mailbox configuration failed: {}",
                    EtherCAT::slaveErrorToString(mb_err));
        master.stop();
        Tether::Examples::shutdownHostEthernet(session);
        return 7;
    }

    auto pre_err = sl.transitionToPreOp();
    if (pre_err != EtherCAT::SlaveError::Ok) {
        TETHER_LOGE(TAG, "PRE-OP transition failed: {}",
                    EtherCAT::slaveErrorToString(pre_err));
        master.stop();
        Tether::Examples::shutdownHostEthernet(session);
        return 8;
    }
    TETHER_LOGI(TAG, "Slave {} is in PRE-OP", slave_idx);

    constexpr uint16_t kIdentityIndex = 0x1018;

    uint8_t num_entries = 0;
    auto err = sl.sdoReadU8(kIdentityIndex, 0, num_entries);
    if (err != EtherCAT::SlaveError::Ok) {
        TETHER_LOGE(TAG, "Failed to read subindex 0 of 0x{:04X}", kIdentityIndex);
        master.stop();
        Tether::Examples::shutdownHostEthernet(session);
        return 9;
    }

    std::cout << "\n=== Identity Object (0x" << std::hex << kIdentityIndex << std::dec << ") ===\n";
    std::cout << "Number of entries (subindex 0): " << static_cast<int>(num_entries) << "\n\n";

    bool identity_ok = true;

    for (uint8_t sub = 1; sub <= num_entries; ++sub) {
        uint32_t value = 0;
        err = sl.sdoReadU32(kIdentityIndex, sub, value);
        if (err != EtherCAT::SlaveError::Ok) {
            TETHER_LOGE(TAG, "Failed to read subindex {} of 0x{:04X}", sub, kIdentityIndex);
            identity_ok = false;
            std::cout << "Subindex " << static_cast<int>(sub) << ": [READ FAILED]\n";
            continue;
        }

        const char* label = nullptr;
        switch (sub) {
            case 1: label = "Vendor ID";       break;
            case 2: label = "Product Code";      break;
            case 3: label = "Revision Number";   break;
            case 4: label = "Serial Number";     break;
            default: label = "Reserved";         break;
        }

        std::cout << "Subindex " << static_cast<int>(sub) << " — " << label << ":\n";
        std::cout << "  Decimal: " << value << "\n";
        std::cout << "  Hex:     0x" << std::hex << std::setw(8) << std::setfill('0')
                  << value << std::dec << "\n\n";
    }
    std::cout.flush();

    master.stop();
    Tether::Examples::shutdownHostEthernet(session);

    return identity_ok ? 0 : 10;
}

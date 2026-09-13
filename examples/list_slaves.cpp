/**
 * @file list_slaves.cpp
 * @brief Minimal EtherCAT slave detection example
 *
 * Scans the bus, reports the number of slaves found,
 * and prints a brief identity / SII summary for each.
 *
 * Usage (Linux, requires root or CAP_NET_RAW):
 *   ./list_slaves              # uses eth0
 *   ./list_slaves -i enp3s0    # or: ./list_slaves --interface enp3s0
 */

#include <memory>
#include <optional>
#include <set>
#include <sstream>
#include <string>
#include <format>

#include "tether/ethercat/Master.hpp"
#include "tether/ethercat/Slave.hpp"
#include "tether/ethercat/SlaveDiscoveryManager.hpp"
#include "tether/ethercat/Types.hpp"
#include "tether/ethercat/SyncManager.hpp"
#include "tether/ethercat/ESIFile.hpp"
#include "tether/ethercat/ESIParser.hpp"
#include "tether/platform/EspCompat.hpp"
#include "tether/sii/SIIReader.hpp"
#include "tether/sii/SIIParser.hpp"

#include "common/ExampleHelpers.hpp"
#include "common/EtherCATHostSetup.hpp"

static const char* TAG = "list_slaves";

int main(int argc, char** argv) {
    argparse::ArgumentParser program("list_slaves");
    Tether::Examples::addInterfaceArg(program);
    Tether::Examples::addListInterfacesArg(program);
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

    if (program.get<bool>("--list-interfaces")) {
        Tether::Examples::listPhysicalInterfaces(TAG);
        return 0;
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
            TETHER_LOGE(TAG, "Failed to parse ESI XML '{}': {}",
                        esi_xml.c_str(), esi->errorMessage().c_str());
            return 1;
        }
        TETHER_LOGI(TAG, "Loaded ESI XML '{}' ({} device(s)) for cross-reference",
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

    TETHER_LOGI(TAG, "list_slaves (host) — interface: {}", iface.c_str());
    if (!debug_flags.empty()) {
        TETHER_LOGI(TAG, "Debug flags: {}", debug_str.c_str());
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

    // Synchronous blocking discovery of all slave information.
    auto slaves = master.discovery().discover();
    if (slaves.empty()) {
        TETHER_LOGW(TAG, "No slaves discovered");
    }

    uint16_t slave_count = master.getDiscoveredSlaveCount();
    TETHER_LOGI(TAG, "=== Discovered {} slave(s) ===", slave_count);
    for (const auto& s : slaves) {
        const char* name = s.device_name ? s.device_name->c_str() : "Unknown";

        // First line: identity (ANSI bold for slave index and device name)
        TETHER_LOGI(TAG, "\033[1mSlave {}\033[0m: \033[1m{}\033[0m (Vendor=0x{:08X} Product=0x{:08X})",
                    s.index,
                    name,
                    s.vendor_id ? *s.vendor_id : 0,
                    s.product_code ? *s.product_code : 0);

        // Revision / serial
        if (s.revision_number || s.serial_number) {
            TETHER_LOGI(TAG, "  Rev=0x{:08X} Serial=0x{:08X}",
                        s.revision_number ? *s.revision_number : 0,
                        s.serial_number ? *s.serial_number : 0);
        }

        // Configured address
        if (s.configured_address) {
            TETHER_LOGI(TAG, "  Configured address: 0x{:04X}", *s.configured_address);
        }

        // Mailbox protocols
        if (s.mailbox_protocols && *s.mailbox_protocols != 0) {
            uint16_t prot = *s.mailbox_protocols;
            std::string prot_str;
            if (prot & EtherCAT::SII::MBX_PROTO_AOE) prot_str += "AoE ";
            if (prot & EtherCAT::SII::MBX_PROTO_EOE) prot_str += "EoE ";
            if (prot & EtherCAT::SII::MBX_PROTO_COE) prot_str += "CoE ";
            if (prot & EtherCAT::SII::MBX_PROTO_FOE) prot_str += "FoE ";
            if (prot & EtherCAT::SII::MBX_PROTO_SOE) prot_str += "SoE ";
            if (prot & EtherCAT::SII::MBX_PROTO_VOE) prot_str += "VoE ";
            if (!prot_str.empty() && prot_str.back() == ' ') prot_str.pop_back();
            TETHER_LOGI(TAG, "  Mailbox protocols: {}", prot_str.c_str());
        }

#if TETHER_ENABLE_SII
        // Mailbox configuration (only if mailbox is actually supported)
        if (s.mailbox_config &&
            (s.mailbox_config->std_rx_size > 0 || s.mailbox_config->std_tx_size > 0)) {
            const auto& mb = *s.mailbox_config;
            TETHER_LOGI(TAG, "  Mailbox: std RX@0x{:04X} ({}B), TX@0x{:04X} ({}B)",
                        mb.std_rx_offset, mb.std_rx_size,
                        mb.std_tx_offset, mb.std_tx_size);
        }

        // Sync managers
        if (s.sync_managers && !s.sync_managers->empty()) {
            TETHER_LOGI(TAG, "  Sync managers: {}", s.sync_managers->size());
            for (const auto& sm : *s.sync_managers) {
                TETHER_LOGI(TAG, "    SM{}: {} start=0x{:04X} len={}",
                            sm.sm_type, sm.getTypeName(),
                            sm.phys_start_address, sm.length);
            }
        }

        // FMMUs
        if (s.fmmus && !s.fmmus->empty()) {
            TETHER_LOGI(TAG, "  FMMUs: {}", s.fmmus->size());
        }

        // PDOs
        if ((s.tx_pdos && !s.tx_pdos->empty()) ||
            (s.rx_pdos && !s.rx_pdos->empty())) {
            size_t tx_count = s.tx_pdos ? s.tx_pdos->size() : 0;
            size_t rx_count = s.rx_pdos ? s.rx_pdos->size() : 0;
            TETHER_LOGI(TAG, "  PDOs: TxPDO={} RxPDO={}", tx_count, rx_count);
        }

        // Distributed clocks
        if (s.dc_configs && !s.dc_configs->empty()) {
            TETHER_LOGI(TAG, "  Distributed clocks: {} config(s)", s.dc_configs->size());
        }

        // General info
        if (s.general_info) {
            const auto& gi = *s.general_info;
            TETHER_LOGI(TAG, "  Physical ports: 0x{:04X}", gi.phys_port);
            if (gi.current_ebus != 0) {
                TETHER_LOGI(TAG, "  E-Bus current: {} mA", gi.current_ebus);
            }

            // Profile / protocol details from SII general category
            std::string profiles;
            if (gi.coe_details) {
                profiles += "CoE";
                if (gi.coeEnableSdo())        profiles += "(SDO)";
                if (gi.coeEnableSdoInfo())   profiles += "(SDOinfo)";
                if (gi.coeEnablePdoAssign()) profiles += "(PDOassign)";
                if (gi.coeEnablePdoConfig()) profiles += "(PDOconfig)";
                if (gi.coeEnableUploadStartup()) profiles += "(Upload)";
                if (gi.coeEnableSdoComplete())  profiles += "(SDOcomplete)";
                profiles += " ";
            }
            if (gi.foe_details) profiles += "FoE ";
            if (gi.eoe_details) profiles += "EoE ";
            if (gi.soe_channels) {
                profiles += std::format("SoE({}ch) ", gi.soe_channels);
            }
            if (gi.ds402_channels) {
                profiles += std::format("DS402({}ch) ", gi.ds402_channels);
            }
            if (!profiles.empty()) {
                TETHER_LOGI(TAG, "  Profile support: {}", profiles.c_str());
            }

            // General flags
            std::string flags_str;
            if (gi.enableNotLRW()) flags_str += "noLRW ";
            if (gi.enableSafeOp()) flags_str += "SafeOp ";
            if (gi.enableLRW())    flags_str += "LRW ";
            if (!flags_str.empty()) {
                TETHER_LOGI(TAG, "  Flags: {}", flags_str.c_str());
            }
        }
#endif

        // Physical ports (from EtherCAT registers, not SII)
        if (s.physical_ports) {
            TETHER_LOGI(TAG, "  Physical ports (reg): 0x{:04X}", *s.physical_ports);
        }

        // EEPROM size
        if (s.eeprom_size_kbits) {
            TETHER_LOGI(TAG, "  EEPROM size: {} kbits", *s.eeprom_size_kbits);
        }
    }

    if (debug_flags.count("sii-derivation") && slave_count > 0) {
        TETHER_LOGI(TAG, "\n=== SII Mailbox Derivation Debug ===");
        for (uint16_t i = 0; i < slave_count; i++) {
            EtherCAT::SII::debugSIIMailboxDerivation(master, i, TAG);
        }
    }

    if (debug_flags.count("mailbox-configuration") && slave_count > 0) {
        TETHER_LOGI(TAG, "\n=== Mailbox Hardware Configuration Debug ===");
        for (uint16_t i = 0; i < slave_count; i++) {
            EtherCAT::debugMailboxConfiguration(master, i, TAG);
        }
    }

    if (debug_flags.count("pdo-sm") && slave_count > 0) {
        TETHER_LOGI(TAG, "\n=== PDO Sync Manager Configuration Debug ===");
        for (uint16_t i = 0; i < slave_count; i++) {
            EtherCAT::debugPDOSyncManagerConfiguration(master, i, TAG);
        }
    }

    if (slave_count == 0) {
        TETHER_LOGW(TAG, "No slaves found — check wiring, power, and interface name");
    }

    // Print ESI device info for cross-reference if --esi-xml was provided
#if TETHER_HAVE_ESI
    if (esi && !esi->empty()) {
        TETHER_LOGI(TAG, "\n=== ESI XML Cross-Reference ({}) ===", esi_xml.c_str());
        for (const auto& dev : esi->devices()) {
            TETHER_LOGI(TAG, "{}",
                        EtherCAT::ESI::formatDeviceHumanReadable(dev, true).c_str());
        }
    }
#endif

    master.stop();
    Tether::Examples::shutdownHostEthernet(session);

    return (slave_count > 0) ? 0 : 4;
}

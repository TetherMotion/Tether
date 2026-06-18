#include "ExampleHelpers.hpp"

#include <iostream>
#include <sstream>

#include "tether/ethercat/Slave.hpp"
#include "tether/platform/Platform.hpp"

namespace Tether::Examples {

// ============================================================================
// Argument parsers
// ============================================================================

void addInterfaceArg(argparse::ArgumentParser& program,
                     const std::string& defaultValue) {
    program.add_argument("-i", "--interface")
        .default_value(defaultValue)
        .help("Network interface name (e.g. eth0, enp3s0)");
}

void addDebugArg(argparse::ArgumentParser& program) {
    program.add_argument("--debug")
        .default_value(std::string(""))
        .help("Comma-separated debug flags");
}

void addVlanArgs(argparse::ArgumentParser& program) {
    program.add_argument("--rx-vlan")
        .default_value(std::string(""))
        .help("RX VLAN filter: single VID, range, or 'any'");
    program.add_argument("--tx-vlan")
        .default_value(std::string(""))
        .help("TX VLAN encapsulation: single VID");
}

void addSlaveArg(argparse::ArgumentParser& program, int defaultValue) {
    program.add_argument("-s", "--slave")
        .scan<'i', int>()
        .default_value(defaultValue)
        .help("Slave index on the bus (0-based)");
}

void addDurationArg(argparse::ArgumentParser& program, double defaultValue) {
    program.add_argument("-t", "--time")
        .scan<'g', double>()
        .default_value(defaultValue)
        .help("Duration in seconds (0 = infinite until Ctrl-C)");
}

// ============================================================================
// Debug flags
// ============================================================================

std::set<std::string> parseDebugFlags(const std::string& debugStr) {
    std::set<std::string> flags;
    if (debugStr.empty()) return flags;

    std::stringstream ss(debugStr);
    std::string flag;
    while (std::getline(ss, flag, ',')) {
        flag.erase(0, flag.find_first_not_of(" \t"));
        flag.erase(flag.find_last_not_of(" \t") + 1);
        if (!flag.empty()) {
            flags.insert(flag);
        }
    }
    return flags;
}

const std::set<std::string>& allKnownDebugFlags() {
    static const std::set<std::string> kFlags = {
        "sii-derivation",
        "mailbox-configuration",
        "al-state",
        "tx-ethercat-packets",
        "rx-ethercat-packets",
        "rx-pdo",
        "tx-pdo",
        "dc",
        "fmmu",
        "sii-eeprom",
        "coe-reads",
        "coe-writes",
        "coe-rx-packets",
        "coe-tx-packets"
    };
    return kFlags;
}

void applyDebugFlags(const std::set<std::string>& flags,
                     const std::set<std::string>* knownFlags,
                     const char* tag) {
    if (knownFlags) {
        std::set<std::string> unknown;
        for (const auto& f : flags) {
            if (knownFlags->find(f) == knownFlags->end()) {
                unknown.insert(f);
            }
        }
        if (!unknown.empty()) {
            TETHER_LOGW(tag, "Unknown debug flags:");
            for (const auto& f : unknown) {
                TETHER_LOGW(tag, "  - %s", f.c_str());
            }
            TETHER_LOGI(tag, "Known debug flags:");
            for (const auto& f : *knownFlags) {
                TETHER_LOGI(tag, "  - %s", f.c_str());
            }
        }
    }

    if (flags.count("al-state")) {
        EtherCAT::enableStateMachineDebug(true);
        TETHER_LOGI(tag, "EtherCAT state machine debug logging enabled");
    }
    if (flags.count("tx-ethercat-packets")) {
        EtherCAT::enableTxPacketDebug(true);
        TETHER_LOGI(tag, "TX EtherCAT packet debug logging enabled");
    }
    if (flags.count("rx-ethercat-packets")) {
        EtherCAT::enableRxPacketDebug(true);
        TETHER_LOGI(tag, "RX EtherCAT packet debug logging enabled");
    }
    if (flags.count("rx-pdo")) {
        EtherCAT::enableRxPDODebug(true);
        TETHER_LOGI(tag, "RxPDO debug logging enabled");
    }
    if (flags.count("tx-pdo")) {
        EtherCAT::enableTxPDODebug(true);
        TETHER_LOGI(tag, "TxPDO debug logging enabled");
    }
    if (flags.count("fmmu")) {
        EtherCAT::enableFmmuDebug(true);
        TETHER_LOGI(tag, "FMMU debug logging enabled");
    }
    if (flags.count("sii-eeprom")) {
        EtherCAT::enableSIIEEPROMDebug(true);
        TETHER_LOGI(tag, "SII/EEPROM debug logging enabled");
    }
    if (flags.count("coe-reads")) {
        EtherCAT::enableCoEReadsDebug(true);
        TETHER_LOGI(tag, "CoE read debug logging enabled");
    }
    if (flags.count("coe-writes")) {
        EtherCAT::enableCoEWritesDebug(true);
        TETHER_LOGI(tag, "CoE write debug logging enabled");
    }
    if (flags.count("coe-rx-packets")) {
        EtherCAT::enableCoERxPacketsDebug(true);
        TETHER_LOGI(tag, "CoE RX packet debug logging enabled");
    }
    if (flags.count("coe-tx-packets")) {
        EtherCAT::enableCoETxPacketsDebug(true);
        TETHER_LOGI(tag, "CoE TX packet debug logging enabled");
    }
}

// ============================================================================
// VLAN helpers
// ============================================================================

bool parseVlanArgs(const std::string& rxVlanStr,
                   const std::string& txVlanStr,
                   VlanConfig& out,
                   const char* /*tag*/) {
    out.enabled = !rxVlanStr.empty() || !txVlanStr.empty();
    out.txVlan = std::nullopt;
    out.rxAny = false;
    out.rxRange = std::nullopt;

    if (!out.enabled) return true;

    if (!txVlanStr.empty()) {
        try {
            int v = std::stoi(txVlanStr);
            if (v < 1 || v > 4095) {
                std::cerr << "--tx-vlan must be in range 1-4095\n";
                return false;
            }
            out.txVlan = static_cast<uint16_t>(v);
        } catch (...) {
            std::cerr << "Invalid --tx-vlan value: " << txVlanStr << "\n";
            return false;
        }
    }

    if (!rxVlanStr.empty()) {
        if (rxVlanStr == "any") {
            out.rxAny = true;
        } else {
            size_t dash = rxVlanStr.find('-');
            try {
                if (dash == std::string::npos) {
                    int v = std::stoi(rxVlanStr);
                    if (v < 1 || v > 4095) {
                        std::cerr << "--rx-vlan must be in range 1-4095\n";
                        return false;
                    }
                    out.rxRange = EtherCAT::VLANRouter::VLANRange{
                        static_cast<uint16_t>(v), static_cast<uint16_t>(v)};
                } else {
                    int start = std::stoi(rxVlanStr.substr(0, dash));
                    int end   = std::stoi(rxVlanStr.substr(dash + 1));
                    if (start < 1 || end > 4095 || start > end) {
                        std::cerr << "--rx-vlan range must be 1-4095 with start <= end\n";
                        return false;
                    }
                    out.rxRange = EtherCAT::VLANRouter::VLANRange{
                        static_cast<uint16_t>(start), static_cast<uint16_t>(end)};
                }
            } catch (...) {
                std::cerr << "Invalid --rx-vlan value: " << rxVlanStr << "\n";
                return false;
            }
        }
    }

    return true;
}

void logVlanConfig(const VlanConfig& config, const char* tag) {
    if (!config.enabled) return;

    if (config.rxAny) {
        TETHER_LOGI(tag, "VLAN mode: RX=any (undefined target), TX=%s",
                    config.txVlan ? std::to_string(*config.txVlan).c_str() : "none");
    } else if (config.rxRange) {
        TETHER_LOGI(tag, "VLAN mode: RX=%u-%u, TX=%s",
                    config.rxRange->start, config.rxRange->end,
                    config.txVlan ? std::to_string(*config.txVlan).c_str() : "none");
    } else {
        TETHER_LOGI(tag, "VLAN mode: RX=untagged, TX=%s",
                    config.txVlan ? std::to_string(*config.txVlan).c_str() : "none");
    }
}

} // namespace Tether::Examples

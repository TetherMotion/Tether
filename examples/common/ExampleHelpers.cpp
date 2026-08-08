#include "ExampleHelpers.hpp"

#include <cstdio>
#include <iostream>
#include <sstream>

#include "tether/ethercat/DebugFlags.hpp"
#include "tether/ethercat/DebugGate.hpp"
#include "tether/ethercat/Master.hpp"
#include "tether/ethercat/SDOErrorDecoder.hpp"
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
        .help("Comma-separated debug flags. Use '--debug help' for a list.");
}

bool printDebugHelpIfRequested(const std::string& debugStr) {
    if (debugStr != "help") return false;

    const auto& registry = EtherCAT::debug::allDebugFlags();
    std::cout << "Available debug flags:\n";
    for (const auto& info : registry) {
        std::cout << "  " << info.name << "\n      " << info.description << "\n";
    }
    std::cout << "\nFilter syntax:\n";
    std::cout << "  --debug flagname:(slaves:0,2,5),otherflag:(slaves:1-3)\n";
    std::cout << "  (default: pass-all for every flag)\n";
    return true;
}

void addDebugConditionArgs(argparse::ArgumentParser& program) {
    program.add_argument("--debug-start")
        .default_value(std::string(""))
        .help("Start debug output when condition fires. Use '--debug-start help' for syntax.");
    program.add_argument("--debug-stop")
        .default_value(std::string(""))
        .help("Stop debug output when condition fires. Use '--debug-start help' for syntax.");
}

bool printDebugConditionHelpIfRequested(const std::string& startStr) {
    if (startStr != "help") return false;
#if TETHER_DEBUG_GATE_ENABLED
    EtherCAT::DebugGate::printHelp();
#else
    std::cout << "Debug gate compiled out (TETHER_DEBUG_GATE_ENABLED=0).\n"
              << "Conditional debugging is not available in this build.\n";
#endif
    return true;
}

bool applyDebugGateConditions(const std::string& startCond,
                              const std::string& stopCond,
                              EtherCAT::Master& master,
                              const char* tag) {
    if (startCond.empty() && stopCond.empty()) return true;

#if !TETHER_DEBUG_GATE_ENABLED
    if (!startCond.empty() || !stopCond.empty()) {
        TETHER_LOGW(tag, "Debug gate compiled out (TETHER_DEBUG_GATE_ENABLED=0); "
                         "ignoring --debug-start/--debug-stop conditions.");
    }
    return true;
#else
    if (!startCond.empty()) {
        auto cond = EtherCAT::DebugGate::parseCondition(startCond);
        if (!cond) {
            TETHER_LOGE(tag, "Failed to parse --debug-start condition: '%s'", startCond.c_str());
            return false;
        }
        TETHER_LOGI(tag, "Debug gate: global start condition = '%s'", startCond.c_str());
        master.debugGate().addGlobalStart(std::move(cond));
    }

    if (!stopCond.empty()) {
        auto cond = EtherCAT::DebugGate::parseCondition(stopCond);
        if (!cond) {
            TETHER_LOGE(tag, "Failed to parse --debug-stop condition: '%s'", stopCond.c_str());
            return false;
        }
        TETHER_LOGI(tag, "Debug gate: global stop condition = '%s'", stopCond.c_str());
        master.debugGate().addGlobalStop(std::move(cond));
    }

    return true;
#endif
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

void applyDebugFlags(const std::set<std::string>& flags,
                     EtherCAT::Master& master,
                     const char* tag) {
    const auto& registry = EtherCAT::debug::allDebugFlags();

    // Build known-names set for unknown-flag detection.
    std::set<std::string> knownNames;
    for (const auto& info : registry) {
        knownNames.insert(info.name);
    }

    // Reconstruct the comma-separated spec string (preserves filter syntax).
    std::string spec;
    for (const auto& f : flags) {
        if (!spec.empty()) spec += ",";
        spec += f;
    }

    master.debugFlags().applyFromString(spec,
                                         static_cast<uint16_t>(master.getDiscoveredSlaveCount()),
                                         tag);

    // Warn about unknown flags.
    std::set<std::string> unknown;
    for (const auto& f : flags) {
        // Strip any filter syntax for name matching.
        std::string name = f;
        size_t colon = name.find(':');
        if (colon != std::string::npos) name = name.substr(0, colon);
        if (knownNames.find(name) == knownNames.end()) {
            unknown.insert(f);
        }
    }
    if (!unknown.empty()) {
        TETHER_LOGW(tag, "Unknown debug flags:");
        for (const auto& f : unknown) {
            TETHER_LOGW(tag, "  - %s", f.c_str());
        }
        TETHER_LOGI(tag, "Known debug flags:");
        for (const auto& f : knownNames) {
            TETHER_LOGI(tag, "  - %s", f.c_str());
        }
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

// ============================================================================
// Mailbox helpers
// ============================================================================

void addMailboxSizeArg(argparse::ArgumentParser& program) {
    program.add_argument("-M", "--mailbox-size")
        .default_value(std::string("256"))
        .help("Mailbox buffer size in bytes. Short form: a single number sets both MbxIn and MbxOut (e.g. -M 256). Long form: in:<size>,out:<size> to set independently (e.g. --mailbox-size in:256,out:512). Range: 1-65535. Default: 256.");
}

void addMailboxAddressArg(argparse::ArgumentParser& program) {
    program.add_argument("--mailbox-address")
        .default_value(std::string("in:0x1000,out:0x1200"))
        .help("Mailbox base addresses in hex. Format: in:<addr>,out:<addr> (e.g. in:0x1080,out:0x1400). The in-address and out-address must be different. Default: in:0x1000,out:0x1200.");
}

static bool parseHexOrDec(const std::string& s, int& out) {
    try {
        if (s.size() > 2 && (s.substr(0, 2) == "0x" || s.substr(0, 2) == "0X")) {
            out = std::stoi(s.substr(2), nullptr, 16);
        } else {
            out = std::stoi(s);
        }
        return true;
    } catch (...) {
        return false;
    }
}

bool parseMailboxSize(const std::string& str, MailboxSizeConfig& out) {
    // Short form: plain number (e.g. "256")
    if (str.find(':') == std::string::npos) {
        int val = 0;
        if (!parseHexOrDec(str, val) || val < 1 || val > 65535) {
            std::cerr << "Invalid --mailbox-size value: " << str << "\n";
            std::cerr << "Expected a number 1-65535 or in:<size>,out:<size>\n";
            return false;
        }
        out.inSize = static_cast<uint16_t>(val);
        out.outSize = static_cast<uint16_t>(val);
        return true;
    }

    // Long form: in:<size>,out:<size>
    size_t inPos = str.find("in:");
    size_t outPos = str.find("out:");
    if (inPos == std::string::npos || outPos == std::string::npos) {
        std::cerr << "Invalid --mailbox-size format: " << str << "\n";
        std::cerr << "Expected in:<size>,out:<size> or a single number\n";
        return false;
    }

    size_t inStart = inPos + 3;
    size_t inEnd = str.find(',', inStart);
    std::string inStr = str.substr(inStart, inEnd - inStart);

    size_t outStart = outPos + 4;
    std::string outStr = str.substr(outStart);

    int inVal = 0, outVal = 0;
    if (!parseHexOrDec(inStr, inVal) || inVal < 1 || inVal > 65535) {
        std::cerr << "Invalid --mailbox-size in-value: " << inStr << "\n";
        return false;
    }
    if (!parseHexOrDec(outStr, outVal) || outVal < 1 || outVal > 65535) {
        std::cerr << "Invalid --mailbox-size out-value: " << outStr << "\n";
        return false;
    }

    out.inSize = static_cast<uint16_t>(inVal);
    out.outSize = static_cast<uint16_t>(outVal);
    return true;
}

bool parseMailboxAddress(const std::string& str, MailboxAddressConfig& out) {
    size_t inPos = str.find("in:");
    size_t outPos = str.find("out:");
    if (inPos == std::string::npos || outPos == std::string::npos) {
        std::cerr << "Invalid --mailbox-address format: " << str << "\n";
        std::cerr << "Expected in:<hex>,out:<hex> (e.g. in:0x1000,out:0x1200)\n";
        return false;
    }

    size_t inStart = inPos + 3;
    size_t inEnd = str.find(',', inStart);
    std::string inStr = str.substr(inStart, inEnd - inStart);

    size_t outStart = outPos + 4;
    std::string outStr = str.substr(outStart);

    int inVal = 0, outVal = 0;
    if (!parseHexOrDec(inStr, inVal) || inVal < 0 || inVal > 65535) {
        std::cerr << "Invalid --mailbox-address in-value: " << inStr << "\n";
        return false;
    }
    if (!parseHexOrDec(outStr, outVal) || outVal < 0 || outVal > 65535) {
        std::cerr << "Invalid --mailbox-address out-value: " << outStr << "\n";
        return false;
    }

    if (inVal == outVal) {
        std::cerr << "--mailbox-address error: in-address (0x" << std::hex << inVal << ") must differ from out-address (0x" << outVal << ")" << std::dec << "\n";
        return false;
    }

    out.inAddress = static_cast<uint16_t>(inVal);
    out.outAddress = static_cast<uint16_t>(outVal);
    return true;
}

void logMailboxConfig(const MailboxSizeConfig& size,
                      const MailboxAddressConfig& addr,
                      const char* tag) {
    TETHER_LOGI(tag, "Mailbox config: MbxOut addr=0x%04X len=%u, MbxIn addr=0x%04X len=%u",
                addr.outAddress, size.outSize,
                addr.inAddress, size.inSize);
}

// ============================================================================
// ESI (EtherCAT Slave Information) XML helpers
// ============================================================================

void addEsiXmlArg(argparse::ArgumentParser& program,
                  const std::string& defaultValue) {
    program.add_argument("--esi-xml")
        .default_value(defaultValue)
        .help("Path to an ESI (EtherCAT Slave Information) XML file. "
              "When provided, mailbox and PDO configuration is read from "
              "the ESI file instead of SII EEPROM. Requires tether_esi "
              "library to be linked (TETHER_HAVE_ESI=1).");
}

// ============================================================================
// SDO abort reporting
// ============================================================================

uint32_t reportSdoAbort(const EtherCAT::Slave& slave, const char* tag) {
    const uint32_t abort_code = slave.lastSdoAbortCode();
    if (abort_code == 0) return 0;

    EtherCAT::Raw::SDOErrorDecoder decoder;
    const char* meaning = decoder.sdoAbortCodeStr(abort_code);

    const bool was_download = slave.lastSdoWasDownload();
    const size_t attempted_len = slave.lastSdoAttemptedLength();
    const char* op_str = was_download ? "download (write)" : "upload (read)";
    const char* len_str = was_download
                              ? "payload length sent to slave"
                              : "read buffer capacity offered";

    TETHER_LOGE(tag, "Slave rejected the SDO request: CoE abort code 0x%08X (%s). "
                     "Operation: %s, attempted %s: %zu bytes.",
                abort_code, meaning, op_str, len_str, attempted_len);

    // Always echo to stderr so the user sees it on the console even when log
    // output is redirected or silenced.
    std::fprintf(stderr,
                 "ERROR: Slave rejected the SDO request.\n"
                 "  CoE SDO abort code: 0x%08X (%s)\n"
                 "  Operation:          %s\n"
                 "  Attempted %s: %zu bytes\n",
                 abort_code, meaning,
                 op_str, len_str, attempted_len);

    // The length-mismatch family — the signature of the original
    // "slave rejects the write because the payload is the wrong size"
    // failure. Point the user at the object dictionary / ESI file.
    if (abort_code == 0x06070010 || abort_code == 0x06070012 ||
        abort_code == 0x06070013) {
        const char* which = (abort_code == 0x06070012)
                                ? "too high"
                                : (abort_code == 0x06070013) ? "too low"
                                                             : "mismatch";
        std::fprintf(stderr,
                     "  Cause: the payload size you sent (%zu bytes) does not match the size\n"
                     "         the slave expects for this object (length %s).\n"
                     "  Fix:   check the object dictionary / ESI (XML) file for the\n"
                     "         target index:subindex to find the correct data type\n"
                     "         and byte length, then send exactly that many bytes.\n",
                     attempted_len, which);
    }

    std::fflush(stderr);
    return abort_code;
}

} // namespace Tether::Examples

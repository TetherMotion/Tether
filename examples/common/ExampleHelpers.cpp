#include "ExampleHelpers.hpp"

#include <cstdio>
#include <cstdlib>
#include <format>
#include <iostream>
#include <sstream>

#include "tether/ethercat/DebugFlags.hpp"
#include "tether/ethercat/DebugGate.hpp"
#include "tether/ethercat/Master.hpp"
#include "tether/hal/IEthernet.hpp"
#include "tether/ethercat/SDOErrorDecoder.hpp"
#include "tether/ethercat/Slave.hpp"
#include "tether/hal/NetworkInterfaceEnumerator.hpp"
#include "tether/platform/Platform.hpp"

namespace Tether::Examples {

// ============================================================================
// Argument parsers
// ============================================================================

void addInterfaceArg(argparse::ArgumentParser& program,
                     const std::string& defaultValue) {
    program.add_argument("-i", "--interface")
        .default_value(defaultValue)
        .help("Network interface name (e.g. eth0, enp3s0). "
              "If omitted, auto-selects the sole physical Ethernet interface.");
    // Encapsulation arg is folded in here so every example that uses
    // addInterfaceArg() automatically gets --encapsulation.
    program.add_argument("--encapsulation")
        .default_value(std::string(""))
        .help("Frame encapsulation, comma-separated tokens: "
              "'raw' (default), 'vlan:<vid|lo-hi|any>' "
              "[+ 'vlantx:<vid|off>'], 'udp[:<port>]'. "
              "Examples: vlan:1999, udp, vlan:100,udp, vlan:100-200,vlantx:off");
}

std::string resolveInterface(const std::string& requested, const char* tag) {
    if (!requested.empty()) {
        return requested;
    }

    auto physIfaces = EtherCAT::HAL::getPhysicalEthernetInterfaces();
    if (physIfaces.size() == 1) {
        TETHER_LOGI(tag, "No --interface given; auto-selected '{}'",
                    physIfaces[0].name.c_str());
        return physIfaces[0].name;
    }

    if (physIfaces.empty()) {
        TETHER_LOGE(tag, "No --interface given and no physical Ethernet "
                         "interfaces found on this system");
    } else {
        std::string names;
        for (const auto& iface : physIfaces) {
            if (!names.empty()) names += ", ";
            names += iface.name;
        }
        TETHER_LOGE(tag, "No --interface given and {} physical Ethernet "
                         "interfaces found; please specify one with -i: {}",
                    physIfaces.size(), names.c_str());
    }
    std::fflush(stderr);
    std::exit(1);
}

void addListInterfacesArg(argparse::ArgumentParser& program) {
    program.add_argument("--list-interfaces")
        .default_value(false)
        .implicit_value(true)
        .help("List available physical Ethernet interfaces and exit");
}

void listPhysicalInterfaces(const char* tag) {
    auto ifaces = EtherCAT::HAL::getPhysicalEthernetInterfaces();
    if (ifaces.empty()) {
        TETHER_LOGW(tag, "No physical Ethernet interfaces found");
        return;
    }

    TETHER_LOGI(tag, "Available physical Ethernet interfaces:");
    for (const auto& iface : ifaces) {
        std::string mac_str = "n/a";
        if (iface.mac) {
            const auto& b = iface.mac->bytes;
            mac_str = std::format("{:02X}:{:02X}:{:02X}:{:02X}:{:02X}:{:02X}",
                                  b[0], b[1], b[2], b[3], b[4], b[5]);
        }

        const char* state = iface.isUp
            ? (iface.isRunning ? "up/running" : "up")
            : (iface.isRunning ? "running" : "down");

        TETHER_LOGI(tag, "  {:<15} type={:<9} state={:<12} mac={}",
                    iface.name.c_str(),
                    EtherCAT::HAL::interfaceTypeToString(iface.type),
                    state,
                    mac_str.c_str());
    }
}

void logPermissionDeniedError(const char* tag) {
    TETHER_LOGE(tag, "Permission denied  —  run via `runec <executable>` or "
                     "`sudo <executable>` (requires CAP_NET_RAW)");
    std::fprintf(stderr,
                 "ERROR: Permission denied opening network interface.\n"
                 "  Run via `runec <executable>` or `sudo <executable>`.\n"
                 "  runec grants CAP_NET_RAW/CAP_NET_ADMIN/CAP_SYS_NICE;\n"
                 "  install: sudo chown root:root runec && sudo chmod 4755 runec\n");
    std::fflush(stderr);
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
            TETHER_LOGE(tag, "Failed to parse --debug-start condition: '{}'", startCond.c_str());
            return false;
        }
        TETHER_LOGI(tag, "Debug gate: global start condition = '{}'", startCond.c_str());
        master.debugGate().addGlobalStart(std::move(cond));
    }

    if (!stopCond.empty()) {
        auto cond = EtherCAT::DebugGate::parseCondition(stopCond);
        if (!cond) {
            TETHER_LOGE(tag, "Failed to parse --debug-stop condition: '{}'", stopCond.c_str());
            return false;
        }
        TETHER_LOGI(tag, "Debug gate: global stop condition = '{}'", stopCond.c_str());
        master.debugGate().addGlobalStop(std::move(cond));
    }

    return true;
#endif
}

void addEncapsulationArg(argparse::ArgumentParser& /*program*/) {
    // Deprecated: --encapsulation is now added by addInterfaceArg() so
    // every example gets it automatically.  Kept as a no-op for source
    // compatibility with examples that still call it explicitly.
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
            TETHER_LOGW(tag, "  — {}", f.c_str());
        }
        TETHER_LOGI(tag, "Known debug flags:");
        for (const auto& f : knownNames) {
            TETHER_LOGI(tag, "  — {}", f.c_str());
        }
    }
}

// ============================================================================
// Encapsulation helpers (802.1Q VLAN + EtherCAT-over-UDP)
// ============================================================================

namespace {

bool parseVid(const std::string& s, uint16_t& out) {
    try {
        int v = std::stoi(s);
        if (v < 1 || v > 4095) return false;
        out = static_cast<uint16_t>(v);
        return true;
    } catch (...) {
        return false;
    }
}

bool parseVidRange(const std::string& s,
                   EtherCAT::VLANRouter::VLANRange& out) {
    const size_t dash = s.find('-');
    try {
        if (dash == std::string::npos) {
            uint16_t v;
            if (!parseVid(s, v)) return false;
            out = EtherCAT::VLANRouter::VLANRange{v, v};
            return true;
        }
        uint16_t lo, hi;
        if (!parseVid(s.substr(0, dash), lo) ||
            !parseVid(s.substr(dash + 1), hi) || lo > hi) return false;
        out = EtherCAT::VLANRouter::VLANRange{lo, hi};
        return true;
    } catch (...) {
        return false;
    }
}

} // namespace

bool parseEncapsulationArg(const std::string& spec,
                           EncapConfig& out,
                           const char* /*tag*/) {
    out = EncapConfig{};
    if (spec.empty() || spec == "raw" || spec == "none")
        return true;   // plain EtherCAT

    std::optional<uint16_t> txVlanToken;   // explicit vlantx:
    bool txVlanSeen  = false;
    bool txVlanOff   = false;
    bool vlanSeen    = false;
    bool udpSeen     = false;

    std::stringstream ss(spec);
    std::string tok;
    while (std::getline(ss, tok, ',')) {
        tok.erase(0, tok.find_first_not_of(" \t"));
        tok.erase(tok.find_last_not_of(" \t") + 1);
        if (tok.empty()) continue;

        if (tok == "raw" || tok == "none") {
            if (spec.find(',') != std::string::npos) {
                std::cerr << "--encapsulation: 'raw'/'none' cannot be combined "
                             "with other tokens\n";
                return false;
            }
            continue;
        }

        if (tok.rfind("vlan:", 0) == 0) {
            if (vlanSeen) {
                std::cerr << "--encapsulation: duplicate 'vlan:' token\n";
                return false;
            }
            vlanSeen = true;
            const std::string arg = tok.substr(5);
            if (arg == "any") {
                out.rxAny = true;
            } else {
                EtherCAT::VLANRouter::VLANRange r{0, 0};
                if (!parseVidRange(arg, r)) {
                    std::cerr << "--encapsulation: invalid vlan value '" << arg
                              << "' (expected vid 1-4095, lo-hi range, or 'any')\n";
                    return false;
                }
                out.rxRange = r;
                out.txVlan  = r.start;   // TX tag defaults to first RX VID
            }
            continue;
        }

        if (tok.rfind("vlantx:", 0) == 0) {
            if (txVlanSeen) {
                std::cerr << "--encapsulation: duplicate 'vlantx:' token\n";
                return false;
            }
            txVlanSeen = true;
            const std::string arg = tok.substr(7);
            if (arg == "off" || arg == "none") {
                txVlanOff = true;
            } else {
                uint16_t v;
                if (!parseVid(arg, v)) {
                    std::cerr << "--encapsulation: invalid vlantx value '"
                              << arg << "' (expected vid 1-4095 or 'off')\n";
                    return false;
                }
                txVlanToken = v;
            }
            continue;
        }

        if (tok == "udp" || tok.rfind("udp:", 0) == 0) {
            if (udpSeen) {
                std::cerr << "--encapsulation: duplicate 'udp' token\n";
                return false;
            }
            udpSeen = true;
            out.udp = true;
            if (tok.size() > 4) {
                try {
                    int p = std::stoi(tok.substr(4));
                    if (p < 1 || p > 65535) throw std::out_of_range("port");
                    out.udpPort = static_cast<uint16_t>(p);
                } catch (...) {
                    std::cerr << "--encapsulation: invalid udp port '"
                              << tok.substr(4) << "' (expected 1-65535)\n";
                    return false;
                }
            }
            continue;
        }

        std::cerr << "--encapsulation: unknown token '" << tok
                  << "' (expected vlan:<vid|lo-hi|any>, vlantx:<vid|off>, "
                     "udp[:<port>], raw)\n";
        return false;
    }

    // An explicit vlantx: overrides the VID implied by vlan:<vid>.
    if (txVlanOff) out.txVlan = std::nullopt;
    else if (txVlanToken) out.txVlan = *txVlanToken;

    if (txVlanSeen && !vlanSeen && !txVlanOff && !txVlanToken) {
        // unreachable, but keep the invariant obvious
        return false;
    }
    return true;
}

void logEncapConfig(const EncapConfig& config, const char* tag) {
    if (!config.enabled()) return;

    if (config.rxAny) {
        TETHER_LOGI(tag, "Encapsulation: VLAN RX=any (undefined target), TX={}",
                    config.txVlan ? std::to_string(*config.txVlan).c_str()
                                  : "untagged");
    } else if (config.rxRange) {
        TETHER_LOGI(tag, "Encapsulation: VLAN RX={}-{}, TX={}",
                    config.rxRange->start, config.rxRange->end,
                    config.txVlan ? std::to_string(*config.txVlan).c_str()
                                  : "untagged");
    } else if (config.txVlan) {
        TETHER_LOGI(tag, "Encapsulation: VLAN RX=untagged, TX={}",
                    *config.txVlan);
    }
    if (config.udp) {
        TETHER_LOGI(tag, "Encapsulation: EtherCAT-over-UDP dst port {}",
                    config.udpPort);
    }
}

std::vector<EtherCAT::CBPFInsn> buildEncapBpfProgram(const EncapConfig& config) {
    using namespace EtherCAT;
    CBPFSpec s;
    s.udp_port = config.udpPort;

    if (config.rxRange) {
        // VLAN mode: reject everything except VID∈range + inner EtherCAT
        // (and inner UDP-encapsulated EtherCAT when udp is on).
        s.untagged_ethercat = false;
        s.untagged_udp      = false;
        s.tagged_ethercat   = true;
        s.tagged_udp        = config.udp;
        s.vlan_range        = CBPFVlanRange{config.rxRange->start,
                                            config.rxRange->end};
    } else if (config.rxAny) {
        // Catch-all tagged traffic: no untagged, any-VID tagged EtherCAT.
        s.untagged_ethercat = false;
        s.untagged_udp      = false;
        s.tagged_ethercat   = true;
        s.tagged_udp        = config.udp;
    } else {
        // Default / TX-tag-only: untagged + any-VID tagged EtherCAT (+UDP).
        s.untagged_ethercat = true;
        s.untagged_udp      = config.udp;
        s.tagged_ethercat   = true;
        s.tagged_udp        = config.udp;
    }
    return CBPFProgramFactory::build(s);
}

void attachEncapBpfFilter(EtherCAT::HAL::IEthernet& eth,
                          const EncapConfig& config,
                          const char* tag) {
    const int fd = static_cast<int>(
        reinterpret_cast<intptr_t>(eth.nativeHandle()));
    if (fd < 0) return;   // backend without a socket fd — nothing to attach

    const auto prog = buildEncapBpfProgram(config);
    if (prog.empty()) {
        TETHER_LOGW(tag, "Encapsulation produced an empty BPF program — "
                         "no filter attached");
        return;
    }
    if (!EtherCAT::CBPFProgramFactory::attach(fd, prog)) {
        // Soft failure: userspace filtering keeps correctness; only the
        // kernel-side wakeup savings are lost.
        TETHER_LOGW(tag, "BPF socket filter attach failed — continuing "
                         "without kernel filtering");
    } else {
        TETHER_LOGI(tag, "Kernel cBPF filter attached ({} insns)", prog.size());
    }
}

// ============================================================================
// Mailbox helpers
// ============================================================================

void addMailboxSizeArg(argparse::ArgumentParser& program, uint16_t default_size) {
    program.add_argument("-M", "--mailbox-size")
        .default_value(std::to_string(default_size))
        .help(std::format("Mailbox buffer size in bytes. Short form: a single number sets both MbxIn and MbxOut (e.g. -M {0}). Long form: in:<size>,out:<size> to set independently (e.g. --mailbox-size in:256,out:512). Range: 1-65535. Default: {0}.", default_size));
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
    TETHER_LOGI(tag, "Mailbox config: MbxOut addr=0x{:04X} len={}, MbxIn addr=0x{:04X} len={}",
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

    TETHER_LOGE(tag, "Slave rejected the SDO request: CoE abort code 0x{:08X} ({}). "
                     "Operation: {}, attempted {}: {} bytes.",
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

    // The length-mismatch family  -  the signature of the original
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

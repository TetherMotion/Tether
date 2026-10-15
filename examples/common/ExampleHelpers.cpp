#include "ExampleHelpers.hpp"

#include <algorithm>
#include <format>
#include <iostream>
#include <limits>
#include <sstream>

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

void addListInterfacesArg(argparse::ArgumentParser& program) {
    program.add_argument("--list-interfaces")
        .default_value(false)
        .implicit_value(true)
        .help("List available physical Ethernet interfaces and exit");
}

void addDebugArg(argparse::ArgumentParser& program) {
    program.add_argument("--debug")
        .default_value(std::string(""))
        .help("Comma-separated debug flags. Use '--debug help' for a list.");
}

void addDebugConditionArgs(argparse::ArgumentParser& program) {
    program.add_argument("--debug-start")
        .default_value(std::string(""))
        .help("Start debug output when condition fires. Use '--debug-start help' for syntax.");
    program.add_argument("--debug-stop")
        .default_value(std::string(""))
        .help("Stop debug output when condition fires. Use '--debug-start help' for syntax.");
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

void addStreamArgs(argparse::ArgumentParser& program,
                   std::string_view streamHelp) {
    program.add_argument("--interactive")
        .help("Force the interactive ncurses TUI")
        .flag();
    program.add_argument("--stream")
        .help(std::string(streamHelp))
        .flag();
}

bool resolveInteractiveMode(const argparse::ArgumentParser& program,
                            const char* tag, bool tui_available,
                            const char* unavailable_reason) {
    if (program.get<bool>("--stream")) return false;
    if (tui_available) return true;
    if (program.get<bool>("--interactive")) {
        TETHER_LOGW(tag, "{}", unavailable_reason);
    }
    return false;
}

std::optional<std::vector<uint16_t>> resolveSlaveIndices(
    const argparse::ArgumentParser& program,
    std::initializer_list<std::string_view> arg_names) {
    std::vector<uint16_t> indices;
    indices.reserve(arg_names.size());
    for (std::string_view name : arg_names) {
        int raw = program.get<int>(std::string(name));
        if (raw < 0) {
            if (indices.empty()) {
                std::cerr << std::format("{} must be >= 0\n", name);
                return std::nullopt;
            }
            raw = indices.back() + 1;
        }
        if (raw > std::numeric_limits<uint16_t>::max() - 1) {
            std::cerr << std::format("Invalid slave index {} from {}\n",
                                     raw, name);
            return std::nullopt;
        }
        const auto idx = static_cast<uint16_t>(raw);
        if (std::find(indices.begin(), indices.end(), idx) != indices.end()) {
            std::cerr << std::format("{} resolves to slave {} — indices "
                                     "must be distinct\n", name, idx);
            return std::nullopt;
        }
        indices.push_back(idx);
    }
    return indices;
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

} // namespace Tether::Examples

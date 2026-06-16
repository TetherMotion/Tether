/**
 * @file interpret_pcapng.cpp
 * @brief Read and interpret a pcapng capture of EtherCAT traffic
 *
 * Usage:
 *   ./interpret_pcapng capture.pcapng
 *   ./interpret_pcapng capture.pcapng --max-packets 100 --verbose
 *   ./interpret_pcapng capture.pcapng --vlan 100 --json
 *   ./interpret_pcapng capture.pcapng --command LRW --max-data 256
 */

#include <algorithm>
#include <cctype>
#include <cstdint>
#include <cstdlib>
#include <iostream>
#include <optional>
#include <string>

#include <argparse/argparse.hpp>

#include "tether/ethercat/Types.hpp"
#include "tether/packetloggers/pcap/PCAPNGReader.hpp"

namespace {

std::string upperCase(std::string s) {
    std::transform(s.begin(), s.end(), s.begin(),
                   [](unsigned char c) { return static_cast<char>(std::toupper(c)); });
    return s;
}

std::optional<EtherCAT::Command> parseCommand(const std::string& name) {
    const std::string u = upperCase(name);
    // Iterate all known command values via magic_enum would be nice, but we
    // keep it dependency-free and explicit.
    struct Entry { const char* name; EtherCAT::Command cmd; };
    static const Entry entries[] = {
        {"NOP", EtherCAT::Command::NOP},
        {"APRD", EtherCAT::Command::APRD},
        {"APWR", EtherCAT::Command::APWR},
        {"APRW", EtherCAT::Command::APRW},
        {"FPRD", EtherCAT::Command::FPRD},
        {"FPWR", EtherCAT::Command::FPWR},
        {"FPRW", EtherCAT::Command::FPRW},
        {"BRD", EtherCAT::Command::BRD},
        {"BWR", EtherCAT::Command::BWR},
        {"BRW", EtherCAT::Command::BRW},
        {"LRD", EtherCAT::Command::LRD},
        {"LWR", EtherCAT::Command::LWR},
        {"LRW", EtherCAT::Command::LRW},
        {"ARMW", EtherCAT::Command::ARMW},
        {"FRMW", EtherCAT::Command::FRMW},
    };
    for (const auto& e : entries) {
        if (u == e.name) return e.cmd;
    }
    return std::nullopt;
}

} // anonymous namespace

int main(int argc, char** argv) {
    argparse::ArgumentParser program("interpret_pcapng");
    program.add_argument("input")
        .help("Path to the pcapng file to interpret");
    program.add_argument("-v", "--verbose")
        .default_value(false)
        .implicit_value(true)
        .help("Print full payload hex dumps");
    program.add_argument("--json")
        .default_value(false)
        .implicit_value(true)
        .help("Output compact JSON instead of human-readable text");
    program.add_argument("--max-packets")
        .default_value(uint64_t{0})
        .scan<'u', uint64_t>()
        .help("Stop after this many packets (0 = unlimited)");
    program.add_argument("--max-data")
        .default_value(size_t{64})
        .scan<'u', size_t>()
        .help("Maximum payload bytes to dump per datagram (0 = no limit)");
    program.add_argument("--vlan")
        .default_value(uint64_t{0})
        .scan<'u', uint64_t>()
        .help("Only show frames with the given VLAN ID (0 = all)");
    program.add_argument("--command")
        .default_value(std::string{})
        .help("Only show datagrams with the given command (e.g. LRW, APRD)");
    program.add_argument("--only-ethercat")
        .default_value(false)
        .implicit_value(true)
        .help("Skip non-EtherCAT frames");

    try {
        program.parse_args(argc, argv);
    } catch (const std::runtime_error& err) {
        std::cerr << err.what() << "\n" << program;
        return 1;
    }

    const std::string input = program.get<std::string>("input");
    const bool verbose = program.get<bool>("--verbose");
    const bool json = program.get<bool>("--json");
    const uint64_t maxPackets = program.get<uint64_t>("--max-packets");
    const size_t maxData = program.get<size_t>("--max-data");
    const uint64_t vlanFilter = program.get<uint64_t>("--vlan");
    const std::string cmdName = program.get<std::string>("--command");
    const bool onlyEtherCAT = program.get<bool>("--only-ethercat");

    std::optional<EtherCAT::Command> cmdFilter;
    if (!cmdName.empty()) {
        cmdFilter = parseCommand(cmdName);
        if (!cmdFilter) {
            std::cerr << "Unknown EtherCAT command: " << cmdName << "\n";
            return 1;
        }
    }

    Tether::PacketLoggers::PCAP::PCAPNGReader reader;
    if (!reader.open(input)) {
        std::cerr << "Failed to open pcapng file: " << input << "\n";
        return 2;
    }

    const auto& section = reader.sectionInfo();
    if (!json) {
        std::cout << "=== PCAPNG Summary ===\n";
        std::cout << "Byte order swapped: " << (section.byteOrderSwapped ? "yes" : "no") << "\n";
        std::cout << "Section length: "
                  << (section.sectionLength >= 0 ? std::to_string(section.sectionLength) : "unknown")
                  << "\n";
        if (!section.hardware.empty()) std::cout << "Hardware: " << section.hardware << "\n";
        if (!section.os.empty()) std::cout << "OS: " << section.os << "\n";
        if (!section.application.empty()) std::cout << "Application: " << section.application << "\n";
        if (!section.comment.empty()) std::cout << "Comment: " << section.comment << "\n";

        const auto& ifaces = reader.interfaces();
        std::cout << "Interfaces: " << ifaces.size() << "\n";
        for (const auto& iface : ifaces) {
            std::cout << "  Interface " << iface.id
                      << ": linkType=" << iface.linkType
                      << " snapLen=" << iface.snapLen;
            if (!iface.name.empty()) std::cout << " name=\"" << iface.name << "\"";
            if (!iface.description.empty()) std::cout << " desc=\"" << iface.description << "\"";
            std::cout << "\n";
        }
        std::cout << "\n";
    }

    uint64_t packetIndex = 0;
    uint64_t shown = 0;
    bool limitReached = false;

    const bool ok = reader.readAll([&](const Tether::PacketLoggers::PCAP::InterpretedFrame& frame) {
        if (limitReached) return;
        ++packetIndex;

        if (maxPackets > 0 && shown >= maxPackets) {
            limitReached = true;
            return;
        }

        if (onlyEtherCAT && !frame.isEtherCAT) {
            return;
        }

        if (vlanFilter > 0 && (!frame.vlanId.has_value() || *frame.vlanId != vlanFilter)) {
            return;
        }

        if (cmdFilter) {
            bool hasCmd = false;
            for (const auto& dg : frame.datagrams) {
                if (dg.cmd == *cmdFilter) {
                    hasCmd = true;
                    break;
                }
            }
            if (!hasCmd) return;
        }

        if (json) {
            std::cout << Tether::PacketLoggers::PCAP::frameToJson(frame);
        } else {
            std::cout << "--- Packet " << packetIndex << " ---\n";
            std::cout << Tether::PacketLoggers::PCAP::formatInterpretedFrame(
                frame, verbose, maxData);
        }
        ++shown;
    });

    if (!ok) {
        std::cerr << "Warning: parse error encountered after " << packetIndex << " packets\n";
        return 3;
    }

    if (!json) {
        std::cout << "\nTotal packets shown: " << shown << " / " << packetIndex << " parsed\n";
    }

    return 0;
}

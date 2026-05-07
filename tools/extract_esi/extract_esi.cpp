#include <iostream>
#include <string>
#include <vector>

#if __has_include(<argparse/argparse.hpp>)
#include <argparse/argparse.hpp>
#else
#include <argparse/argparse.hpp>
#endif

#include "tether/ethercat/ESIParser.hpp"

int main(int argc, char** argv) {
    argparse::ArgumentParser program("extract_esi_xml");

    program.add_argument("xmlfile").help("ESI XML file path").required();
    program.add_argument("--mailbox").help("Only print mailbox settings").nargs(0);
    program.add_argument("--sync-managers").help("Only print sync manager settings").nargs(0);
    program.add_argument("--json").help("Emit structured JSON instead of human-readable text").nargs(0);
    program.add_argument("--device-index").help("Device index (0-based) to select").default_value(std::string("0"));

    try {
        program.parse_args(argc, argv);
    } catch (const std::runtime_error& err) {
        std::cerr << err.what() << "\n";
        std::cerr << program;
        return 2;
    }

    std::string xml = program.get<std::string>("xmlfile");
    bool onlyMailbox = program.is_used("--mailbox");
    bool onlySM = program.is_used("--sync-managers");
    int devIdx = 0;
    try { devIdx = std::stoi(program.get<std::string>("--device-index")); } catch(...) { devIdx = 0; }

    std::vector<EtherCAT::ESI::DeviceInfo> devices;
    std::string err;
    if (!EtherCAT::ESI::parseESIFile(xml, devices, err)) {
        std::cerr << "Error parsing ESI: " << err << "\n";
        return 1;
    }

    if (devices.empty()) {
        std::cout << "No devices found in ESI XML" << std::endl;
        return 0;
    }

    if (devIdx < 0 || static_cast<size_t>(devIdx) >= devices.size()) {
        std::cerr << "Invalid device index. Valid range: 0.." << (devices.size()-1) << "\n";
        return 3;
    }

    const auto& dev = devices[devIdx];

    bool emitJson = program.is_used("--json");

    if (emitJson) {
        std::cout << EtherCAT::ESI::formatDeviceJSON(dev) << std::endl;
    } else if (onlyMailbox) {
        std::cout << EtherCAT::ESI::formatDeviceHumanReadable(dev, true);
    } else if (onlySM) {
        // print only sync managers in a concise form
        std::cout << "Sync Managers:\n";
        for (size_t i=0;i<dev.syncManagers.size();++i) {
            const auto& s = dev.syncManagers[i];
            std::cout << "  SM" << i << ": start=0x" << std::hex << std::uppercase << std::setw(4) << std::setfill('0') << s.startAddress << std::dec
                      << " len=" << s.defaultSize << " ctrl=0x" << std::hex << (unsigned)s.control << std::dec << " (humanized)" << "\n";
        }
    } else {
        std::cout << EtherCAT::ESI::formatDeviceHumanReadable(dev, false);
    }

    return 0;
}

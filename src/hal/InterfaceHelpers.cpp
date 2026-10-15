#include "tether/hal/InterfaceHelpers.hpp"

#include <cstdio>
#include <cstdlib>
#include <format>

#include "logging/Logger.hpp"

#include "tether/hal/NetworkInterfaceEnumerator.hpp"

namespace EtherCAT {
namespace HAL {

std::string resolveInterface(const std::string& requested, const char* tag) {
    if (!requested.empty()) {
        return requested;
    }

    auto physIfaces = getPhysicalEthernetInterfaces();
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

void listPhysicalInterfaces(const char* tag) {
    auto ifaces = getPhysicalEthernetInterfaces();
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
                    interfaceTypeToString(iface.type),
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

} // namespace HAL
} // namespace EtherCAT

/**
 * @file SystemStatsProvider.cpp
 * @brief Linux implementation of ISystemStatsProvider.
 */

#include "tether/klipper/klippy/SystemStatsProvider.hpp"

#include <fstream>
#include <string>

namespace tether::klipper::klippy {

SystemStatsSnapshot LinuxSystemStatsProvider::readStats() {
    SystemStatsSnapshot snap;

    // Read /proc/loadavg for sysload
    std::ifstream loadavg("/proc/loadavg");
    if (loadavg) {
        double load;
        if (loadavg >> load) {
            snap.sysload = load;
        }
    }

    // Read /proc/meminfo for memavail
    std::ifstream meminfo("/proc/meminfo");
    if (meminfo) {
        std::string line;
        while (std::getline(meminfo, line)) {
            if (line.rfind("MemAvailable:", 0) == 0) {
                // Parse "MemAvailable:   1234567 kB"
                size_t colon = line.find(':');
                if (colon != std::string::npos) {
                    std::string val = line.substr(colon + 1);
                    // Strip whitespace and "kB"
                    size_t start = val.find_first_not_of(" \t");
                    if (start != std::string::npos) {
                        snap.memAvailable = std::stod(val.substr(start)) / 1024.0;
                    }
                }
                break;
            }
        }
    }

    return snap;
}

} // namespace tether::klipper::klippy

#include "tether/ethercat/DebugHelpers.hpp"

#include <iostream>
#include <sstream>

#include "tether/ethercat/DebugFlags.hpp"
#include "tether/ethercat/DebugGate.hpp"
#include "tether/ethercat/Master.hpp"

namespace EtherCAT {
namespace debug {

std::set<std::string> parseFlags(const std::string& spec) {
    std::set<std::string> flags;
    if (spec.empty()) return flags;

    std::stringstream ss(spec);
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

void applyFlags(const std::set<std::string>& flags,
                Master& master,
                const char* tag) {
    const auto& registry = allDebugFlags();

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

bool applyGateConditions(const std::string& startCond,
                         const std::string& stopCond,
                         Master& master,
                         const char* tag) {
    if (startCond.empty() && stopCond.empty()) return true;

#if !TETHER_DEBUG_GATE_ENABLED
    if (!startCond.empty() || !stopCond.empty()) {
        TETHER_LOGW(tag, "Debug gate compiled out (TETHER_DEBUG_GATE_ENABLED=0); "
                         "ignoring debug start/stop conditions.");
    }
    return true;
#else
    if (!startCond.empty()) {
        auto cond = DebugGate::parseCondition(startCond);
        if (!cond) {
            TETHER_LOGE(tag, "Failed to parse debug start condition: '{}'", startCond.c_str());
            return false;
        }
        TETHER_LOGI(tag, "Debug gate: global start condition = '{}'", startCond.c_str());
        master.debugGate().addGlobalStart(std::move(cond));
    }

    if (!stopCond.empty()) {
        auto cond = DebugGate::parseCondition(stopCond);
        if (!cond) {
            TETHER_LOGE(tag, "Failed to parse debug stop condition: '{}'", stopCond.c_str());
            return false;
        }
        TETHER_LOGI(tag, "Debug gate: global stop condition = '{}'", stopCond.c_str());
        master.debugGate().addGlobalStop(std::move(cond));
    }

    return true;
#endif
}

bool printFlagsHelp(const std::string& spec) {
    if (spec != "help") return false;

    const auto& registry = allDebugFlags();
    std::cout << "Available debug flags:\n";
    for (const auto& info : registry) {
        std::cout << "  " << info.name << "\n      " << info.description << "\n";
    }
    std::cout << "\nFilter syntax:\n";
    std::cout << "  --debug flagname:(slaves:0,2,5),otherflag:(slaves:1-3)\n";
    std::cout << "  (default: pass-all for every flag)\n";
    return true;
}

bool printConditionHelp(const std::string& spec) {
    if (spec != "help") return false;
#if TETHER_DEBUG_GATE_ENABLED
    DebugGate::printHelp();
#else
    std::cout << "Debug gate compiled out (TETHER_DEBUG_GATE_ENABLED=0).\n"
              << "Conditional debugging is not available in this build.\n";
#endif
    return true;
}

} // namespace debug
} // namespace EtherCAT

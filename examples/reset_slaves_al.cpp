/**
 * @file reset_slaves_al.cpp
 * @brief Reset EtherCAT slave(s) to a target ESM state via AL Control
 *
 * Scans the bus, then for each selected slave performs a bounded AL reset
 * loop: write target state with error-ack clear, then write target state
 * with error-ack set, until the slave reports the target state with no error
 * bit, or the iteration limit is reached.
 *
 * Usage (Linux, requires root or CAP_NET_RAW):
 *   ./reset_slaves_al                  # reset all slaves to INIT
 *   ./reset_slaves_al -s 0             # reset only slave 0
 *   ./reset_slaves_al -s 1 -s 2        # reset slaves 1 and 2
 *   ./reset_slaves_al -s 1,2,3         # reset slaves 1, 2 and 3
 *   ./reset_slaves_al --target-state 2 # reset to PRE_OP
 *   ./reset_slaves_al -n 20 --sleep 10 # 20 iterations, 10 ms sleep
 */

#include <cstring>
#include <iostream>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include "tether/ethercat/Master.hpp"
#include "tether/ethercat/Slave.hpp"
#include "tether/ethercat/Types.hpp"
#include "tether/ethercat/FaultDetection.hpp"
#include "tether/ethercat/ALResetController.hpp"
#include "tether/ethercat/SyncManager.hpp"
#include "tether/platform/EspCompat.hpp"

#include "common/ExampleHelpers.hpp"
#include "common/EtherCATHostSetup.hpp"

static const char* TAG = "reset_slaves_al";

// ============================================================================
// Parse repeatable -s flags that accept single index or comma list
// ============================================================================

static std::vector<uint16_t> parseSlaveIndices(const std::vector<std::string>& args) {
    std::vector<uint16_t> indices;
    for (const auto& arg : args) {
        std::stringstream ss(arg);
        std::string token;
        while (std::getline(ss, token, ',')) {
            token.erase(0, token.find_first_not_of(" \t"));
            token.erase(token.find_last_not_of(" \t") + 1);
            if (token.empty()) continue;
            try {
                int v = std::stoi(token);
                if (v < 0 || v > 65535) {
                    std::cerr << "Invalid slave index: " << token << " (must be 0-65535)\n";
                    std::exit(1);
                }
                indices.push_back(static_cast<uint16_t>(v));
            } catch (...) {
                std::cerr << "Invalid slave index: " << token << "\n";
                std::exit(1);
            }
        }
    }
    return indices;
}

// ============================================================================
// Host-side helpers
// ============================================================================

int main(int argc, char** argv) {
    argparse::ArgumentParser program("reset_slaves_al");
    Tether::Examples::addInterfaceArg(program);
    Tether::Examples::addDebugArg(program);
    Tether::Examples::addVlanArgs(program);
    Tether::Examples::addMailboxSizeArg(program);
    Tether::Examples::addMailboxAddressArg(program);
    program.add_argument("-s", "--slave")
        .append()
        .help("Slave index to reset. Repeatable. Accepts comma list (e.g. -s 1,2,3). Default: all discovered slaves.");
    program.add_argument("-n", "--max-iterations")
        .default_value(std::string("50"))
        .help("Maximum reset iterations per slave (default 50)");
    program.add_argument("--sleep")
        .default_value(std::string("50"))
        .help("Sleep in ms between reset iterations (default 50)");
    program.add_argument("--target-state")
        .default_value(std::string("1"))
        .help("Target ESM state as decimal or hex (default 1 = INIT). Prefix with 0x for hex.");

    try { program.parse_args(argc, argv); }
    catch (const std::runtime_error& err) {
        std::cerr << err.what() << "\n" << program;
        return 1;
    }

    std::string iface       = program.get<std::string>("--interface");
    std::string debug_str   = program.get<std::string>("--debug");

    int max_iterations = 50;
    try {
        max_iterations = std::stoi(program.get<std::string>("--max-iterations"));
        if (max_iterations < 1 || max_iterations > 10000) {
            std::cerr << "--max-iterations must be 1-10000\n";
            return 1;
        }
    } catch (...) {
        std::cerr << "Invalid --max-iterations value\n";
        return 1;
    }

    int sleep_ms = 50;
    try {
        sleep_ms = std::stoi(program.get<std::string>("--sleep"));
        if (sleep_ms < 0 || sleep_ms > 60000) {
            std::cerr << "--sleep must be 0-60000\n";
            return 1;
        }
    } catch (...) {
        std::cerr << "Invalid --sleep value\n";
        return 1;
    }

    uint8_t target_state = 0x01;
    try {
        std::string ts_str = program.get<std::string>("--target-state");
        int ts_val = 0;
        if (ts_str.size() > 2 && (ts_str.substr(0, 2) == "0x" || ts_str.substr(0, 2) == "0X")) {
            ts_val = std::stoi(ts_str.substr(2), nullptr, 16);
        } else {
            ts_val = std::stoi(ts_str);
        }
        if (ts_val < 0 || ts_val > 15) {
            std::cerr << "--target-state must be 0-15\n";
            return 1;
        }
        target_state = static_cast<uint8_t>(ts_val);
    } catch (...) {
        std::cerr << "Invalid --target-state value\n";
        return 1;
    }

    std::vector<uint16_t> selected_slaves;
    if (program.is_used("--slave")) {
        selected_slaves = parseSlaveIndices(program.get<std::vector<std::string>>("--slave"));
    }

    if (Tether::Examples::printDebugHelpIfRequested(debug_str)) return 0;
    auto debug_flags = Tether::Examples::parseDebugFlags(debug_str);

    Tether::Examples::VlanConfig vlan;
    if (!Tether::Examples::parseVlanArgs(
            program.get<std::string>("--rx-vlan"),
            program.get<std::string>("--tx-vlan"),
            vlan, TAG)) {
        return 1;
    }

    TETHER_LOGI(TAG, "reset_slaves_al (host) — interface: %s", iface.c_str());
    TETHER_LOGI(TAG, "Parameters: target-state=0x%02X, max-iterations=%d, sleep=%d ms",
                target_state, max_iterations, sleep_ms);
    if (!selected_slaves.empty()) {
        std::string list;
        for (size_t i = 0; i < selected_slaves.size(); ++i) {
            if (i) list += ",";
            list += std::to_string(selected_slaves[i]);
        }
        TETHER_LOGI(TAG, "Selected slaves: %s", list.c_str());
    } else {
        TETHER_LOGI(TAG, "Selected slaves: all");
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

    if (!master.discoverSlaves()) {
        TETHER_LOGW(TAG, "No slaves discovered");
        master.stop();
        Tether::Examples::shutdownHostEthernet(session);
        return 4;
    }

    uint16_t slaves = master.getDiscoveredSlaveCount();
    TETHER_LOGI(TAG, "=== Discovered %u slave(s) ===", slaves);
    master.logDiscoveredSlavesSummary(TAG);

    if (slaves == 0) {
        TETHER_LOGW(TAG, "No slaves found — check wiring, power, and interface name");
        master.stop();
        Tether::Examples::shutdownHostEthernet(session);
        return 4;
    }

    std::vector<uint16_t> targets;
    if (selected_slaves.empty()) {
        for (uint16_t i = 0; i < slaves; ++i) targets.push_back(i);
    } else {
        for (uint16_t si : selected_slaves) {
            if (si >= slaves) {
                TETHER_LOGW(TAG, "Slave %u not discovered (only %u slave(s) on bus), skipping", si, slaves);
                continue;
            }
            targets.push_back(si);
        }
    }

    if (targets.empty()) {
        TETHER_LOGW(TAG, "No valid slaves selected for reset");
        master.stop();
        Tether::Examples::shutdownHostEthernet(session);
        return 4;
    }

    EtherCAT::ALResetController ctrl(master);
    ctrl.setProgressCallback(
        [](uint16_t si, int iter, int max_iter, uint16_t al, uint16_t code, bool reached) {
            if (!reached) {
                const char* state_name = EtherCAT::al_status_get_state_name(al);
                const bool has_err = EtherCAT::al_status_has_error(al);
                TETHER_LOGI(TAG, "  Slave %u iter %d/%d: AL_STATUS=0x%04X (state=%s, error=%s)",
                            si, iter, max_iter, al,
                            state_name, has_err ? "true" : "false");
                if (code != 0) {
                    TETHER_LOGI(TAG, "    AL_STATUS_CODE=0x%04X (%s)",
                                code, EtherCAT::getALStatusCodeName(code));
                }
            }
        });

    uint16_t success_count = 0;
    uint16_t fail_count    = 0;

    for (uint16_t si : targets) {
        TETHER_LOGI(TAG, "Resetting slave %u to state 0x%02X (max %d iterations, %d ms sleep) ...",
                    si, target_state, max_iterations, sleep_ms);

        auto result = ctrl.resetSlave(si, target_state, max_iterations, sleep_ms);

        if (result.success) {
            TETHER_LOGI(TAG, "  Slave %u => target state 0x%02X OK (%s)",
                        si, target_state, result.message.c_str());
            ++success_count;
        } else {
            TETHER_LOGE(TAG, "  Slave %u => target state 0x%02X FAILED (%s)",
                        si, target_state, result.message.c_str());
            ++fail_count;
        }
    }

    TETHER_LOGI(TAG, "=== Reset Summary: %u succeeded, %u failed ===", success_count, fail_count);

    master.stop();
    Tether::Examples::shutdownHostEthernet(session);

    if (fail_count == 0) {
        return 0;
    } else if (success_count == 0) {
        return 7;
    } else {
        return 5;
    }
}

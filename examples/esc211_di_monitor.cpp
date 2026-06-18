/**
 * @file esc211_di_monitor.cpp
 * @brief Nexcobot ESC211 Safety Digital-Input Monitor
 *
 * Discovers an ESC211 on the EtherCAT bus, transitions to PRE-OP,
 * and continuously reads Safety DI A/B via CoE/SDO.
 * Renders a live ncurses panel of green/empty circles at 50 Hz.
 *
 * Usage (Linux, requires root or CAP_NET_RAW):
 *   ./esc211_di_monitor              # uses eth0, slave 0
 *   ./esc211_di_monitor -i enp3s0  # specify interface
 *   ./esc211_di_monitor -s 1       # specify slave index
 */

#include <array>
#include <atomic>
#include <chrono>
#include <csignal>
#include <cstring>
#include <iostream>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

#include <clocale>

#include "tether/drives/NexcobotESC211/NexcobotESC211Registers.hpp"

// ncurses defines OK/ERR as macros; undefine them before Tether headers
// that use Error::OK are parsed.
#include <ncurses.h>
#undef OK
#undef ERR
#include "tether/ethercat/Master.hpp"
#include "tether/ethercat/Slave.hpp"
#include "tether/ethercat/Types.hpp"
#include "tether/ethercat/SyncManager.hpp"
#include "tether/sii/SIIReader.hpp"
#include "tether/sii/SIIParser.hpp"

#include "common/ExampleHelpers.hpp"
#include "common/EtherCATHostSetup.hpp"

static const char* TAG = "esc211_di_monitor";
static std::atomic<bool> g_cancel{false};
static EtherCAT::Master* g_master = nullptr;

void signalHandler(int) {
    g_cancel.store(true);
    if (g_master) {
        g_master->requestCancel();
    }
}

// ---------------------------------------------------------------------------
// Shared DI state between SDO reader and ncurses display
// ---------------------------------------------------------------------------

struct DIState {
    mutable std::mutex mtx;
    std::array<bool, 10> ia{};   // IA1..IA10
    std::array<bool, 10> ib{};   // IB1..IB10
    bool stale = true;
    uint64_t read_count = 0;
};

// ---------------------------------------------------------------------------
// SDO reader thread (50 Hz)
// ---------------------------------------------------------------------------

static void sdoReaderThread(EtherCAT::Slave& slave, DIState& state) {
    using namespace std::chrono;
    const auto period = milliseconds(20);
    auto next_time = steady_clock::now();

    while (!g_cancel.load(std::memory_order_relaxed)) {
        std::array<bool, 10> ia{};
        std::array<bool, 10> ib{};
        bool ok = true;

        for (int i = 0; i < 10 && ok; ++i) {
            uint8_t val = 0;
            auto err = slave.sdoReadU8(
                EtherCAT::Drives::NexcobotESC211::kSafetyInputAIndex,
                static_cast<uint8_t>(i + 1), val);
            if (err != EtherCAT::SlaveError::Ok) {
                ok = false;
            } else {
                ia[i] = (val != 0);
            }
        }
        for (int i = 0; i < 10 && ok; ++i) {
            uint8_t val = 0;
            auto err = slave.sdoReadU8(
                EtherCAT::Drives::NexcobotESC211::kSafetyInputBIndex,
                static_cast<uint8_t>(i + 1), val);
            if (err != EtherCAT::SlaveError::Ok) {
                ok = false;
            } else {
                ib[i] = (val != 0);
            }
        }

        {
            std::lock_guard<std::mutex> lock(state.mtx);
            if (ok) {
                state.ia = ia;
                state.ib = ib;
                state.stale = false;
                ++state.read_count;
            } else {
                state.stale = true;
            }
        }

        next_time += period;
        std::this_thread::sleep_until(next_time);
    }
}

// ---------------------------------------------------------------------------
// Ncurses helpers
// ---------------------------------------------------------------------------

static void initColors() {
    if (has_colors()) {
        start_color();
        use_default_colors();
        init_pair(1, COLOR_GREEN, -1);   // active
        init_pair(2, COLOR_WHITE, -1);   // inactive label
        init_pair(3, COLOR_RED, -1);     // stale / error
    }
}

static void drawScreen(const DIState& state) {
    std::lock_guard<std::mutex> lock(state.mtx);

    clear();
    mvprintw(0, 0, "Nexcobot ESC211 — Safety Digital Inputs (50 Hz)");
    mvprintw(1, 0, "Press Ctrl-C to quit");

    if (state.stale) {
        attron(COLOR_PAIR(3));
        mvprintw(3, 0, "[STALE / SDO ERROR]");
        attroff(COLOR_PAIR(3));
    }

    // Header
    attron(A_BOLD);
    mvprintw(3, 2,  "IA (left)");
    mvprintw(3, 22, "IB (right)");
    attroff(A_BOLD);

    for (int row = 0; row < 10; ++row) {
        int y = 4 + row;

        // Left: IA1..IA10
        bool a = state.ia[row];
        mvprintw(y, 2, "IA%2d  ", row + 1);
        if (a) {
            attron(COLOR_PAIR(1));
            mvprintw(y, 10, "\u25CF");   // filled circle
            attroff(COLOR_PAIR(1));
        } else {
            mvprintw(y, 10, "\u25CB");   // empty circle
        }

        // Right: IB1..IB10
        bool b = state.ib[row];
        mvprintw(y, 22, "IB%2d  ", row + 1);
        if (b) {
            attron(COLOR_PAIR(1));
            mvprintw(y, 30, "\u25CF");
            attroff(COLOR_PAIR(1));
        } else {
            mvprintw(y, 30, "\u25CB");
        }
    }

    mvprintw(16, 0, "Read count: %llu", static_cast<unsigned long long>(state.read_count));
    refresh();
}

// ---------------------------------------------------------------------------
// Identity helper
// ---------------------------------------------------------------------------

static void readIdentityObject(EtherCAT::Slave& sl) {
    uint32_t vendor_id = 0;
    uint32_t product_code = 0;
    uint32_t revision = 0;
    uint32_t serial = 0;

    bool identity_ok = true;
    if (sl.sdoReadU32(0x1018, 1, vendor_id) != EtherCAT::SlaveError::Ok) { identity_ok = false; }
    if (sl.sdoReadU32(0x1018, 2, product_code) != EtherCAT::SlaveError::Ok) { identity_ok = false; }
    if (sl.sdoReadU32(0x1018, 3, revision) != EtherCAT::SlaveError::Ok) { identity_ok = false; }
    if (sl.sdoReadU32(0x1018, 4, serial) != EtherCAT::SlaveError::Ok) { identity_ok = false; }

    if (!identity_ok) {
        TETHER_LOGE(TAG, "Failed to read Identity Object 0x1018");
    } else {
        std::cout << "=== Identity Object (0x1018) ===" << "\n";
        std::cout << "Vendor ID:    0x" << std::hex << vendor_id << std::dec << "\n";
        std::cout << "Product Code: 0x" << std::hex << product_code << std::dec << "\n";
        std::cout << "Revision:     0x" << std::hex << revision << std::dec << "\n";
        std::cout << "Serial:       0x" << std::hex << serial << std::dec << "\n";
        std::cout.flush();
    }
}

// ---------------------------------------------------------------------------
// Main
// ---------------------------------------------------------------------------

int main(int argc, char** argv) {
    argparse::ArgumentParser program("esc211_di_monitor");
    Tether::Examples::addInterfaceArg(program);
    Tether::Examples::addSlaveArg(program);
    Tether::Examples::addDurationArg(program);
    Tether::Examples::addDebugArg(program);
    Tether::Examples::addVlanArgs(program);
    Tether::Examples::addMailboxSizeArg(program);
    Tether::Examples::addMailboxAddressArg(program);
    program.add_argument("--stream")
        .default_value(false)
        .implicit_value(true)
        .help("Stream DI values to stdout instead of ncurses UI");

    try { program.parse_args(argc, argv); }
    catch (const std::runtime_error& err) {
        std::cerr << err.what() << "\n" << program;
        return 1;
    }

    std::string iface = program.get<std::string>("--interface");
    int slave_idx = program.get<int>("--slave");
    double duration_sec = program.get<double>("--time");
    std::string debug_str = program.get<std::string>("--debug");
    bool stream_mode = program.get<bool>("--stream");

    if (Tether::Examples::printDebugHelpIfRequested(debug_str)) return 0;
    auto debug_flags = Tether::Examples::parseDebugFlags(debug_str);
    Tether::Examples::applyDebugFlags(debug_flags, TAG);

    Tether::Examples::VlanConfig vlan;
    if (!Tether::Examples::parseVlanArgs(
            program.get<std::string>("--rx-vlan"),
            program.get<std::string>("--tx-vlan"),
            vlan, TAG)) {
        return 1;
    }

    Tether::Examples::MailboxSizeConfig mbSize;
    if (!Tether::Examples::parseMailboxSize(program.get<std::string>("--mailbox-size"), mbSize)) {
        return 1;
    }
    Tether::Examples::MailboxAddressConfig mbAddr;
    if (!Tether::Examples::parseMailboxAddress(program.get<std::string>("--mailbox-address"), mbAddr)) {
        return 1;
    }

    TETHER_LOGI(TAG, "esc211_di_monitor — interface: %s, slave: %d",
                iface.c_str(), slave_idx);
    Tether::Examples::logMailboxConfig(mbSize, mbAddr, TAG);

    Tether::Examples::HostEtherNetSession session;
    if (!Tether::Examples::initHostEthernet(session, iface, TAG)) {
        return 2;
    }

    EtherCAT::Master master;
    g_master = &master;

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
    }

    uint16_t slaves = master.getDiscoveredSlaveCount();
    TETHER_LOGI(TAG, "Discovered %u slave(s)", slaves);
    master.logDiscoveredSlavesSummary(TAG);

    if (debug_flags.count("sii-derivation") && slaves > 0) {
        TETHER_LOGI(TAG, "\n=== SII Mailbox Derivation Debug ===");
        for (uint16_t i = 0; i < slaves; i++) {
            EtherCAT::SII::debugSIIMailboxDerivation(master, i, TAG);
        }
    }

    if (debug_flags.count("mailbox-configuration") && slaves > 0) {
        TETHER_LOGI(TAG, "\n=== Mailbox Hardware Configuration Debug ===");
        for (uint16_t i = 0; i < slaves; i++) {
            EtherCAT::debugMailboxConfiguration(master, i, TAG);
        }
    }

    if (slaves == 0) {
        TETHER_LOGE(TAG, "No slaves found — check wiring, power, and interface name");
        master.stop();
        Tether::Examples::shutdownHostEthernet(session);
        return 4;
    }

    if (slave_idx < 0 || static_cast<uint16_t>(slave_idx) >= slaves) {
        TETHER_LOGE(TAG, "Slave index %d out of range (0..%u)", slave_idx, slaves - 1);
        master.stop();
        Tether::Examples::shutdownHostEthernet(session);
        return 4;
    }

    // ---- Verify identity (optional but helpful) ----
    auto& sl = master.slave(static_cast<uint16_t>(slave_idx));
    EtherCAT::Identity::SlaveIdentity expected_id;
    expected_id.vendor_id = EtherCAT::Drives::NexcobotESC211::kVendorId;
    expected_id.product_code = EtherCAT::Drives::NexcobotESC211::kProductCode;
    master.verifySlaveIdentity(static_cast<uint16_t>(slave_idx), expected_id, false, TAG);

    TETHER_LOGI(TAG, "Configuring mailbox for slave %d...", slave_idx);
    auto mb_err = sl.configureMailbox(
        {.address = mbAddr.outAddress, .length = mbSize.outSize},
        {.address = mbAddr.inAddress, .length = mbSize.inSize},
        0x0004);
    if (mb_err != EtherCAT::SlaveError::Ok) {
        TETHER_LOGE(TAG, "Mailbox config failed: %s", EtherCAT::slaveErrorToString(mb_err));
        master.stop();
        Tether::Examples::shutdownHostEthernet(session);
        return 7;
    }

    auto pre_err = sl.transitionToPreOp();
    if (pre_err != EtherCAT::SlaveError::Ok) {
        TETHER_LOGE(TAG, "PRE-OP transition failed: %s", EtherCAT::slaveErrorToString(pre_err));
        master.stop();
        Tether::Examples::shutdownHostEthernet(session);
        return 7;
    }

    TETHER_LOGI(TAG, "Slave %d in PRE-OP", slave_idx);

    // readIdentityObject(sl);

    auto pdo_err = sl.configurePDOSyncManagers();
    if (pdo_err != EtherCAT::SlaveError::Ok) {
        TETHER_LOGE(TAG, "PDO sync-manager config failed: %s", EtherCAT::slaveErrorToString(pdo_err));
        master.stop();
        Tether::Examples::shutdownHostEthernet(session);
        return 7;
    }

    auto safe_err = sl.transitionToSafeOp();
    if (safe_err != EtherCAT::SlaveError::Ok) {
        TETHER_LOGE(TAG, "SAFE-OP transition failed: %s", EtherCAT::slaveErrorToString(safe_err));
        master.stop();
        Tether::Examples::shutdownHostEthernet(session);
        return 7;
    }

    TETHER_LOGI(TAG, "Slave %d in SAFE-OP", slave_idx);

    master.stop();
    Tether::Examples::shutdownHostEthernet(session);
    return 0;
}

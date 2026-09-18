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
#include <memory>
#include <mutex>
#include <sstream>
#include <string>
#include <thread>
#include <print>
#include <format>

#include <clocale>

#include "tether/drives/NexcobotESC211/NexcobotESC211Registers.hpp"

namespace Reg = EtherCAT::Drives::Registers::NexcobotESC211;

// ncurses defines OK/ERR as macros; undefine them before Tether headers
// that use Error::OK are parsed.
#include <ncurses.h>
#undef OK
#undef ERR
#include "tether/ethercat/Master.hpp"
#include "tether/ethercat/Slave.hpp"
#include "tether/ethercat/Types.hpp"
#include "tether/ethercat/SyncManager.hpp"
#include "tether/ethercat/PDOManager.hpp"
#include "tether/ethercat/ALResetController.hpp"
#include "tether/sii/SIIReader.hpp"
#include "tether/sii/SIIParser.hpp"
#include "tether/terminal_ui/PanelScreen.hpp"
#include "tether/utils/SignalHandler.hpp"

#include "tether/drives/NexcobotESC211/PdoSetup.hpp"

#include "common/ExampleHelpers.hpp"
#include "common/EtherCATHostSetup.hpp"

namespace TUI = Tether::TUI;

// Packed TxPDO 0x1A01 (7 entries x UDINT = 28 bytes, per ESI v0.9)
struct TxPDO_1A01_Remapped {
    uint32_t input_counter;
    uint32_t safe_di;
    uint32_t power_status;
    uint32_t do_monitor;
    uint32_t do_valu;
    uint32_t di_valu;
    uint32_t do_command;
} __attribute__((packed));
static_assert(sizeof(TxPDO_1A01_Remapped) == 28, "TxPDO_1A01_Remapped size mismatch");

static const char* TAG = "esc211_di_monitor";
static EtherCAT::Master* g_master = nullptr;

// ---------------------------------------------------------------------------
// Shared DI state between PDO exchange loop and ncurses display
// ---------------------------------------------------------------------------

struct DIState {
    mutable std::mutex mtx;
    std::array<bool, 10> ia{};   // IA1..IA10  (di_val bits 0–9)
    std::array<bool, 10> ib{};   // IB1..IB10  (di_val bits 16–25)
    bool stale = true;
    uint64_t read_count = 0;
    uint32_t last_safe_di = 0;
};

// Extract IA/IB from the 32-bit SAFE_DI field
static void parseSafeDI(uint32_t di_val, DIState& state) {
    std::lock_guard<std::mutex> lock(state.mtx);
    for (int i = 0; i < 10; ++i) {
        state.ia[i] = (di_val >> i) & 1u;           // IA1..IA10 = bits 0..9
        state.ib[i] = (di_val >> (i + 16)) & 1u;    // IB1..IB10 = bits 16..25
    }
    state.last_safe_di = di_val;
    state.stale = false;
    ++state.read_count;
}

// ---------------------------------------------------------------------------
// PanelScreen body renderer
// ---------------------------------------------------------------------------

/// Draw the DI panel into stdscr rows [top, bottom) — called each frame by
/// PanelScreen, which owns the ncurses session, header, log pane and
/// footer.  Uses the shared Session palette (PalValue=green active,
/// PalError=red stale, PalMuted=inactive).
static void drawBody(const DIState& state, int top, int bottom) {
    std::lock_guard<std::mutex> lock(state.mtx);

    int y = top;
    if (y >= bottom) return;

    if (state.stale) {
        attron(COLOR_PAIR(TUI::PalError));
        mvprintw(y, 0, "[STALE / SDO ERROR]");
        attroff(COLOR_PAIR(TUI::PalError));
    }

    // Header (same row as the stale banner, different columns — as before)
    attron(A_BOLD);
    mvprintw(y, 2,  "IA (left)");
    mvprintw(y, 22, "IB (right)");
    attroff(A_BOLD);
    ++y;

    for (int row = 0; row < 10 && y < bottom; ++row, ++y) {
        // Left: IA1..IA10
        bool a = state.ia[row];
        mvprintw(y, 2, "IA%2d", row + 1);
        if (a) {
            attron(COLOR_PAIR(TUI::PalValue));
            mvprintw(y, 8, "  ON");
            attroff(COLOR_PAIR(TUI::PalValue));
        } else {
            mvprintw(y, 8, " off");
        }

        // Right: IB1..IB10
        bool b = state.ib[row];
        mvprintw(y, 22, "IB%2d", row + 1);
        if (b) {
            attron(COLOR_PAIR(TUI::PalValue));
            mvprintw(y, 28, "  ON");
            attroff(COLOR_PAIR(TUI::PalValue));
        } else {
            mvprintw(y, 28, " off");
        }
    }
    ++y;

    if (y < bottom) {
        mvprintw(y++, 0, "Read count: %llu  di_val=0x%08X",
                 static_cast<unsigned long long>(state.read_count),
                 state.last_safe_di);
    }
}

// ---------------------------------------------------------------------------
// Main
// ---------------------------------------------------------------------------

int main(int argc, char** argv) {
    Tether::Utils::SignalHandler sig_handler;

    argparse::ArgumentParser program("esc211_di_monitor");
    Tether::Examples::addInterfaceArg(program);
    Tether::Examples::addSlaveArg(program);
    Tether::Examples::addDurationArg(program);
    Tether::Examples::addDebugArg(program);
    Tether::Examples::addDebugConditionArgs(program);
    Tether::Examples::addVlanArgs(program);
    Tether::Examples::addMailboxSizeArg(program, 512);
    Tether::Examples::addMailboxAddressArg(program);
    program.add_argument("--stream")
        .default_value(false)
        .implicit_value(true)
        .help("Stream DI values to stdout instead of ncurses UI");

    try { program.parse_args(argc, argv); }
    catch (const std::runtime_error& err) {
        std::ostringstream oss;
        oss << program;
        std::print(stderr, "{}\n{}", err.what(), oss.str());
        return 1;
    }

    std::string iface = Tether::Examples::resolveInterface(program.get<std::string>("--interface"), TAG);
    int slave_idx = program.get<int>("--slave");
    double duration_sec = program.get<double>("--time");
    std::string debug_str = program.get<std::string>("--debug");
    bool stream_mode = program.get<bool>("--stream");

    if (Tether::Examples::printDebugHelpIfRequested(debug_str)) return 0;
    auto debug_flags = Tether::Examples::parseDebugFlags(debug_str);

    std::string debug_start = program.get<std::string>("--debug-start");
    std::string debug_stop = program.get<std::string>("--debug-stop");
    if (Tether::Examples::printDebugConditionHelpIfRequested(debug_start)) return 0;

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

    TETHER_LOGI(TAG, "esc211_di_monitor — interface: {}, slave: {}",
                iface.c_str(), slave_idx);
    Tether::Examples::logMailboxConfig(mbSize, mbAddr, TAG);

    Tether::Examples::HostEtherNetSession session;
    if (!Tether::Examples::initHostEthernet(session, iface, TAG)) {
        return 2;
    }

    EtherCAT::Master master;
    g_master = &master;
    sig_handler.setCancelCallback([&master]() { master.requestCancel(); });
    Tether::Examples::applyDebugFlags(debug_flags, master, TAG);
    Tether::Examples::applyDebugGateConditions(debug_start, debug_stop, master, TAG);

    if (!Tether::Examples::setupVlanAndRxCallback(session, master, vlan, TAG)) {
        Tether::Examples::shutdownHostEthernet(session);
        return 5;
    }

    Tether::Examples::startHostPollThread(session, TAG);

    if (!Tether::Examples::startHostMaster(session, master, vlan, TAG)) {
        Tether::Examples::shutdownHostEthernet(session);
        return 5;
    }

    auto discovered = master.discovery().discover();
    if (discovered.empty()) {
        TETHER_LOGW(TAG, "No slaves discovered");
    }

    uint16_t slaves = master.getDiscoveredSlaveCount();
    TETHER_LOGI(TAG, "Discovered {} slave(s)", slaves);
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
        TETHER_LOGE(TAG, "Slave index {} out of range (0..{})", slave_idx, slaves - 1);
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

    // Force slave to INIT before any configuration.  This ensures a clean
    // starting state regardless of what the slave firmware was doing before
    // (e.g. stuck in PRE-OP with stale mailbox data from a previous run,
    // or in an error state after a failed init).
    TETHER_LOGI(TAG, "Forcing slave {} to INIT before configuration...", slave_idx);
    {
        EtherCAT::ALResetController reset_ctrl(master);
        auto reset_result = reset_ctrl.resetSlave(static_cast<uint16_t>(slave_idx), 0x01, 50, 50);
        if (!reset_result.success) {
            TETHER_LOGW(TAG, "AL reset to INIT failed (AL_STATUS=0x{:04X}, code=0x{:04X}) — continuing",
                        reset_result.final_al_status, reset_result.final_al_status_code);
        }
    }

    TETHER_LOGI(TAG, "Configuring mailbox for slave {}...", slave_idx);
    auto mb_err = sl.configureMailbox(
        {.address = mbAddr.outAddress, .length = mbSize.outSize},
        {.address = mbAddr.inAddress, .length = mbSize.inSize},
        0x0004);
    if (mb_err != EtherCAT::SlaveError::Ok) {
        TETHER_LOGE(TAG, "Mailbox config failed: {}", EtherCAT::slaveErrorToString(mb_err));
        master.stop();
        Tether::Examples::shutdownHostEthernet(session);
        return 7;
    }

    auto pre_err = sl.transitionToPreOp();
    if (pre_err != EtherCAT::SlaveError::Ok) {
        TETHER_LOGE(TAG, "PRE-OP transition failed: {}", EtherCAT::slaveErrorToString(pre_err));
        master.stop();
        Tether::Examples::shutdownHostEthernet(session);
        return 7;
    }

    TETHER_LOGI(TAG, "Slave {} in PRE-OP", slave_idx);

    // Drain any stale mailbox data that the slave firmware may have written
    // into SM1 during or after the PRE-OP transition.  configureMailbox()
    // drains once before the transition, but the firmware can emit AL status
    // notifications or initialization messages as it enters PRE-OP, leaving
    // stale data whose mailbox counter doesn't match the master's first SDO
    // request — this causes "Stale mailbox response" errors and SDO failures
    // on the first PDO mapping write.
    TETHER_LOGI(TAG, "Draining stale mailbox data after PRE-OP transition...");
    if (!sl.drainMailbox()) {
        TETHER_LOGW(TAG, "Mailbox drain after PRE-OP did not complete — "
                    "stale responses may occur on first SDO exchange");
    }

    // ---- ESI PDO assignment (see PdoSetup.hpp): mandatory status
    //      PDOs 0x1601/0x1A01 + RSAP PDOs 0x1A02/0x1A03, registered from the
    //      device's own mapping (assign-only, no mapping rewrite). ----
    TETHER_LOGI(TAG, "Configuring ESI PDO assignment (status + RSAP)...");
    if (!EtherCAT::Drives::ESC211::configureEsc211PdoAssignment(
            sl, TAG, /*flat_fsoe_maps=*/false,
            /*fsoe_channels=*/0)) {
        TETHER_LOGE(TAG, "ESI PDO assignment failed");
        master.stop();
        Tether::Examples::shutdownHostEthernet(session);
        return 7;
    }

    // Register PDO buffers and assign PDOs to sync managers
    auto apply_err = sl.applyCustomPDOs();
    if (apply_err != EtherCAT::SlaveError::Ok) {
        TETHER_LOGE(TAG, "Apply custom PDOs failed: {}", EtherCAT::slaveErrorToString(apply_err));
        master.stop();
        Tether::Examples::shutdownHostEthernet(session);
        return 7;
    }

    auto pdo_err = sl.configurePDOSyncManagers();
    if (pdo_err != EtherCAT::SlaveError::Ok) {
        TETHER_LOGE(TAG, "PDO sync-manager config failed: {}", EtherCAT::slaveErrorToString(pdo_err));
        master.stop();
        Tether::Examples::shutdownHostEthernet(session);
        return 7;
    }

    auto safe_err = sl.transitionToSafeOp();
    if (safe_err != EtherCAT::SlaveError::Ok) {
        TETHER_LOGE(TAG, "SAFE-OP transition failed: {}", EtherCAT::slaveErrorToString(safe_err));
        master.stop();
        Tether::Examples::shutdownHostEthernet(session);
        return 7;
    }

    TETHER_LOGI(TAG, "Slave {} in SAFE-OP", slave_idx);

    // ---- PDO exchange + display loop ----
    using namespace std::chrono;
    const auto period = milliseconds(20);  // 50 Hz
    auto next_time = steady_clock::now();
    uint64_t cycle = 0;

    // Look up a registered custom-PDO application buffer by PDO mapping
    // index + direction (buffers are registered by applyCustomPDOs()).
    auto findPDOBuffer = [&](uint16_t pdo_index,
                             EtherCAT::PDO::PDODirection dir) -> uint8_t* {
        const auto& mapping = master.pdo().mapping();
        for (size_t i = 0; i < mapping.entry_count(); ++i) {
            const auto* entry = mapping.get_entry(i);
            if (entry && entry->slave_index == static_cast<uint16_t>(slave_idx) &&
                entry->direction == dir && entry->pdo_index == pdo_index) {
                return static_cast<uint8_t*>(entry->app_buffer);
            }
        }
        return nullptr;
    };

    // Helper: exchange PDOs and return the 0x1A01 TxPDO pointer.
    auto getTxPDO = [&]() -> const TxPDO_1A01_Remapped* {
        EtherCAT::Drives::ESC211::bumpOutputCounter(master.pdo(),
                                                  static_cast<uint16_t>(slave_idx));
        if (!master.pdo().exchangeAll()) return nullptr;
        return reinterpret_cast<const TxPDO_1A01_Remapped*>(
            findPDOBuffer(0x1A01, EtherCAT::PDO::PDODirection::TxPDO));
    };

    if (stream_mode) {
        TETHER_LOGI(TAG, "Streaming TxPDO 0x1A01 (DI monitor)...");
        while (!sig_handler.stop_requested()) {
            const auto* tx = getTxPDO();
            if (tx) {
                std::print("cycle={} ic=0x{:X} safe_di=0x{:X} pwr=0x{:X} do_mon=0x{:X} do_val=0x{:X} di_val=0x{:X} do_cmd=0x{:X}\n",
                           cycle, tx->input_counter, tx->safe_di, tx->power_status,
                           tx->do_monitor, tx->do_valu, tx->di_valu, tx->do_command);
            } else {
                TETHER_LOGW(TAG, "PDO exchange failed (cycle {})",
                            static_cast<unsigned long long>(cycle));
            }
            ++cycle;
            next_time += period;
            std::this_thread::sleep_until(next_time);
        }
    } else {
        // ncurses UI mode — TUI::PanelScreen owns the curses session,
        // palette, header/log/footer layout and key loop.  The 50 Hz PDO
        // exchange runs on a background thread into the mutex-guarded
        // DIState so rendering and bus I/O stay decoupled.
        DIState di_state;
        std::atomic<bool> cancel{false};
        sig_handler.setCancelCallback([&] {
            cancel.store(true);
            master.requestCancel();
        });

        std::thread pdo_thread([&] {
            auto next = steady_clock::now();
            while (!cancel.load(std::memory_order_relaxed)) {
                const auto* tx = getTxPDO();
                if (tx) {
                    parseSafeDI(tx->di_valu, di_state);
                } else {
                    std::lock_guard<std::mutex> lock(di_state.mtx);
                    di_state.stale = true;
                }
                next += period;
                std::this_thread::sleep_until(next);
            }
        });

        TETHER_LOGI(TAG, "Starting ncurses DI monitor (PDO-based)...");

        TUI::PanelScreenHooks hooks;
        hooks.renderBody = [&](int top, int bottom) {
            drawBody(di_state, top, bottom);
        };

        TUI::PanelScreen screen(
            "Nexcobot ESC211 — Safety Digital Inputs (50 Hz)",
            std::move(hooks));
        screen.run(cancel, duration_sec);

        cancel.store(true);
        pdo_thread.join();
    }

    master.stop();
    Tether::Examples::shutdownHostEthernet(session);
    return 0;
}

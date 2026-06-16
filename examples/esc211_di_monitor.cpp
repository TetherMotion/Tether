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
#include <set>
#include <sstream>
#include <string>
#include <thread>
#include <magic_enum/magic_enum.hpp>

#include <clocale>
#include <optional>

#include "tether/drives/NexcobotESC211/NexcobotESC211Registers.hpp"

// ncurses defines OK/ERR as macros; undefine them before Tether headers
// that use Error::OK are parsed.
#include <ncurses.h>
#undef OK
#undef ERR
#include "tether/ethercat/Master.hpp"
#include "tether/ethercat/Slave.hpp"
#include "tether/ethercat/Types.hpp"
#include "tether/ethercat/VLANRouter.hpp"
#include "tether/hal/IEthernet.hpp"
#include "tether/platform/Platform.hpp"
#include "tether/sii/SIIReader.hpp"

#include <argparse/argparse.hpp>

// Forward-declare host transport helpers
namespace EtherCAT {
namespace Raw {
    void set_network_interface(const ::EtherCAT::NetworkInterface* iface);
    const ::EtherCAT::NetworkInterface* network_interface();
    void set_src_mac(const uint8_t src_mac[6]);
}
}

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
// Main
// ---------------------------------------------------------------------------

int main(int argc, char** argv) {
    // ---- Argument parsing ----
    argparse::ArgumentParser program("esc211_di_monitor");
    program.add_argument("-i", "--interface")
        .default_value(std::string("eth0"))
        .help("Network interface name (e.g. eth0, enp3s0)");
    program.add_argument("-s", "--slave")
        .scan<'i', int>()
        .default_value(0)
        .help("Slave index on the bus (0-based)");
    program.add_argument("-t", "--time")
        .scan<'g', double>()
        .default_value(0.0)
        .help("Monitor duration in seconds (0 = infinite until Ctrl-C)");
    program.add_argument("--debug")
        .default_value(std::string(""))
        .help("Comma-separated debug flags. Known flags: sii-derivation, mailbox-configuration, al-state, tx-ethercat-packets, rx-ethercat-packets");
    program.add_argument("--rx-vlan")
        .default_value(std::string(""))
        .help("RX VLAN filter: single VID, range, or 'any'");
    program.add_argument("--tx-vlan")
        .default_value(std::string(""))
        .help("TX VLAN encapsulation: single VID");
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
    std::string rx_vlan_str = program.get<std::string>("--rx-vlan");
    std::string tx_vlan_str = program.get<std::string>("--tx-vlan");
    bool stream_mode = program.get<bool>("--stream");

    const std::set<std::string> known_debug_flags = {
        "sii-derivation",
        "mailbox-configuration",
        "al-state",
        "tx-ethercat-packets",
        "rx-ethercat-packets"
    };

    std::set<std::string> debug_flags;
    std::set<std::string> unknown_flags;
    if (!debug_str.empty()) {
        std::stringstream ss(debug_str);
        std::string flag;
        while (std::getline(ss, flag, ',')) {
            flag.erase(0, flag.find_first_not_of(" \t"));
            flag.erase(flag.find_last_not_of(" \t") + 1);
            if (!flag.empty()) {
                debug_flags.insert(flag);
                if (known_debug_flags.find(flag) == known_debug_flags.end()) {
                    unknown_flags.insert(flag);
                }
            }
        }
    }
    if (!unknown_flags.empty()) {
        TETHER_LOGW(TAG, "Unknown debug flags:");
        for (const auto& f : unknown_flags) TETHER_LOGW(TAG, "  - %s", f.c_str());
        TETHER_LOGI(TAG, "Available debug flags:");
        for (const auto& f : known_debug_flags) TETHER_LOGI(TAG, "  - %s", f.c_str());
    }
    if (debug_flags.count("al-state")) {
        EtherCAT::enableStateMachineDebug(true);
        TETHER_LOGI(TAG, "EtherCAT state machine debug enabled");
    }
    if (debug_flags.count("tx-ethercat-packets")) {
        EtherCAT::enableTxPacketDebug(true);
        TETHER_LOGI(TAG, "TX packet debug enabled");
    }
    if (debug_flags.count("rx-ethercat-packets")) {
        EtherCAT::enableRxPacketDebug(true);
        TETHER_LOGI(TAG, "RX packet debug enabled");
    }

    // ---- Parse VLAN arguments ----
    bool vlan_mode = !rx_vlan_str.empty() || !tx_vlan_str.empty();
    std::optional<uint16_t> tx_vlan;
    bool rx_any = false;
    std::optional<EtherCAT::VLANRouter::VLANRange> rx_range;

    if (vlan_mode) {
        if (!tx_vlan_str.empty()) {
            try {
                int v = std::stoi(tx_vlan_str);
                if (v < 1 || v > 4095) {
                    std::cerr << "--tx-vlan must be in range 1-4095\n";
                    return 1;
                }
                tx_vlan = static_cast<uint16_t>(v);
            } catch (...) {
                std::cerr << "Invalid --tx-vlan value: " << tx_vlan_str << "\n";
                return 1;
            }
        }
        if (!rx_vlan_str.empty()) {
            if (rx_vlan_str == "any") {
                rx_any = true;
            } else {
                size_t dash = rx_vlan_str.find('-');
                try {
                    if (dash == std::string::npos) {
                        int v = std::stoi(rx_vlan_str);
                        if (v < 1 || v > 4095) {
                            std::cerr << "--rx-vlan must be in range 1-4095\n";
                            return 1;
                        }
                        rx_range = EtherCAT::VLANRouter::VLANRange{
                            static_cast<uint16_t>(v), static_cast<uint16_t>(v)};
                    } else {
                        int start = std::stoi(rx_vlan_str.substr(0, dash));
                        int end   = std::stoi(rx_vlan_str.substr(dash + 1));
                        if (start < 1 || end > 4095 || start > end) {
                            std::cerr << "--rx-vlan range must be 1-4095 with start <= end\n";
                            return 1;
                        }
                        rx_range = EtherCAT::VLANRouter::VLANRange{
                            static_cast<uint16_t>(start), static_cast<uint16_t>(end)};
                    }
                } catch (...) {
                    std::cerr << "Invalid --rx-vlan value: " << rx_vlan_str << "\n";
                    return 1;
                }
            }
        }
    }

    TETHER_LOGI(TAG, "esc211_di_monitor — interface: %s, slave: %d",
                iface.c_str(), slave_idx);

    // ---- Open raw socket ----
    auto eth = EtherCAT::HAL::createDefaultEthernet();
    if (!eth) { TETHER_LOGE(TAG, "No Ethernet HAL available"); return 1; }

    EtherCAT::HAL::EthernetConfig cfg;
    cfg.interfaceName = iface.c_str();
    cfg.promiscuous   = true;
    cfg.ethertypeFilter = static_cast<uint16_t>(EtherCAT::kEtherTypeEtherCAT);

    {
        auto err = eth->init(cfg);
        if (err != EtherCAT::HAL::Error::OK) {
            if (err == EtherCAT::HAL::Error::InterfaceNotFound)
                TETHER_LOGE(TAG, "Interface '%s' not found", iface.c_str());
            else if (err == EtherCAT::HAL::Error::PermissionDenied)
                TETHER_LOGE(TAG, "Permission denied — run as root or with CAP_NET_RAW");
            else
                TETHER_LOGE(TAG, "Failed to init '%s' (%s)", iface.c_str(),
                            magic_enum::enum_name(err).data());
            return 2;
        }
        auto ls = eth->getLinkStatus();
        if (!ls.up) {
            TETHER_LOGE(TAG, "Link DOWN on '%s'", iface.c_str());
            return 6;
        }
    }

    // ---- MAC + NetworkInterface ----
    EtherCAT::HAL::MacAddress mac;
    if (eth->getMacAddress(mac) != EtherCAT::HAL::Error::OK) {
        TETHER_LOGE(TAG, "Failed to read MAC address");
        return 3;
    }
    uint8_t src_mac[6];
    std::memcpy(src_mac, mac.bytes, 6);

    auto ni_ptr = std::make_unique<EtherCAT::NetworkInterface>();
    ni_ptr->send = [eth_raw = eth.get()](const uint8_t* data, size_t len) -> bool {
        return eth_raw->transmit(data, len) == EtherCAT::HAL::Error::OK;
    };
    EtherCAT::Raw::set_network_interface(ni_ptr.get());
    EtherCAT::Raw::set_src_mac(src_mac);

    // ---- Create master ----
    EtherCAT::Master master;
    g_master = &master;

    // ---- Optional VLAN router ----
    std::unique_ptr<EtherCAT::VLANRouter> router;
    if (vlan_mode) {
        router = std::make_unique<EtherCAT::VLANRouter>();
        router->setBackend(ni_ptr.get());
        if (rx_any) {
            router->setUndefinedTarget(
                std::shared_ptr<EtherCAT::Master>(&master, [](auto*){}),
                tx_vlan, true);
        } else if (rx_range) {
            router->addMaster(
                std::shared_ptr<EtherCAT::Master>(&master, [](auto*){}),
                *rx_range, tx_vlan);
        } else {
            router->addMaster(
                std::shared_ptr<EtherCAT::Master>(&master, [](auto*){}),
                std::nullopt, tx_vlan);
        }
    }

    if (router) {
        eth->setRxCallback([&router](const uint8_t* frame, size_t len,
                                      const EtherCAT::HAL::RxFrameInfo&, void*) {
            router->processRxFrame(frame, len);
        }, nullptr);
    } else {
        eth->setRxCallback([&master](const uint8_t* frame, size_t len,
                                      const EtherCAT::HAL::RxFrameInfo&, void*) {
            master.handleRxFrame(frame, len);
        }, nullptr);
    }

    // ---- Poll thread ----
    std::atomic<bool> poll_running{true};
    std::thread poll_thread([&]() {
        if (!Tether::Platform::setCurrentThreadRealtime(-1)) {
            TETHER_LOGW(TAG, "poll_thread: could not set realtime scheduling (continuing)");
        }
        while (poll_running.load()) eth->poll(1);
    });

    // ---- Start master + discover ----
    if (router) {
        EtherCAT::NetworkInterface* master_iface = rx_any
            ? router->undefinedNetworkInterface()
            : router->networkInterfaceFor(&master);
        if (!master_iface) {
            TETHER_LOGE(TAG, "Failed to obtain per-master NetworkInterface from VLAN router");
            poll_running = false;
            poll_thread.join();
            eth->shutdown();
            return 5;
        }
        master.start(*master_iface, src_mac);
    } else {
        master.start(*EtherCAT::Raw::network_interface(), src_mac);
    }

    if (!master.discoverSlaves()) {
        TETHER_LOGW(TAG, "No slaves discovered");
    }

    uint16_t slaves = master.getDiscoveredSlaveCount();
    TETHER_LOGI(TAG, "Discovered %u slave(s)", slaves);
    master.logDiscoveredSlavesSummary(TAG);

    if (slaves == 0) {
        TETHER_LOGE(TAG, "No slaves found — check wiring, power, and interface name");
        master.stop();
        poll_running = false;
        poll_thread.join();
        eth->shutdown();
        return 4;
    }

    if (slave_idx < 0 || static_cast<uint16_t>(slave_idx) >= slaves) {
        TETHER_LOGE(TAG, "Slave index %d out of range (0..%u)", slave_idx, slaves - 1);
        master.stop();
        poll_running = false;
        poll_thread.join();
        eth->shutdown();
        return 4;
    }

    // ---- Verify identity (optional but helpful) ----
    auto& sl = master.slave(static_cast<uint16_t>(slave_idx));
    EtherCAT::Identity::SlaveIdentity expected_id;
    expected_id.vendor_id = EtherCAT::Drives::NexcobotESC211::kVendorId;
    expected_id.product_code = EtherCAT::Drives::NexcobotESC211::kProductCode;
    master.verifySlaveIdentity(static_cast<uint16_t>(slave_idx), expected_id, false, TAG);

    // ---- Configure mailbox + transition to PRE-OP ----
    // Override with ESI values to avoid "Mailbox size too large" from SII:
    //   SM0 (MBoxOut / M->S): addr=0x1000 len=0x200 ctrl=0x26
    //   SM1 (MBoxIn  / S->M): addr=0x1200 len=0x200 ctrl=0x22  proto=CoE (0x0004)
    TETHER_LOGI(TAG, "Configuring mailbox for slave %d...", slave_idx);
    auto mb_err = sl.configureMailbox(
        {.address = 0x1000, .length = 0x0100},
        {.address = 0x1200, .length = 0x0100},
        0x0004);
    if (mb_err != EtherCAT::SlaveError::Ok) {
        TETHER_LOGE(TAG, "Mailbox config failed: %s", EtherCAT::slaveErrorToString(mb_err));
        master.stop();
        poll_running = false;
        poll_thread.join();
        eth->shutdown();
        return 7;
    }

    auto pre_err = sl.transitionToPreOp();
    if (pre_err != EtherCAT::SlaveError::Ok) {
        TETHER_LOGE(TAG, "PRE-OP transition failed: %s", EtherCAT::slaveErrorToString(pre_err));
        master.stop();
        poll_running = false;
        poll_thread.join();
        eth->shutdown();
        return 7;
    }

    TETHER_LOGI(TAG, "Slave %d in PRE-OP — starting SDO reads", slave_idx);

    // ---- Install signal handler ----
    std::signal(SIGINT, signalHandler);
    std::signal(SIGTERM, signalHandler);

    // ---- Init ncurses (unless streaming) ----
    if (!stream_mode) {
        setlocale(LC_ALL, "");
        initscr();
        cbreak();
        noecho();
        nodelay(stdscr, TRUE);
        curs_set(0);
        initColors();
    }

    // ---- Start SDO reader thread ----
    DIState di_state;
    std::thread reader_thread(sdoReaderThread, std::ref(sl), std::ref(di_state));

    // ---- Display / stream loop ----
    auto start_time = std::chrono::steady_clock::now();
    auto end_time = start_time + std::chrono::seconds(static_cast<int>(duration_sec));

    while (!g_cancel.load(std::memory_order_relaxed)) {
        if (duration_sec > 0.0 && std::chrono::steady_clock::now() >= end_time) {
            g_cancel.store(true);
            break;
        }
        if (stream_mode) {
            std::lock_guard<std::mutex> lock(di_state.mtx);
            if (di_state.stale) {
                std::cout << "[STALE]\n";
            } else {
                std::cout << "count=" << di_state.read_count << " IA=";
                for (bool v : di_state.ia) std::cout << (v ? '1' : '0');
                std::cout << " IB=";
                for (bool v : di_state.ib) std::cout << (v ? '1' : '0');
                std::cout << "\n";
            }
            std::cout.flush();
        } else {
            drawScreen(di_state);
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }

    // ---- Cleanup ----
    g_cancel.store(true);
    reader_thread.join();

    if (!stream_mode) {
        endwin();
    }

    master.stop();
    poll_running = false;
    poll_thread.join();
    eth->shutdown();

    TETHER_LOGI(TAG, "Exiting.");
    return 0;
}

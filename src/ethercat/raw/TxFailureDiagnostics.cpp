/**
 * @file TxFailureDiagnostics.cpp
 * @brief Wire-level TX failure probe + non-RT escalation worker.
 */

#include "raw/TxFailureDiagnostics.hpp"

#include "tether/platform/Platform.hpp"

#include <chrono>
#include <cstring>
#include <format>
#include <string>

#ifdef __linux__
#include <cerrno>
#include <fstream>
#include <net/if.h>
#include <linux/ethtool.h>
#include <linux/if_packet.h>
#include <linux/sockios.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>
#endif

namespace EtherCAT {

#ifdef __linux__
namespace {
long txDiagSysfsLong(const std::string& path)
{
    std::ifstream f(path);
    long v = -1;
    if (f >> v) return v;
    return -1;
}
std::string txDiagSysfsLine(const std::string& path)
{
    std::ifstream f(path);
    std::string s;
    std::getline(f, s);
    return s;
}
} // namespace
#endif

std::string TxFailureDiagnostics::probe(int fd)
{
#ifdef __linux__
    if (fd < 0) {
        return "no TX socket fd available — cannot diagnose (router stub?)";
    }

    sockaddr_ll sll{};
    socklen_t sll_len = sizeof(sll);
    if (getsockname(fd, reinterpret_cast<sockaddr*>(&sll), &sll_len) < 0 ||
        sll.sll_family != AF_PACKET || sll.sll_ifindex == 0) {
        return std::format("tx fd {} is not a bound AF_PACKET socket — "
                           "no interface to diagnose", fd);
    }

    int ctl = ::socket(AF_INET, SOCK_DGRAM, 0);
    if (ctl < 0) {
        return std::format("cannot open ioctl socket: {}", strerror(errno));
    }

    char ifname[IFNAMSIZ] = {};
    ifreq ifr{};
    ifr.ifr_ifindex = sll.sll_ifindex;
    if (ioctl(ctl, SIOCGIFNAME, &ifr) == 0) {
        std::strncpy(ifname, ifr.ifr_name, IFNAMSIZ - 1);
    }

    bool admin_up = false, running = false;
    if (ifname[0] && ioctl(ctl, SIOCGIFFLAGS, &ifr) == 0) {
        admin_up = (ifr.ifr_flags & IFF_UP) != 0;
        running  = (ifr.ifr_flags & IFF_RUNNING) != 0;
    }

    long ethtool_link = -1;
    if (ifname[0]) {
        ethtool_value eval{};
        eval.cmd = ETHTOOL_GLINK;
        std::memset(&ifr, 0, sizeof(ifr));
        std::strncpy(ifr.ifr_name, ifname, IFNAMSIZ - 1);
        ifr.ifr_data = reinterpret_cast<char*>(&eval);
        if (ioctl(ctl, SIOCETHTOOL, &ifr) == 0) {
            ethtool_link = static_cast<long>(eval.data);
        }
    }

    // Pending asynchronous error on the TX socket itself — a wedged
    // TX ring surfaces here (e.g. ENOBUFS) before sendto reports it.
    int so_error = 0;
    socklen_t so_len = sizeof(so_error);
    getsockopt(fd, SOL_SOCKET, SO_ERROR, &so_error, &so_len);
    close(ctl);

    const std::string base = std::format("/sys/class/net/{}", ifname);
    const long sys_carrier = txDiagSysfsLong(base + "/carrier");
    const long tx_errors   = txDiagSysfsLong(base + "/statistics/tx_errors");
    const long tx_dropped  = txDiagSysfsLong(base + "/statistics/tx_dropped");
    const long rx_dropped  = txDiagSysfsLong(base + "/statistics/rx_dropped");
    const std::string oper = txDiagSysfsLine(base + "/operstate");

    std::string report = std::format(
        "if={} idx={} admin_up={} running(IFF_RUNNING)={} ethtool_link={} "
        "operstate={} sysfs_carrier={} so_error={}({}) tx_errors={} "
        "tx_dropped={} rx_dropped={}",
        ifname[0] ? ifname : "?", sll.sll_ifindex,
        admin_up ? 1 : 0, running ? 1 : 0, ethtool_link,
        oper.empty() ? "?" : oper.c_str(), sys_carrier,
        so_error, so_error ? strerror(so_error) : "none",
        tx_errors, tx_dropped, rx_dropped);

    if (!admin_up) {
        report += " | DIAGNOSIS: interface administratively down — "
                  "'ip link set <if> up'; check NetworkManager is not "
                  "managing it";
    } else if (!running || ethtool_link == 0 || sys_carrier == 0) {
        report += " | DIAGNOSIS: link/carrier lost — TX ring cannot "
                  "drain; check cable and slave link state";
    } else if (so_error != 0) {
        report += std::format(" | DIAGNOSIS: socket carries pending error "
                              "{}({})", so_error, strerror(so_error));
    } else {
        report += " | DIAGNOSIS: link looks up — TX failure likely transient "
                  "congestion or driver-level TX stall";
    }
    return report;
#else
    (void)fd;
    return {};
#endif
}

TxFailureDiagnostics::TxFailureDiagnostics(ProbeFn probe, const char* log_tag)
    : probe_(std::move(probe)), log_tag_(log_tag)
{
}

TxFailureDiagnostics::~TxFailureDiagnostics()
{
    stop();
}

void TxFailureDiagnostics::stop()
{
    stop_.store(true, std::memory_order_release);
    if (thread_.joinable()) thread_.join();
}

void TxFailureDiagnostics::noteSendFailure()
{
    // Re-arm the diagnostic worker every 10 consecutive failures.
    const uint32_t streak =
        send_fail_streak_.fetch_add(1, std::memory_order_relaxed) + 1;
    if (streak < 10 || (streak % 10) != 0) return;

    requested_.store(true, std::memory_order_release);
    bool expected = false;
    if (!running_.compare_exchange_strong(
            expected, true, std::memory_order_acq_rel)) {
        return;  // worker alive — the flag is its re-trigger
    }
    if (thread_.joinable()) thread_.join();  // reap self-exited worker
    try {
        thread_ = std::thread(&TxFailureDiagnostics::workerMain, this);
        spawns_.fetch_add(1, std::memory_order_relaxed);
    } catch (const std::exception& e) {
        running_.store(false, std::memory_order_release);
        TETHER_LOGW(log_tag_, "TX diagnostics thread spawn failed: {}",
                    e.what());
    }
}

void TxFailureDiagnostics::workerMain()
{
    for (;;) {
        requested_.store(false, std::memory_order_release);
        const std::string report = probe_();
        if (!report.empty()) {
            TETHER_LOGE(log_tag_, "TX diagnostics — {}", report);
        }
        // The cyclic thread re-arms requested_ every 10 failed sends;
        // exit when no new failure arrives for 1 s.  10 ms poll keeps
        // destruction prompt via stop_.
        bool retriggered = false;
        for (int i = 0; i < 100 && !retriggered; ++i) {
            if (stop_.load(std::memory_order_acquire)) break;
            if (requested_.load(std::memory_order_acquire)) {
                retriggered = true;
                break;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
        if (!retriggered) break;
    }
    running_.store(false, std::memory_order_release);
}

} // namespace EtherCAT

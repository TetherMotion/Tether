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

TxFailureDiagnostics::TxFailureDiagnostics(ProbeFn probe, ErrnoFn errno_fn,
                                           EscalationFn escalation,
                                           const char* log_tag)
    : probe_(std::move(probe)), errno_fn_(std::move(errno_fn)),
      escalation_fn_(std::move(escalation)), log_tag_(log_tag)
{
}

TxFailureDiagnostics::~TxFailureDiagnostics()
{
    stop();
}

void TxFailureDiagnostics::start()
{
    bool expected = false;
    if (!running_.compare_exchange_strong(
            expected, true, std::memory_order_acq_rel)) {
        return;  // already running
    }
    if (thread_.joinable()) thread_.join();  // reap a stopped worker
    try {
        stop_.store(false, std::memory_order_release);
        thread_ = std::thread(&TxFailureDiagnostics::workerMain, this);
        spawns_.fetch_add(1, std::memory_order_relaxed);
    } catch (const std::exception& e) {
        running_.store(false, std::memory_order_release);
        TETHER_LOGW(log_tag_, "TX diagnostics thread spawn failed: {}",
                    e.what());
    }
}

void TxFailureDiagnostics::stop()
{
    stop_.store(true, std::memory_order_release);
    if (thread_.joinable()) thread_.join();
    running_.store(false, std::memory_order_release);
}

void TxFailureDiagnostics::noteSendFailure(uint32_t mask)
{
    send_fails_.fetch_add(1, std::memory_order_relaxed);
    consec_send_fails_.fetch_add(1, std::memory_order_relaxed);
    last_send_fail_mask_.store(mask, std::memory_order_relaxed);
}

void TxFailureDiagnostics::noteNoResponseSlot()
{
    no_slot_.fetch_add(1, std::memory_order_relaxed);
}

void TxFailureDiagnostics::noteTimeout(const char* what)
{
    timeouts_.fetch_add(1, std::memory_order_relaxed);
    consec_timeouts_.fetch_add(1, std::memory_order_relaxed);
    last_timeout_op_.store(what, std::memory_order_relaxed);
}

void TxFailureDiagnostics::noteWkcZero(const char* what)
{
    wkc_zero_.fetch_add(1, std::memory_order_relaxed);
    last_wkc_op_.store(what, std::memory_order_relaxed);
}

void TxFailureDiagnostics::noteStall(int64_t gap_ns)
{
    stalls_.fetch_add(1, std::memory_order_relaxed);
    last_stall_gap_ns_.store(gap_ns, std::memory_order_relaxed);
}

void TxFailureDiagnostics::noteSendOk()
{
    consec_send_fails_.store(0, std::memory_order_relaxed);
}

void TxFailureDiagnostics::noteSuccess()
{
    consec_send_fails_.store(0, std::memory_order_relaxed);
    consec_timeouts_.store(0, std::memory_order_relaxed);
}

void TxFailureDiagnostics::workerMain()
{
    using clock = std::chrono::steady_clock;
    static constexpr auto kRateLimit = std::chrono::milliseconds(250);  // 4 Hz
    const auto never = clock::time_point::min();

    // Pending counts accumulate until the category's rate limit allows
    // an emit — "(N suppressed)" reports everything in between.
    uint32_t sf_pending = 0, ns_pending = 0, to_pending = 0, st_pending = 0;
    uint32_t wk_pending = 0;
    auto last_send_log = never, last_noslot_log = never;
    auto last_timeo_log = never, last_stall_log = never, last_wkc_log = never;

    // Wire probe: emitted once per streak, then on verdict change or
    // once per second while the streak persists.
    std::string last_verdict;
    auto last_probe = never, last_probe_emit = never;
    bool probed_this_streak = false;

    // Ring-probe escalation: fires whenever the consecutive-timeout
    // streak crosses a new multiple of escalate_after_ (comparing
    // quotients survives the poll missing intermediate counts).
    uint32_t last_escalated_ct = 0;

    while (!stop_.load(std::memory_order_acquire)) {
        const auto now = clock::now();
        sf_pending += send_fails_.exchange(0, std::memory_order_relaxed);
        ns_pending += no_slot_.exchange(0, std::memory_order_relaxed);
        to_pending += timeouts_.exchange(0, std::memory_order_relaxed);
        st_pending += stalls_.exchange(0, std::memory_order_relaxed);
        wk_pending += wkc_zero_.exchange(0, std::memory_order_relaxed);
        const int64_t stall_gap_ns =
            last_stall_gap_ns_.load(std::memory_order_relaxed);
        const uint32_t csf =
            consec_send_fails_.load(std::memory_order_relaxed);
        const uint32_t ct =
            consec_timeouts_.load(std::memory_order_relaxed);

        if (sf_pending && now - last_send_log >= kRateLimit) {
            const int e = errno_fn_ ? errno_fn_() : 0;
            const uint32_t mask =
                last_send_fail_mask_.load(std::memory_order_relaxed);
            const std::string msg = mask
                ? std::format("exchangeLRWForSlaves: send failed "
                              "(mask=0x{:08X}, errno={}: {})",
                              mask, e, e ? strerror(e) : "n/a")
                : std::format("exchangeLRW: send failed (errno={}: {})",
                              e, e ? strerror(e) : "n/a");
            if (sf_pending > 1) {
                TETHER_LOGE(log_tag_, "{} ({} suppressed since last log)",
                            msg, sf_pending - 1);
            } else {
                TETHER_LOGE(log_tag_, "{}", msg);
            }
            sf_pending = 0;
            last_send_log = now;
        }
        if (ns_pending && now - last_noslot_log >= kRateLimit) {
            TETHER_LOGE(log_tag_,
                "exchangeLRW: no free response slot — all async waiters "
                "busy{}{}",
                ns_pending > 1 ? " (" : "",
                ns_pending > 1
                    ? std::format("{} suppressed since last log)",
                                  ns_pending - 1)
                    : "");
            ns_pending = 0;
            last_noslot_log = now;
        }
        if (to_pending && now - last_timeo_log >= kRateLimit) {
            const char* what =
                last_timeout_op_.load(std::memory_order_relaxed);
            if (to_pending > 1) {
                TETHER_LOGE(log_tag_,
                    "{}: response timeout ({} suppressed since last log)",
                    what, to_pending - 1);
            } else {
                TETHER_LOGE(log_tag_, "{}: response timeout", what);
            }
            to_pending = 0;
            last_timeo_log = now;
        }
        if (wk_pending && now - last_wkc_log >= kRateLimit) {
            const char* what = last_wkc_op_.load(std::memory_order_relaxed);
            if (wk_pending > 1) {
                TETHER_LOGW(log_tag_, "{}: WKC=0 ({} suppressed since last log)",
                            what, wk_pending - 1);
            } else {
                TETHER_LOGW(log_tag_, "{}: WKC=0", what);
            }
            wk_pending = 0;
            last_wkc_log = now;
        }
        if (st_pending && now - last_stall_log >= kRateLimit) {
            if (st_pending > 1) {
                TETHER_LOGW(log_tag_,
                    "exchangeLRW: host stall ({:.1f} ms since last call) — "
                    "drained wire backlog ({} suppressed since last log)",
                    static_cast<double>(stall_gap_ns) / 1e6, st_pending - 1);
            } else {
                TETHER_LOGW(log_tag_,
                    "exchangeLRW: host stall ({:.1f} ms since last call) — "
                    "drained wire backlog",
                    static_cast<double>(stall_gap_ns) / 1e6);
            }
            st_pending = 0;
            last_stall_log = now;
        }

        // Ring-probe escalation at each multiple of the timeout streak.
        const uint32_t esc = escalate_after_.load(std::memory_order_relaxed);
        if (ct == 0) {
            last_escalated_ct = 0;
        } else if (esc && ct / esc > last_escalated_ct / esc) {
            last_escalated_ct = ct;
            if (escalation_fn_) {
                const std::string verdict = escalation_fn_();
                if (!verdict.empty()) {
                    TETHER_LOGE(log_tag_,
                        "exchangeLRW: {} consecutive timeouts — ring probe {}"
                        ". Check NIC drops / slave link state.", ct, verdict);
                }
            }
        }

        // Wire probe once the send-failure streak hits 10 — first emit
        // per streak, then verdict-change or 1 s heartbeat.
        if (csf >= 10) {
            if (!probed_this_streak || now - last_probe >=
                    std::chrono::seconds(1)) {
                const std::string report = probe_ ? probe_() : "";
                last_probe = now;
                probed_this_streak = true;
                if (!report.empty()) {
                    // Compare on the verdict only — the raw report embeds
                    // live counters that change on every probe.
                    const auto pos = report.find(" | DIAGNOSIS:");
                    const std::string verdict =
                        report.substr(pos != std::string::npos ? pos : 0);
                    if (verdict != last_verdict ||
                        now - last_probe_emit >= std::chrono::seconds(1)) {
                        TETHER_LOGE(log_tag_, "TX diagnostics — {}", report);
                        last_verdict = verdict;
                        last_probe_emit = now;
                    }
                }
            }
        } else if (csf == 0) {
            probed_this_streak = false;
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
}

} // namespace EtherCAT

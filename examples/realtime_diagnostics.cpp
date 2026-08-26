/**
 * @file realtime_diagnostics.cpp
 * @brief Realtime kernel + network diagnostics example
 *
 * Performs the existing realtime-kernel detection, then runs a fake realtime
 * motion loop that sends a configurable PDO-sized LRW datagram to a non-existent
 * slave (index 999) on the given interface.  It sweeps SCHED_FIFO priorities,
 * loop frequencies and CPU-affinity modes, measuring cycle timing, raw sendto
 * latency and CPU usage.  Results are printed color-coded with actionable
 * diagnostics.
 *
 * Usage (Linux, requires root or CAP_NET_RAW + CAP_SYS_NICE):
 *   sudo ./realtime_diagnostics -i eth0
 *   sudo ./realtime_diagnostics -i eth0 -s 64
 */

#include "common/ExampleHelpers.hpp"
#include "common/EtherCATHostSetup.hpp"

#include "tether/ethercat/Types.hpp"
#include "tether/hal/HALTypes.hpp"
#include "tether/hal/IEthernet.hpp"
#include "tether/hal/IThreading.hpp"
#include "tether/platform/EspCompat.hpp"
#include "tether/platform/Platform.hpp"

#include <argparse/argparse.hpp>

#include <algorithm>
#include <arpa/inet.h>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <iomanip>
#include <iostream>
#include <limits>
#include <map>
#include <set>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#ifdef __linux__
#include <linux/if_ether.h>
#include <linux/if_packet.h>
#include <net/if.h>
#include <netinet/in.h>
#include <poll.h>
#include <sched.h>
#include <sys/socket.h>
#include <time.h>
#include <unistd.h>
#endif

namespace {

constexpr const char* TAG = "realtime_diagnostics";
constexpr int kDefaultBasePriority = 80;
constexpr uint16_t kFakeSlaveIndex = 999;

// ============================================================================
// ANSI color helpers (only when stdout is a TTY)
// ============================================================================
struct ColorTags {
    const char* reset = "";
    const char* red = "";
    const char* green = "";
    const char* orange = "";
    const char* yellow = "";
    const char* cyan = "";
    const char* bold = "";
};

ColorTags makeColors() {
    ColorTags c;
    if (isatty(STDOUT_FILENO)) {
        c.reset = "\033[0m";
        c.red = "\033[31m";
        c.green = "\033[32m";
        c.orange = "\033[38;5;208m";
        c.yellow = "\033[33m";
        c.cyan = "\033[36m";
        c.bold = "\033[1m";
    }
    return c;
}

// ============================================================================
// Timing helpers
// ============================================================================
#ifdef __linux__

uint64_t now_ns() {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return static_cast<uint64_t>(ts.tv_sec) * 1'000'000'000ULL +
           static_cast<uint64_t>(ts.tv_nsec);
}

uint64_t thread_cpu_ns() {
    struct timespec ts;
    clock_gettime(CLOCK_THREAD_CPUTIME_ID, &ts);
    return static_cast<uint64_t>(ts.tv_sec) * 1'000'000'000ULL +
           static_cast<uint64_t>(ts.tv_nsec);
}

#endif // __linux__

// ============================================================================
// Statistics
// ============================================================================
struct SampleStats {
    size_t count = 0;
    double mean_us = 0.0;
    double min_us = 0.0;
    double max_us = 0.0;
    double stddev_us = 0.0;
    bool valid = false;
};

SampleStats computeStats(const std::vector<uint64_t>& samples_ns) {
    SampleStats s;
    s.count = samples_ns.size();
    if (s.count == 0) return s;

    double sum = 0.0;
    double min_val = std::numeric_limits<double>::max();
    double max_val = 0.0;
    for (uint64_t v : samples_ns) {
        double us = static_cast<double>(v) / 1000.0;
        sum += us;
        if (us < min_val) min_val = us;
        if (us > max_val) max_val = us;
    }
    s.mean_us = sum / static_cast<double>(s.count);
    s.min_us = min_val;
    s.max_us = max_val;

    double sq_sum = 0.0;
    for (uint64_t v : samples_ns) {
        double us = static_cast<double>(v) / 1000.0;
        double d = us - s.mean_us;
        sq_sum += d * d;
    }
    s.stddev_us = std::sqrt(sq_sum / static_cast<double>(s.count));
    s.valid = true;
    return s;
}

// ============================================================================
// Run configuration and result
// ============================================================================
#ifdef __linux__

enum class RunGrade {
    Good,
    Warn,
    HighJitter,
    ExtremeJitter,
    CycleLoss,
    FrequentCycleLoss,
};

struct RunConfig {
    bool affinity = false;
    int priority = kDefaultBasePriority;
    int frequency_hz = 1000;
    uint32_t period_us = 1000;
    int affinity_core = 1;
    double duration_s = 1.0;
    double warmup_s = 0.1;
    const std::vector<uint8_t>* frame = nullptr;
    const sockaddr_ll* sll = nullptr;
    int sock = -1;
    bool real_slave = false;       // true: APRD + recvfrom; false: LRW, no RX
};

struct RunResult {
    bool affinity = false;
    int priority = kDefaultBasePriority;
    int frequency_hz = 1000;
    uint32_t period_us = 1000;
    bool sched_ok = false;
    bool real_slave = false;
    std::vector<uint64_t> cycle_deltas_ns;
    std::vector<uint64_t> tx_durations_ns;
    std::vector<uint64_t> rx_delays_ns;    // transmit-to-receive delay (real slave only)
    uint64_t cpu_time_ns = 0;
    uint64_t wall_time_ns = 0;
    uint64_t total_cycles = 0;
    uint64_t lost_cycles = 0;
    uint64_t lost_responses = 0;           // cycles with no RX response (real slave only)
};

void runThreadFunc(const RunConfig& cfg, RunResult* out) {
    *out = RunResult{};
    out->affinity = cfg.affinity;
    out->priority = cfg.priority;
    out->frequency_hz = cfg.frequency_hz;
    out->period_us = cfg.period_us;
    out->real_slave = cfg.real_slave;

    out->sched_ok = Tether::Platform::setCurrentThreadRealtime(cfg.priority);

    const uint64_t period_ns = static_cast<uint64_t>(cfg.period_us) * 1000ULL;
    const uint64_t duration_ns =
        static_cast<uint64_t>(cfg.duration_s * 1'000'000'000.0);
    const uint64_t warmup_ns =
        static_cast<uint64_t>(cfg.warmup_s * 1'000'000'000.0);
    const uint64_t num_cycles =
        (duration_ns + period_ns - 1) / period_ns;

    // RX poll timeout: half the period, capped to [100us, 10ms].
    const int rx_poll_timeout_ms = static_cast<int>(
        std::clamp(static_cast<double>(period_ns) / 2.0 / 1'000'000.0,
                   0.1, 10.0));

    const uint64_t cpu_start = thread_cpu_ns();
    const uint64_t wall_start = now_ns();
    const uint64_t base_ns = wall_start;

    out->total_cycles = num_cycles;

    uint64_t next_ns = base_ns + period_ns;
    uint64_t prev_ns = base_ns;
    uint64_t lost_cycles = 0;
    uint64_t lost_responses = 0;

    uint8_t rx_buffer[1514];

    for (uint64_t i = 0; i < num_cycles; ++i) {
        struct timespec next_ts;
        next_ts.tv_sec = static_cast<time_t>(next_ns / 1'000'000'000ULL);
        next_ts.tv_nsec = static_cast<long>(next_ns % 1'000'000'000ULL);
        clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &next_ts, nullptr);
        const uint64_t wake_ns = now_ns();

        const uint64_t delta_ns = wake_ns - prev_ns;
        const bool recording = (wake_ns - base_ns) >= warmup_ns;
        if (recording) {
            out->cycle_deltas_ns.push_back(delta_ns);
        }

        const uint64_t tx_start = now_ns();
        (void)::sendto(cfg.sock, cfg.frame->data(), cfg.frame->size(), 0,
                       reinterpret_cast<const struct sockaddr*>(cfg.sll),
                       sizeof(*cfg.sll));
        const uint64_t tx_end = now_ns();
        if (recording) {
            out->tx_durations_ns.push_back(tx_end - tx_start);
        }

        // In real-slave mode, wait for the response and measure TX->RX delay.
        if (cfg.real_slave) {
            struct pollfd pfd;
            pfd.fd = cfg.sock;
            pfd.events = POLLIN;
            pfd.revents = 0;

            bool got_response = false;
            uint64_t rx_end = 0;

            if (::poll(&pfd, 1, rx_poll_timeout_ms) > 0 &&
                (pfd.revents & POLLIN)) {
                struct sockaddr_ll rx_sll;
                socklen_t rx_sll_len = sizeof(rx_sll);
                const ssize_t rx_len =
                    ::recvfrom(cfg.sock, rx_buffer, sizeof(rx_buffer),
                               MSG_DONTWAIT,
                               reinterpret_cast<struct sockaddr*>(&rx_sll),
                               &rx_sll_len);
                rx_end = now_ns();
                if (rx_len > 0) {
                    got_response = true;
                }
            }

            if (recording) {
                if (got_response) {
                    out->rx_delays_ns.push_back(rx_end - tx_end);
                } else {
                    ++lost_responses;
                }
            }
        }

        next_ns += period_ns;
        if (next_ns < wake_ns) {
            const uint64_t missed =
                (wake_ns - next_ns + period_ns - 1) / period_ns;
            lost_cycles += (missed > 0) ? missed : 1;
            next_ns = wake_ns + period_ns;
        }
        prev_ns = wake_ns;
    }

    const uint64_t wall_end = now_ns();
    const uint64_t cpu_end = thread_cpu_ns();
    out->wall_time_ns = wall_end - wall_start;
    out->cpu_time_ns = cpu_end - cpu_start;
    out->lost_cycles = lost_cycles;
    out->lost_responses = lost_responses;
}

// ============================================================================
// Result formatting
// ============================================================================
const char* gradeString(RunGrade g) {
    switch (g) {
        case RunGrade::Good:              return "GOOD";
        case RunGrade::Warn:              return "WARN";
        case RunGrade::HighJitter:        return "HIGH-JITTER";
        case RunGrade::ExtremeJitter:     return "EXTREME-JITTER";
        case RunGrade::CycleLoss:         return "CYCLE-LOSS";
        case RunGrade::FrequentCycleLoss: return "FREQUENT-CYCLE-LOSS";
    }
    return "?";
}

std::string formatFrequency(int freq_hz) {
    if (freq_hz >= 1000) {
        return std::to_string(freq_hz / 1000) + "kHz";
    }
    return std::to_string(freq_hz) + "Hz";
}

const char* gradeColor(const ColorTags& c, RunGrade g) {
    switch (g) {
        case RunGrade::Good:              return c.green;
        case RunGrade::Warn:
        case RunGrade::HighJitter:        return c.orange;
        case RunGrade::ExtremeJitter:
        case RunGrade::CycleLoss:
        case RunGrade::FrequentCycleLoss: return c.red;
    }
    return c.reset;
}

RunGrade classifyCycleJitter(uint32_t period_us, double max_delta_us) {
    const double p = static_cast<double>(period_us);
    const double deviation = std::abs(max_delta_us - p);
    // Thresholds as fractions of the nominal period P:
    //   Good          < P/10  (10%)  — excellent, expected for RT
    //   Warn          < P/5   (20%)  — noticeable, monitor
    //   HighJitter    < P/3   (33%)  — significant, investigate
    //   ExtremeJitter >= P/3  (33%)  — severe, RT budget compromised
    // P/3 is the high->extreme boundary: at 33% jitter a third of the
    // cycle budget is consumed by timing uncertainty, leaving little
    // headroom before cycle loss begins (~50%).
    if (deviation < p / 10.0) return RunGrade::Good;
    if (deviation < p / 5.0) return RunGrade::Warn;
    if (deviation < p / 3.0) return RunGrade::HighJitter;
    return RunGrade::ExtremeJitter;
}

RunGrade classifyTxLatency(uint32_t period_us, double mean_tx_us) {
    const double p = static_cast<double>(period_us);
    if (mean_tx_us < p / 10.0) return RunGrade::Good;
    if (mean_tx_us < p / 5.0) return RunGrade::Warn;
    if (mean_tx_us < p / 3.0) return RunGrade::HighJitter;
    return RunGrade::ExtremeJitter;
}

RunGrade classifyCycleLoss(uint64_t lost_cycles, uint64_t total_cycles) {
    if (lost_cycles == 0 || total_cycles == 0) return RunGrade::Good;
    const double lost_pct = 100.0 * static_cast<double>(lost_cycles) /
                            static_cast<double>(total_cycles);
    if (lost_pct > 1.0) return RunGrade::FrequentCycleLoss;
    return RunGrade::CycleLoss;
}

// Classify the transmit-to-receive (round-trip) delay.  Uses the same
// P-fraction thresholds as TX latency so the two are directly comparable.
RunGrade classifyRxDelay(uint32_t period_us, double mean_rx_us) {
    const double p = static_cast<double>(period_us);
    if (mean_rx_us < p / 10.0) return RunGrade::Good;
    if (mean_rx_us < p / 5.0) return RunGrade::Warn;
    if (mean_rx_us < p / 3.0) return RunGrade::HighJitter;
    return RunGrade::ExtremeJitter;
}

// Classify the jitter (stddev) of the receive delay independently of the
// mean.  A low mean with high stddev means the system is mostly fast but
// occasionally stalls — a classic realtime warning sign.
RunGrade classifyRxJitter(uint32_t period_us, double stddev_rx_us) {
    const double p = static_cast<double>(period_us);
    if (stddev_rx_us < p / 20.0) return RunGrade::Good;   // 5%
    if (stddev_rx_us < p / 10.0) return RunGrade::Warn;   // 10%
    if (stddev_rx_us < p / 5.0)  return RunGrade::HighJitter; // 20%
    return RunGrade::ExtremeJitter;
}

// Classify lost responses (timeouts waiting for the slave to reply).
RunGrade classifyRxLoss(uint64_t lost_responses, uint64_t total_cycles) {
    if (lost_responses == 0 || total_cycles == 0) return RunGrade::Good;
    const double lost_pct = 100.0 * static_cast<double>(lost_responses) /
                            static_cast<double>(total_cycles);
    if (lost_pct > 1.0) return RunGrade::FrequentCycleLoss;
    return RunGrade::CycleLoss;
}

int gradeRank(RunGrade g) {
    switch (g) {
        case RunGrade::Good:              return 0;
        case RunGrade::Warn:              return 1;
        case RunGrade::HighJitter:        return 2;
        case RunGrade::ExtremeJitter:     return 3;
        case RunGrade::CycleLoss:         return 4;
        case RunGrade::FrequentCycleLoss: return 5;
    }
    return 0;
}

RunGrade worstGrade(RunGrade a, RunGrade b) {
    return gradeRank(a) > gradeRank(b) ? a : b;
}

RunGrade overallGrade(RunGrade cycle_jitter, RunGrade tx_latency,
                      RunGrade cycle_loss, bool sched_ok) {
    if (!sched_ok) return RunGrade::ExtremeJitter;
    return worstGrade(worstGrade(cycle_jitter, tx_latency), cycle_loss);
}

// Extended overall grade that includes RX metrics (real-slave mode).
RunGrade overallGradeRx(RunGrade cycle_jitter, RunGrade tx_latency,
                        RunGrade rx_delay, RunGrade rx_jitter,
                        RunGrade rx_loss, RunGrade cycle_loss,
                        bool sched_ok) {
    if (!sched_ok) return RunGrade::ExtremeJitter;
    RunGrade g = worstGrade(cycle_jitter, tx_latency);
    g = worstGrade(g, rx_delay);
    g = worstGrade(g, rx_jitter);
    g = worstGrade(g, rx_loss);
    g = worstGrade(g, cycle_loss);
    return g;
}

std::string padRight(const std::string& s, size_t width) {
    if (s.size() >= width) return s;
    return s + std::string(width - s.size(), ' ');
}

std::string coloredValue(const ColorTags& c, RunGrade g, double value,
                         int prec, size_t width, const char* unit = "") {
    std::ostringstream oss;
    oss << std::fixed << std::setprecision(prec) << value << unit;
    return gradeColor(c, g) + padRight(oss.str(), width) + c.reset;
}

// ============================================================================
// Recommendations
// ============================================================================
const char* preemptModelName(Tether::Platform::PreemptModel m) {
    using P = Tether::Platform::PreemptModel;
    switch (m) {
        case P::PreemptRt:        return "PREEMPT_RT";
        case P::PreemptDynamic:   return "PREEMPT_DYNAMIC";
        case P::PreemptFull:      return "PREEMPT (full)";
        case P::PreemptVoluntary: return "PREEMPT_VOLUNTARY";
        case P::PreemptNone:      return "PREEMPT_NONE";
        default:                  return "Unknown";
    }
}

const char* realtimeClassName(Tether::Platform::RealtimeClass c) {
    using R = Tether::Platform::RealtimeClass;
    switch (c) {
        case R::HardRealtime: return "HardRealtime";
        case R::LowLatency:   return "LowLatency";
        case R::Voluntary:    return "Voluntary";
        case R::None:         return "None";
        default:              return "Unknown";
    }
}

bool runIsGood(const RunResult& r) {
    if (!r.sched_ok || r.lost_cycles > 0) return false;
    const SampleStats cs = computeStats(r.cycle_deltas_ns);
    const SampleStats ts = computeStats(r.tx_durations_ns);
    const RunGrade cyc = classifyCycleJitter(r.period_us, cs.max_us);
    const RunGrade tx = classifyTxLatency(r.period_us, ts.mean_us);
    const RunGrade loss = classifyCycleLoss(r.lost_cycles, r.total_cycles);
    if (r.real_slave) {
        const SampleStats rs = computeStats(r.rx_delays_ns);
        const RunGrade rxd = classifyRxDelay(r.period_us, rs.mean_us);
        const RunGrade rxj = classifyRxJitter(r.period_us, rs.stddev_us);
        const RunGrade rxl = classifyRxLoss(r.lost_responses, r.total_cycles);
        return overallGradeRx(cyc, tx, rxd, rxj, rxl, loss, r.sched_ok) ==
               RunGrade::Good;
    }
    return overallGrade(cyc, tx, loss, r.sched_ok) == RunGrade::Good;
}

double averageJitterPercent(const RunResult& r) {
    const SampleStats cs = computeStats(r.cycle_deltas_ns);
    if (!cs.valid) return 0.0;
    const double p = static_cast<double>(r.period_us);
    return cs.stddev_us * 100.0 / p; // stddev as % of period
}

void printRecommendations(const ColorTags& c,
                          const std::vector<RunResult>& results,
                          const Tether::Platform::RealtimeKernelInfo& kernel_info,
                          const std::vector<int>& priorities,
                          int affinity_core) {
    std::cout << "\n" << c.bold << c.cyan << "=== Diagnostics & Recommendations ==="
              << c.reset << "\n";

    // 1. System/kernel summary.
    std::cout << "  Kernel:              " << kernel_info.sysname << " "
              << kernel_info.kernel_release << "\n";
    std::cout << "  Build preempt model: "
              << preemptModelName(kernel_info.build_model) << "\n";
    std::cout << "  Active preempt mode: " << kernel_info.active_preempt_mode
              << "\n";
    std::cout << "  Realtime class:      "
              << realtimeClassName(kernel_info.realtime_class) << "\n";

    const bool kernel_is_rt =
        kernel_info.realtime_class ==
        Tether::Platform::RealtimeClass::HardRealtime;

    if (!kernel_is_rt) {
        std::cout << c.red
                  << "  * Please install a PREEMPT_RT realtime kernel."
                  << c.reset << "\n";
    }

    // 2. Network card latency assessment (best observed mean sendto time).
    double best_tx_mean = std::numeric_limits<double>::max();
    double worst_tx_mean = 0.0;
    for (const auto& r : results) {
        const SampleStats ts = computeStats(r.tx_durations_ns);
        if (!ts.valid) continue;
        if (ts.mean_us < best_tx_mean) best_tx_mean = ts.mean_us;
        if (ts.mean_us > worst_tx_mean) worst_tx_mean = ts.mean_us;
    }
    if (best_tx_mean != std::numeric_limits<double>::max()) {
        std::cout << "  Raw sendto latency:  best mean " << std::fixed
                  << std::setprecision(1) << best_tx_mean << " µs, worst mean "
                  << worst_tx_mean << " µs\n";
        if (best_tx_mean < 10.0) {
            std::cout << c.green
                      << "  * Network interface latency looks excellent."
                      << c.reset << "\n";
        } else if (best_tx_mean < 50.0) {
            std::cout << c.green
                      << "  * Network interface latency is good."
                      << c.reset << "\n";
        } else if (best_tx_mean < 200.0) {
            std::cout << c.yellow
                      << "  * Network interface latency is moderate; consider a "
                         "dedicated NIC or driver tuning."
                      << c.reset << "\n";
        } else {
            std::cout << c.red
                      << "  * Network interface latency is poor; this will limit "
                         "high-frequency loops."
                      << c.reset << "\n";
        }
    }

    // 2b. Round-trip (TX->RX) performance assessment — real-slave mode only.
    bool has_real_slave = false;
    for (const auto& r : results) {
        if (r.real_slave) { has_real_slave = true; break; }
    }
    if (has_real_slave) {
        double best_rx_mean = std::numeric_limits<double>::max();
        double worst_rx_mean = 0.0;
        double best_rx_stddev = std::numeric_limits<double>::max();
        uint64_t total_lost_resp = 0;
        uint64_t total_cycles_all = 0;
        for (const auto& r : results) {
            const SampleStats rs = computeStats(r.rx_delays_ns);
            if (rs.valid) {
                if (rs.mean_us < best_rx_mean) best_rx_mean = rs.mean_us;
                if (rs.mean_us > worst_rx_mean) worst_rx_mean = rs.mean_us;
                if (rs.stddev_us < best_rx_stddev) best_rx_stddev = rs.stddev_us;
            }
            total_lost_resp += r.lost_responses;
            total_cycles_all += r.total_cycles;
        }

        std::cout << "  Round-trip (TX->RX) delay:";
        if (best_rx_mean != std::numeric_limits<double>::max()) {
            std::cout << "  best mean " << std::fixed
                      << std::setprecision(1) << best_rx_mean
                      << " µs, worst mean " << worst_rx_mean
                      << " µs, best stddev " << best_rx_stddev << " µs\n";
        } else {
            std::cout << "  no responses received\n";
        }

        if (total_cycles_all > 0) {
            const double resp_loss_pct =
                100.0 * static_cast<double>(total_lost_resp) /
                static_cast<double>(total_cycles_all);
            std::cout << "  Response loss:       " << std::fixed
                      << std::setprecision(5) << resp_loss_pct << "% ("
                      << total_lost_resp << "/" << total_cycles_all
                      << " cycles without response)\n";
        }

        if (best_rx_mean != std::numeric_limits<double>::max()) {
            if (best_rx_mean < 50.0 && best_rx_stddev < 10.0) {
                std::cout << c.green
                          << "  * Round-trip performance is excellent — the slave "
                             "and network path are well-tuned."
                          << c.reset << "\n";
            } else if (best_rx_mean < 200.0) {
                std::cout << c.green
                          << "  * Round-trip performance is good for typical "
                             "EtherCAT cycles."
                          << c.reset << "\n";
            } else if (best_rx_mean < 1000.0) {
                std::cout << c.yellow
                          << "  * Round-trip delay is moderate; high-frequency "
                             "loops may be limited by slave processing time "
                             "or cable length."
                          << c.reset << "\n";
            } else {
                std::cout << c.red
                          << "  * Round-trip delay is high; check cable length, "
                             "slave firmware, and NIC interrupt coalescing."
                          << c.reset << "\n";
            }
        }

        if (total_lost_resp > 0 && total_cycles_all > 0) {
            const double resp_loss_pct =
                100.0 * static_cast<double>(total_lost_resp) /
                static_cast<double>(total_cycles_all);
            if (resp_loss_pct > 1.0) {
                std::cout << c.red
                          << "  * Frequent response loss (>1%) — the slave may not "
                             "be responding in time, or frames are being dropped. "
                             "Check link quality and slave state."
                          << c.reset << "\n";
            } else {
                std::cout << c.orange
                          << "  * Occasional response loss (<1%) — may be caused by "
                             "rare scheduling preemption during the receive window."
                          << c.reset << "\n";
            }
        }

        // RX jitter vs TX latency comparison.
        if (best_rx_mean != std::numeric_limits<double>::max() &&
            best_tx_mean != std::numeric_limits<double>::max()) {
            const double rx_overhead = best_rx_mean - best_tx_mean;
            std::cout << "  RX overhead over TX: ~" << std::fixed
                      << std::setprecision(1) << rx_overhead
                      << " µs (slave processing + cable round-trip)\n";
            if (rx_overhead > 5.0 * best_tx_mean && best_tx_mean > 0.0) {
                std::cout << c.orange
                          << "  * Receive delay is dominated by slave/cable, not "
                             "the host TX path."
                          << c.reset << "\n";
            }
        }
    }

    // 3. Priority sensitivity.
    std::map<int, double> jitter_by_priority;
    std::map<int, int> count_by_priority;
    for (const auto& r : results) {
        const double j = averageJitterPercent(r);
        jitter_by_priority[r.priority] += j;
        ++count_by_priority[r.priority];
    }
    for (auto& [prio, sum] : jitter_by_priority) {
        const int n = count_by_priority[prio];
        if (n > 0) sum /= static_cast<double>(n);
    }

    std::cout << "  Avg cycle jitter by priority (stddev % of period):\n";
    for (const auto& [prio, j] : jitter_by_priority) {
        std::cout << "    prio " << prio << ": "
                  << std::fixed << std::setprecision(1) << j << "%\n";
    }

    int min_prio = priorities.empty() ? 0 : priorities.front();
    int max_prio = priorities.empty() ? 0 : priorities.front();
    for (int p : priorities) {
        if (p < min_prio) min_prio = p;
        if (p > max_prio) max_prio = p;
    }
    const double j_min = jitter_by_priority.count(min_prio)
                             ? jitter_by_priority[min_prio]
                             : 0.0;
    const double j_max = jitter_by_priority.count(max_prio)
                             ? jitter_by_priority[max_prio]
                             : 0.0;

    if (jitter_by_priority.size() > 1 && j_min > j_max * 1.5 && j_max > 0.0) {
        std::cout << c.orange
                  << "  * Higher realtime priority noticeably improves timing; "
                     "check for other realtime processes on the same CPU."
                  << c.reset << "\n";
    } else if (jitter_by_priority.size() > 1 &&
               std::abs(j_min - j_max) <
                   0.5 * std::max(j_min, j_max)) {
        std::cout << c.green
                  << "  * Timing is largely insensitive to priority changes, "
                     "suggesting the system is not heavily contended."
                  << c.reset << "\n";
    }

    // 4. Affinity sensitivity.
    double avg_jitter_affinity = 0.0, avg_jitter_no_affinity = 0.0;
    int n_aff = 0, n_no_aff = 0;
    for (const auto& r : results) {
        const double j = averageJitterPercent(r);
        if (r.affinity) {
            avg_jitter_affinity += j; ++n_aff;
        } else {
            avg_jitter_no_affinity += j; ++n_no_aff;
        }
    }
    if (n_aff > 0) avg_jitter_affinity /= static_cast<double>(n_aff);
    if (n_no_aff > 0)
        avg_jitter_no_affinity /= static_cast<double>(n_no_aff);

    std::cout << "  Avg cycle jitter with affinity:    "
              << std::fixed << std::setprecision(1) << avg_jitter_affinity
              << "%\n";
    std::cout << "  Avg cycle jitter without affinity: "
              << avg_jitter_no_affinity << "%\n";

    enum class AffinityVerdict { Positive, Inconclusive, Negative };
    AffinityVerdict verdict = AffinityVerdict::Inconclusive;
    if (avg_jitter_no_affinity > avg_jitter_affinity * 1.5 &&
        avg_jitter_affinity > 0.0) {
        verdict = AffinityVerdict::Positive;
    } else if (avg_jitter_affinity > avg_jitter_no_affinity * 1.5 &&
               avg_jitter_no_affinity > 0.0) {
        verdict = AffinityVerdict::Negative;
    } else {
        verdict = AffinityVerdict::Inconclusive;
    }

    std::cout << "  Core affinity effect: ";
    if (verdict == AffinityVerdict::Positive) {
        std::cout << c.green << "POSITIVE" << c.reset
                  << " (affinity reduces jitter).\n";
    } else if (verdict == AffinityVerdict::Negative) {
        std::cout << c.red << "NEGATIVE" << c.reset
                  << " (affinity increases jitter).\n";
    } else {
        std::cout << c.orange << "INCONCLUSIVE" << c.reset
                  << " (no clear difference).\n";
    }

    if (verdict == AffinityVerdict::Negative ||
        verdict == AffinityVerdict::Inconclusive) {
        std::cout << c.orange
                  << "  * If you depend on core affinity, try to configure Linux "
                     "to avoid core "
                  << affinity_core
                  << " for normal scheduling:\n"
                  << "    - Add 'isolcpus=" << affinity_core
                  << "' to the kernel command line.\n"
                  << "      Example in /etc/default/grub:\n"
                  << "        GRUB_CMDLINE_LINUX_DEFAULT=\"quiet isolcpus="
                  << affinity_core << "\"\n"
                  << "      Then run: sudo update-grub && sudo reboot\n"
                  << "    - Alternatively, use 'cset shield --cpu=" << affinity_core
                  << " --kthread=on' from the cpuset package\n"
                  << "      to reserve the core without rebooting."
                  << c.reset << "\n";
    } else {
        std::cout << c.green
                  << "  * CPU affinity is helping; consider making it permanent "
                     "with isolcpus or cset."
                  << c.reset << "\n";
    }

    // 5. Frequency capability.
    std::set<int> unique_freqs;
    for (const auto& r : results) unique_freqs.insert(r.frequency_hz);

    int pass_1k = 0, fail_high_freq = 0;
    int max_freq = unique_freqs.empty() ? 0 : *unique_freqs.rbegin();
    for (int f : unique_freqs) {
        int pass = 0, total = 0;
        for (const auto& r : results) {
            if (r.frequency_hz != f) continue;
            ++total;
            if (runIsGood(r)) ++pass;
        }
        if (f == 1000) pass_1k = pass;
        if (f >= 4000 && pass < total / 2) {
            fail_high_freq = 1;
        }
    }
    if (pass_1k > 0 && fail_high_freq) {
        std::cout << c.orange
                  << "  * 1 kHz is expected to work well, but "
                  << formatFrequency(max_freq)
                  << " may require kernel tuning or a faster NIC."
                  << c.reset << "\n";
    }

    // 6. Overall fallback recommendation.
    int good_total = 0;
    for (const auto& r : results) {
        if (runIsGood(r)) ++good_total;
    }
    if (good_total == 0 && kernel_is_rt) {
        std::cout << c.red
                  << "  * Check for interrupt-heavy drivers, CPU power-saving "
                     "states, or run on a dedicated realtime core."
                  << c.reset << "\n";
    }
}

// ============================================================================
// Throughput test
// ============================================================================
// Sends 1000 frames at a given inter-frame delay, measuring how many
// responses are received (real-slave mode) or simply how many frames
// are sent successfully (fake-slave mode).  The delay is halved from
// 2000 µs down to 10 µs until either 10 µs is reached or the RX loss
// exceeds the configured threshold.  When the threshold is crossed,
// a piecewise-linear iterative refinement finds the exact delay at
// which the threshold is reached.

struct ThroughputStepResult {
    uint64_t delay_us = 0;
    uint64_t frames_sent = 0;
    uint64_t responses_received = 0;
    double loss_pct = 0.0;       // 0 in fake-slave mode (no RX)
    double mean_rtt_us = 0.0;    // 0 in fake-slave mode
    double max_rtt_us = 0.0;     // 0 in fake-slave mode
};

ThroughputStepResult runThroughputStep(int sock,
                                       const std::vector<uint8_t>& frame,
                                       const sockaddr_ll* sll,
                                       uint64_t delay_us,
                                       bool real_slave,
                                       uint64_t frame_count = 1000) {
    ThroughputStepResult r;
    r.delay_us = delay_us;
    r.frames_sent = frame_count;

    const uint64_t delay_ns = delay_us * 1000ULL;
    uint8_t rx_buffer[1514];

    // Use absolute-time scheduling so jitter in sendto doesn't accumulate.
    const uint64_t start_ns = now_ns();
    uint64_t next_ns = start_ns + delay_ns;

    std::vector<uint64_t> rtt_ns;

    for (uint64_t i = 0; i < frame_count; ++i) {
        // Wait until next send time.
        if (delay_ns > 0) {
            struct timespec next_ts;
            next_ts.tv_sec = static_cast<time_t>(next_ns / 1'000'000'000ULL);
            next_ts.tv_nsec = static_cast<long>(next_ns % 1'000'000'000ULL);
            clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &next_ts,
                            nullptr);
        }

        const uint64_t tx_end = now_ns();

        (void)::sendto(sock, frame.data(), frame.size(), 0,
                       reinterpret_cast<const struct sockaddr*>(sll),
                       sizeof(*sll));

        if (real_slave) {
            // Poll for response with timeout = delay (so we don't miss
            // the next send slot, but still give the slave time to reply).
            struct pollfd pfd;
            pfd.fd = sock;
            pfd.events = POLLIN;
            pfd.revents = 0;

            const int poll_timeout_ms = static_cast<int>(
                std::clamp(static_cast<double>(delay_us) / 1000.0,
                           0.0, 50.0));

            if (::poll(&pfd, 1, poll_timeout_ms) > 0 &&
                (pfd.revents & POLLIN)) {
                struct sockaddr_ll rx_sll;
                socklen_t rx_sll_len = sizeof(rx_sll);
                const ssize_t rx_len =
                    ::recvfrom(sock, rx_buffer, sizeof(rx_buffer),
                               MSG_DONTWAIT,
                               reinterpret_cast<struct sockaddr*>(&rx_sll),
                               &rx_sll_len);
                if (rx_len > 0) {
                    const uint64_t rx_end = now_ns();
                    ++r.responses_received;
                    rtt_ns.push_back(rx_end - tx_end);
                }
            }
        }

        next_ns += delay_ns;
    }

    if (real_slave && frame_count > 0) {
        const uint64_t lost = frame_count - r.responses_received;
        r.loss_pct = 100.0 * static_cast<double>(lost) /
                     static_cast<double>(frame_count);
    }

    if (!rtt_ns.empty()) {
        const SampleStats rs = computeStats(rtt_ns);
        r.mean_rtt_us = rs.mean_us;
        r.max_rtt_us = rs.max_us;
    }

    return r;
}

void printThroughputResults(const ColorTags& c,
                            const std::vector<ThroughputStepResult>& steps,
                            bool real_slave,
                            double threshold_pct) {
    std::cout << "\n" << c.bold << c.cyan
              << "=== Throughput Test Results ===" << c.reset << "\n\n";

    std::cout << "  Method: 1000 frames per delay step, delay halved from "
                 "2000 µs to 10 µs.\n";
    if (real_slave) {
        std::cout << "  Mode: real slave — RX loss measured against threshold "
                  << std::fixed << std::setprecision(5) << threshold_pct
                  << "%\n\n";
    } else {
        std::cout << "  Mode: fake slave — no RX expected, measuring TX "
                     "throughput only.\n\n";
    }

    std::cout << "  Column explanation:\n"
              << "    Delay[µs]   Inter-frame delay.\n"
              << "    Frames      Number of frames sent.\n";
    if (real_slave) {
        std::cout << "    RX          Responses received.\n"
                  << "    Loss[%]     Percentage of frames with no response.\n"
                  << "    RTTmean[µs] Mean round-trip time.\n"
                  << "    RTTmax[µs]  Maximum round-trip time.\n";
    }
    std::cout << "    Status      Step status.\n\n";

    std::cout << c.bold << c.cyan << std::left
              << std::setw(14) << "Delay[µs]"
              << std::setw(10) << "Frames";
    if (real_slave) {
        std::cout << std::setw(10) << "RX"
                  << std::setw(14) << "Loss[%]"
                  << std::setw(14) << "RTTmean[µs]"
                  << std::setw(14) << "RTTmax[µs]";
    }
    std::cout << std::setw(20) << "Status"
              << c.reset << "\n";

    for (const auto& s : steps) {
        std::ostringstream delay_oss;
        delay_oss << s.delay_us;

        std::ostringstream loss_oss;
        loss_oss << std::fixed << std::setprecision(5) << s.loss_pct << "%";

        const char* status = "OK";
        const char* status_color = c.green;
        if (real_slave) {
            if (s.loss_pct > threshold_pct) {
                status = "THRESHOLD-EXCEEDED";
                status_color = c.red;
            } else if (s.loss_pct > 0.0) {
                status = "LOSS";
                status_color = c.orange;
            }
        }

        std::ostringstream rtt_mean_oss, rtt_max_oss;
        rtt_mean_oss << std::fixed << std::setprecision(1)
                     << s.mean_rtt_us << "µs";
        rtt_max_oss << std::fixed << std::setprecision(1)
                    << s.max_rtt_us << "µs";

        std::cout << std::left
                  << padRight(delay_oss.str() + "µs", 14)
                  << padRight(std::to_string(s.frames_sent), 10);
        if (real_slave) {
            std::cout << padRight(std::to_string(s.responses_received), 10)
                      << padRight(loss_oss.str(), 14)
                      << padRight(rtt_mean_oss.str(), 14)
                      << padRight(rtt_max_oss.str(), 14);
        }
        std::cout << status_color << padRight(status, 20) << c.reset << "\n";
    }
}

// Piecewise-linear iterative refinement: given two delays (lo, hi) where
// lo has loss < threshold and hi has loss > threshold, find the delay
// at which loss crosses the threshold by linearly interpolating and
// testing the midpoint, similar to binary search.
std::vector<ThroughputStepResult> refineThreshold(
    int sock, const std::vector<uint8_t>& frame,
    const sockaddr_ll* sll, bool real_slave,
    double threshold_pct,
    uint64_t lo_delay_us, double lo_loss_pct,
    uint64_t hi_delay_us, double hi_loss_pct,
    int max_iterations = 10) {
    std::vector<ThroughputStepResult> refined;

    // lo = good (loss < threshold), hi = bad (loss > threshold).
    // In our sweep, lo_delay > hi_delay (more delay → less loss).
    // The interpolation works regardless of ordering, but the bounds
    // check must use min/max to handle both directions.
    const uint64_t bound_lo = std::min(lo_delay_us, hi_delay_us);
    const uint64_t bound_hi = std::max(lo_delay_us, hi_delay_us);

    for (int iter = 0; iter < max_iterations; ++iter) {
        const double d_lo = static_cast<double>(lo_delay_us);
        const double d_hi = static_cast<double>(hi_delay_us);
        const double l_lo = lo_loss_pct;
        const double l_hi = hi_loss_pct;

        if (std::abs(l_hi - l_lo) < 1e-9) break;

        // Linear estimate:
        //   delay = d_lo + (threshold - l_lo) * (d_hi - d_lo) / (l_hi - l_lo)
        double est_delay = d_lo +
            (threshold_pct - l_lo) * (d_hi - d_lo) / (l_hi - l_lo);
        uint64_t mid_delay = static_cast<uint64_t>(std::round(est_delay));
        if (mid_delay < 10) mid_delay = 10;
        if (mid_delay == lo_delay_us || mid_delay == hi_delay_us) break;
        // mid must be strictly inside the [bound_lo, bound_hi] interval.
        if (mid_delay <= bound_lo || mid_delay >= bound_hi) break;

        ThroughputStepResult mid = runThroughputStep(
            sock, frame, sll, mid_delay, real_slave);

        std::cout << "    refinement iter " << iter + 1
                  << ": delay=" << mid_delay << "µs, loss="
                  << std::fixed << std::setprecision(5) << mid.loss_pct
                  << "%\n";

        refined.push_back(mid);

        if (mid.loss_pct < threshold_pct) {
            lo_delay_us = mid_delay;
            lo_loss_pct = mid.loss_pct;
        } else {
            hi_delay_us = mid_delay;
            hi_loss_pct = mid.loss_pct;
        }

        // Convergence: the interval is small enough.
        const uint64_t interval =
            (lo_delay_us > hi_delay_us) ? (lo_delay_us - hi_delay_us)
                                        : (hi_delay_us - lo_delay_us);
        if (interval <= 1) break;
    }

    return refined;
}

void runThroughputTest(const ColorTags& c,
                       int sock,
                       const std::vector<uint8_t>& frame,
                       const sockaddr_ll* sll,
                       bool real_slave,
                       double threshold_pct) {
    // threshold_pct is given as a fraction (0.01 = 1%).  Convert to the
    // same unit as loss_pct (percent: 1.0 = 1%) for all comparisons.
    const double threshold_percent = threshold_pct * 100.0;

    std::cout << "\n" << c.bold << c.cyan
              << "=== Throughput Test ===" << c.reset << "\n";
    std::cout << "  Sweeping inter-frame delay from 2000 µs down to 10 µs.\n";
    std::cout << "  1000 frames per step.  ";
    if (real_slave) {
        std::cout << "Stopping when RX loss > "
                  << std::fixed << std::setprecision(5) << threshold_percent
                  << "%.\n";
    } else {
        std::cout << "No RX expected (fake slave mode).\n";
    }
    std::cout << "\n";

    std::vector<ThroughputStepResult> steps;

    // Coarse sweep: start at 2000 µs, halve until 10 µs or threshold exceeded.
    constexpr uint64_t kStartDelayUs = 2000;
    constexpr uint64_t kMinDelayUs = 10;
    constexpr uint64_t kFramesPerStep = 1000;

    uint64_t delay = kStartDelayUs;
    bool threshold_crossed = false;
    uint64_t last_good_delay = 0;
    double last_good_loss = 0.0;
    uint64_t first_bad_delay = 0;
    double first_bad_loss = 0.0;

    while (true) {
        ThroughputStepResult r = runThroughputStep(
            sock, frame, sll, delay, real_slave, kFramesPerStep);
        steps.push_back(r);

        std::cout << "  delay=" << std::setw(6) << delay << "µs"
                  << "  frames=" << r.frames_sent;
        if (real_slave) {
            std::cout << "  RX=" << std::setw(5) << r.responses_received
                      << "  loss=" << std::fixed << std::setprecision(5)
                      << r.loss_pct << "%";
        }
        std::cout << "\n";

        if (real_slave && r.loss_pct > threshold_percent) {
            threshold_crossed = true;
            first_bad_delay = delay;
            first_bad_loss = r.loss_pct;
            break;
        }

        last_good_delay = delay;
        last_good_loss = r.loss_pct;

        if (delay <= kMinDelayUs) break;
        delay /= 2;
        if (delay < kMinDelayUs) delay = kMinDelayUs;
    }

    // Refinement: find exact threshold crossing point.
    std::vector<ThroughputStepResult> refined;
    if (threshold_crossed && last_good_delay > 0 && first_bad_delay > 0 &&
        last_good_delay != first_bad_delay) {
        std::cout << "\n  Threshold crossed between "
                  << last_good_delay << "µs (loss="
                  << std::fixed << std::setprecision(5) << last_good_loss
                  << "%) and " << first_bad_delay << "µs (loss="
                  << first_bad_loss << "%).\n"
                  << "  Refining with piecewise-linear interpolation:\n";

        refined = refineThreshold(sock, frame, sll, real_slave,
                                  threshold_percent,
                                  last_good_delay, last_good_loss,
                                  first_bad_delay, first_bad_loss);
    }

    // Append refined steps to the results.
    for (const auto& r : refined) steps.push_back(r);

    // Print results table.
    printThroughputResults(c, steps, real_slave, threshold_percent);

    // Summary.
    std::cout << "\n" << c.bold << c.cyan
              << "=== Throughput Summary ===" << c.reset << "\n";
    if (real_slave) {
        if (threshold_crossed) {
            // Find the minimum delay (fastest frame rate) with loss <= threshold.
            uint64_t best_delay = UINT64_MAX;
            double best_loss = 0.0;
            for (const auto& s : steps) {
                if (s.loss_pct <= threshold_percent && s.delay_us < best_delay) {
                    best_delay = s.delay_us;
                    best_loss = s.loss_pct;
                }
            }
            // Find the maximum delay (slowest frame rate) with loss > threshold.
            uint64_t worst_delay = 0;
            double worst_loss = 0.0;
            for (const auto& s : steps) {
                if (s.loss_pct > threshold_percent && s.delay_us > worst_delay) {
                    worst_delay = s.delay_us;
                    worst_loss = s.loss_pct;
                }
            }

            if (best_delay != UINT64_MAX) {
                std::cout << "  Minimum reliable inter-frame delay: "
                          << best_delay << "µs (loss="
                          << std::fixed << std::setprecision(5) << best_loss
                          << "%)\n";
            }
            if (worst_delay > 0) {
                std::cout << "  First failing inter-frame delay:    "
                          << worst_delay << "µs (loss="
                          << std::fixed << std::setprecision(5) << worst_loss
                          << "%)\n";
            }
            if (best_delay != UINT64_MAX && worst_delay > 0) {
                const double max_fps =
                    1'000'000.0 / static_cast<double>(best_delay);
                const double fail_fps =
                    1'000'000.0 / static_cast<double>(worst_delay);
                std::cout << "  Maximum reliable frame rate:       "
                          << std::fixed << std::setprecision(0) << max_fps
                          << " fps (at " << best_delay << "µs delay)\n";
                std::cout << "  Frame rate at first failure:       "
                          << std::fixed << std::setprecision(0) << fail_fps
                          << " fps (at " << worst_delay << "µs delay)\n";
            }
            std::cout << c.orange
                      << "  * Throughput is limited by slave response time "
                         "and/or network turnaround."
                      << c.reset << "\n";
        } else {
            // All steps passed — find the minimum delay tested.
            uint64_t min_delay = UINT64_MAX;
            for (const auto& s : steps) {
                if (s.delay_us < min_delay) min_delay = s.delay_us;
            }
            if (min_delay != UINT64_MAX) {
                const double max_fps =
                    1'000'000.0 / static_cast<double>(min_delay);
                std::cout << c.green
                          << "  All steps passed — no RX loss exceeded "
                             "threshold down to "
                          << min_delay << "µs delay ("
                          << std::fixed << std::setprecision(0) << max_fps
                          << " fps).\n"
                          << c.reset;
            }
        }
    } else {
        // Fake-slave mode: report TX throughput at minimum delay.
        uint64_t min_delay = UINT64_MAX;
        for (const auto& s : steps) {
            if (s.delay_us < min_delay) min_delay = s.delay_us;
        }
        if (min_delay != UINT64_MAX) {
            const double max_fps =
                1'000'000.0 / static_cast<double>(min_delay);
            std::cout << "  Minimum tested inter-frame delay: "
                      << min_delay << "µs ("
                      << std::fixed << std::setprecision(0) << max_fps
                      << " fps TX rate)\n";
            std::cout << c.green
                      << "  * TX throughput test completed.  Use --slave N to "
                         "measure RX throughput limits."
                      << c.reset << "\n";
        }
    }
}

#endif // __linux__

} // namespace

// ============================================================================
// main
// ============================================================================
int main(int argc, char** argv) {
#ifdef __linux__
    argparse::ArgumentParser program("realtime_diagnostics");
    Tether::Examples::addInterfaceArg(program);
    program.add_argument("-s", "--pdo-size")
        .scan<'i', int>()
        .default_value(128)
        .help("PDO payload size in bytes (1-1486, default 128)");
    program.add_argument("--affinity-core")
        .scan<'i', int>()
        .default_value(1)
        .help("CPU core to pin to when affinity is enabled (default 1)");
    program.add_argument("-d", "--duration")
        .scan<'g', double>()
        .default_value(1.0)
        .help("Measurement duration per parameter set in seconds (default 1.0)");
    program.add_argument("-w", "--warmup")
        .scan<'g', double>()
        .default_value(0.1)
        .help("Warmup duration to discard at the start of each run (default 0.1)");
    program.add_argument("--prio-standard")
        .scan<'i', int>()
        .default_value(80)
        .help("Standard SCHED_FIFO priority (default 80)");
    program.add_argument("--prio-low")
        .scan<'i', int>()
        .default_value(70)
        .help("Low SCHED_FIFO priority relative to standard (default 70)");
    program.add_argument("--prio-high")
        .scan<'i', int>()
        .default_value(90)
        .help("High SCHED_FIFO priority relative to standard (default 90)");
    program.add_argument("--freqs")
        .default_value(std::string("500,1000,2000,4000,10000"))
        .help("Comma-separated loop frequencies in Hz (default 500,1000,2000,4000,10000)");
    program.add_argument("--slave")
        .scan<'i', int>()
        .default_value(-1)
        .help("Real slave index for round-trip communication (default -1 = fake slave, "
              "no reception). When >= 0, sends APRD datagrams to the slave at the "
              "given position and measures transmit-to-receive delay.");
    program.add_argument("--throughput")
        .default_value(false)
        .implicit_value(true)
        .help("Run an additional throughput test after the main diagnostics. "
              "Sweeps inter-frame delay from 2000 µs down to 10 µs (1000 frames "
              "per step) and, in real-slave mode, finds the delay at which RX "
              "loss exceeds --throughput-threshold.");
    program.add_argument("--throughput-threshold")
        .scan<'g', double>()
        .default_value(0.01)
        .help("RX loss percentage threshold (1% = 0.01) at which the throughput "
              "test stops the coarse sweep and refines the exact crossing point "
              "(default 0.01 = 1%).");

    try {
        program.parse_args(argc, argv);
    } catch (const std::runtime_error& err) {
        std::cerr << err.what() << "\n" << program;
        return 1;
    }

    const std::string iface = Tether::Examples::resolveInterface(program.get<std::string>("--interface"), TAG);
    int pdo_size = program.get<int>("--pdo-size");
    if (pdo_size < 1) pdo_size = 1;
    if (pdo_size > 1486) pdo_size = 1486;
    const int affinity_core = program.get<int>("--affinity-core");
    const double duration_s = program.get<double>("--duration");
    const double warmup_s = program.get<double>("--warmup");

    if (warmup_s >= duration_s) {
        std::cerr << "Warmup must be shorter than duration\n";
        return 1;
    }

    const std::vector<int> priorities = {
        program.get<int>("--prio-low"),
        program.get<int>("--prio-standard"),
        program.get<int>("--prio-high"),
    };

    std::vector<int> frequencies;
    {
        std::string freqs_str = program.get<std::string>("--freqs");
        std::stringstream ss(freqs_str);
        std::string token;
        while (std::getline(ss, token, ',')) {
            try {
                size_t pos = 0;
                const int f = std::stoi(token, &pos);
                if (pos == token.size() && f > 0) {
                    frequencies.push_back(f);
                } else {
                    std::cerr << "Invalid frequency token: " << token << "\n";
                    return 1;
                }
            } catch (...) {
                std::cerr << "Invalid frequency token: " << token << "\n";
                return 1;
            }
        }
    }
    if (frequencies.empty()) {
        std::cerr << "At least one frequency must be specified\n";
        return 1;
    }

    const int slave_index = program.get<int>("--slave");
    const bool real_slave_mode = (slave_index >= 0);
    if (real_slave_mode && slave_index > 65534) {
        std::cerr << "Slave index must be 0-65534\n";
        return 1;
    }

    const bool run_throughput = program.get<bool>("--throughput");
    const double throughput_threshold = program.get<double>("--throughput-threshold");
    if (throughput_threshold <= 0.0 || throughput_threshold >= 100.0) {
        std::cerr << "Throughput threshold must be between 0 and 100 (exclusive)\n";
        return 1;
    }

    const ColorTags color = makeColors();

    TETHER_LOGI(TAG, "realtime_diagnostics — interface: %s, pdo-size: %d",
                iface.c_str(), pdo_size);

    const auto kernel_info = Tether::Platform::ensureRealtimeKernelOrExit(
        Tether::Platform::RealtimeRequirement::None);
    const bool kernel_is_rt =
        (kernel_info.realtime_class ==
         Tether::Platform::RealtimeClass::HardRealtime);

    Tether::Examples::HostEtherNetSession session;
    if (!Tether::Examples::initHostEthernet(session, iface, TAG)) {
        return 2;
    }

    // Native raw socket fd for direct sendto timing.
    const int sock = static_cast<int>(reinterpret_cast<intptr_t>(
        session.eth->nativeHandle()));
    if (sock < 0) {
        TETHER_LOGE(TAG, "Invalid native socket handle");
        Tether::Examples::shutdownHostEthernet(session);
        return 2;
    }

    // Interface index for sendto.
    const unsigned int ifindex = if_nametoindex(iface.c_str());
    if (ifindex == 0) {
        TETHER_LOGE(TAG, "Failed to resolve interface index for %s",
                    iface.c_str());
        Tether::Examples::shutdownHostEthernet(session);
        return 2;
    }

    // Build the EtherCAT frame.
    //
    // Fake-slave mode (default): an LRW datagram with a configurable PDO
    // payload is sent to logical address 0.  No slave will ever respond, so
    // no receive path is exercised — the measurement is purely TX-side
    // (sendto latency + cycle timing).
    //
    // Real-slave mode (--slave N): an APRD datagram reads 2 bytes from
    // register 0x0000 (TYPE) of the slave at physical position N.  The slave
    // responds, and the receive path is measured (transmit-to-receive delay
    // and its jitter).  Physical addressing uses ADP = 0xFFFF - N.
    std::vector<uint8_t> frame;
    if (real_slave_mode) {
        // APRD: Ethernet (14) + EtherCAT hdr (2) + datagram hdr (10) +
        //       data (2) + WKC (2) = 30 bytes
        const size_t aprd_frame_size = 14 + 2 + 10 + 2 + 2;
        frame.resize(aprd_frame_size, 0);

        // Ethernet header — destination is the slave's MAC, but EtherCAT
        // slaves accept broadcast frames, so use the broadcast MAC.
        std::memcpy(frame.data(), EtherCAT::kEtherCATBroadcastMAC.data(), 6);
        std::memcpy(frame.data() + 6, session.srcMac, 6);
        frame[12] = static_cast<uint8_t>(
            (EtherCAT::kEtherTypeEtherCAT >> 8) & 0xFF);
        frame[13] = static_cast<uint8_t>(
            EtherCAT::kEtherTypeEtherCAT & 0xFF);

        // EtherCAT frame header: payload = datagram hdr + data + WKC.
        EtherCAT::FrameHeader ecat_hdr{};
        ecat_hdr.set(static_cast<uint16_t>(10 + 2 + 2), 1);
        std::memcpy(frame.data() + 14, &ecat_hdr, sizeof(ecat_hdr));

        // APRD datagram: read 2 bytes from register 0x0000 (TYPE) of
        // slave at physical position N.
        EtherCAT::DatagramHeader dgram_hdr{};
        dgram_hdr.cmd = EtherCAT::Command::APRD;
        dgram_hdr.idx = 0;
        // Physical addressing: ADP = 0xFFFF - slave_position.
        dgram_hdr.adp_le = static_cast<uint16_t>(
            0u - static_cast<uint16_t>(slave_index));
        dgram_hdr.ado_le = 0x0000;  // TYPE register
        dgram_hdr.setDataLength(2);
        dgram_hdr.setMore(false);
        dgram_hdr.setCirculating(false);
        dgram_hdr.irq_le = 0;
        std::memcpy(frame.data() + 16, &dgram_hdr, sizeof(dgram_hdr));
        // Data (2 bytes) and WKC (2 bytes) are already zeroed.
    } else {
        // Fake-slave LRW frame.
        const size_t frame_size = 14 + 2 + 10 + pdo_size + 2;
        frame.resize(frame_size, 0);

        // Ethernet header.
        std::memcpy(frame.data(), EtherCAT::kEtherCATBroadcastMAC.data(), 6);
        std::memcpy(frame.data() + 6, session.srcMac, 6);
        frame[12] = static_cast<uint8_t>(
            (EtherCAT::kEtherTypeEtherCAT >> 8) & 0xFF);
        frame[13] = static_cast<uint8_t>(
            EtherCAT::kEtherTypeEtherCAT & 0xFF);

        // EtherCAT frame header.
        EtherCAT::FrameHeader ecat_hdr{};
        ecat_hdr.set(static_cast<uint16_t>(10 + pdo_size + 2), 1);
        std::memcpy(frame.data() + 14, &ecat_hdr, sizeof(ecat_hdr));

        // Datagram header — LRW to logical address 0 (no real slave).
        EtherCAT::DatagramHeader dgram_hdr{};
        dgram_hdr.cmd = EtherCAT::Command::LRW;
        dgram_hdr.idx = 0;
        dgram_hdr.setLogicalAddress(0);
        dgram_hdr.setDataLength(static_cast<uint16_t>(pdo_size));
        dgram_hdr.setMore(false);
        dgram_hdr.setCirculating(false);
        dgram_hdr.irq_le = 0;
        std::memcpy(frame.data() + 16, &dgram_hdr, sizeof(dgram_hdr));
        // Payload and trailing WKC are already zeroed.
        (void)kFakeSlaveIndex;  // Logical addressing; index is conceptual.
    }

    // sockaddr_ll for sendto.
    struct sockaddr_ll sll;
    std::memset(&sll, 0, sizeof(sll));
    sll.sll_family = AF_PACKET;
    sll.sll_ifindex = static_cast<int>(ifindex);
    sll.sll_halen = ETH_ALEN;
    std::memcpy(sll.sll_addr, frame.data(), 6);

    const std::vector<bool> affinities = {false, true};

    std::vector<RunResult> results;
    results.reserve(affinities.size() * priorities.size() * frequencies.size());

    // Build the results table in memory so realtime-thread log messages do not
    // break the rendered table.
    std::ostringstream table_oss;

    // ---- Test explanation ----
    table_oss << "\n"
              << color.bold << color.cyan
              << "=== Test Description ===" << color.reset << "\n\n";
    if (real_slave_mode) {
        table_oss
            << "  Mode: REAL SLAVE round-trip communication.\n"
            << "  Target: slave at physical position " << slave_index
            << " (APRD command, register 0x0000 / TYPE, 2-byte read).\n"
            << "  Each cycle sends an APRD datagram to the slave via sendto(),\n"
            << "  then waits for the response frame via poll()+recvfrom().\n"
            << "  The transmit-to-receive delay (TX->RX) and its jitter are\n"
            << "  measured in addition to cycle timing and raw sendto latency.\n"
            << "  Lost responses (timeouts) are tracked separately from lost\n"
            << "  cycles.\n\n";
    } else {
        table_oss
            << "  Mode: FAKE SLAVE (transmit-only, no reception).\n"
            << "  Each cycle sends an LRW datagram with a " << pdo_size
            << "-byte PDO payload to logical address 0 via sendto().\n"
            << "  The datagram targets a non-existent slave (index "
            << kFakeSlaveIndex << "), so no slave will ever process or\n"
            << "  respond to the frame.  The receive path is NOT exercised —\n"
            << "  no recvfrom() is called and no response is expected.\n"
            << "  This isolates the transmit-side performance of the kernel\n"
            << "  network stack and the NIC driver: sendto() latency, cycle\n"
            << "  timing jitter, and CPU usage under SCHED_FIFO load.\n"
            << "  Use --slave N to additionally measure round-trip performance\n"
            << "  against a real slave at position N.\n\n";
    }
    table_oss << "  Each parameter combination (frequency x priority x affinity)\n"
              << "  runs for " << std::fixed << std::setprecision(1) << duration_s
              << " s with " << std::fixed << std::setprecision(1) << warmup_s
              << " s warmup discarded.\n\n";

    // ---- Results table ----
    table_oss << color.bold << color.cyan << "=== Results Table ===" << color.reset
              << "\n\n"
              << "Column explanation:\n"
              << "  Freq      Loop frequency.\n"
              << "  Prio      SCHED_FIFO priority used for the realtime thread.\n"
              << "  Affinity  CPU affinity: on = pinned to core " << affinity_core
              << ", off = any core.\n"
              << "  Sched     Whether SCHED_FIFO was successfully applied.\n"
              << "  CycMean   Mean inter-cycle delta (µs).\n"
              << "  CycMax    Maximum observed inter-cycle delta (µs).\n"
              << "  CycStd    Standard deviation of inter-cycle deltas (µs).\n";
    if (real_slave_mode) {
        table_oss
            << "  TxMean    Mean raw sendto() duration for the APRD frame (µs).\n"
            << "  TxMax     Maximum observed sendto() duration (µs).\n"
            << "  RxMean    Mean transmit-to-receive delay (µs) — time from\n"
            << "            sendto() completion to recvfrom() completion.\n"
            << "  RxMax     Maximum observed transmit-to-receive delay (µs).\n"
            << "  RxStd     Standard deviation of receive delays (µs).\n"
            << "  RxLoss%   Percentage of cycles with no response (timeout).\n";
    } else {
        table_oss
            << "  TxMean    Mean raw sendto() duration for the " << pdo_size
            << "-byte PDO (µs).\n"
            << "  TxMax     Maximum observed sendto() duration (µs).\n";
    }
    table_oss << "  CPU%      Thread CPU time / wall time * 100.\n"
              << "  Lost%     Estimated percentage of lost cycles (when the "
                 "previous cycle did not finish in time).\n"
              << "  Status    Overall run status.\n\n"
              << "Color thresholds are evaluated per row relative to the nominal "
                 "period P (µs):\n"
              << "  Cycle jitter (CycMean/CycMax/CycStd):\n"
              << "    green           = max deviation < P/10  (10%)\n"
              << "    orange (warn)   = max deviation < P/5   (20%)\n"
              << "    orange (high)   = max deviation < P/3   (33%)\n"
              << "    red (extreme)   = max deviation >= P/3  (33%)\n"
              << "  Transmit time (TxMean/TxMax):\n"
              << "    green           = mean < P/10  (10%)\n"
              << "    orange (warn)   = mean < P/5   (20%)\n"
              << "    orange (high)   = mean < P/3   (33%)\n"
              << "    red (extreme)   = mean >= P/3  (33%)\n";
    if (real_slave_mode) {
        table_oss
            << "  Receive delay (RxMean/RxMax):\n"
            << "    green           = mean < P/10  (10%)\n"
            << "    orange (warn)   = mean < P/5   (20%)\n"
            << "    orange (high)   = mean < P/3   (33%)\n"
            << "    red (extreme)   = mean >= P/3  (33%)\n"
            << "  Receive jitter (RxStd):\n"
            << "    green           = stddev < P/20  (5%)\n"
            << "    orange (warn)   = stddev < P/10  (10%)\n"
            << "    orange (high)   = stddev < P/5   (20%)\n"
            << "    red (extreme)   = stddev >= P/5  (20%)\n"
            << "  Response loss (RxLoss%):\n"
            << "    red (cycle-loss)          = at least one lost response, but <= 1%\n"
            << "    red (frequent-cycle-loss) = more than 1 lost response per 100 cycles\n";
    }
    table_oss << "  Cycle loss:\n"
              << "    red (cycle-loss)          = at least one lost cycle, but <= 1%\n"
              << "    red (frequent-cycle-loss) = more than 1 lost cycle per 100 cycles\n"
              << "  Status is the worst of the above; it is red if SCHED_FIFO "
                 "could not be applied.\n\n"
              << color.bold << color.cyan
              << std::left << std::setw(8) << "Freq"
              << std::setw(10) << "Prio"
              << std::setw(10) << "Affinity"
              << std::setw(8) << "Sched"
              << std::setw(14) << "CycMean[µs]"
              << std::setw(14) << "CycMax[µs]"
              << std::setw(14) << "CycStd[µs]"
              << std::setw(14) << "TxMean[µs]"
              << std::setw(14) << "TxMax[µs]";
    if (real_slave_mode) {
        table_oss << std::setw(14) << "RxMean[µs]"
                  << std::setw(14) << "RxMax[µs]"
                  << std::setw(14) << "RxStd[µs]"
                  << std::setw(12) << "RxLoss[%]";
    }
    table_oss << std::setw(12) << "CPU[%]"
              << std::setw(12) << "Lost[%]"
              << std::setw(20) << "Status"
              << color.reset << "\n";

    // Suppress INFO-level logs from realtime threads (e.g. the SCHED_FIFO
    // priority announcement) so they do not clutter the rendered table.
    const auto saved_log_level =
        Tether::Platform::Logger::instance().getLevel();
    Tether::Platform::Logger::instance().setLevel(
        Tether::Platform::LogLevel::Warn);

    for (int freq : frequencies) {
        for (int priority : priorities) {
            for (bool affinity : affinities) {
                RunConfig cfg;
                cfg.affinity = affinity;
                cfg.priority = priority;
                cfg.frequency_hz = freq;
                cfg.period_us =
                    static_cast<uint32_t>(1'000'000 / freq);
                cfg.affinity_core = affinity_core;
                cfg.duration_s = duration_s;
                cfg.warmup_s = warmup_s;
                cfg.frame = &frame;
                cfg.sll = &sll;
                cfg.sock = sock;
                cfg.real_slave = real_slave_mode;

                EtherCAT::HAL::ThreadConfig tcfg;
                tcfg.name = "rt_diag";
                tcfg.stackSize = 65536;
                tcfg.priority = EtherCAT::HAL::ThreadPriority::Normal;
                tcfg.useRealtimeScheduling = false;  // We set SCHED_FIFO manually.
                tcfg.cpuAffinity = affinity ? affinity_core : -1;

                RunResult result;
                auto thread =
                    EtherCAT::HAL::getThreadingFactory().createThread(tcfg);
                auto err = thread->start([&cfg, &result]() {
                    runThreadFunc(cfg, &result);
                });
                if (err != EtherCAT::HAL::Error::OK) {
                    TETHER_LOGE(TAG, "Failed to start realtime thread");
                    Tether::Examples::shutdownHostEthernet(session);
                    return 3;
                }
                thread->join();
                results.push_back(result);

                const SampleStats cs = computeStats(result.cycle_deltas_ns);
                const SampleStats ts = computeStats(result.tx_durations_ns);
                const SampleStats rs = computeStats(result.rx_delays_ns);
                const RunGrade cyc_grade =
                    classifyCycleJitter(result.period_us, cs.max_us);
                const RunGrade tx_grade =
                    classifyTxLatency(result.period_us, ts.mean_us);
                const RunGrade loss_grade =
                    classifyCycleLoss(result.lost_cycles, result.total_cycles);

                RunGrade run_grade;
                if (real_slave_mode) {
                    const RunGrade rxd_grade =
                        classifyRxDelay(result.period_us, rs.mean_us);
                    const RunGrade rxj_grade =
                        classifyRxJitter(result.period_us, rs.stddev_us);
                    const RunGrade rxl_grade =
                        classifyRxLoss(result.lost_responses,
                                       result.total_cycles);
                    run_grade = overallGradeRx(cyc_grade, tx_grade,
                                               rxd_grade, rxj_grade,
                                               rxl_grade, loss_grade,
                                               result.sched_ok);
                } else {
                    run_grade = overallGrade(cyc_grade, tx_grade, loss_grade,
                                             result.sched_ok);
                }

                const double cpu_pct =
                    result.wall_time_ns > 0
                        ? (static_cast<double>(result.cpu_time_ns) /
                           static_cast<double>(result.wall_time_ns)) *
                              100.0
                        : 0.0;
                const double lost_pct =
                    result.total_cycles > 0
                        ? (static_cast<double>(result.lost_cycles) /
                           static_cast<double>(result.total_cycles)) *
                              100.0
                        : 0.0;

                std::ostringstream cpu_oss;
                cpu_oss << std::fixed << std::setprecision(1) << cpu_pct << "%";
                std::ostringstream lost_oss;
                lost_oss << std::fixed << std::setprecision(5) << lost_pct << "%";

                table_oss << std::left
                          << std::setw(8) << formatFrequency(freq)
                          << std::setw(10) << priority
                          << std::setw(10) << (affinity ? "on" : "off")
                          << std::setw(8) << (result.sched_ok ? "OK" : "NO")
                          << coloredValue(color, cyc_grade, cs.mean_us, 1, 14, "µs")
                          << coloredValue(color, cyc_grade, cs.max_us, 1, 14, "µs")
                          << coloredValue(color, cyc_grade, cs.stddev_us, 1, 14, "µs")
                          << coloredValue(color, tx_grade, ts.mean_us, 1, 14, "µs")
                          << coloredValue(color, tx_grade, ts.max_us, 1, 14, "µs");
                if (real_slave_mode) {
                    const RunGrade rxd_grade =
                        classifyRxDelay(result.period_us, rs.mean_us);
                    const RunGrade rxj_grade =
                        classifyRxJitter(result.period_us, rs.stddev_us);
                    const double rxloss_pct =
                        result.total_cycles > 0
                            ? (static_cast<double>(result.lost_responses) /
                               static_cast<double>(result.total_cycles)) *
                                  100.0
                            : 0.0;
                    std::ostringstream rxloss_oss;
                    rxloss_oss << std::fixed << std::setprecision(5)
                               << rxloss_pct << "%";
                    table_oss
                        << coloredValue(color, rxd_grade, rs.mean_us, 1, 14, "µs")
                        << coloredValue(color, rxd_grade, rs.max_us, 1, 14, "µs")
                        << coloredValue(color, rxj_grade, rs.stddev_us, 1, 14, "µs")
                        << padRight(rxloss_oss.str(), 12);
                }
                table_oss << padRight(cpu_oss.str(), 12)
                          << padRight(lost_oss.str(), 12)
                          << gradeColor(color, run_grade)
                          << padRight(gradeString(run_grade), 20)
                          << color.reset << "\n";
            }
        }
    }

    Tether::Platform::Logger::instance().setLevel(saved_log_level);

    std::cout << table_oss.str();

    printRecommendations(color, results, kernel_info, priorities,
                         affinity_core);

    if (run_throughput) {
        runThroughputTest(color, sock, frame, &sll, real_slave_mode,
                          throughput_threshold);
    }

    Tether::Examples::shutdownHostEthernet(session);
    return 0;
#else
    (void)argc;
    (void)argv;
    std::cerr
        << "realtime_diagnostics is only supported on Linux.\n";
    return 1;
#endif // __linux__
}

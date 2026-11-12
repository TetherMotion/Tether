/**
 * @file Platform.cpp
 * @brief Default (portable) platform implementation
 * 
 * This provides std::chrono based timing and printf-based logging.
 * ESP32-specific implementation is in hal/esp32/ESP32Platform.cpp
 */

#include "tether/platform/Platform.hpp"

#include <thread>

#ifdef __linux__
#include <sys/utsname.h>
#include <sys/syscall.h>
#include <unistd.h>
#include <sched.h>
#include <algorithm>
#include <fstream>
#include <vector>
#include <cstdlib>
#include <cstring>
#include <cerrno>
#endif

namespace Tether {
namespace Platform {

//=============================================================================
// Clock Implementation
//=============================================================================

Clock& Clock::instance() {
    static Clock clock;
    return clock;
}

Clock::Clock() : startTime_(std::chrono::steady_clock::now()) {
    // Default implementations using std::chrono
    getMicros_ = [this]() -> int64_t {
        auto now = std::chrono::steady_clock::now();
        return std::chrono::duration_cast<std::chrono::microseconds>(
            now - startTime_).count();
    };
    
    delayMicros_ = [](uint32_t us) {
        if (us < 1000) {
            // Busy wait for short delays
            auto start = std::chrono::steady_clock::now();
            while (std::chrono::duration_cast<std::chrono::microseconds>(
                std::chrono::steady_clock::now() - start).count() < us) {
                // Spin
            }
        } else {
            // Use sleep for longer delays
            std::this_thread::sleep_for(std::chrono::microseconds(us));
        }
    };
    
    yieldFn_ = []() {
        std::this_thread::yield();
    };
}

int64_t Clock::getMicroseconds() const {
    return getMicros_();
}

int64_t Clock::getMilliseconds() const {
    return getMicros_() / 1000;
}

void Clock::delayMicroseconds(uint32_t us) const {
    delayMicros_(us);
}

void Clock::delayMilliseconds(uint32_t ms) const {
    delayMicros_(ms * 1000);
}

void Clock::yield() const {
    if (yieldFn_) {
        yieldFn_();
    }
}

//=============================================================================
// Platform Detection
//=============================================================================

void initialize() {
    // Default initialization - nothing special needed for portable code
}

bool isEsp32() {
#ifdef ESP_PLATFORM
    return true;
#else
    return false;
#endif
}

bool isLinux() {
#ifdef __linux__
    return true;
#else
    return false;
#endif
}

bool setCurrentThreadRealtime(int priority) {
#ifdef __linux__
    // Default priority if caller didn't specify
    if (priority <= 0) priority = 80;

    // Clamp to system limits for SCHED_FIFO
    int prio_max = sched_get_priority_max(SCHED_FIFO);
    int prio_min = sched_get_priority_min(SCHED_FIFO);
    if (prio_max <= 0) {
        TETHER_LOGW("Platform", "Failed to query SCHED_FIFO priority range; refusing to set realtime");
        return false;
    }
    if (priority > prio_max) priority = prio_max;
    if (priority < prio_min) priority = prio_min;

    struct sched_param param;
    param.sched_priority = priority;
    int ret = pthread_setschedparam(pthread_self(), SCHED_FIFO, &param);
    if (ret != 0) {
        if (ret == EPERM) {
            TETHER_LOGW("Platform", "setCurrentThreadRealtime: insufficient permissions to set SCHED_FIFO (CAP_SYS_NICE required)");
        } else {
            TETHER_LOGW("Platform", "setCurrentThreadRealtime: pthread_setschedparam failed ({})", ret);
        }
        return false;
    }

    TETHER_LOGI("Platform", "setCurrentThreadRealtime: SCHED_FIFO priority={} applied", param.sched_priority);
    return true;
#else
    // Non-Linux platforms: best-effort no-op (ESP32 FreeRTOS scheduling handled elsewhere)
    (void)priority;
    return true;
#endif
}

bool setCurrentThreadDeadline(uint64_t runtime_ns, uint64_t deadline_ns,
                              uint64_t period_ns) {
#ifdef __linux__
    // sched_attr has no stable glibc wrapper — use the syscall directly.
    struct sched_attr_local {
        uint32_t size;
        uint32_t sched_policy;
        uint64_t sched_flags;
        int32_t  sched_nice;
        uint32_t sched_priority;
        uint64_t sched_runtime;
        uint64_t sched_deadline;
        uint64_t sched_period;
    } attr{};
    attr.size           = sizeof(attr);
    attr.sched_policy   = 6 /* SCHED_DEADLINE */;
    attr.sched_runtime  = runtime_ns;
    attr.sched_deadline = deadline_ns;
    attr.sched_period   = period_ns;
    const long ret = syscall(SYS_sched_setattr, 0, &attr, 0);
    if (ret != 0) {
        TETHER_LOGW("Platform",
            "setCurrentThreadDeadline: sched_setattr failed ({}) — "
            "caller should fall back to SCHED_FIFO", strerror(errno));
        return false;
    }
    TETHER_LOGI("Platform",
        "SCHED_DEADLINE applied: runtime={}ns deadline={}ns period={}ns",
        runtime_ns, deadline_ns, period_ns);
    return true;
#else
    (void)runtime_ns; (void)deadline_ns; (void)period_ns;
    return false;
#endif
}

bool setCurrentThreadAffinity(int cpu) {
#ifdef __linux__
    if (cpu < 0) return false;
    cpu_set_t set;
    CPU_ZERO(&set);
    CPU_SET(cpu, &set);
    return sched_setaffinity(0, sizeof(set), &set) == 0;
#else
    (void)cpu;
    return false;
#endif
}

//=============================================================================
// CPU Isolation Detection
//=============================================================================

#ifdef __linux__

namespace {

/**Parse a kernel cpulist ("0,2-3,5") into indices; empty → {}. */
std::vector<int> parseCpuList(const std::string& list) {
    std::vector<int> cpus;
    size_t pos = 0;
    while (pos < list.size()) {
        const char* p = list.c_str() + pos;
        char* end = nullptr;
        const long first = std::strtol(p, &end, 10);
        if (end == p) break;
        long last = first;
        pos = static_cast<size_t>(end - list.c_str());
        if (pos < list.size() && list[pos] == '-') {
            const long hi = std::strtol(list.c_str() + pos + 1, &end, 10);
            if (end == list.c_str() + pos + 1) break;
            last = hi;
            pos = static_cast<size_t>(end - list.c_str());
        }
        for (long c = first; c <= last && c < 4096; ++c)
            cpus.push_back(static_cast<int>(c));
        if (pos >= list.size() || list[pos] != ',') break;
        ++pos;
    }
    return cpus;
}

std::string readSmallFile(const char* path) {
    std::ifstream f(path);
    if (!f.is_open()) return {};
    std::string s;
    std::getline(f, s);
    while (!s.empty() && (s.back() == '\n' || s.back() == '\r' ||
                          s.back() == ' ' || s.back() == '\t'))
        s.pop_back();
    return s;
}

/**
 * One line of /proc/thread-self/cgroup: "hier:subsys:path".  The v2
 * unified hierarchy has an empty subsys ("0::/path"); v1 mounts list
 * comma-separated controllers ("4:cpuset:/tether_rt").  Thread-granular
 * on purpose: an RT thread migrated into the exclusive cpuset sees its
 * own membership here — /proc/self/cgroup reports the main thread's.
 */
struct OwnCgroup {
    std::string subsys; /**< "" for the v2 unified hierarchy */
    std::string path;
};

std::vector<OwnCgroup> ownCgroups() {
    std::vector<OwnCgroup> out;
    std::ifstream f("/proc/thread-self/cgroup");
    std::string line;
    while (std::getline(f, line)) {
        const auto c1 = line.find(':');
        const auto c2 = c1 == std::string::npos
            ? std::string::npos : line.find(':', c1 + 1);
        if (c1 == std::string::npos || c2 == std::string::npos) continue;
        out.push_back({line.substr(c1 + 1, c2 - c1 - 1),
                       line.substr(c2 + 1)});
    }
    return out;
}

bool subsysHasCpuset(const std::string& subsys) {
    return subsys == "cpuset" ||
           subsys.rfind("cpuset,", 0) == 0 ||
           subsys.find(",cpuset,") != std::string::npos ||
           (subsys.size() > 7 &&
            subsys.compare(subsys.size() - 7, 7, ",cpuset") == 0);
}

/**
 * CPUs dedicated to this thread by the cgroups it belongs to:
 *
 *  - v2: our own cgroup is an "isolated" partition → its isolated set.
 *  - v1: we sit inside a cpu_exclusive cpuset (CpuIsolation's tether_rt
 *    or an admin-made one) → its cpuset.cpus.
 *
 * Foreign partitions/exclusive cpusets are deliberately NOT counted —
 * their CPUs are outside our effective mask, so this thread could never
 * be scheduled on them anyway.
 */
std::vector<int> cpusetIsolatedCpus() {
    std::vector<int> cpus;
    auto add = [&](const std::string& list) {
        for (int cpu : parseCpuList(list)) {
            if (std::find(cpus.begin(), cpus.end(), cpu) == cpus.end())
                cpus.push_back(cpu);
        }
    };
    for (const auto& g : ownCgroups()) {
        if (g.subsys.empty()) {
            // cgroup v2 unified hierarchy.
            const std::string dir = "/sys/fs/cgroup" + g.path;
            if (readSmallFile((dir + "/cpuset.cpus.partition").c_str())
                    != "isolated")
                continue;
            // cpuset.cpus.isolated (newer kernels) is the precise set;
            // fall back to the partition's effective/requested cpus.
            std::string c =
                readSmallFile((dir + "/cpuset.cpus.isolated").c_str());
            if (c.empty())
                c = readSmallFile((dir + "/cpuset.cpus.effective").c_str());
            if (c.empty())
                c = readSmallFile((dir + "/cpuset.cpus").c_str());
            add(c);
        } else if (subsysHasCpuset(g.subsys)) {
            // cgroup v1 cpuset hierarchy — cpu_exclusive asserts that no
            // sibling cgroup may schedule on these CPUs.
            const std::string dir = "/sys/fs/cgroup/cpuset" + g.path;
            if (readSmallFile((dir + "/cpuset.cpu_exclusive").c_str())
                    != "1")
                continue;
            add(readSmallFile((dir + "/cpuset.cpus").c_str()));
        }
    }
    return cpus;
}

/**Union of every hard-isolated CPU visible to us. */
std::vector<int> hardIsolatedCpus() {
    std::vector<int> cpus = cpusetIsolatedCpus();
    for (int cpu : parseCpuList(
             readSmallFile("/sys/devices/system/cpu/isolated"))) {
        if (std::find(cpus.begin(), cpus.end(), cpu) == cpus.end())
            cpus.push_back(cpu);
    }
    return cpus;
}

} // anonymous namespace

#endif // __linux__

bool cpuIsHardIsolated(int cpu) {
#ifdef __linux__
    if (cpu < 0) return false;
    const auto iso = hardIsolatedCpus();
    return std::find(iso.begin(), iso.end(), cpu) != iso.end();
#else
    (void)cpu;
    return false;
#endif
}

int64_t rtThrottleRuntimeUs() {
#ifdef __linux__
    const std::string v =
        readSmallFile("/proc/sys/kernel/sched_rt_runtime_us");
    if (v.empty()) return -2;
    char* end = nullptr;
    const long long us = std::strtoll(v.c_str(), &end, 10);
    if (end == v.c_str()) return -2;
    return static_cast<int64_t>(us);
#else
    return -2;
#endif
}

bool disableRtThrottle() {
#ifdef __linux__
    std::ofstream f("/proc/sys/kernel/sched_rt_runtime_us");
    if (!f.is_open()) return false;
    f << "-1\n";
    f.flush();
    return f.good();
#else
    return false;
#endif
}

bool currentThreadRunsIsolatedOnly() {
#ifdef __linux__
    const auto iso = hardIsolatedCpus();
    if (iso.empty()) return false;
    // sched_getaffinity already returns the thread's mask intersected
    // with its cpuset(s) — if every runnable CPU is hard-isolated, the
    // thread is fully on dedicated cores.
    cpu_set_t set;
    if (sched_getaffinity(0, sizeof(set), &set) != 0) return false;
    bool any = false;
    for (int c = 0; c < CPU_SETSIZE; ++c) {
        if (!CPU_ISSET(c, &set)) continue;
        any = true;
        if (std::find(iso.begin(), iso.end(), c) == iso.end())
            return false;
    }
    return any;
#else
    return false;
#endif
}

//=============================================================================
// Realtime Kernel Detection
//=============================================================================

#ifdef __linux__

namespace {

/**Read a small file into a string; returns empty on failure. */
std::string readFileToString(const char* path) {
    std::ifstream f(path);
    if (!f.is_open()) return {};
    std::string content;
    std::getline(f, content);
    return content;
}

/**Map uname.version substring to PreemptModel (build model). */
PreemptModel parseBuildModel(const std::string& version) {
    if (version.find("PREEMPT_RT") != std::string::npos)
        return PreemptModel::PreemptRt;
    if (version.find("PREEMPT_DYNAMIC") != std::string::npos)
        return PreemptModel::PreemptDynamic;
    if (version.find("PREEMPT") != std::string::npos)
        return PreemptModel::PreemptFull;
    if (version.find("VOLUNTARY") != std::string::npos)
        return PreemptModel::PreemptVoluntary;
    return PreemptModel::PreemptNone;
}

/**Map a mode string to PreemptModel. */
PreemptModel modeStringToModel(const std::string& mode) {
    if (mode == "full")  return PreemptModel::PreemptFull;
    if (mode == "lazy")  return PreemptModel::PreemptFull; // lazy is a relaxed full
    if (mode == "voluntary") return PreemptModel::PreemptVoluntary;
    if (mode == "none")  return PreemptModel::PreemptNone;
    return PreemptModel::Unknown;
}

/**
 * For PREEMPT_DYNAMIC kernels, resolve the currently active preempt mode.
 * Tries /sys/kernel/debug/sched/preempt first (format: "(full) lazy"),
 * then falls back to the preempt= boot parameter in /proc/cmdline.
 */
std::string parseActiveDynamicMode(std::string& source_out) {
    // 1. debugfs: /sys/kernel/debug/sched/preempt
    //    Format: "(full) lazy" — parenthesized token is the active mode.
    {
        std::string content = readFileToString("/sys/kernel/debug/sched/preempt");
        if (!content.empty()) {
            // Find the parenthesized token
            auto lparen = content.find('(');
            auto rparen = content.find(')', lparen == std::string::npos ? 0 : lparen);
            if (lparen != std::string::npos && rparen != std::string::npos && rparen > lparen) {
                std::string mode = content.substr(lparen + 1, rparen - lparen - 1);
                source_out = "/sys/kernel/debug/sched/preempt";
                return mode;
            }
        }
    }

    // 2. /proc/cmdline: look for preempt=<mode>
    {
        std::string cmdline = readFileToString("/proc/cmdline");
        if (!cmdline.empty()) {
            auto pos = cmdline.find("preempt=");
            if (pos != std::string::npos) {
                auto start = pos + 8; // length of "preempt="
                auto end = cmdline.find(' ', start);
                if (end == std::string::npos) end = cmdline.size();
                std::string mode = cmdline.substr(start, end - start);
                source_out = "/proc/cmdline";
                return mode;
            }
        }
    }

    source_out = "unavailable (debugfs unreadable, no preempt= boot param)";
    return "unknown";
}

/**Classify the realtime level from build model and active mode string. */
RealtimeClass classifyRealtime(PreemptModel build_model,
                               const std::string& active_mode_str) {
    if (build_model == PreemptModel::PreemptRt)
        return RealtimeClass::HardRealtime;

    // For dynamic kernels, classify based on the active mode
    if (build_model == PreemptModel::PreemptDynamic) {
        if (active_mode_str == "full")
            return RealtimeClass::LowLatency;
        if (active_mode_str == "lazy" || active_mode_str == "voluntary")
            return RealtimeClass::Voluntary;
        return RealtimeClass::None; // "none" or "unknown"
    }

    // Static models
    if (build_model == PreemptModel::PreemptFull)
        return RealtimeClass::LowLatency;
    if (build_model == PreemptModel::PreemptVoluntary)
        return RealtimeClass::Voluntary;
    return RealtimeClass::None;
}

const char* preemptModelStr(PreemptModel m) {
    switch (m) {
        case PreemptModel::PreemptRt:        return "PREEMPT_RT";
        case PreemptModel::PreemptDynamic:   return "PREEMPT_DYNAMIC";
        case PreemptModel::PreemptFull:      return "PREEMPT (full)";
        case PreemptModel::PreemptVoluntary: return "PREEMPT_VOLUNTARY";
        case PreemptModel::PreemptNone:      return "PREEMPT_NONE";
        default:                             return "Unknown";
    }
}

const char* realtimeClassStr(RealtimeClass c) {
    switch (c) {
        case RealtimeClass::HardRealtime: return "HardRealtime";
        case RealtimeClass::LowLatency:   return "LowLatency";
        case RealtimeClass::Voluntary:    return "Voluntary";
        case RealtimeClass::None:         return "None";
        default:                          return "Unknown";
    }
}

} // anonymous namespace

RealtimeKernelInfo detectRealtimeKernel() {
    RealtimeKernelInfo info;

    struct utsname uname_data;
    if (uname(&uname_data) != 0) {
        info.detection_source = "uname() failed";
        info.active_preempt_mode = "unknown";
        return info;
    }

    info.sysname = uname_data.sysname;
    info.kernel_release = uname_data.release;
    info.kernel_version = uname_data.version;
    info.build_model = parseBuildModel(uname_data.version);

    // Determine is_realtime: check /sys/kernel/realtime first, then uname fallback
    std::string sysfs_rt = readFileToString("/sys/kernel/realtime");
    if (!sysfs_rt.empty()) {
        // Trim whitespace
        while (!sysfs_rt.empty() &&
               (sysfs_rt.back() == '\n' || sysfs_rt.back() == ' ' || sysfs_rt.back() == '\r'))
            sysfs_rt.pop_back();
        info.is_realtime = (sysfs_rt == "1");
    } else {
        // Fallback: uname version contains PREEMPT_RT
        info.is_realtime = (info.build_model == PreemptModel::PreemptRt);
    }

    // Resolve active mode
    if (info.build_model == PreemptModel::PreemptDynamic) {
        std::string source;
        info.active_preempt_mode = parseActiveDynamicMode(source);
        info.detection_source = source;
        info.active_model = modeStringToModel(info.active_preempt_mode);
    } else {
        info.active_model = info.build_model;
        // For non-dynamic kernels, the "active mode" is implied by the build model
        switch (info.build_model) {
            case PreemptModel::PreemptRt:        info.active_preempt_mode = "rt"; break;
            case PreemptModel::PreemptFull:      info.active_preempt_mode = "full"; break;
            case PreemptModel::PreemptVoluntary: info.active_preempt_mode = "voluntary"; break;
            case PreemptModel::PreemptNone:      info.active_preempt_mode = "none"; break;
            default:                             info.active_preempt_mode = "unknown"; break;
        }
        info.detection_source = "uname";
    }

    // Classify
    info.realtime_class = classifyRealtime(info.build_model, info.active_preempt_mode);
    info.is_low_latency = (info.realtime_class == RealtimeClass::LowLatency ||
                           info.realtime_class == RealtimeClass::HardRealtime);

    return info;
}

RealtimeKernelInfo ensureRealtimeKernelOrExit(RealtimeRequirement req) {
    RealtimeKernelInfo info = detectRealtimeKernel();

    // Log detection results
    TETHER_LOGI("Platform", "Kernel: {} {}", info.sysname.c_str(), info.kernel_release.c_str());
    TETHER_LOGI("Platform", "  Build model: {}", preemptModelStr(info.build_model));
    if (info.build_model == PreemptModel::PreemptDynamic) {
        TETHER_LOGI("Platform", "  Active dynamic mode: {} (source: {})",
                    info.active_preempt_mode.c_str(), info.detection_source.c_str());
    }
    TETHER_LOGI("Platform", "  Realtime class: {} (is_realtime={}, is_low_latency={})",
                realtimeClassStr(info.realtime_class),
                info.is_realtime, info.is_low_latency);

    if (req == RealtimeRequirement::None) {
        return info; // detect and log only
    }

    // Evaluate requirement
    if (req == RealtimeRequirement::HardRealtime) {
        if (info.realtime_class == RealtimeClass::HardRealtime) {
            TETHER_LOGI("Platform", "Hard realtime requirement met (PREEMPT_RT kernel)");
            return info;
        }
        if (info.realtime_class == RealtimeClass::LowLatency) {
            TETHER_LOGW("Platform",
                "Hard realtime requirement NOT met: kernel is low-latency desktop "
                "(build={}, active={}), not PREEMPT_RT. Continuing with degraded "
                "realtime guarantees. For hard realtime, install a PREEMPT_RT kernel.",
                preemptModelStr(info.build_model), info.active_preempt_mode.c_str());
            return info;
        }
        // Active mode unknown (e.g. PREEMPT_DYNAMIC with debugfs unreadable
        // without root): we cannot determine the actual active preempt mode.
        // Warn and continue, optimistically assuming the user has configured
        // the kernel for low latency (e.g. via boot param or runtime knob).
        if (info.active_preempt_mode == "unknown") {
            TETHER_LOGW("Platform",
                "Hard realtime requirement: kernel realtime class is '{}' "
                "(build={}, active={}). The active preempt mode could not be "
                "determined — this is unknowable without root/sudo access to "
                "read debugfs (/sys/kernel/debug/sched/preempt). Assuming the "
                "kernel has been configured for low latency. Continuing with "
                "NO realtime guarantees; verify with `sudo cat "
                "/sys/kernel/debug/sched/preempt` and a PREEMPT_RT kernel for "
                "hard realtime PDO exchange.",
                realtimeClassStr(info.realtime_class),
                preemptModelStr(info.build_model), info.active_preempt_mode.c_str());
            return info;
        }

        // Below low-latency → error
        TETHER_LOGE("Platform",
            "Hard realtime requirement NOT met: kernel realtime class is '{}' "
            "(build={}, active={}). A PREEMPT_RT (or at minimum full-preempt) kernel "
            "is required for realtime PDO exchange. Aborting.",
            realtimeClassStr(info.realtime_class),
            preemptModelStr(info.build_model), info.active_preempt_mode.c_str());
        std::exit(EXIT_FAILURE);
    }

    if (req == RealtimeRequirement::LowLatency) {
        if (info.is_low_latency) {
            TETHER_LOGI("Platform", "Low-latency requirement met");
            return info;
        }
        TETHER_LOGE("Platform",
            "Low-latency requirement NOT met: kernel realtime class is '{}' "
            "(build={}, active={}). At least full preempt is required. Aborting.",
            realtimeClassStr(info.realtime_class),
            preemptModelStr(info.build_model), info.active_preempt_mode.c_str());
        std::exit(EXIT_FAILURE);
    }

    return info;
}

#else // !__linux__

RealtimeKernelInfo detectRealtimeKernel() {
    RealtimeKernelInfo info;
    info.is_realtime = true; // best-effort: non-Linux platforms handled elsewhere
    info.is_low_latency = true;
    info.realtime_class = RealtimeClass::HardRealtime;
    info.build_model = PreemptModel::Unknown;
    info.active_model = PreemptModel::Unknown;
    info.active_preempt_mode = "n/a";
    info.detection_source = "non-linux (unsupported)";
    return info;
}

RealtimeKernelInfo ensureRealtimeKernelOrExit(RealtimeRequirement req) {
    (void)req;
    return detectRealtimeKernel();
}

#endif // __linux__

} // namespace Platform
} // namespace Tether

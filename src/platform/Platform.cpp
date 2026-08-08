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
#include <fstream>
#include <cstdlib>
#include <cstring>
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
            TETHER_LOGW("Platform", "setCurrentThreadRealtime: pthread_setschedparam failed (%d)", ret);
        }
        return false;
    }

    TETHER_LOGI("Platform", "setCurrentThreadRealtime: SCHED_FIFO priority=%d applied", param.sched_priority);
    return true;
#else
    // Non-Linux platforms: best-effort no-op (ESP32 FreeRTOS scheduling handled elsewhere)
    (void)priority;
    return true;
#endif
}

//=============================================================================
// Realtime Kernel Detection
//=============================================================================

#ifdef __linux__

namespace {

/// Read a small file into a string; returns empty on failure.
std::string readFileToString(const char* path) {
    std::ifstream f(path);
    if (!f.is_open()) return {};
    std::string content;
    std::getline(f, content);
    return content;
}

/// Map uname.version substring to PreemptModel (build model).
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

/// Map a mode string to PreemptModel.
PreemptModel modeStringToModel(const std::string& mode) {
    if (mode == "full")  return PreemptModel::PreemptFull;
    if (mode == "lazy")  return PreemptModel::PreemptFull; // lazy is a relaxed full
    if (mode == "voluntary") return PreemptModel::PreemptVoluntary;
    if (mode == "none")  return PreemptModel::PreemptNone;
    return PreemptModel::Unknown;
}

/// For PREEMPT_DYNAMIC kernels, resolve the currently active preempt mode.
/// Tries /sys/kernel/debug/sched/preempt first (format: "(full) lazy"),
/// then falls back to the preempt= boot parameter in /proc/cmdline.
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

/// Classify the realtime level from build model and active mode string.
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
    TETHER_LOGI("Platform", "Kernel: %s %s", info.sysname.c_str(), info.kernel_release.c_str());
    TETHER_LOGI("Platform", "  Build model: %s", preemptModelStr(info.build_model));
    if (info.build_model == PreemptModel::PreemptDynamic) {
        TETHER_LOGI("Platform", "  Active dynamic mode: %s (source: %s)",
                    info.active_preempt_mode.c_str(), info.detection_source.c_str());
    }
    TETHER_LOGI("Platform", "  Realtime class: %s (is_realtime=%d, is_low_latency=%d)",
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
                "(build=%s, active=%s), not PREEMPT_RT. Continuing with degraded "
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
                "Hard realtime requirement: kernel realtime class is '%s' "
                "(build=%s, active=%s). The active preempt mode could not be "
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
            "Hard realtime requirement NOT met: kernel realtime class is '%s' "
            "(build=%s, active=%s). A PREEMPT_RT (or at minimum full-preempt) kernel "
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
            "Low-latency requirement NOT met: kernel realtime class is '%s' "
            "(build=%s, active=%s). At least full preempt is required. Aborting.",
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

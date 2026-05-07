#include "logging/DeduplicatingLogger.hpp"

namespace Tether {
namespace Logging {

DeduplicatingLogger::DeduplicatingLogger(const char* group_key, DedupLogConfig config)
    : group_key_(group_key ? group_key : ""), config_(config) {}

bool DeduplicatingLogger::shouldLogLocked(int64_t now_ms, const char* tag)
{
    if (!initialized_) {
        initialized_ = true;
        window_start_ms_ = now_ms;
    }

    const int64_t elapsed = now_ms - window_start_ms_;
    if (elapsed >= static_cast<int64_t>(config_.window_ms)) {
        if (config_.emit_summary && suppressed_in_window_ > 0) {
            emitSummaryLocked(tag);
        }
        window_start_ms_ = now_ms;
        emitted_in_window_ = 0;
        suppressed_in_window_ = 0;
    }

    if (emitted_in_window_ < config_.max_per_window) {
        emitted_in_window_++;
        return true;
    }

    suppressed_in_window_++;
    return false;
}

void DeduplicatingLogger::emitSummaryLocked(const char* tag)
{
    if (suppressed_in_window_ == 0) return;

    const uint32_t window_ms = config_.window_ms;
    const uint32_t suppressed = suppressed_in_window_;

    Tether::Platform::Logger::instance().log(
        config_.summary_level,
        tag,
        "(%s suppressed %u messages in last %u ms)",
        group_key_,
        suppressed,
        window_ms);
}

bool DeduplicatingLogger::log(Tether::Platform::LogLevel level, const char* tag, const char* msg)
{
    if (!tag) tag = "";
    if (!msg) msg = "";

    std::lock_guard<std::mutex> lock(mutex_);
    const int64_t now_ms = Tether::Platform::Clock::instance().getMilliseconds();

    if (!shouldLogLocked(now_ms, tag)) return false;

    Tether::Platform::Logger::instance().log(level, tag, "%s", msg);
    return true;
}

bool DeduplicatingLogger::logLegacy(int level, const char* tag, const char* msg)
{
    Tether::Platform::LogLevel lvl = Tether::Platform::LogLevel::Debug;
    switch (level) {
        case 0: lvl = Tether::Platform::LogLevel::Info; break;
        case 1: lvl = Tether::Platform::LogLevel::Warn; break;
        case 2: lvl = Tether::Platform::LogLevel::Error; break;
        default: lvl = Tether::Platform::LogLevel::Debug; break;
    }
    return log(lvl, tag, msg);
}

void DeduplicatingLogger::flush(const char* tag)
{
    if (!tag) tag = "";
    std::lock_guard<std::mutex> lock(mutex_);
    if (config_.emit_summary && suppressed_in_window_ > 0) {
        emitSummaryLocked(tag);
    }
    emitted_in_window_ = 0;
    suppressed_in_window_ = 0;
    initialized_ = false;
}

void DeduplicatingLogger::reset()
{
    std::lock_guard<std::mutex> lock(mutex_);
    emitted_in_window_ = 0;
    suppressed_in_window_ = 0;
    initialized_ = false;
    window_start_ms_ = 0;
}

}  // namespace Logging
}  // namespace Tether

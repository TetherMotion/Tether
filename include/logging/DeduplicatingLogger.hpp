#pragma once

#include <cstdint>
#include <mutex>

#include "tether/platform/Platform.hpp"

namespace Tether {
namespace Logging {

struct DedupLogConfig {
    uint32_t max_per_window = 2;
    uint32_t window_ms = 10'000;
    bool emit_summary = true;
    Tether::Platform::LogLevel summary_level = Tether::Platform::LogLevel::Info;
};

/**
 * @brief Rate-limits/deduplicates a specific "group" of log messages.
 *
 * Each instance represents one message group (same kind of event, varying string content).
 * Within a fixed time window it allows up to N logs and suppresses the rest.
 */
class DeduplicatingLogger {
public:
    DeduplicatingLogger(const char* group_key, DedupLogConfig config = {});

    bool log(Tether::Platform::LogLevel level, const char* tag, const char* msg);

    // Compatibility helper for legacy integer levels used by some code paths.
    // Mapping matches the old EtherCATMaster logDedup: 0=Info, 1=Warn, 2=Error, default=Debug.
    bool logLegacy(int level, const char* tag, const char* msg);

    // Emit a one-line "suppressed N" message (if any) and reset counters.
    void flush(const char* tag);

    void reset();

private:
    void emitSummaryLocked(const char* tag);
    bool shouldLogLocked(int64_t now_ms, const char* tag);

    const char* group_key_;
    DedupLogConfig config_;

    int64_t window_start_ms_ = 0;
    uint32_t emitted_in_window_ = 0;
    uint32_t suppressed_in_window_ = 0;
    bool initialized_ = false;

    std::mutex mutex_;
};

}  // namespace Logging
}  // namespace Tether

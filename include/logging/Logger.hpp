#pragma once

#include <cstdint>
#include <cstddef>
#include <cstdarg>
#include <cstdio>
#include <format>
#include <functional>
#include <string>
#include <string_view>
#include <algorithm>
#include <array>
#include <atomic>
#include <iterator>
#include <mutex>
#include <thread>
#include <utility>
#include <vector>

namespace Tether {
namespace Platform {

//=============================================================================
// Log Levels
//=============================================================================

enum class LogLevel {
    None = 0,
    Error = 1,
    Warn = 2,
    Info = 3,
    Debug = 4,
    Verbose = 5
};

namespace detail {

/**
 * Stack buffer size for the non-allocating log format path.  Messages
 * that don't fit fall back to the heap-formatting path (non-deferred
 * threads) or are truncated (deferred/RT threads).
 */
inline constexpr size_t kLogStackBufSize = 704;

/**
 * Bounded output iterator for std::vformat_to / fmt::vformat_to: writes
 * into a fixed buffer and silently drops overflow.  `pos` is a pointer
 * to the caller's cursor so post-increment copies (`*it++ = c`) still
 * advance the shared position — required for the output-iterator idiom.
 */
struct BoundedFormatOut {
    using iterator_category = std::output_iterator_tag;
    using value_type        = char;
    using difference_type   = std::ptrdiff_t;

    char** pos;
    char*  end;

    BoundedFormatOut& operator*() { return *this; }
    BoundedFormatOut& operator=(char c) {
        if (*pos < end) { **pos = c; ++*pos; }
        return *this;
    }
    BoundedFormatOut& operator++() { return *this; }
    BoundedFormatOut  operator++(int) { return *this; }
};

} // namespace detail

// Helper that wraps std::vformat so callers can format with a runtime
// format string. Arguments are taken by value (to support packed
// bitfields and rvalues) and referenced by std::make_format_args.
template <typename... Args>
inline std::string logFormat(std::string_view fmt, Args... args) {
    return std::vformat(fmt, std::make_format_args(args...));
}

//=============================================================================
// Logging Interface
//=============================================================================

/**
 * @brief Platform-independent logging interface
 *
 * Default implementation uses printf. Can be overridden for ESP-IDF, etc.
 *
 * Realtime threads: a thread marked with setThreadDeferred(true) never
 * formats onto the heap, takes a mutex, calls handlers, or prints from
 * the log call itself — messages are copied into a bounded lock-free
 * ring and emitted by an internal flusher thread (see rt_queue_).  This
 * is what keeps e.g. the cyclic EtherCAT thread free of logging-induced
 * jitter: a critical warning must not stall the loop that triggered it.
 */
class Logger {
public:
    static Logger& instance();

    ~Logger();

    void setLevel(LogLevel level) {
        level_.store(level, std::memory_order_relaxed);
    }
    LogLevel getLevel() const {
        return level_.load(std::memory_order_relaxed);
    }

    /**
     * @brief Cheap pre-check used by the log macros before formatting.
     * One relaxed atomic load — a suppressed message costs nothing.
     */
    bool enabled(LogLevel level) const {
        return level <= level_.load(std::memory_order_relaxed);
    }

    void log(LogLevel level, const char* tag, const char* format, ...);
    void logv(LogLevel level, const char* tag, const char* format, va_list args);
    void logFormatted(LogLevel level, const char* tag, std::string_view message);

    /**
     * Runtime-format entry point for the TETHER_LOGx macros.  Checks the
     * level BEFORE formatting, formats into a stack buffer (no heap
     * allocation) and hands a string_view to logFormatted — which on a
     * deferred-marked thread only enqueues, never formats/locks/prints.
     * Arguments are taken by value (packed bitfields, rvalues).
     */
    template <typename... Args>
    void logV(LogLevel level, const char* tag, std::string_view fmt,
              Args... args) {
        if (!enabled(level)) return;
        char buf[detail::kLogStackBufSize];
        char* p = buf;
        detail::BoundedFormatOut out{&p, buf + sizeof(buf)};
        std::vformat_to(out, fmt, std::make_format_args(args...));
        if (p == buf + sizeof(buf)) {
            // Buffer filled — may be truncated.  The deferred path keeps
            // the bounded text; the inline path falls back to heap so
            // long non-RT messages stay untruncated.
            if (threadDeferred()) {
                logFormatted(level, tag,
                             std::string_view(buf, sizeof(buf)));
            } else {
                logFormatted(level, tag, logFormat(fmt, args...));
            }
            return;
        }
        logFormatted(level, tag,
                     std::string_view(buf, static_cast<size_t>(p - buf)));
    }

    /**
     * Compile-time checked entry point for the TETHER_LOGx_FMT macros.
     * The consteval literal check already ran at the conversion to
     * format_string, so this simply delegates to the bounded vformat
     * path through fmt.get().
     */
    template <typename... Args>
    void logF(LogLevel level, const char* tag,
              std::format_string<Args...> fmt, Args... args) {
        const auto sv = fmt.get();
        logV(level, tag, std::string_view(sv.data(), sv.size()), args...);
    }

    using LogHandler = std::function<void(LogLevel, const char*, const char*)>;
    void setHandler(LogHandler handler) {
        std::lock_guard<std::mutex> lock(mutex_);
        handler_ = std::move(handler);
    }
    using HandlerId = size_t;
    HandlerId addHandler(LogHandler handler);
    void removeHandler(HandlerId id);

    // Enable or disable printing ISO8601-like timestamps on console output
    // (default: enabled). This affects the default printf-based output only
    // and does not modify messages delivered to a custom handler.
    void setTimestampEnabled(bool enabled) {
        std::lock_guard<std::mutex> lock(mutex_);
        timestampEnabled_ = enabled;
    }
    bool isTimestampEnabled() const {
        std::lock_guard<std::mutex> lock(mutex_);
        return timestampEnabled_;
    }

    /**
     * @brief Mark the calling thread as a realtime (deferred) logger.
     *
     * After this call the log macros on this thread only format into a
     * stack buffer and push into the bounded RT ring — no heap, no
     * mutex, no printf, no handler invocation.  The internal flusher
     * thread emits them.  Called once from RT thread entry points
     * (CyclicExecutive, AsyncCyclicLoop, RealtimeLoop).
     */
    static void setThreadDeferred(bool en = true);
    static bool threadDeferred();

    /** @brief Messages dropped because the RT ring was full. */
    uint64_t deferredDropped() const {
        return rt_dropped_.load(std::memory_order_relaxed);
    }

    /**
     * @brief Drain pending deferred messages on the calling thread.
     * For tests and shutdown paths — RT producers never call this.
     */
    void flushDeferred();

private:
    Logger();

    std::atomic<LogLevel> level_{LogLevel::Info};
    LogHandler handler_;
    std::vector<std::pair<HandlerId, LogHandler>> handlers_;
    HandlerId nextHandlerId_ = 1;
    bool timestampEnabled_ = true;
    mutable std::mutex mutex_;

    //=====================================================================
    // Deferred RT log ring
    //
    // Bounded MPMC ring using Dmitry Vyukov's per-slot sequence-counter
    // scheme.  Producers (deferred threads) only CAS claim a slot, memcpy
    // a bounded payload, and publish the sequence — no allocation, no
    // mutex, no wait; a full ring drops the message and counts it.
    // A dedicated flusher thread dequeues and runs the normal
    // logFormatted path (handlers/printf) off the RT thread.
    //=====================================================================
    static constexpr uint32_t kRtQueueCapacity = 64;   ///< power of two
    static constexpr uint32_t kRtTagSize       = 32;
    static constexpr uint32_t kRtTextSize      = detail::kLogStackBufSize;

    struct RtLogSlot {
        std::atomic<uint64_t> seq{0};
        LogLevel level{LogLevel::Info};
        char     tag[kRtTagSize]{};
        uint32_t text_len{0};
        char     text[kRtTextSize]{};
    };

    std::array<RtLogSlot, kRtQueueCapacity> rt_queue_;
    std::atomic<uint64_t> rt_head_{0};   ///< next dequeue position
    std::atomic<uint64_t> rt_tail_{0};   ///< next enqueue position
    std::atomic<uint64_t> rt_dropped_{0};
    std::atomic<uint32_t> rt_wake_gen_{0};
    std::atomic<bool>     flusher_stop_{false};
    std::atomic<bool>     rt_dead_{false};   ///< set in ~Logger
    std::once_flag        flusher_once_;
    std::thread           flusher_;

    static bool& deferredFlag() {
        static thread_local bool flag = false;
        return flag;
    }

    void ensureFlusher();
    bool enqueueRt(LogLevel level, const char* tag, std::string_view msg);
    bool dequeueRt(RtLogSlot& out);
    void drainRtQueue();
    void rtFlushLoop();
};

// Convenience macros matching ESP-IDF style (std::format-style, runtime vformat)
//
// The level is checked (one relaxed atomic load) before ANY formatting
// happens — a suppressed message costs nothing.  On deferred-marked
// realtime threads the formatted text is only copied into the bounded
// RT ring; a flusher thread emits it.
#ifndef TETHER_LOGE
#define TETHER_LOGE(tag, fmt, ...) \
    Tether::Platform::Logger::instance().logV( \
        Tether::Platform::LogLevel::Error, tag, fmt __VA_OPT__(,) __VA_ARGS__)
#endif
#ifndef TETHER_LOGW
#define TETHER_LOGW(tag, fmt, ...) \
    Tether::Platform::Logger::instance().logV( \
        Tether::Platform::LogLevel::Warn, tag, fmt __VA_OPT__(,) __VA_ARGS__)
#endif
#ifndef TETHER_LOGI
#define TETHER_LOGI(tag, fmt, ...) \
    Tether::Platform::Logger::instance().logV( \
        Tether::Platform::LogLevel::Info, tag, fmt __VA_OPT__(,) __VA_ARGS__)
#endif
#ifndef TETHER_LOGD
#define TETHER_LOGD(tag, fmt, ...) \
    Tether::Platform::Logger::instance().logV( \
        Tether::Platform::LogLevel::Debug, tag, fmt __VA_OPT__(,) __VA_ARGS__)
#endif
#ifndef TETHER_LOGV
#define TETHER_LOGV(tag, fmt, ...) \
    Tether::Platform::Logger::instance().logV( \
        Tether::Platform::LogLevel::Verbose, tag, fmt __VA_OPT__(,) __VA_ARGS__)
#endif

// Compile-time validated std::format variants. These give compile-time
// format checking and better optimization for simple call sites, but they
// cannot be used inside lambdas that are stored in std::function (because
// std::format's format_string constructor is consteval and would cause
// immediate-escalation of the lambda's operator()).
#ifndef TETHER_LOGE_FMT
#define TETHER_LOGE_FMT(tag, fmt, ...) \
    Tether::Platform::Logger::instance().logF( \
        Tether::Platform::LogLevel::Error, tag, fmt __VA_OPT__(,) __VA_ARGS__)
#endif
#ifndef TETHER_LOGW_FMT
#define TETHER_LOGW_FMT(tag, fmt, ...) \
    Tether::Platform::Logger::instance().logF( \
        Tether::Platform::LogLevel::Warn, tag, fmt __VA_OPT__(,) __VA_ARGS__)
#endif
#ifndef TETHER_LOGI_FMT
#define TETHER_LOGI_FMT(tag, fmt, ...) \
    Tether::Platform::Logger::instance().logF( \
        Tether::Platform::LogLevel::Info, tag, fmt __VA_OPT__(,) __VA_ARGS__)
#endif
#ifndef TETHER_LOGD_FMT
#define TETHER_LOGD_FMT(tag, fmt, ...) \
    Tether::Platform::Logger::instance().logF( \
        Tether::Platform::LogLevel::Debug, tag, fmt __VA_OPT__(,) __VA_ARGS__)
#endif
#ifndef TETHER_LOGV_FMT
#define TETHER_LOGV_FMT(tag, fmt, ...) \
    Tether::Platform::Logger::instance().logF( \
        Tether::Platform::LogLevel::Verbose, tag, fmt __VA_OPT__(,) __VA_ARGS__)
#endif

static inline void log_buffer_hex(const char* tag, const void* buffer, size_t len, LogLevel lvl) {
    const uint8_t* b = static_cast<const uint8_t*>(buffer);
    const size_t per_line = 16;
    for (size_t off = 0; off < len; off += per_line) {
        size_t chunk = std::min(per_line, len - off);
        std::string s;
        s.reserve(chunk * 3);
        for (size_t i = 0; i < chunk; ++i) {
            char tmp[4];
            std::snprintf(tmp, sizeof(tmp), "%02X ", b[off + i]);
            s += tmp;
        }
        switch (lvl) {
            case LogLevel::Error: TETHER_LOGE(tag, "{}", s); break;
            case LogLevel::Warn:  TETHER_LOGW(tag, "{}", s); break;
            case LogLevel::Info:  TETHER_LOGI(tag, "{}", s); break;
            case LogLevel::Debug: TETHER_LOGD(tag, "{}", s); break;
            case LogLevel::Verbose: TETHER_LOGV(tag, "{}", s); break;
            default: TETHER_LOGD(tag, "{}", s); break;
        }
    }
}

static inline void log_buffer_hex_level(const char* tag, const void* buffer, size_t len, int level) {
    LogLevel lvl = LogLevel::Debug;
    switch (level) {
        case 1: lvl = LogLevel::Error; break;
        case 2: lvl = LogLevel::Warn; break;
        case 3: lvl = LogLevel::Info; break;
        case 4: lvl = LogLevel::Debug; break;
        case 5: lvl = LogLevel::Verbose; break;
        default: lvl = LogLevel::Debug; break;
    }
    log_buffer_hex(tag, buffer, len, lvl);
}

#ifndef TETHER_LOG_BUFFER_HEX
#define TETHER_LOG_BUFFER_HEX(tag, buffer, len) Tether::Platform::log_buffer_hex(tag, buffer, len, Tether::Platform::LogLevel::Debug)
#endif
#ifndef TETHER_LOG_BUFFER_HEX_LEVEL
#define TETHER_LOG_BUFFER_HEX_LEVEL(tag, buffer, len, level) Tether::Platform::log_buffer_hex_level(tag, buffer, len, level)
#endif

}  // namespace Platform
}  // namespace Tether

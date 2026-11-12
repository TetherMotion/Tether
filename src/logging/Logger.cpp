#include "logging/Logger.hpp"

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <ctime>
#include <chrono>
#include <mutex>
#include <algorithm>
#include <vector>
#include <unistd.h>

#include "tether/platform/AtomicWait.hpp"

// Determine whether to use ANSI color codes for console output. Colors are
// enabled when stdout is a TTY and the NO_COLOR environment variable is not set.
static inline int tether_platform_use_color(void) {
    static int enabled = 0;
    static std::once_flag initFlag;
    std::call_once(initFlag, [&]() {
        // Force color if TETHER_FORCE_COLOR is set, otherwise require a TTY and no NO_COLOR.
        enabled = ((isatty(fileno(stdout)) || (getenv("TETHER_FORCE_COLOR") != NULL)) && (getenv("NO_COLOR") == NULL));
    });
    return enabled;
}

namespace Tether {
namespace Platform {

Logger& Logger::instance() {
    static Logger logger;
    return logger;
}

Logger::Logger() {
    // Vyukov ring: each slot's sequence starts at its own index.
    for (uint32_t i = 0; i < kRtQueueCapacity; ++i) {
        rt_queue_[i].seq.store(i, std::memory_order_relaxed);
    }
}

Logger::~Logger() {
    rt_dead_.store(true, std::memory_order_release);
    if (flusher_.joinable()) {
        flusher_stop_.store(true, std::memory_order_release);
        rt_wake_gen_.fetch_add(1, std::memory_order_release);
        atomicWakeAll(&rt_wake_gen_);
        flusher_.join();
    }
}

void Logger::setThreadDeferred(bool en) {
    deferredFlag() = en;
    if (en) instance().ensureFlusher();
}

bool Logger::threadDeferred() {
    return deferredFlag();
}

void Logger::ensureFlusher() {
    std::call_once(flusher_once_, [this] {
        flusher_ = std::thread([this] { rtFlushLoop(); });
    });
}

bool Logger::enqueueRt(LogLevel level, const char* tag,
                       std::string_view msg) {
    if (rt_dead_.load(std::memory_order_relaxed)) {
        rt_dropped_.fetch_add(1, std::memory_order_relaxed);
        return false;
    }
    ensureFlusher();

    RtLogSlot* cell;
    uint64_t pos = rt_tail_.load(std::memory_order_relaxed);
    for (;;) {
        cell = &rt_queue_[pos & (kRtQueueCapacity - 1)];
        const uint64_t seq = cell->seq.load(std::memory_order_acquire);
        const intptr_t dif =
            static_cast<intptr_t>(seq) - static_cast<intptr_t>(pos);
        if (dif == 0) {
            if (rt_tail_.compare_exchange_weak(
                    pos, pos + 1, std::memory_order_relaxed))
                break;
        } else if (dif < 0) {
            // Ring full — bounded drop instead of blocking or
            // overwriting an unconsumed message.
            rt_dropped_.fetch_add(1, std::memory_order_relaxed);
            return false;
        } else {
            pos = rt_tail_.load(std::memory_order_relaxed);
        }
    }

    cell->level = level;
    const size_t tn =
        tag ? std::min(strlen(tag), static_cast<size_t>(kRtTagSize - 1))
            : 0;
    if (tn) std::memcpy(cell->tag, tag, tn);
    cell->tag[tn] = '\0';
    const size_t mn =
        std::min(msg.size(), static_cast<size_t>(kRtTextSize));
    if (mn) std::memcpy(cell->text, msg.data(), mn);
    cell->text_len = static_cast<uint32_t>(mn);
    cell->seq.store(pos + 1, std::memory_order_release);

    rt_wake_gen_.fetch_add(1, std::memory_order_release);
    atomicWakeAll(&rt_wake_gen_);
    return true;
}

bool Logger::dequeueRt(RtLogSlot& out) {
    RtLogSlot* cell;
    uint64_t pos = rt_head_.load(std::memory_order_relaxed);
    for (;;) {
        cell = &rt_queue_[pos & (kRtQueueCapacity - 1)];
        const uint64_t seq = cell->seq.load(std::memory_order_acquire);
        const intptr_t dif =
            static_cast<intptr_t>(seq) - static_cast<intptr_t>(pos + 1);
        if (dif == 0) {
            if (rt_head_.compare_exchange_weak(
                    pos, pos + 1, std::memory_order_relaxed))
                break;
        } else if (dif < 0) {
            return false;   // empty
        } else {
            pos = rt_head_.load(std::memory_order_relaxed);
        }
    }
    out.level = cell->level;
    std::memcpy(out.tag, cell->tag, kRtTagSize);
    out.text_len = cell->text_len;
    if (out.text_len)
        std::memcpy(out.text, cell->text, out.text_len);
    cell->seq.store(pos + kRtQueueCapacity, std::memory_order_release);
    return true;
}

void Logger::drainRtQueue() {
    RtLogSlot s;
    while (dequeueRt(s)) {
        logFormatted(s.level, s.tag,
                     std::string_view(s.text, s.text_len));
    }
    const uint64_t d =
        rt_dropped_.exchange(0, std::memory_order_relaxed);
    if (d) {
        char b[96];
        const int n = std::snprintf(
            b, sizeof(b),
            "rt_log: %llu deferred log message(s) dropped (queue full)",
            static_cast<unsigned long long>(d));
        logFormatted(LogLevel::Error, "rt_log",
                     std::string_view(b, static_cast<size_t>(n)));
    }
}

void Logger::rtFlushLoop() {
    for (;;) {
        drainRtQueue();
        if (flusher_stop_.load(std::memory_order_acquire)) {
            drainRtQueue();   // last drain before exit
            return;
        }
        // Wake-stamped wait: enqueueRt bumps rt_wake_gen_ after
        // publishing a slot, so a missed dequeue can only delay a
        // message by this poll bound.
        const uint32_t g = rt_wake_gen_.load(std::memory_order_acquire);
        atomicWait(&rt_wake_gen_, g, 250'000'000);
    }
}

void Logger::flushDeferred() {
    drainRtQueue();
}

void Logger::log(LogLevel level, const char* tag, const char* format, ...) {
    va_list args;
    va_start(args, format);
    logv(level, tag, format, args);
    va_end(args);
}

void Logger::logv(LogLevel level, const char* tag, const char* format, va_list args) {
    // Format the message into a string (handle arbitrarily long messages)
    va_list args_copy;
    va_copy(args_copy, args);
    char smallbuf[512];
    int needed = vsnprintf(smallbuf, sizeof(smallbuf), format, args_copy);
    va_end(args_copy);

    std::string msg;
    if (needed < 0) {
        msg = "";
    } else if ((size_t)needed < sizeof(smallbuf)) {
        msg.assign(smallbuf, (size_t)needed);
    } else {
        msg.resize((size_t)needed);
        va_list args_copy2;
        va_copy(args_copy2, args);
        vsnprintf(&msg[0], msg.size() + 1, format, args_copy2);
        va_end(args_copy2);
    }

    logFormatted(level, tag, msg);
}

void Logger::logFormatted(LogLevel level, const char* tag, std::string_view message) {
    // Realtime/deferred threads never run this path inline: format is
    // already done (bounded, on the producer's stack), so we only copy
    // the text into the RT ring and return — no heap, no mutex, no I/O.
    if (deferredFlag()) {
        enqueueRt(level, tag, message);
        return;
    }
    // Filtered messages bail before touching mutex_ or copying handlers.
    if (!enabled(level)) return;

    LogHandler primaryHandler;
    std::vector<LogHandler> handlers;
    bool timestampEnabled;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        primaryHandler = handler_;
        timestampEnabled = timestampEnabled_;
        handlers.reserve(handlers_.size());
        for (const auto& [id, registered] : handlers_) handlers.push_back(registered);
    }

    std::string msg(message);

    // Split on '\n' and remove a trailing empty line if the message ends with a newline
    std::vector<std::string> lines;
    size_t start = 0;
    while (start <= msg.size()) {
        size_t pos = msg.find('\n', start);
        if (pos == std::string::npos) pos = msg.size();
        lines.emplace_back(msg.substr(start, pos - start));
        if (pos == msg.size()) break;
        start = pos + 1;
    }
    if (!lines.empty() && msg.size() > 0 && msg.back() == '\n') {
        // drop trailing empty segment produced by a trailing '\n'
        if (!lines.empty() && lines.back().empty()) lines.pop_back();
    }

    if (primaryHandler) {
        for (const auto& line : lines) {
            primaryHandler(level, tag, line.c_str());
        }
    }
    for (const auto& registered : handlers) {
        for (const auto& line : lines) {
            registered(level, tag, line.c_str());
        }
    }
    if (primaryHandler || !handlers.empty()) return;

    // Default printf-based output with optional ANSI color support; print prefix per line
    const char* levelStr = "?";
    switch (level) {
        case LogLevel::Error:   levelStr = "E"; break;
        case LogLevel::Warn:    levelStr = "W"; break;
        case LogLevel::Info:    levelStr = "I"; break;
        case LogLevel::Debug:   levelStr = "D"; break;
        case LogLevel::Verbose: levelStr = "V"; break;
        default: break;
    }

    const char* color_start = "";
    const char* color_end = "";
    if (tether_platform_use_color()) {
        switch (level) {
            case LogLevel::Error:   color_start = "\x1b[1;31m"; break; // bright red
            case LogLevel::Warn:    color_start = "\x1b[1;33m"; break; // bright yellow
            case LogLevel::Info:    color_start = "\x1b[1;32m"; break; // bright green
            case LogLevel::Debug:   color_start = "\x1b[1;36m"; break; // bright cyan
            case LogLevel::Verbose: color_start = "\x1b[1;35m"; break; // bright magenta
            default: break;
        }
        color_end = "\x1b[0m";
    }

    for (const auto& line : lines) {
        // Optional ISO8601-like UTC timestamp with millisecond precision
        char timebuf[64] = "";
        if (timestampEnabled) {
            using namespace std::chrono;
            auto now = system_clock::now();
            auto ms = duration_cast<milliseconds>(now.time_since_epoch()) % 1000;
            std::time_t secs = system_clock::to_time_t(now);
            struct tm tm;
            gmtime_r(&secs, &tm); // produce UTC time
            std::snprintf(timebuf, sizeof(timebuf), "%04d-%02d-%02dT%02d:%02d:%02d.%03dZ",
                          tm.tm_year + 1900, tm.tm_mon + 1, tm.tm_mday,
                          tm.tm_hour, tm.tm_min, tm.tm_sec,
                          static_cast<int>(ms.count()));
        }

        if (color_start[0]) {
            if (timebuf[0]) {
                // keep timestamp uncolored, color only the level field
                printf("%s %s[%s]%s %s: %s\n", timebuf, color_start, levelStr, color_end, tag, line.c_str());
            } else {
                printf("%s[%s]%s %s: %s\n", color_start, levelStr, color_end, tag, line.c_str());
            }
        } else {
            if (timebuf[0]) {
                printf("%s [%s] %s: %s\n", timebuf, levelStr, tag, line.c_str());
            } else {
                printf("[%s] %s: %s\n", levelStr, tag, line.c_str());
            }
        }
    }
}

Logger::HandlerId Logger::addHandler(LogHandler handler) {
    std::lock_guard<std::mutex> lock(mutex_);
    const HandlerId id = nextHandlerId_++;
    handlers_.emplace_back(id, std::move(handler));
    return id;
}

void Logger::removeHandler(HandlerId id) {
    std::lock_guard<std::mutex> lock(mutex_);
    handlers_.erase(std::remove_if(handlers_.begin(), handlers_.end(),
                                   [id](const auto& item) { return item.first == id; }),
                    handlers_.end());
}

}  // namespace Platform
}  // namespace Tether

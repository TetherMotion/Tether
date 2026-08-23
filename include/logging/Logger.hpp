#pragma once

#include <cstdint>
#include <cstdarg>
#include <cstdio>
#include <functional>
#include <string>
#include <algorithm>
#include <mutex>
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

//=============================================================================
// Logging Interface
//=============================================================================

/**
 * @brief Platform-independent logging interface
 *
 * Default implementation uses printf. Can be overridden for ESP-IDF, etc.
 */
class Logger {
public:
    static Logger& instance();

    void setLevel(LogLevel level) {
        std::lock_guard<std::mutex> lock(mutex_);
        level_ = level;
    }
    LogLevel getLevel() const {
        std::lock_guard<std::mutex> lock(mutex_);
        return level_;
    }

    void log(LogLevel level, const char* tag, const char* format, ...);
    void logv(LogLevel level, const char* tag, const char* format, va_list args);

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

private:
    Logger() = default;
    LogLevel level_ = LogLevel::Info;
    LogHandler handler_;
    std::vector<std::pair<HandlerId, LogHandler>> handlers_;
    HandlerId nextHandlerId_ = 1;
    bool timestampEnabled_ = true;
    mutable std::mutex mutex_;
};

// Convenience macros matching ESP-IDF style
#ifndef TETHER_LOGE
#define TETHER_LOGE(tag, format, ...) \
    Tether::Platform::Logger::instance().log(Tether::Platform::LogLevel::Error, tag, format, ##__VA_ARGS__)
#endif
#ifndef TETHER_LOGW
#define TETHER_LOGW(tag, format, ...) \
    Tether::Platform::Logger::instance().log(Tether::Platform::LogLevel::Warn, tag, format, ##__VA_ARGS__)
#endif
#ifndef TETHER_LOGI
#define TETHER_LOGI(tag, format, ...) \
    Tether::Platform::Logger::instance().log(Tether::Platform::LogLevel::Info, tag, format, ##__VA_ARGS__)
#endif
#ifndef TETHER_LOGD
#define TETHER_LOGD(tag, format, ...) \
    Tether::Platform::Logger::instance().log(Tether::Platform::LogLevel::Debug, tag, format, ##__VA_ARGS__)
#endif
#ifndef TETHER_LOGV
#define TETHER_LOGV(tag, format, ...) \
    Tether::Platform::Logger::instance().log(Tether::Platform::LogLevel::Verbose, tag, format, ##__VA_ARGS__)
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
            case LogLevel::Error: TETHER_LOGE(tag, "%s", s.c_str()); break;
            case LogLevel::Warn:  TETHER_LOGW(tag, "%s", s.c_str()); break;
            case LogLevel::Info:  TETHER_LOGI(tag, "%s", s.c_str()); break;
            case LogLevel::Debug: TETHER_LOGD(tag, "%s", s.c_str()); break;
            case LogLevel::Verbose: TETHER_LOGV(tag, "%s", s.c_str()); break;
            default: TETHER_LOGD(tag, "%s", s.c_str()); break;
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

/**
 * @file KlipperLog.cpp
 * @brief Implementation of the Klipper module logging.
 */

#include "tether/klipper/KlipperLog.hpp"

#include <atomic>
#include <cstdio>
#include <mutex>
#include <string>

namespace tether::klipper::logging {

namespace {
std::atomic<Level> g_logLevel{Level::Warn};
LogCallback g_callback;
std::mutex g_mutex;

const char* levelStr(Level l) {
    switch (l) {
        case Level::Error: return "ERROR";
        case Level::Warn:  return "WARN";
        case Level::Info:  return "INFO";
        case Level::Debug: return "DEBUG";
    }
    return "?";
}
} // namespace

void setLogCallback(LogCallback cb) {
    std::lock_guard<std::mutex> lock(g_mutex);
    g_callback = std::move(cb);
}

LogCallback getLogCallback() {
    std::lock_guard<std::mutex> lock(g_mutex);
    return g_callback;
}

void setLogLevel(Level level) {
    g_logLevel.store(level);
}

Level getLogLevel() {
    return g_logLevel.load();
}

void emit(Level level, std::string_view file, int line, std::string_view msg) {
    if (static_cast<uint8_t>(level) > static_cast<uint8_t>(g_logLevel.load()))
        return;

    // Extract basename from file path.
    std::string_view basename = file;
    if (auto pos = basename.rfind('/'); pos != std::string_view::npos)
        basename = basename.substr(pos + 1);

    std::lock_guard<std::mutex> lock(g_mutex);
    if (g_callback) {
        g_callback(level, basename, line, msg);
    } else {
        // Default: stderr
        std::fprintf(stderr, "[Klipper %s %.*s:%d] %.*s\n",
                     levelStr(level),
                     static_cast<int>(basename.size()), basename.data(),
                     line,
                     static_cast<int>(msg.size()), msg.data());
    }
}

} // namespace tether::klipper::logging

/**
 * @file KlipperLog.hpp
 * @brief Lightweight logging for the Klipper module.
 *
 * @details
 * Provides KLIPPER_LOG_ERROR, KLIPPER_LOG_WARN, and KLIPPER_LOG_INFO macros
 * that log to stderr by default. Logging can be disabled at compile time by
 * defining KLIPPER_DISABLE_LOGGING, or redirected at runtime by setting a
 * custom log callback.
 *
 * This is intentionally minimal - it does not depend on any external logging
 * library and adds zero overhead when disabled.
 */

#pragma once

#include <cstdio>
#include <functional>
#include <string>
#include <string_view>

namespace tether::klipper::logging {

/// @brief Log level.
enum class Level {
    Error = 0,
    Warn  = 1,
    Info  = 2,
    Debug = 3,
};

/// @brief Log callback type. Receives level, file, line, and message.
using LogCallback = std::function<void(Level, std::string_view, int, std::string_view)>;

/// @brief Set a custom log callback. Pass nullptr to restore default (stderr).
void setLogCallback(LogCallback cb);

/// @brief Get the current log callback (or nullptr for default).
LogCallback getLogCallback();

/// @brief Set the minimum log level. Messages below this level are dropped.
void setLogLevel(Level level);

/// @brief Get the current minimum log level.
Level getLogLevel();

/// @brief Internal: emit a log message.
void emit(Level level, std::string_view file, int line, std::string_view msg);

} // namespace tether::klipper::logging

/// @brief Log an error message.
#define KLIPPER_LOG_ERROR(msg) \
    ::tether::klipper::logging::emit(::tether::klipper::logging::Level::Error, __FILE__, __LINE__, msg)

/// @brief Log a warning message.
#define KLIPPER_LOG_WARN(msg) \
    ::tether::klipper::logging::emit(::tether::klipper::logging::Level::Warn, __FILE__, __LINE__, msg)

/// @brief Log an info message.
#define KLIPPER_LOG_INFO(msg) \
    ::tether::klipper::logging::emit(::tether::klipper::logging::Level::Info, __FILE__, __LINE__, msg)

#ifdef KLIPPER_ENABLE_DEBUG_LOG
/// @brief Log a debug message (only when KLIPPER_ENABLE_DEBUG_LOG is defined).
#define KLIPPER_LOG_DEBUG(msg) \
    ::tether::klipper::logging::emit(::tether::klipper::logging::Level::Debug, __FILE__, __LINE__, msg)
#else
#define KLIPPER_LOG_DEBUG(msg) ((void)0)
#endif

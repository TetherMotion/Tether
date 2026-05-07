/**
 * @file ILogger.hpp
 * @brief Logging abstraction interface
 *
 * Provides a platform-independent logging interface with support for
 * different log levels and optional output destinations.
 */

#pragma once

#include "hal/HALTypes.hpp"
#include <cstdarg>
#include <memory>

namespace EtherCAT {
namespace HAL {

// ============================================================================
// Log Levels
// ============================================================================

/**
 * @brief Log severity levels
 */
enum class LogLevel {
    None = 0,       ///< Logging disabled
    Error,          ///< Error conditions
    Warn,           ///< Warning conditions
    Info,           ///< Informational messages
    Debug,          ///< Debug messages
    Verbose,        ///< Verbose debug messages
};

/**
 * @brief Convert log level to string
 */
inline const char* logLevelToString(LogLevel level) {
    switch (level) {
        case LogLevel::None:    return "NONE";
        case LogLevel::Error:   return "ERROR";
        case LogLevel::Warn:    return "WARN";
        case LogLevel::Info:    return "INFO";
        case LogLevel::Debug:   return "DEBUG";
        case LogLevel::Verbose: return "VERBOSE";
        default: return "?";
    }
}

/**
 * @brief Convert log level to single character
 */
inline char logLevelToChar(LogLevel level) {
    switch (level) {
        case LogLevel::None:    return 'N';
        case LogLevel::Error:   return 'E';
        case LogLevel::Warn:    return 'W';
        case LogLevel::Info:    return 'I';
        case LogLevel::Debug:   return 'D';
        case LogLevel::Verbose: return 'V';
        default: return '?';
    }
}

// ============================================================================
// Logger Interface
// ============================================================================

/**
 * @brief Abstract logger interface
 */
class ILogger {
public:
    virtual ~ILogger() = default;
    
    /**
     * @brief Log a message
     * @param level Log level
     * @param tag Component tag
     * @param format printf-style format string
     * @param args Variable arguments
     */
    virtual void log(LogLevel level, const char* tag, const char* format, va_list args) = 0;
    
    /**
     * @brief Log with format string (convenience)
     */
    void logf(LogLevel level, const char* tag, const char* format, ...) {
        va_list args;
        va_start(args, format);
        log(level, tag, format, args);
        va_end(args);
    }
    
    /**
     * @brief Error level log
     */
    void error(const char* tag, const char* format, ...) {
        va_list args;
        va_start(args, format);
        log(LogLevel::Error, tag, format, args);
        va_end(args);
    }
    
    /**
     * @brief Warning level log
     */
    void warn(const char* tag, const char* format, ...) {
        va_list args;
        va_start(args, format);
        log(LogLevel::Warn, tag, format, args);
        va_end(args);
    }
    
    /**
     * @brief Info level log
     */
    void info(const char* tag, const char* format, ...) {
        va_list args;
        va_start(args, format);
        log(LogLevel::Info, tag, format, args);
        va_end(args);
    }
    
    /**
     * @brief Debug level log
     */
    void debug(const char* tag, const char* format, ...) {
        va_list args;
        va_start(args, format);
        log(LogLevel::Debug, tag, format, args);
        va_end(args);
    }
    
    /**
     * @brief Verbose level log
     */
    void verbose(const char* tag, const char* format, ...) {
        va_list args;
        va_start(args, format);
        log(LogLevel::Verbose, tag, format, args);
        va_end(args);
    }
    
    /**
     * @brief Set minimum log level
     */
    virtual void setLevel(LogLevel level) = 0;
    
    /**
     * @brief Get current log level
     */
    virtual LogLevel getLevel() const = 0;
    
    /**
     * @brief Set log level for specific tag
     */
    virtual void setTagLevel(const char* tag, LogLevel level) = 0;
    
    /**
     * @brief Enable/disable timestamps
     */
    virtual void setTimestamps(bool enable) = 0;
    
    /**
     * @brief Enable/disable colors (ANSI escape codes)
     */
    virtual void setColors(bool enable) = 0;
    
    /**
     * @brief Flush any buffered output
     */
    virtual void flush() = 0;
};

// ============================================================================
// Log Output Sink Interface
// ============================================================================

/**
 * @brief Log output sink interface
 */
class ILogSink {
public:
    virtual ~ILogSink() = default;
    
    /**
     * @brief Write formatted log message
     * @param level Log level
     * @param tag Component tag
     * @param timestamp Timestamp in microseconds
     * @param message Formatted message
     */
    virtual void write(LogLevel level, const char* tag, 
                       Timestamp timestamp, const char* message) = 0;
    
    /**
     * @brief Flush output
     */
    virtual void flush() = 0;
};

// ============================================================================
// Factory Functions
// ============================================================================

/**
 * @brief Get the global logger instance
 */
ILogger& getLogger();

/**
 * @brief Set custom logger
 */
void setLogger(std::unique_ptr<ILogger> logger);

/**
 * @brief Create platform-appropriate logger
 */
std::unique_ptr<ILogger> createDefaultLogger();

/**
 * @brief Create console logger
 */
std::unique_ptr<ILogger> createConsoleLogger();

/**
 * @brief Create file logger
 */
std::unique_ptr<ILogger> createFileLogger(const char* filename);

/**
 * @brief Create null logger (discards all output)
 */
std::unique_ptr<ILogger> createNullLogger();

// ============================================================================
// Logging Macros
// ============================================================================

#ifndef HAL_LOG_TAG
#define HAL_LOG_TAG "HAL"
#endif

#define HAL_LOGE(format, ...) EtherCAT::HAL::getLogger().error(HAL_LOG_TAG, format, ##__VA_ARGS__)
#define HAL_LOGW(format, ...) EtherCAT::HAL::getLogger().warn(HAL_LOG_TAG, format, ##__VA_ARGS__)
#define HAL_LOGI(format, ...) EtherCAT::HAL::getLogger().info(HAL_LOG_TAG, format, ##__VA_ARGS__)
#define HAL_LOGD(format, ...) EtherCAT::HAL::getLogger().debug(HAL_LOG_TAG, format, ##__VA_ARGS__)
#define HAL_LOGV(format, ...) EtherCAT::HAL::getLogger().verbose(HAL_LOG_TAG, format, ##__VA_ARGS__)

} // namespace HAL
} // namespace EtherCAT

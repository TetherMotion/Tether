/**
 * @file SlaveLogger.hpp
 * @brief Logging infrastructure for EtherCAT slave
 *
 * @details
 * Provides comprehensive logging for slave operations with:
 * - Category-based filtering (state machine, FMMU, SM, DC, mailbox, etc.)
 * - Realtime-safe logging via lock-free queue
 * - PcapNG support for frame logging (shares code with master)
 * - Optional verbose mode for debugging
 *
 * The logging system is designed to have minimal impact on realtime
 * performance when disabled or configured for minimal logging.
 */

#pragma once

#include "packetloggers/PacketLogger.hpp"
#include "packetloggers/pcap/PCAPLoggerConfig.hpp"
#include "slave/core/SlaveTypes.hpp"

#include <atomic>
#include <cstdarg>
#include <cstdint>
#include <cstdio>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

namespace EtherCAT {
namespace slave {

// ============================================================================
// Log Categories
// ============================================================================

/**
 * @brief Logging categories for slave operations
 */
enum class SlaveLogCategory : uint32_t {
    None           = 0x00000000,
    
    // Core operations
    StateMachine   = 0x00000001,  ///< AL state machine transitions
    Register       = 0x00000002,  ///< ESC register access
    FMMU           = 0x00000004,  ///< FMMU configuration and translation
    SyncManager    = 0x00000008,  ///< Sync Manager operations
    DC             = 0x00000010,  ///< Distributed Clock
    Watchdog       = 0x00000020,  ///< Watchdog events
    SII            = 0x00000040,  ///< SII/EEPROM access
    
    // Frame processing
    FrameRx        = 0x00000100,  ///< Received frames
    FrameTx        = 0x00000200,  ///< Transmitted frames
    Datagram       = 0x00000400,  ///< Individual datagram processing
    LogicalAddr    = 0x00000800,  ///< Logical address translation
    
    // Mailbox
    Mailbox        = 0x00001000,  ///< General mailbox
    CoE            = 0x00002000,  ///< CANopen over EtherCAT
    FoE            = 0x00004000,  ///< File over EtherCAT
    EoE            = 0x00008000,  ///< Ethernet over EtherCAT
    VoE            = 0x00010000,  ///< Vendor over EtherCAT
    SoE            = 0x00020000,  ///< Servo over EtherCAT
    AoE            = 0x00040000,  ///< ADS over EtherCAT
    
    // Process data
    PDO            = 0x00100000,  ///< PDO exchange
    TxPDO          = 0x00200000,  ///< TxPDO (slave → master)
    RxPDO          = 0x00400000,  ///< RxPDO (master → slave)
    
    // Profile specific
    CiA401         = 0x01000000,  ///< CiA 401 I/O profile
    CiA402         = 0x02000000,  ///< CiA 402 drive profile
    
    // Debug
    Verbose        = 0x40000000,  ///< Verbose output
    Trace          = 0x80000000,  ///< Trace level (very verbose)
    
    // Common combinations
    All            = 0xFFFFFFFF,
    Default        = StateMachine | Mailbox | CoE | FoE,
    Minimal        = StateMachine,
    Debug          = Default | FMMU | SyncManager | DC | Datagram,
    Frames         = FrameRx | FrameTx,
    AllMailbox     = Mailbox | CoE | FoE | EoE | VoE | SoE | AoE,
};

inline SlaveLogCategory operator|(SlaveLogCategory a, SlaveLogCategory b) {
    return static_cast<SlaveLogCategory>(
        static_cast<uint32_t>(a) | static_cast<uint32_t>(b));
}

inline SlaveLogCategory operator&(SlaveLogCategory a, SlaveLogCategory b) {
    return static_cast<SlaveLogCategory>(
        static_cast<uint32_t>(a) & static_cast<uint32_t>(b));
}

inline SlaveLogCategory operator~(SlaveLogCategory a) {
    return static_cast<SlaveLogCategory>(~static_cast<uint32_t>(a));
}

// ============================================================================
// Log Level
// ============================================================================

/**
 * @brief Log severity levels
 */
enum class SlaveLogLevel : uint8_t {
    Trace    = 0,  ///< Very detailed trace info
    Debug    = 1,  ///< Debug information
    Info     = 2,  ///< Informational messages
    Warning  = 3,  ///< Warnings
    Error    = 4,  ///< Errors
    Critical = 5,  ///< Critical errors
    None     = 6,  ///< Disable all logging
};

// ============================================================================
// Log Entry
// ============================================================================

/**
 * @brief Single log entry
 */
struct SlaveLogEntry {
    uint64_t timestamp = 0;           ///< Timestamp (ns)
    SlaveLogCategory category;        ///< Log category
    SlaveLogLevel level;              ///< Log level
    uint16_t slaveAddress = 0;        ///< Slave address (for identification)
    char message[256];                ///< Log message
    
    // Optional binary data (for frame logging)
    uint8_t data[64];
    size_t dataLen = 0;
};

// ============================================================================
// Log Configuration
// ============================================================================

/**
 * @brief Logger configuration
 */
struct SlaveLogConfig {
    SlaveLogCategory enabledCategories = SlaveLogCategory::Default;
    SlaveLogLevel minLevel = SlaveLogLevel::Info;
    
    // Console output
    bool consoleEnabled = true;
    bool consoleColors = true;
    
    // File output
    bool fileEnabled = false;
    std::string logFilePath;
    size_t maxFileSize = 10 * 1024 * 1024;  // 10MB
    int maxRotatedFiles = 5;
    
    // PCAP output (frame logging)
    bool pcapEnabled = false;
    Tether::PacketLoggers::PCAP::PCAPLoggerConfig pcapConfig;
    
    // Realtime queue
    size_t queueSize = 1024;  ///< Number of entries in RT queue
    
    // Custom output callback
    using OutputCallback = std::function<void(const SlaveLogEntry&)>;
    OutputCallback customCallback;
};

// ============================================================================
// Slave Logger Class
// ============================================================================

/**
 * @brief Realtime-safe logger for EtherCAT slave
 *
 * The logger uses a lock-free queue for realtime safety. Log calls
 * from the realtime thread are non-blocking and simply enqueue entries.
 * A background thread processes the queue and writes to outputs.
 *
 * Usage:
 * @code
 * SlaveLogConfig config;
 * config.enabledCategories = SlaveLogCategory::StateMachine | SlaveLogCategory::CoE;
 * config.minLevel = SlaveLogLevel::Debug;
 *
 * SlaveLogger logger(config);
 * logger.start();
 *
 * // From any thread (including realtime)
 * logger.log(SlaveLogCategory::StateMachine, SlaveLogLevel::Info,
 *            "State changed: %s -> %s", oldState, newState);
 *
 * // Log a frame
 * logger.logFrame(SlaveLogCategory::FrameRx, frameData, frameLen, "Received");
 * @endcode
 */
class SlaveLogger {
public:
    /**
     * @brief Constructor
     * @param config Logger configuration
     */
    explicit SlaveLogger(const SlaveLogConfig& config = {});
    
    /**
     * @brief Destructor
     */
    ~SlaveLogger();
    
    // Non-copyable
    SlaveLogger(const SlaveLogger&) = delete;
    SlaveLogger& operator=(const SlaveLogger&) = delete;
    
    // ========================================================================
    // Control
    // ========================================================================
    
    /**
     * @brief Start the background processing thread
     */
    void start();
    
    /**
     * @brief Stop the background thread and flush pending entries
     */
    void stop();
    
    /**
     * @brief Check if logger is running
     */
    bool isRunning() const { return running_.load(); }
    
    /**
     * @brief Flush all pending log entries
     */
    void flush();
    
    // ========================================================================
    // Configuration
    // ========================================================================
    
    /**
     * @brief Set enabled categories
     */
    void setEnabledCategories(SlaveLogCategory categories);
    
    /**
     * @brief Enable/disable a specific category
     */
    void setCategoryEnabled(SlaveLogCategory category, bool enabled);
    
    /**
     * @brief Check if category is enabled
     */
    bool isCategoryEnabled(SlaveLogCategory category) const;
    
    /**
     * @brief Set minimum log level
     */
    void setMinLevel(SlaveLogLevel level);
    
    /**
     * @brief Get minimum log level
     */
    SlaveLogLevel getMinLevel() const { return config_.minLevel; }
    
    /**
     * @brief Set slave address (for identification in logs)
     */
    void setSlaveAddress(uint16_t addr) { slaveAddress_ = addr; }
    
    // ========================================================================
    // Logging Methods (Realtime-Safe)
    // ========================================================================
    
    /**
     * @brief Log a message
     *
     * This method is realtime-safe and can be called from any thread.
     *
     * @param category Log category
     * @param level Log level
     * @param format Printf-style format string
     * @param ... Format arguments
     */
    void log(SlaveLogCategory category, SlaveLogLevel level,
             const char* format, ...) __attribute__((format(printf, 4, 5)));
    
    /**
     * @brief Log a message with va_list
     */
    void logv(SlaveLogCategory category, SlaveLogLevel level,
              const char* format, va_list args);
    
    /**
     * @brief Log a frame with optional description
     *
     * @param category Log category
     * @param frame Frame data
     * @param length Frame length
     * @param description Optional description
     */
    void logFrame(SlaveLogCategory category,
                  const uint8_t* frame, size_t length,
                  const char* description = nullptr);
    
    /**
     * @brief Log binary data as hex
     *
     * @param category Log category
     * @param level Log level
     * @param data Binary data
     * @param length Data length
     * @param prefix Optional prefix
     */
    void logHex(SlaveLogCategory category, SlaveLogLevel level,
                const uint8_t* data, size_t length,
                const char* prefix = nullptr);
    
    // ========================================================================
    // Convenience Methods
    // ========================================================================
    
    void trace(SlaveLogCategory cat, const char* fmt, ...)
        __attribute__((format(printf, 3, 4)));
    
    void debug(SlaveLogCategory cat, const char* fmt, ...)
        __attribute__((format(printf, 3, 4)));
    
    void info(SlaveLogCategory cat, const char* fmt, ...)
        __attribute__((format(printf, 3, 4)));
    
    void warn(SlaveLogCategory cat, const char* fmt, ...)
        __attribute__((format(printf, 3, 4)));
    
    void error(SlaveLogCategory cat, const char* fmt, ...)
        __attribute__((format(printf, 3, 4)));
    
    void critical(SlaveLogCategory cat, const char* fmt, ...)
        __attribute__((format(printf, 3, 4)));
    
    // ========================================================================
    // PcapNG Logging
    // ========================================================================
    
    /**
     * @brief Log frame to PcapNG file
     *
     * @param frame Frame data
     * @param length Frame length
     * @param direction RX or TX
     * @param timestamp Timestamp (0 = current time)
     */
    void logFrameToPcap(const uint8_t* frame, size_t length,
                        Tether::PacketLoggers::FrameDirection direction,
                        uint64_t timestamp = 0);

    /**
     * @brief Get packet logger
     */
    Tether::PacketLoggers::PacketLogger* getPcapLogger() { return pcapLogger_.get(); }
    
    // ========================================================================
    // Statistics
    // ========================================================================
    
    /**
     * @brief Get number of logged entries
     */
    uint64_t getLogCount() const { return logCount_.load(); }
    
    /**
     * @brief Get number of dropped entries (queue full)
     */
    uint64_t getDropCount() const { return dropCount_.load(); }
    
    /**
     * @brief Reset statistics
     */
    void resetStats();
    
private:
    // Background processing thread
    void processingThread();
    
    // Write entry to outputs
    void writeEntry(const SlaveLogEntry& entry);
    void writeToConsole(const SlaveLogEntry& entry);
    void writeToFile(const SlaveLogEntry& entry);
    void writeToCallback(const SlaveLogEntry& entry);
    
    // Get current timestamp
    uint64_t getTimestamp() const;
    
    // Configuration
    SlaveLogConfig config_;
    std::atomic<SlaveLogCategory> enabledCategories_;
    uint16_t slaveAddress_ = 0;
    
    // State
    std::atomic<bool> running_{false};
    
    // Lock-free queue (simplified ring buffer)
    std::vector<SlaveLogEntry> queue_;
    std::atomic<size_t> queueHead_{0};
    std::atomic<size_t> queueTail_{0};
    
    // Background thread
    std::unique_ptr<std::thread> processingThread_;
    
    // File output
    FILE* logFile_ = nullptr;
    size_t currentFileSize_ = 0;
    
    // Packet logger (shared with master)
    std::unique_ptr<Tether::PacketLoggers::PacketLogger> pcapLogger_;
    
    // Statistics
    std::atomic<uint64_t> logCount_{0};
    std::atomic<uint64_t> dropCount_{0};
    
    // Mutex for non-RT operations
    mutable std::mutex mutex_;
};

// ============================================================================
// Logging Macros
// ============================================================================

#define SLAVE_LOG(logger, cat, level, ...) \
    do { \
        if ((logger) && (logger)->isCategoryEnabled(cat)) { \
            (logger)->log(cat, level, __VA_ARGS__); \
        } \
    } while (0)

#define SLAVE_TRACE(logger, cat, ...) \
    SLAVE_LOG(logger, cat, EtherCAT::slave::SlaveLogLevel::Trace, __VA_ARGS__)

#define SLAVE_DEBUG(logger, cat, ...) \
    SLAVE_LOG(logger, cat, EtherCAT::slave::SlaveLogLevel::Debug, __VA_ARGS__)

#define SLAVE_INFO(logger, cat, ...) \
    SLAVE_LOG(logger, cat, EtherCAT::slave::SlaveLogLevel::Info, __VA_ARGS__)

#define SLAVE_WARN(logger, cat, ...) \
    SLAVE_LOG(logger, cat, EtherCAT::slave::SlaveLogLevel::Warning, __VA_ARGS__)

#define SLAVE_ERROR(logger, cat, ...) \
    SLAVE_LOG(logger, cat, EtherCAT::slave::SlaveLogLevel::Error, __VA_ARGS__)

#define SLAVE_CRITICAL(logger, cat, ...) \
    SLAVE_LOG(logger, cat, EtherCAT::slave::SlaveLogLevel::Critical, __VA_ARGS__)

}  // namespace slave
}  // namespace EtherCAT

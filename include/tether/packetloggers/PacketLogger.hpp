/**
 * @file PacketLogger.hpp
 * @brief Abstract packet capture logger interface
 *
 * Provides a virtual (abstract) interface for packet logging. Concrete
 * implementations (e.g. PCAP) are supplied separately and injected at
 * runtime, avoiding a circular dependency on any specific implementation.
 */

#pragma once

#include <cstdint>
#include <cstddef>
#include <memory>

namespace Tether {
namespace PacketLoggers {

// Forward declaration of PCAP configuration used by concrete implementations.
namespace PCAP {
struct PCAPLoggerConfig;
}

// ============================================================================
// Error Codes
// ============================================================================

enum class Error {
    OK = 0,
    InvalidArgument,
    NotInitialized,
    AlreadyInitialized,
    Timeout,
    WouldBlock,
    NoMemory,
    BufferTooSmall,
    BufferFull,
    Empty,
    NotSupported,
    PermissionDenied,
    InterfaceNotFound,
    LinkDown,
    TransmitFailed,
    ReceiveFailed,
    ConfigurationFailed,
    InternalError,
    Cancelled,
};

// ============================================================================
// Frame Direction
// ============================================================================

enum class FrameDirection {
    Rx,     ///< Received frame
    Tx,     ///< Transmitted frame
};

// ============================================================================
// Statistics
// ============================================================================

struct Stats {
    uint64_t rxFrames = 0;
    uint64_t txFrames = 0;
    uint64_t totalBytes = 0;
    uint64_t droppedFrames = 0;
};

// ============================================================================
// Abstract Packet Logger Interface
// ============================================================================

class PacketLogger {
public:
    virtual ~PacketLogger() = default;

    /**
     * @brief Initialize the logger
     * @param config Configuration (concrete implementations may specialize)
     * @return Error code
     */
    virtual Error init(const PCAP::PCAPLoggerConfig& config) = 0;

    /**
     * @brief Close the logger
     */
    virtual void close() = 0;

    /**
     * @brief Check if logger is open
     */
    virtual bool isOpen() const = 0;

    /**
     * @brief Log a frame
     * @param frame Frame data
     * @param length Frame length
     * @param direction RX or TX
     * @param timestamp Timestamp in microseconds (0 = use current time)
     * @return Error code
     */
    virtual Error logFrame(const uint8_t* frame, size_t length,
                           FrameDirection direction,
                           uint64_t timestamp = 0) = 0;

    /**
     * @brief Flush pending writes
     */
    virtual void flush() = 0;

    /**
     * @brief Get number of logged frames
     */
    virtual uint64_t getFrameCount() const = 0;

    /**
     * @brief Get current file size
     */
    virtual size_t getFileSize() const = 0;

    /**
     * @brief Get statistics
     */
    virtual Stats getStats() const = 0;
};

using PacketLoggerPtr = std::shared_ptr<PacketLogger>;

} // namespace PacketLoggers
} // namespace Tether

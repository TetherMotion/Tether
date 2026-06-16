/**
 * @file ISlaveHAL.hpp
 * @brief Hardware Abstraction Layer interface for EtherCAT slave
 *
 * @details
 * This file defines the HAL interfaces for EtherCAT slave implementations:
 *
 * - ISlaveHAL: Main interface for frame transmission/reception
 * - ISlaveTransport: Low-level transport abstraction
 *
 * Multiple implementations are provided:
 * - DirectSlaveHAL: In-process connection for unit testing
 * - FIFOSlaveHAL: POSIX FIFO for inter-process communication
 * - NetworkSlaveHAL: Real network interface for hardware testing
 */

#pragma once

#include "hal/HALTypes.hpp"
#include "hal/IEthernet.hpp"
#include "packetloggers/PacketLogger.hpp"
#include "packetloggers/pcap/PCAPLoggerConfig.hpp"
#include <cstdint>
#include <functional>
#include <memory>
#include <vector>

namespace EtherCAT {
namespace slave {

// ============================================================================
// HAL Configuration
// ============================================================================

/**
 * @brief Configuration for slave HAL
 */
struct SlaveHALConfig {
    // MAC address for the slave
    HAL::MacAddress macAddress{0x02, 0x00, 0x00, 0x00, 0x00, 0x01};
    
    // Interface name (for NetworkSlaveHAL)
    std::string interfaceName;
    
    // FIFO paths (for FIFOSlaveHAL)
    std::string fifoRxPath;   ///< Path for receiving frames
    std::string fifoTxPath;   ///< Path for sending frames
    
    // Packet logging
    bool enablePcapLogging = false;
    Tether::PacketLoggers::PCAP::PCAPLoggerConfig pcapConfig;
    
    // Timeout for blocking operations
    uint32_t timeoutMs = 1000;
    
    // Buffer size
    size_t rxBufferSize = 2048;
    size_t txBufferSize = 2048;
};

// ============================================================================
// Frame Reception Callback
// ============================================================================

/**
 * @brief Frame reception callback
 * @param frame Frame data (Ethernet header + payload)
 * @param length Frame length
 * @param userData User data pointer
 */
using SlaveRxCallback = std::function<void(const uint8_t* frame, size_t length, void* userData)>;

// ============================================================================
// ISlaveHAL Interface
// ============================================================================

/**
 * @brief Hardware Abstraction Layer interface for EtherCAT slave
 *
 * This interface provides frame-level access for slave implementations.
 * The slave receives EtherCAT frames, processes them, and sends back
 * modified frames.
 *
 * Implementations:
 * - DirectSlaveHAL: In-process connection (no networking)
 * - FIFOSlaveHAL: POSIX FIFO inter-process communication
 * - NetworkSlaveHAL: Real network interface (raw sockets)
 */
class ISlaveHAL {
public:
    virtual ~ISlaveHAL() = default;
    
    // ========================================================================
    // Initialization
    // ========================================================================
    
    /**
     * @brief Initialize the HAL
     * @param config HAL configuration
     * @return Error code
     */
    virtual HAL::Error init(const SlaveHALConfig& config) = 0;
    
    /**
     * @brief Shutdown the HAL
     */
    virtual void shutdown() = 0;
    
    /**
     * @brief Check if HAL is initialized
     */
    virtual bool isInitialized() const = 0;
    
    // ========================================================================
    // MAC Address
    // ========================================================================
    
    /**
     * @brief Get MAC address
     */
    virtual HAL::Error getMacAddress(HAL::MacAddress& mac) const = 0;
    
    /**
     * @brief Set MAC address
     */
    virtual HAL::Error setMacAddress(const HAL::MacAddress& mac) = 0;
    
    // ========================================================================
    // Frame Transmission
    // ========================================================================
    
    /**
     * @brief Transmit a frame
     *
     * For slave operation, this typically sends the modified frame
     * back to the master (response to received frame).
     *
     * @param frame Frame data (complete Ethernet frame)
     * @param length Frame length
     * @return Error code
     */
    virtual HAL::Error transmit(const uint8_t* frame, size_t length) = 0;
    
    // ========================================================================
    // Frame Reception
    // ========================================================================
    
    /**
     * @brief Set frame reception callback
     *
     * The callback is invoked for each received frame. The slave
     * should process the frame and call transmit() with the response.
     *
     * @param callback Callback function
     * @param userData User data passed to callback
     */
    virtual void setRxCallback(SlaveRxCallback callback, void* userData) = 0;
    
    /**
     * @brief Poll for received frames
     *
     * In polling mode, call this to check for and process received frames.
     * The RX callback will be invoked for each received frame.
     *
     * @param timeoutMs Timeout in milliseconds (0 = non-blocking)
     * @return Number of frames received, or negative on error
     */
    virtual int poll(HAL::Milliseconds timeoutMs) = 0;
    
    /**
     * @brief Receive a frame (blocking)
     *
     * Alternative to callback-based reception. Blocks until a frame
     * is received or timeout occurs.
     *
     * @param buffer Buffer to receive frame into
     * @param bufferSize Size of buffer
     * @param receivedLength Actual received length (output)
     * @param timeoutMs Timeout in milliseconds
     * @return Error code
     */
    virtual HAL::Error receive(uint8_t* buffer, size_t bufferSize,
                               size_t& receivedLength,
                               HAL::Milliseconds timeoutMs) = 0;
    
    // ========================================================================
    // Link Status
    // ========================================================================
    
    /**
     * @brief Get link status
     */
    virtual HAL::LinkStatus getLinkStatus() const = 0;
    
    /**
     * @brief Wait for link up
     */
    virtual HAL::Error waitForLinkUp(HAL::Milliseconds timeoutMs) = 0;
    
    // ========================================================================
    // Statistics
    // ========================================================================
    
    /**
     * @brief Get statistics
     */
    struct Stats {
        uint64_t rxFrames = 0;
        uint64_t txFrames = 0;
        uint64_t rxBytes = 0;
        uint64_t txBytes = 0;
        uint64_t rxErrors = 0;
        uint64_t txErrors = 0;
        uint64_t rxDropped = 0;
    };
    virtual Stats getStats() const = 0;
    
    /**
     * @brief Reset statistics
     */
    virtual void resetStats() = 0;
    
    // ========================================================================
    // Packet Logging
    // ========================================================================

    /**
     * @brief Enable packet logging
     */
    virtual HAL::Error enablePcapLogging(
        const Tether::PacketLoggers::PCAP::PCAPLoggerConfig& config) = 0;

    /**
     * @brief Disable packet logging
     */
    virtual void disablePcapLogging() = 0;

    /**
     * @brief Check if packet logging is enabled
     */
    virtual bool isPcapLoggingEnabled() const = 0;

    /**
     * @brief Get packet logger (may be null)
     */
    virtual Tether::PacketLoggers::PacketLogger* getPcapLogger() = 0;
};

// ============================================================================
// ILoopbackTarget Interface
// ============================================================================

/**
 * @brief Interface for loopback connection target
 *
 * Used by DirectSlaveHAL and DirectMasterHAL to connect directly
 * without any network or FIFO.
 */
class ILoopbackTarget {
public:
    virtual ~ILoopbackTarget() = default;
    
    /**
     * @brief Receive a frame from the connected peer
     *
     * Called by the peer when it transmits a frame.
     *
     * @param frame Frame data
     * @param length Frame length
     * @return Error code
     */
    virtual HAL::Error onFrameReceived(const uint8_t* frame, size_t length) = 0;
};

// ============================================================================
// Factory Functions
// ============================================================================

/**
 * @brief Create a direct (in-process) slave HAL
 *
 * This HAL is for unit testing. It connects directly to a master HAL
 * without any network or inter-process communication.
 */
std::unique_ptr<ISlaveHAL> createDirectSlaveHAL();

/**
 * @brief Create a POSIX FIFO slave HAL
 *
 * This HAL uses POSIX FIFOs for inter-process communication.
 * Useful for testing with separate master and slave processes.
 */
std::unique_ptr<ISlaveHAL> createFIFOSlaveHAL();

/**
 * @brief Create a network slave HAL
 *
 * This HAL uses real network interfaces via raw sockets.
 * For hardware-in-the-loop testing.
 */
std::unique_ptr<ISlaveHAL> createNetworkSlaveHAL();

}  // namespace slave
}  // namespace EtherCAT

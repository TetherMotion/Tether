/**
 * @file LoopbackHAL.hpp
 * @brief Loopback HAL for Master-Slave Communication
 *
 * @details
 * Provides a HAL implementation that enables direct in-process communication
 * between an EtherCAT master and one or more virtual slaves. Supports:
 *
 * - **Direct mode**: Frames are passed directly to slave in same thread
 * - **FIFO mode**: Frames pass through POSIX FIFOs (named pipes)
 * - **Network mode**: Frames pass through virtual network interface
 *
 * All modes support PcapNG logging using the shared IPcapLogger interface.
 *
 * ## Usage Example
 *
 * ```cpp
 * // Create master and slave
 * auto slave = createCiA402Slave(config);
 * auto loopbackHAL = createLoopbackHAL();
 *
 * // Attach slave to loopback
 * loopbackHAL->attachSlave(std::move(slave));
 *
 * // Use loopbackHAL as master's ethernet interface
 * master.setHAL(loopbackHAL);
 * ```
 */

#pragma once

#include "hal/IEthernet.hpp"
#include "hal/IPcapLogger.hpp"
#include "slave/core/SlaveCore.hpp"

#include <memory>
#include <vector>
#include <functional>
#include <mutex>
#include <thread>
#include <atomic>
#include <chrono>

namespace EtherCAT {
namespace HAL {

// Forward declarations
class ILoopbackTarget;

// ============================================================================
// Loopback Configuration
// ============================================================================

enum class LoopbackMode {
    Direct,     // In-process, same thread
    Threaded,   // In-process, separate realtime thread
    FIFO,       // POSIX FIFO (named pipe)
    Network,    // Virtual network interface (veth)
};

struct LoopbackHALConfig {
    LoopbackMode mode = LoopbackMode::Direct;
    
    // FIFO mode settings
    std::string txFifoPath = "/tmp/ethercat_master_tx";
    std::string rxFifoPath = "/tmp/ethercat_master_rx";
    
    // Network mode settings
    std::string networkInterface = "veth0";
    
    // Realtime settings (for Threaded mode)
    int realtimePriority = 80;
    bool useRealtimeScheduler = true;
    
    // Timing
    uint32_t responseDelayNs = 1000;    // Simulated propagation delay
    uint32_t processingDelayNs = 5000;  // Simulated slave processing time
    
    // Logging
    bool enablePcapLogging = false;
    std::string pcapFilePath = "ethercat_loopback.pcapng";
    
    // Error injection (for testing)
    float frameDropRate = 0.0f;         // 0.0 = no drops, 1.0 = all dropped
    float frameCrcErrorRate = 0.0f;
    uint32_t jitterMaxNs = 0;           // Max random delay jitter
};

// ============================================================================
// Loopback Statistics
// ============================================================================

struct LoopbackStats {
    uint64_t framesSent = 0;
    uint64_t framesReceived = 0;
    uint64_t framesDropped = 0;
    uint64_t framesCrcError = 0;
    uint64_t bytesTransmitted = 0;
    uint64_t bytesReceived = 0;
    uint64_t processingTimeNs = 0;
    uint64_t maxProcessingTimeNs = 0;
    uint64_t minProcessingTimeNs = UINT64_MAX;
    
    void reset() {
        framesSent = 0;
        framesReceived = 0;
        framesDropped = 0;
        framesCrcError = 0;
        bytesTransmitted = 0;
        bytesReceived = 0;
        processingTimeNs = 0;
        maxProcessingTimeNs = 0;
        minProcessingTimeNs = UINT64_MAX;
    }
};

// ============================================================================
// ILoopbackTarget Interface
// ============================================================================

/**
 * @brief Interface for objects that can receive EtherCAT frames
 */
class ILoopbackTarget {
public:
    virtual ~ILoopbackTarget() = default;
    
    /**
     * @brief Process an incoming EtherCAT frame
     * @param frame Raw frame data (including Ethernet header)
     * @param length Frame length
     * @param responseBuffer Buffer to write response
     * @param responseLength Output: length of response
     * @return true if frame was processed and response generated
     */
    virtual bool processFrame(const uint8_t* frame, size_t length,
                              uint8_t* responseBuffer, size_t& responseLength) = 0;
    
    /**
     * @brief Get the configured station address of this target
     */
    virtual uint16_t getConfiguredAddress() const = 0;
    
    /**
     * @brief Check if target is operational
     */
    virtual bool isOperational() const = 0;
};

// ============================================================================
// LoopbackHAL Class
// ============================================================================

/**
 * @brief HAL implementation that routes frames to virtual slaves
 *
 * Implements IEthernet for master-side use and manages frame routing
 * to attached virtual slaves.
 */
class LoopbackHAL : public IEthernet {
public:
    explicit LoopbackHAL(const LoopbackHALConfig& config = {});
    ~LoopbackHAL() override;
    
    // IEthernet interface
    bool init() override;
    void deinit() override;
    bool sendFrame(const uint8_t* data, size_t length) override;
    bool receiveFrame(uint8_t* buffer, size_t bufferSize, size_t& receivedLength,
                      uint32_t timeoutMs) override;
    bool getMacAddress(uint8_t* mac) override;
    bool isLinkUp() override;
    
    // Slave management
    void attachSlave(std::shared_ptr<slave::SlaveCore> slave);
    void attachSlave(std::unique_ptr<slave::SlaveCore> slave);
    void attachTarget(std::shared_ptr<ILoopbackTarget> target);
    void detachAllSlaves();
    size_t getSlaveCount() const;
    
    // Direct slave access (for testing)
    slave::SlaveCore* getSlave(size_t index);
    const slave::SlaveCore* getSlave(size_t index) const;
    
    // PcapNG logging
    void setPcapLogger(std::shared_ptr<IPcapLogger> logger);
    void enableLogging(bool enable);
    
    // Statistics
    const LoopbackStats& getStats() const { return stats_; }
    void resetStats() { stats_.reset(); }
    
    // Configuration
    void setResponseDelay(uint32_t delayNs) { config_.responseDelayNs = delayNs; }
    void setProcessingDelay(uint32_t delayNs) { config_.processingDelayNs = delayNs; }
    void setFrameDropRate(float rate) { config_.frameDropRate = rate; }
    void setJitter(uint32_t maxNs) { config_.jitterMaxNs = maxNs; }
    
    // Mode switching
    bool switchMode(LoopbackMode newMode);
    LoopbackMode getMode() const { return config_.mode; }
    
private:
    LoopbackHALConfig config_;
    LoopbackStats stats_;
    
    // Slaves (owned)
    std::vector<std::shared_ptr<slave::SlaveCore>> slaves_;
    
    // External targets (not owned)
    std::vector<std::shared_ptr<ILoopbackTarget>> targets_;
    
    // Logging
    std::shared_ptr<IPcapLogger> pcapLogger_;
    bool loggingEnabled_ = false;
    
    // Threading (for Threaded mode)
    std::thread workerThread_;
    std::atomic<bool> running_{false};
    
    // FIFO mode
    int txFifoFd_ = -1;
    int rxFifoFd_ = -1;
    
    // Network mode
    int rawSocket_ = -1;
    
    // Response queue (for async modes)
    std::mutex responseMutex_;
    std::vector<std::vector<uint8_t>> responseQueue_;
    
    // MAC address
    uint8_t macAddress_[6] = {0x02, 0x00, 0x00, 0x00, 0x00, 0x01};
    
    bool initialized_ = false;
    
    // Mode-specific initialization
    bool initDirect();
    bool initThreaded();
    bool initFIFO();
    bool initNetwork();
    
    // Frame processing
    bool processFrameDirect(const uint8_t* frame, size_t length);
    bool processFrameAsync(const uint8_t* frame, size_t length);
    
    // Worker thread
    void workerLoop();
    
    // Error injection
    bool shouldDropFrame();
    bool shouldInjectCrcError();
    void applyJitter();
    
    // Logging
    void logFrame(const uint8_t* frame, size_t length, bool isRx);
};

// ============================================================================
// Factory Functions
// ============================================================================

std::unique_ptr<LoopbackHAL> createLoopbackHAL(const LoopbackHALConfig& config = {});
std::unique_ptr<LoopbackHAL> createDirectLoopbackHAL();
std::unique_ptr<LoopbackHAL> createFIFOLoopbackHAL(const std::string& txPath, const std::string& rxPath);
std::unique_ptr<LoopbackHAL> createThreadedLoopbackHAL(int priority = 80);

// ============================================================================
// SlaveToTargetAdapter
// ============================================================================

/**
 * @brief Adapter to use SlaveCore as ILoopbackTarget
 */
class SlaveToTargetAdapter : public ILoopbackTarget {
public:
    explicit SlaveToTargetAdapter(std::shared_ptr<slave::SlaveCore> slave);
    
    bool processFrame(const uint8_t* frame, size_t length,
                      uint8_t* responseBuffer, size_t& responseLength) override;
    uint16_t getConfiguredAddress() const override;
    bool isOperational() const override;
    
private:
    std::shared_ptr<slave::SlaveCore> slave_;
};

}  // namespace hal
}  // namespace EtherCAT

/**
 * @file FIFOHAL.hpp
 * @brief POSIX FIFO HAL for Inter-Process EtherCAT Communication
 *
 * @details
 * Provides HAL implementations that use POSIX FIFOs (named pipes) for
 * communication between separate processes. Useful for:
 *
 * - Testing master/slave in separate processes
 * - Debugging with process isolation
 * - Integration with external test frameworks
 *
 * ## Architecture
 *
 * ```
 * Master Process                     Slave Process
 * +---------------+                  +---------------+
 * |   Master      |                  |   Slave       |
 * |   Stack       |                  |   Stack       |
 * +-------+-------+                  +-------+-------+
 *         |                                  |
 * +-------v-------+                  +-------v-------+
 * | MasterFIFOHAL |                  | SlaveFIFOHAL  |
 * +-------+-------+                  +-------+-------+
 *         |                                  |
 *         |    /tmp/ethercat_tx.fifo         |
 *         +--------------------------------->+
 *         |                                  |
 *         |    /tmp/ethercat_rx.fifo         |
 *         +<---------------------------------+
 * ```
 */

#pragma once

#include "hal/IEthernet.hpp"
#include "slave/hal/ISlaveHAL.hpp"
#include "hal/IPcapLogger.hpp"

#include <memory>
#include <string>
#include <mutex>
#include <thread>
#include <atomic>
#include <functional>

namespace EtherCAT {
namespace hal {

// ============================================================================
// FIFO Configuration
// ============================================================================

struct FIFOHALConfig {
    // FIFO paths
    std::string txFifoPath = "/tmp/ethercat_tx.fifo";
    std::string rxFifoPath = "/tmp/ethercat_rx.fifo";
    
    // Create FIFOs if they don't exist
    bool createFifos = true;
    mode_t fifoMode = 0666;
    
    // Blocking behavior
    bool blockingRead = true;
    bool blockingWrite = true;
    
    // Timeouts (ms)
    uint32_t readTimeoutMs = 1000;
    uint32_t writeTimeoutMs = 100;
    
    // Buffer sizes
    size_t txBufferSize = 2048;
    size_t rxBufferSize = 2048;
    
    // Logging
    bool enablePcapLogging = false;
    std::string pcapFilePath;
    
    // Frame framing (for FIFO stream)
    // Each frame is preceded by a 4-byte length header
    bool useLengthHeader = true;
};

// ============================================================================
// FIFO Statistics
// ============================================================================

struct FIFOStats {
    uint64_t framesSent = 0;
    uint64_t framesReceived = 0;
    uint64_t bytesTransmitted = 0;
    uint64_t bytesReceived = 0;
    uint64_t writeErrors = 0;
    uint64_t readErrors = 0;
    uint64_t timeouts = 0;
    uint64_t fifoOverruns = 0;
    
    void reset() {
        framesSent = 0;
        framesReceived = 0;
        bytesTransmitted = 0;
        bytesReceived = 0;
        writeErrors = 0;
        readErrors = 0;
        timeouts = 0;
        fifoOverruns = 0;
    }
};

// ============================================================================
// MasterFIFOHAL - For Master Side
// ============================================================================

/**
 * @brief FIFO-based HAL for the master side
 *
 * Implements IEthernet, sending frames to TX FIFO and receiving from RX FIFO.
 */
class MasterFIFOHAL : public IEthernet {
public:
    explicit MasterFIFOHAL(const FIFOHALConfig& config = {});
    ~MasterFIFOHAL() override;
    
    // IEthernet interface
    bool init() override;
    void deinit() override;
    bool sendFrame(const uint8_t* data, size_t length) override;
    bool receiveFrame(uint8_t* buffer, size_t bufferSize, size_t& receivedLength,
                      uint32_t timeoutMs) override;
    bool getMacAddress(uint8_t* mac) override;
    bool isLinkUp() override;
    
    // Statistics
    const FIFOStats& getStats() const { return stats_; }
    void resetStats() { stats_.reset(); }
    
    // PcapNG logging
    void setPcapLogger(std::shared_ptr<IPcapLogger> logger);
    
    // FIFO management
    bool createFifos();
    bool removeFifos();
    bool areFifosOpen() const { return txFd_ >= 0 && rxFd_ >= 0; }
    
private:
    FIFOHALConfig config_;
    FIFOStats stats_;
    
    int txFd_ = -1;
    int rxFd_ = -1;
    
    uint8_t macAddress_[6] = {0x02, 0x00, 0x00, 0x00, 0x00, 0x01};
    
    std::shared_ptr<IPcapLogger> pcapLogger_;
    bool initialized_ = false;
    
    bool writeFrame(const uint8_t* data, size_t length);
    bool readFrame(uint8_t* buffer, size_t bufferSize, size_t& receivedLength,
                   uint32_t timeoutMs);
};

// ============================================================================
// SlaveFIFOHAL - For Slave Side
// ============================================================================

/**
 * @brief FIFO-based HAL for the slave side
 *
 * Implements ISlaveHAL, receiving frames from RX FIFO (master's TX)
 * and sending responses to TX FIFO (master's RX).
 *
 * Note: From slave's perspective, the FIFOs are reversed:
 * - Slave reads from master's TX FIFO
 * - Slave writes to master's RX FIFO
 */
class SlaveFIFOHAL : public slave::ISlaveHAL {
public:
    explicit SlaveFIFOHAL(const FIFOHALConfig& config = {});
    ~SlaveFIFOHAL() override;
    
    // ISlaveHAL interface
    bool init() override;
    void deinit() override;
    bool sendResponse(const uint8_t* data, size_t length) override;
    bool waitForFrame(uint8_t* buffer, size_t bufferSize, size_t& receivedLength,
                      uint32_t timeoutMs) override;
    bool isConnected() const override;
    
    // Frame processing callback
    using FrameCallback = std::function<bool(const uint8_t* frame, size_t length,
                                              uint8_t* response, size_t& respLength)>;
    void setFrameCallback(FrameCallback callback) { frameCallback_ = callback; }
    
    // Start/stop background processing thread
    bool startProcessing();
    void stopProcessing();
    bool isProcessing() const { return processing_.load(); }
    
    // Statistics
    const FIFOStats& getStats() const { return stats_; }
    void resetStats() { stats_.reset(); }
    
    // PcapNG logging
    void setPcapLogger(std::shared_ptr<IPcapLogger> logger);
    
private:
    FIFOHALConfig config_;
    FIFOStats stats_;
    
    // FIFOs (reversed from master's perspective)
    int rxFd_ = -1;   // Read from master's TX
    int txFd_ = -1;   // Write to master's RX
    
    std::shared_ptr<IPcapLogger> pcapLogger_;
    
    // Processing thread
    std::thread processingThread_;
    std::atomic<bool> processing_{false};
    std::atomic<bool> running_{false};
    
    FrameCallback frameCallback_;
    
    bool initialized_ = false;
    
    void processingLoop();
};

// ============================================================================
// FIFOBridge - Connects Master and Slave HALs
// ============================================================================

/**
 * @brief Manages FIFO creation and lifecycle for master-slave communication
 *
 * Creates and manages the FIFOs, and provides both MasterFIFOHAL and SlaveFIFOHAL
 * instances pre-configured to communicate through them.
 */
class FIFOBridge {
public:
    struct Config {
        std::string basePath = "/tmp/ethercat";
        bool cleanupOnDestroy = true;
        mode_t fifoMode = 0666;
    };
    
    explicit FIFOBridge(const Config& config = {});
    ~FIFOBridge();
    
    // Initialization
    bool init();
    void deinit();
    
    // Get HAL instances
    std::unique_ptr<MasterFIFOHAL> createMasterHAL();
    std::unique_ptr<SlaveFIFOHAL> createSlaveHAL();
    
    // Get FIFO paths
    const std::string& getMasterToSlavePath() const { return masterToSlavePath_; }
    const std::string& getSlaveToMasterPath() const { return slaveToMasterPath_; }
    
private:
    Config config_;
    std::string masterToSlavePath_;
    std::string slaveToMasterPath_;
    bool initialized_ = false;
};

// ============================================================================
// Factory Functions
// ============================================================================

std::unique_ptr<MasterFIFOHAL> createMasterFIFOHAL(const FIFOHALConfig& config = {});
std::unique_ptr<MasterFIFOHAL> createMasterFIFOHAL(const std::string& txPath, const std::string& rxPath);

std::unique_ptr<SlaveFIFOHAL> createSlaveFIFOHAL(const FIFOHALConfig& config = {});
std::unique_ptr<SlaveFIFOHAL> createSlaveFIFOHAL(const std::string& rxPath, const std::string& txPath);

std::unique_ptr<FIFOBridge> createFIFOBridge(const std::string& basePath = "/tmp/ethercat");

}  // namespace hal
}  // namespace EtherCAT

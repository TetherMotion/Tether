/**
 * @file FIFOHAL.cpp
 * @brief POSIX FIFO HAL Implementation
 */

#include "hal/FIFOHAL.hpp"

#include <cstring>
#include <fcntl.h>
#include <unistd.h>
#include <sys/stat.h>
#include <sys/select.h>
#include <poll.h>
#include <errno.h>

namespace EtherCAT {
namespace HAL {

// ============================================================================
// MasterFIFOHAL Implementation
// ============================================================================

MasterFIFOHAL::MasterFIFOHAL(const FIFOHALConfig& config)
    : config_(config)
{
}

MasterFIFOHAL::~MasterFIFOHAL() {
    deinit();
}

bool MasterFIFOHAL::init() {
    if (initialized_) {
        return true;
    }
    
    // Create FIFOs if needed
    if (config_.createFifos) {
        if (!createFifos()) {
            return false;
        }
    }
    
    // Open TX FIFO for writing (non-blocking initially)
    txFd_ = open(config_.txFifoPath.c_str(), O_WRONLY | O_NONBLOCK);
    if (txFd_ < 0) {
        // Retry without O_NONBLOCK (will block until reader opens)
        if (config_.blockingWrite) {
            txFd_ = open(config_.txFifoPath.c_str(), O_WRONLY);
        }
    }
    
    // Open RX FIFO for reading
    int flags = O_RDONLY;
    if (!config_.blockingRead) {
        flags |= O_NONBLOCK;
    }
    rxFd_ = open(config_.rxFifoPath.c_str(), flags);
    
    if (txFd_ < 0 || rxFd_ < 0) {
        if (txFd_ >= 0) { close(txFd_); txFd_ = -1; }
        if (rxFd_ >= 0) { close(rxFd_); rxFd_ = -1; }
        return false;
    }
    
    initialized_ = true;
    return true;
}

void MasterFIFOHAL::deinit() {
    if (txFd_ >= 0) {
        close(txFd_);
        txFd_ = -1;
    }
    if (rxFd_ >= 0) {
        close(rxFd_);
        rxFd_ = -1;
    }
    initialized_ = false;
}

bool MasterFIFOHAL::sendFrame(const uint8_t* data, size_t length) {
    if (!initialized_ || txFd_ < 0) {
        return false;
    }
    
    // Log to packet logger
    if (pcapLogger_) {
        pcapLogger_->logFrame(data, length, Tether::PacketLoggers::FrameDirection::Tx,
                              getSystemClock().nowMicros());
    }

    bool result = writeFrame(data, length);
    
    if (result) {
        stats_.framesSent++;
        stats_.bytesTransmitted += length;
    } else {
        stats_.writeErrors++;
    }
    
    return result;
}

bool MasterFIFOHAL::writeFrame(const uint8_t* data, size_t length) {
    if (config_.useLengthHeader) {
        // Write length header first
        uint32_t len = static_cast<uint32_t>(length);
        ssize_t n = write(txFd_, &len, sizeof(len));
        if (n != sizeof(len)) {
            return false;
        }
    }
    
    // Write frame data
    ssize_t n = write(txFd_, data, length);
    return n == static_cast<ssize_t>(length);
}

bool MasterFIFOHAL::receiveFrame(uint8_t* buffer, size_t bufferSize,
                                  size_t& receivedLength, uint32_t timeoutMs) {
    if (!initialized_ || rxFd_ < 0) {
        return false;
    }
    
    bool result = readFrame(buffer, bufferSize, receivedLength, timeoutMs);
    
    if (result) {
        stats_.framesReceived++;
        stats_.bytesReceived += receivedLength;
        
        // Log to packet logger
        if (pcapLogger_) {
            pcapLogger_->logFrame(buffer, receivedLength,
                                  Tether::PacketLoggers::FrameDirection::Rx,
                                  getSystemClock().nowMicros());
        }
    } else if (errno == EAGAIN || errno == EWOULDBLOCK) {
        stats_.timeouts++;
    } else {
        stats_.readErrors++;
    }
    
    return result;
}

bool MasterFIFOHAL::readFrame(uint8_t* buffer, size_t bufferSize,
                               size_t& receivedLength, uint32_t timeoutMs) {
    // Wait for data with timeout
    struct pollfd pfd;
    pfd.fd = rxFd_;
    pfd.events = POLLIN;
    
    int ret = poll(&pfd, 1, timeoutMs);
    if (ret <= 0) {
        return false;  // Timeout or error
    }
    
    if (config_.useLengthHeader) {
        // Read length header
        uint32_t len;
        ssize_t n = read(rxFd_, &len, sizeof(len));
        if (n != sizeof(len)) {
            return false;
        }
        
        if (len > bufferSize) {
            // Frame too large - skip it
            uint8_t discard[256];
            while (len > 0) {
                size_t toRead = std::min(len, (uint32_t)sizeof(discard));
                read(rxFd_, discard, toRead);
                len -= toRead;
            }
            stats_.fifoOverruns++;
            return false;
        }
        
        // Read frame data
        n = read(rxFd_, buffer, len);
        if (n != static_cast<ssize_t>(len)) {
            return false;
        }
        
        receivedLength = len;
    } else {
        // Read available data
        ssize_t n = read(rxFd_, buffer, bufferSize);
        if (n <= 0) {
            return false;
        }
        receivedLength = n;
    }
    
    return true;
}

bool MasterFIFOHAL::getMacAddress(uint8_t* mac) {
    std::memcpy(mac, macAddress_, 6);
    return true;
}

bool MasterFIFOHAL::isLinkUp() {
    return initialized_ && txFd_ >= 0 && rxFd_ >= 0;
}

void MasterFIFOHAL::setPcapLogger(
        std::shared_ptr<Tether::PacketLoggers::PacketLogger> logger) {
    pcapLogger_ = logger;
}

bool MasterFIFOHAL::createFifos() {
    // Remove existing FIFOs
    unlink(config_.txFifoPath.c_str());
    unlink(config_.rxFifoPath.c_str());
    
    // Create TX FIFO
    if (mkfifo(config_.txFifoPath.c_str(), config_.fifoMode) != 0) {
        if (errno != EEXIST) {
            return false;
        }
    }
    
    // Create RX FIFO
    if (mkfifo(config_.rxFifoPath.c_str(), config_.fifoMode) != 0) {
        if (errno != EEXIST) {
            unlink(config_.txFifoPath.c_str());
            return false;
        }
    }
    
    return true;
}

bool MasterFIFOHAL::removeFifos() {
    bool result = true;
    
    if (unlink(config_.txFifoPath.c_str()) != 0 && errno != ENOENT) {
        result = false;
    }
    
    if (unlink(config_.rxFifoPath.c_str()) != 0 && errno != ENOENT) {
        result = false;
    }
    
    return result;
}

// ============================================================================
// SlaveFIFOHAL Implementation
// ============================================================================

SlaveFIFOHAL::SlaveFIFOHAL(const FIFOHALConfig& config)
    : config_(config)
{
}

SlaveFIFOHAL::~SlaveFIFOHAL() {
    stopProcessing();
    deinit();
}

bool SlaveFIFOHAL::init() {
    if (initialized_) {
        return true;
    }
    
    // Create FIFOs if needed
    if (config_.createFifos) {
        unlink(config_.rxFifoPath.c_str());
        unlink(config_.txFifoPath.c_str());
        mkfifo(config_.rxFifoPath.c_str(), config_.fifoMode);
        mkfifo(config_.txFifoPath.c_str(), config_.fifoMode);
    }
    
    // Slave reads from master's TX (which is slave's RX)
    // Open master's TX FIFO for reading
    int flags = O_RDONLY;
    if (!config_.blockingRead) {
        flags |= O_NONBLOCK;
    }
    rxFd_ = open(config_.rxFifoPath.c_str(), flags);
    
    // Open master's RX FIFO for writing (slave's TX)
    txFd_ = open(config_.txFifoPath.c_str(), O_WRONLY | O_NONBLOCK);
    if (txFd_ < 0 && config_.blockingWrite) {
        txFd_ = open(config_.txFifoPath.c_str(), O_WRONLY);
    }
    
    if (rxFd_ < 0 || txFd_ < 0) {
        if (rxFd_ >= 0) { close(rxFd_); rxFd_ = -1; }
        if (txFd_ >= 0) { close(txFd_); txFd_ = -1; }
        return false;
    }
    
    initialized_ = true;
    return true;
}

void SlaveFIFOHAL::deinit() {
    if (rxFd_ >= 0) {
        close(rxFd_);
        rxFd_ = -1;
    }
    if (txFd_ >= 0) {
        close(txFd_);
        txFd_ = -1;
    }
    initialized_ = false;
}

bool SlaveFIFOHAL::sendResponse(const uint8_t* data, size_t length) {
    if (!initialized_ || txFd_ < 0) {
        return false;
    }
    
    // Log to packet logger
    if (pcapLogger_) {
        pcapLogger_->logFrame(data, length, Tether::PacketLoggers::FrameDirection::Tx,
                              getSystemClock().nowMicros());
    }

    if (config_.useLengthHeader) {
        uint32_t len = static_cast<uint32_t>(length);
        if (write(txFd_, &len, sizeof(len)) != sizeof(len)) {
            stats_.writeErrors++;
            return false;
        }
    }
    
    ssize_t n = write(txFd_, data, length);
    if (n == static_cast<ssize_t>(length)) {
        stats_.framesSent++;
        stats_.bytesTransmitted += length;
        return true;
    }
    
    stats_.writeErrors++;
    return false;
}

bool SlaveFIFOHAL::waitForFrame(uint8_t* buffer, size_t bufferSize,
                                 size_t& receivedLength, uint32_t timeoutMs) {
    if (!initialized_ || rxFd_ < 0) {
        return false;
    }
    
    struct pollfd pfd;
    pfd.fd = rxFd_;
    pfd.events = POLLIN;
    
    int ret = poll(&pfd, 1, timeoutMs);
    if (ret <= 0) {
        if (ret == 0) stats_.timeouts++;
        return false;
    }
    
    if (config_.useLengthHeader) {
        uint32_t len;
        ssize_t n = read(rxFd_, &len, sizeof(len));
        if (n != sizeof(len)) {
            stats_.readErrors++;
            return false;
        }
        
        if (len > bufferSize) {
            // Skip oversized frame
            uint8_t discard[256];
            while (len > 0) {
                size_t toRead = std::min(len, (uint32_t)sizeof(discard));
                read(rxFd_, discard, toRead);
                len -= toRead;
            }
            stats_.fifoOverruns++;
            return false;
        }
        
        n = read(rxFd_, buffer, len);
        if (n != static_cast<ssize_t>(len)) {
            stats_.readErrors++;
            return false;
        }
        
        receivedLength = len;
    } else {
        ssize_t n = read(rxFd_, buffer, bufferSize);
        if (n <= 0) {
            stats_.readErrors++;
            return false;
        }
        receivedLength = n;
    }
    
    stats_.framesReceived++;
    stats_.bytesReceived += receivedLength;
    
    // Log to packet logger
    if (pcapLogger_) {
        pcapLogger_->logFrame(buffer, receivedLength,
                              Tether::PacketLoggers::FrameDirection::Rx,
                              getSystemClock().nowMicros());
    }

    return true;
}

bool SlaveFIFOHAL::isConnected() const {
    return initialized_ && rxFd_ >= 0 && txFd_ >= 0;
}

bool SlaveFIFOHAL::startProcessing() {
    if (processing_) {
        return true;
    }
    
    if (!frameCallback_) {
        return false;
    }
    
    running_ = true;
    processing_ = true;
    
    processingThread_ = std::thread([this]() {
        processingLoop();
    });
    
    return true;
}

void SlaveFIFOHAL::stopProcessing() {
    running_ = false;
    
    if (processingThread_.joinable()) {
        processingThread_.join();
    }
    
    processing_ = false;
}

void SlaveFIFOHAL::processingLoop() {
    std::vector<uint8_t> frameBuffer(config_.rxBufferSize);
    std::vector<uint8_t> responseBuffer(config_.txBufferSize);
    
    while (running_) {
        size_t receivedLength = 0;
        
        if (waitForFrame(frameBuffer.data(), frameBuffer.size(), receivedLength, 100)) {
            if (frameCallback_) {
                size_t responseLength = 0;
                
                if (frameCallback_(frameBuffer.data(), receivedLength,
                                   responseBuffer.data(), responseLength)) {
                    if (responseLength > 0) {
                        sendResponse(responseBuffer.data(), responseLength);
                    }
                }
            }
        }
    }
}

void SlaveFIFOHAL::setPcapLogger(
        std::shared_ptr<Tether::PacketLoggers::PacketLogger> logger) {
    pcapLogger_ = logger;
}

// ============================================================================
// FIFOBridge Implementation
// ============================================================================

FIFOBridge::FIFOBridge(const Config& config)
    : config_(config)
    , masterToSlavePath_(config.basePath + "_m2s.fifo")
    , slaveToMasterPath_(config.basePath + "_s2m.fifo")
{
}

FIFOBridge::~FIFOBridge() {
    deinit();
    
    if (config_.cleanupOnDestroy) {
        unlink(masterToSlavePath_.c_str());
        unlink(slaveToMasterPath_.c_str());
    }
}

bool FIFOBridge::init() {
    if (initialized_) {
        return true;
    }
    
    // Remove any existing FIFOs
    unlink(masterToSlavePath_.c_str());
    unlink(slaveToMasterPath_.c_str());
    
    // Create FIFOs
    if (mkfifo(masterToSlavePath_.c_str(), config_.fifoMode) != 0) {
        if (errno != EEXIST) {
            return false;
        }
    }
    
    if (mkfifo(slaveToMasterPath_.c_str(), config_.fifoMode) != 0) {
        if (errno != EEXIST) {
            unlink(masterToSlavePath_.c_str());
            return false;
        }
    }
    
    initialized_ = true;
    return true;
}

void FIFOBridge::deinit() {
    initialized_ = false;
}

std::unique_ptr<MasterFIFOHAL> FIFOBridge::createMasterHAL() {
    FIFOHALConfig config;
    config.txFifoPath = masterToSlavePath_;
    config.rxFifoPath = slaveToMasterPath_;
    config.createFifos = false;  // Already created by bridge
    
    return std::make_unique<MasterFIFOHAL>(config);
}

std::unique_ptr<SlaveFIFOHAL> FIFOBridge::createSlaveHAL() {
    FIFOHALConfig config;
    // Slave reads from master's TX and writes to master's RX
    config.rxFifoPath = masterToSlavePath_;
    config.txFifoPath = slaveToMasterPath_;
    config.createFifos = false;  // Already created by bridge
    
    return std::make_unique<SlaveFIFOHAL>(config);
}

// ============================================================================
// Factory Functions
// ============================================================================

std::unique_ptr<MasterFIFOHAL> createMasterFIFOHAL(const FIFOHALConfig& config) {
    return std::make_unique<MasterFIFOHAL>(config);
}

std::unique_ptr<MasterFIFOHAL> createMasterFIFOHAL(const std::string& txPath, const std::string& rxPath) {
    FIFOHALConfig config;
    config.txFifoPath = txPath;
    config.rxFifoPath = rxPath;
    return std::make_unique<MasterFIFOHAL>(config);
}

std::unique_ptr<SlaveFIFOHAL> createSlaveFIFOHAL(const FIFOHALConfig& config) {
    return std::make_unique<SlaveFIFOHAL>(config);
}

std::unique_ptr<SlaveFIFOHAL> createSlaveFIFOHAL(const std::string& rxPath, const std::string& txPath) {
    FIFOHALConfig config;
    config.rxFifoPath = rxPath;
    config.txFifoPath = txPath;
    return std::make_unique<SlaveFIFOHAL>(config);
}

std::unique_ptr<FIFOBridge> createFIFOBridge(const std::string& basePath) {
    FIFOBridge::Config config;
    config.basePath = basePath;
    return std::make_unique<FIFOBridge>(config);
}

}  // namespace hal
}  // namespace EtherCAT

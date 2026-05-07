/**
 * @file LoopbackHAL.cpp
 * @brief Loopback HAL Implementation
 */

#include "hal/LoopbackHAL.hpp"
#include "slave/core/SlaveCore.hpp"

#include <cstring>
#include <chrono>
#include <random>
#include <fcntl.h>
#include <unistd.h>
#include <sys/stat.h>
#include <sys/select.h>

#ifdef __linux__
#include <sys/socket.h>
#include <linux/if_packet.h>
#include <net/if.h>
#include <net/ethernet.h>
#include <sys/ioctl.h>
#endif

namespace EtherCAT {
namespace HAL {

// ============================================================================
// LoopbackHAL Implementation
// ============================================================================

LoopbackHAL::LoopbackHAL(const LoopbackHALConfig& config)
    : config_(config)
{
}

LoopbackHAL::~LoopbackHAL() {
    deinit();
}

bool LoopbackHAL::init() {
    if (initialized_) {
        return true;
    }
    
    bool result = false;
    
    switch (config_.mode) {
        case LoopbackMode::Direct:
            result = initDirect();
            break;
        case LoopbackMode::Threaded:
            result = initThreaded();
            break;
        case LoopbackMode::FIFO:
            result = initFIFO();
            break;
        case LoopbackMode::Network:
            result = initNetwork();
            break;
    }
    
    if (result) {
        initialized_ = true;
        
        // Setup PcapNG logging if enabled
        if (config_.enablePcapLogging && !pcapLogger_) {
            // Create default logger (user can override with setPcapLogger)
        }
    }
    
    return result;
}

bool LoopbackHAL::initDirect() {
    // Direct mode needs no special initialization
    return true;
}

bool LoopbackHAL::initThreaded() {
    running_ = true;
    
    workerThread_ = std::thread([this]() {
        workerLoop();
    });
    
    return true;
}

bool LoopbackHAL::initFIFO() {
#ifdef __linux__
    // Create FIFOs if they don't exist
    mkfifo(config_.txFifoPath.c_str(), 0666);
    mkfifo(config_.rxFifoPath.c_str(), 0666);
    
    // Open TX FIFO for writing
    txFifoFd_ = open(config_.txFifoPath.c_str(), O_WRONLY | O_NONBLOCK);
    if (txFifoFd_ < 0) {
        // May need to wait for reader
        txFifoFd_ = open(config_.txFifoPath.c_str(), O_WRONLY);
    }
    
    // Open RX FIFO for reading
    rxFifoFd_ = open(config_.rxFifoPath.c_str(), O_RDONLY | O_NONBLOCK);
    
    return txFifoFd_ >= 0 && rxFifoFd_ >= 0;
#else
    return false;  // FIFO mode only on Linux/POSIX
#endif
}

bool LoopbackHAL::initNetwork() {
#ifdef __linux__
    // Create raw socket for virtual network interface
    rawSocket_ = socket(AF_PACKET, SOCK_RAW, htons(ETH_P_ALL));
    if (rawSocket_ < 0) {
        return false;
    }
    
    // Bind to interface
    struct ifreq ifr;
    std::memset(&ifr, 0, sizeof(ifr));
    std::strncpy(ifr.ifr_name, config_.networkInterface.c_str(), IFNAMSIZ - 1);
    
    if (ioctl(rawSocket_, SIOCGIFINDEX, &ifr) < 0) {
        close(rawSocket_);
        rawSocket_ = -1;
        return false;
    }
    
    struct sockaddr_ll sll;
    std::memset(&sll, 0, sizeof(sll));
    sll.sll_family = AF_PACKET;
    sll.sll_ifindex = ifr.ifr_ifindex;
    sll.sll_protocol = htons(ETH_P_ALL);
    
    if (bind(rawSocket_, (struct sockaddr*)&sll, sizeof(sll)) < 0) {
        close(rawSocket_);
        rawSocket_ = -1;
        return false;
    }
    
    return true;
#else
    return false;
#endif
}

void LoopbackHAL::deinit() {
    if (!initialized_) {
        return;
    }
    
    // Stop worker thread
    if (running_) {
        running_ = false;
        if (workerThread_.joinable()) {
            workerThread_.join();
        }
    }
    
    // Close FIFO descriptors
    if (txFifoFd_ >= 0) {
        close(txFifoFd_);
        txFifoFd_ = -1;
    }
    if (rxFifoFd_ >= 0) {
        close(rxFifoFd_);
        rxFifoFd_ = -1;
    }
    
    // Close raw socket
    if (rawSocket_ >= 0) {
        close(rawSocket_);
        rawSocket_ = -1;
    }
    
    initialized_ = false;
}

bool LoopbackHAL::sendFrame(const uint8_t* data, size_t length) {
    if (!initialized_) {
        return false;
    }
    
    // Log outgoing frame
    logFrame(data, length, false);
    
    // Error injection
    if (shouldDropFrame()) {
        stats_.framesDropped++;
        return true;  // Pretend it was sent
    }
    
    if (shouldInjectCrcError()) {
        // Corrupt the frame
        std::vector<uint8_t> corruptFrame(data, data + length);
        if (length > 0) {
            corruptFrame[length - 1] ^= 0xFF;
        }
        data = corruptFrame.data();
        stats_.framesCrcError++;
    }
    
    applyJitter();
    
    stats_.framesSent++;
    stats_.bytesTransmitted += length;
    
    switch (config_.mode) {
        case LoopbackMode::Direct:
            return processFrameDirect(data, length);
            
        case LoopbackMode::Threaded:
            return processFrameAsync(data, length);
            
        case LoopbackMode::FIFO:
            if (txFifoFd_ >= 0) {
                uint32_t len = static_cast<uint32_t>(length);
                write(txFifoFd_, &len, sizeof(len));
                write(txFifoFd_, data, length);
                return true;
            }
            return false;
            
        case LoopbackMode::Network:
            if (rawSocket_ >= 0) {
                return send(rawSocket_, data, length, 0) == static_cast<ssize_t>(length);
            }
            return false;
    }
    
    return false;
}

bool LoopbackHAL::processFrameDirect(const uint8_t* frame, size_t length) {
    // Simulate processing delay
    if (config_.processingDelayNs > 0) {
        auto start = std::chrono::high_resolution_clock::now();
        while (true) {
            auto now = std::chrono::high_resolution_clock::now();
            auto elapsed = std::chrono::duration_cast<std::chrono::nanoseconds>(now - start).count();
            if (elapsed >= config_.processingDelayNs) break;
        }
    }
    
    // Process through all attached slaves
    std::vector<uint8_t> response(1600);
    size_t responseLen = 0;
    
    // Copy frame as starting point for response
    std::memcpy(response.data(), frame, length);
    responseLen = length;
    
    bool processed = false;
    
    // Process through slaves in order
    for (auto& slave : slaves_) {
        if (slave->processFrame(response.data(), responseLen, 
                                response.data(), responseLen)) {
            processed = true;
        }
    }
    
    // Process through external targets
    for (auto& target : targets_) {
        if (target->processFrame(response.data(), responseLen,
                                 response.data(), responseLen)) {
            processed = true;
        }
    }
    
    // Simulate response delay
    if (config_.responseDelayNs > 0) {
        auto start = std::chrono::high_resolution_clock::now();
        while (true) {
            auto now = std::chrono::high_resolution_clock::now();
            auto elapsed = std::chrono::duration_cast<std::chrono::nanoseconds>(now - start).count();
            if (elapsed >= config_.responseDelayNs) break;
        }
    }
    
    // Queue response
    if (processed) {
        std::lock_guard<std::mutex> lock(responseMutex_);
        responseQueue_.emplace_back(response.begin(), response.begin() + responseLen);
    }
    
    return processed;
}

bool LoopbackHAL::processFrameAsync(const uint8_t* frame, size_t length) {
    // In threaded mode, frame is queued for processing
    // For simplicity, we use direct mode logic here
    return processFrameDirect(frame, length);
}

bool LoopbackHAL::receiveFrame(uint8_t* buffer, size_t bufferSize, 
                                size_t& receivedLength, uint32_t timeoutMs) {
    if (!initialized_) {
        return false;
    }
    
    switch (config_.mode) {
        case LoopbackMode::Direct:
        case LoopbackMode::Threaded:
            {
                // Check response queue
                auto startTime = std::chrono::steady_clock::now();
                
                while (true) {
                    {
                        std::lock_guard<std::mutex> lock(responseMutex_);
                        if (!responseQueue_.empty()) {
                            auto& resp = responseQueue_.front();
                            size_t copyLen = std::min(resp.size(), bufferSize);
                            std::memcpy(buffer, resp.data(), copyLen);
                            receivedLength = copyLen;
                            responseQueue_.erase(responseQueue_.begin());
                            
                            logFrame(buffer, receivedLength, true);
                            stats_.framesReceived++;
                            stats_.bytesReceived += receivedLength;
                            
                            return true;
                        }
                    }
                    
                    auto elapsed = std::chrono::steady_clock::now() - startTime;
                    if (std::chrono::duration_cast<std::chrono::milliseconds>(elapsed).count() >= timeoutMs) {
                        break;
                    }
                    
                    // Small sleep to avoid busy waiting
                    std::this_thread::sleep_for(std::chrono::microseconds(100));
                }
            }
            return false;
            
        case LoopbackMode::FIFO:
            if (rxFifoFd_ >= 0) {
                fd_set fds;
                FD_ZERO(&fds);
                FD_SET(rxFifoFd_, &fds);
                
                struct timeval tv;
                tv.tv_sec = timeoutMs / 1000;
                tv.tv_usec = (timeoutMs % 1000) * 1000;
                
                if (select(rxFifoFd_ + 1, &fds, nullptr, nullptr, &tv) > 0) {
                    uint32_t len;
                    if (read(rxFifoFd_, &len, sizeof(len)) == sizeof(len)) {
                        if (len <= bufferSize) {
                            ssize_t n = read(rxFifoFd_, buffer, len);
                            if (n > 0) {
                                receivedLength = n;
                                logFrame(buffer, receivedLength, true);
                                stats_.framesReceived++;
                                stats_.bytesReceived += receivedLength;
                                return true;
                            }
                        }
                    }
                }
            }
            return false;
            
        case LoopbackMode::Network:
            if (rawSocket_ >= 0) {
                fd_set fds;
                FD_ZERO(&fds);
                FD_SET(rawSocket_, &fds);
                
                struct timeval tv;
                tv.tv_sec = timeoutMs / 1000;
                tv.tv_usec = (timeoutMs % 1000) * 1000;
                
                if (select(rawSocket_ + 1, &fds, nullptr, nullptr, &tv) > 0) {
                    ssize_t n = recv(rawSocket_, buffer, bufferSize, 0);
                    if (n > 0) {
                        receivedLength = n;
                        logFrame(buffer, receivedLength, true);
                        stats_.framesReceived++;
                        stats_.bytesReceived += receivedLength;
                        return true;
                    }
                }
            }
            return false;
    }
    
    return false;
}

bool LoopbackHAL::getMacAddress(uint8_t* mac) {
    std::memcpy(mac, macAddress_, 6);
    return true;
}

bool LoopbackHAL::isLinkUp() {
    return initialized_;
}

void LoopbackHAL::attachSlave(std::shared_ptr<slave::SlaveCore> slave) {
    slaves_.push_back(slave);
}

void LoopbackHAL::attachSlave(std::unique_ptr<slave::SlaveCore> slave) {
    slaves_.push_back(std::move(slave));
}

void LoopbackHAL::attachTarget(std::shared_ptr<ILoopbackTarget> target) {
    targets_.push_back(target);
}

void LoopbackHAL::detachAllSlaves() {
    slaves_.clear();
    targets_.clear();
}

size_t LoopbackHAL::getSlaveCount() const {
    return slaves_.size() + targets_.size();
}

slave::SlaveCore* LoopbackHAL::getSlave(size_t index) {
    if (index < slaves_.size()) {
        return slaves_[index].get();
    }
    return nullptr;
}

const slave::SlaveCore* LoopbackHAL::getSlave(size_t index) const {
    if (index < slaves_.size()) {
        return slaves_[index].get();
    }
    return nullptr;
}

void LoopbackHAL::setPcapLogger(std::shared_ptr<IPcapLogger> logger) {
    pcapLogger_ = logger;
}

void LoopbackHAL::enableLogging(bool enable) {
    loggingEnabled_ = enable;
}

bool LoopbackHAL::switchMode(LoopbackMode newMode) {
    if (newMode == config_.mode) {
        return true;
    }
    
    deinit();
    config_.mode = newMode;
    return init();
}

void LoopbackHAL::workerLoop() {
    // Set realtime priority if configured
#ifdef __linux__
    if (config_.useRealtimeScheduler) {
        struct sched_param param;
        param.sched_priority = config_.realtimePriority;
        pthread_setschedparam(pthread_self(), SCHED_FIFO, &param);
    }
#endif
    
    while (running_) {
        // Process any pending work
        // In this simple implementation, work is done synchronously
        std::this_thread::sleep_for(std::chrono::microseconds(100));
    }
}

bool LoopbackHAL::shouldDropFrame() {
    if (config_.frameDropRate <= 0.0f) {
        return false;
    }
    
    static std::mt19937 rng(std::random_device{}());
    static std::uniform_real_distribution<float> dist(0.0f, 1.0f);
    
    return dist(rng) < config_.frameDropRate;
}

bool LoopbackHAL::shouldInjectCrcError() {
    if (config_.frameCrcErrorRate <= 0.0f) {
        return false;
    }
    
    static std::mt19937 rng(std::random_device{}());
    static std::uniform_real_distribution<float> dist(0.0f, 1.0f);
    
    return dist(rng) < config_.frameCrcErrorRate;
}

void LoopbackHAL::applyJitter() {
    if (config_.jitterMaxNs == 0) {
        return;
    }
    
    static std::mt19937 rng(std::random_device{}());
    std::uniform_int_distribution<uint32_t> dist(0, config_.jitterMaxNs);
    
    uint32_t jitter = dist(rng);
    auto start = std::chrono::high_resolution_clock::now();
    while (true) {
        auto now = std::chrono::high_resolution_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::nanoseconds>(now - start).count();
        if (elapsed >= jitter) break;
    }
}

void LoopbackHAL::logFrame(const uint8_t* frame, size_t length, bool isRx) {
    if (!loggingEnabled_ || !pcapLogger_) {
        return;
    }
    
    pcapLogger_->logFrame(frame, length);
}

// ============================================================================
// SlaveToTargetAdapter Implementation
// ============================================================================

SlaveToTargetAdapter::SlaveToTargetAdapter(std::shared_ptr<slave::SlaveCore> slave)
    : slave_(slave)
{
}

bool SlaveToTargetAdapter::processFrame(const uint8_t* frame, size_t length,
                                         uint8_t* responseBuffer, size_t& responseLength) {
    return slave_->processFrame(frame, length, responseBuffer, responseLength);
}

uint16_t SlaveToTargetAdapter::getConfiguredAddress() const {
    return slave_->getConfiguredAddress();
}

bool SlaveToTargetAdapter::isOperational() const {
    return slave_->getCurrentState() == slave::SlaveState::Op;
}

// ============================================================================
// Factory Functions
// ============================================================================

std::unique_ptr<LoopbackHAL> createLoopbackHAL(const LoopbackHALConfig& config) {
    return std::make_unique<LoopbackHAL>(config);
}

std::unique_ptr<LoopbackHAL> createDirectLoopbackHAL() {
    LoopbackHALConfig config;
    config.mode = LoopbackMode::Direct;
    return std::make_unique<LoopbackHAL>(config);
}

std::unique_ptr<LoopbackHAL> createFIFOLoopbackHAL(const std::string& txPath, const std::string& rxPath) {
    LoopbackHALConfig config;
    config.mode = LoopbackMode::FIFO;
    config.txFifoPath = txPath;
    config.rxFifoPath = rxPath;
    return std::make_unique<LoopbackHAL>(config);
}

std::unique_ptr<LoopbackHAL> createThreadedLoopbackHAL(int priority) {
    LoopbackHALConfig config;
    config.mode = LoopbackMode::Threaded;
    config.realtimePriority = priority;
    return std::make_unique<LoopbackHAL>(config);
}

}  // namespace hal
}  // namespace EtherCAT

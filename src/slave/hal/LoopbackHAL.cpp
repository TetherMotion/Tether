/**
 * @file LoopbackHAL.cpp
 * @brief Loopback HAL implementations for slave testing
 */

#include "slave/hal/ISlaveHAL.hpp"
#include "hal/HALTypes.hpp"
#include <cstring>
#include <mutex>
#include <queue>
#include <thread>
#include <atomic>
#include <condition_variable>
#include <vector>

#ifdef __linux__
#include <sys/socket.h>
#include <sys/stat.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <fcntl.h>
#include <poll.h>
#include <errno.h>
#endif

namespace EtherCAT {
namespace hal = HAL;
namespace slave {

// ============================================================================
// DirectLoopbackHAL - In-process loopback for unit testing
// ============================================================================

class DirectLoopbackHAL : public ISlaveHAL, public ILoopbackTarget {
public:
    DirectLoopbackHAL() = default;
    ~DirectLoopbackHAL() override { shutdown(); }

    HAL::Error init(const SlaveHALConfig& config) override {
        std::lock_guard<std::mutex> lock(mutex_);
        if (initialized_) return HAL::Error::AlreadyInitialized;
        config_ = config;
        macAddress_ = config.macAddress;
        initialized_ = true;
        return HAL::Error::OK;
    }

    void shutdown() override {
        std::lock_guard<std::mutex> lock(mutex_);
        initialized_ = false;
        target_ = nullptr;
    }

    bool isInitialized() const override { return initialized_; }

    hal::Error getMacAddress(hal::MacAddress& mac) const override {
        std::lock_guard<std::mutex> lock(mutex_);
        mac = macAddress_;
        return hal::Error::OK;
    }

    hal::Error setMacAddress(const hal::MacAddress& mac) override {
        std::lock_guard<std::mutex> lock(mutex_);
        macAddress_ = mac;
        return hal::Error::OK;
    }

    HAL::Error transmit(const uint8_t* frame, size_t length) override {
        std::lock_guard<std::mutex> lock(mutex_);
        if (!initialized_) return HAL::Error::NotInitialized;
        stats_.txFrames++;
        stats_.txBytes += length;
        if (target_) return target_->onFrameReceived(frame, length);
        return HAL::Error::OK;
    }

    void setRxCallback(SlaveRxCallback callback, void* userData) override {
        std::lock_guard<std::mutex> lock(mutex_);
        rxCallback_ = callback;
        rxUserData_ = userData;
    }

    int poll(HAL::Milliseconds /*timeoutMs*/) override {
        std::lock_guard<std::mutex> lock(mutex_);
        return static_cast<int>(rxQueue_.size());
    }

    HAL::Error receive(uint8_t* buffer, size_t bufferSize, 
                       size_t& receivedLength, HAL::Milliseconds /*timeoutMs*/) override {
        std::lock_guard<std::mutex> lock(mutex_);
        if (rxQueue_.empty()) {
            receivedLength = 0;
            return HAL::Error::WouldBlock;
        }
        auto& front = rxQueue_.front();
        if (front.size() > bufferSize) return HAL::Error::BufferTooSmall;
        std::memcpy(buffer, front.data(), front.size());
        receivedLength = front.size();
        rxQueue_.pop();
        return HAL::Error::OK;
    }

    hal::LinkStatus getLinkStatus() const override {
        hal::LinkStatus s;
        s.up = initialized_;
        s.speedMbps = 1000;
        s.fullDuplex = true;
        return s;
    }

    hal::Error waitForLinkUp(hal::Milliseconds /*timeoutMs*/) override {
        return initialized_ ? hal::Error::OK : hal::Error::LinkDown;
    }

    Stats getStats() const override {
        std::lock_guard<std::mutex> lock(mutex_);
        return stats_;
    }

    void resetStats() override {
        std::lock_guard<std::mutex> lock(mutex_);
        stats_ = Stats{};
    }

    hal::Error enablePcapLogging(const hal::PcapLoggerConfig& /*config*/) override {
        return hal::Error::OK;
    }
    void disablePcapLogging() override {}
    bool isPcapLoggingEnabled() const override { return false; }
    hal::IPcapLogger* getPcapLogger() override { return nullptr; }

    // ILoopbackTarget
    HAL::Error onFrameReceived(const uint8_t* frame, size_t length) override {
        std::lock_guard<std::mutex> lock(mutex_);
        stats_.rxFrames++;
        stats_.rxBytes += length;
        if (rxCallback_) {
            rxCallback_(frame, length, rxUserData_);
        } else {
            rxQueue_.push(std::vector<uint8_t>(frame, frame + length));
        }
        return HAL::Error::OK;
    }

    void connect(ILoopbackTarget* target) {
        std::lock_guard<std::mutex> lock(mutex_);
        target_ = target;
    }

private:
    mutable std::mutex mutex_;
    bool initialized_ = false;
    SlaveHALConfig config_;
    hal::MacAddress macAddress_{};
    ILoopbackTarget* target_ = nullptr;
    SlaveRxCallback rxCallback_;
    void* rxUserData_ = nullptr;
    std::queue<std::vector<uint8_t>> rxQueue_;
    Stats stats_;
};

// ============================================================================
// FIFOLoopbackHAL - POSIX FIFO based IPC
// ============================================================================

#ifdef __linux__
class FIFOLoopbackHAL : public ISlaveHAL {
public:
    FIFOLoopbackHAL() = default;
    ~FIFOLoopbackHAL() override { shutdown(); }

    HAL::Error init(const SlaveHALConfig& config) override {
        std::lock_guard<std::mutex> lock(mutex_);
        if (initialized_) return HAL::Error::AlreadyInitialized;
        config_ = config;
        macAddress_ = config.macAddress;
        
        // Create FIFOs if they don't exist
        mkfifo(config.fifoRxPath.c_str(), 0666);
        mkfifo(config.fifoTxPath.c_str(), 0666);
        
        // Open read FIFO (non-blocking)
        rxFd_ = open(config.fifoRxPath.c_str(), O_RDONLY | O_NONBLOCK);
        if (rxFd_ < 0) return HAL::Error::ConfigurationFailed;
        
        // Open write FIFO
        txFd_ = open(config.fifoTxPath.c_str(), O_WRONLY | O_NONBLOCK);
        if (txFd_ < 0) {
            close(rxFd_);
            rxFd_ = -1;
            return HAL::Error::ConfigurationFailed;
        }
        
        initialized_ = true;
        return HAL::Error::OK;
    }

    void shutdown() override {
        std::lock_guard<std::mutex> lock(mutex_);
        if (rxFd_ >= 0) { close(rxFd_); rxFd_ = -1; }
        if (txFd_ >= 0) { close(txFd_); txFd_ = -1; }
        initialized_ = false;
    }

    bool isInitialized() const override { return initialized_; }

    hal::Error getMacAddress(hal::MacAddress& mac) const override {
        std::lock_guard<std::mutex> lock(mutex_);
        mac = macAddress_;
        return hal::Error::OK;
    }

    hal::Error setMacAddress(const hal::MacAddress& mac) override {
        std::lock_guard<std::mutex> lock(mutex_);
        macAddress_ = mac;
        return hal::Error::OK;
    }

    hal::Error transmit(const uint8_t* frame, size_t length) override {
        std::lock_guard<std::mutex> lock(mutex_);
        if (!initialized_) return hal::Error::NotInitialized;
        if (txFd_ < 0) return hal::Error::NotInitialized;
        
        ssize_t written = write(txFd_, frame, length);
        if (written < 0) {
            stats_.txErrors++;
            return hal::Error::TransmitFailed;
        }
        stats_.txFrames++;
        stats_.txBytes += length;
        return hal::Error::OK;
    }

    void setRxCallback(SlaveRxCallback callback, void* userData) override {
        std::lock_guard<std::mutex> lock(mutex_);
        rxCallback_ = callback;
        rxUserData_ = userData;
    }

    int poll(HAL::Milliseconds timeoutMs) override {
        if (rxFd_ < 0) return -1;
        
        struct pollfd pfd;
        pfd.fd = rxFd_;
        pfd.events = POLLIN;
        
        int ret = ::poll(&pfd, 1, timeoutMs);
        if (ret > 0 && (pfd.revents & POLLIN)) {
            uint8_t buffer[2048];
            ssize_t len = read(rxFd_, buffer, sizeof(buffer));
            if (len > 0) {
                stats_.rxFrames++;
                stats_.rxBytes += len;
                if (rxCallback_) {
                    rxCallback_(buffer, len, rxUserData_);
                }
                return 1;
            }
        }
        return 0;
    }

    hal::Error receive(uint8_t* buffer, size_t bufferSize, 
                       size_t& receivedLength, hal::Milliseconds timeoutMs) override {
        if (rxFd_ < 0) return hal::Error::NotInitialized;
        
        struct pollfd pfd;
        pfd.fd = rxFd_;
        pfd.events = POLLIN;
        
        int ret = ::poll(&pfd, 1, timeoutMs);
        if (ret <= 0) {
            receivedLength = 0;
            return ret == 0 ? hal::Error::Timeout : hal::Error::ReceiveFailed;
        }
        
        ssize_t len = read(rxFd_, buffer, bufferSize);
        if (len <= 0) {
            receivedLength = 0;
            return HAL::Error::WouldBlock;
        }
        
        receivedLength = len;
        stats_.rxFrames++;
        stats_.rxBytes += len;
        return hal::Error::OK;
    }

    hal::LinkStatus getLinkStatus() const override {
        hal::LinkStatus s;
        s.up = initialized_;
        s.speedMbps = 1000;
        s.fullDuplex = true;
        return s;
    }

    hal::Error waitForLinkUp(hal::Milliseconds /*timeoutMs*/) override {
        return initialized_ ? hal::Error::OK : hal::Error::LinkDown;
    }

    Stats getStats() const override {
        std::lock_guard<std::mutex> lock(mutex_);
        return stats_;
    }

    void resetStats() override {
        std::lock_guard<std::mutex> lock(mutex_);
        stats_ = Stats{};
    }

    hal::Error enablePcapLogging(const hal::PcapLoggerConfig& /*config*/) override {
        return hal::Error::OK;
    }
    void disablePcapLogging() override {}
    bool isPcapLoggingEnabled() const override { return false; }
    hal::IPcapLogger* getPcapLogger() override { return nullptr; }

private:
    mutable std::mutex mutex_;
    bool initialized_ = false;
    SlaveHALConfig config_;
    hal::MacAddress macAddress_{};
    int rxFd_ = -1;
    int txFd_ = -1;
    SlaveRxCallback rxCallback_;
    void* rxUserData_ = nullptr;
    Stats stats_;
};
#endif

// ============================================================================
// Factory Functions
// ============================================================================

std::unique_ptr<ISlaveHAL> createDirectSlaveHAL() {
    return std::make_unique<DirectLoopbackHAL>();
}

std::unique_ptr<ISlaveHAL> createFIFOSlaveHAL() {
#ifdef __linux__
    return std::make_unique<FIFOLoopbackHAL>();
#else
    return nullptr;
#endif
}

std::unique_ptr<ISlaveHAL> createNetworkSlaveHAL() {
    // Network HAL requires platform-specific implementation
    return nullptr;
}

}  // namespace slave
}  // namespace EtherCAT

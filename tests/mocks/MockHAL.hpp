/**
 * @file MockHAL.hpp
 * @brief Mock implementations of HAL interfaces for unit testing
 *
 * Provides MockEthernet (gmock), FakeEthernet (functional), MockClock, FakeClock,
 * and other test doubles matching the HAL interface definitions.
 */

#pragma once

#include "tether/hal/HAL.hpp"
#include "tether/hal/IEthernet.hpp"
#include "tether/hal/IThreading.hpp"
#include "tether/hal/IClock.hpp"
#include "tether/hal/ILogger.hpp"
#include "tether/hal/IPcapLogger.hpp"
#include "tether/hal/StateMachineLogger.hpp"

#include <gmock/gmock.h>
#include <queue>
#include <mutex>
#include <condition_variable>
#include <functional>
#include <chrono>

namespace EtherCAT {
namespace HAL {
namespace mock {

// ============================================================================
// Mock Ethernet - GMock based
// ============================================================================

class MockEthernet : public IEthernet {
public:
    MOCK_METHOD(Error, init, (const EthernetConfig& config), (override));
    MOCK_METHOD(void, shutdown, (), (override));
    MOCK_METHOD(bool, isInitialized, (), (const, override));
    MOCK_METHOD(Error, getMacAddress, (MacAddress& mac), (const, override));
    MOCK_METHOD(Error, setMacAddress, (const MacAddress& mac), (override));
    MOCK_METHOD(Error, transmit, (const uint8_t* frame, size_t length), (override));
    MOCK_METHOD(Error, transmitVlan, (const uint8_t* frame, size_t length, uint16_t vlanId, uint8_t priority), (override));
    MOCK_METHOD(Error, transmitGather, (const BufferDesc* iov, size_t count), (override));
    MOCK_METHOD(void, setRxCallback, (RxCallback callback, void* userData), (override));
    MOCK_METHOD(int, poll, (Milliseconds timeoutMs), (override));
    MOCK_METHOD(void, setEthertypeFilter, (uint16_t ethertype), (override));
    MOCK_METHOD(Error, setPromiscuous, (bool enable), (override));
    MOCK_METHOD(Error, addMulticastAddress, (const MacAddress& mac), (override));
    MOCK_METHOD(Error, removeMulticastAddress, (const MacAddress& mac), (override));
    MOCK_METHOD(Error, setAllMulticast, (bool enable), (override));
    MOCK_METHOD(LinkStatus, getLinkStatus, (), (const, override));
    MOCK_METHOD(void, setLinkCallback, (LinkCallback callback, void* userData), (override));
    MOCK_METHOD(Error, waitForLinkUp, (Milliseconds timeoutMs), (override));
    MOCK_METHOD(EthernetStats, getStats, (), (const, override));
    MOCK_METHOD(void, resetStats, (), (override));
    MOCK_METHOD(void*, nativeHandle, (), (override));
    MOCK_METHOD(const char*, getInterfaceName, (), (const, override));
};

// ============================================================================
// Fake Ethernet - Functional fake for integration tests
// ============================================================================

class FakeEthernet : public IEthernet {
public:
    FakeEthernet() 
        : m_initialized(false)
        , m_promiscuous(false)
        , m_callback(nullptr)
        , m_callbackData(nullptr)
    {
        m_mac = MacAddress(0x02, 0x00, 0x00, 0x00, 0x00, 0x01);
        m_linkStatus.up = true;
        m_linkStatus.speedMbps = 100;
        m_linkStatus.fullDuplex = true;
    }

    Error init(const EthernetConfig& config) override {
        (void)config;
        m_initialized = true;
        return Error::OK;
    }

    void shutdown() override {
        m_initialized = false;
    }

    bool isInitialized() const override {
        return m_initialized;
    }

    Error getMacAddress(MacAddress& mac) const override {
        mac = m_mac;
        return Error::OK;
    }

    Error setMacAddress(const MacAddress& mac) override {
        m_mac = mac;
        return Error::OK;
    }

    Error transmit(const uint8_t* frame, size_t length) override {
        std::lock_guard<std::mutex> lock(m_mutex);
        m_txFrames.emplace(std::vector<uint8_t>(frame, frame + length));
        m_stats.txFrames++;
        m_stats.txBytes += length;
        return Error::OK;
    }

    Error transmitVlan(const uint8_t* frame, size_t length, 
                       uint16_t vlanId, uint8_t priority) override {
        (void)vlanId; (void)priority;
        return transmit(frame, length);  // Simplified
    }

    Error transmitGather(const BufferDesc* iov, size_t count) override {
        std::vector<uint8_t> frame;
        for (size_t i = 0; i < count; i++) {
            frame.insert(frame.end(), iov[i].data, iov[i].data + iov[i].length);
        }
        return transmit(frame.data(), frame.size());
    }

    void setRxCallback(RxCallback callback, void* userData) override {
        m_callback = callback;
        m_callbackData = userData;
    }

    int poll(Milliseconds timeoutMs) override {
        std::unique_lock<std::mutex> lock(m_mutex);
        
        if (m_rxFrames.empty()) {
            if (timeoutMs == 0) return 0;
            m_rxCv.wait_for(lock, std::chrono::milliseconds(timeoutMs), [this]() {
                return !m_rxFrames.empty();
            });
        }
        
        int count = 0;
        while (!m_rxFrames.empty()) {
            auto& frameData = m_rxFrames.front();
            if (m_callback) {
                RxFrameInfo info;
                info.timestamp = std::chrono::duration_cast<std::chrono::microseconds>(
                    std::chrono::steady_clock::now().time_since_epoch()).count();
                lock.unlock();
                m_callback(frameData.data(), frameData.size(), info, m_callbackData);
                lock.lock();
            }
            m_stats.rxFrames++;
            m_stats.rxBytes += frameData.size();
            m_rxFrames.pop();
            count++;
        }
        return count;
    }

    void setEthertypeFilter(uint16_t ethertype) override {
        m_ethertypeFilter = ethertype;
    }

    Error setPromiscuous(bool enable) override {
        m_promiscuous = enable;
        return Error::OK;
    }

    Error addMulticastAddress(const MacAddress& mac) override {
        m_multicastAddresses.push_back(mac);
        return Error::OK;
    }

    Error removeMulticastAddress(const MacAddress& mac) override {
        m_multicastAddresses.erase(
            std::remove_if(m_multicastAddresses.begin(), m_multicastAddresses.end(),
                           [&mac](const MacAddress& m) {
                               return std::memcmp(m.bytes, mac.bytes, 6) == 0;
                           }),
            m_multicastAddresses.end());
        return Error::OK;
    }

    Error setAllMulticast(bool enable) override {
        m_allMulticast = enable;
        return Error::OK;
    }

    LinkStatus getLinkStatus() const override {
        return m_linkStatus;
    }

    void setLinkCallback(LinkCallback callback, void* userData) override {
        m_linkCallback = callback;
        m_linkCallbackData = userData;
    }

    Error waitForLinkUp(Milliseconds timeoutMs) override {
        (void)timeoutMs;
        return m_linkStatus.up ? Error::OK : Error::Timeout;
    }

    EthernetStats getStats() const override {
        return m_stats;
    }

    void resetStats() override {
        m_stats = EthernetStats{};
    }

    void* nativeHandle() override {
        return nullptr;
    }

    const char* getInterfaceName() const override {
        return "fake0";
    }

    // =====================================================================
    // Test Helpers
    // =====================================================================

    void injectFrame(const uint8_t* frame, size_t length) {
        {
            std::lock_guard<std::mutex> lock(m_mutex);
            m_rxFrames.emplace(std::vector<uint8_t>(frame, frame + length));
        }
        m_rxCv.notify_one();
    }

    std::vector<uint8_t> popTxFrame() {
        std::lock_guard<std::mutex> lock(m_mutex);
        if (m_txFrames.empty()) return {};
        auto frame = m_txFrames.front();
        m_txFrames.pop();
        return frame;
    }

    size_t getTxQueueSize() const {
        std::lock_guard<std::mutex> lock(m_mutex);
        return m_txFrames.size();
    }

    void setLinkUp(bool up) {
        m_linkStatus.up = up;
        if (m_linkCallback) {
            m_linkCallback(m_linkStatus, m_linkCallbackData);
        }
    }

private:
    bool m_initialized;
    bool m_promiscuous;
    bool m_allMulticast = false;
    uint16_t m_ethertypeFilter = 0;
    MacAddress m_mac;
    LinkStatus m_linkStatus;
    EthernetStats m_stats;
    RxCallback m_callback;
    void* m_callbackData;
    LinkCallback m_linkCallback = nullptr;
    void* m_linkCallbackData = nullptr;
    std::vector<MacAddress> m_multicastAddresses;
    
    mutable std::mutex m_mutex;
    std::condition_variable m_rxCv;
    std::queue<std::vector<uint8_t>> m_rxFrames;
    std::queue<std::vector<uint8_t>> m_txFrames;
};

// ============================================================================
// Mock Clock - GMock based
// ============================================================================

class MockClock : public IClock {
public:
    MOCK_METHOD(Timestamp, nowMicros, (), (override));
    MOCK_METHOD(Timestamp, nowNanos, (), (override));
    MOCK_METHOD(Timestamp, systemTimeMillis, (), (override));
    MOCK_METHOD(Nanoseconds, resolution, (), (override));
    MOCK_METHOD(void, delayMicros, (Microseconds us), (override));
    MOCK_METHOD(void, delayMillis, (Milliseconds ms), (override));
};

// ============================================================================
// Fake Clock - Controllable time for deterministic tests
// ============================================================================

class FakeClock : public IClock {
public:
    FakeClock() : m_time(0), m_systemTime(1700000000000) {}  // Start at some reasonable time

    Timestamp nowMicros() override {
        return m_time;
    }

    Timestamp nowNanos() override {
        return m_time * 1000;
    }

    Timestamp systemTimeMillis() override {
        return m_systemTime;
    }

    Nanoseconds resolution() override {
        return 1000;  // 1 microsecond
    }

    void delayMicros(Microseconds us) override {
        m_time += us;
    }

    void delayMillis(Milliseconds ms) override {
        m_time += ms * 1000;
    }

    // Test helpers
    void setTime(Timestamp us) { m_time = us; }
    void advance(Microseconds us) { m_time += us; }
    void setSystemTime(Timestamp ms) { m_systemTime = ms; }

private:
    Timestamp m_time;
    Timestamp m_systemTime;
};

// ============================================================================
// Mock Periodic Timer
// ============================================================================

class MockPeriodicTimer : public IPeriodicTimer {
public:
    MOCK_METHOD(bool, init, (uint32_t frequencyHz), (override));
    MOCK_METHOD(void, start, (), (override));
    MOCK_METHOD(void, stop, (), (override));
    MOCK_METHOD(bool, isRunning, (), (const, override));
    MOCK_METHOD(void, waitForCycle, (), (override));
    MOCK_METHOD(void, setCallback, (TimerCallback callback), (override));
    MOCK_METHOD(Microseconds, getPeriodMicros, (), (const, override));
    MOCK_METHOD(Stats, getStats, (), (const, override));
    MOCK_METHOD(void, resetStats, (), (override));
};

// ============================================================================
// Fake Periodic Timer
// ============================================================================

class FakePeriodicTimer : public IPeriodicTimer {
public:
    FakePeriodicTimer() : m_running(false), m_periodUs(1000), m_tickCount(0) {}

    bool init(uint32_t frequencyHz) override {
        if (frequencyHz == 0) return false;
        m_periodUs = 1000000 / frequencyHz;
        return true;
    }

    void start() override { m_running = true; }
    void stop() override { m_running = false; }
    bool isRunning() const override { return m_running; }

    void waitForCycle() override {
        // In tests, just increment tick count
        m_tickCount++;
    }

    void setCallback(TimerCallback callback) override {
        m_callback = callback;
    }

    Microseconds getPeriodMicros() const override {
        return m_periodUs;
    }

    Stats getStats() const override {
        Stats s;
        s.tickCount = m_tickCount;
        return s;
    }

    void resetStats() override {
        m_tickCount = 0;
    }

    // Test helper to trigger callback
    void tick() {
        if (m_running && m_callback) {
            m_callback();
        }
        m_tickCount++;
    }

private:
    bool m_running;
    Microseconds m_periodUs;
    uint64_t m_tickCount;
    TimerCallback m_callback;
};

// ============================================================================
// Mock PcapLogger
// ============================================================================

class MockPcapLogger : public IPcapLogger {
public:
    MOCK_METHOD(Error, init, (const PcapLoggerConfig& config), (override));
    MOCK_METHOD(void, close, (), (override));
    MOCK_METHOD(bool, isOpen, (), (const, override));
    MOCK_METHOD(Error, logFrame, (const uint8_t* frame, size_t length, FrameDirection dir, Timestamp ts), (override));
    MOCK_METHOD(Error, logFrameWithInfo, (const uint8_t* frame, size_t length, FrameDirection dir, const RxFrameInfo& info), (override));
    MOCK_METHOD(void, flush, (), (override));
    MOCK_METHOD(uint64_t, getFrameCount, (), (const, override));
    MOCK_METHOD(size_t, getFileSize, (), (const, override));
    MOCK_METHOD(Stats, getStats, (), (const, override));
};

// ============================================================================
// Fake PcapLogger - Records frames in memory
// ============================================================================

class FakePcapLogger : public IPcapLogger {
public:
    struct LoggedFrame {
        std::vector<uint8_t> data;
        FrameDirection direction;
        Timestamp timestamp;
    };

    FakePcapLogger() : m_open(false) {}

    Error init(const PcapLoggerConfig& config) override {
        m_config = config;
        m_open = true;
        m_frames.clear();
        return Error::OK;
    }

    void close() override {
        m_open = false;
    }

    bool isOpen() const override {
        return m_open;
    }

    Error logFrame(const uint8_t* frame, size_t length, 
                   FrameDirection dir, Timestamp ts) override {
        if (!m_open) return Error::NotInitialized;
        m_frames.push_back({
            std::vector<uint8_t>(frame, frame + length),
            dir, ts
        });
        return Error::OK;
    }

    Error logFrameWithInfo(const uint8_t* frame, size_t length,
                           FrameDirection dir, const RxFrameInfo& info) override {
        return logFrame(frame, length, dir, info.timestamp);
    }

    void flush() override {}

    uint64_t getFrameCount() const override {
        return m_frames.size();
    }

    size_t getFileSize() const override {
        return 0;  // Not a real file
    }

    Stats getStats() const override {
        Stats s;
        for (const auto& f : m_frames) {
            if (f.direction == FrameDirection::Tx) s.txFrames++;
            else s.rxFrames++;
            s.totalBytes += f.data.size();
        }
        return s;
    }

    // Test helper
    const std::vector<LoggedFrame>& getFrames() const { return m_frames; }
    void clearFrames() { m_frames.clear(); }

private:
    bool m_open;
    PcapLoggerConfig m_config;
    std::vector<LoggedFrame> m_frames;
};

// ============================================================================
// Mock State Machine Logger
// ============================================================================

class MockStateMachineLogger : public IStateMachineLogger {
public:
    MOCK_METHOD(Error, init, (const StateMachineLoggerConfig& config), (override));
    MOCK_METHOD(void, recordTransition, (const StateTransition& transition), (override));
    MOCK_METHOD(void, setCallback, (StateTransitionCallback callback), (override));
    MOCK_METHOD(std::vector<StateTransition>, getHistory, (uint16_t slaveIndex), (const, override));
    MOCK_METHOD(std::vector<StateTransition>, getAllHistory, (), (const, override));
    MOCK_METHOD(void, clearHistory, (), (override));
    MOCK_METHOD(ALState, getCurrentState, (uint16_t slaveIndex), (const, override));
    MOCK_METHOD(size_t, getSlaveCount, (), (const, override));
    MOCK_METHOD(Stats, getStats, (), (const, override));
    MOCK_METHOD(void, setLogLevel, (LogLevel level), (override));
};

} // namespace mock
} // namespace HAL
} // namespace EtherCAT

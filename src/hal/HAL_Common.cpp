/**
 * @file HAL_Common.cpp
 * @brief Common HAL implementation (platform-independent)
 *
 * This file contains implementations matching the interfaces in:
 * - IEthernet.hpp (VlanEthernetWrapper, TrafficSplitter, LoggingEthernetWrapper)
 * - HAL.hpp (HALInstance)
 *
 * PcapNGLoggerImpl is in PcapNGLogger.cpp
 * StateMachineLoggerImpl is in StateMachineLogger.cpp
 */

#include "hal/HAL.hpp"
#include "hal/IEthernet.hpp"
#include "hal/IThreading.hpp"
#include "hal/IClock.hpp"
#include "hal/ILogger.hpp"
#include "hal/IPcapLogger.hpp"
#include "hal/StateMachineLogger.hpp"
#include "hal/HALTypes.hpp"

#include <cstring>
#include <cstdio>
#include <atomic>
#include <vector>
#include <algorithm>
#include <mutex>
#include <string>

namespace EtherCAT {
namespace HAL {

// Platform-specific factory function (defined in LinuxEthernet.cpp, ESP32Ethernet.cpp, etc.)
extern std::unique_ptr<IEthernet> createDefaultEthernet();

// ============================================================================
// VlanEthernetWrapper Implementation
// ============================================================================

VlanEthernetWrapper::VlanEthernetWrapper(std::unique_ptr<IEthernet> inner,
                                         uint16_t vlanId, uint8_t priority,
                                         bool stripTags)
    : m_inner(std::move(inner))
    , m_vlanId(vlanId)
    , m_vlanPriority(priority)
    , m_stripTags(stripTags)
    , m_userCallback(nullptr)
    , m_userData(nullptr)
{}

VlanEthernetWrapper::~VlanEthernetWrapper() = default;

Error VlanEthernetWrapper::init(const EthernetConfig& config) {
    return m_inner ? m_inner->init(config) : Error::NotInitialized;
}

void VlanEthernetWrapper::shutdown() {
    if (m_inner) m_inner->shutdown();
}

bool VlanEthernetWrapper::isInitialized() const {
    return m_inner && m_inner->isInitialized();
}

Error VlanEthernetWrapper::getMacAddress(MacAddress& mac) const {
    return m_inner ? m_inner->getMacAddress(mac) : Error::NotInitialized;
}

Error VlanEthernetWrapper::setMacAddress(const MacAddress& mac) {
    return m_inner ? m_inner->setMacAddress(mac) : Error::NotInitialized;
}

// Insert 4-byte VLAN tag after dst+src MAC addresses
static size_t insertVlanTag(const uint8_t* frame, size_t len,
                            uint8_t* out, size_t outSize,
                            uint16_t vlanId, uint8_t priority) {
    static constexpr uint16_t VLAN_ETHERTYPE = 0x8100;
    
    if (len < 14 || len + 4 > outSize) return 0;
    
    // Copy dst + src MAC (12 bytes)
    std::memcpy(out, frame, 12);
    
    // Build VLAN tag
    uint16_t tci = ((priority & 0x07) << 13) | (vlanId & 0x0FFF);
    
    // Insert VLAN tag
    out[12] = VLAN_ETHERTYPE >> 8;
    out[13] = VLAN_ETHERTYPE & 0xFF;
    out[14] = tci >> 8;
    out[15] = tci & 0xFF;
    
    // Copy rest of frame (original EtherType + payload)
    std::memcpy(out + 16, frame + 12, len - 12);
    
    return len + 4;
}

// Strip VLAN tag if present
static size_t stripVlanTag(const uint8_t* frame, size_t len,
                           uint8_t* out, size_t outSize,
                           RxFrameInfo& info) {
    static constexpr uint16_t VLAN_ETHERTYPE = 0x8100;
    
    if (len < 18) return 0;  // Too short for VLAN-tagged frame
    
    uint16_t etherType = (frame[12] << 8) | frame[13];
    if (etherType != VLAN_ETHERTYPE) return 0;  // Not VLAN-tagged
    
    // Extract VLAN info
    uint16_t tci = (frame[14] << 8) | frame[15];
    info.vlanId = tci & 0x0FFF;
    info.vlanPriority = (tci >> 13) & 0x07;
    info.vlanTagPresent = true;
    
    size_t outLen = len - 4;
    if (outLen > outSize) return 0;
    
    // Copy dst + src MAC
    std::memcpy(out, frame, 12);
    // Copy original EtherType + payload (skip VLAN tag)
    std::memcpy(out + 12, frame + 16, len - 16);
    
    return outLen;
}

Error VlanEthernetWrapper::transmit(const uint8_t* frame, size_t length) {
    if (!m_inner) return Error::NotInitialized;
    
    // Add VLAN tag to frame
    uint8_t vlanFrame[1522];  // Max frame + VLAN tag
    size_t vlanLen = insertVlanTag(frame, length, vlanFrame, sizeof(vlanFrame),
                                   m_vlanId, m_vlanPriority);
    if (vlanLen == 0) return Error::InvalidArgument;
    
    return m_inner->transmit(vlanFrame, vlanLen);
}

Error VlanEthernetWrapper::transmitVlan(const uint8_t* frame, size_t length,
                                        uint16_t vlanId, uint8_t priority) {
    return m_inner ? m_inner->transmitVlan(frame, length, vlanId, priority)
                   : Error::NotInitialized;
}

Error VlanEthernetWrapper::transmitGather(const BufferDesc* iov, size_t count) {
    if (!m_inner) return Error::NotInitialized;
    
    // Build complete frame first, then add VLAN tag
    uint8_t tempFrame[1518];
    size_t totalLen = 0;
    
    for (size_t i = 0; i < count && totalLen < sizeof(tempFrame); i++) {
        size_t copyLen = std::min(iov[i].length, sizeof(tempFrame) - totalLen);
        std::memcpy(tempFrame + totalLen, iov[i].data, copyLen);
        totalLen += copyLen;
    }
    
    return transmit(tempFrame, totalLen);
}

void VlanEthernetWrapper::handleRx(const uint8_t* frame, size_t length,
                                   const RxFrameInfo& info, void* userData) {
    (void)userData;
    
    if (!m_userCallback) return;
    
    if (m_stripTags) {
        uint8_t strippedFrame[1518];
        RxFrameInfo newInfo = info;
        size_t strippedLen = stripVlanTag(frame, length, strippedFrame, 
                                          sizeof(strippedFrame), newInfo);
        if (strippedLen > 0) {
            m_userCallback(strippedFrame, strippedLen, newInfo, m_userData);
            return;
        }
    }
    
    // Pass through as-is
    m_userCallback(frame, length, info, m_userData);
}

void VlanEthernetWrapper::setRxCallback(RxCallback callback, void* userData) {
    m_userCallback = callback;
    m_userData = userData;
    
    if (m_inner) {
        if (callback) {
            m_inner->setRxCallback(
                [this](const uint8_t* f, size_t l, const RxFrameInfo& i, void* u) {
                    this->handleRx(f, l, i, u);
                }, this);
        } else {
            m_inner->setRxCallback(nullptr, nullptr);
        }
    }
}

int VlanEthernetWrapper::poll(Milliseconds timeoutMs) {
    return m_inner ? m_inner->poll(timeoutMs) : 0;
}

void VlanEthernetWrapper::setEthertypeFilter(uint16_t ethertype) {
    if (m_inner) m_inner->setEthertypeFilter(ethertype);
}

Error VlanEthernetWrapper::setPromiscuous(bool enable) {
    return m_inner ? m_inner->setPromiscuous(enable) : Error::NotInitialized;
}

Error VlanEthernetWrapper::addMulticastAddress(const MacAddress& mac) {
    return m_inner ? m_inner->addMulticastAddress(mac) : Error::NotInitialized;
}

Error VlanEthernetWrapper::removeMulticastAddress(const MacAddress& mac) {
    return m_inner ? m_inner->removeMulticastAddress(mac) : Error::NotInitialized;
}

Error VlanEthernetWrapper::setAllMulticast(bool enable) {
    return m_inner ? m_inner->setAllMulticast(enable) : Error::NotInitialized;
}

LinkStatus VlanEthernetWrapper::getLinkStatus() const {
    return m_inner ? m_inner->getLinkStatus() : LinkStatus{};
}

void VlanEthernetWrapper::setLinkCallback(LinkCallback callback, void* userData) {
    if (m_inner) m_inner->setLinkCallback(callback, userData);
}

Error VlanEthernetWrapper::waitForLinkUp(Milliseconds timeoutMs) {
    return m_inner ? m_inner->waitForLinkUp(timeoutMs) : Error::NotInitialized;
}

EthernetStats VlanEthernetWrapper::getStats() const {
    return m_inner ? m_inner->getStats() : EthernetStats{};
}

void VlanEthernetWrapper::resetStats() {
    if (m_inner) m_inner->resetStats();
}

void* VlanEthernetWrapper::nativeHandle() {
    return m_inner ? m_inner->nativeHandle() : nullptr;
}

const char* VlanEthernetWrapper::getInterfaceName() const {
    return m_inner ? m_inner->getInterfaceName() : "";
}

void VlanEthernetWrapper::setDefaultVlan(uint16_t vlanId, uint8_t priority) {
    m_vlanId = vlanId;
    m_vlanPriority = priority;
}

void VlanEthernetWrapper::setTagStripping(bool strip) {
    m_stripTags = strip;
}

// ============================================================================
// TrafficSplitter Implementation
// ============================================================================

TrafficSplitter::TrafficSplitter(std::unique_ptr<IEthernet> inner)
    : m_inner(std::move(inner))
    , m_defaultCallback(nullptr)
    , m_defaultUserData(nullptr)
    , m_nextRuleId(1)
{}

TrafficSplitter::~TrafficSplitter() = default;

int TrafficSplitter::addRule(const TrafficRule& rule) {
    int id = m_nextRuleId++;
    m_rules.push_back({id, rule});
    
    // Sort by priority (descending)
    std::sort(m_rules.begin(), m_rules.end(),
              [](const auto& a, const auto& b) {
                  return a.second.priority > b.second.priority;
              });
    
    return id;
}

void TrafficSplitter::removeRule(int ruleId) {
    m_rules.erase(
        std::remove_if(m_rules.begin(), m_rules.end(),
                       [ruleId](const auto& p) { return p.first == ruleId; }),
        m_rules.end());
}

void TrafficSplitter::setDefaultCallback(RxCallback callback, void* userData) {
    m_defaultCallback = callback;
    m_defaultUserData = userData;
}

void TrafficSplitter::handleRx(const uint8_t* frame, size_t length,
                               const RxFrameInfo& info, void* userData) {
    (void)userData;
    
    if (length < 14) return;
    
    // Extract frame info
    uint16_t ethertype = (frame[12] << 8) | frame[13];
    
    // Try each rule in priority order
    for (const auto& [id, rule] : m_rules) {
        (void)id;
        
        // Check EtherType match
        if (rule.ethertype != 0 && rule.ethertype != ethertype) continue;
        
        // Check destination MAC match
        if (rule.matchDstMac) {
            if (std::memcmp(frame, rule.dstMac.bytes, 6) != 0) continue;
        }
        
        // Check VLAN match
        if (rule.vlanId != 0 && info.vlanId != rule.vlanId) continue;
        
        // Rule matched - call callback
        if (rule.callback) {
            rule.callback(frame, length, info, rule.userData);
        }
        return;  // Stop at first match
    }
    
    // No rule matched - use default
    if (m_defaultCallback) {
        m_defaultCallback(frame, length, info, m_defaultUserData);
    }
}

Error TrafficSplitter::init(const EthernetConfig& config) {
    return m_inner ? m_inner->init(config) : Error::NotInitialized;
}

void TrafficSplitter::shutdown() {
    if (m_inner) m_inner->shutdown();
}

bool TrafficSplitter::isInitialized() const {
    return m_inner && m_inner->isInitialized();
}

Error TrafficSplitter::getMacAddress(MacAddress& mac) const {
    return m_inner ? m_inner->getMacAddress(mac) : Error::NotInitialized;
}

Error TrafficSplitter::setMacAddress(const MacAddress& mac) {
    return m_inner ? m_inner->setMacAddress(mac) : Error::NotInitialized;
}

Error TrafficSplitter::transmit(const uint8_t* frame, size_t length) {
    return m_inner ? m_inner->transmit(frame, length) : Error::NotInitialized;
}

Error TrafficSplitter::transmitVlan(const uint8_t* frame, size_t length,
                                    uint16_t vlanId, uint8_t priority) {
    return m_inner ? m_inner->transmitVlan(frame, length, vlanId, priority)
                   : Error::NotInitialized;
}

Error TrafficSplitter::transmitGather(const BufferDesc* iov, size_t count) {
    return m_inner ? m_inner->transmitGather(iov, count) : Error::NotInitialized;
}

void TrafficSplitter::setRxCallback(RxCallback callback, void* userData) {
    (void)callback;
    (void)userData;
    // Traffic splitter routes frames via rules, not a single callback
    // But we need to register our handler with inner
    if (m_inner) {
        m_inner->setRxCallback(
            [this](const uint8_t* f, size_t l, const RxFrameInfo& i, void* u) {
                this->handleRx(f, l, i, u);
            }, this);
    }
}

int TrafficSplitter::poll(Milliseconds timeoutMs) {
    return m_inner ? m_inner->poll(timeoutMs) : 0;
}

void TrafficSplitter::setEthertypeFilter(uint16_t ethertype) {
    if (m_inner) m_inner->setEthertypeFilter(ethertype);
}

Error TrafficSplitter::setPromiscuous(bool enable) {
    return m_inner ? m_inner->setPromiscuous(enable) : Error::NotInitialized;
}

Error TrafficSplitter::addMulticastAddress(const MacAddress& mac) {
    return m_inner ? m_inner->addMulticastAddress(mac) : Error::NotInitialized;
}

Error TrafficSplitter::removeMulticastAddress(const MacAddress& mac) {
    return m_inner ? m_inner->removeMulticastAddress(mac) : Error::NotInitialized;
}

Error TrafficSplitter::setAllMulticast(bool enable) {
    return m_inner ? m_inner->setAllMulticast(enable) : Error::NotInitialized;
}

LinkStatus TrafficSplitter::getLinkStatus() const {
    return m_inner ? m_inner->getLinkStatus() : LinkStatus{};
}

void TrafficSplitter::setLinkCallback(LinkCallback callback, void* userData) {
    if (m_inner) m_inner->setLinkCallback(callback, userData);
}

Error TrafficSplitter::waitForLinkUp(Milliseconds timeoutMs) {
    return m_inner ? m_inner->waitForLinkUp(timeoutMs) : Error::NotInitialized;
}

EthernetStats TrafficSplitter::getStats() const {
    return m_inner ? m_inner->getStats() : EthernetStats{};
}

void TrafficSplitter::resetStats() {
    if (m_inner) m_inner->resetStats();
}

void* TrafficSplitter::nativeHandle() {
    return m_inner ? m_inner->nativeHandle() : nullptr;
}

const char* TrafficSplitter::getInterfaceName() const {
    return m_inner ? m_inner->getInterfaceName() : "";
}

// ============================================================================
// PcapNG Logger - see PcapNGLogger.cpp
// ============================================================================

// Forward declaration - implementation in PcapNGLogger.cpp
class PcapNGLoggerImpl;

// ============================================================================
// Logging Ethernet Wrapper Implementation
// ============================================================================

LoggingEthernetWrapper::LoggingEthernetWrapper(std::unique_ptr<IEthernet> inner,
                                               std::unique_ptr<IPcapLogger> logger)
    : m_inner(std::move(inner))
    , m_logger(std::move(logger))
    , m_userCallback(nullptr)
    , m_userData(nullptr)
    , m_logTx(true)
    , m_logRx(true)
{}

LoggingEthernetWrapper::~LoggingEthernetWrapper() = default;

Error LoggingEthernetWrapper::init(const EthernetConfig& config) {
    return m_inner ? m_inner->init(config) : Error::NotInitialized;
}

void LoggingEthernetWrapper::shutdown() {
    if (m_inner) m_inner->shutdown();
}

bool LoggingEthernetWrapper::isInitialized() const {
    return m_inner && m_inner->isInitialized();
}

Error LoggingEthernetWrapper::getMacAddress(MacAddress& mac) const {
    return m_inner ? m_inner->getMacAddress(mac) : Error::NotInitialized;
}

Error LoggingEthernetWrapper::setMacAddress(const MacAddress& mac) {
    return m_inner ? m_inner->setMacAddress(mac) : Error::NotInitialized;
}

Error LoggingEthernetWrapper::transmit(const uint8_t* frame, size_t length) {
    if (!m_inner) return Error::NotInitialized;
    
    // Log before transmit
    if (m_logTx && m_logger) {
        m_logger->logFrame(frame, length, FrameDirection::Tx, getSystemClock().nowMicros());
    }
    
    return m_inner->transmit(frame, length);
}

Error LoggingEthernetWrapper::transmitVlan(const uint8_t* frame, size_t length,
                                           uint16_t vlanId, uint8_t priority) {
    if (!m_inner) return Error::NotInitialized;
    
    if (m_logTx && m_logger) {
        m_logger->logFrame(frame, length, FrameDirection::Tx, getSystemClock().nowMicros());
    }
    
    return m_inner->transmitVlan(frame, length, vlanId, priority);
}

Error LoggingEthernetWrapper::transmitGather(const BufferDesc* iov, size_t count) {
    if (!m_inner) return Error::NotInitialized;
    
    if (m_logTx && m_logger) {
        // Build complete frame for logging
        uint8_t tempFrame[1518];
        size_t totalLen = 0;
        for (size_t i = 0; i < count && totalLen < sizeof(tempFrame); i++) {
            size_t copyLen = std::min(iov[i].length, sizeof(tempFrame) - totalLen);
            std::memcpy(tempFrame + totalLen, iov[i].data, copyLen);
            totalLen += copyLen;
        }
        m_logger->logFrame(tempFrame, totalLen, FrameDirection::Tx, getSystemClock().nowMicros());
    }
    
    return m_inner->transmitGather(iov, count);
}

void LoggingEthernetWrapper::handleRx(const uint8_t* frame, size_t length,
                                      const RxFrameInfo& info, void* userData) {
    (void)userData;
    
    if (m_logRx && m_logger) {
        m_logger->logFrameWithInfo(frame, length, FrameDirection::Rx, info);
    }
    
    if (m_userCallback) {
        m_userCallback(frame, length, info, m_userData);
    }
}

void LoggingEthernetWrapper::setRxCallback(RxCallback callback, void* userData) {
    m_userCallback = callback;
    m_userData = userData;
    
    if (m_inner) {
        if (callback || m_logRx) {
            m_inner->setRxCallback(
                [this](const uint8_t* f, size_t l, const RxFrameInfo& i, void* u) {
                    this->handleRx(f, l, i, u);
                }, this);
        } else {
            m_inner->setRxCallback(nullptr, nullptr);
        }
    }
}

int LoggingEthernetWrapper::poll(Milliseconds timeoutMs) {
    return m_inner ? m_inner->poll(timeoutMs) : 0;
}

void LoggingEthernetWrapper::setEthertypeFilter(uint16_t ethertype) {
    if (m_inner) m_inner->setEthertypeFilter(ethertype);
}

Error LoggingEthernetWrapper::setPromiscuous(bool enable) {
    return m_inner ? m_inner->setPromiscuous(enable) : Error::NotInitialized;
}

Error LoggingEthernetWrapper::addMulticastAddress(const MacAddress& mac) {
    return m_inner ? m_inner->addMulticastAddress(mac) : Error::NotInitialized;
}

Error LoggingEthernetWrapper::removeMulticastAddress(const MacAddress& mac) {
    return m_inner ? m_inner->removeMulticastAddress(mac) : Error::NotInitialized;
}

Error LoggingEthernetWrapper::setAllMulticast(bool enable) {
    return m_inner ? m_inner->setAllMulticast(enable) : Error::NotInitialized;
}

LinkStatus LoggingEthernetWrapper::getLinkStatus() const {
    return m_inner ? m_inner->getLinkStatus() : LinkStatus{};
}

void LoggingEthernetWrapper::setLinkCallback(LinkCallback callback, void* userData) {
    if (m_inner) m_inner->setLinkCallback(callback, userData);
}

Error LoggingEthernetWrapper::waitForLinkUp(Milliseconds timeoutMs) {
    return m_inner ? m_inner->waitForLinkUp(timeoutMs) : Error::NotInitialized;
}

EthernetStats LoggingEthernetWrapper::getStats() const {
    return m_inner ? m_inner->getStats() : EthernetStats{};
}

void LoggingEthernetWrapper::resetStats() {
    if (m_inner) m_inner->resetStats();
}

void* LoggingEthernetWrapper::nativeHandle() {
    return m_inner ? m_inner->nativeHandle() : nullptr;
}

const char* LoggingEthernetWrapper::getInterfaceName() const {
    return m_inner ? m_inner->getInterfaceName() : "";
}

void LoggingEthernetWrapper::enableTxLogging(bool enable) {
    m_logTx = enable;
}

void LoggingEthernetWrapper::enableRxLogging(bool enable) {
    m_logRx = enable;
}

// ============================================================================
// State Machine Logger - see StateMachineLogger.cpp
// ============================================================================

// StateMachineLoggerImpl, SlaveStateTracker, and alStatusCodeToString 
// are implemented in StateMachineLogger.cpp

// ============================================================================
// Factory Functions
// ============================================================================

std::unique_ptr<IEthernet> createVlanEthernet(std::unique_ptr<IEthernet> inner,
                                               uint16_t vlanId, uint8_t priority) {
    return std::make_unique<VlanEthernetWrapper>(std::move(inner), vlanId, priority);
}

std::unique_ptr<TrafficSplitter> createTrafficSplitter(std::unique_ptr<IEthernet> inner) {
    return std::make_unique<TrafficSplitter>(std::move(inner));
}

// createPcapLogger() is in PcapNGLogger.cpp
// createStateMachineLogger() is in StateMachineLogger.cpp

std::unique_ptr<IEthernet> createLoggingEthernet(
    std::unique_ptr<IEthernet> inner,
    const PcapLoggerConfig& config) {
    auto logger = createPcapLogger();
    if (logger->init(config) != Error::OK) {
        return inner;  // Return without logging on failure
    }
    return std::make_unique<LoggingEthernetWrapper>(std::move(inner), std::move(logger));
}

// ============================================================================
// HALInstance Implementation
// ============================================================================

// ============================================================================
// HAL Singleton Lifecycle
//
// The HAL uses a controlled singleton pattern with explicit lifecycle:
// - initHAL(config):  Creates and initializes the singleton
// - getHAL():         Returns the singleton (creates uninitialized if needed)
// - shutdownHAL():    Shuts down and destroys the singleton
// - resetHAL():       Alias for shutdownHAL(), primarily for test cleanup
//
// Thread-safe: All access is protected by g_halMutex.
// ============================================================================

// Global HAL instance (singleton with explicit init/shutdown lifecycle)
static std::unique_ptr<HALInstance> g_halInstance;
static std::mutex g_halMutex;

HALInstance::HALInstance() = default;
HALInstance::~HALInstance() = default;

Error HALInstance::init(const HALConfig& config) {
    if (m_initialized) {
        return Error::AlreadyInitialized;
    }
    
    m_config = config;
    
    // Create Ethernet interface
    m_ethernet = createDefaultEthernet();
    if (!m_ethernet) {
        return Error::InternalError;
    }
    
    // Initialize Ethernet
    Error err = m_ethernet->init(config.ethernet);
    if (err != Error::OK) {
        return err;
    }
    
    // Create PcapNG logger if enabled
    if (config.enablePcapLogging) {
        auto logger = createPcapLogger();
        err = logger->init(config.pcapConfig);
        if (err == Error::OK) {
            m_pcapLogger = std::move(logger);
            // Wrap Ethernet with logging
            m_ethernet = std::make_unique<LoggingEthernetWrapper>(
                std::move(m_ethernet), createPcapLogger());
        }
        // Non-fatal if logging fails - continue without it
    }
    
    // Create state machine logger
    m_stateLogger = createStateMachineLogger();
    m_stateLogger->init({});
    
    // Wrap Ethernet with VLAN if configured
    if (config.enableVlan && config.vlanId != 0) {
        m_ethernet = std::make_unique<VlanEthernetWrapper>(
            std::move(m_ethernet), config.vlanId, config.vlanPriority);
    }
    
    // Create traffic splitter if configured
    if (config.enableTrafficSplitting) {
        auto splitter = std::make_unique<TrafficSplitter>(std::move(m_ethernet));
        m_trafficSplitter = splitter.get();
        m_ethernet = std::move(splitter);
    }
    
    m_initialized = true;
    return Error::OK;
}

void HALInstance::shutdown() {
    if (!m_initialized) return;
    
    m_trafficSplitter = nullptr;
    
    if (m_pcapLogger) {
        m_pcapLogger->flush();
        m_pcapLogger->close();
        m_pcapLogger.reset();
    }
    
    m_stateLogger.reset();
    
    if (m_ethernet) {
        m_ethernet->shutdown();
        m_ethernet.reset();
    }
    
    m_initialized = false;
}

// Global HAL access
HALInstance& getHAL() {
    std::lock_guard<std::mutex> lock(g_halMutex);
    if (!g_halInstance) {
        g_halInstance = std::make_unique<HALInstance>();
    }
    return *g_halInstance;
}

Error initHAL(const HALConfig& config) {
    std::lock_guard<std::mutex> lock(g_halMutex);
    if (!g_halInstance) {
        g_halInstance = std::make_unique<HALInstance>();
    }
    return g_halInstance->init(config);
}

void shutdownHAL() {
    std::lock_guard<std::mutex> lock(g_halMutex);
    if (g_halInstance) {
        g_halInstance->shutdown();
        g_halInstance.reset();
    }
}

void resetHAL() {
    shutdownHAL();
}

} // namespace HAL
} // namespace EtherCAT

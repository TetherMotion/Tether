/**
 * @file HAL.hpp
 * @brief Main Hardware Abstraction Layer header
 *
 * This header includes all HAL components and provides the main
 * initialization/configuration interface.
 */

#pragma once

// HAL Components
#include "hal/HALTypes.hpp"
#include "hal/IEthernet.hpp"
#include "hal/IThreading.hpp"
#include "hal/IClock.hpp"
#include "hal/ILogger.hpp"
#include "hal/IPcapLogger.hpp"
#include "hal/StateMachineLogger.hpp"

namespace EtherCAT {
namespace HAL {

// ============================================================================
// HAL Configuration
// ============================================================================

/**
 * @brief HAL initialization configuration
 */
struct HALConfig {
    // Ethernet
    EthernetConfig ethernet;
    
    // Logging
    LogLevel logLevel = LogLevel::Info;
    bool enablePcapLogging = false;
    PcapLoggerConfig pcapConfig;
    
    // Threading
    bool useRealtimeScheduling = false;
    int realtimePriority = 80;  // Linux: 1-99, higher = more priority
    
    // VLAN
    bool enableVlan = false;
    uint16_t vlanId = 0;
    uint8_t vlanPriority = 0;
    
    // Traffic splitting
    bool enableTrafficSplitting = false;
    uint16_t etherCATEthertype = kEtherTypeEtherCAT;
};

// ============================================================================
// HAL Instance
// ============================================================================

/**
 * @brief Main HAL instance providing access to all subsystems
 */
class HALInstance {
public:
    HALInstance();
    ~HALInstance();
    
    /**
     * @brief Initialize HAL with configuration
     */
    Error init(const HALConfig& config);
    
    /**
     * @brief Shutdown HAL
     */
    void shutdown();
    
    /**
     * @brief Check if initialized
     */
    bool isInitialized() const { return m_initialized; }
    
    // --- Component Access ---
    
    /**
     * @brief Get Ethernet interface
     */
    IEthernet& ethernet() { return *m_ethernet; }
    const IEthernet& ethernet() const { return *m_ethernet; }
    
    /**
     * @brief Get threading factory
     */
    IThreadingFactory& threading() { return getThreadingFactory(); }
    
    /**
     * @brief Get clock factory
     */
    IClockFactory& clock() { return getClockFactory(); }
    
    /**
     * @brief Get system clock
     */
    IClock& systemClock() { return getSystemClock(); }
    
    /**
     * @brief Get logger
     */
    ILogger& logger() { return getLogger(); }
    
    /**
     * @brief Get state machine logger
     */
    IStateMachineLogger& stateLogger() { return *m_stateLogger; }
    
    /**
     * @brief Get PcapNG logger (may be null)
     */
    IPcapLogger* pcapLogger() { return m_pcapLogger.get(); }
    
    /**
     * @brief Get traffic splitter (may be null)
     */
    TrafficSplitter* trafficSplitter() { return m_trafficSplitter; }
    
    // --- Convenience Methods ---
    
    /**
     * @brief Create a periodic timer
     */
    std::unique_ptr<IPeriodicTimer> createPeriodicTimer() {
        return clock().createPeriodicTimer();
    }
    
    /**
     * @brief Create a thread
     */
    std::unique_ptr<IThread> createThread(const ThreadConfig& config = {}) {
        return threading().createThread(config);
    }
    
    /**
     * @brief Create a mutex
     */
    std::unique_ptr<IMutex> createMutex(MutexType type = MutexType::Normal) {
        return threading().createMutex(type);
    }
    
    /**
     * @brief Get current time in microseconds
     */
    Timestamp nowMicros() {
        return systemClock().nowMicros();
    }
    
    /**
     * @brief Sleep
     */
    void sleep(Milliseconds ms) {
        threading().sleep(ms);
    }
    
private:
    bool m_initialized = false;
    HALConfig m_config;
    
    std::unique_ptr<IEthernet> m_ethernet;
    std::unique_ptr<IStateMachineLogger> m_stateLogger;
    std::unique_ptr<IPcapLogger> m_pcapLogger;
    TrafficSplitter* m_trafficSplitter = nullptr;  // Points into m_ethernet chain
};

// ============================================================================
// Global HAL Access
// ============================================================================

/**
 * @brief Get global HAL instance
 */
HALInstance& getHAL();

/**
 * @brief Initialize global HAL
 */
Error initHAL(const HALConfig& config);

/**
 * @brief Shutdown global HAL
 */
void shutdownHAL();

/**
 * @brief Reset global HAL for testing
 *
 * Shuts down and destroys the HAL singleton, allowing re-initialization.
 * This is primarily for unit tests that need clean HAL state between cases.
 */
void resetHAL();

// ============================================================================
// Platform Detection Helpers
// ============================================================================

/**
 * @brief Get platform name string
 */
inline const char* getPlatformName() {
#if defined(HAL_PLATFORM_ESP32)
    return "ESP32";
#elif defined(HAL_PLATFORM_LINUX)
    return "Linux";
#elif defined(HAL_PLATFORM_STM32)
    return "STM32";
#elif defined(HAL_PLATFORM_TEST)
    return "Test";
#else
    return "Unknown";
#endif
}

/**
 * @brief Check if running on ESP32
 */
inline bool isESP32() {
#ifdef HAL_PLATFORM_ESP32
    return true;
#else
    return false;
#endif
}

/**
 * @brief Check if running on Linux
 */
inline bool isLinux() {
#ifdef HAL_PLATFORM_LINUX
    return true;
#else
    return false;
#endif
}

/**
 * @brief Check if running on STM32
 */
inline bool isSTM32() {
#ifdef HAL_PLATFORM_STM32
    return true;
#else
    return false;
#endif
}

/**
 * @brief Check if in test mode
 */
inline bool isTestMode() {
#ifdef HAL_PLATFORM_TEST
    return true;
#else
    return false;
#endif
}

} // namespace HAL
} // namespace EtherCAT

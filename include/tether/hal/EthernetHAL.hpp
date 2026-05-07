/**
 * @file EthernetHAL.hpp
 * @brief Hardware Abstraction Layer for Raw Ethernet Access
 *
 * @details
 * This module provides a platform-agnostic interface for raw Ethernet
 * frame transmission and reception. It enables the same EtherCAT code
 * to run on:
 *
 * - ESP32 (using esp_eth driver)
 * - Linux host (using raw sockets)
 * - Unit tests (using mock/loopback)
 *
 * ## Architecture
 *
 * ```
 * ┌─────────────────────────────────────────────────────────────┐
 * │                    Application Code                         │
 * │                    (EtherCAT Master/Slave)                  │
 * └────────────────────────────┬────────────────────────────────┘
 *                              │
 * ┌────────────────────────────┼────────────────────────────────┐
 * │                    EthernetHAL Interface                    │
 * │                                                             │
 * │  ┌─────────────┐  ┌─────────────┐  ┌─────────────┐         │
 * │  │ ESP32 HAL   │  │ Linux HAL   │  │ Mock HAL    │         │
 * │  │ esp_eth.h   │  │ raw socket  │  │ loopback    │         │
 * │  └─────────────┘  └─────────────┘  └─────────────┘         │
 * └─────────────────────────────────────────────────────────────┘
 * ```
 */

#pragma once

#include <cstdint>
#include <cstddef>
#include <functional>
#include <memory>

namespace EtherCAT {
namespace HAL {

// ============================================================================
// Configuration
// ============================================================================

/**
 * @brief Maximum Ethernet frame size (without FCS)
 */
constexpr size_t kMaxFrameSize = 1514;

/**
 * @brief Minimum Ethernet frame size (without FCS)
 */
constexpr size_t kMinFrameSize = 60;

/**
 * @brief EtherCAT EtherType
 */
constexpr uint16_t kEtherTypeEtherCAT = 0x88A4;

// ============================================================================
// Callback Types
// ============================================================================

/**
 * @brief Callback for received frames
 *
 * @param frame Pointer to frame data (includes Ethernet header)
 * @param length Frame length in bytes
 * @param user_data User-provided context pointer
 */
using RxCallback = std::function<void(const uint8_t* frame, size_t length, void* user_data)>;

// ============================================================================
// HAL Interface
// ============================================================================

/**
 * @brief Abstract interface for Ethernet HAL
 */
class IEthernetHAL {
public:
    virtual ~IEthernetHAL() = default;

    /**
     * @brief Initialize the Ethernet interface
     *
     * @param interface_name Interface name (e.g., "eth0" for Linux, ignored for ESP32)
     * @return true on success
     */
    virtual bool init(const char* interface_name = nullptr) = 0;

    /**
     * @brief Shutdown the Ethernet interface
     */
    virtual void shutdown() = 0;

    /**
     * @brief Check if interface is initialized
     */
    virtual bool isInitialized() const = 0;

    /**
     * @brief Get the MAC address of this interface
     *
     * @param[out] mac Buffer for 6-byte MAC address
     * @return true on success
     */
    virtual bool getMacAddress(uint8_t mac[6]) const = 0;

    /**
     * @brief Set a custom MAC address
     *
     * @param mac 6-byte MAC address
     * @return true on success
     */
    virtual bool setMacAddress(const uint8_t mac[6]) = 0;

    /**
     * @brief Transmit a raw Ethernet frame
     *
     * @param frame Complete frame including Ethernet header
     * @param length Frame length in bytes
     * @return true on success
     */
    virtual bool transmit(const uint8_t* frame, size_t length) = 0;

    /**
     * @brief Register callback for received frames
     *
     * @param callback Function to call when a frame is received
     * @param user_data User context passed to callback
     */
    virtual void setRxCallback(RxCallback callback, void* user_data) = 0;

    /**
     * @brief Set EtherType filter
     *
     * When set, only frames with matching EtherType are passed to the callback.
     *
     * @param ethertype EtherType to filter (0 = no filter)
     */
    virtual void setEtherTypeFilter(uint16_t ethertype) = 0;

    /**
     * @brief Enable promiscuous mode
     *
     * @param enable true to enable promiscuous mode
     * @return true on success
     */
    virtual bool setPromiscuous(bool enable) = 0;

    /**
     * @brief Process pending RX packets (for polling-based implementations)
     *
     * Call this periodically if the HAL doesn't use interrupt-driven RX.
     *
     * @param timeout_ms Maximum time to wait for packets (0 = non-blocking)
     * @return Number of packets processed
     */
    virtual int poll(uint32_t timeout_ms = 0) = 0;

    /**
     * @brief Get statistics
     */
    struct Stats {
        uint64_t tx_frames;
        uint64_t tx_bytes;
        uint64_t tx_errors;
        uint64_t rx_frames;
        uint64_t rx_bytes;
        uint64_t rx_errors;
        uint64_t rx_filtered;
    };

    virtual Stats getStats() const = 0;
    virtual void resetStats() = 0;
};

// ============================================================================
// Factory Functions
// ============================================================================

/**
 * @brief Create platform-appropriate HAL instance
 *
 * Returns:
 * - ESP32HAL on ESP32
 * - LinuxRawSocketHAL on Linux
 * - MockHAL in unit tests
 */
std::unique_ptr<IEthernetHAL> createDefaultHAL();

/**
 * @brief Create ESP32 HAL (ESP-IDF only)
 */
#ifndef UNIT_TEST_HOST
std::unique_ptr<IEthernetHAL> createESP32HAL();
#endif

/**
 * @brief Create Linux raw socket HAL
 */
#ifdef __linux__
std::unique_ptr<IEthernetHAL> createLinuxRawSocketHAL();
#endif

/**
 * @brief Create mock/loopback HAL for testing
 */
std::unique_ptr<IEthernetHAL> createMockHAL();

// ============================================================================
// Utility Functions
// ============================================================================

/**
 * @brief Build an Ethernet frame header
 *
 * @param[out] buffer Output buffer (must be at least 14 bytes)
 * @param dst_mac Destination MAC address
 * @param src_mac Source MAC address
 * @param ethertype EtherType (host byte order)
 */
void buildEthernetHeader(uint8_t* buffer,
                          const uint8_t dst_mac[6],
                          const uint8_t src_mac[6],
                          uint16_t ethertype);

/**
 * @brief Extract EtherType from frame
 *
 * @param frame Frame buffer
 * @return EtherType in host byte order
 */
uint16_t getEtherType(const uint8_t* frame);

/**
 * @brief Check if frame is an EtherCAT frame
 */
inline bool isEtherCATFrame(const uint8_t* frame) {
    return getEtherType(frame) == kEtherTypeEtherCAT;
}

} // namespace HAL
} // namespace EtherCAT

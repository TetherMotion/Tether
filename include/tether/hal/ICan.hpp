/**
 * @file ICan.hpp
 * @brief Controller Area Network (CAN) Hardware Abstraction Layer interface.
 *
 * @details
 * This interface abstracts CAN 2.0A (classic CAN) frame send/receive across
 * platforms (Linux SocketCAN, ESP32 TWAI, STM32 bxCAN). It mirrors the style
 * of IEthernet.hpp: a configuration struct, a statistics struct, and a pure
 * virtual interface with open/close/send/recv.
 *
 * CAN 2.0A frames carry an 11-bit ID and up to 8 data bytes (DLC 0..8). This
 * interface does not cover CAN FD (64-byte frames) or extended 29-bit IDs;
 * those may be added later via a separate interface if needed.
 *
 * The Linux implementation (LinuxCan) uses SocketCAN and is compiled only
 * when TETHER_ENABLE_KLIPPER_CAN is ON.
 */

#pragma once

#include "hal/HALTypes.hpp"

#include <cstdint>
#include <cstddef>
#include <string>

namespace tether::hal {

/// @brief A classic CAN 2.0A frame (11-bit ID, up to 8 data bytes).
struct CanFrame {
    /// 11-bit standard CAN ID.
    uint32_t id = 0;
    /// Data length code (0..8).
    uint8_t dlc = 0;
    /// Data bytes (only the first @p dlc bytes are valid).
    uint8_t data[8] = {0};
};

/// @brief CAN interface configuration.
struct CanConfig {
    /// Interface name ("can0", "vcan0", etc.).
    std::string interfaceName;
    /// Bitrate in bits/second (used for non-socketcan backends; SocketCAN
    /// configures bitrate via iproute2, so this is informational only).
    uint32_t bitrate = 500000;
    /// Receive buffer size (number of frames).
    size_t rxBufferSize = 32;
    /// Transmit buffer size (number of frames).
    size_t txBufferSize = 16;
    /// If true, loopback own sent frames back to the receive queue.
    bool loopback = false;
    /// If true, receive own sent frames (requires loopback).
    bool receiveOwn = false;
};

/// @brief CAN interface statistics.
struct CanStats {
    uint64_t txFrames = 0;
    uint64_t txBytes = 0;
    uint64_t txErrors = 0;
    uint64_t txDropped = 0;
    uint64_t rxFrames = 0;
    uint64_t rxBytes = 0;
    uint64_t rxErrors = 0;
    uint64_t rxDropped = 0;
};

/**
 * @brief Abstract CAN interface (CAN 2.0A).
 */
class ICan {
public:
    virtual ~ICan() = default;

    /// @brief Open and configure the CAN interface.
    virtual bool open(const CanConfig& config) = 0;

    /// @brief Close the interface.
    virtual void close() = 0;

    /// @return true if the interface is open.
    virtual bool isOpen() const = 0;

    /**
     * @brief Send a CAN frame.
     * @param frame Frame to send (dlc must be <= 8).
     * @return true on success.
     */
    virtual bool send(const CanFrame& frame) = 0;

    /**
     * @brief Receive a CAN frame.
     * @param out      Output frame.
     * @param canBlock If true, block until a frame arrives.
     * @return true on success, false if no frame available / error.
     */
    virtual bool recv(CanFrame& out, bool canBlock = false) = 0;

    /// @return Current statistics.
    virtual CanStats stats() const = 0;
};

} // namespace tether::hal

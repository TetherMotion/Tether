/**
 * @file EtherCATTransport.hpp
 * @brief Low-level frame transport for EtherCAT — sending and UDP encapsulation
 *
 * @details
 * Extracted from the Master god-class to isolate the "how frames get sent
 * over the network" responsibility.  EtherCATTransport owns:
 *  - Raw frame sending via the NetworkInterface
 *  - EtherCAT-over-UDP encapsulation (ETG.1000.3) when enabled
 *  - IP identification counter for UDP-encapsulated frames
 *
 * The class is intentionally small and testable in isolation — pass it a
 * NetworkInterface with a mock `send` function and verify the bytes that
 * appear on the wire.
 */

#pragma once

#include "tether/ethercat/Types.hpp"

#if TETHER_ENABLE_UDP_ENCAPSULATION
#include "tether/ethercat/TetherConfig.hpp"
#endif

#include <atomic>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <functional>

namespace EtherCAT {

/// @brief Configuration for EtherCAT-over-UDP encapsulation (ETG.1000.3).
/// @note Only used when TETHER_ENABLE_UDP_ENCAPSULATION is defined.
struct UdpEncapsulationConfig {
    bool     enabled          = false;
    uint32_t source_ip        = 0;
    uint32_t destination_ip   = 0xFFFFFFFF;
    uint16_t source_port      = 0x88A4;
    uint16_t destination_port = 0x88A4;
};

/// @brief Low-level frame transport: sends EtherCAT frames over a NetworkInterface,
///        optionally wrapping them in IPv4/UDP when EtherCAT-over-UDP is enabled.
class EtherCATTransport {
public:
#if TETHER_ENABLE_UDP_ENCAPSULATION
    EtherCATTransport(const NetworkInterface* iface,
                      const UdpEncapsulationConfig* udp_config)
        : iface_(iface), udp_config_(udp_config) {}
#else
    explicit EtherCATTransport(const NetworkInterface* iface)
        : iface_(iface) {}
#endif

    /// @brief Send a raw frame, applying UDP encapsulation if enabled.
    /// @return true if the network interface accepted the frame.
    bool send(const uint8_t* frame, size_t len) const {
#if TETHER_ENABLE_UDP_ENCAPSULATION
        return sendWithEncapsulation(frame, len);
#else
        return (iface_ && iface_->send) ? iface_->send(frame, len) : false;
#endif
    }

    /// @brief Maximum EtherCAT payload bytes per frame, accounting for UDP overhead.
    size_t maxEtherCATPayloadPerFrame() const {
#if TETHER_ENABLE_UDP_ENCAPSULATION
        if (udp_config_ && udp_config_->enabled) {
            return kMaxEtherCATPayloadPerFrame_ - kUdpEncapOverhead_;
        }
#endif
        return kMaxEtherCATPayloadPerFrame_;
    }

#if TETHER_ENABLE_UDP_ENCAPSULATION
    /// @brief Compute the IPv4 header checksum (ones-complement sum over 20-byte header).
    static uint16_t computeIpChecksum(const uint8_t* ip_header);

    /// @brief Encapsulate a direct Ethernet/EtherCAT frame into Ethernet/IPv4/UDP/EtherCAT.
    /// @param in_frame  Original frame (Ethernet + EtherCAT, EtherType 0x88A4).
    /// @param in_len    Length of the original frame.
    /// @param out_buf   Output buffer (must be at least in_len + kUdpEncapOverhead bytes).
    /// @param out_cap   Capacity of out_buf.
    /// @param[out] out_len  Actual length of the encapsulated frame.
    /// @return true on success, false if the frame is too short or output buffer too small.
    bool encapsulateFrame(const uint8_t* in_frame, size_t in_len,
                          uint8_t* out_buf, size_t out_cap, size_t* out_len) const;

    /// @brief Send a frame, applying UDP encapsulation if enabled.
    bool sendWithEncapsulation(const uint8_t* frame, size_t len) const;
#endif

private:
    const NetworkInterface* iface_;

#if TETHER_ENABLE_UDP_ENCAPSULATION
    const UdpEncapsulationConfig* udp_config_;
    mutable std::atomic<uint16_t> ip_id_counter_{0};
#endif

    static constexpr size_t kMaxEtherCATPayloadPerFrame_ = 1498;

#if TETHER_ENABLE_UDP_ENCAPSULATION
    static constexpr size_t kUdpEncapOverhead_ = 28; // IPv4 (20) + UDP (8)
#endif
};

} // namespace EtherCAT

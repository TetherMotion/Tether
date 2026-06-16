/**
 * @file LinuxDevNullNetworkInterface.hpp
 * @brief A /dev/null-style NetworkInterface that silently discards all traffic
 *
 * `LinuxDevNullNetworkInterface` produces a `NetworkInterface` whose:
 *  - `send()` always succeeds but discards the frame.
 *  - `receive()` always returns "no data" immediately.
 *
 * This is useful for unit-testing the master in isolation (no real
 * Ethernet interface required, no root privileges, no hardware).
 *
 * @code
 *   LinuxDevNullNetworkInterface devnull;
 *   EtherCAT::Master master;
 *   uint8_t mac[6] = {0x02, 0x00, 0x00, 0x00, 0x00, 0x01};
 *   master.start(devnull.iface(), mac);
 * @endcode
 */

#pragma once

#include <atomic>
#include <cstddef>
#include <cstdint>

#include "tether/ethercat/Types.hpp"

namespace EtherCAT {

/**
 * @brief /dev/null network interface — discards all TX, never receives.
 */
class LinuxDevNullNetworkInterface {
public:
    LinuxDevNullNetworkInterface() {
        iface_.send = [this](const uint8_t* /*data*/, size_t len) -> bool {
            tx_count_.fetch_add(1, std::memory_order_relaxed);
            tx_bytes_.fetch_add(static_cast<uint64_t>(len), std::memory_order_relaxed);
            return true;
        };
        iface_.receive = [](uint8_t*, size_t, size_t* out_len) -> bool {
            if (out_len) *out_len = 0;
            return false;
        };
    }

    ~LinuxDevNullNetworkInterface() = default;

    /// Get a reference to the underlying NetworkInterface.
    NetworkInterface& iface() { return iface_; }
    const NetworkInterface& iface() const { return iface_; }

    /// Get a pointer (useful for set_network_interface).
    NetworkInterface* ifacePtr() { return &iface_; }

    /// Number of frames "sent" (discarded).
    uint64_t txCount() const { return tx_count_.load(std::memory_order_relaxed); }

    /// Total bytes "sent" (discarded).
    uint64_t txBytes() const { return tx_bytes_.load(std::memory_order_relaxed); }

    /// Reset counters.
    void resetCounters() {
        tx_count_.store(0, std::memory_order_relaxed);
        tx_bytes_.store(0, std::memory_order_relaxed);
    }

private:
    NetworkInterface iface_{};
    std::atomic<uint64_t> tx_count_{0};
    std::atomic<uint64_t> tx_bytes_{0};
};

} // namespace EtherCAT

/**
 * @file LinuxPairedNetworkInterface.hpp
 * @brief Paired loopback network interfaces for integration testing
 *
 * `LinuxPairedNetworkInterface` creates two `NetworkInterface` objects
 * (A and B) connected back-to-back in software — anything sent on A
 * is delivered as a received frame on B, and vice-versa.
 *
 * This allows testing master ↔ slave (or master ↔ master) communication
 * entirely in user-space, without real hardware or root privileges.
 *
 * ## Architecture
 *
 * ```
 *   Master          PairedIface            Slave emulator
 *   ──────          ───────────            ──────────────
 *   .send() ─► A.send() ──► B.callbacks   (process + respond)
 *                                   │
 *   .handleRx ◄─ A.callbacks ◄── B.send()
 * ```
 *
 * ## Usage
 *
 * ```cpp
 *   EtherCAT::LinuxPairedNetworkInterface pair;
 *
 *   // Give side-A to the master
 *   EtherCAT::Master master;
 *   master.start(pair.ifaceA(), mac);
 *
 *   // Side-B receives master frames & can reply
 *   pair.setRxCallbackB([&](const uint8_t* frame, size_t len) {
 *       // process frame, build response
 *       pair.ifaceB().send(response, resp_len);
 *   });
 * ```
 */

#pragma once

#include <atomic>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <deque>
#include <functional>
#include <mutex>
#include <vector>

#include "tether/ethercat/EtherCATTypes.hpp"

namespace EtherCAT {

/**
 * @brief Paired (loopback) network interfaces: A ↔ B.
 *
 * Frames sent on side A are delivered to side B's RX callback
 * (and vice-versa).  This is fully synchronous — the send call
 * invokes the peer's callback inline before returning.
 */
class LinuxPairedNetworkInterface {
public:
    /// Receive callback: (frame data, length)
    using RxCallback = std::function<void(const uint8_t* data, size_t len)>;

    LinuxPairedNetworkInterface() {
        setupInterface(iface_a_, rx_cb_b_, stats_a_);
        setupInterface(iface_b_, rx_cb_a_, stats_b_);
    }

    ~LinuxPairedNetworkInterface() = default;

    // ---- Interface accessors -----------------------------------------------

    /// Network interface for side A (typically given to the master).
    NetworkInterface& ifaceA() { return iface_a_; }
    const NetworkInterface& ifaceA() const { return iface_a_; }
    NetworkInterface* ifaceAPtr() { return &iface_a_; }

    /// Network interface for side B (typically given to a slave emulator).
    NetworkInterface& ifaceB() { return iface_b_; }
    const NetworkInterface& ifaceB() const { return iface_b_; }
    NetworkInterface* ifaceBPtr() { return &iface_b_; }

    // ---- RX callback registration ------------------------------------------

    /**
     * @brief Set callback invoked when a frame arrives on side A
     *        (i.e. when side B sends a frame).
     */
    void setRxCallbackA(RxCallback cb) {
        std::lock_guard<std::mutex> lock(mtx_);
        rx_cb_a_ = std::move(cb);
    }

    /**
     * @brief Set callback invoked when a frame arrives on side B
     *        (i.e. when side A sends a frame).
     */
    void setRxCallbackB(RxCallback cb) {
        std::lock_guard<std::mutex> lock(mtx_);
        rx_cb_b_ = std::move(cb);
    }

    // ---- Statistics --------------------------------------------------------

    struct Stats {
        std::atomic<uint64_t> tx_count{0};
        std::atomic<uint64_t> tx_bytes{0};
        std::atomic<uint64_t> rx_count{0};
        std::atomic<uint64_t> rx_bytes{0};
        std::atomic<uint64_t> dropped{0};  ///< Frames with no callback registered
    };

    const Stats& statsA() const { return stats_a_; }
    const Stats& statsB() const { return stats_b_; }

    void resetStats() {
        auto reset = [](Stats& s) {
            s.tx_count.store(0); s.tx_bytes.store(0);
            s.rx_count.store(0); s.rx_bytes.store(0);
            s.dropped.store(0);
        };
        reset(stats_a_);
        reset(stats_b_);
    }

private:
    /**
     * @brief Wire up a NetworkInterface so that `send()` invokes the
     *        peer's RX callback.
     *
     * @param iface      The interface being configured.
     * @param peer_rx_cb Reference to the PEER's rx callback (invoked on send).
     * @param tx_stats   Stats for the sending side.
     */
    void setupInterface(NetworkInterface& iface,
                        RxCallback& peer_rx_cb,
                        Stats& tx_stats) {
        iface.send = [this, &peer_rx_cb, &tx_stats](const uint8_t* data, size_t len) -> bool {
            tx_stats.tx_count.fetch_add(1, std::memory_order_relaxed);
            tx_stats.tx_bytes.fetch_add(len, std::memory_order_relaxed);

            RxCallback cb;
            {
                std::lock_guard<std::mutex> lock(mtx_);
                cb = peer_rx_cb;
            }
            if (cb) {
                cb(data, len);
            } else {
                tx_stats.dropped.fetch_add(1, std::memory_order_relaxed);
            }
            return true;
        };
        iface.receive = [](uint8_t*, size_t, size_t* out_len) -> bool {
            if (out_len) *out_len = 0;
            return false;
        };
    }

    NetworkInterface iface_a_{};
    NetworkInterface iface_b_{};

    std::mutex mtx_;   ///< Guards rx_cb_a_ and rx_cb_b_
    RxCallback rx_cb_a_;
    RxCallback rx_cb_b_;

    Stats stats_a_;
    Stats stats_b_;
};

} // namespace EtherCAT

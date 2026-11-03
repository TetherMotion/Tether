/**
 * @file EmulatedBus.hpp
 * @brief Shared test fixture: a Master wired to a NetworkEmulator through a
 *        NetworkInterface, giving tests a full master ↔ emulated-slave wire.
 *
 * Frame flow:
 *   master → iface.send → NetworkEmulator::processFrame → response queued
 *   with a small ready delay → a background drain thread pulls it through
 *   iface.receive → master.drainWire routes it to the pending waiter.
 *
 * The ready delay emulates wire latency: on a real link the response lands
 * in the socket RX buffer after the caller has registered its router
 * waiter.  Delivering synchronously inside send() routes the reply before
 * any waiter exists for send-then-wait callers (waitForResponseIdx), which
 * drops it into the unrouted catch-all and fails the exchange.
 *
 * Usage:
 *   EmulatedBus bus;
 *   auto* slave = bus.addSlave();          // configure slave(s)
 *   Master master;
 *   bus.attach(master);                    // wire the interface
 *   master.start(bus.iface, kEmulatedMac);
 *   master.initSlaves(1);
 *
 * Call bus.shutdown() (or let the bus destruct) before the Master is
 * destroyed — the drain thread calls back into it.
 */

#pragma once

#include <atomic>
#include <chrono>
#include <cstdint>
#include <cstring>
#include <mutex>
#include <queue>
#include <thread>
#include <vector>

#include "tether/ethercat/Master.hpp"
#include "tether/ethercat/NetworkEmulator.hpp"
#include "tether/ethercat/SlaveEmulator.hpp"

class EmulatedBus {
public:
    EtherCAT::Emulator::NetworkEmulator net;
    EtherCAT::NetworkInterface iface;

    EmulatedBus() = default;

    ~EmulatedBus() { shutdown(); }

    EmulatedBus(const EmulatedBus&) = delete;
    EmulatedBus& operator=(const EmulatedBus&) = delete;

    /// Emulated wire latency: how long a slave response sits before the
    /// drain thread may deliver it.  Long enough that any send-then-wait
    /// caller has registered its router waiter first.
    static constexpr auto kWireDelay = std::chrono::microseconds(200);

    /// Wire send/receive so frames round-trip through the emulator.
    void attach(EtherCAT::Master& master) {
        master_ = &master;
        iface.send = [this](const uint8_t* data, size_t len) -> bool {
            auto response = net.processFrame(data, len);
            if (!response.empty()) {
                std::lock_guard<std::mutex> lk(mutex_);
                rx_.push({std::move(response),
                          std::chrono::steady_clock::now() + kWireDelay});
            }
            return true;
        };
        iface.receive = [this](uint8_t* buf, size_t max_len,
                               size_t* out_len) -> bool {
            std::lock_guard<std::mutex> lk(mutex_);
            if (rx_.empty() ||
                rx_.front().ready_at > std::chrono::steady_clock::now()) {
                return false;
            }
            auto frame = std::move(rx_.front().bytes);
            rx_.pop();
            if (frame.size() > max_len) frame.resize(max_len);
            std::memcpy(buf, frame.data(), frame.size());
            *out_len = frame.size();
            return true;
        };
        running_ = true;
        drainer_ = std::thread([this] { drainLoop(); });
    }

    /// Stop the drain thread.  Call before the attached Master goes away.
    void shutdown() {
        running_ = false;
        if (drainer_.joinable()) drainer_.join();
    }

    EtherCAT::Emulator::SlaveEmulator* addSlave() {
        auto s = std::make_unique<EtherCAT::Emulator::SlaveEmulator>();
        auto* raw = s.get();
        net.addSlave(std::move(s));
        return raw;
    }

    /// Pending response frames (test diagnostics only).
    size_t queuedResponses() {
        std::lock_guard<std::mutex> lk(mutex_);
        return rx_.size();
    }

private:
    void drainLoop() {
        while (running_.load(std::memory_order_relaxed)) {
            // drainWire touches Master::iface_, which Master::start() is
            // still writing while attach() has already spawned this
            // thread — only drain once the master reports running.
            if (master_ && master_->isRunning()) master_->drainWire(64);
            std::this_thread::sleep_for(std::chrono::microseconds(50));
        }
    }

    struct QueuedFrame {
        std::vector<uint8_t> bytes;
        std::chrono::steady_clock::time_point ready_at;
    };

    EtherCAT::Master* master_ = nullptr;
    std::atomic<bool> running_{false};
    std::thread drainer_;
    std::mutex mutex_;
    std::queue<QueuedFrame> rx_;
};

/// Conventional locally-administered test MAC.
inline constexpr uint8_t kEmulatedMac[6] = {0x02, 0x00, 0x00, 0x00, 0x00, 0x01};

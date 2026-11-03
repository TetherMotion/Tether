/**
 * @file EmulatedBus.hpp
 * @brief Shared test fixture: a Master wired to a NetworkEmulator through a
 *        NetworkInterface, giving tests a full master ↔ emulated-slave wire.
 *
 * Frame flow:
 *   master → iface.send → NetworkEmulator::processFrame → response queued →
 *   master.drainWire() pulls it back through iface.receive and routes it to
 *   the pending waiter slot — all synchronously inside the send call, so
 *   round-trips are deterministic with no poll thread or sleeps.
 *
 * Usage:
 *   EmulatedBus bus;
 *   auto* slave = bus.addSlave();          // configure slave(s)
 *   Master master;
 *   bus.attach(master);                    // wire the interface
 *   master.start(bus.iface, kEmulatedMac);
 *   master.initSlaves(1);
 */

#pragma once

#include <cstdint>
#include <cstring>
#include <mutex>
#include <queue>
#include <vector>

#include "tether/ethercat/Master.hpp"
#include "tether/ethercat/NetworkEmulator.hpp"
#include "tether/ethercat/SlaveEmulator.hpp"

class EmulatedBus {
public:
    EtherCAT::Emulator::NetworkEmulator net;
    EtherCAT::NetworkInterface iface;

    EmulatedBus() = default;

    /// Wire send/receive so frames round-trip through the emulator and the
    /// drained response reaches the master's packet router synchronously.
    void attach(EtherCAT::Master& master) {
        iface.send = [this, &master](const uint8_t* data, size_t len) -> bool {
            auto response = net.processFrame(data, len);
            if (!response.empty()) {
                {
                    std::lock_guard<std::mutex> lk(mutex_);
                    rx_.push(std::move(response));
                }
                master.drainWire(8);
            }
            return true;
        };
        iface.receive = [this](uint8_t* buf, size_t max_len,
                               size_t* out_len) -> bool {
            std::lock_guard<std::mutex> lk(mutex_);
            if (rx_.empty()) return false;
            auto frame = std::move(rx_.front());
            rx_.pop();
            if (frame.size() > max_len) frame.resize(max_len);
            std::memcpy(buf, frame.data(), frame.size());
            *out_len = frame.size();
            return true;
        };
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
    std::mutex mutex_;
    std::queue<std::vector<uint8_t>> rx_;
};

/// Conventional locally-administered test MAC.
inline constexpr uint8_t kEmulatedMac[6] = {0x02, 0x00, 0x00, 0x00, 0x00, 0x01};

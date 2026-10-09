/**
 * @file PDOKeepAlive.cpp
 * @brief PDOKeepAlive — background exchange that keeps already-configured
 *        slaves fed while another slave runs its slow INIT→OP configuration.
 *
 * Split out of PDOManager.cpp; the class is declared in PDOManager.hpp.
 */

#include "tether/ethercat/PDOManager.hpp"

#include <algorithm>
#include <chrono>
#include <thread>

namespace EtherCAT {

PDOKeepAlive::PDOKeepAlive(PDOManager& manager,
                           std::chrono::milliseconds period)
    : manager_(manager)
    , period_(period)
    , worker_([this] { run(); })
{}

PDOKeepAlive::~PDOKeepAlive() {
    stop();
}

void PDOKeepAlive::stop() {
    const bool was = stop_.exchange(true, std::memory_order_acq_rel);
    if (!was && worker_.joinable()) {
        worker_.join();
    }
}

void PDOKeepAlive::run() {
    while (!stop_.load(std::memory_order_acquire)) {
        // One datagram per configured slave — a slave still being
        // initialized has no logical window yet and is skipped until its
        // configuration assigns one.
        manager_.exchangeConfiguredSlaveWindows();
        // Sleep in small slices so stop() reacts promptly.
        auto remaining = period_;
        while (remaining.count() > 0 &&
               !stop_.load(std::memory_order_acquire)) {
            const auto slice = std::min(remaining,
                                        std::chrono::milliseconds{1});
            std::this_thread::sleep_for(slice);
            remaining -= slice;
        }
    }
}

std::unique_ptr<PDOKeepAlive> PDOManager::startKeepAlive(
    std::chrono::milliseconds period) {
    return std::make_unique<PDOKeepAlive>(*this, period);
}

} // namespace EtherCAT

/**
 * @file PDOKeepAlive.hpp
 * @brief PDOKeepAlive — RAII background PDO keep-alive.
 *
 * Split out of PDOManager.hpp.
 */

#pragma once

#include <atomic>
#include <chrono>
#include <thread>

namespace EtherCAT {

class PDOManager;

class PDOKeepAlive {
public:
    PDOKeepAlive(PDOManager& manager, std::chrono::milliseconds period);
    ~PDOKeepAlive();

    PDOKeepAlive(const PDOKeepAlive&)            = delete;
    PDOKeepAlive& operator=(const PDOKeepAlive&) = delete;
    PDOKeepAlive(PDOKeepAlive&&)                 = delete;
    PDOKeepAlive& operator=(PDOKeepAlive&&)      = delete;

    /// Stop the worker thread (idempotent; also runs on destruction).
    void stop();
    bool running() const { return !stop_.load(std::memory_order_acquire); }

private:
    void run();

    PDOManager&                manager_;
    std::chrono::milliseconds  period_;
    std::atomic<bool>          stop_{false};
    std::thread                worker_;
};
} // namespace EtherCAT

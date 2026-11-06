/**
 * @file Poller.hpp
 * @brief Periodic read scheduler over a ModbusClient (plan §11).
 */
#pragma once

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <mutex>
#include <optional>
#include <thread>
#include <vector>

#include "tether/modbus/client/ModbusClient.hpp"

namespace tether::modbus {

struct PollItem {
    FunctionCode fc;               ///< Read* function code
    uint16_t     addr = 0;
    uint16_t     qty  = 0;
    int          intervalMs = 100;
    int          priority = 0;     ///< larger = scheduled first on ties
    /// Called with decoded values (bits→uint16 0/1 lanes for bit reads).
    std::function<void(const PollItem&, const std::vector<uint16_t>&,
                       const ModbusError*)> onResult;
    /// Change-only notification: only fire when value differs.
    bool changedOnly = false;
    /// Deadband (with changedOnly): fire only when some lane moved by
    /// more than `epsilon` since last notification.
    int      epsilon = 0;
    /// Debounce: require the change to persist for `debounceN` polls
    /// before onResult fires (0 = fire immediately).
    int      debounceN = 0;
    bool enabled = true;
    std::string  name;

    // internal state
    std::chrono::steady_clock::time_point nextRun{};
    std::vector<uint16_t> last;
    bool haveLast = false;
    int  changeStreak = 0;
};

class Poller {
public:
    explicit Poller(ModbusClient& client) : client_(client) {}
    ~Poller() { stop(); }

    PollItem& add(PollItem it);
    void remove(const std::string& name);

    Expected<void> start();
    void           stop();
    bool           running() const { return running_; }

    /// Force immediate re-poll of an item (or all, name empty).
    void kick(const std::string& name = {});

    /// Run one scheduling pass synchronously (tests).
    void runOnce();

    struct Stats {
        std::atomic<uint64_t> polls{0}, errors{0}, changes{0};
    } stats;

private:
    void loop();

    ModbusClient&           client_;
    std::vector<PollItem>   items_;
    mutable std::mutex      m_;
    std::condition_variable cv_;
    std::thread             th_;
    std::atomic<bool>       running_{false};
    bool                    stop_ = false;
};

} // namespace tether::modbus

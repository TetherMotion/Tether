/**
 * @file MotionLoops.cpp
 * @brief Legacy motion-control loop strategy implementations.
 */

#include "raw/MotionLoops.hpp"

#include "tether/ethercat/DC.hpp"
#include "tether/ethercat/PDOManager.hpp"
#include "tether/ethercat/RealtimeLoop.hpp"
#include "tether/platform/Platform.hpp"

#include <chrono>
#include <thread>
#include <utility>

namespace EtherCAT {

class RealtimeMotionControlLoop final : public IMotionControlLoop {
public:
    RealtimeMotionControlLoop(Master::MotionControlCallback callback,
                              Master::RealtimeMotionLoopConfig config,
                              EtherCAT::DCManager* dc_manager)
        : callback_(std::move(callback))
        , dt_seconds_(static_cast<double>(config.cycle_period_us) / 1000000.0)
        , loop_(
            [this]() { return callback_ ? callback_(dt_seconds_) : true; },
            [this, config, dc_manager]() {
                if (!config.enable_dc_synchronization || dc_manager == nullptr) {
                    return true;
                }
                return dc_manager->get()->sendSyncFrame();
            },
            []() {
                return static_cast<uint64_t>(Tether::Platform::Clock::instance().getMicroseconds()) * 1000ULL;
            },
            RealtimeLoop::Config::defaults(config.cycle_period_us, config.sync_interval_cycles))
    {
    }

    bool start() override {
        loop_.setPDOEnabled(true);
        return loop_.start();
    }

    void stop() override {
        loop_.stop();
    }

    bool isRunning() const override {
        return loop_.isRunning();
    }

    void setShutdownDebug(bool enabled) override {
        loop_.setShutdownDebug(enabled);
    }

private:
    Master::MotionControlCallback callback_;
    double dt_seconds_;
    RealtimeLoop loop_;
};

class PollingMotionControlLoop final : public IMotionControlLoop {
public:
    PollingMotionControlLoop(Master::MotionControlCallback callback,
                             Master::PollingMotionLoopConfig config,
                             EtherCAT::DCManager* dc_manager)
        : callback_(std::move(callback))
        , config_(config)
        , dc_manager_(dc_manager)
    {
    }

    ~PollingMotionControlLoop() override {
        stop();
    }

    bool start() override {
        if (running_.exchange(true, std::memory_order_acq_rel)) {
            return false;
        }

        thread_ = std::thread([this]() {
            if (config_.request_realtime_priority) {
                (void)Tether::Platform::setCurrentThreadRealtime(-1);
            }

            const auto period = std::chrono::microseconds(config_.cycle_period_us);
            const double dt_seconds = static_cast<double>(config_.cycle_period_us) / 1000000.0;
            auto next_tick = std::chrono::steady_clock::now();
            uint32_t cycle = 0;

            while (running_.load(std::memory_order_acquire)) {
                next_tick += period;
                if (callback_ && !callback_(dt_seconds)) {
                    running_.store(false, std::memory_order_release);
                    break;
                }

                if (config_.enable_dc_synchronization &&
                    dc_manager_ != nullptr &&
                    config_.sync_interval_cycles != 0 &&
                    (++cycle % config_.sync_interval_cycles) == 0) {
                    if (!dc_manager_->get()->sendSyncFrame()) {
                        running_.store(false, std::memory_order_release);
                        break;
                    }
                }

                std::this_thread::sleep_until(next_tick);
            }
        });

        return true;
    }

    void stop() override {
        running_.store(false, std::memory_order_release);
        if (thread_.joinable()) {
            thread_.join();
        }
    }

    bool isRunning() const override {
        return running_.load(std::memory_order_acquire);
    }

private:
    Master::MotionControlCallback callback_;
    Master::PollingMotionLoopConfig config_;
    EtherCAT::DCManager* dc_manager_{nullptr};
    std::atomic<bool> running_{false};
    std::thread thread_;
};
class QueueMotionControlLoop final : public IMotionControlLoop {
public:
    QueueMotionControlLoop(PDOManager* pdo_manager,
                           Master::RealtimeMotionLoopConfig config,
                           EtherCAT::DCManager* dc_manager)
        : pdo_manager_(pdo_manager)
        , loop_(
            [this]() { return pdo_manager_ ? pdo_manager_->queueCycle() : false; },
            [this, config, dc_manager]() {
                if (!config.enable_dc_synchronization || dc_manager == nullptr) {
                    return true;
                }
                return dc_manager->get()->sendSyncFrame();
            },
            []() {
                return static_cast<uint64_t>(Tether::Platform::Clock::instance().getMicroseconds()) * 1000ULL;
            },
            RealtimeLoop::Config::defaults(config.cycle_period_us, config.sync_interval_cycles))
    {
    }

    bool start() override {
        loop_.setPDOEnabled(true);
        return loop_.start();
    }

    void stop() override {
        loop_.stop();
    }

    bool isRunning() const override {
        return loop_.isRunning();
    }

    void setShutdownDebug(bool enabled) override {
        loop_.setShutdownDebug(enabled);
    }

private:
    PDOManager* pdo_manager_;
    RealtimeLoop loop_;
};

std::unique_ptr<IMotionControlLoop> makeRealtimeMotionControlLoop(
    Master::MotionControlCallback callback,
    Master::RealtimeMotionLoopConfig config,
    DCManager* dc_manager) {
    return std::make_unique<RealtimeMotionControlLoop>(
        std::move(callback), config, dc_manager);
}

std::unique_ptr<IMotionControlLoop> makePollingMotionControlLoop(
    Master::MotionControlCallback callback,
    Master::PollingMotionLoopConfig config,
    DCManager* dc_manager) {
    return std::make_unique<PollingMotionControlLoop>(
        std::move(callback), config, dc_manager);
}

std::unique_ptr<IMotionControlLoop> makeQueueMotionControlLoop(
    PDOManager* pdo_manager,
    Master::RealtimeMotionLoopConfig config,
    DCManager* dc_manager) {
    return std::make_unique<QueueMotionControlLoop>(
        pdo_manager, config, dc_manager);
}

} // namespace EtherCAT

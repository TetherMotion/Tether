/**
 * @file Master_motionloop.cpp
 * @brief Master — motion-control loop and queue-mode RT loop lifecycle.
 *
 * TU split out of Master_loops.cpp.
 */

#include "tether/ethercat/Master.hpp"
#include "tether/ethercat/Slave.hpp"
#include "tether/ethercat/DC.hpp"
#include "tether/ethercat/PDOManager.hpp"
#include "tether/ethercat/LogicalAddressManager.hpp"
#include "tether/ethercat/RealtimeLoop.hpp"
#include "tether/ethercat/CyclicChannel.hpp"
#include "tether/ethercat/DebugFlags.hpp"
#include "tether/ethercat/EtherCATConfig.hpp"
#include "raw/internal.hpp"
#include "raw/MotionLoops.hpp"
#include "raw/CyclicDatapath.hpp"
#include "tether/platform/Platform.hpp"
#include "tether/platform/RtMemory.hpp"
#include "tether/platform/CpuIsolation.hpp"

#include <thread>
#include <chrono>
#include <algorithm>
#include <cstring>
#include <format>
#include <inttypes.h>
#ifdef __linux__
#include <sys/eventfd.h>
#include <sys/socket.h>
#include <netpacket/packet.h>
#include <unistd.h>
#endif

namespace EtherCAT {

static const char* TAG = "ethercat";

void Master::setMotionControlCallback(MotionControlCallback callback)
{
    motion_control_callback_ = std::move(callback);
}

bool Master::startRealtimeMotionControlLoop()
{
    return startRealtimeMotionControlLoop(RealtimeMotionLoopConfig{});
}

bool Master::startRealtimeMotionControlLoop(const RealtimeMotionLoopConfig& config)
{
    if (!motion_control_callback_) {
        TETHER_LOGE(TAG, "No motion control callback configured");
        return false;
    }

    stopMotionControlLoop();
    clearCancel();
    auto wrapped_callback = [this](double dt) -> bool {
        if (cancel_requested_.load(std::memory_order_acquire)) {
            return false;
        }
        return motion_control_callback_ ? motion_control_callback_(dt) : true;
    };
    motion_control_loop_ = makeRealtimeMotionControlLoop(std::move(wrapped_callback), config, dc_.get());
    motion_control_loop_->setShutdownDebug(debug_flags_.shutdown);
    return motion_control_loop_->start();
}

bool Master::startPollingMotionControlLoop()
{
    return startPollingMotionControlLoop(PollingMotionLoopConfig{});
}

bool Master::startPollingMotionControlLoop(const PollingMotionLoopConfig& config)
{
    if (!motion_control_callback_) {
        TETHER_LOGE(TAG, "No motion control callback configured");
        return false;
    }

    stopMotionControlLoop();
    clearCancel();
    auto wrapped_callback = [this](double dt) -> bool {
        if (cancel_requested_.load(std::memory_order_acquire)) {
            return false;
        }
        return motion_control_callback_ ? motion_control_callback_(dt) : true;
    };
    motion_control_loop_ = makePollingMotionControlLoop(std::move(wrapped_callback), config, dc_.get());
    motion_control_loop_->setShutdownDebug(debug_flags_.shutdown);
    return motion_control_loop_->start();
}

void Master::stopMotionControlLoop()
{
    if (motion_control_loop_) {
        motion_control_loop_->stop();
        motion_control_loop_.reset();
    }
}

bool Master::isMotionControlLoopRunning() const
{
    return motion_control_loop_ && motion_control_loop_->isRunning();
}

// Queue-mode RT loop
// ============================================================================

bool Master::startQueueModeLoop()
{
    return startQueueModeLoop(RealtimeMotionLoopConfig{});
}

bool Master::startQueueModeLoop(const RealtimeMotionLoopConfig& config)
{
    if (pdo_->getMode() != PDOMode::Queue) {
        TETHER_LOGE(TAG, "PDOManager is not in Queue mode; call configureQueueMode() first");
        return false;
    }

    stopMotionControlLoop();  // Ensure no other loop is running
    clearCancel();
    motion_control_loop_ = makeQueueMotionControlLoop(pdo_.get(), config, dc_.get());
    motion_control_loop_->setShutdownDebug(debug_flags_.shutdown);
    return motion_control_loop_->start();
}

void Master::stopQueueModeLoop()
{
    stopMotionControlLoop();
}

bool Master::isQueueModeLoopRunning() const
{
    return motion_control_loop_ && motion_control_loop_->isRunning();
}

} // namespace EtherCAT

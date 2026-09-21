/**
 * @file MotionLoops.hpp
 * @brief Legacy motion-control loop wrappers used by Master.
 *
 * Internal header — not part of the public API.  Defines the
 * IMotionControlLoop interface (forward-declared in Master.hpp) plus
 * factories for the three legacy loop strategies: realtime (RealtimeLoop
 * with PDO exchange), polling (plain periodic thread), and queue mode
 * (RealtimeLoop driving PDOManager::queueCycle()).
 */

#pragma once

#include "tether/ethercat/Master.hpp"

#include <memory>

namespace EtherCAT {

class DCManager;
class PDOManager;

/// Common interface for the legacy motion-control loop strategies.
class IMotionControlLoop {
public:
    virtual ~IMotionControlLoop() = default;
    virtual bool start() = 0;
    virtual void stop() = 0;
    virtual bool isRunning() const = 0;
    virtual void setShutdownDebug(bool) {}
};

/// RealtimeLoop-driven loop: callback + optional DC sync per cycle.
std::unique_ptr<IMotionControlLoop> makeRealtimeMotionControlLoop(
    Master::MotionControlCallback callback,
    Master::RealtimeMotionLoopConfig config,
    DCManager* dc_manager);

/// Plain periodic std::thread loop (non-RT fallback).
std::unique_ptr<IMotionControlLoop> makePollingMotionControlLoop(
    Master::MotionControlCallback callback,
    Master::PollingMotionLoopConfig config,
    DCManager* dc_manager);

/// Queue-mode RT loop: calls PDOManager::queueCycle() each cycle.
std::unique_ptr<IMotionControlLoop> makeQueueMotionControlLoop(
    PDOManager* pdo_manager,
    Master::RealtimeMotionLoopConfig config,
    DCManager* dc_manager);

} // namespace EtherCAT

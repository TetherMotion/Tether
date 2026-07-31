/**
 * @file TorqueHomingController.cpp
 * @brief Torque-based sensorless homing controller implementation
 */

#include "profiles/cia402/TorqueHomingController.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <sstream>

namespace CiA402 {

namespace {
constexpr uint32_t kStallWindowCapacity = 256; ///< Max samples in stall window
}

TorqueHomingController::TorqueHomingController() {
    m_positionWindow.reserve(kStallWindowCapacity);
    m_timeWindow.reserve(kStallWindowCapacity);
}

// ============================================================================
// Public API
// ============================================================================

bool TorqueHomingController::start(int8_t direction) {
    if (m_state != TorqueHomingState::Idle &&
        m_state != TorqueHomingState::Attained &&
        m_state != TorqueHomingState::Error) {
        return false; // Already running
    }

    if (!validateConfig()) {
        return false;
    }

    // Resolve direction
    if (direction == 0) {
        if (m_config.coarsePass.targetTorque == 0) {
            setError(TorqueHomingError::InvalidConfig,
                     "coarsePass.targetTorque is zero and no direction given");
            return false;
        }
        direction = (m_config.coarsePass.targetTorque > 0) ? 1 : -1;
    }
    m_direction = direction;

    // Force signs of every pass to match the chosen direction.
    if (static_cast<int16_t>(m_config.coarsePass.targetTorque * direction) < 0) {
        m_config.coarsePass.targetTorque = -m_config.coarsePass.targetTorque;
    }
    for (auto& p : m_config.finePasses) {
        if (static_cast<int16_t>(p.targetTorque * direction) < 0) {
            p.targetTorque = -p.targetTorque;
        }
        // backOffDistance should be opposite sign of approach; force it.
        if (static_cast<int32_t>(p.backOffDistance * direction) > 0) {
            p.backOffDistance = -p.backOffDistance;
        }
    }

    m_lastError = TorqueHomingError::None;
    m_lastErrorMsg.clear();
    m_homePosition = 0;
    m_currentPass = -1;
    m_modeSaved = false;
    m_previousMode = OperatingMode::NoMode;

    m_startTimeMs = getTimeMs();
    setState(TorqueHomingState::Configuring);
    return true;
}

void TorqueHomingController::abort() {
    if (m_state == TorqueHomingState::Idle ||
        m_state == TorqueHomingState::Attained ||
        m_state == TorqueHomingState::Error) {
        return;
    }
    setError(TorqueHomingError::Aborted, "Homing aborted by user");
}

void TorqueHomingController::reset() {
    if (m_state != TorqueHomingState::Idle &&
        m_state != TorqueHomingState::Attained &&
        m_state != TorqueHomingState::Error) {
        // Active sequence — refuse; caller should abort() first.
        return;
    }
    m_state = TorqueHomingState::Idle;
    m_lastError = TorqueHomingError::None;
    m_lastErrorMsg.clear();
    m_homePosition = 0;
    m_currentPass = -1;
    m_modeSaved = false;
    m_previousMode = OperatingMode::NoMode;
    m_positionWindow.clear();
    m_timeWindow.clear();
    m_windowIndex = 0;
    m_windowFilled = false;
}

bool TorqueHomingController::update() {
    switch (m_state) {
        case TorqueHomingState::Idle:
            return false;
        case TorqueHomingState::Configuring:    handleConfiguring(); break;
        case TorqueHomingState::CoarseApproach: handleApproach(m_config.coarsePass); break;
        case TorqueHomingState::FineApproach: {
            const auto& pc = m_config.finePasses.at(
                static_cast<size_t>(m_currentPass));
            handleApproach(pc);
            break;
        }
        case TorqueHomingState::BackOff:        handleBackOff(); break;
        case TorqueHomingState::Settling:       handleSettling(); break;
        case TorqueHomingState::Attained:       handleAttained(); return false;
        case TorqueHomingState::Error:          handleError(); return false;
    }

    // Global timeout
    if (m_state != TorqueHomingState::Idle &&
        m_state != TorqueHomingState::Attained &&
        m_state != TorqueHomingState::Error) {
        if (m_config.totalTimeoutMs > 0 &&
            (getTimeMs() - m_startTimeMs) > m_config.totalTimeoutMs) {
            setError(TorqueHomingError::Timeout,
                     "Total homing timeout exceeded");
        }
    }

    return m_state != TorqueHomingState::Idle &&
           m_state != TorqueHomingState::Attained &&
           m_state != TorqueHomingState::Error;
}

// ============================================================================
// State handlers
// ============================================================================

void TorqueHomingController::handleConfiguring() {
    if (callHasDriveFault()) {
        setError(TorqueHomingError::DriveFault, "Drive fault at start");
        return;
    }

    // Save the current operating mode so we can restore it later.
    if (m_config.restorePreviousMode && !m_modeSaved) {
        m_previousMode = callGetOperatingMode();
        m_modeSaved = true;
    }

    if (!callSetOperatingMode(OperatingMode::CyclicSyncTorque)) {
        setError(TorqueHomingError::ModeChangeFailed,
                 "Failed to switch to CST mode");
        return;
    }

    // Begin coarse approach.
    m_currentPass = -1;
    int32_t pos = callGetActualPosition();
    resetStallWindow(pos);
    m_positionAnchor = pos;
    m_activeTorque = 0;
    setState(TorqueHomingState::CoarseApproach);
}

void TorqueHomingController::handleApproach(const TorqueHomingPassConfig& pc) {
    if (callHasDriveFault()) {
        setError(TorqueHomingError::DriveFault, "Drive fault during approach");
        return;
    }

    int32_t pos = callGetActualPosition();
    int32_t vel = callGetActualVelocity();

    // Software position-deviation limit
    if (m_config.usePositionDeviationLimit &&
        m_config.maxPositionDeviation > 0) {
        if (!checkPositionDeviation(pos)) {
            callSetTargetTorque(0);
            std::ostringstream oss;
            oss << "Position deviation exceeded: |" << (pos - m_positionAnchor)
                << "| > " << m_config.maxPositionDeviation;
            setError(TorqueHomingError::PositionDeviationExceeded, oss.str());
            return;
        }
    }

    // Per-pass timeout
    uint64_t stateElapsed = getTimeMs() - m_stateStartTimeMs;
    if (pc.passTimeoutMs > 0 && stateElapsed > pc.passTimeoutMs) {
        callSetTargetTorque(0);
        std::ostringstream oss;
        oss << "Pass timeout (" << pc.passTimeoutMs << " ms) without stall";
        setError(TorqueHomingError::PassTimeout, oss.str());
        return;
    }

    // Apply velocity-limited torque
    int16_t cmd = applyVelocityLimit(pc.targetTorque, vel, pc.velocityLimit);
    m_activeTorque = cmd;
    callSetTargetTorque(cmd);

    // Sample stall window
    m_positionWindow[m_windowIndex] = pos;
    m_timeWindow[m_windowIndex] = getTimeMs();
    m_windowIndex = (m_windowIndex + 1) % m_positionWindow.size();
    if (m_windowIndex == 0) m_windowFilled = true;

    // Detect stall
    if (detectStall(pc)) {
        callSetTargetTorque(0);
        m_activeTorque = 0;
        if (!m_config.finePasses.empty()) {
            // startNextPass() handles both advancing to the next fine pass
            // (via BackOff) and transitioning to Settling when no more
            // fine passes remain.
            startNextPass();
        } else {
            // No fine passes configured — settle then attain.
            setState(TorqueHomingState::Settling);
        }
    }
}

void TorqueHomingController::handleBackOff() {
    if (callHasDriveFault()) {
        setError(TorqueHomingError::DriveFault, "Drive fault during back-off");
        return;
    }

    int32_t pos = callGetActualPosition();
    int32_t vel = callGetActualVelocity();

    // Position-deviation limit still applies (anchor was reset on entry).
    if (m_config.usePositionDeviationLimit &&
        m_config.maxPositionDeviation > 0) {
        if (!checkPositionDeviation(pos)) {
            callSetTargetTorque(0);
            setError(TorqueHomingError::PositionDeviationExceeded,
                     "Position deviation exceeded during back-off");
            return;
        }
    }

    // Per-pass timeout for the back-off phase uses the upcoming fine pass.
    const auto& pc = m_config.finePasses.at(
        static_cast<size_t>(m_currentPass));
    uint64_t stateElapsed = getTimeMs() - m_stateStartTimeMs;
    if (pc.passTimeoutMs > 0 && stateElapsed > pc.passTimeoutMs) {
        callSetTargetTorque(0);
        setError(TorqueHomingError::PassTimeout,
                 "Pass timeout during back-off");
        return;
    }

    // Apply back-off torque, velocity-limited.
    int16_t cmd = applyVelocityLimit(m_backOffTorque, vel, pc.velocityLimit);
    callSetTargetTorque(cmd);

    // Terminate when we have travelled the requested distance.
    int32_t travelled = std::abs(pos - m_backOffStartPos);
    if (travelled >= m_backOffTargetDelta) {
        callSetTargetTorque(0);
        // Begin the fine approach.
        int32_t p = callGetActualPosition();
        resetStallWindow(p);
        m_positionAnchor = p;
        m_activeTorque = 0;
        setState(TorqueHomingState::FineApproach);
    }
}

void TorqueHomingController::handleSettling() {
    callSetTargetTorque(0);
    m_activeTorque = 0;
    // Use the last pass's settleMs (or coarsePass if no fine passes).
    uint32_t settleMs = m_config.finePasses.empty()
        ? m_config.coarsePass.settleMs
        : m_config.finePasses.back().settleMs;
    if (settleMs == 0 || (getTimeMs() - m_stateStartTimeMs) >= settleMs) {
        completeHoming();
    }
}

void TorqueHomingController::completeHoming() {
    int32_t pos = callGetActualPosition();
    m_homePosition = pos + m_config.homeOffset;
    if (m_config.setHomePosition) {
        callSetHomePosition(m_homePosition);
    }
    if (m_callbacks.onComplete) {
        m_callbacks.onComplete(m_homePosition);
    }
    if (m_config.restorePreviousMode && m_modeSaved) {
        callSetOperatingMode(m_previousMode);
        m_modeSaved = false;
    }
    setState(TorqueHomingState::Attained);
}

void TorqueHomingController::handleAttained() {
    // Completion work is done in completeHoming() when transitioning to
    // Attained. Nothing to do here.
}

void TorqueHomingController::handleError() {
    // Cleanup is done in setError() when transitioning to Error. Nothing
    // to do here except ensure torque stays zero.
    callSetTargetTorque(0);
}

// ============================================================================
// Helpers
// ============================================================================

int16_t TorqueHomingController::applyVelocityLimit(int16_t rawTorque,
                                                    int32_t actualVel,
                                                    int32_t velLimit) const {
    if (velLimit <= 0) return rawTorque;
    int32_t v = std::abs(actualVel);
    if (v >= velLimit) return 0;

    if (m_config.velocityStrategy ==
        VelocityLimitStrategy::HardZeroAboveLimit) {
        return rawTorque;
    }

    // Proportional reduction
    double softStart = m_config.softStartFraction;
    if (softStart < 0.0) softStart = 0.0;
    if (softStart > 1.0) softStart = 1.0;
    int32_t softStartVel = static_cast<int32_t>(softStart * velLimit);
    if (v <= softStartVel) return rawTorque;

    double span = static_cast<double>(velLimit - softStartVel);
    if (span <= 0.0) return 0;
    double scale = static_cast<double>(velLimit - v) / span;
    if (scale < 0.0) scale = 0.0;
    if (scale > 1.0) scale = 1.0;
    return static_cast<int16_t>(rawTorque * scale);
}

bool TorqueHomingController::detectStall(const TorqueHomingPassConfig& pc) {
    if (m_positionWindow.empty()) return false;

    // Need at least 2 samples.
    if (!m_windowFilled && m_windowIndex < 2) return false;

    // Determine the window slice covering the last stallWindowMs.
    uint64_t now = getTimeMs();
    if (pc.stallWindowMs == 0) return false;
    uint64_t windowStart = (now >= pc.stallWindowMs)
        ? (now - pc.stallWindowMs) : 0;

    // Walk backwards through the ring buffer collecting samples within window.
    size_t n = m_positionWindow.size();
    size_t end = m_windowIndex;      // points to next write slot
    size_t count = 0;
    int32_t minPos = 0, maxPos = 0;
    bool first = true;
    uint64_t oldestTime = 0;

    for (size_t i = 0; i < n; ++i) {
        size_t idx = (end + n - 1 - i) % n;
        if (!m_windowFilled && i >= m_windowIndex) break; // unwritten slot
        if (m_timeWindow[idx] < windowStart) break;

        int32_t p = m_positionWindow[idx];
        // The loop walks backwards (newest -> oldest), so the last sample
        // we read is the oldest. Update oldestTime on every iteration.
        oldestTime = m_timeWindow[idx];
        if (first) {
            minPos = maxPos = p;
            first = false;
        } else {
            if (p < minPos) minPos = p;
            if (p > maxPos) maxPos = p;
        }
        ++count;
    }

    if (count < 2) return false;

    // Require the window to span at least stallWindowMs before declaring a
    // stall. This prevents false positives in the first few cycles when only
    // a handful of samples have been collected and per-cycle movement is
    // small relative to the threshold.
    if ((now - oldestTime) < pc.stallWindowMs) return false;

    // If neither detector is enabled, we cannot declare a stall.
    if (!m_config.usePositionStallDetection &&
        !m_config.useTorqueSaturationDetection) {
        return false;
    }

    // Position-derivative detector
    bool posStall = (maxPos - minPos) <= pc.stallPositionThreshold;

    // Torque-saturation detector: only check the most recent sample.
    bool torqueOk = false;
    bool torqueChecked = false;
    if (m_config.useTorqueSaturationDetection) {
        int16_t actual = callGetActualTorque();
        int16_t target = pc.targetTorque;
        int16_t diff = static_cast<int16_t>(std::abs(
            static_cast<int>(actual) - static_cast<int>(target)));
        torqueOk = (diff <= pc.stallTorqueTolerance);
        torqueChecked = true;
    }

    bool posOk = !m_config.usePositionStallDetection || posStall;
    bool tqOk = !m_config.useTorqueSaturationDetection ||
                (torqueChecked && torqueOk);

    return posOk && tqOk;
}

void TorqueHomingController::resetStallWindow(int32_t initialPosition) {
    // Size the ring buffer so that it can hold at least stallWindowMs worth
    // of samples. We don't know the cycle time, so use a generous capacity
    // bounded by kStallWindowCapacity. Pre-fill with the initial position
    // and current time so detectStall() has a valid baseline.
    uint32_t windowMs = 0;
    if (m_state == TorqueHomingState::CoarseApproach) {
        windowMs = m_config.coarsePass.stallWindowMs;
    } else if (m_state == TorqueHomingState::FineApproach &&
               m_currentPass >= 0 &&
               static_cast<size_t>(m_currentPass) < m_config.finePasses.size()) {
        windowMs = m_config.finePasses[static_cast<size_t>(m_currentPass)]
                       .stallWindowMs;
    }
    if (windowMs == 0) windowMs = 200;

    // Heuristic capacity: assume ~1 ms cycle. Capped.
    size_t cap = windowMs + 4;
    if (cap > kStallWindowCapacity) cap = kStallWindowCapacity;
    if (cap < 4) cap = 4;

    m_positionWindow.assign(cap, initialPosition);
    // Pre-fill times with 0 so unwritten slots are excluded by the
    // `m_timeWindow[idx] < windowStart` check in detectStall() until they
    // are overwritten with a real sample time.
    m_timeWindow.assign(cap, 0);
    m_windowIndex = 0;
    m_windowFilled = false;
}

bool TorqueHomingController::checkPositionDeviation(int32_t actualPos) {
    int32_t dev = actualPos - m_positionAnchor;
    if (dev < 0) dev = -dev;
    return dev <= m_config.maxPositionDeviation;
}

void TorqueHomingController::startBackOff(const TorqueHomingPassConfig& pc) {
    int32_t pos = callGetActualPosition();
    m_backOffStartPos = pos;
    m_backOffTargetDelta = std::abs(pc.backOffDistance);
    // Back-off torque: use the upcoming fine pass's torque magnitude with
    // opposite sign to the approach direction.
    int16_t mag = std::abs(pc.targetTorque);
    if (mag == 0) mag = std::abs(m_config.coarsePass.targetTorque);
    if (mag == 0) mag = 100; // last-resort default
    m_backOffTorque = static_cast<int16_t>(-m_direction * mag);
    m_positionAnchor = pos; // reset deviation anchor for back-off
    callSetTargetTorque(0);
    setState(TorqueHomingState::BackOff);
}

void TorqueHomingController::startNextPass() {
    // m_currentPass is -1 after coarse; first call advances to 0.
    int next = m_currentPass + 1;
    if (static_cast<size_t>(next) >= m_config.finePasses.size()) {
        // No more fine passes — settle and attain.
        setState(TorqueHomingState::Settling);
        return;
    }
    m_currentPass = next;
    startBackOff(m_config.finePasses[static_cast<size_t>(next)]);
}

void TorqueHomingController::complete(bool success, TorqueHomingError err,
                                       const std::string& msg) {
    if (success) {
        setState(TorqueHomingState::Attained);
    } else {
        setError(err, msg);
    }
}

void TorqueHomingController::setState(TorqueHomingState s) {
    if (s == m_state) return;
    m_state = s;
    m_stateStartTimeMs = getTimeMs();
    if (m_callbacks.onStateChange) {
        m_callbacks.onStateChange(s);
    }
}

void TorqueHomingController::setError(TorqueHomingError err,
                                       const std::string& msg) {
    m_lastError = err;
    m_lastErrorMsg = msg;
    setState(TorqueHomingState::Error);
    // Perform cleanup immediately so the drive is safe even if update()
    // is not called again (e.g. the caller breaks out of the loop because
    // update() returned false).
    if (m_config.stopOnError) {
        callSetTargetTorque(0);
        callStopMotion();
    }
    if (m_config.restorePreviousMode && m_modeSaved) {
        callSetOperatingMode(m_previousMode);
        m_modeSaved = false;
    }
    if (m_callbacks.onError) {
        m_callbacks.onError(err, msg);
    }
}

uint64_t TorqueHomingController::getTimeMs() const {
    if (m_callbacks.getTimeMs) return m_callbacks.getTimeMs();
    using namespace std::chrono;
    return duration_cast<milliseconds>(
        steady_clock::now().time_since_epoch()
    ).count();
}

// ============================================================================
// Callback / backend fallback accessors
// ============================================================================

int32_t TorqueHomingController::callGetActualPosition() {
    if (m_callbacks.getActualPosition) return m_callbacks.getActualPosition();
    if (m_backend) return m_backend->getActualPosition();
    return 0;
}

int32_t TorqueHomingController::callGetActualVelocity() {
    if (m_callbacks.getActualVelocity) return m_callbacks.getActualVelocity();
    if (m_backend) return m_backend->getActualVelocity();
    return 0;
}

int16_t TorqueHomingController::callGetActualTorque() {
    if (m_callbacks.getActualTorque) return m_callbacks.getActualTorque();
    if (m_backend) return m_backend->getActualTorque();
    return 0;
}

bool TorqueHomingController::callHasDriveFault() {
    if (m_callbacks.hasDriveFault) return m_callbacks.hasDriveFault();
    if (m_backend) {
        uint8_t reg = m_backend->getErrorRegister();
        return reg != 0;
    }
    return false;
}

void TorqueHomingController::callSetTargetTorque(int16_t torque) {
    if (m_callbacks.setTargetTorque) {
        m_callbacks.setTargetTorque(torque);
        return;
    }
    if (m_backend) m_backend->setTargetTorque(torque);
}

bool TorqueHomingController::callSetOperatingMode(OperatingMode mode) {
    if (m_callbacks.setOperatingMode) return m_callbacks.setOperatingMode(mode);
    if (m_backend) return m_backend->setOperatingMode(mode);
    return false;
}

OperatingMode TorqueHomingController::callGetOperatingMode() {
    if (m_callbacks.getOperatingMode) return m_callbacks.getOperatingMode();
    if (m_backend) return m_backend->getOperatingMode();
    return OperatingMode::NoMode;
}

void TorqueHomingController::callStopMotion() {
    if (m_callbacks.stopMotion) {
        m_callbacks.stopMotion();
        return;
    }
    if (m_backend) m_backend->setTargetTorque(0);
}

void TorqueHomingController::callSetHomePosition(int32_t pos) {
    if (m_callbacks.setHomePosition) {
        m_callbacks.setHomePosition(pos);
        return;
    }
    (void)pos;
}

bool TorqueHomingController::validateConfig() {
    if (m_config.coarsePass.targetTorque == 0) {
        setError(TorqueHomingError::InvalidConfig,
                 "coarsePass.targetTorque must be non-zero");
        return false;
    }
    if (!m_callbacks.getActualPosition && !m_backend) {
        setError(TorqueHomingError::NotInitialized,
                 "No position source (callback or backend)");
        return false;
    }
    if (!m_callbacks.setTargetTorque && !m_backend) {
        setError(TorqueHomingError::NotInitialized,
                 "No torque sink (callback or backend)");
        return false;
    }
    if (!m_callbacks.setOperatingMode && !m_backend) {
        setError(TorqueHomingError::NotInitialized,
                 "No operating-mode setter (callback or backend)");
        return false;
    }
    if (m_config.usePositionStallDetection &&
        m_config.coarsePass.stallWindowMs == 0) {
        setError(TorqueHomingError::InvalidConfig,
                 "usePositionStallDetection requires coarsePass.stallWindowMs > 0");
        return false;
    }
    if (m_config.useTorqueSaturationDetection &&
        !m_callbacks.getActualTorque && !m_backend) {
        setError(TorqueHomingError::NotInitialized,
                 "useTorqueSaturationDetection requires an actual-torque source");
        return false;
    }
    if (!m_config.usePositionStallDetection &&
        !m_config.useTorqueSaturationDetection) {
        setError(TorqueHomingError::InvalidConfig,
                 "At least one stall detector must be enabled");
        return false;
    }
    return true;
}

} // namespace CiA402

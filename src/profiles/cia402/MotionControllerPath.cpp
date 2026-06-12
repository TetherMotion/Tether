/**
 * @file MotionControllerPath.cpp
 * @brief MotionController implementation - Multi-axis coordination and path execution
 * 
 * This file contains the MotionController class implementation including:
 * - Axis management (add, remove, enable/disable)
 * - Coordinated motion (linear, circular, helical)
 * - Path execution and sampling
 * - Gearing and synchronization
 * - Homing coordination
 * - Utility functions
 */

#include "profiles/cia402/MotionController.hpp"
#include "tether/platform/EspCompat.hpp"
#include <algorithm>
#include <cmath>

static const char* TAG = "MotionController";

namespace CiA402 {

// ============================================================================
// MotionController Implementation - Construction and Axis Management
// ============================================================================

MotionController::MotionController() = default;
MotionController::~MotionController() = default;

CiA402AxisPtr MotionController::addAxis(CiA402Axis::AxisId id, DriveBackendUPtr backend) {
    std::lock_guard<std::mutex> lock(m_mutex);
    
    auto axis = std::make_shared<CiA402Axis>(id, std::move(backend));
    m_axes[id] = axis;
    
    TETHER_LOGI(TAG, "Added axis %lu", (unsigned long)id);
    return axis;
}

CiA402AxisPtr MotionController::getAxis(CiA402Axis::AxisId id) {
    std::lock_guard<std::mutex> lock(m_mutex);
    
    auto it = m_axes.find(id);
    return (it != m_axes.end()) ? it->second : nullptr;
}

bool MotionController::removeAxis(CiA402Axis::AxisId id) {
    std::lock_guard<std::mutex> lock(m_mutex);
    return m_axes.erase(id) > 0;
}

// ============================================================================
// Collective Operations
// ============================================================================

bool MotionController::enableAll(uint32_t timeoutMs) {
    for (auto& [id, axis] : m_axes) {
        if (!axis->enable(timeoutMs)) {
            TETHER_LOGE(TAG, "Failed to enable axis %lu", (unsigned long)id);
            return false;
        }
    }
    return true;
}

bool MotionController::disableAll(uint32_t timeoutMs) {
    bool success = true;
    for (auto& [id, axis] : m_axes) {
        if (!axis->disable(timeoutMs)) {
            success = false;
        }
    }
    return success;
}

void MotionController::quickStopAll() {
    for (auto& [id, axis] : m_axes) {
        axis->quickStop();
    }
}

void MotionController::clearAllFaults() {
    for (auto& [id, axis] : m_axes) {
        axis->clearFault();
    }
}

bool MotionController::allEnabled() const {
    for (const auto& [id, axis] : m_axes) {
        if (axis->getState() != State::OperationEnabled) {
            return false;
        }
    }
    return true;
}

bool MotionController::anyFault() const {
    for (const auto& [id, axis] : m_axes) {
        if (axis->getState() == State::Fault) {
            return true;
        }
    }
    return false;
}

// ============================================================================
// Coordinated Motion
// ============================================================================

void MotionController::setCoordinatedAxes(const std::vector<CiA402Axis::AxisId>& axisIds) {
    std::lock_guard<std::mutex> lock(m_mutex);
    m_coordinatedAxes = axisIds;
}

bool MotionController::moveLinear(const std::vector<double>& targetPositions, double velocity) {
    if (targetPositions.size() != m_coordinatedAxes.size()) {
        TETHER_LOGE(TAG, "Target position count doesn't match coordinated axis count");
        return false;
    }
    
    // Get current positions
    std::vector<double> startPositions;
    for (auto id : m_coordinatedAxes) {
        auto axis = getAxis(id);
        if (!axis) return false;
        startPositions.push_back(static_cast<double>(axis->getStatus().actualPosition));
    }
    
    // Create linear path using LinearConfig
    LinearConfig config;
    config.numAxes = std::min(startPositions.size(), static_cast<size_t>(MAX_PATH_AXES));
    for (size_t i = 0; i < config.numAxes; ++i) {
        config.start[i] = startPositions[i];
        config.end[i] = targetPositions[i];
    }
    auto path = std::make_unique<LinearPath>(config);
    
    // Create path sampler
    m_pathSampler = std::make_unique<PathSampler>(std::move(path), velocity);
    
    // Start execution
    m_pathTime = 0.0;
    m_pathExecuting = true;
    
    // Set all axes to CSP mode
    for (auto id : m_coordinatedAxes) {
        auto axis = getAxis(id);
        if (axis) {
            axis->setOperatingMode(OperatingMode::CyclicSyncPosition);
        }
    }
    
    return true;
}

bool MotionController::moveCircular(double centerX, double centerY, 
                                    double endX, double endY,
                                    bool clockwise, double velocity) {
    if (m_coordinatedAxes.size() < 2) {
        TETHER_LOGE(TAG, "Circular move requires at least 2 axes");
        return false;
    }
    
    // Get current positions for first two axes
    auto axis0 = getAxis(m_coordinatedAxes[0]);
    auto axis1 = getAxis(m_coordinatedAxes[1]);
    if (!axis0 || !axis1) return false;
    
    double startX = static_cast<double>(axis0->getStatus().actualPosition);
    double startY = static_cast<double>(axis1->getStatus().actualPosition);
    
    // Create circular path using CircularConfig
    CircularConfig config;
    config.center = {centerX, centerY, 0.0};
    config.start = {startX, startY, 0.0};
    config.end = {endX, endY, 0.0};
    config.plane = Plane::XY;
    config.direction = clockwise ? ArcDirection::CW : ArcDirection::CCW;
    
    auto path = std::make_unique<CircularPath>(config);
    
    // Create path sampler
    m_pathSampler = std::make_unique<PathSampler>(std::move(path), velocity);
    
    // Start execution
    m_pathTime = 0.0;
    m_pathExecuting = true;
    
    return true;
}

bool MotionController::moveHelical(double centerX, double centerY,
                                   double endX, double endY,
                                   double pitch, bool clockwise, double velocity) {
    if (m_coordinatedAxes.size() < 3) {
        TETHER_LOGE(TAG, "Helical move requires at least 3 axes");
        return false;
    }
    
    auto axis0 = getAxis(m_coordinatedAxes[0]);
    auto axis1 = getAxis(m_coordinatedAxes[1]);
    auto axis2 = getAxis(m_coordinatedAxes[2]);
    if (!axis0 || !axis1 || !axis2) return false;
    
    double startX = static_cast<double>(axis0->getStatus().actualPosition);
    double startY = static_cast<double>(axis1->getStatus().actualPosition);
    // startZ not needed for helical config, Z is computed from pitch
    
    // Calculate radius from start position to center
    double dx = startX - centerX;
    double dy = startY - centerY;
    double radius = std::sqrt(dx * dx + dy * dy);
    
    // Calculate start and end angles
    double startAngle = std::atan2(dy, dx);
    double endDx = endX - centerX;
    double endDy = endY - centerY;
    double endAngle = std::atan2(endDy, endDx);
    
    // Calculate total angle based on direction
    double totalAngle = endAngle - startAngle;
    if (clockwise && totalAngle > 0) {
        totalAngle -= 2 * M_PI;
    } else if (!clockwise && totalAngle < 0) {
        totalAngle += 2 * M_PI;
    }
    
    // Create helical path using HelicalConfig
    HelicalConfig config;
    config.center = {centerX, centerY, 0.0};
    config.radius = radius;
    config.pitch = pitch;
    config.startAngle = startAngle;
    config.totalAngle = totalAngle;
    config.plane = Plane::XY;
    config.direction = clockwise ? ArcDirection::CW : ArcDirection::CCW;
    
    auto path = std::make_unique<HelicalPath>(config);
    
    m_pathSampler = std::make_unique<PathSampler>(std::move(path), velocity);
    m_pathTime = 0.0;
    m_pathExecuting = true;
    
    return true;
}

// ============================================================================
// Path Execution
// ============================================================================

bool MotionController::executePath(MultiSegmentPath& path, double velocity) {
    m_pathSampler = std::make_unique<PathSampler>(
        std::make_unique<MultiSegmentPath>(std::move(path)), velocity);
    m_pathTime = 0.0;
    m_pathExecuting = true;
    
    return true;
}

bool MotionController::addPathSegment(std::unique_ptr<PathSegment> segment) {
    if (!m_activePath) {
        m_activePath = std::make_unique<MultiSegmentPath>();
    }
    m_activePath->addSegment(std::move(segment));
    return true;
}

bool MotionController::startPath(double velocity) {
    if (!m_activePath) return false;
    
    m_pathSampler = std::make_unique<PathSampler>(std::move(m_activePath), velocity);
    m_pathTime = 0.0;
    m_pathExecuting = true;
    
    return true;
}

void MotionController::stopPath() {
    m_pathExecuting = false;
    m_pathSampler.reset();
}

// ============================================================================
// Global Parameters
// ============================================================================

void MotionController::setSpeedFactor(float factor) {
    if (m_globalParams.allowNegativeSpeed) {
        factor = std::clamp(factor, -m_globalParams.maxSpeedFactor, 
                           m_globalParams.maxSpeedFactor);
    } else {
        factor = std::clamp(factor, m_globalParams.minSpeedFactor,
                           m_globalParams.maxSpeedFactor);
    }
    m_globalParams.speedFactor = factor;
}

void MotionController::setGlobalParams(const GlobalParams& params) {
    m_globalParams = params;
}

// ============================================================================
// Gearing
// ============================================================================

bool MotionController::configureGearing(CiA402Axis::AxisId slaveId, 
                                        CiA402Axis::AxisId masterId,
                                        int32_t numerator, int32_t denominator) {
    auto slave = getAxis(slaveId);
    auto master = getAxis(masterId);
    
    if (!slave || !master) {
        TETHER_LOGE(TAG, "Invalid axis IDs for gearing");
        return false;
    }
    
    slave->setGearingMaster(master.get());
    slave->setGearRatio(numerator, denominator);
    
    return true;
}

bool MotionController::enableGearing(CiA402Axis::AxisId slaveId, bool enable) {
    auto slave = getAxis(slaveId);
    if (!slave) return false;
    
    slave->enableGearing(enable);
    return true;
}

// ============================================================================
// Homing
// ============================================================================

bool MotionController::homeAxis(CiA402Axis::AxisId id, const HomingCommand& cmd) {
    auto axis = getAxis(id);
    if (!axis) return false;
    
    return axis->startHoming(cmd);
}

bool MotionController::homeAxesSequential(const std::vector<CiA402Axis::AxisId>& ids,
                                          const HomingCommand& cmd) {
    for (auto id : ids) {
        auto axis = getAxis(id);
        if (!axis) return false;
        
        if (!axis->startHoming(cmd)) {
            return false;
        }
        
        if (!axis->waitHomingComplete(static_cast<uint32_t>(cmd.timeoutMs))) {
            return false;
        }
    }
    return true;
}

bool MotionController::homeAxesSimultaneous(const std::vector<CiA402Axis::AxisId>& ids,
                                            const HomingCommand& cmd) {
    // Start homing on all axes
    for (auto id : ids) {
        auto axis = getAxis(id);
        if (!axis || !axis->startHoming(cmd)) {
            return false;
        }
    }
    
    // Wait for all to complete
    int64_t startTime = esp_timer_get_time();
    int64_t timeoutUs = cmd.timeoutMs * 1000;
    
    while ((esp_timer_get_time() - startTime) < timeoutUs) {
        bool allComplete = true;
        
        for (auto id : ids) {
            auto axis = getAxis(id);
            if (axis && !axis->isHomingComplete()) {
                allComplete = false;
                break;
            }
        }
        
        if (allComplete) {
            return true;
        }
        
        Tether::Platform::Clock::instance().delayMilliseconds(10);
    }
    
    return false;
}

bool MotionController::allHomed() const {
    for (const auto& [id, axis] : m_axes) {
        if (!axis->isHomed()) {
            return false;
        }
    }
    return true;
}

// ============================================================================
// Cyclic Update
// ============================================================================

void MotionController::update(float dtSeconds) {
    // Apply speed factor to dt
    float effectiveDt = dtSeconds * m_globalParams.speedFactor;
    
    // Update path execution
    if (m_pathExecuting) {
        updatePathExecution(effectiveDt);
    }
    
    // Update all axes
    for (auto& [id, axis] : m_axes) {
        axis->update(dtSeconds); // Axes use real dt
    }
}

void MotionController::updatePathExecution(float dt) {
    if (!m_pathSampler || !m_pathExecuting) return;
    
    // Advance time
    m_pathTime += static_cast<double>(dt);
    
    // Sample path
    PathPoint point = m_pathSampler->sampleAtTime(m_pathTime);
    
    // Apply to axes
    applyPathPoint(point);
    
    // Check if complete
    if (m_pathSampler->isComplete(m_pathTime)) {
        m_pathExecuting = false;
        TETHER_LOGI(TAG, "Path execution complete");
    }
}

void MotionController::applyPathPoint(const PathPoint& point) {
    size_t numAxes = std::min(m_coordinatedAxes.size(), point.position.size());
    
    for (size_t i = 0; i < numAxes; i++) {
        auto axis = getAxis(m_coordinatedAxes[i]);
        if (axis) {
            int32_t pos = static_cast<int32_t>(point.position[i]);
            int32_t vel = (i < point.velocity.size()) ? 
                         static_cast<int32_t>(point.velocity[i]) : 0;
            axis->setCyclicPosition(pos, vel, 0);
        }
    }
}

// ============================================================================
// Utility Functions
// ============================================================================

std::unique_ptr<MotionProfile> createProfile(ProfileType type, 
                                             const MotionLimits& limits) {
    std::unique_ptr<MotionProfile> profile;
    
    switch (type) {
        case ProfileType::Linear:
            profile = std::make_unique<LinearProfile>();
            break;
        case ProfileType::Trapezoidal:
            profile = std::make_unique<TrapezoidalProfile>();
            break;
        case ProfileType::Triangular:
            profile = std::make_unique<TriangularProfile>();
            break;
        case ProfileType::SCurve:
            profile = std::make_unique<SCurveProfile>();
            break;
        case ProfileType::Polynomial:
            profile = std::make_unique<PolynomialProfile>();
            break;
        default:
            profile = std::make_unique<TrapezoidalProfile>();
            break;
    }
    
    if (profile) {
        profile->setLimits(limits);
    }
    
    return profile;
}

const char* errorToString(uint16_t errorCode) {
    switch (static_cast<ErrorCode>(errorCode)) {
        case ErrorCode::None:                  return "No Error";
        case ErrorCode::GenericError:          return "Generic Error";
        case ErrorCode::OverCurrent:           return "Overcurrent";
        case ErrorCode::OverVoltage:           return "Overvoltage";
        case ErrorCode::UnderVoltage:          return "Undervoltage";
        case ErrorCode::OverTemperature:       return "Overtemperature";
        case ErrorCode::SupplyTemp:            return "Supply Temperature Error";
        case ErrorCode::EncoderError:          return "Encoder Error";
        case ErrorCode::MotorBlocked:          return "Motor Blocked";
        case ErrorCode::FollowingError:        return "Following Error";
        case ErrorCode::PositionLimit:         return "Position Limit";
        case ErrorCode::VelocityLimit:         return "Velocity Limit";
        case ErrorCode::CommunicationError:    return "Communication Error";
        case ErrorCode::HomingError:           return "Homing Error";
        default:                               return "Unknown Error";
    }
}

} // namespace CiA402

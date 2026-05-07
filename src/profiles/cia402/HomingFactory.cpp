/**
 * @file HomingFactory.cpp
 * @brief HomingFactory and HomingUtils implementations
 * 
 * Split from HomingModes.cpp for maintainability.
 */

#include "profiles/cia402/HomingModes.hpp"
#include <algorithm>
#include <sstream>

namespace CiA402 {
namespace Homing {

// ============================================================================
// HomingFactory Implementation
// ============================================================================

HomingConfig HomingFactory::createServoConfig(
    int32_t searchVelocity,
    int32_t zeroVelocity,
    int32_t homeOffset
) {
    HomingConfig config;
    config.searchVelocity = searchVelocity;
    config.zeroVelocity = zeroVelocity;
    config.homeOffset = homeOffset;
    config.acceleration = searchVelocity * 10;
    config.deceleration = searchVelocity * 10;
    config.searchTimeoutMs = 30000;
    config.indexTimeoutMs = 5000;
    config.validateEndstops = true;
    config.validateIndex = true;
    return config;
}

HomingConfig HomingFactory::createLinearAxisConfig(
    int32_t maxTravel,
    int32_t searchVelocity,
    int32_t zeroVelocity
) {
    HomingConfig config;
    config.searchVelocity = searchVelocity;
    config.zeroVelocity = zeroVelocity;
    config.acceleration = searchVelocity * 5;
    config.deceleration = searchVelocity * 5;
    
    // Calculate timeout based on max travel
    uint32_t travelTime = static_cast<uint32_t>(
        (maxTravel * 1000.0) / searchVelocity + 5000);
    config.searchTimeoutMs = travelTime;
    config.indexTimeoutMs = 10000;
    
    config.validateEndstops = true;
    config.stopOnError = true;
    config.faultOnError = true;
    
    return config;
}

HomingConfig HomingFactory::createRotaryAxisConfig(
    int32_t searchVelocity,
    bool useIndex
) {
    HomingConfig config;
    config.searchVelocity = searchVelocity;
    config.zeroVelocity = searchVelocity / 10;
    config.acceleration = searchVelocity * 20;
    config.deceleration = searchVelocity * 20;
    
    // For rotary, we might not have limits
    config.searchTimeoutMs = 10000;
    config.indexTimeoutMs = 3000;
    
    config.validateEndstops = false; // Rotary may not have limits
    config.validateIndex = useIndex;
    
    return config;
}

HomingConfig HomingFactory::createStrictConfig(const HomingConfig& base) {
    HomingConfig strict = base;
    strict.validateEndstops = true;
    strict.validateIndex = true;
    strict.stopOnError = true;
    strict.faultOnError = true;
    strict.endstopDebounceMs = 20;
    return strict;
}

HomingConfig HomingFactory::createLenientConfig(const HomingConfig& base) {
    HomingConfig lenient = base;
    lenient.validateEndstops = false;
    lenient.validateIndex = false;
    lenient.stopOnError = true;
    lenient.faultOnError = false;
    lenient.endstopDebounceMs = 5;
    lenient.searchTimeoutMs *= 2;
    lenient.indexTimeoutMs *= 2;
    return lenient;
}

// ============================================================================
// HomingUtils Implementation
// ============================================================================

namespace HomingUtils {

std::string describeMethod(HomingMethod method) {
    return HomingStateMachine::getMethodName(method);
}

MethodRequirements getMethodRequirements(HomingMethod method) {
    MethodRequirements req;
    req.needsNegativeLimit = HomingStateMachine::methodRequiresNegativeLimit(method);
    req.needsPositiveLimit = HomingStateMachine::methodRequiresPositiveLimit(method);
    req.needsHomeSwitch = HomingStateMachine::methodRequiresHomeSwitch(method);
    req.needsIndex = HomingStateMachine::methodRequiresIndex(method);
    req.needsMotion = (method != HomingMethod::MethodCurrentPosition);
    req.description = HomingStateMachine::getMethodName(method);
    return req;
}

HomingMethod suggestMethod(
    bool hasNegativeLimit,
    bool hasPositiveLimit,
    bool hasHomeSwitch,
    bool hasIndex
) {
    // Prefer methods with more precision
    if (hasHomeSwitch && hasIndex) {
        return HomingMethod::MethodHomePosIndex; // Method 7
    }
    if (hasNegativeLimit && hasIndex) {
        return HomingMethod::MethodNegLimitIndex; // Method 1
    }
    if (hasPositiveLimit && hasIndex) {
        return HomingMethod::MethodPosLimitIndex; // Method 2
    }
    if (hasHomeSwitch) {
        return HomingMethod::MethodHomePos; // Method 23
    }
    if (hasNegativeLimit) {
        return HomingMethod::MethodNegLimit; // Method 17
    }
    if (hasPositiveLimit) {
        return HomingMethod::MethodPosLimit; // Method 18
    }
    if (hasIndex) {
        return HomingMethod::MethodIndexPos; // Method 34
    }
    
    // Fall back to current position
    return HomingMethod::MethodCurrentPosition; // Method 37
}

bool validateMethod(
    HomingMethod method,
    bool hasNegativeLimit,
    bool hasPositiveLimit,
    bool hasHomeSwitch,
    bool hasIndex
) {
    MethodRequirements req = getMethodRequirements(method);
    
    if (req.needsNegativeLimit && !hasNegativeLimit) return false;
    if (req.needsPositiveLimit && !hasPositiveLimit) return false;
    if (req.needsHomeSwitch && !hasHomeSwitch) return false;
    if (req.needsIndex && !hasIndex) return false;
    
    return true;
}

} // namespace HomingUtils

} // namespace Homing
} // namespace CiA402

/**
 * @file MachineTesterTrajectory.cpp
 * @brief Trajectory generation functions for machine testing
 * 
 * Contains all trajectory generation methods for MachineTester:
 * - Single axis: sinusoid, ramp, S-curve, step, triangular, trapezoidal
 * - Multi axis: circle, ellipse, helix, Lissajous, square, rounded square
 * 
 * Split from MachineTester.cpp
 */

#include "MachineTester.hpp"
#include <cmath>
#include <array>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace MotionReplanner {

//-----------------------------------------------------------------------------
// Single Axis Trajectory Generation
//-----------------------------------------------------------------------------

std::vector<PositionSample> MachineTester::generateSinusoid(const SingleAxisTestConfig& config) {
    std::vector<PositionSample> samples;
    
    const double dt = 0.001; // 1ms time step
    const double duration = config.cycles / config.frequency;
    const size_t numSamples = static_cast<size_t>(duration / dt);
    
    samples.reserve(numSamples);
    
    for (size_t i = 0; i < numSamples; ++i) {
        double t = i * dt;
        double omega = 2.0 * M_PI * config.frequency;
        
        PositionSample sample;
        sample.timestamp = t;
        sample.position[config.axis] = config.centerPosition + 
                                       config.amplitude * std::sin(omega * t);
        sample.velocity[config.axis] = config.amplitude * omega * std::cos(omega * t);
        sample.velocityValid = true;
        
        samples.push_back(sample);
    }
    
    return samples;
}

std::vector<PositionSample> MachineTester::generateRamp(const SingleAxisTestConfig& config) {
    std::vector<PositionSample> samples;
    
    const double dt = 0.001;
    double velMmPerSec = config.velocity / 60.0;
    double duration = std::abs(config.amplitude) / velMmPerSec;
    const size_t numSamples = static_cast<size_t>(duration / dt) + 1;
    
    samples.reserve(numSamples);
    
    double direction = (config.amplitude >= 0) ? 1.0 : -1.0;
    
    for (size_t i = 0; i < numSamples; ++i) {
        double t = i * dt;
        
        PositionSample sample;
        sample.timestamp = t;
        sample.position[config.axis] = config.centerPosition + direction * velMmPerSec * t;
        sample.velocity[config.axis] = direction * velMmPerSec;
        sample.velocityValid = true;
        
        samples.push_back(sample);
    }
    
    return samples;
}

std::vector<PositionSample> MachineTester::generateSCurve(const SingleAxisTestConfig& config) {
    std::vector<PositionSample> samples;
    
    const double dt = 0.001;
    double jerk = config.jerk;
    double accel = config.acceleration;
    double velMax = config.velocity / 60.0;
    
    // Calculate S-curve timing
    double ta = accel / jerk;                    // Jerk phase time
    double va = 0.5 * jerk * ta * ta;            // Velocity after jerk phase
    
    double tcv = (velMax - 2 * va) / accel;      // Constant accel phase time
    if (tcv < 0) tcv = 0;
    
    double totalTime = 4 * ta + 2 * tcv;         // Total acceleration time
    double distAccel = velMax * (2 * ta + tcv) - 0.5 * accel * (2 * ta + tcv) * (2 * ta + tcv);
    
    double tcruise = (config.amplitude - 2 * distAccel) / velMax;
    if (tcruise < 0) tcruise = 0;
    
    double totalDuration = 2 * totalTime + tcruise;
    const size_t numSamples = static_cast<size_t>(totalDuration / dt) + 1;
    
    samples.reserve(numSamples);
    
    double pos = config.centerPosition - config.amplitude / 2;
    double vel = 0.0;
    double acc = 0.0;
    
    for (size_t i = 0; i < numSamples; ++i) {
        double t = i * dt;
        
        // Simplified S-curve integration
        PositionSample sample;
        sample.timestamp = t;
        
        // Calculate position based on phase
        // (Simplified - proper S-curve would have 7 phases)
        if (t < totalTime) {
            // Acceleration phase
            double phase = t / totalTime;
            double smoothPhase = phase * phase * (3 - 2 * phase); // Smoothstep
            sample.position[config.axis] = config.centerPosition - config.amplitude / 2 +
                                          distAccel * smoothPhase;
            sample.velocity[config.axis] = velMax * 6 * phase * (1 - phase) / totalTime;
        } else if (t < totalTime + tcruise) {
            // Cruise phase
            sample.position[config.axis] = config.centerPosition - config.amplitude / 2 +
                                          distAccel + velMax * (t - totalTime);
            sample.velocity[config.axis] = velMax;
        } else {
            // Deceleration phase
            double tDecel = t - totalTime - tcruise;
            double phase = tDecel / totalTime;
            double smoothPhase = phase * phase * (3 - 2 * phase);
            sample.position[config.axis] = config.centerPosition + config.amplitude / 2 -
                                          distAccel * (1 - smoothPhase);
            sample.velocity[config.axis] = velMax * (1 - 6 * phase * (1 - phase) / totalTime);
        }
        
        sample.velocityValid = true;
        samples.push_back(sample);
    }
    
    return samples;
}

std::vector<PositionSample> MachineTester::generateStep(const SingleAxisTestConfig& config) {
    std::vector<PositionSample> samples;
    
    const double dt = 0.001;
    double stepTime = config.duration / config.cycles / 2;
    const size_t numSamples = static_cast<size_t>(config.duration / dt);
    
    samples.reserve(numSamples);
    
    for (size_t i = 0; i < numSamples; ++i) {
        double t = i * dt;
        int stepNum = static_cast<int>(t / stepTime);
        
        PositionSample sample;
        sample.timestamp = t;
        sample.position[config.axis] = config.centerPosition + 
            ((stepNum % 2 == 0) ? 0 : config.amplitude);
        sample.velocity[config.axis] = 0;
        sample.velocityValid = true;
        
        samples.push_back(sample);
    }
    
    return samples;
}

std::vector<PositionSample> MachineTester::generateTriangular(const SingleAxisTestConfig& config) {
    std::vector<PositionSample> samples;
    
    const double dt = 0.001;
    double period = 1.0 / config.frequency;
    const size_t numSamples = static_cast<size_t>(config.duration / dt);
    
    samples.reserve(numSamples);
    
    for (size_t i = 0; i < numSamples; ++i) {
        double t = i * dt;
        double phase = std::fmod(t, period) / period;
        
        PositionSample sample;
        sample.timestamp = t;
        
        // Triangle wave
        double value = (phase < 0.5) ? 
            (4 * phase - 1) : 
            (3 - 4 * phase);
        
        sample.position[config.axis] = config.centerPosition + config.amplitude * value;
        sample.velocity[config.axis] = config.amplitude * 4 * config.frequency * 
            ((phase < 0.5) ? 1 : -1);
        sample.velocityValid = true;
        
        samples.push_back(sample);
    }
    
    return samples;
}

std::vector<PositionSample> MachineTester::generateTrapezoidal(const SingleAxisTestConfig& config) {
    std::vector<PositionSample> samples;
    
    const double dt = 0.001;
    double velMax = config.velocity / 60.0;
    double accel = config.acceleration;
    
    double tAccel = velMax / accel;
    double distAccel = 0.5 * accel * tAccel * tAccel;
    double distCruise = config.amplitude - 2 * distAccel;
    double tCruise = distCruise / velMax;
    if (tCruise < 0) {
        // Triangle profile
        tAccel = std::sqrt(config.amplitude / accel);
        tCruise = 0;
        velMax = accel * tAccel;
        distAccel = config.amplitude / 2;
    }
    
    double totalTime = 2 * tAccel + tCruise;
    const size_t numSamples = static_cast<size_t>(totalTime / dt) + 1;
    
    samples.reserve(numSamples);
    
    for (size_t i = 0; i < numSamples; ++i) {
        double t = i * dt;
        
        PositionSample sample;
        sample.timestamp = t;
        
        if (t < tAccel) {
            // Acceleration
            sample.position[config.axis] = config.centerPosition + 0.5 * accel * t * t;
            sample.velocity[config.axis] = accel * t;
        } else if (t < tAccel + tCruise) {
            // Cruise
            double tCr = t - tAccel;
            sample.position[config.axis] = config.centerPosition + distAccel + velMax * tCr;
            sample.velocity[config.axis] = velMax;
        } else {
            // Deceleration
            double tDec = t - tAccel - tCruise;
            sample.position[config.axis] = config.centerPosition + distAccel + distCruise +
                velMax * tDec - 0.5 * accel * tDec * tDec;
            sample.velocity[config.axis] = velMax - accel * tDec;
        }
        
        sample.velocityValid = true;
        samples.push_back(sample);
    }
    
    return samples;
}

//-----------------------------------------------------------------------------
// Multi Axis Trajectory Generation
//-----------------------------------------------------------------------------

std::vector<PositionSample> MachineTester::generateCircle(const MultiAxisTestConfig& config) {
    std::vector<PositionSample> samples;
    
    const double dt = 0.001;
    double circumference = 2 * M_PI * config.radiusU;
    double duration = (circumference * config.revolutions) / (config.feedRate / 60.0);
    const size_t numSamples = static_cast<size_t>(duration / dt);
    
    samples.reserve(numSamples);
    
    double angularVel = (config.feedRate / 60.0) / config.radiusU;
    
    for (size_t i = 0; i < numSamples; ++i) {
        double t = i * dt;
        double angle = angularVel * t;
        
        PositionSample sample;
        sample.timestamp = t;
        
        sample.position[config.uAxis] = config.center[0] + config.radiusU * std::cos(angle);
        sample.position[config.vAxis] = config.center[1] + config.radiusV * std::sin(angle);
        sample.position[config.wAxis] = config.center[2];
        
        sample.velocity[config.uAxis] = -config.radiusU * angularVel * std::sin(angle);
        sample.velocity[config.vAxis] = config.radiusV * angularVel * std::cos(angle);
        sample.velocityValid = true;
        
        samples.push_back(sample);
    }
    
    return samples;
}

std::vector<PositionSample> MachineTester::generateEllipse(const MultiAxisTestConfig& config) {
    std::vector<PositionSample> samples;
    
    // Approximate ellipse circumference
    double a = config.radiusU, b = config.radiusV;
    double circumference = M_PI * (3 * (a + b) - std::sqrt((3*a + b) * (a + 3*b)));
    
    const double dt = 0.001;
    double duration = (circumference * config.revolutions) / (config.feedRate / 60.0);
    const size_t numSamples = static_cast<size_t>(duration / dt);
    
    samples.reserve(numSamples);
    
    double totalAngle = 2 * M_PI * config.revolutions;
    double angularVel = totalAngle / duration;
    
    // Apply rotation if specified
    double rotRad = config.rotationAngle * M_PI / 180.0;
    double cosRot = std::cos(rotRad);
    double sinRot = std::sin(rotRad);
    
    for (size_t i = 0; i < numSamples; ++i) {
        double t = i * dt;
        double angle = angularVel * t;
        
        double localU = config.radiusU * std::cos(angle);
        double localV = config.radiusV * std::sin(angle);
        
        PositionSample sample;
        sample.timestamp = t;
        
        // Apply rotation
        sample.position[config.uAxis] = config.center[0] + localU * cosRot - localV * sinRot;
        sample.position[config.vAxis] = config.center[1] + localU * sinRot + localV * cosRot;
        sample.position[config.wAxis] = config.center[2];
        
        double localVelU = -config.radiusU * angularVel * std::sin(angle);
        double localVelV = config.radiusV * angularVel * std::cos(angle);
        
        sample.velocity[config.uAxis] = localVelU * cosRot - localVelV * sinRot;
        sample.velocity[config.vAxis] = localVelU * sinRot + localVelV * cosRot;
        sample.velocityValid = true;
        
        samples.push_back(sample);
    }
    
    return samples;
}

std::vector<PositionSample> MachineTester::generateHelix(const MultiAxisTestConfig& config) {
    auto samples = generateCircle(config);
    
    // Add Z motion
    double totalHeight = config.pitchW * config.revolutions;
    double duration = samples.back().timestamp;
    
    for (auto& sample : samples) {
        sample.position[config.wAxis] = config.center[2] + 
            (sample.timestamp / duration) * totalHeight;
        sample.velocity[config.wAxis] = totalHeight / duration;
    }
    
    return samples;
}

std::vector<PositionSample> MachineTester::generateLissajous(const MultiAxisTestConfig& config) {
    std::vector<PositionSample> samples;
    
    const double dt = 0.001;
    const size_t numSamples = static_cast<size_t>(config.duration / dt);
    
    samples.reserve(numSamples);
    
    double omegaU = 2 * M_PI * config.lissajousRatioU / config.duration * config.revolutions;
    double omegaV = 2 * M_PI * config.lissajousRatioV / config.duration * config.revolutions;
    double phaseRad = config.lissajousPhase * M_PI / 180.0;
    
    for (size_t i = 0; i < numSamples; ++i) {
        double t = i * dt;
        
        PositionSample sample;
        sample.timestamp = t;
        
        sample.position[config.uAxis] = config.center[0] + config.radiusU * std::sin(omegaU * t);
        sample.position[config.vAxis] = config.center[1] + config.radiusV * std::sin(omegaV * t + phaseRad);
        sample.position[config.wAxis] = config.center[2];
        
        sample.velocity[config.uAxis] = config.radiusU * omegaU * std::cos(omegaU * t);
        sample.velocity[config.vAxis] = config.radiusV * omegaV * std::cos(omegaV * t + phaseRad);
        sample.velocityValid = true;
        
        samples.push_back(sample);
    }
    
    return samples;
}

std::vector<PositionSample> MachineTester::generateSquare(const MultiAxisTestConfig& config) {
    std::vector<PositionSample> samples;
    
    const double dt = 0.001;
    double sideLength = 2 * config.radiusU;
    double perimeter = 4 * sideLength;
    double duration = perimeter * config.revolutions / (config.feedRate / 60.0);
    const size_t numSamples = static_cast<size_t>(duration / dt);
    
    samples.reserve(numSamples);
    
    double velMmPerSec = config.feedRate / 60.0;
    double halfSide = config.radiusU;
    
    // Corner positions
    std::array<std::array<double, 2>, 4> corners = {{
        {halfSide, halfSide},
        {-halfSide, halfSide},
        {-halfSide, -halfSide},
        {halfSide, -halfSide}
    }};
    
    double timePerSide = sideLength / velMmPerSec;
    
    for (size_t i = 0; i < numSamples; ++i) {
        double t = i * dt;
        double cycleTime = std::fmod(t, 4 * timePerSide);
        
        int side = static_cast<int>(cycleTime / timePerSide);
        double sideProgress = std::fmod(cycleTime, timePerSide) / timePerSide;
        
        PositionSample sample;
        sample.timestamp = t;
        
        int nextSide = (side + 1) % 4;
        sample.position[config.uAxis] = config.center[0] + 
            corners[side][0] + sideProgress * (corners[nextSide][0] - corners[side][0]);
        sample.position[config.vAxis] = config.center[1] + 
            corners[side][1] + sideProgress * (corners[nextSide][1] - corners[side][1]);
        sample.position[config.wAxis] = config.center[2];
        
        // Velocity direction based on side
        sample.velocity[config.uAxis] = (corners[nextSide][0] - corners[side][0]) / timePerSide;
        sample.velocity[config.vAxis] = (corners[nextSide][1] - corners[side][1]) / timePerSide;
        sample.velocityValid = true;
        
        samples.push_back(sample);
    }
    
    return samples;
}

std::vector<PositionSample> MachineTester::generateRoundedSquare(const MultiAxisTestConfig& config) {
    std::vector<PositionSample> samples;
    
    const double dt = 0.001;
    double sideLength = 2 * config.radiusU - 2 * config.cornerRadius;
    double arcLength = 0.5 * M_PI * config.cornerRadius;
    double perimeter = 4 * (sideLength + arcLength);
    double duration = perimeter * config.revolutions / (config.feedRate / 60.0);
    const size_t numSamples = static_cast<size_t>(duration / dt);
    
    samples.reserve(numSamples);
    
    double velMmPerSec = config.feedRate / 60.0;
    double halfSide = config.radiusU;
    double r = config.cornerRadius;
    
    double timePerSide = sideLength / velMmPerSec;
    double timePerArc = arcLength / velMmPerSec;
    double cycleTime = 4 * (timePerSide + timePerArc);
    
    for (size_t i = 0; i < numSamples; ++i) {
        double t = i * dt;
        double tc = std::fmod(t, cycleTime);
        
        PositionSample sample;
        sample.timestamp = t;
        
        // Determine which segment we're in
        double segmentTime = timePerSide + timePerArc;
        int segment = static_cast<int>(tc / segmentTime);
        double segProgress = std::fmod(tc, segmentTime);
        
        bool inArc = segProgress > timePerSide;
        double localT = inArc ? (segProgress - timePerSide) / timePerArc : segProgress / timePerSide;
        
        // Starting positions and directions for each segment
        // Simplified - full implementation would properly compute each segment
        double angle = segment * M_PI / 2;
        double cosA = std::cos(angle), sinA = std::sin(angle);
        
        if (!inArc) {
            // Straight segment
            double startU = (halfSide - r) * cosA - halfSide * sinA;
            double startV = (halfSide - r) * sinA + halfSide * cosA;
            double dirU = -sinA;
            double dirV = cosA;
            
            sample.position[config.uAxis] = config.center[0] + startU + localT * sideLength * dirU;
            sample.position[config.vAxis] = config.center[1] + startV + localT * sideLength * dirV;
            sample.velocity[config.uAxis] = velMmPerSec * dirU;
            sample.velocity[config.vAxis] = velMmPerSec * dirV;
        } else {
            // Arc segment
            double centerU = (halfSide - r) * (-sinA) + (halfSide - r) * (-cosA);
            double centerV = (halfSide - r) * cosA + (halfSide - r) * (-sinA);
            double startAngle = angle + M_PI;
            double arcAngle = startAngle + localT * M_PI / 2;
            
            sample.position[config.uAxis] = config.center[0] + centerU + r * std::cos(arcAngle);
            sample.position[config.vAxis] = config.center[1] + centerV + r * std::sin(arcAngle);
            
            double angularVel = velMmPerSec / r;
            sample.velocity[config.uAxis] = -r * angularVel * std::sin(arcAngle);
            sample.velocity[config.vAxis] = r * angularVel * std::cos(arcAngle);
        }
        
        sample.position[config.wAxis] = config.center[2];
        sample.velocityValid = true;
        samples.push_back(sample);
    }
    
    return samples;
}

} // namespace MotionReplanner

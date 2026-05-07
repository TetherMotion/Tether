/**
 * @file MachineTester.cpp
 * @brief Implementation of machine performance testing framework
 */

#include "MachineTester.hpp"
#include <cmath>
#include <algorithm>
#include <numeric>
#include <sstream>

namespace MotionReplanner {

//=============================================================================
// MachineTester Implementation
//=============================================================================

MachineTester::MachineTester() {}

void MachineTester::reportStatus(const std::string& message) {
    if (statusCallback_) {
        statusCallback_(message);
    }
}

//-----------------------------------------------------------------------------
// Single Axis Test
//-----------------------------------------------------------------------------

TestResult MachineTester::runSingleAxisTest(const SingleAxisTestConfig& config) {
    TestResult result;
    result.testType = "SingleAxis";
    
    std::string typeName;
    switch (config.type) {
        case SingleAxisTestType::Ramp: typeName = "Ramp"; break;
        case SingleAxisTestType::Sinusoid: typeName = "Sinusoid"; break;
        case SingleAxisTestType::SCurve: typeName = "SCurve"; break;
        case SingleAxisTestType::Step: typeName = "Step"; break;
        case SingleAxisTestType::Triangular: typeName = "Triangular"; break;
        case SingleAxisTestType::Trapezoidal: typeName = "Trapezoidal"; break;
    }
    
    std::stringstream ss;
    ss << "Axis" << config.axis << "_" << typeName 
       << "_A" << config.amplitude << "_F" << config.frequency;
    result.testName = ss.str();
    
    reportStatus("Running test: " + result.testName);
    
    // Generate desired trajectory
    std::vector<PositionSample> desired;
    switch (config.type) {
        case SingleAxisTestType::Ramp:
            desired = generateRamp(config);
            break;
        case SingleAxisTestType::Sinusoid:
            desired = generateSinusoid(config);
            break;
        case SingleAxisTestType::SCurve:
            desired = generateSCurve(config);
            break;
        case SingleAxisTestType::Step:
            desired = generateStep(config);
            break;
        case SingleAxisTestType::Triangular:
            desired = generateTriangular(config);
            break;
        case SingleAxisTestType::Trapezoidal:
            desired = generateTrapezoidal(config);
            break;
    }
    
    result.desiredSamples = desired;
    
    // Execute if callback is set
    if (commandCallback_ && feedbackCallback_) {
        if (commandCallback_(desired)) {
            result.actualSamples = feedbackCallback_();
            
            // Analyze results
            RollingStatistics errorStats;
            double maxVel = 0.0, maxAccel = 0.0;
            
            for (size_t i = 0; i < std::min(desired.size(), result.actualSamples.size()); ++i) {
                double error = std::abs(result.actualSamples[i].position[config.axis] - 
                                       desired[i].position[config.axis]);
                errorStats.addSample(error);
                
                double vel = std::abs(result.actualSamples[i].velocity[config.axis]);
                maxVel = std::max(maxVel, vel);
            }
            
            result.positionError.sampleCount = errorStats.count();
            result.positionError.minError = errorStats.min();
            result.positionError.maxError = errorStats.max();
            result.positionError.meanError = errorStats.mean();
            result.positionError.stdDev = errorStats.stdDev();
            result.positionError.rmsError = errorStats.rms();
            
            result.maxVelocityAchieved = maxVel * 60.0; // Convert to mm/min
            
            // Analyze step response if applicable
            if (config.type == SingleAxisTestType::Step) {
                analyzeStepResponse(result, desired, result.actualSamples);
            }
        } else {
            result.passed = false;
            result.failureReason = "Command execution failed";
        }
    }
    
    results_.push_back(result);
    return result;
}

//-----------------------------------------------------------------------------
// Multi Axis Test
//-----------------------------------------------------------------------------

TestResult MachineTester::runMultiAxisTest(const MultiAxisTestConfig& config) {
    TestResult result;
    result.testType = "MultiAxis";
    
    std::string typeName;
    switch (config.type) {
        case MultiAxisTestType::Circle: typeName = "Circle"; break;
        case MultiAxisTestType::Ellipse: typeName = "Ellipse"; break;
        case MultiAxisTestType::Helix: typeName = "Helix"; break;
        case MultiAxisTestType::Lissajous: typeName = "Lissajous"; break;
        case MultiAxisTestType::Square: typeName = "Square"; break;
        case MultiAxisTestType::RoundedSquare: typeName = "RoundedSquare"; break;
        case MultiAxisTestType::DiagonalBox: typeName = "DiagonalBox"; break;
    }
    
    std::stringstream ss;
    ss << typeName << "_R" << config.radiusU << "_F" << config.feedRate;
    result.testName = ss.str();
    
    reportStatus("Running test: " + result.testName);
    
    // Generate desired trajectory
    std::vector<PositionSample> desired;
    switch (config.type) {
        case MultiAxisTestType::Circle:
            desired = generateCircle(config);
            break;
        case MultiAxisTestType::Ellipse:
            desired = generateEllipse(config);
            break;
        case MultiAxisTestType::Helix:
            desired = generateHelix(config);
            break;
        case MultiAxisTestType::Lissajous:
            desired = generateLissajous(config);
            break;
        case MultiAxisTestType::Square:
            desired = generateSquare(config);
            break;
        case MultiAxisTestType::RoundedSquare:
            desired = generateRoundedSquare(config);
            break;
        case MultiAxisTestType::DiagonalBox:
            // TODO: Implement
            break;
    }
    
    result.desiredSamples = desired;
    
    // Execute if callback is set
    if (commandCallback_ && feedbackCallback_) {
        if (commandCallback_(desired)) {
            result.actualSamples = feedbackCallback_();
            
            // Analyze results
            analyzeCircularity(result, result.actualSamples, config);
            
            // Position error
            RollingStatistics errorStats;
            for (size_t i = 0; i < std::min(desired.size(), result.actualSamples.size()); ++i) {
                double dx = result.actualSamples[i].position[config.uAxis] - 
                           desired[i].position[config.uAxis];
                double dy = result.actualSamples[i].position[config.vAxis] - 
                           desired[i].position[config.vAxis];
                double error = std::sqrt(dx*dx + dy*dy);
                errorStats.addSample(error);
            }
            
            result.contourError.sampleCount = errorStats.count();
            result.contourError.minError = errorStats.min();
            result.contourError.maxError = errorStats.max();
            result.contourError.meanError = errorStats.mean();
            result.contourError.stdDev = errorStats.stdDev();
            result.contourError.rmsError = errorStats.rms();
        } else {
            result.passed = false;
            result.failureReason = "Command execution failed";
        }
    }
    
    results_.push_back(result);
    return result;
}

//-----------------------------------------------------------------------------
// Friction Test
//-----------------------------------------------------------------------------

TestResult MachineTester::runFrictionTest(const FrictionTestConfig& config) {
    TestResult result;
    result.testName = "FrictionTest_Axis" + std::to_string(config.axis);
    result.testType = "Friction";
    
    reportStatus("Running friction identification test on axis " + std::to_string(config.axis));
    
    std::vector<std::pair<double, std::vector<PositionSample>>> testData;
    
    for (double vel : config.velocities) {
        for (int dir = 0; dir < (config.bidirectional ? 2 : 1); ++dir) {
            double signedVel = (dir == 0) ? vel : -vel;
            
            for (int rep = 0; rep < config.repeats; ++rep) {
                SingleAxisTestConfig moveConfig;
                moveConfig.axis = config.axis;
                moveConfig.type = SingleAxisTestType::Ramp;
                moveConfig.velocity = std::abs(signedVel);
                moveConfig.amplitude = config.distance * (signedVel > 0 ? 1 : -1);
                
                auto desired = generateRamp(moveConfig);
                
                if (commandCallback_ && feedbackCallback_) {
                    if (commandCallback_(desired)) {
                        auto actual = feedbackCallback_();
                        testData.push_back({signedVel, actual});
                    }
                }
            }
        }
    }
    
    // Analyze friction
    analyzeFriction(result, testData);
    
    results_.push_back(result);
    return result;
}

//-----------------------------------------------------------------------------
// Delay Test
//-----------------------------------------------------------------------------

TestResult MachineTester::runDelayTest(const DelayTestConfig& config) {
    TestResult result;
    result.testName = "DelayTest_Axis" + std::to_string(config.axis);
    result.testType = "Delay";
    
    reportStatus("Running delay identification test on axis " + std::to_string(config.axis));
    
    // Generate step sequence
    SingleAxisTestConfig stepConfig;
    stepConfig.axis = config.axis;
    stepConfig.type = SingleAxisTestType::Step;
    stepConfig.amplitude = config.stepAmplitude;
    stepConfig.velocity = config.stepVelocity;
    stepConfig.cycles = config.stepCount;
    stepConfig.duration = config.stepCount * config.settleTime * 2;
    
    auto desired = generateStep(stepConfig);
    result.desiredSamples = desired;
    
    if (commandCallback_ && feedbackCallback_) {
        if (commandCallback_(desired)) {
            result.actualSamples = feedbackCallback_();
            
            // Find delay using cross-correlation
            result.detectedDelay = findDelay(desired, result.actualSamples,
                                             config.maxExpectedDelay,
                                             config.delayResolution);
            
            reportStatus("Detected system delay: " + 
                        std::to_string(result.detectedDelay * 1000.0) + " ms");
        }
    }
    
    results_.push_back(result);
    return result;
}

//-----------------------------------------------------------------------------
// PID Test
//-----------------------------------------------------------------------------

TestResult MachineTester::runPIDTest(const PIDTestConfig& config) {
    TestResult result;
    result.testName = "PIDTest_Axis" + std::to_string(config.axis);
    result.testType = "PID";
    
    reportStatus("Running PID tuning analysis on axis " + std::to_string(config.axis));
    
    // Run step response tests at different velocities
    std::vector<TestResult> stepResults;
    
    for (double vel : config.stepVelocities) {
        SingleAxisTestConfig stepConfig;
        stepConfig.axis = config.axis;
        stepConfig.type = SingleAxisTestType::Step;
        stepConfig.amplitude = config.stepAmplitude;
        stepConfig.velocity = vel;
        stepConfig.cycles = config.repeats;
        
        auto stepResult = runSingleAxisTest(stepConfig);
        stepResults.push_back(stepResult);
    }
    
    // Average the results
    if (!stepResults.empty()) {
        double sumOvershoot = 0, sumRiseTime = 0, sumSettlingTime = 0;
        for (const auto& sr : stepResults) {
            sumOvershoot += sr.overshoot;
            sumRiseTime += sr.riseTime;
            sumSettlingTime += sr.settlingTime;
        }
        result.overshoot = sumOvershoot / stepResults.size();
        result.riseTime = sumRiseTime / stepResults.size();
        result.settlingTime = sumSettlingTime / stepResults.size();
    }
    
    // Generate chirp signal for frequency response
    // (simplified - full implementation would do FFT analysis)
    
    results_.push_back(result);
    return result;
}

//-----------------------------------------------------------------------------
// Test Sequence
//-----------------------------------------------------------------------------

std::vector<TestResult> MachineTester::runTestSequence(const TestSequence& sequence) {
    std::vector<TestResult> sequenceResults;
    
    reportStatus("Starting test sequence: " + sequence.name);
    
    // Run delay test first if specified
    if (sequence.delayTest) {
        sequenceResults.push_back(runDelayTest(*sequence.delayTest));
    }
    
    // Run single axis tests
    for (const auto& config : sequence.singleAxisTests) {
        if (sequence.sweepAmplitudes) {
            for (double amp : sequence.amplitudes) {
                auto modConfig = config;
                modConfig.amplitude = amp;
                sequenceResults.push_back(runSingleAxisTest(modConfig));
            }
        } else if (sequence.sweepFrequencies) {
            for (double freq : sequence.frequencies) {
                auto modConfig = config;
                modConfig.frequency = freq;
                sequenceResults.push_back(runSingleAxisTest(modConfig));
            }
        } else {
            sequenceResults.push_back(runSingleAxisTest(config));
        }
    }
    
    // Run multi axis tests
    for (const auto& config : sequence.multiAxisTests) {
        if (sequence.sweepFeedRates) {
            for (double feed : sequence.feedRates) {
                auto modConfig = config;
                modConfig.feedRate = feed;
                sequenceResults.push_back(runMultiAxisTest(modConfig));
            }
        } else {
            sequenceResults.push_back(runMultiAxisTest(config));
        }
    }
    
    // Run friction test if specified
    if (sequence.frictionTest) {
        sequenceResults.push_back(runFrictionTest(*sequence.frictionTest));
    }
    
    // Run PID test if specified
    if (sequence.pidTest) {
        sequenceResults.push_back(runPIDTest(*sequence.pidTest));
    }
    
    reportStatus("Completed test sequence: " + sequence.name + 
                 " (" + std::to_string(sequenceResults.size()) + " tests)");
    
    return sequenceResults;
}

//-----------------------------------------------------------------------------
// Standard Test Sequences
//-----------------------------------------------------------------------------

TestSequence MachineTester::createQuickCalibration() {
    TestSequence seq;
    seq.name = "QuickCalibration";
    seq.description = "Quick calibration for basic machine limits";
    
    // Single axis sinusoids on X, Y, Z
    for (int axis = 0; axis < 3; ++axis) {
        SingleAxisTestConfig config;
        config.axis = axis;
        config.type = SingleAxisTestType::Sinusoid;
        config.amplitude = 50.0;
        config.frequency = 1.0;
        config.duration = 5.0;
        seq.singleAxisTests.push_back(config);
    }
    
    // XY circle
    MultiAxisTestConfig circleConfig;
    circleConfig.type = MultiAxisTestType::Circle;
    circleConfig.radiusU = 50.0;
    circleConfig.radiusV = 50.0;
    circleConfig.feedRate = 1000.0;
    circleConfig.revolutions = 3;
    seq.multiAxisTests.push_back(circleConfig);
    
    // Delay test
    seq.delayTest = DelayTestConfig{};
    seq.delayTest->axis = 0;
    
    return seq;
}

TestSequence MachineTester::createFullCalibration() {
    TestSequence seq;
    seq.name = "FullCalibration";
    seq.description = "Comprehensive calibration with parameter sweeps";
    
    // Single axis tests on all axes
    for (int axis = 0; axis < 3; ++axis) {
        SingleAxisTestConfig sinConfig;
        sinConfig.axis = axis;
        sinConfig.type = SingleAxisTestType::Sinusoid;
        sinConfig.amplitude = 50.0;
        seq.singleAxisTests.push_back(sinConfig);
        
        SingleAxisTestConfig rampConfig;
        rampConfig.axis = axis;
        rampConfig.type = SingleAxisTestType::Ramp;
        rampConfig.amplitude = 100.0;
        seq.singleAxisTests.push_back(rampConfig);
        
        SingleAxisTestConfig scurveConfig;
        scurveConfig.axis = axis;
        scurveConfig.type = SingleAxisTestType::SCurve;
        scurveConfig.amplitude = 50.0;
        seq.singleAxisTests.push_back(scurveConfig);
    }
    
    // Multi-axis tests
    MultiAxisTestConfig circleXY;
    circleXY.type = MultiAxisTestType::Circle;
    circleXY.uAxis = 0; circleXY.vAxis = 1;
    circleXY.radiusU = 50.0; circleXY.radiusV = 50.0;
    seq.multiAxisTests.push_back(circleXY);
    
    MultiAxisTestConfig circleXZ;
    circleXZ.type = MultiAxisTestType::Circle;
    circleXZ.uAxis = 0; circleXZ.vAxis = 2;
    circleXZ.radiusU = 50.0; circleXZ.radiusV = 50.0;
    seq.multiAxisTests.push_back(circleXZ);
    
    MultiAxisTestConfig ellipse;
    ellipse.type = MultiAxisTestType::Ellipse;
    ellipse.radiusU = 75.0; ellipse.radiusV = 50.0;
    seq.multiAxisTests.push_back(ellipse);
    
    // Friction and delay tests
    seq.frictionTest = FrictionTestConfig{};
    seq.delayTest = DelayTestConfig{};
    seq.pidTest = PIDTestConfig{};
    
    // Enable parameter sweeps
    seq.sweepFrequencies = true;
    seq.frequencies = {0.5, 1.0, 2.0, 5.0, 10.0};
    
    seq.sweepFeedRates = true;
    seq.feedRates = {500, 1000, 2000, 3000, 5000, 8000};
    
    return seq;
}

TestSequence MachineTester::createAxisCharacterization(int axis) {
    TestSequence seq;
    seq.name = "AxisCharacterization_" + std::to_string(axis);
    seq.description = "Detailed characterization of single axis";
    
    // Various test types
    for (auto type : {SingleAxisTestType::Sinusoid, SingleAxisTestType::Ramp,
                      SingleAxisTestType::SCurve, SingleAxisTestType::Triangular,
                      SingleAxisTestType::Trapezoidal}) {
        SingleAxisTestConfig config;
        config.axis = axis;
        config.type = type;
        config.amplitude = 50.0;
        seq.singleAxisTests.push_back(config);
    }
    
    // Sweep amplitudes and frequencies
    seq.sweepAmplitudes = true;
    seq.amplitudes = {10, 25, 50, 75, 100, 150, 200};
    
    seq.sweepFrequencies = true;
    seq.frequencies = {0.25, 0.5, 1.0, 2.0, 5.0, 10.0, 20.0};
    
    // Friction and delay
    seq.frictionTest = FrictionTestConfig{};
    seq.frictionTest->axis = axis;
    
    seq.delayTest = DelayTestConfig{};
    seq.delayTest->axis = axis;
    
    seq.pidTest = PIDTestConfig{};
    seq.pidTest->axis = axis;
    
    return seq;
}

TestSequence MachineTester::createWorkspaceMapping(const HeatmapConfig& heatmapConfig) {
    TestSequence seq;
    seq.name = "WorkspaceMapping";
    seq.description = "Map performance across workspace";
    
    // Generate test points across workspace
    double xRange = heatmapConfig.maxBounds[0] - heatmapConfig.minBounds[0];
    double yRange = heatmapConfig.maxBounds[1] - heatmapConfig.minBounds[1];
    double zRange = heatmapConfig.maxBounds[2] - heatmapConfig.minBounds[2];
    
    int numX = 5, numY = 5, numZ = 3;
    
    for (int iz = 0; iz < numZ; ++iz) {
        double z = heatmapConfig.minBounds[2] + (iz + 0.5) * zRange / numZ;
        
        for (int iy = 0; iy < numY; ++iy) {
            double y = heatmapConfig.minBounds[1] + (iy + 0.5) * yRange / numY;
            
            for (int ix = 0; ix < numX; ++ix) {
                double x = heatmapConfig.minBounds[0] + (ix + 0.5) * xRange / numX;
                
                // Small circle at each grid point
                MultiAxisTestConfig circleConfig;
                circleConfig.type = MultiAxisTestType::Circle;
                circleConfig.center = {x, y, z};
                circleConfig.radiusU = 20.0;
                circleConfig.radiusV = 20.0;
                circleConfig.feedRate = 1000.0;
                circleConfig.revolutions = 2;
                seq.multiAxisTests.push_back(circleConfig);
            }
        }
    }
    
    return seq;
}

//-----------------------------------------------------------------------------
// Trajectory Generation
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

//-----------------------------------------------------------------------------
// Analysis Helpers
//-----------------------------------------------------------------------------

void MachineTester::analyzeStepResponse(TestResult& result,
                                        const std::vector<PositionSample>& desired,
                                        const std::vector<PositionSample>& actual) {
    if (actual.size() < 10) return;
    
    // Find step transitions in desired
    std::vector<size_t> stepIndices;
    for (size_t i = 1; i < desired.size(); ++i) {
        if (std::abs(desired[i].position[0] - desired[i-1].position[0]) > 0.1) {
            stepIndices.push_back(i);
        }
    }
    
    if (stepIndices.empty()) return;
    
    // Analyze first step response
    size_t stepIdx = stepIndices[0];
    double targetPos = desired[stepIdx].position[0];
    double startPos = desired[stepIdx - 1].position[0];
    double stepSize = targetPos - startPos;
    
    // Find rise time (10% to 90%)
    double pos10 = startPos + 0.1 * stepSize;
    double pos90 = startPos + 0.9 * stepSize;
    
    double time10 = 0, time90 = 0;
    double maxPos = startPos;
    double timeMax = 0;
    
    for (size_t i = stepIdx; i < actual.size(); ++i) {
        double pos = actual[i].position[0];
        double t = actual[i].timestamp - actual[stepIdx].timestamp;
        
        if (time10 == 0 && pos >= pos10) time10 = t;
        if (time90 == 0 && pos >= pos90) time90 = t;
        
        if (pos > maxPos) {
            maxPos = pos;
            timeMax = t;
        }
        
        // Check for settling (within 2% of target)
        if (std::abs(pos - targetPos) < 0.02 * std::abs(stepSize)) {
            result.settlingTime = t;
            break;
        }
    }
    
    result.riseTime = time90 - time10;
    result.overshoot = (maxPos - targetPos) / std::abs(stepSize) * 100.0;
    result.steadyStateError = std::abs(actual.back().position[0] - targetPos);
}

void MachineTester::analyzeCircularity(TestResult& result,
                                       const std::vector<PositionSample>& actual,
                                       const MultiAxisTestConfig& config) {
    if (actual.size() < 10) return;
    
    double sumR = 0, sumR2 = 0;
    double minR = std::numeric_limits<double>::max();
    double maxR = std::numeric_limits<double>::lowest();
    
    for (const auto& sample : actual) {
        double du = sample.position[config.uAxis] - config.center[0];
        double dv = sample.position[config.vAxis] - config.center[1];
        double r = std::sqrt(du*du + dv*dv);
        
        sumR += r;
        sumR2 += r * r;
        minR = std::min(minR, r);
        maxR = std::max(maxR, r);
    }
    
    double meanR = sumR / actual.size();
    double variance = sumR2 / actual.size() - meanR * meanR;
    
    result.radiusError = meanR - config.radiusU;
    result.circularityError = maxR - minR;  // Total indicator reading
}

void MachineTester::analyzeFriction(TestResult& result,
                                    const std::vector<std::pair<double, std::vector<PositionSample>>>& data) {
    // Collect velocity vs required force (estimated from tracking error)
    std::vector<double> velocities;
    std::vector<double> forces;
    
    for (const auto& [vel, samples] : data) {
        velocities.push_back(std::abs(vel));
        
        // Estimate force from mean tracking error (simplified)
        double sumError = 0;
        for (const auto& s : samples) {
            sumError += std::abs(s.position[0]); // Simplified
        }
        forces.push_back(sumError / samples.size());
    }
    
    // Fit friction model
    auto model = FrictionModel::fitFromData(velocities, forces);
    
    result.staticFriction = model.staticFriction;
    result.viscousFriction = model.viscousFriction;
    result.coulombFriction = model.coulombFriction;
}

double MachineTester::findDelay(const std::vector<PositionSample>& desired,
                                const std::vector<PositionSample>& actual,
                                double maxDelay, double resolution) {
    double bestDelay = 0;
    double bestCorrelation = -1e9;
    
    for (double testDelay = 0; testDelay <= maxDelay; testDelay += resolution) {
        double correlation = 0;
        size_t count = 0;
        
        for (size_t i = 0; i < actual.size(); ++i) {
            double shiftedTime = actual[i].timestamp - testDelay;
            if (shiftedTime < 0 || shiftedTime > desired.back().timestamp) continue;
            
            // Find nearest desired sample
            size_t desiredIdx = static_cast<size_t>(shiftedTime / 0.001);
            if (desiredIdx >= desired.size()) continue;
            
            correlation += actual[i].velocity[0] * desired[desiredIdx].velocity[0];
            count++;
        }
        
        if (count > 0) {
            correlation /= count;
            if (correlation > bestCorrelation) {
                bestCorrelation = correlation;
                bestDelay = testDelay;
            }
        }
    }
    
    return bestDelay;
}

//=============================================================================
// FrictionModel Implementation
//=============================================================================

FrictionModel FrictionModel::fitFromData(const std::vector<double>& velocities,
                                          const std::vector<double>& forces) {
    FrictionModel model;
    
    if (velocities.empty() || forces.empty()) return model;
    
    // Find static friction (force at near-zero velocity)
    double minVel = *std::min_element(velocities.begin(), velocities.end());
    for (size_t i = 0; i < velocities.size(); ++i) {
        if (velocities[i] == minVel) {
            model.staticFriction = forces[i];
            break;
        }
    }
    
    // Fit viscous friction using linear regression
    // F = Fc + Fv * v
    double sumV = 0, sumF = 0, sumVV = 0, sumVF = 0;
    size_t n = velocities.size();
    
    for (size_t i = 0; i < n; ++i) {
        sumV += velocities[i];
        sumF += forces[i];
        sumVV += velocities[i] * velocities[i];
        sumVF += velocities[i] * forces[i];
    }
    
    double denom = n * sumVV - sumV * sumV;
    if (std::abs(denom) > 1e-9) {
        model.viscousFriction = (n * sumVF - sumV * sumF) / denom;
        model.coulombFriction = (sumF - model.viscousFriction * sumV) / n;
    }
    
    // Stribeck parameters (simplified)
    model.stribeckVelocity = 50.0;  // mm/min, typical value
    
    return model;
}

} // namespace MotionReplanner

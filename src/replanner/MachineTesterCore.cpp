/**
 * @file MachineTesterCore.cpp
 * @brief Core implementation of machine performance testing framework
 * 
 * Contains MachineTester class core methods:
 * - Test execution (single axis, multi axis, friction, delay, PID)
 * - Test sequences
 * - Standard test sequence creation
 * 
 * Split from MachineTester.cpp
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

} // namespace MotionReplanner

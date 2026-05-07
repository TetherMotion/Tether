/**
 * @file machine_tester_01_basic.cpp
 * @brief Example 1: Basic single-axis sinusoidal test
 * 
 * This is the simplest example demonstrating:
 * - Single axis (X) sinusoidal motion
 * - Basic configuration
 * - Simple data export
 */

#include <iostream>
#include "MachineTester.hpp"
#include "TestDataExporter.hpp"

using namespace MotionReplanner;

int main() {
    std::cout << "=== Machine Tester Example 1: Basic Single-Axis Test ===" << std::endl;
    
    // Create machine tester
    MachineTester tester;
    
    // Configure single-axis sinusoidal test
    SingleAxisTestConfig config;
    config.axis = 0;                    // X axis
    config.type = SingleAxisTestType::Sinusoid;
    config.amplitude = 50.0;            // mm
    config.frequency = 1.0;             // Hz
    config.cycles = 5;                  // 5 complete cycles
    
    std::cout << "Running single-axis sinusoidal test..." << std::endl;
    std::cout << "  Axis: X" << std::endl;
    std::cout << "  Amplitude: " << config.amplitude << " mm" << std::endl;
    std::cout << "  Frequency: " << config.frequency << " Hz" << std::endl;
    std::cout << "  Cycles: " << config.cycles << std::endl;
    
    // Run test
    TestResult result = tester.runSingleAxisTest(config);
    
    // Display results
    std::cout << "\nTest Results:" << std::endl;
    std::cout << "  Status: " << (result.passed ? "PASSED" : "FAILED") << std::endl;
    std::cout << "  Max Velocity: " << result.maxVelocityAchieved << " mm/min" << std::endl;
    std::cout << "  Max Acceleration: " << result.maxAccelerationAchieved << " mm/s²" << std::endl;
    std::cout << "  Position Error (RMS): " << result.positionError.rms << " mm" << std::endl;
    std::cout << "  Position Error (Max): " << result.positionError.max << " mm" << std::endl;
    
    // Export data
    TestDataExporter exporter;
    exporter.exportToCSV("machine_test_01_data.csv", {result});
    exporter.exportSummary("machine_test_01_summary.txt", {result});
    
    std::cout << "\nData exported to:" << std::endl;
    std::cout << "  - machine_test_01_data.csv" << std::endl;
    std::cout << "  - machine_test_01_summary.txt" << std::endl;
    
    return result.passed ? 0 : 1;
}

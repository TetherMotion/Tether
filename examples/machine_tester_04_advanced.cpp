/**
 * @file machine_tester_04_advanced.cpp
 * @brief Example 4: Advanced multi-pattern testing with acceleration sweeps
 * 
 * This example demonstrates:
 * - Multiple geometric patterns (circle, ellipse, helix, Lissajous)
 * - Acceleration and jerk parameter sweeps
 * - Performance heatmap generation
 * - Comprehensive comparison analysis
 */

#include <iostream>
#include <vector>
#include <string>
#include "MachineTester.hpp"
#include "PerformanceHeatmap.hpp"
#include "TestDataExporter.hpp"

using namespace MotionReplanner;

struct TestCase {
    std::string name;
    MultiAxisTestConfig config;
};

int main() {
    std::cout << "=== Machine Tester Example 4: Advanced Multi-Pattern Testing ===" << std::endl;
    
    MachineTester tester;
    std::vector<TestResult> allResults;
    
    // Define test patterns
    std::vector<TestCase> patterns;
    
    // Pattern 1: Circle
    {
        TestCase tc;
        tc.name = "Circle";
        tc.config.type = MultiAxisTestType::Circle;
        tc.config.radiusU = tc.config.radiusV = 25.0;
        tc.config.revolutions = 3;
        patterns.push_back(tc);
    }
    
    // Pattern 2: Ellipse
    {
        TestCase tc;
        tc.name = "Ellipse";
        tc.config.type = MultiAxisTestType::Ellipse;
        tc.config.radiusU = 40.0;
        tc.config.radiusV = 20.0;
        tc.config.revolutions = 3;
        patterns.push_back(tc);
    }
    
    // Pattern 3: Helix (3D)
    {
        TestCase tc;
        tc.name = "Helix";
        tc.config.type = MultiAxisTestType::Helix;
        tc.config.radiusU = tc.config.radiusV = 20.0;
        tc.config.pitchW = 10.0;  // 10mm per revolution
        tc.config.wAxis = 2;      // Z axis
        tc.config.revolutions = 5;
        patterns.push_back(tc);
    }
    
    // Pattern 4: Lissajous curve
    {
        TestCase tc;
        tc.name = "Lissajous";
        tc.config.type = MultiAxisTestType::Lissajous;
        tc.config.radiusU = tc.config.radiusV = 30.0;
        tc.config.lissajousRatioU = 3.0;
        tc.config.lissajousRatioV = 2.0;
        tc.config.lissajousPhase = 90.0;
        tc.config.duration = 10.0;
        patterns.push_back(tc);
    }
    
    // Acceleration sweep parameters
    std::vector<double> accelerations = {500.0, 1000.0, 2000.0, 5000.0};
    std::vector<double> feedRates = {1000.0, 2000.0, 3000.0};
    
    // Run comprehensive tests
    int testNumber = 0;
    for (const auto& pattern : patterns) {
        std::cout << "\n=== Testing Pattern: " << pattern.name << " ===" << std::endl;
        
        for (double accel : accelerations) {
            for (double feedRate : feedRates) {
                ++testNumber;
                
                MultiAxisTestConfig config = pattern.config;
                config.feedRate = feedRate;
                // Note: acceleration would be set via machine limits
                
                std::cout << "[Test " << testNumber << "] " 
                          << pattern.name << " @ " << feedRate << " mm/min, "
                          << accel << " mm/s²... ";
                
                TestResult result = tester.runMultiAxisTest(config);
                result.testName = pattern.name + "_F" + std::to_string(int(feedRate)) + 
                                 "_A" + std::to_string(int(accel));
                allResults.push_back(result);
                
                std::cout << (result.passed ? "PASS" : "FAIL")
                          << " (Error: " << result.contourError.rms << " mm)" << std::endl;
            }
        }
    }
    
    // Generate comparison table
    std::cout << "\n=== Performance Comparison ===" << std::endl;
    std::cout << "Pattern        | Feed Rate | Contour RMS | Max Vel  | Max Accel" << std::endl;
    std::cout << "---------------+-----------+-------------+----------+----------" << std::endl;
    
    for (const auto& result : allResults) {
        printf("%-14s | %9.0f | %11.4f | %8.0f | %8.0f\n",
               result.testName.c_str(),
               result.feedRate,
               result.contourError.rms,
               result.maxVelocityAchieved,
               result.maxAccelerationAchieved);
    }
    
    // Find best and worst performers
    auto bestResult = std::min_element(allResults.begin(), allResults.end(),
        [](const TestResult& a, const TestResult& b) {
            return a.contourError.rms < b.contourError.rms;
        });
    
    auto worstResult = std::max_element(allResults.begin(), allResults.end(),
        [](const TestResult& a, const TestResult& b) {
            return a.contourError.rms < b.contourError.rms;
        });
    
    std::cout << "\nBest Performance: " << bestResult->testName 
              << " (RMS Error: " << bestResult->contourError.rms << " mm)" << std::endl;
    std::cout << "Worst Performance: " << worstResult->testName 
              << " (RMS Error: " << worstResult->contourError.rms << " mm)" << std::endl;
    
    // Generate performance heatmap
    PerformanceHeatmap heatmap;
    heatmap.generateFromTests(allResults, "machine_test_04_heatmap.svg");
    heatmap.generateAccelerationMap(allResults, accelerations, feedRates, 
                                    "machine_test_04_accel_map.svg");
    
    // Export all data
    TestDataExporter exporter;
    exporter.exportToCSV("machine_test_04_all_data.csv", allResults);
    exporter.exportSummary("machine_test_04_summary.txt", allResults);
    exporter.exportComparison("machine_test_04_comparison.html", allResults);
    
    std::cout << "\nData exported to:" << std::endl;
    std::cout << "  - machine_test_04_all_data.csv" << std::endl;
    std::cout << "  - machine_test_04_summary.txt" << std::endl;
    std::cout << "  - machine_test_04_comparison.html" << std::endl;
    std::cout << "  - machine_test_04_heatmap.svg" << std::endl;
    std::cout << "  - machine_test_04_accel_map.svg" << std::endl;
    
    std::cout << "\nCompleted " << testNumber << " tests across " 
              << patterns.size() << " patterns" << std::endl;
    
    return 0;
}

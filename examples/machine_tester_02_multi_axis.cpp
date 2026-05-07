/**
 * @file machine_tester_02_multi_axis.cpp
 * @brief Example 2: Multi-axis circular motion test
 * 
 * This example demonstrates:
 * - Two-axis circular motion (XY plane)
 * - Feed rate variation
 * - Circularity error analysis
 */

#include <iostream>
#include <vector>
#include "MachineTester.hpp"
#include "TestDataExporter.hpp"
#include "PerformanceHeatmap.hpp"

using namespace MotionReplanner;

int main() {
    std::cout << "=== Machine Tester Example 2: Multi-Axis Circular Motion ===" << std::endl;
    
    MachineTester tester;
    
    // Configure circular motion test
    MultiAxisTestConfig config;
    config.type = MultiAxisTestType::Circle;
    config.uAxis = 0;                   // X axis
    config.vAxis = 1;                   // Y axis
    config.radiusU = 30.0;              // 30mm radius
    config.radiusV = 30.0;
    config.center = {0.0, 0.0, 0.0};
    config.revolutions = 3;             // 3 complete circles
    
    // Test multiple feed rates
    std::vector<double> feedRates = {500.0, 1000.0, 2000.0, 3000.0, 5000.0};
    std::vector<TestResult> results;
    
    for (double feedRate : feedRates) {
        config.feedRate = feedRate;
        
        std::cout << "\nRunning circular test at " << feedRate << " mm/min..." << std::endl;
        
        TestResult result = tester.runMultiAxisTest(config);
        results.push_back(result);
        
        std::cout << "  Circularity Error: " << result.circularityError << " mm" << std::endl;
        std::cout << "  Radius Error: " << result.radiusError << " mm" << std::endl;
        std::cout << "  Max Velocity: " << result.maxVelocityAchieved << " mm/min" << std::endl;
        std::cout << "  Contour Error (RMS): " << result.contourError.rms << " mm" << std::endl;
    }
    
    // Analyze performance vs feed rate
    std::cout << "\n=== Performance Summary ===" << std::endl;
    std::cout << "Feed Rate (mm/min) | Circularity Error (mm) | Contour RMS (mm)" << std::endl;
    std::cout << "-------------------+------------------------+-----------------" << std::endl;
    
    for (size_t i = 0; i < results.size(); ++i) {
        printf("%18.0f | %22.4f | %15.4f\n", 
               feedRates[i], 
               results[i].circularityError,
               results[i].contourError.rms);
    }
    
    // Export data
    TestDataExporter exporter;
    exporter.exportToCSV("machine_test_02_data.csv", results);
    exporter.exportSummary("machine_test_02_summary.txt", results);
    
    // Create performance heatmap
    PerformanceHeatmap heatmap;
    heatmap.generateFromTests(results, "machine_test_02_heatmap.svg");
    
    std::cout << "\nData exported to:" << std::endl;
    std::cout << "  - machine_test_02_data.csv" << std::endl;
    std::cout << "  - machine_test_02_summary.txt" << std::endl;
    std::cout << "  - machine_test_02_heatmap.svg" << std::endl;
    
    return 0;
}

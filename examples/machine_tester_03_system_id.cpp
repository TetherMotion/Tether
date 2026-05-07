/**
 * @file machine_tester_03_system_id.cpp
 * @brief Example 3: System identification and characterization
 * 
 * This example demonstrates:
 * - Friction identification (multiple velocities)
 * - Delay identification (step response)
 * - PID tuning analysis (frequency response)
 */

#include <iostream>
#include <vector>
#include "MachineTester.hpp"
#include "SystemIdentifier.hpp"
#include "TestDataExporter.hpp"

using namespace MotionReplanner;

int main() {
    std::cout << "=== Machine Tester Example 3: System Identification ===" << std::endl;
    
    MachineTester tester;
    SystemIdentifier sysId;
    
    // === Part 1: Friction Identification ===
    std::cout << "\n[1/3] Friction Identification..." << std::endl;
    
    FrictionTestConfig frictionConfig;
    frictionConfig.axis = 0;  // X axis
    frictionConfig.velocities = {10, 50, 100, 200, 500, 1000, 2000};  // mm/min
    frictionConfig.distance = 100.0;  // mm
    frictionConfig.repeats = 3;
    frictionConfig.bidirectional = true;
    
    TestResult frictionResult = tester.runFrictionTest(frictionConfig);
    
    std::cout << "Friction Model Parameters:" << std::endl;
    std::cout << "  Static Friction: " << frictionResult.staticFriction << " N" << std::endl;
    std::cout << "  Viscous Friction: " << frictionResult.viscousFriction << " N/(mm/s)" << std::endl;
    std::cout << "  Coulomb Friction: " << frictionResult.coulombFriction << " N" << std::endl;
    
    // === Part 2: Delay Identification ===
    std::cout << "\n[2/3] Delay Identification..." << std::endl;
    
    DelayTestConfig delayConfig;
    delayConfig.axis = 0;
    delayConfig.stepAmplitude = 10.0;  // mm
    delayConfig.stepVelocity = 5000.0;  // mm/min (fast for clear response)
    delayConfig.stepCount = 10;
    delayConfig.maxExpectedDelay = 0.01;  // 10ms max
    
    TestResult delayResult = tester.runDelayTest(delayConfig);
    
    std::cout << "System Delay: " << (delayResult.detectedDelay * 1000.0) << " ms" << std::endl;
    
    // === Part 3: PID Tuning (Frequency Response) ===
    std::cout << "\n[3/3] PID Analysis (Frequency Response)..." << std::endl;
    
    PIDTestConfig pidConfig;
    pidConfig.axis = 0;
    pidConfig.chirpStartFreq = 0.5;   // Hz
    pidConfig.chirpEndFreq = 50.0;    // Hz
    pidConfig.chirpAmplitude = 2.0;   // mm
    pidConfig.chirpDuration = 20.0;   // seconds
    
    TestResult pidResult = tester.runPIDTest(pidConfig);
    
    std::cout << "Step Response Metrics:" << std::endl;
    std::cout << "  Rise Time: " << (pidResult.riseTime * 1000.0) << " ms" << std::endl;
    std::cout << "  Settling Time: " << (pidResult.settlingTime * 1000.0) << " ms" << std::endl;
    std::cout << "  Overshoot: " << pidResult.overshoot << " %" << std::endl;
    std::cout << "  Steady-State Error: " << pidResult.steadyStateError << " mm" << std::endl;
    
    // === System Identification Report ===
    SystemIdentifier::Report report = sysId.generateReport({frictionResult, delayResult, pidResult});
    
    std::cout << "\n=== System Identification Report ===" << std::endl;
    std::cout << report.summary << std::endl;
    
    // Export all results
    TestDataExporter exporter;
    exporter.exportToCSV("machine_test_03_friction.csv", {frictionResult});
    exporter.exportToCSV("machine_test_03_delay.csv", {delayResult});
    exporter.exportToCSV("machine_test_03_pid.csv", {pidResult});
    exporter.exportSummary("machine_test_03_report.txt", {frictionResult, delayResult, pidResult});
    
    // Export system ID report
    sysId.exportReport(report, "machine_test_03_system_id.json");
    
    std::cout << "\nData exported to:" << std::endl;
    std::cout << "  - machine_test_03_friction.csv" << std::endl;
    std::cout << "  - machine_test_03_delay.csv" << std::endl;
    std::cout << "  - machine_test_03_pid.csv" << std::endl;
    std::cout << "  - machine_test_03_report.txt" << std::endl;
    std::cout << "  - machine_test_03_system_id.json" << std::endl;
    
    return 0;
}

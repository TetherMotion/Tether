/**
 * @file trajectory_analysis.cpp
 * @brief Example: Trajectory analysis with error statistics
 * 
 * Demonstrates how to use the MotionReplanner to analyze trajectory
 * execution with commanded vs actual positions, including corner detection.
 */

#include "MotionReplanner.hpp"
#include <iostream>
#include <iomanip>
#include <cmath>

using namespace MotionReplanner;

// Generate synthetic commanded trajectory (square path)
std::vector<TrajectorySample> generateCommandedSquare(double sideLength, double velocity, double dt) {
    std::vector<TrajectorySample> trajectory;
    double time = 0.0;
    
    // Side duration
    double sideDuration = sideLength / velocity;
    int samplesPerSide = static_cast<int>(sideDuration / dt);
    
    // Start at origin
    double x = 0.0, y = 0.0, z = 0.0;
    
    // Side 1: +X
    for (int i = 0; i < samplesPerSide; ++i) {
        TrajectorySample sample;
        sample.timestamp = time;
        sample.position = {x, y, z};
        sample.velocity = {velocity, 0.0, 0.0};
        sample.acceleration = {0.0, 0.0, 0.0};
        sample.segmentIndex = 0;
        trajectory.push_back(sample);
        
        x += velocity * dt;
        time += dt;
    }
    
    // Side 2: +Y
    x = sideLength;
    for (int i = 0; i < samplesPerSide; ++i) {
        TrajectorySample sample;
        sample.timestamp = time;
        sample.position = {x, y, z};
        sample.velocity = {0.0, velocity, 0.0};
        sample.acceleration = {0.0, 0.0, 0.0};
        sample.segmentIndex = 1;
        trajectory.push_back(sample);
        
        y += velocity * dt;
        time += dt;
    }
    
    // Side 3: -X
    y = sideLength;
    for (int i = 0; i < samplesPerSide; ++i) {
        TrajectorySample sample;
        sample.timestamp = time;
        sample.position = {x, y, z};
        sample.velocity = {-velocity, 0.0, 0.0};
        sample.acceleration = {0.0, 0.0, 0.0};
        sample.segmentIndex = 2;
        trajectory.push_back(sample);
        
        x -= velocity * dt;
        time += dt;
    }
    
    // Side 4: -Y
    x = 0.0;
    for (int i = 0; i < samplesPerSide; ++i) {
        TrajectorySample sample;
        sample.timestamp = time;
        sample.position = {x, y, z};
        sample.velocity = {0.0, -velocity, 0.0};
        sample.acceleration = {0.0, 0.0, 0.0};
        sample.segmentIndex = 3;
        trajectory.push_back(sample);
        
        y -= velocity * dt;
        time += dt;
    }
    
    return trajectory;
}

// Simulate actual trajectory with various imperfections
std::vector<TrajectorySample> simulateActualTrajectory(
    const std::vector<TrajectorySample>& commanded,
    double systemDelay,
    double cornerOvershoot,
    double noiseAmplitude,
    double dt
) {
    std::vector<TrajectorySample> actual;
    
    int delaySamples = static_cast<int>(systemDelay / dt);
    
    for (size_t i = 0; i < commanded.size(); ++i) {
        TrajectorySample sample = commanded[i];
        
        // Apply delay (use earlier commanded position)
        if (i >= static_cast<size_t>(delaySamples)) {
            sample.position = commanded[i - delaySamples].position;
            sample.velocity = commanded[i - delaySamples].velocity;
        }
        
        // Detect corners (velocity direction change)
        bool atCorner = false;
        if (i > 0 && i < commanded.size() - 1) {
            const auto& prevVel = commanded[i-1].velocity;
            const auto& currVel = commanded[i].velocity;
            
            // Check for significant velocity direction change
            double dot = 0.0;
            double mag1 = 0.0, mag2 = 0.0;
            for (size_t j = 0; j < 3; ++j) {
                dot += prevVel[j] * currVel[j];
                mag1 += prevVel[j] * prevVel[j];
                mag2 += currVel[j] * currVel[j];
            }
            
            if (mag1 > 1e-6 && mag2 > 1e-6) {
                double cosAngle = dot / (std::sqrt(mag1) * std::sqrt(mag2));
                if (cosAngle < 0.5) {  // > 60 degree change
                    atCorner = true;
                }
            }
        }
        
        // Add corner overshoot
        if (atCorner) {
            for (size_t j = 0; j < 3; ++j) {
                sample.position[j] += cornerOvershoot * commanded[i].velocity[j] / 10.0;
            }
        }
        
        // Add noise
        for (size_t j = 0; j < 3; ++j) {
            sample.position[j] += noiseAmplitude * (static_cast<double>(rand()) / RAND_MAX - 0.5);
        }
        
        actual.push_back(sample);
    }
    
    return actual;
}

void printErrorStatistics(const ErrorStatistics& stats, const std::string& label) {
    std::cout << "\n" << label << ":\n";
    std::cout << std::fixed << std::setprecision(6);
    std::cout << "  Min:       " << stats.min << " mm\n";
    std::cout << "  Max:       " << stats.max << " mm\n";
    std::cout << "  Mean:      " << stats.mean << " mm\n";
    std::cout << "  Geo Mean:  " << stats.geometricMean << " mm\n";
    std::cout << "  Std Dev:   " << stats.stdDev << " mm\n";
    std::cout << "  RMS:       " << stats.rms << " mm\n";
    std::cout << "  P95:       " << stats.percentile95 << " mm\n";
    std::cout << "  P99:       " << stats.percentile99 << " mm\n";
    std::cout << "  Samples:   " << stats.sampleCount << "\n";
}

int main() {
    std::cout << "=== Motion Replanner: Trajectory Analysis Example ===\n\n";
    
    // Configure replanner
    ReplannerConfig config;
    config.systemDelay = 0.001;  // 1ms delay
    config.samplePeriod = 0.001; // 1kHz sample rate
    config.cornerVelocityThreshold = 0.1;
    config.cornerAccelThreshold = 100.0;
    config.errorThreshold = 0.1;
    config.mode = OperationMode::Monitor;
    
    MotionReplanner replanner(config);
    
    // Generate test trajectories
    double sideLength = 100.0;  // 100mm square
    double velocity = 50.0;     // 50mm/s
    double dt = 0.001;          // 1ms sample period
    
    std::cout << "Generating " << sideLength << "mm square trajectory at " 
              << velocity << "mm/s\n";
    
    auto commanded = generateCommandedSquare(sideLength, velocity, dt);
    
    // Simulate different imperfection scenarios
    struct Scenario {
        std::string name;
        double delay;
        double overshoot;
        double noise;
    };
    
    std::vector<Scenario> scenarios = {
        {"Ideal (minimal delay, no overshoot)", 0.001, 0.0, 0.001},
        {"Slight delay (2ms)", 0.002, 0.0, 0.001},
        {"Corner overshoot", 0.001, 0.5, 0.001},
        {"High noise", 0.001, 0.0, 0.05},
        {"Combined imperfections", 0.003, 0.3, 0.02},
    };
    
    for (const auto& scenario : scenarios) {
        std::cout << "\n--- Scenario: " << scenario.name << " ---\n";
        
        auto actual = simulateActualTrajectory(
            commanded, scenario.delay, scenario.overshoot, scenario.noise, dt
        );
        
        // Reset replanner
        replanner.reset();
        
        // Feed samples
        for (size_t i = 0; i < commanded.size(); ++i) {
            replanner.processSample(commanded[i], actual[i]);
        }
        
        // Get statistics
        auto globalStats = replanner.getGlobalStatistics();
        auto cornerStats = replanner.getCornerStatistics();
        
        printErrorStatistics(globalStats, "Global Error Statistics");
        printErrorStatistics(cornerStats, "Corner Error Statistics");
        
        // Check for limit violations
        auto violations = replanner.getLimitViolations();
        if (!violations.empty()) {
            std::cout << "\n  Limit violations detected: " << violations.size() << "\n";
        }
    }
    
    std::cout << "\n=== Analysis Complete ===\n";
    
    return 0;
}

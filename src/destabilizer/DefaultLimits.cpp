/// @file DefaultLimits.cpp
/// @brief Per-system default perturbation limits based on real-world physics.

#include "tether/destabilizer/DefaultLimits.hpp"
#include <unordered_map>

namespace Destabilizer {

/// Helper: create a single-channel default.
static PerturbationChannel makeChannel(int inputIdx, const char* name,
                                         double aMax, double rateMax,
                                         double energyMax, const char* rationale) {
    PerturbationChannel ch;
    ch.inputIndex = inputIdx;
    ch.name = name;
    ch.constraints.amplitudeMax = aMax;
    ch.constraints.rateMax = rateMax;
    ch.constraints.energyMax = energyMax;
    ch.tooltipRationale = rationale;
    return ch;
}

std::vector<PerturbationChannel> getDefaultChannels(int systemId) {
    switch (systemId) {
    // --- Mechanical (1-19) ---
    case 1: // Mass-Spring-Damper (1 kg)
        return { makeChannel(0, "External Force", 2.0, 100.0, 50.0,
            "~0.2g acceleration on 1 kg mass") };
    case 2: // Coupled Mass-Spring-Damper
        return { makeChannel(0, "External Force", 2.0, 100.0, 50.0,
            "~0.2g per body") };
    case 3: // Inverted Pendulum on Cart (1 kg cart, 0.3 m pole)
        return { makeChannel(0, "Horizontal Force on Cart", 5.0, 200.0, 100.0,
            "Roughly a gentle hand push; cart weight × 0.5g") };
    case 4: // Double Inverted Pendulum on Cart
        return { makeChannel(0, "Horizontal Force on Cart", 3.0, 150.0, 50.0,
            "Reduced because system is more fragile") };
    case 5: // Triple Inverted Pendulum on Cart
        return { makeChannel(0, "Horizontal Force on Cart", 3.0, 150.0, 50.0,
            "Reduced because system is more fragile") };
    case 6: // Pendubot
        return { makeChannel(0, "Torque on Joint 1", 0.5, 50.0, 20.0,
            "10% of typical actuator max torque") };
    case 7: // Acrobot
        return { makeChannel(0, "Torque on Joint 2", 0.5, 50.0, 20.0,
            "10% of typical actuator max torque") };
    case 8: // Furuta Pendulum
        return { makeChannel(0, "Torque on Pivot", 0.2, 50.0, 10.0,
            "10% of actuator max torque") };
    case 9: // Ball on Beam
        return { makeChannel(0, "Lateral Impulse on Ball", 0.1, 20.0, 5.0,
            "Gentle flick on small ball") };
    case 10: // Ball on Plate
        return {
            makeChannel(0, "Lateral Force X", 0.1, 20.0, 5.0, "Gentle flick on ball, X axis"),
            makeChannel(1, "Lateral Force Y", 0.1, 20.0, 5.0, "Gentle flick on ball, Y axis")
        };
    case 11: // Bouncing Ball
        return { makeChannel(0, "Vertical Force", 0.5, 50.0, 20.0,
            "Small vertical disturbance") };
    case 12: // Segway Robot
        return { makeChannel(0, "Horizontal Push on Body", 10.0, 200.0, 200.0,
            "Typical shove by a person") };
    case 13: // Gantry Crane (500 kg payload)
        return { makeChannel(0, "Horizontal Wind on Payload", 50.0, 500.0, 5000.0,
            "Light breeze on crate-sized load") };
    case 14: // Double Pendulum Gantry Crane
        return { makeChannel(0, "Horizontal Wind on Payload", 50.0, 500.0, 5000.0,
            "Light breeze on crate-sized load") };
    case 15: // Magnetic Levitation
        return { makeChannel(0, "Vertical Force on Ball", 0.05, 10.0, 5.0,
            "10% of ball weight for typical maglev setup") };
    case 16: // Dual Magnetic Levitation
        return { makeChannel(0, "Vertical Force on Ball 1", 0.05, 10.0, 5.0,
            "10% of ball weight") };
    case 17: // Quarter-Car Suspension
        return { makeChannel(0, "Road Profile Amplitude", 0.05, 5.0, 10.0,
            "Typical pothole depth (0.05 m)") };
    case 18: // Half-Car Suspension
        return { makeChannel(0, "Road Profile Amplitude", 0.05, 5.0, 10.0,
            "Typical pothole depth (0.05 m)") };
    case 19: // Vibration Isolation Platform
        return { makeChannel(0, "Base Acceleration", 4.9, 500.0, 200.0,
            "0.5g at 10-100 Hz, typical floor vibration") };

    // --- Rotational (20-27) ---
    case 20: // DC Motor Speed
        return { makeChannel(0, "Load Torque Disturbance", 0.2, 50.0, 20.0,
            "20% of rated torque, standard industry spec") };
    case 21: // DC Motor Position
        return { makeChannel(0, "Load Torque Disturbance", 0.2, 50.0, 20.0,
            "20% of rated torque, standard industry spec") };
    case 22: // Flexible Shaft / Two-Inertia
        return { makeChannel(0, "Load Torque Step", 0.25, 50.0, 30.0,
            "25% of rated torque") };
    case 23: // Disk Drive Head
        return { makeChannel(0, "External Shock", 49.0, 5000.0, 500.0,
            "5g, 2 ms impulse — laptop bump spec") };
    case 24: // Reaction Wheel (Single Axis)
        return { makeChannel(0, "External Torque", 0.001, 0.1, 0.01,
            "Solar pressure / gravity gradient scale") };
    case 25: // Reaction Wheel 2D
        return {
            makeChannel(0, "External Torque X", 0.001, 0.1, 0.01,
                "Solar pressure scale, axis X"),
            makeChannel(1, "External Torque Y", 0.001, 0.1, 0.01,
                "Solar pressure scale, axis Y")
        };
    case 26: // CMG
        return { makeChannel(0, "External Torque", 0.001, 0.1, 0.01,
            "Solar pressure / gravity gradient scale") };
    case 27: // Flywheel Energy Storage
        return { makeChannel(0, "Load Torque Disturbance", 0.2, 50.0, 20.0,
            "20% of rated torque") };

    // --- Aerospace (28-32) ---
    case 28: // Planar Quadrotor (1 kg)
        return { makeChannel(0, "Wind Force (Horizontal)", 3.0, 100.0, 100.0,
            "~5 m/s gust on small frame") };
    case 29: // Rocket Landing 2D
        return { makeChannel(0, "Wind Force", 5.0, 200.0, 200.0,
            "Moderate wind gust during landing") };
    case 30: // Fixed-Wing Aircraft 2D
        return { makeChannel(0, "Wind Gust", 5.0, 200.0, 200.0,
            "Moderate wind gust") };
    case 31: // Bicycle Lean
        return { makeChannel(0, "Lateral Wind Force", 5.0, 100.0, 100.0,
            "Moderate crosswind on cyclist") };
    case 32: // Hovercraft 2D
        return { makeChannel(0, "Wind Force", 3.0, 100.0, 100.0,
            "~5 m/s gust") };

    // --- Thermal (33-37) ---
    case 33: // Single-Zone Oven
        return { makeChannel(0, "Ambient Temperature Disturbance", 5.0, 10.0, 500.0,
            "±5 K typical upstream variation") };
    case 34: // Multi-Zone Oven
        return { makeChannel(0, "Ambient Temperature Disturbance", 5.0, 10.0, 500.0,
            "±5 K typical upstream variation") };
    case 35: // Heat Exchanger
        return { makeChannel(0, "Flow Rate Disturbance", 0.1, 1.0, 10.0,
            "±10% of nominal flow") };
    case 36: // Thermoelectric Cooler
        return { makeChannel(0, "Heat Load Disturbance", 1.0, 10.0, 50.0,
            "±10% of rated cooling capacity") };
    case 37: // Room HVAC
        return { makeChannel(0, "Occupancy Heat Load", 100.0, 200.0, 50000.0,
            "1-2 persons entering room") };

    // --- Fluid (38-43) ---
    case 38: // Single Tank Level
        return { makeChannel(0, "Inlet Flow Disturbance", 0.15, 1.0, 10.0,
            "±15% of nominal flow, typical pump variation") };
    case 39: // Coupled Two-Tank
        return { makeChannel(0, "Inlet Flow Disturbance", 0.15, 1.0, 10.0,
            "±15% of nominal flow") };
    case 40: // Four-Tank System
        return { makeChannel(0, "Inlet Flow Disturbance", 0.15, 1.0, 10.0,
            "±15% of nominal flow") };
    case 41: // Hydraulic Actuator
        return { makeChannel(0, "Load Force Disturbance", 1000.0, 10000.0, 100000.0,
            "10% of max thrust") };
    case 42: // Pneumatic Muscle
        return { makeChannel(0, "Supply Pressure Disturbance", 50000.0, 500000.0, 5e9,
            "±0.5 bar, typical compressor variation") };
    case 43: // Pressure Vessel
        return { makeChannel(0, "Flow Rate Disturbance", 0.1, 1.0, 10.0,
            "±10% of nominal flow") };

    // --- Electrical (44-49) ---
    case 44: // Buck Converter
        return {
            makeChannel(0, "Input Voltage Ripple", 2.4, 100.0, 100.0,
                "±10% of 24V input, typical supply ripple"),
        };
    case 45: // Boost Converter
        return { makeChannel(0, "Input Voltage Ripple", 1.2, 100.0, 50.0,
            "±10% of 12V input") };
    case 46: // Buck-Boost Converter
        return {
            makeChannel(0, "Input Voltage Ripple", 2.4, 100.0, 100.0,
                "±10% of V_in, typical supply ripple"),
        };
    case 47: // Power Grid Frequency
        return { makeChannel(0, "Load Step", 5.0, 100.0, 500.0,
            "±5% of nominal load, typical grid event") };
    case 48: // Active Power Filter
        return { makeChannel(0, "Load Disturbance", 1.0, 50.0, 50.0,
            "10% of rated load") };
    case 49: // Phase-Lock Loop
        return { makeChannel(0, "Frequency Step", 1.0, 50.0, 50.0,
            "1 Hz step, typical reference perturbation") };

    // --- Chemical (50-53) ---
    case 50: // CSTR
        return {
            makeChannel(0, "Feed Temperature Deviation", 5.0, 10.0, 500.0,
                "±5 K typical upstream variation"),
        };
    case 51: // pH Neutralization
        return { makeChannel(0, "Inlet Flow Disturbance", 0.15, 1.0, 10.0,
            "±15% of nominal, typical supply pump variation") };
    case 52: // Distillation Column
        return { makeChannel(0, "Feed Flow Disturbance", 0.1, 1.0, 10.0,
            "±10% of nominal, standard disturbance") };
    case 53: // Bioreactor
        return { makeChannel(0, "Substrate Feed Rate Disturbance", 0.15, 1.0, 10.0,
            "±15%, typical pump variation") };

    // --- Biological (54-56) ---
    case 54: // Blood Glucose (Insulin Pump)
        return { makeChannel(0, "Meal Disturbance", 60.0, 10.0, 10000.0,
            "60 g carbs over 30 min, typical meal") };
    case 55: // Anesthesia Control
        return { makeChannel(0, "Surgical Stimulus", 1.0, 5.0, 50.0,
            "Scaled to typical incision response, clinical guideline") };
    case 56: // Predator-Prey
        return { makeChannel(0, "Population Perturbation", 0.1, 1.0, 10.0,
            "10% population disturbance") };

    // --- Chaotic (57-62) ---
    case 57: // Lorenz System
        return { makeChannel(0, "State Perturbation", 1.0, 50.0, 50.0,
            "Small perturbation near chaotic attractor") };
    case 58: // Rossler System
        return { makeChannel(0, "State Perturbation", 0.5, 25.0, 25.0,
            "Small perturbation") };
    case 59: // Chua Circuit
        return { makeChannel(0, "Voltage Perturbation", 0.1, 10.0, 5.0,
            "Small voltage disturbance") };
    case 60: // Duffing Oscillator
        return { makeChannel(0, "Force Perturbation", 0.5, 25.0, 25.0,
            "Moderate force perturbation") };
    case 61: // Kapitza Pendulum
        return { makeChannel(0, "Base Vibration Perturbation", 0.1, 50.0, 10.0,
            "Small base vibration change") };
    case 62: // Triple-Link Gymnast
        return { makeChannel(0, "Joint Torque Perturbation", 1.0, 50.0, 50.0,
            "Moderate torque disturbance") };

    // --- Delay (63-65) ---
    case 63: // Smith Predictor Plant
        return { makeChannel(0, "Delay Mismatch / Input Disturbance", 0.2, 10.0, 20.0,
            "±20% of nominal delay, typical modeling error") };
    case 64: // Networked Control System
        return { makeChannel(0, "Input Disturbance", 0.5, 25.0, 25.0,
            "Packet delay equivalent: 0-200 ms jitter") };
    case 65: // Conveyor Belt Tracking
        return { makeChannel(0, "Speed Perturbation", 0.1, 5.0, 10.0,
            "10% speed variation") };

    default:
        // Generic fallback
        return { makeChannel(0, "External Disturbance", 1.0, 100.0, 100.0,
            "Generic default: 1 unit amplitude") };
    }
}

std::string getDefaultRationale(int systemId) {
    auto channels = getDefaultChannels(systemId);
    std::string rationale;
    for (const auto& ch : channels) {
        if (!rationale.empty()) rationale += "\n";
        rationale += ch.name + ": " + ch.tooltipRationale;
    }
    return rationale;
}

ChannelConstraints getDefaultConstraints(int systemId, int channelIndex) {
    auto channels = getDefaultChannels(systemId);
    if (channelIndex >= 0 && channelIndex < static_cast<int>(channels.size())) {
        return channels[channelIndex].constraints;
    }
    // Fallback
    ChannelConstraints c;
    c.amplitudeMax = 1.0;
    return c;
}

} // namespace Destabilizer

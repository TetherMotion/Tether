#include "tether/simulation/SensorActuatorModels.hpp"
#include <cmath>
#include <algorithm>

namespace Simulation {

// ============================================================================
// NoiseGenerator
// ============================================================================
NoiseGenerator::NoiseGenerator() : rng_(42) {}
NoiseGenerator::NoiseGenerator(uint64_t seed) : rng_(seed) {}

double NoiseGenerator::generate(const NoiseParams& params, double time, double actuatorOutput) {
    if (params.type == NoiseType::None || params.amplitude == 0.0) return 0.0;

    std::normal_distribution<double> normal(0.0, 1.0);

    switch (params.type) {
        case NoiseType::White:
            return params.amplitude * normal(rng_);

        case NoiseType::Brown: {
            double white = normal(rng_);
            brownState_ += white * params.amplitude * 0.01;
            brownState_ *= 0.999;  // slow decay to prevent drift
            return brownState_;
        }

        case NoiseType::Purple: {
            double white = params.amplitude * normal(rng_);
            double purple = white - prevWhite_;
            prevWhite_ = white;
            return purple;
        }

        case NoiseType::Grey: {
            // A-weighted noise approximation
            double white = params.amplitude * normal(rng_);
            double grey = 0.7 * white + 0.3 * prevWhite_;
            prevWhite_ = white;
            return grey;
        }

        case NoiseType::PeriodicGSM: {
            // 217 Hz GSM pulse noise
            double freq = (params.frequency > 0) ? params.frequency : 217.0;
            double phase = std::fmod(time * freq, 1.0);
            double pulse = (phase < 0.125) ? 1.0 : 0.0;
            return params.amplitude * pulse * (0.5 + 0.5 * normal(rng_));
        }

        case NoiseType::ActuatorDependent:
            return params.amplitude * std::abs(actuatorOutput) * params.actuatorGain * normal(rng_);

        default:
            return 0.0;
    }
}

void NoiseGenerator::reset() {
    brownState_ = 0.0;
    purpleState_ = 0.0;
    prevWhite_ = 0.0;
}

void NoiseGenerator::setSeed(uint64_t seed) {
    rng_.seed(seed);
    reset();
}

// ============================================================================
// SensorModel
// ============================================================================
SensorModel::SensorModel() : delayRng_(123) {}
SensorModel::SensorModel(const SensorConfig& config)
    : config_(config), noise_(config.noise.seed), delayRng_(config.noise.seed + 1) {}

double SensorModel::measure(double trueValue, double time, double actuatorOutput) {
    double value = trueValue;

    // Add noise
    value += noise_.generate(config_.noise, time, actuatorOutput);

    // Apply quantization
    if (config_.quantization > 0.0) {
        value = std::round(value / config_.quantization) * config_.quantization;
    }

    // Apply saturation
    value = std::clamp(value, config_.saturationMin, config_.saturationMax);

    // Apply sample rate (ZOH)
    if (config_.sampleRate > 0.0) {
        double samplePeriod = 1.0 / config_.sampleRate;
        if (time - lastSampleTime_ >= samplePeriod) {
            lastSampledValue_ = value;
            lastSampleTime_ = time;
        }
        value = lastSampledValue_;
    }

    // Apply delay
    if (config_.delay > 0.0 || config_.delayVariance > 0.0) {
        double totalDelay = config_.delay;
        if (config_.delayVariance > 0.0) {
            std::normal_distribution<double> delayDist(0.0, config_.delayVariance);
            totalDelay += std::abs(delayDist(delayRng_));
        }

        delayBuffer_.push_back({time, value});

        // Find the value at (time - delay)
        double targetTime = time - totalDelay;
        value = 0.0;  // default if no past data
        for (auto it = delayBuffer_.rbegin(); it != delayBuffer_.rend(); ++it) {
            if (it->first <= targetTime) {
                value = it->second;
                break;
            }
        }

        // Prune old entries
        while (delayBuffer_.size() > 1000) {
            delayBuffer_.erase(delayBuffer_.begin());
        }
    }

    return value;
}

void SensorModel::reset() {
    noise_.reset();
    delayBuffer_.clear();
    lastSampleTime_ = -1e9;
    lastSampledValue_ = 0.0;
}

// ============================================================================
// ActuatorModel
// ============================================================================
ActuatorModel::ActuatorModel() {}
ActuatorModel::ActuatorModel(const ActuatorConfig& config) : config_(config) {}

double ActuatorModel::apply(double command, double dt) {
    if (failed_) return 0.0;

    double output = command;

    // Apply dead zone
    if (config_.deadZone > 0.0) {
        if (std::abs(output) < config_.deadZone) {
            output = 0.0;
        } else {
            output -= std::copysign(config_.deadZone, output);
        }
    }

    // Apply backlash (direction reversal dead-zone)
    if (config_.backlash > 0.0) {
        double diff = output - backlashState_;
        if (std::abs(diff) > config_.backlash) {
            backlashState_ = output - std::copysign(config_.backlash, diff);
        }
        output = backlashState_;
    }

    // Apply rate limiting
    if (config_.maxRate < 1e5 && dt > 0.0) {
        double rate = (output - prevOutput_) / dt;
        if (std::abs(rate) > config_.maxRate) {
            output = prevOutput_ + std::copysign(config_.maxRate * dt, rate);
        }
    }

    // Apply saturation
    double range = config_.maxOutput - config_.minOutput;
    if (config_.reducedPerfNearLimits && range > 0.0) {
        double relPos = (output - config_.minOutput) / range;
        double zone = config_.perfReductionZone;
        if (relPos < zone) {
            output *= relPos / zone;
        } else if (relPos > 1.0 - zone) {
            output *= (1.0 - relPos) / zone;
        }
    }

    output = std::clamp(output, config_.minOutput, config_.maxOutput);

    // Check fail-on-limit
    if (config_.failOnLimit) {
        if (output >= config_.maxOutput || output <= config_.minOutput) {
            failed_ = true;
            return 0.0;
        }
    }

    // Apply hysteresis
    if (config_.hysteresis > 0.0) {
        double diff = output - hysteresisState_;
        if (std::abs(diff) > config_.hysteresis) {
            hysteresisState_ = output;
        }
        output = hysteresisState_;
    }

    // Apply stiction
    if (config_.stiction.model != FrictionModel::None) {
        double stictionForce = config_.stiction.compute(output - prevOutput_, 1.0, dt);
        output += stictionForce * dt;
    }

    prevOutput_ = output;
    prevCommand_ = command;
    return output;
}

void ActuatorModel::reset() {
    prevOutput_ = 0.0;
    prevCommand_ = 0.0;
    backlashState_ = 0.0;
    hysteresisState_ = 0.0;
    failed_ = false;
}

} // namespace Simulation

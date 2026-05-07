#include "tether/simulation/SimulationEngine.hpp"
#include <cmath>
#include <algorithm>
#include <numeric>
#include <complex>

namespace Simulation {

SimulationEngine::SimulationEngine() {}
SimulationEngine::~SimulationEngine() = default;

void SimulationEngine::setSystem(std::shared_ptr<DynamicalSystem> system) {
    system_ = std::move(system);
}

void SimulationEngine::setController(std::shared_ptr<SimController> controller) {
    controller_ = std::move(controller);
}

void SimulationEngine::setConfig(const SimConfig& config) {
    config_ = config;
    integrator_ = createIntegrator(config.method, config.absTolerance, config.relTolerance);
}

void SimulationEngine::setSensorConfigs(const std::vector<SensorConfig>& configs) {
    sensors_.clear();
    for (const auto& c : configs) {
        sensors_.emplace_back(c);
    }
}

void SimulationEngine::setActuatorConfigs(const std::vector<ActuatorConfig>& configs) {
    actuators_.clear();
    for (const auto& c : configs) {
        actuators_.emplace_back(c);
    }
}

void SimulationEngine::initialize() {
    if (!system_) return;

    if (!integrator_) {
        integrator_ = createIntegrator(config_.method, config_.absTolerance, config_.relTolerance);
    }

    state_ = hasInitialState_ ? initialState_ : system_->defaultInitialState();
    time_ = 0.0;
    initialized_ = true;

    // Initialize sensors
    if (sensors_.empty()) {
        auto defaults = system_->defaultSensorConfigs();
        for (const auto& sc : defaults) {
            sensors_.emplace_back(sc);
        }
    }

    // Initialize actuators
    if (actuators_.empty()) {
        auto defaults = system_->defaultActuatorConfigs();
        for (const auto& ac : defaults) {
            actuators_.emplace_back(ac);
        }
    }

    // Reset sensors and actuators
    for (auto& s : sensors_) s.reset();
    for (auto& a : actuators_) a.reset();

    if (reference_.empty()) {
        reference_ = StateVector(system_->outputDim(), 0.0);
    }
}

SimStepResult SimulationEngine::step() {
    if (!initialized_ || !system_) {
        return {0.0, {}, {}, 0.0, 0.0};
    }

    // Get true output
    StateVector u_command = input_.empty() ? system_->defaultInput() : input_;
    StateVector y_true = system_->output(time_, state_, u_command);

    // Apply sensor models to get measured output
    StateVector y_measured(y_true.size());
    for (size_t i = 0; i < y_true.size(); ++i) {
        if (i < sensors_.size()) {
            y_measured[i] = sensors_[i].measure(y_true[i], time_);
        } else {
            y_measured[i] = y_true[i];
        }
    }

    // Compute control input
    if (controller_) {
        u_command = controller_->compute(time_, y_measured, reference_, config_.dt);
    }

    // Apply actuator models
    for (size_t i = 0; i < u_command.size(); ++i) {
        if (i < actuators_.size()) {
            u_command[i] = actuators_[i].apply(u_command[i], config_.dt);
        }
    }

    // Add external forces
    StateVector u_total = u_command;
    if (externalForce_) {
        StateVector f_ext = externalForce_(time_, state_);
        for (size_t i = 0; i < std::min(u_total.size(), f_ext.size()); ++i) {
            u_total[i] += f_ext[i];
        }
    }

    // Create ODE function capturing the input
    auto odefun = [this, &u_total](double t, const StateVector& s) -> StateVector {
        return system_->dynamics(t, s, u_total);
    };

    // Integrate
    double dt = config_.dt;
    if (config_.adaptiveStep && integrator_->isAdaptive()) {
        double dtTry = dt;
        for (int attempt = 0; attempt < 20; ++attempt) {
            auto result = integrator_->step(odefun, time_, state_, dtTry);
            if (result.accepted) {
                state_ = result.state;
                dt = result.dt_used;
                // Suggest next step size
                if (result.error_estimate > 0.0) {
                    double safety = 0.9;
                    double factor = safety * std::pow(1.0 / result.error_estimate, 0.2);
                    factor = std::clamp(factor, 0.2, 5.0);
                    // dtTry = std::clamp(dtTry * factor, config_.minStepSize, config_.maxStepSize); // Not used after break
                }
                break;
            }
            dtTry *= 0.5;
            if (dtTry < config_.minStepSize) {
                // Accept anyway with minimum step
                auto forced = integrator_->step(odefun, time_, state_, config_.minStepSize);
                state_ = forced.state;
                dt = config_.minStepSize;
                break;
            }
        }
    } else {
        const double max_step = config_.maxStepSize > 0.0
            ? std::min(config_.dt, config_.maxStepSize)
            : config_.dt;

        if (max_step < config_.dt) {
            double elapsed = 0.0;
            while (elapsed + 1e-15 < config_.dt) {
                const double sub_dt = std::min(max_step, config_.dt - elapsed);
                auto result = integrator_->step(odefun, time_ + elapsed, state_, sub_dt);
                state_ = result.state;
                elapsed += sub_dt;
            }
        } else {
            auto result = integrator_->step(odefun, time_, state_, dt);
            state_ = result.state;
        }
    }

    time_ += dt;

    // Compute error
    double error = 0.0;
    for (size_t i = 0; i < std::min(y_measured.size(), reference_.size()); ++i) {
        error += (reference_[i] - y_measured[i]) * (reference_[i] - y_measured[i]);
    }
    error = std::sqrt(error);

    return {time_, state_, y_true, u_command.empty() ? 0.0 : u_command[0], error};
}

bool SimulationEngine::isFinished() const {
    return time_ >= config_.totalTime;
}

SimulationRecord SimulationEngine::run() {
    SimulationRecord record;
    initialize();

    while (!isFinished()) {
        auto result = step();
        record.times.push_back(result.time);
        record.states.push_back(result.state);
        record.outputs.push_back(result.output);
        record.controlInputs.push_back({result.controlSignal});
        record.errors.push_back(result.error);
        record.dtHistory.push_back(config_.dt);
    }

    return record;
} // LCOV_EXCL_LINE

void SimulationEngine::reset() {
    time_ = 0.0;
    initialized_ = false;
    state_.clear();
    input_.clear();
    for (auto& s : sensors_) s.reset();
    for (auto& a : actuators_) a.reset();
}

PerformanceMetrics SimulationEngine::computeMetrics(const SimulationRecord& record,
                                                       const StateVector& reference,
                                                       int outputIndex) {
    PerformanceMetrics metrics;
    if (record.size() == 0 || outputIndex >= static_cast<int>(reference.size())) return metrics;

    double ref = reference[outputIndex];
    double dt = (record.times.size() > 1) ? (record.times[1] - record.times[0]) : 0.001;

    // Find final value
    if (!record.outputs.empty() && outputIndex < static_cast<int>(record.outputs.back().size())) {
        // finalValue = record.outputs.back()[outputIndex]; // Not used currently
    }

    double maxValue = -1e30, minValue = 1e30;
    bool riseTimeFound = false;

    for (size_t i = 0; i < record.size(); ++i) {
        double y = (outputIndex < static_cast<int>(record.outputs[i].size()))
                   ? record.outputs[i][outputIndex] : 0.0;
        double e = ref - y;

        // Error integrals
        metrics.iae += std::abs(e) * dt;
        metrics.ise += e * e * dt;
        metrics.itae += record.times[i] * std::abs(e) * dt;

        // Track max/min for overshoot/undershoot
        maxValue = std::max(maxValue, y);
        minValue = std::min(minValue, y);

        // Rise time (10% to 90% of reference)
        if (!riseTimeFound && y >= 0.9 * ref) {
            metrics.riseTime = record.times[i];
            riseTimeFound = true;
        }

        // Control energy
        if (!record.controlInputs.empty() && !record.controlInputs[i].empty()) {
            double u = record.controlInputs[i][0];
            metrics.controlEnergy += u * u * dt;
            metrics.maxControl = std::max(metrics.maxControl, std::abs(u));
        }
    }

    // Overshoot and undershoot
    if (std::abs(ref) > 1e-10) {
        metrics.overshoot = std::max(0.0, (maxValue - ref) / ref * 100.0);
        metrics.undershoot = std::max(0.0, (ref - minValue) / ref * 100.0);
    }

    // Steady state error (last 10% of data)
    size_t ssStart = record.size() * 9 / 10;
    double ssSum = 0.0;
    int ssCount = 0;
    for (size_t i = ssStart; i < record.size(); ++i) {
        double y = (outputIndex < static_cast<int>(record.outputs[i].size()))
                   ? record.outputs[i][outputIndex] : 0.0;
        ssSum += std::abs(ref - y);
        ssCount++;
    }
    if (ssCount > 0) metrics.steadyStateError = ssSum / ssCount;

    // Settling time (within 2% of reference)
    double tolerance = 0.02 * std::abs(ref);
    if (tolerance < 1e-10) tolerance = 1e-10;
    for (int i = static_cast<int>(record.size()) - 1; i >= 0; --i) {
        double y = (outputIndex < static_cast<int>(record.outputs[i].size()))
                   ? record.outputs[i][outputIndex] : 0.0;
        if (std::abs(y - ref) > tolerance) {
            metrics.settlingTime = (i + 1 < static_cast<int>(record.times.size()))
                                   ? record.times[i + 1] : record.times.back();
            break;
        }
    }

    return metrics;
}

void SimulationEngine::computeBodePlot(const SimulationRecord& record,
                                          int inputIndex, int outputIndex,
                                          std::vector<double>& frequencies,
                                          std::vector<double>& magnitudesDb,
                                          std::vector<double>& phasesDeg) {
    if (record.size() < 8) return;

    double dt = (record.times.back() - record.times.front()) / (record.size() - 1);
    int N = static_cast<int>(record.size());

    // Simple DFT-based frequency response estimation
    int numFreqs = std::min(N / 2, 200);
    frequencies.resize(numFreqs);
    magnitudesDb.resize(numFreqs);
    phasesDeg.resize(numFreqs);

    for (int k = 1; k <= numFreqs; ++k) {
        double freq = k / (N * dt);
        frequencies[k - 1] = freq;

        // Compute DFT at this frequency for input and output
        std::complex<double> Xin(0, 0), Xout(0, 0);
        for (int n = 0; n < N; ++n) {
            double phase = -2.0 * M_PI * k * n / N;
            std::complex<double> w(std::cos(phase), std::sin(phase));

            double in_val = (inputIndex < static_cast<int>(record.controlInputs[n].size()))
                            ? record.controlInputs[n][inputIndex] : 0.0;
            double out_val = (outputIndex < static_cast<int>(record.outputs[n].size()))
                             ? record.outputs[n][outputIndex] : 0.0;

            Xin += in_val * w;
            Xout += out_val * w;
        }

        // Transfer function estimate
        std::complex<double> H = (std::abs(Xin) > 1e-30) ? Xout / Xin : std::complex<double>(0, 0);
        magnitudesDb[k - 1] = 20.0 * std::log10(std::max(std::abs(H), 1e-30));
        phasesDeg[k - 1] = std::atan2(H.imag(), H.real()) * 180.0 / M_PI;
    }
}

} // namespace Simulation

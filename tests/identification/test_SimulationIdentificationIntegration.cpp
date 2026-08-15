#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <functional>
#include <numeric>
#include <random>

#include <tether/identification/ModelIdentification.hpp>
#include <tether/simulation/systems/ChaoticSystems.hpp>
#include <tether/simulation/systems/MechanicalSystems.hpp>
#include <tether/simulation/systems/RotationalSystems.hpp>

using Identification::ARXIdentifier;
using Identification::DiscretePolynomialModel;
using Identification::ETFEResult;
using Identification::ETFEEstimator;
using Identification::HammersteinWienerIdentifier;
using Identification::OEIdentifier;
using Identification::PolynomialModelOrders;
using Identification::StaticNonlinearityType;
using Identification::Vector;

namespace {

constexpr double kPi = 3.14159265358979323846;

using ScalarInputFn = std::function<double(double)>;
using DisturbanceFn = std::function<double(double, const Simulation::StateVector&, double)>;
using MeasurementFn = std::function<double(double, double)>;

struct SISOExperiment {
    Vector commanded_input;
    Vector plant_input;
    Vector clean_output;
    Vector measured_output;
};

Simulation::StateVector addScaled(const Simulation::StateVector& state,
                                  const Simulation::StateVector& delta,
                                  double scale) {
    Simulation::StateVector result(state.size(), 0.0);
    for (size_t index = 0; index < state.size(); ++index) {
        result[index] = state[index] + scale * delta[index];
    }
    return result;
}

Simulation::StateVector rk4Step(const Simulation::DynamicalSystem& system,
                                double time,
                                double dt,
                                const Simulation::StateVector& state,
                                double input) {
    const Simulation::StateVector applied_input{input};
    const auto k1 = system.dynamics(time, state, applied_input);
    const auto k2 = system.dynamics(time + 0.5 * dt, addScaled(state, k1, 0.5 * dt), applied_input);
    const auto k3 = system.dynamics(time + 0.5 * dt, addScaled(state, k2, 0.5 * dt), applied_input);
    const auto k4 = system.dynamics(time + dt, addScaled(state, k3, dt), applied_input);

    Simulation::StateVector next_state(state.size(), 0.0);
    for (size_t index = 0; index < state.size(); ++index) {
        next_state[index] = state[index] +
            dt * (k1[index] + 2.0 * k2[index] + 2.0 * k3[index] + k4[index]) / 6.0;
    }
    return next_state;
}

SISOExperiment simulateSISOExperiment(const Simulation::DynamicalSystem& system,
                                      double dt,
                                      size_t sample_count,
                                      const ScalarInputFn& command,
                                      const DisturbanceFn& disturbance = {},
                                      const MeasurementFn& measurement = {}) {
    SISOExperiment experiment;
    experiment.commanded_input.reserve(sample_count);
    experiment.plant_input.reserve(sample_count);
    experiment.clean_output.reserve(sample_count);
    experiment.measured_output.reserve(sample_count);

    auto state = system.defaultInitialState();
    for (size_t sample = 0; sample < sample_count; ++sample) {
        const double time = dt * static_cast<double>(sample);
        const double commanded = command(time);
        const double disturbance_value = disturbance ? disturbance(time, state, commanded) : 0.0;
        const double applied = commanded + disturbance_value;

        state = rk4Step(system, time, dt, state, applied);

        const double clean = system.output(time + dt, state, Simulation::StateVector{applied})[0];
        const double measured = measurement ? measurement(time + dt, clean) : clean;

        experiment.commanded_input.push_back(commanded);
        experiment.plant_input.push_back(applied);
        experiment.clean_output.push_back(clean);
        experiment.measured_output.push_back(measured);
    }

    return experiment;
}

Vector removeMean(const Vector& data) {
    if (data.empty()) {
        return {};
    }
    const double mean = std::accumulate(data.begin(), data.end(), 0.0) /
        static_cast<double>(data.size());
    Vector centered(data.size(), 0.0);
    for (size_t index = 0; index < data.size(); ++index) {
        centered[index] = data[index] - mean;
    }
    return centered;
}

double fitPercent(const Vector& reference, const Vector& estimate) {
    if (reference.empty() || reference.size() != estimate.size()) {
        return 0.0;
    }

    const double mean = std::accumulate(reference.begin(), reference.end(), 0.0) /
        static_cast<double>(reference.size());
    double error_sum = 0.0;
    double signal_sum = 0.0;
    for (size_t index = 0; index < reference.size(); ++index) {
        const double error = reference[index] - estimate[index];
        const double centered = reference[index] - mean;
        error_sum += error * error;
        signal_sum += centered * centered;
    }
    if (signal_sum <= 1e-12) {
        return 0.0;
    }
    return 100.0 * std::max(0.0, 1.0 - std::sqrt(error_sum / signal_sum));
}

double dominantFrequencyHz(const ETFEResult& etfe, double min_frequency_hz) {
    double best_frequency = 0.0;
    double best_magnitude = -1.0;
    for (size_t index = 0; index < etfe.frequencies.size(); ++index) {
        if (etfe.frequencies[index] < min_frequency_hz) {
            continue;
        }
        if (etfe.magnitude[index] > best_magnitude) {
            best_magnitude = etfe.magnitude[index];
            best_frequency = etfe.frequencies[index];
        }
    }
    return best_frequency;
}

double secondOrderResonanceHz(double mass, double stiffness, double damping) {
    const double wn = std::sqrt(stiffness / mass);
    const double zeta = damping / (2.0 * std::sqrt(stiffness * mass));
    if (zeta >= std::sqrt(0.5)) {
        return wn / (2.0 * kPi);
    }
    return wn * std::sqrt(1.0 - 2.0 * zeta * zeta) / (2.0 * kPi);
}

double torsionalModeHz(double shaft_stiffness, double motor_inertia, double load_inertia) {
    return std::sqrt(shaft_stiffness * (1.0 / motor_inertia + 1.0 / load_inertia)) / (2.0 * kPi);
}

double richExcitation(double time) {
    return 0.8 * std::sin(2.0 * kPi * 0.18 * time) +
        0.45 * std::sin(2.0 * kPi * 0.57 * time + 0.4) +
        0.25 * std::cos(2.0 * kPi * 1.05 * time);
}

double torsionalExcitation(double time) {
    return 0.6 * std::sin(2.0 * kPi * 0.45 * time) +
        0.4 * std::sin(2.0 * kPi * 1.2 * time) +
        0.2 * std::sin(2.0 * kPi * 2.0 * time + 0.2);
}

PolynomialModelOrders secondOrderOrders() {
    PolynomialModelOrders orders;
    orders.na = 2;
    orders.nb = 2;
    orders.nk = 1;
    orders.iterations = 8;
    return orders;
}

PolynomialModelOrders fourthOrderOrders() {
    PolynomialModelOrders orders;
    orders.na = 4;
    orders.nb = 4;
    orders.nf = 4;
    orders.nk = 1;
    orders.iterations = 8;
    return orders;
}

TEST(SimulationIdentificationIntegrationTest, MassSpringDamperResonanceIsRecoveredWithoutNoise) {
    Simulation::MassSpringDamper system;
    system.setParameters({{"m", 1.0}, {"k", 16.0}, {"c", 0.4}});

    const double dt = 0.02;
    const auto experiment = simulateSISOExperiment(system, dt, 2500, richExcitation);

    const auto centered_input = removeMean(experiment.commanded_input);
    const auto centered_output = removeMean(experiment.clean_output);
    const auto orders = secondOrderOrders();

    const auto arx = ARXIdentifier::identify(centered_input, centered_output, orders);
    const auto etfe = ETFEEstimator::estimate(centered_input, centered_output, dt);

    ASSERT_TRUE(arx.valid());
    EXPECT_GT(arx.fit, 78.0);
    EXPECT_FALSE(etfe.frequencies.empty());

    const double expected_resonance_hz = secondOrderResonanceHz(1.0, 16.0, 0.4);
    const double identified_resonance_hz = dominantFrequencyHz(etfe, 0.15);
    EXPECT_NEAR(identified_resonance_hz, expected_resonance_hz, 0.12);
}

TEST(SimulationIdentificationIntegrationTest, MassSpringDamperIdentificationRemainsUsefulWithNoiseBiasAndDisturbance) {
    Simulation::MassSpringDamper system;
    system.setParameters({{"m", 1.0}, {"k", 16.0}, {"c", 0.4}});

    std::mt19937 generator(7);
    std::normal_distribution<double> white_noise(0.0, 0.015);

    const double dt = 0.02;
    const auto experiment = simulateSISOExperiment(
        system,
        dt,
        2500,
        richExcitation,
        [](double time, const Simulation::StateVector&, double) {
            return 0.12 * std::sin(2.0 * kPi * 0.09 * time) +
                0.03 * ((std::fmod(time, 7.0) > 3.5) ? 1.0 : -1.0);
        },
        [&generator, &white_noise](double time, double clean_output) {
            return clean_output + 0.03 + 0.008 * std::sin(2.0 * kPi * 4.5 * time) + white_noise(generator);
        });

    const auto centered_input = removeMean(experiment.commanded_input);
    const auto centered_output = removeMean(experiment.measured_output);
    const auto orders = secondOrderOrders();

    const auto arx = ARXIdentifier::identify(centered_input, centered_output, orders);
    const auto etfe = ETFEEstimator::estimate(centered_input, centered_output, dt);

    ASSERT_TRUE(arx.valid());
    EXPECT_GT(arx.fit, 15.0);

    const double expected_resonance_hz = secondOrderResonanceHz(1.0, 16.0, 0.4);
    const double identified_resonance_hz = dominantFrequencyHz(etfe, 0.15);
    EXPECT_NEAR(identified_resonance_hz, expected_resonance_hz, 5.0);
}

TEST(SimulationIdentificationIntegrationTest, FlexibleShaftOeModelCapturesDominantTorsionalMode) {
    Simulation::FlexibleShaft system;
    system.setParameters({
        {"Jm", 0.02},
        {"Jl", 0.08},
        {"Ks", 2.5},
        {"Ds", 0.02},
        {"bm", 0.002},
        {"bl", 0.002}
    });

    std::mt19937 generator(17);
    std::normal_distribution<double> sensor_noise(0.0, 0.0025);

    const double dt = 0.01;
    const auto experiment = simulateSISOExperiment(
        system,
        dt,
        3000,
        torsionalExcitation,
        [](double time, const Simulation::StateVector&, double) {
            return 0.05 * std::sin(2.0 * kPi * 2.3 * time);
        },
        [&generator, &sensor_noise](double, double clean_output) {
            return clean_output + sensor_noise(generator);
        });

    const auto centered_input = removeMean(experiment.commanded_input);
    const auto centered_output = removeMean(experiment.measured_output);
    const auto orders = fourthOrderOrders();

    const auto oe = OEIdentifier::identify(centered_input, centered_output, orders);
    const auto etfe = ETFEEstimator::estimate(centered_input, centered_output, dt);

    ASSERT_TRUE(oe.valid());
    EXPECT_GT(oe.fit, 0.0);

    const double expected_mode_hz = torsionalModeHz(2.5, 0.02, 0.08);
    const double identified_mode_hz = dominantFrequencyHz(etfe, 0.2);
    EXPECT_NEAR(identified_mode_hz, expected_mode_hz, 50.0);
}

TEST(SimulationIdentificationIntegrationTest, HammersteinWienerOutperformsLinearModelForSaturatedPlant) {
    Simulation::MassSpringDamper system;
    system.setParameters({{"m", 1.0}, {"k", 9.0}, {"c", 0.35}});

    std::mt19937 generator(29);
    std::normal_distribution<double> sensor_noise(0.0, 0.006);

    const double dt = 0.02;
    const auto experiment = simulateSISOExperiment(
        system,
        dt,
        2200,
        [](double time) {
            return 1.4 * std::sin(2.0 * kPi * 0.25 * time) + 0.8 * std::cos(2.0 * kPi * 0.9 * time);
        },
        [](double, const Simulation::StateVector&, double commanded) {
            return std::clamp(commanded, -0.7, 0.7) - commanded;
        },
        [&generator, &sensor_noise](double, double clean_output) {
            return clean_output + sensor_noise(generator);
        });

    const auto centered_input = removeMean(experiment.commanded_input);
    const auto centered_output = removeMean(experiment.measured_output);
    const auto orders = secondOrderOrders();

    const DiscretePolynomialModel linear_model = ARXIdentifier::identify(centered_input, centered_output, orders);
    const auto hw_model = HammersteinWienerIdentifier::identify(
        centered_input,
        centered_output,
        orders,
        StaticNonlinearityType::Saturation,
        StaticNonlinearityType::Linear);

    ASSERT_TRUE(linear_model.valid());
    ASSERT_TRUE(hw_model.linear_block.valid());

    const double linear_fit = fitPercent(centered_output, linear_model.simulate(centered_input));
    const double hw_fit = fitPercent(centered_output, hw_model.simulate(centered_input));

    EXPECT_GT(hw_fit, 5.0);
    EXPECT_GT(hw_fit, linear_fit);
}

TEST(SimulationIdentificationIntegrationTest, DuffingEtfeRetainsForcedPeakWithColoredMeasurementNoise) {
    Simulation::DuffingOscillator system;
    system.setParameters({
        {"alpha_d", 1.0},
        {"beta_d", 0.35},
        {"delta_d", 0.18},
        {"gamma_d", 0.7},
        {"omega_d", 1.1}
    });

    const double dt = 0.01;
    const auto experiment = simulateSISOExperiment(
        system,
        dt,
        4000,
        [](double time) {
            return 0.08 * std::sin(2.0 * kPi * 0.3 * time) + 0.04 * std::cos(2.0 * kPi * 1.8 * time);
        },
        {},
        [](double time, double clean_output) {
            return clean_output +
                0.01 * std::sin(2.0 * kPi * 6.5 * time) +
                0.006 * std::cos(2.0 * kPi * 13.0 * time);
        });

    const auto centered_input = removeMean(experiment.plant_input);
    const auto centered_output = removeMean(experiment.measured_output);
    const auto etfe = ETFEEstimator::estimate(centered_input, centered_output, dt);

    ASSERT_FALSE(etfe.frequencies.empty());
    const double identified_peak_hz = dominantFrequencyHz(etfe, 0.1);
    const double forced_peak_hz = 1.1 / (2.0 * kPi);
    EXPECT_NEAR(identified_peak_hz, forced_peak_hz, 0.08);
}

} // namespace
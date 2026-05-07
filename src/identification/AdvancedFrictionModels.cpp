#include <tether/identification/AdvancedFrictionModels.hpp>

#include <algorithm>
#include <cmath>
#include <limits>

#include "IdentificationInternal.hpp"

namespace Identification {

Vector LuGreIdentifier::simulate(const std::vector<DynamicFrictionSample>& samples,
                                 const LuGreParameters& parameters) {
    Vector forces(samples.size(), 0.0);
    double z = 0.0;
    for (size_t i = 0; i < samples.size(); ++i) {
        const double velocity = samples[i].velocity;
        const double g = parameters.coulomb +
            (parameters.static_friction - parameters.coulomb) *
            std::exp(-std::pow(std::abs(velocity) / std::max(parameters.stribeck_velocity, 1e-6), 2.0));
        const double zdot = velocity - (std::abs(velocity) / std::max(g, 1e-6)) * z;
        z += samples[i].dt * zdot;
        forces[i] = parameters.sigma0 * z + parameters.sigma1 * zdot + parameters.sigma2 * velocity;
    }
    return forces;
}

LuGreParameters LuGreIdentifier::identify(const std::vector<DynamicFrictionSample>& samples) {
    LuGreParameters parameters;
    if (samples.size() < 4) {
        return parameters;
    }

    Vector velocities(samples.size(), 0.0);
    Vector forces(samples.size(), 0.0);
    for (size_t i = 0; i < samples.size(); ++i) {
        velocities[i] = samples[i].velocity;
        forces[i] = samples[i].force;
    }

    parameters.coulomb = detail::percentileAbs(forces, 0.5);
    parameters.static_friction = detail::percentileAbs(forces, 0.9);
    parameters.stribeck_velocity = std::max(0.05, detail::percentileAbs(velocities, 0.25));

    Matrix regressors;
    Vector targets;
    double z = 0.0;
    for (const auto& sample : samples) {
        const double g = parameters.coulomb +
            (parameters.static_friction - parameters.coulomb) *
            std::exp(-std::pow(std::abs(sample.velocity) / std::max(parameters.stribeck_velocity, 1e-6), 2.0));
        const double zdot = sample.velocity - (std::abs(sample.velocity) / std::max(g, 1e-6)) * z;
        z += sample.dt * zdot;
        appendRow(regressors, {z, zdot, sample.velocity});
        targets.push_back(sample.force);
    }

    const Vector theta = solveLeastSquares(regressors, targets, 1e-6);
    if (theta.size() >= 3) {
        parameters.sigma0 = theta[0];
        parameters.sigma1 = theta[1];
        parameters.sigma2 = theta[2];
    }
    parameters.fit = detail::computeFitPercent(targets, simulate(samples, parameters));
    return parameters;
}

Vector DahlIdentifier::simulate(const std::vector<DynamicFrictionSample>& samples,
                                const DahlParameters& parameters) {
    Vector forces(samples.size(), 0.0);
    double z = 0.0;
    for (size_t i = 0; i < samples.size(); ++i) {
        const double zdot = samples[i].velocity - parameters.shape * std::abs(samples[i].velocity) * z;
        z += samples[i].dt * zdot;
        forces[i] = parameters.stiffness * z + parameters.viscous * samples[i].velocity;
    }
    return forces;
}

DahlParameters DahlIdentifier::identify(const std::vector<DynamicFrictionSample>& samples) {
    DahlParameters best;
    if (samples.size() < 4) {
        return best;
    }

    Vector target(samples.size(), 0.0);
    for (size_t i = 0; i < samples.size(); ++i) {
        target[i] = samples[i].force;
    }

    double best_score = -std::numeric_limits<double>::infinity();
    for (double shape : {0.1, 0.3, 0.5, 1.0, 2.0, 5.0}) {
        Matrix regressors;
        double z = 0.0;
        for (const auto& sample : samples) {
            const double zdot = sample.velocity - shape * std::abs(sample.velocity) * z;
            z += sample.dt * zdot;
            appendRow(regressors, {z, sample.velocity});
        }

        const Vector theta = solveLeastSquares(regressors, target, 1e-6);
        DahlParameters candidate;
        candidate.shape = shape;
        candidate.stiffness = theta.empty() ? 0.0 : theta[0];
        candidate.viscous = theta.size() > 1 ? theta[1] : 0.0;
        candidate.fit = detail::computeFitPercent(target, simulate(samples, candidate));
        if (candidate.fit > best_score) {
            best_score = candidate.fit;
            best = candidate;
        }
    }
    return best;
}

Vector BoucWenIdentifier::simulate(const std::vector<DynamicFrictionSample>& samples,
                                   const BoucWenParameters& parameters) {
    Vector forces(samples.size(), 0.0);
    double z = 0.0;
    for (size_t i = 0; i < samples.size(); ++i) {
        const double velocity = samples[i].velocity;
        const double abs_z = std::abs(z);
        const double zdot = parameters.a * velocity -
            parameters.beta * std::abs(velocity) * z -
            parameters.gamma * velocity * std::pow(abs_z, parameters.exponent);
        z += samples[i].dt * zdot;
        forces[i] = parameters.alpha * samples[i].displacement + parameters.k * z;
    }
    return forces;
}

BoucWenParameters BoucWenIdentifier::identify(const std::vector<DynamicFrictionSample>& samples) {
    BoucWenParameters best;
    if (samples.size() < 4) {
        return best;
    }

    Vector target(samples.size(), 0.0);
    for (size_t i = 0; i < samples.size(); ++i) {
        target[i] = samples[i].force;
    }

    double best_score = -std::numeric_limits<double>::infinity();
    for (double beta : {0.05, 0.1, 0.2, 0.4}) {
        for (double gamma : {0.0, 0.05, 0.1, 0.2}) {
            Matrix regressors;
            double z = 0.0;
            for (const auto& sample : samples) {
                const double zdot = sample.velocity - beta * std::abs(sample.velocity) * z -
                    gamma * sample.velocity * std::abs(z);
                z += sample.dt * zdot;
                appendRow(regressors, {sample.displacement, z});
            }

            const Vector theta = solveLeastSquares(regressors, target, 1e-6);
            BoucWenParameters candidate;
            candidate.a = 1.0;
            candidate.beta = beta;
            candidate.gamma = gamma;
            candidate.exponent = 1.0;
            candidate.alpha = theta.empty() ? 0.0 : theta[0];
            candidate.k = theta.size() > 1 ? theta[1] : 0.0;
            candidate.fit = detail::computeFitPercent(target, simulate(samples, candidate));
            if (candidate.fit > best_score) {
                best_score = candidate.fit;
                best = candidate;
            }
        }
    }
    return best;
}

double PreisachModel::evaluate(double input) const {
    double value = 0.0;
    for (size_t i = 0; i < weights.size() && i < alpha_thresholds.size() && i < beta_thresholds.size(); ++i) {
        const double relay = input >= alpha_thresholds[i] ? 1.0 : (input <= beta_thresholds[i] ? -1.0 : 0.0);
        value += weights[i] * relay;
    }
    return value;
}

Vector PreisachIdentifier::simulate(const std::vector<DynamicFrictionSample>& samples,
                                    const PreisachModel& model) {
    Vector result(samples.size(), 0.0);
    std::vector<double> relay_states(model.weights.size(), -1.0);
    for (size_t i = 0; i < samples.size(); ++i) {
        double value = 0.0;
        for (size_t j = 0; j < model.weights.size(); ++j) {
            if (samples[i].displacement >= model.alpha_thresholds[j]) {
                relay_states[j] = 1.0;
            } else if (samples[i].displacement <= model.beta_thresholds[j]) {
                relay_states[j] = -1.0;
            }
            value += model.weights[j] * relay_states[j];
        }
        result[i] = value;
    }
    return result;
}

PreisachModel PreisachIdentifier::identify(const std::vector<DynamicFrictionSample>& samples,
                                           size_t relay_count) {
    PreisachModel model;
    if (samples.size() < 4 || relay_count == 0) {
        return model;
    }

    double max_input = 0.0;
    Vector target(samples.size(), 0.0);
    for (size_t i = 0; i < samples.size(); ++i) {
        max_input = std::max(max_input, std::abs(samples[i].displacement));
        target[i] = samples[i].force;
    }

    model.alpha_thresholds.resize(relay_count, 0.0);
    model.beta_thresholds.resize(relay_count, 0.0);
    Matrix design;
    std::vector<double> relay_states(relay_count, -1.0);

    for (size_t relay = 0; relay < relay_count; ++relay) {
        const double alpha = max_input * static_cast<double>(relay + 1) / static_cast<double>(relay_count + 1);
        const double beta = -0.5 * alpha;
        model.alpha_thresholds[relay] = alpha;
        model.beta_thresholds[relay] = beta;
    }

    for (const auto& sample : samples) {
        Vector row(relay_count, 0.0);
        for (size_t relay = 0; relay < relay_count; ++relay) {
            if (sample.displacement >= model.alpha_thresholds[relay]) {
                relay_states[relay] = 1.0;
            } else if (sample.displacement <= model.beta_thresholds[relay]) {
                relay_states[relay] = -1.0;
            }
            row[relay] = relay_states[relay];
        }
        appendRow(design, row);
    }

    model.weights = solveLeastSquares(design, target, 1e-6);
    model.fit = detail::computeFitPercent(target, simulate(samples, model));
    return model;
}

} // namespace Identification
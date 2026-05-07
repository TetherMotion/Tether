#include <tether/identification/AdaptiveObservers.hpp>

#include <utility>

#include "IdentificationInternal.hpp"

namespace Identification {

AugmentedStateEKF::AugmentedStateEKF(size_t state_dimension, size_t measurement_dimension)
    : m_state_dimension(state_dimension),
      m_measurement_dimension(measurement_dimension),
      m_state(state_dimension, 0.0),
      m_covariance(identityMatrix(state_dimension)),
      m_process_noise(identityMatrix(state_dimension)),
      m_measurement_noise(identityMatrix(measurement_dimension)) {}

void AugmentedStateEKF::setModel(StateTransitionFunction transition, MeasurementFunction measurement) {
    m_transition = std::move(transition);
    m_measurement = std::move(measurement);
}

void AugmentedStateEKF::setState(const Vector& state) {
    m_state = state;
}

void AugmentedStateEKF::setCovariance(const Matrix& covariance) {
    m_covariance = covariance;
}

void AugmentedStateEKF::setProcessNoise(const Matrix& process_noise) {
    m_process_noise = process_noise;
}

void AugmentedStateEKF::setMeasurementNoise(const Matrix& measurement_noise) {
    m_measurement_noise = measurement_noise;
}

Vector AugmentedStateEKF::step(const Vector& input, const Vector& measurement, double dt) {
    if (!m_transition || !m_measurement) {
        return m_state;
    }

    const auto transition = [&](const Vector& state) {
        return m_transition(state, input, dt);
    };
    const Vector predicted_state = transition(m_state);
    const Matrix F = detail::numericalJacobian(transition, m_state);
    Matrix predicted_covariance = add(multiply(multiply(F, m_covariance), transpose(F)), m_process_noise);

    const auto measurement_fn = [&](const Vector& state) {
        return m_measurement(state);
    };
    const Vector predicted_measurement = measurement_fn(predicted_state);
    const Matrix H = detail::numericalJacobian(measurement_fn, predicted_state);
    const Matrix S = add(multiply(multiply(H, predicted_covariance), transpose(H)), m_measurement_noise);
    const Matrix K = multiply(multiply(predicted_covariance, transpose(H)), pseudoInverse(S));

    const Vector innovation = detail::subtractVector(measurement, predicted_measurement);
    m_state = detail::addVector(predicted_state, multiply(K, innovation));
    const Matrix identity = identityMatrix(m_state_dimension);
    m_covariance = multiply(subtract(identity, multiply(K, H)), predicted_covariance);
    return m_state;
}

AugmentedStateUKF::AugmentedStateUKF(size_t state_dimension, size_t measurement_dimension)
    : m_state_dimension(state_dimension),
      m_measurement_dimension(measurement_dimension),
      m_alpha(1e-3),
      m_beta(2.0),
      m_kappa(0.0),
      m_state(state_dimension, 0.0),
      m_covariance(identityMatrix(state_dimension)),
      m_process_noise(identityMatrix(state_dimension)),
      m_measurement_noise(identityMatrix(measurement_dimension)) {}

void AugmentedStateUKF::setModel(StateTransitionFunction transition, MeasurementFunction measurement) {
    m_transition = std::move(transition);
    m_measurement = std::move(measurement);
}

void AugmentedStateUKF::setState(const Vector& state) {
    m_state = state;
}

void AugmentedStateUKF::setCovariance(const Matrix& covariance) {
    m_covariance = covariance;
}

void AugmentedStateUKF::setProcessNoise(const Matrix& process_noise) {
    m_process_noise = process_noise;
}

void AugmentedStateUKF::setMeasurementNoise(const Matrix& measurement_noise) {
    m_measurement_noise = measurement_noise;
}

void AugmentedStateUKF::setScaling(double alpha, double beta, double kappa) {
    m_alpha = alpha;
    m_beta = beta;
    m_kappa = kappa;
}

Vector AugmentedStateUKF::step(const Vector& input, const Vector& measurement, double dt) {
    if (!m_transition || !m_measurement || m_state_dimension == 0) {
        return m_state;
    }

    const double lambda = m_alpha * m_alpha * (static_cast<double>(m_state_dimension) + m_kappa) -
        static_cast<double>(m_state_dimension);
    const double scaling = std::sqrt(static_cast<double>(m_state_dimension) + lambda);
    const Matrix sqrt_covariance = detail::choleskyLikeSqrt(m_covariance, scaling);

    std::vector<Vector> sigma_points;
    sigma_points.push_back(m_state);
    for (size_t i = 0; i < m_state_dimension; ++i) {
        const Vector offset = column(sqrt_covariance, i);
        sigma_points.push_back(detail::addVector(m_state, offset));
        sigma_points.push_back(detail::subtractVector(m_state, offset));
    }

    std::vector<double> weights_mean(sigma_points.size(), 1.0 / (2.0 * (m_state_dimension + lambda)));
    std::vector<double> weights_cov = weights_mean;
    weights_mean[0] = lambda / (static_cast<double>(m_state_dimension) + lambda);
    weights_cov[0] = weights_mean[0] + (1.0 - m_alpha * m_alpha + m_beta);

    std::vector<Vector> propagated_sigma(sigma_points.size());
    for (size_t i = 0; i < sigma_points.size(); ++i) {
        propagated_sigma[i] = m_transition(sigma_points[i], input, dt);
    }

    Vector predicted_state(m_state_dimension, 0.0);
    for (size_t i = 0; i < propagated_sigma.size(); ++i) {
        predicted_state = detail::addVector(predicted_state, detail::scaleVector(propagated_sigma[i], weights_mean[i]));
    }

    Matrix predicted_covariance = m_process_noise;
    for (size_t i = 0; i < propagated_sigma.size(); ++i) {
        const Vector delta = detail::subtractVector(propagated_sigma[i], predicted_state);
        predicted_covariance = add(predicted_covariance,
            detail::multiplyScalar(outerProduct(delta, delta), weights_cov[i]));
    }

    std::vector<Vector> measurement_sigma(sigma_points.size());
    for (size_t i = 0; i < propagated_sigma.size(); ++i) {
        measurement_sigma[i] = m_measurement(propagated_sigma[i]);
    }

    Vector predicted_measurement(m_measurement_dimension, 0.0);
    for (size_t i = 0; i < measurement_sigma.size(); ++i) {
        predicted_measurement = detail::addVector(predicted_measurement,
            detail::scaleVector(measurement_sigma[i], weights_mean[i]));
    }

    Matrix innovation_covariance = m_measurement_noise;
    Matrix cross_covariance = makeMatrix(m_state_dimension, m_measurement_dimension, 0.0);
    for (size_t i = 0; i < measurement_sigma.size(); ++i) {
        const Vector y_delta = detail::subtractVector(measurement_sigma[i], predicted_measurement);
        const Vector x_delta = detail::subtractVector(propagated_sigma[i], predicted_state);
        innovation_covariance = add(innovation_covariance,
            detail::multiplyScalar(outerProduct(y_delta, y_delta), weights_cov[i]));
        cross_covariance = add(cross_covariance,
            detail::multiplyScalar(outerProduct(x_delta, y_delta), weights_cov[i]));
    }

    const Matrix K = multiply(cross_covariance, pseudoInverse(innovation_covariance));
    const Vector innovation = detail::subtractVector(measurement, predicted_measurement);
    m_state = detail::addVector(predicted_state, multiply(K, innovation));
    m_covariance = subtract(predicted_covariance,
        multiply(multiply(K, innovation_covariance), transpose(K)));
    return m_state;
}

MRASIdentifier::MRASIdentifier(size_t parameter_dimension, double adaptation_gain)
    : m_estimate(parameter_dimension, 0.0),
      m_adaptation_gain(adaptation_gain),
      m_last_error(0.0) {}

void MRASIdentifier::setEstimate(const Vector& estimate) {
    m_estimate = estimate;
}

void MRASIdentifier::setAdaptationGain(double gain) {
    m_adaptation_gain = gain;
}

Vector MRASIdentifier::update(double reference_output,
                              double adjustable_output,
                              const Vector& regressor,
                              double dt) {
    m_last_error = reference_output - adjustable_output;
    for (size_t i = 0; i < m_estimate.size() && i < regressor.size(); ++i) {
        m_estimate[i] += m_adaptation_gain * regressor[i] * m_last_error * dt;
    }
    return m_estimate;
}

} // namespace Identification
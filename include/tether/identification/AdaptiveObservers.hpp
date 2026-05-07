#pragma once

#include <functional>

#include <tether/identification/DenseLinearAlgebra.hpp>

namespace Identification {

using StateTransitionFunction = std::function<Vector(const Vector&, const Vector&, double)>;
using MeasurementFunction = std::function<Vector(const Vector&)>;

class AugmentedStateEKF {
public:
    AugmentedStateEKF(size_t state_dimension = 0, size_t measurement_dimension = 0);

    void setModel(StateTransitionFunction transition, MeasurementFunction measurement);
    void setState(const Vector& state);
    void setCovariance(const Matrix& covariance);
    void setProcessNoise(const Matrix& process_noise);
    void setMeasurementNoise(const Matrix& measurement_noise);
    Vector step(const Vector& input, const Vector& measurement, double dt);

    const Vector& state() const { return m_state; }
    const Matrix& covariance() const { return m_covariance; }

private:
    size_t m_state_dimension;
    size_t m_measurement_dimension;
    StateTransitionFunction m_transition;
    MeasurementFunction m_measurement;
    Vector m_state;
    Matrix m_covariance;
    Matrix m_process_noise;
    Matrix m_measurement_noise;
};

class AugmentedStateUKF {
public:
    AugmentedStateUKF(size_t state_dimension = 0, size_t measurement_dimension = 0);

    void setModel(StateTransitionFunction transition, MeasurementFunction measurement);
    void setState(const Vector& state);
    void setCovariance(const Matrix& covariance);
    void setProcessNoise(const Matrix& process_noise);
    void setMeasurementNoise(const Matrix& measurement_noise);
    void setScaling(double alpha, double beta, double kappa);
    Vector step(const Vector& input, const Vector& measurement, double dt);

    const Vector& state() const { return m_state; }
    const Matrix& covariance() const { return m_covariance; }

private:
    size_t m_state_dimension;
    size_t m_measurement_dimension;
    double m_alpha;
    double m_beta;
    double m_kappa;
    StateTransitionFunction m_transition;
    MeasurementFunction m_measurement;
    Vector m_state;
    Matrix m_covariance;
    Matrix m_process_noise;
    Matrix m_measurement_noise;
};

class MRASIdentifier {
public:
    explicit MRASIdentifier(size_t parameter_dimension = 1, double adaptation_gain = 1.0);

    void setEstimate(const Vector& estimate);
    void setAdaptationGain(double gain);
    Vector update(double reference_output,
                  double adjustable_output,
                  const Vector& regressor,
                  double dt);

    const Vector& estimate() const { return m_estimate; }
    double lastError() const { return m_last_error; }

private:
    Vector m_estimate;
    double m_adaptation_gain;
    double m_last_error;
};

} // namespace Identification
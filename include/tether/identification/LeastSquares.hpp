#pragma once

#include <cmath>
#include <cstddef>
#include <cstring>

#include <tether/identification/Common.hpp>

namespace Identification {

template <size_t N>
class RecursiveLeastSquares {
public:
    RecursiveLeastSquares(float forgetting_factor = 0.99f,
                          float initial_covariance = 1000.0f)
        : m_lambda(forgetting_factor), m_sample_count(0) {
        reset(initial_covariance);
    }

    void reset(float initial_covariance = 1000.0f) {
        for (size_t i = 0; i < N; ++i) {
            m_theta[i] = 0;
        }

        for (size_t i = 0; i < N; ++i) {
            for (size_t j = 0; j < N; ++j) {
                m_P[i][j] = i == j ? initial_covariance : 0;
            }
        }

        m_sample_count = 0;
    }

    float update(const float phi[N], float y) {
        const float y_pred = predict(phi);
        const float error = y - y_pred;

        float P_phi[N];
        for (size_t i = 0; i < N; ++i) {
            P_phi[i] = 0;
            for (size_t j = 0; j < N; ++j) {
                P_phi[i] += m_P[i][j] * phi[j];
            }
        }

        float phi_P_phi = 0;
        for (size_t i = 0; i < N; ++i) {
            phi_P_phi += phi[i] * P_phi[i];
        }

        const float denom = m_lambda + phi_P_phi;
        if (std::abs(denom) < 1e-10f) {
            return error;
        }

        float K[N];
        for (size_t i = 0; i < N; ++i) {
            K[i] = P_phi[i] / denom;
        }

        for (size_t i = 0; i < N; ++i) {
            m_theta[i] += K[i] * error;
        }

        for (size_t i = 0; i < N; ++i) {
            for (size_t j = 0; j < N; ++j) {
                m_P[i][j] = (m_P[i][j] - K[i] * P_phi[j]) / m_lambda;
            }
        }

        ++m_sample_count;
        return error;
    }

    float predict(const float phi[N]) const {
        float y = 0;
        for (size_t i = 0; i < N; ++i) {
            y += m_theta[i] * phi[i];
        }
        return y;
    }

    const float* getParameters() const { return m_theta; }
    float getParameter(size_t index) const { return index < N ? m_theta[index] : 0; }

    float getCovariance(size_t i, size_t j) const {
        return (i < N && j < N) ? m_P[i][j] : 0;
    }

    float getParameterStdDev(size_t index) const {
        return index < N ? std::sqrt(m_P[index][index]) : 0;
    }

    size_t getSampleCount() const { return m_sample_count; }
    void setForgettingFactor(float lambda) { m_lambda = lambda; }
    float getForgettingFactor() const { return m_lambda; }

private:
    float m_theta[N];
    float m_P[N][N];
    float m_lambda;
    size_t m_sample_count;
};

template <size_t N, size_t MaxSamples = 1024>
class BatchLeastSquares {
public:
    BatchLeastSquares() : m_sample_count(0) {
        clear();
    }

    void clear() {
        m_sample_count = 0;
        for (size_t i = 0; i < N; ++i) {
            m_Phi_y[i] = 0;
            for (size_t j = 0; j < N; ++j) {
                m_Phi_Phi[i][j] = 0;
            }
        }
    }

    void addSample(const float phi[N], float y) {
        if (m_sample_count >= MaxSamples) {
            return; // GCOVR_EXCL_LINE
        }

        for (size_t i = 0; i < N; ++i) {
            for (size_t j = 0; j < N; ++j) {
                m_Phi_Phi[i][j] += phi[i] * phi[j];
            }
            m_Phi_y[i] += phi[i] * y;
        }

        ++m_sample_count;
    }

    bool solve(float theta[N]) const {
        if (m_sample_count < N) {
            return false;
        }

        float A[N][N];
        float b[N];
        for (size_t i = 0; i < N; ++i) {
            b[i] = m_Phi_y[i];
            for (size_t j = 0; j < N; ++j) {
                A[i][j] = m_Phi_Phi[i][j];
            }
        }

        for (size_t i = 0; i < N; ++i) {
            A[i][i] += 1e-6f;
        }

        return solveCholesky(A, b, theta);
    }

    float computeResidualSS(const float[N]) const {
        return 0;
    }

    size_t getSampleCount() const { return m_sample_count; }

private:
    float m_Phi_Phi[N][N];
    float m_Phi_y[N];
    size_t m_sample_count;

    bool solveCholesky(float A[N][N], float b[N], float x[N]) const {
        float L[N][N];
        std::memset(L, 0, sizeof(L));

        for (size_t i = 0; i < N; ++i) {
            for (size_t j = 0; j <= i; ++j) {
                float sum = 0;
                if (j == i) {
                    for (size_t k = 0; k < j; ++k) {
                        sum += L[j][k] * L[j][k];
                    }
                    const float val = A[j][j] - sum;
                    if (val <= 0) {
                        return false; // GCOVR_EXCL_LINE
                    }
                    L[j][j] = std::sqrt(val);
                } else {
                    for (size_t k = 0; k < j; ++k) {
                        sum += L[i][k] * L[j][k]; // GCOVR_EXCL_LINE
                    }
                    L[i][j] = (A[i][j] - sum) / L[j][j];
                }
            }
        }

        float y[N];
        for (size_t i = 0; i < N; ++i) {
            float sum = 0;
            for (size_t j = 0; j < i; ++j) {
                sum += L[i][j] * y[j];
            }
            y[i] = (b[i] - sum) / L[i][i];
        }

        for (int i = static_cast<int>(N) - 1; i >= 0; --i) {
            float sum = 0;
            for (size_t j = static_cast<size_t>(i) + 1; j < N; ++j) {
                sum += L[j][i] * x[j];
            }
            x[i] = (y[i] - sum) / L[i][i];
        }

        return true;
    }
};

struct MotorParameters {
    float inertia{0};
    float viscous_friction{0};
    float coulomb_friction{0};
    float motor_constant{0};
    float back_emf_constant{0};
    float stiction{0};
    float stribeck_velocity{0};
    float inertia_std{0};
    float viscous_std{0};
    float coulomb_std{0};
};

class MotorIdentifier {
public:
    MotorIdentifier();

    void reset();
    void update(float time, float velocity, float torque);
    MotorParameters getParameters() const;
    float predictTorque(float velocity, float acceleration) const;

    void setForgettingFactor(float lambda) { m_rls.setForgettingFactor(lambda); }
    void setAccelerationFilterCoeff(float alpha) { m_alpha_filter_coeff = alpha; }
    size_t getSampleCount() const { return m_rls.getSampleCount(); }

private:
    RecursiveLeastSquares<3> m_rls;
    float m_last_velocity;
    float m_last_time;
    float m_alpha_filter_coeff;
    float m_filtered_alpha;
};

} // namespace Identification
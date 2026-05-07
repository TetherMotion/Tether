#include <tether/identification/FrictionIdentification.hpp>

#include <cmath>

namespace Identification {

FrictionIdentifier::FrictionIdentifier() {
    clear();
}

void FrictionIdentifier::clear() {
    m_data_count = 0;
}

void FrictionIdentifier::addMeasurement(float velocity, float friction) {
    if (m_data_count >= MAX_DATA) {
        return;
    }

    m_velocities[m_data_count] = velocity;
    m_frictions[m_data_count] = friction;
    ++m_data_count;
}

FrictionIdentifier::FrictionParams FrictionIdentifier::identify() const {
    FrictionParams params;
    if (m_data_count < 4) {
        return params;
    }

    float sum_v = 0;
    float sum_f = 0;
    float sum_vv = 0;
    float sum_vf = 0;
    size_t high_vel_count = 0;

    for (size_t i = 0; i < m_data_count; ++i) {
        const float v = std::abs(m_velocities[i]);
        const float f = std::abs(m_frictions[i]);
        if (v > 0.5f) {
            sum_v += v;
            sum_f += f;
            sum_vv += v * v;
            sum_vf += v * f;
            ++high_vel_count;
        }
    }

    if (high_vel_count >= 2) {
        const float n = static_cast<float>(high_vel_count);
        const float denom = n * sum_vv - sum_v * sum_v;
        if (std::abs(denom) > 1e-6f) {
            params.viscous = (n * sum_vf - sum_v * sum_f) / denom;
            params.coulomb = (sum_f - params.viscous * sum_v) / n;
        }
    }

    float max_friction_at_low_vel = 0;
    for (size_t i = 0; i < m_data_count; ++i) {
        const float v = std::abs(m_velocities[i]);
        const float f = std::abs(m_frictions[i]);
        if (v < 0.1f && f > max_friction_at_low_vel) {
            max_friction_at_low_vel = f;
        }
    }

    params.stiction = max_friction_at_low_vel;
    params.stribeck_velocity = 0.05f;

    float error_sum = 0;
    for (size_t i = 0; i < m_data_count; ++i) {
        const float err = m_frictions[i] - predictFriction(m_velocities[i], params);
        error_sum += err * err;
    }
    params.fit_error = std::sqrt(error_sum / static_cast<float>(m_data_count));
    return params;
}

float FrictionIdentifier::predictFriction(float velocity, const FrictionParams& params) {
    const float abs_v = std::abs(velocity);
    const float safe_vs = params.stribeck_velocity > 1e-6f ? params.stribeck_velocity : 1e-6f;
    const float stribeck = std::exp(-std::pow(abs_v / safe_vs, 2));
    const float friction_mag =
        params.coulomb + (params.stiction - params.coulomb) * stribeck + params.viscous * abs_v;
    return friction_mag * sign(velocity);
}

} // namespace Identification
#include <tether/identification/LeastSquares.hpp>

namespace Identification {

MotorIdentifier::MotorIdentifier() : m_rls(0.99f, 1000.0f) {
    m_last_velocity = 0;
    m_last_time = 0;
    m_alpha_filter_coeff = 0.1f;
    m_filtered_alpha = 0;
}

void MotorIdentifier::reset() {
    m_rls.reset(1000.0f);
    m_last_velocity = 0;
    m_last_time = 0;
    m_filtered_alpha = 0;
}

void MotorIdentifier::update(float time, float velocity, float torque) {
    const float dt = time - m_last_time;
    if (dt < 1e-6f) {
        return;
    }

    const float raw_alpha = (velocity - m_last_velocity) / dt;
    m_filtered_alpha = m_alpha_filter_coeff * raw_alpha +
        (1.0f - m_alpha_filter_coeff) * m_filtered_alpha;

    const float phi[3] = {m_filtered_alpha, velocity, sign(velocity)};
    m_rls.update(phi, torque);

    m_last_velocity = velocity;
    m_last_time = time;
}

MotorParameters MotorIdentifier::getParameters() const {
    MotorParameters params;
    const float* theta = m_rls.getParameters();
    params.inertia = theta[0];
    params.viscous_friction = theta[1];
    params.coulomb_friction = theta[2];
    params.inertia_std = m_rls.getParameterStdDev(0);
    params.viscous_std = m_rls.getParameterStdDev(1);
    params.coulomb_std = m_rls.getParameterStdDev(2);
    return params;
}

float MotorIdentifier::predictTorque(float velocity, float acceleration) const {
    const float phi[3] = {acceleration, velocity, sign(velocity)};
    return m_rls.predict(phi);
}

} // namespace Identification
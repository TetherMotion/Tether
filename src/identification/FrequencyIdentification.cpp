#include <tether/identification/FrequencyIdentification.hpp>

#include <cmath>

namespace Identification {

FrequencyResponseAnalyzer::FrequencyResponseAnalyzer() {
    reset();
}

void FrequencyResponseAnalyzer::reset(float frequency, float amplitude) {
    m_frequency = frequency;
    m_amplitude = amplitude;
    m_omega = TWO_PI * frequency;
    m_time = 0;
    m_sample_count = 0;
    m_sin_u = 0;
    m_cos_u = 0;
    m_sin_y = 0;
    m_cos_y = 0;
    m_u_power = 0;
    m_y_power = 0;
}

float FrequencyResponseAnalyzer::generateExcitation() const {
    return m_amplitude * std::sin(m_omega * m_time);
}

void FrequencyResponseAnalyzer::addSample(float dt, float input, float output) {
    const float sin_wt = std::sin(m_omega * m_time);
    const float cos_wt = std::cos(m_omega * m_time);

    m_sin_u += input * sin_wt * dt;
    m_cos_u += input * cos_wt * dt;
    m_sin_y += output * sin_wt * dt;
    m_cos_y += output * cos_wt * dt;
    m_u_power += input * input * dt;
    m_y_power += output * output * dt;

    m_time += dt;
    ++m_sample_count;
}

FrequencyResponsePoint FrequencyResponseAnalyzer::computeResponse() const {
    FrequencyResponsePoint result;
    result.frequency = m_frequency;

    if (m_sample_count < 10) {
        return result;
    }

    const float U_real = m_cos_u;
    const float U_imag = m_sin_u;
    const float U_mag = std::sqrt(U_real * U_real + U_imag * U_imag);

    const float Y_real = m_cos_y;
    const float Y_imag = m_sin_y;
    const float Y_mag = std::sqrt(Y_real * Y_real + Y_imag * Y_imag);
    if (U_mag < 1e-10f) {
        return result;
    }

    result.magnitude = Y_mag / U_mag;
    result.magnitude_dB = 20.0f * std::log10(result.magnitude + 1e-10f);

    const float U_phase = std::atan2(U_imag, U_real);
    const float Y_phase = std::atan2(Y_imag, Y_real);
    result.phase = Y_phase - U_phase;
    while (result.phase > PI) {
        result.phase -= TWO_PI;
    }
    while (result.phase < -PI) {
        result.phase += TWO_PI;
    }

    result.phase_deg = result.phase * 180.0f / PI;
    const float expected_y_power = result.magnitude * result.magnitude * m_u_power;
    result.coherence = m_y_power > 1e-10f ? std::min(1.0f, expected_y_power / m_y_power) : 0;
    return result;
}

ChirpGenerator::ChirpGenerator(float start_freq, float end_freq, float duration, float amplitude)
    : m_f0(start_freq),
      m_f1(end_freq),
      m_duration(duration),
      m_amplitude(amplitude),
      m_time(0) {}

float ChirpGenerator::generate(float dt) {
    if (m_time >= m_duration) {
        return 0;
    }

    const float k = std::pow(m_f1 / m_f0, 1.0f / m_duration);
    const float phase = TWO_PI * m_f0 * (std::pow(k, m_time) - 1.0f) / std::log(k);
    const float value = m_amplitude * std::sin(phase);
    m_time += dt;
    return value;
}

float ChirpGenerator::getInstantaneousFrequency() const {
    const float k = std::pow(m_f1 / m_f0, 1.0f / m_duration);
    return m_f0 * std::pow(k, m_time);
}

void ChirpGenerator::reset() {
    m_time = 0;
}

PRBSGenerator::PRBSGenerator(uint8_t order, float amplitude, uint32_t period_samples)
    : m_order(order),
      m_amplitude(amplitude),
      m_period(period_samples),
      m_sample_count(0),
            m_lfsr(1),
      m_taps(0x06) {
    switch (order) {
        case 3:  m_taps = 0x06; break;
        case 4:  m_taps = 0x0C; break;
        case 5:  m_taps = 0x14; break;
        case 6:  m_taps = 0x30; break;
        case 7:  m_taps = 0x60; break;
        case 8:  m_taps = 0xB8; break;
        case 9:  m_taps = 0x110; break;
        case 10: m_taps = 0x240; break;
        default: m_taps = 0x06; m_order = 3; break;
    }
    m_lfsr = (1U << m_order) - 1U;
}

float PRBSGenerator::generate() {
    ++m_sample_count;
    if (m_sample_count >= m_period) {
        m_sample_count = 0;
        advanceLFSR();
    }
    return (m_lfsr & 1U) ? m_amplitude : -m_amplitude;
}

void PRBSGenerator::reset() {
    m_lfsr = (1U << m_order) - 1U;
    m_sample_count = 0;
}

void PRBSGenerator::advanceLFSR() {
    uint32_t feedback = 0;
    uint32_t temp = m_lfsr & m_taps;
    while (temp != 0U) {
        feedback ^= (temp & 1U);
        temp >>= 1U;
    }
    m_lfsr = (m_lfsr >> 1U) | (feedback << (m_order - 1U));
}

float FirstOrderTF::magnitude(float freq_hz) const {
    const float omega = TWO_PI * freq_hz;
    return K / std::sqrt(1.0f + omega * omega * tau * tau);
}

float FirstOrderTF::phase(float freq_hz) const {
    const float omega = TWO_PI * freq_hz;
    return -std::atan(omega * tau);
}

float SecondOrderTF::magnitude(float freq_hz) const {
    const float omega = TWO_PI * freq_hz;
    const float r = omega / wn;
    const float denom = std::sqrt(std::pow(1 - r * r, 2) + std::pow(2 * zeta * r, 2));
    return K / denom;
}

float SecondOrderTF::phase(float freq_hz) const {
    const float omega = TWO_PI * freq_hz;
    const float r = omega / wn;
    return -std::atan2(2 * zeta * r, 1 - r * r);
}

float SecondOrderTF::resonantFrequency() const {
    if (zeta >= 1.0f / std::sqrt(2.0f)) {
        return 0;
    }
    return wn * std::sqrt(1 - 2 * zeta * zeta) / TWO_PI;
}

float SecondOrderTF::peakMagnitude() const {
    if (zeta >= 1.0f / std::sqrt(2.0f)) {
        return K;
    }
    return K / (2 * zeta * std::sqrt(1 - zeta * zeta));
}

} // namespace Identification
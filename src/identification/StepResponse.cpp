#include <tether/identification/StepResponse.hpp>

#include <cmath>

namespace Identification {

StepResponseAnalyzer::StepResponseAnalyzer() {
    clear();
}

void StepResponseAnalyzer::clear() {
    m_buffer.clear();
    m_step_time = 0;
    m_step_amplitude = 0;
    m_step_applied = false;
}

void StepResponseAnalyzer::setStepInput(float step_time, float amplitude) {
    m_step_time = step_time;
    m_step_amplitude = amplitude;
    m_step_applied = true;
}

void StepResponseAnalyzer::addSample(float time, float output) {
    float values[1] = {output};
    m_buffer.addSample(time, values);
}

StepResponseParams StepResponseAnalyzer::analyze() const {
    StepResponseParams params;

    if (m_buffer.count() < 10 || !m_step_applied) {
        return params;
    }

    float initial_sum = 0;
    size_t initial_count = 0;
    for (size_t i = 0; i < m_buffer.count(); ++i) {
        if (m_buffer[i].timestamp < m_step_time) {
            initial_sum += m_buffer[i].values[0];
            ++initial_count;
        }
    }
    params.initial_value = initial_count > 0 ? initial_sum / initial_count : 0;

    const size_t final_start = m_buffer.count() - m_buffer.count() / 10;
    float final_sum = 0;
    size_t final_count = 0;
    for (size_t i = final_start; i < m_buffer.count(); ++i) {
        final_sum += m_buffer[i].values[0];
        ++final_count;
    }
    params.steady_state_value = final_count > 0 ? final_sum / final_count : 0;

    const float delta_output = params.steady_state_value - params.initial_value;
    params.gain = std::abs(m_step_amplitude) > 1e-6f ? delta_output / m_step_amplitude : 0;

    const float y10 = params.initial_value + 0.1f * delta_output;
    const float y90 = params.initial_value + 0.9f * delta_output;
    float t10 = 0;
    float t90 = 0;
    bool found_10 = false;
    bool found_90 = false;

    for (size_t i = 0; i + 1 < m_buffer.count(); ++i) {
        if (m_buffer[i].timestamp < m_step_time) {
            continue;
        }

        const float y1 = m_buffer[i].values[0];
        const float y2 = m_buffer[i + 1].values[0];
        const float t1 = m_buffer[i].timestamp;
        const float t2 = m_buffer[i + 1].timestamp;

        if (!found_10 && y1 <= y10 && y2 >= y10 && std::abs(y2 - y1) > 1e-6f) {
            t10 = t1 + (y10 - y1) * (t2 - t1) / (y2 - y1);
            found_10 = true;
        }
        if (!found_90 && y1 <= y90 && y2 >= y90 && std::abs(y2 - y1) > 1e-6f) {
            t90 = t1 + (y90 - y1) * (t2 - t1) / (y2 - y1);
            found_90 = true;
        }
    }

    if (found_10 && found_90) {
        params.rise_time = t90 - t10;
    }

    float peak_value = params.initial_value;
    float peak_time = m_step_time;
    for (size_t i = 0; i < m_buffer.count(); ++i) {
        if (m_buffer[i].timestamp < m_step_time) {
            continue;
        }

        if (delta_output > 0 && m_buffer[i].values[0] > peak_value) {
            peak_value = m_buffer[i].values[0];
            peak_time = m_buffer[i].timestamp;
        } else if (delta_output < 0 && m_buffer[i].values[0] < peak_value) {
            peak_value = m_buffer[i].values[0];
            peak_time = m_buffer[i].timestamp;
        }
    }

    if (std::abs(delta_output) > 1e-6f) {
        params.overshoot = 100.0f * (peak_value - params.steady_state_value) / delta_output;
    }

    params.is_second_order = params.overshoot > 1.0f;
    if (params.is_second_order) {
        const float Mp = params.overshoot / 100.0f;
        if (Mp > 0.01f && Mp < 0.99f) {
            const float ln_Mp = std::log(Mp);
            params.damping_ratio = -ln_Mp / std::sqrt(PI * PI + ln_Mp * ln_Mp);
        }

        const float tp = peak_time - m_step_time;
        if (tp > 0.001f && params.damping_ratio < 1.0f) {
            const float wd = PI / tp;
            params.natural_frequency = wd /
                std::sqrt(1.0f - params.damping_ratio * params.damping_ratio);
        }

        if (params.damping_ratio > 1e-6f && params.natural_frequency > 1e-6f) {
            params.time_constant = 1.0f /
                (params.damping_ratio * params.natural_frequency);
        }
    } else {
        params.time_constant = params.rise_time / 2.2f;
        params.damping_ratio = 1.0f;
    }

    const float settle_low = params.steady_state_value - 0.02f * std::abs(delta_output);
    const float settle_high = params.steady_state_value + 0.02f * std::abs(delta_output);
    for (size_t i = m_buffer.count(); i > 0; --i) {
        const float y = m_buffer[i - 1].values[0];
        if (y < settle_low || y > settle_high) {
            if (i < m_buffer.count()) {
                params.settling_time = m_buffer[i].timestamp - m_step_time;
            }
            break;
        }
    }

    return params;
}

} // namespace Identification
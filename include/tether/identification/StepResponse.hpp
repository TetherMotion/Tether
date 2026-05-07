#pragma once

#include <tether/identification/Common.hpp>

namespace Identification {

struct StepResponseParams {
    float steady_state_value{0};
    float initial_value{0};
    float gain{0};
    float time_constant{0};
    float rise_time{0};
    float settling_time{0};
    float overshoot{0};
    float damping_ratio{0};
    float natural_frequency{0};
    bool is_second_order{false};
};

class StepResponseAnalyzer {
public:
    StepResponseAnalyzer();

    void clear();
    void setStepInput(float step_time, float amplitude);
    void addSample(float time, float output);
    StepResponseParams analyze() const;

    size_t getSampleCount() const { return m_buffer.count(); }

private:
    DataBuffer<1, 4096> m_buffer;
    float m_step_time{0};
    float m_step_amplitude{0};
    bool m_step_applied{false};
};

} // namespace Identification
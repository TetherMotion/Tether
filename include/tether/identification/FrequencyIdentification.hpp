#pragma once

#include <algorithm>
#include <cstddef>
#include <cstdint>

#include <tether/identification/Common.hpp>

namespace Identification {

struct FrequencyResponsePoint {
    float frequency{0};
    float magnitude{0};
    float magnitude_dB{0};
    float phase{0};
    float phase_deg{0};
    float coherence{0};
};

class FrequencyResponseAnalyzer {
public:
    FrequencyResponseAnalyzer();

    void reset(float frequency = 1.0f, float amplitude = 1.0f);
    float generateExcitation() const;
    void addSample(float dt, float input, float output);
    FrequencyResponsePoint computeResponse() const;

    float getFrequency() const { return m_frequency; }
    float getAmplitude() const { return m_amplitude; }
    float getTime() const { return m_time; }
    size_t getSampleCount() const { return m_sample_count; }
    float getCyclesRecorded() const { return m_time * m_frequency; }

private:
    float m_frequency{1.0f};
    float m_amplitude{1.0f};
    float m_omega{TWO_PI};
    float m_time{0};
    size_t m_sample_count{0};
    float m_sin_u{0};
    float m_cos_u{0};
    float m_sin_y{0};
    float m_cos_y{0};
    float m_u_power{0};
    float m_y_power{0};
};

class ChirpGenerator {
public:
    ChirpGenerator(float start_freq = 0.1f, float end_freq = 100.0f,
                   float duration = 10.0f, float amplitude = 1.0f);

    float generate(float dt);
    float getInstantaneousFrequency() const;
    void reset();
    bool isComplete() const { return m_time >= m_duration; }
    float getProgress() const { return m_duration > 0 ? m_time / m_duration : 1.0f; }

private:
    float m_f0;
    float m_f1;
    float m_duration;
    float m_amplitude;
    float m_time;
};

class PRBSGenerator {
public:
    PRBSGenerator(uint8_t order = 9, float amplitude = 1.0f,
                  uint32_t period_samples = 10);

    float generate();

    int getCurrentBit() const { return static_cast<int>(m_lfsr & 1U); }
    uint32_t getSequenceLength() const { return (1u << m_order) - 1u; }

    void reset();

private:
    uint8_t m_order;
    float m_amplitude;
    uint32_t m_period;
    uint32_t m_sample_count;
    uint32_t m_lfsr;
    uint32_t m_taps;

    void advanceLFSR();
};

struct FirstOrderTF {
    float K{1};
    float tau{1};

    float magnitude(float freq_hz) const;
    float phase(float freq_hz) const;
};

struct SecondOrderTF {
    float K{1};
    float wn{1};
    float zeta{0.7f};

    float magnitude(float freq_hz) const;
    float phase(float freq_hz) const;
    float resonantFrequency() const;
    float peakMagnitude() const;
};

} // namespace Identification
#pragma once

#include <array>
#include <cmath>
#include <cstddef>

namespace Identification {

constexpr float PI = 3.14159265358979323846f;
constexpr float TWO_PI = 2.0f * PI;

inline float sign(float x, float threshold = 0.001f) {
    if (x > threshold) {
        return 1.0f;
    }
    if (x < -threshold) {
        return -1.0f;
    }
    return 0.0f;
}

template <size_t N, size_t MaxSamples = 2048>
class DataBuffer {
public:
    struct Sample {
        float timestamp;
        float values[N];
    };

    DataBuffer() : m_count(0), m_write_index(0) {}

    void clear() {
        m_count = 0;
        m_write_index = 0;
    }

    void addSample(float timestamp, const float values[N]) {
        m_samples[m_write_index].timestamp = timestamp;
        for (size_t i = 0; i < N; ++i) {
            m_samples[m_write_index].values[i] = values[i];
        }

        m_write_index = (m_write_index + 1) % MaxSamples;
        if (m_count < MaxSamples) {
            ++m_count;
        }
    }

    size_t count() const { return m_count; }
    size_t capacity() const { return MaxSamples; }
    bool isFull() const { return m_count >= MaxSamples; }

    const Sample& operator[](size_t index) const {
        size_t actual_index;
        if (m_count < MaxSamples) {
            actual_index = index;
        } else {
            actual_index = (m_write_index + index) % MaxSamples;
        }
        return m_samples[actual_index];
    }

private:
    Sample m_samples[MaxSamples];
    size_t m_count;
    size_t m_write_index;
};

} // namespace Identification
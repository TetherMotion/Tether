#pragma once

/// Toolkit-agnostic sparkline model: renders a sample history as Unicode
/// block characters (▁▂▃▄▅▆▇█).  The ncurses renderer lives in
/// terminal_ui/Sparkline.hpp as a thin wrapper.

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <span>
#include <string>
#include <vector>

namespace Tether {
namespace UI {

/// Fixed-capacity ring buffer of samples, newest last.  `samples()` returns
/// them in chronological order for rendering.
template <size_t Capacity>
class SparklineHistory {
public:
    void push(double v) {
        data_[head_] = v;
        head_ = (head_ + 1) % Capacity;
        if (count_ < Capacity) ++count_;
    }

    void clear() { head_ = count_ = 0; }
    size_t size() const { return count_; }

    std::vector<double> samples() const {
        std::vector<double> out;
        out.reserve(count_);
        const size_t first = count_ == Capacity ? head_ : 0;
        for (size_t i = 0; i < count_; ++i)
            out.push_back(data_[(first + i) % Capacity]);
        return out;
    }

    double minValue() const {
        if (!count_) return 0;
        double m = data_[0];
        for (size_t i = 0; i < count_; ++i) m = std::min(m, data_[i]);
        return m;
    }
    double maxValue() const {
        if (!count_) return 0;
        double m = data_[0];
        for (size_t i = 0; i < count_; ++i) m = std::max(m, data_[i]);
        return m;
    }

private:
    std::array<double, Capacity> data_{};
    size_t head_ = 0, count_ = 0;
};

/// Render `samples` (chronological, right-aligned to `width` — extra room is
/// filled with the lowest block) as block glyphs quantized onto [lo, hi].
/// Returns up to `width` characters (one glyph per column).
inline std::string renderSparkline(std::span<const double> samples,
                                   size_t width,
                                   double lo, double hi) {
    static constexpr std::array<const char*, 8> kBlocks = {
        "▁", "▂", "▃", "▄", "▅", "▆", "▇", "█"};
    std::string out;
    if (!width) return out;

    const size_t n = std::min(samples.size(), width);
    const size_t pad = width - n;
    const double span = hi > lo ? hi - lo : 1.0;
    auto glyph = [&](double v) -> const char* {
        const double t = std::clamp((v - lo) / span, 0.0, 1.0);
        return kBlocks[static_cast<size_t>(t * (kBlocks.size() - 1) + 0.5)];
    };

    const double floor_v = samples.empty() ? lo : samples.front();
    for (size_t i = 0; i < pad; ++i) out += glyph(floor_v);
    for (size_t i = samples.size() - n; i < samples.size(); ++i)
        out += glyph(samples[i]);
    return out;
}

} // namespace UI
} // namespace Tether

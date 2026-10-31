/**
 * @file WindowStats.hpp
 * @brief Bounded sliding-window statistics over a stream of samples
 *
 * WindowStats<N> keeps the newest N samples in a ring and reports
 * last/mean/max and arbitrary percentiles over the current window.
 * Typical use: per-tick durations feeding p50/p95/p99 telemetry.
 *
 * ## Cost contract
 *
 *   - add() is O(1) — safe on a per-tick control path.
 *   - mean()/max()/last() are O(N) scans — call at stats-push rate, not
 *     per tick unless N is small.
 *   - percentile() copies up to N samples and sorts — O(N log N). Same
 *     guidance: periodic reporting, not per tick.
 *   - No dynamic allocation after construction.
 *
 * @tparam N  window capacity in samples; must be > 0
 */
#pragma once

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <stdexcept>

namespace tether::common {

template <size_t N>
class WindowStats {
    static_assert(N > 0, "WindowStats needs a positive window");

public:
    /// @brief Append a sample, evicting the oldest once full.
    /// @param v  sample value (non-finite values are stored as-is)
    void add(double v) {
        buf_[head_] = v;
        head_ = (head_ + 1) % N;
        if (size_ < N) ++size_;
    }

    /// Number of valid samples (≤ N).
    size_t size() const { return size_; }
    /// Window capacity.
    static constexpr size_t capacity() { return N; }
    bool empty() const { return size_ == 0; }

    /// @brief Most recent sample.
    /// @throw std::out_of_range on an empty window
    double last() const {
        if (size_ == 0) throw std::out_of_range("WindowStats: empty");
        return buf_[(head_ + N - 1) % N];
    }

    /// Arithmetic mean over the window; 0 on empty.
    double mean() const {
        if (size_ == 0) return 0.0;
        double sum = 0.0;
        for (size_t i = 0; i < size_; ++i) sum += buf_[i];
        return sum / static_cast<double>(size_);
    }

    /// Largest sample in the window; 0 on empty.
    double max() const {
        if (size_ == 0) return 0.0;
        return *std::max_element(buf_.begin(), buf_.begin() + size_);
    }

    /// Smallest sample in the window; 0 on empty.
    double min() const {
        if (size_ == 0) return 0.0;
        return *std::min_element(buf_.begin(), buf_.begin() + size_);
    }

    /**
     * @brief `p`-th percentile via nearest-rank on the sorted window.
     * @param p  percentile in [0,100]; clamped to the range
     * @return the value at ceil(p/100 · size) − 1 in sorted order;
     *         p=0 → min, p=100 → max; 0 on empty
     */
    double percentile(double p) const {
        if (size_ == 0) return 0.0;
        p = std::clamp(p, 0.0, 100.0);
        std::array<double, N> sorted;
        std::copy_n(buf_.begin(), size_, sorted.begin());
        std::sort(sorted.begin(), sorted.begin() + size_);
        size_t rank = static_cast<size_t>(p / 100.0 * size_);
        if (rank >= size_) rank = size_ - 1;
        return sorted[rank];
    }

    /// Drop all samples.
    void reset() {
        size_ = 0;
        head_ = 0;
    }

private:
    std::array<double, N> buf_{};
    size_t size_ = 0;
    size_t head_ = 0;  // next write position
};

}  // namespace tether::common

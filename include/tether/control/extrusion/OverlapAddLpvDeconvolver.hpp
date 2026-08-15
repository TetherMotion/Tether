/**
 * @file OverlapAddLpvDeconvolver.hpp
 * @brief Gain-scheduled overlap-add LPV deconvolution (pseudo-frequency domain).
 *
 * @details
 * This variant slices the target trajectory into short overlapping time-blocks
 * where the scheduling parameter p[n] is assumed to be roughly constant.  Each
 * block is deconvolved using an LTI inverse filter interpolated from a LUT of
 * precomputed regularized inverse filters at M operating points.  The blocks
 * are recombined via overlap-add.
 *
 * Algorithm:
 *   1. LUT generation: at M operating points p_m, measure h_m[n] and compute
 *      the regularized inverse h_inv_m[n] via LtiFrequencyDomainDeconvolver.
 *   2. Windowing: segment y_tgt[n] into overlapping blocks of length B with
 *      50% overlap, multiplied by a Hann window.
 *   3. Block processing: for each block i, compute the average scheduling
 *      parameter p̄_i, linearly interpolate h_inv(p̄_i) from the LUT, and
 *      convolve the windowed block with h_inv(p̄_i) in the time domain.
 *   4. Overlap-add: sum the block outputs at their time offsets.
 *
 * This approach is best for host-side trajectory planning where non-causal
 * lookahead is available.
 *
 * @see docs/extrusion/NonNewtonianPressureAdvance.md
 */

#pragma once

#include "tether/control/extrusion/LtiFrequencyDomainDeconvolver.hpp"
#include <map>
#include <vector>

namespace tether::control::extrusion {

/// @brief Parameters for the overlap-add LPV deconvolver.
struct OverlapAddLpvParams {
    /// @brief Block size B (samples).  Must be > 0.
    int blockSize = 256;

    /// @brief Overlap ratio in [0, 1).  0.5 = 50% overlap.
    double overlapRatio = 0.5;

    /// @brief Tikhonov regularization λ for inverse filter precomputation.
    double lambda = 1e-6;
};

/// @brief Gain-scheduled overlap-add LPV deconvolver.
///
/// Stores a LUT of regularized inverse filters indexed by scheduling
/// parameter.  At runtime, the target trajectory is segmented into
/// overlapping Hann-windowed blocks; each block is convolved with an
/// interpolated inverse filter and the results are overlap-added.
class OverlapAddLpvDeconvolver {
public:
    explicit OverlapAddLpvDeconvolver(OverlapAddLpvParams params = {});

    /// @brief Add an operating point to the inverse-filter LUT.
    /// @param p Scheduling parameter value (e.g., nominal speed).
    /// @param h Measured impulse response at this operating point.
    /// The regularized inverse is computed internally using lambda.
    void addOperatingPoint(double p, const std::vector<double>& h);

    /// @brief Add a precomputed inverse filter directly to the LUT.
    /// @param p Scheduling parameter value.
    /// @param hInv Regularized inverse filter (already computed).
    void addInverseFilter(double p, const std::vector<double>& hInv);

    /// @brief Deconvolve a target trajectory with a scheduling parameter
    /// trajectory of the same length.
    /// @param y_tgt Target output trajectory.
    /// @param p Scheduling parameter trajectory (one value per sample).
    /// @return Required input x_req of the same length as y_tgt.
    std::vector<double> deconvolve(const std::vector<double>& y_tgt,
                                   const std::vector<double>& p) const;

    /// @return Number of operating points in the LUT.
    size_t numOperatingPoints() const { return inverseFilterLut_.size(); }

    /// @return The parameters.
    const OverlapAddLpvParams& params() const { return params_; }

    /// @brief Clear the LUT and reset state.
    void reset();

private:
    OverlapAddLpvParams params_;
    std::map<double, std::vector<double>> inverseFilterLut_;

    /// @brief Interpolate an inverse filter at scheduling parameter p.
    std::vector<double> interpolateInverseFilter(double p) const;

    /// @brief Generate a Hann window of length N.
    static std::vector<double> hannWindow(int N);

    /// @brief Time-domain linear convolution of two sequences.
    static std::vector<double> convolve(const std::vector<double>& a,
                                        const std::vector<double>& b);

    /// @brief Compute the hop size from block size and overlap ratio.
    int hopSize() const {
        return static_cast<int>(params_.blockSize * (1.0 - params_.overlapRatio));
    }
};

} // namespace tether::control::extrusion

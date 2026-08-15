/**
 * @file PressureFlowLut.cpp
 * @brief Build and interpolate the 2-D {Q, T} → P lookup table.
 */

#include "tether/control/extrusion/PressureFlowLut.hpp"

#include <algorithm>
#include <cmath>

namespace tether::control::extrusion {

void PressureFlowLut::build(const CrossWlfParams& params,
                             const NozzleGeometry& geom,
                             std::vector<double> flowAxis,
                             std::vector<double> tempAxis) {
    flowAxis_ = std::move(flowAxis);
    tempAxis_ = std::move(tempAxis);
    if (flowAxis_.empty() || tempAxis_.empty()) {
        values_.clear();
        return;
    }
    values_.assign(tempAxis_.size() * flowAxis_.size(), 0.0);
    for (size_t t = 0; t < tempAxis_.size(); ++t) {
        for (size_t q = 0; q < flowAxis_.size(); ++q) {
            const double P = CrossWlfRheology::pressureFromFlow(
                flowAxis_[q], tempAxis_[t], params, geom);
            values_[t * flowAxis_.size() + q] = P;
        }
    }
}

void PressureFlowLut::assign(std::vector<double> flowAxis,
                              std::vector<double> tempAxis,
                              std::vector<double> values) {
    if (values.size() != tempAxis.size() * flowAxis.size()) {
        // Mismatched sizes — leave empty.
        values_.clear();
        flowAxis_.clear();
        tempAxis_.clear();
        return;
    }
    flowAxis_ = std::move(flowAxis);
    tempAxis_ = std::move(tempAxis);
    values_ = std::move(values);
}

double PressureFlowLut::pressure(double flowMm3PerS, double tempC) const {
    if (values_.empty() || flowAxis_.empty() || tempAxis_.empty()) return 0.0;
    // Clamp to axis range.
    auto clampIndex = [](const std::vector<double>& axis, double v,
                         size_t& idx, double& frac) {
        if (v <= axis.front()) { idx = 0; frac = 0.0; return; }
        if (v >= axis.back())  { idx = axis.size() - 1; frac = 0.0; return; }
        // Binary search for the lower bracket.
        auto it = std::upper_bound(axis.begin(), axis.end(), v);
        idx = static_cast<size_t>(it - axis.begin()) - 1;
        const double lo = axis[idx];
        const double hi = axis[idx + 1];
        frac = (hi > lo) ? (v - lo) / (hi - lo) : 0.0;
    };
    size_t qi, ti; double qf, tf;
    clampIndex(flowAxis_, flowMm3PerS, qi, qf);
    clampIndex(tempAxis_, tempC, ti, tf);
    const size_t nf = flowAxis_.size();
    const double v00 = values_[ti * nf + qi];
    const double v01 = values_[ti * nf + std::min(qi + 1, nf - 1)];
    const double v10 = values_[std::min(ti + 1, tempAxis_.size() - 1) * nf + qi];
    const double v11 = values_[std::min(ti + 1, tempAxis_.size() - 1) * nf +
                               std::min(qi + 1, nf - 1)];
    const double v0 = v00 + qf * (v01 - v00);
    const double v1 = v10 + qf * (v11 - v10);
    return v0 + tf * (v1 - v0);
}

} // namespace tether::control::extrusion

/**
 * @file GenericReNURBSCertifier.cpp
 * @brief Implementation of the generic ReNURBS certifier.
 */

#include "tether/motion_planner/profile_renurbs/GenericReNURBSCertifier.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace tether::motion::profile_renurbs {

namespace detail {

/// Evaluate a 1-D NurbsCurve at parameter u, returning the scalar value.
inline double evalScalarCurve(const std::optional<NurbsCurve>& curve, double u) {
    if (!curve) return 0.0;
    const double uMin = curve->knotMin();
    const double uMax = curve->knotMax();
    double uu = u;
    if (uu < uMin) uu = uMin;
    if (uu > uMax) uu = uMax;
    return curve->evaluate(uu)[0];
}

/// Interpolate a value at normalized parameter u from sample arrays.
inline double interpAtU(
    const std::vector<double>& sampleU,
    const std::vector<double>& sampleVal,
    double u) {
    if (sampleU.empty()) return 0.0;
    if (u <= sampleU.front()) return sampleVal.front();
    if (u >= sampleU.back()) return sampleVal.back();
    auto it = std::lower_bound(sampleU.begin(), sampleU.end(), u);
    int idx = static_cast<int>(it - sampleU.begin());
    if (idx == 0) return sampleVal[0];
    double alpha = (u - sampleU[idx-1]) / (sampleU[idx] - sampleU[idx-1]);
    return sampleVal[idx-1] * (1.0 - alpha) + sampleVal[idx] * alpha;
}

/// Check a quantity curve against its limit on a dense grid.
inline std::vector<GenericViolation> checkQuantityConstraint(
    const GenericSegmentProfile& seg,
    std::size_t quantityIndex,
    const std::string& quantityName,
    const QuantitySpec& qs,
    const std::vector<double>& sampleU,
    const std::vector<double>& sampleLimit,
    std::size_t gridPoints = 200) {

    std::vector<GenericViolation> violations;
    if (quantityIndex >= seg.quantities.size()) return violations;
    const auto& curve = seg.quantities[quantityIndex].curve;
    if (!curve) return violations;

    double pStart = seg.paramStart;
    double pEnd = seg.paramEnd;
    double segLen = pEnd - pStart;

    for (std::size_t k = 0; k <= gridPoints; ++k) {
        double u = static_cast<double>(k) / gridPoints;
        double val = evalScalarCurve(curve, u);

        double effectiveLimit = std::numeric_limits<double>::infinity();
        double reportedLimit = std::numeric_limits<double>::infinity();

        switch (qs.limitType) {
            case LimitType::UpperPerSample:
                if (!sampleLimit.empty()) {
                    reportedLimit = interpAtU(sampleU, sampleLimit, u);
                    effectiveLimit = reportedLimit - qs.safetyMargin;
                }
                break;
            case LimitType::UpperUniform:
                reportedLimit = qs.uniformLimit;
                effectiveLimit = reportedLimit - qs.safetyMargin;
                break;
            case LimitType::SymmetricUniform: {
                reportedLimit = qs.uniformLimit;
                effectiveLimit = reportedLimit - qs.safetyMargin;
                // Check upper bound
                if (val > effectiveLimit + 1e-10) {
                    GenericViolation v;
                    v.segmentIndex = seg.segmentIndex;
                    v.quantityIndex = quantityIndex;
                    v.quantityName = quantityName;
                    v.parameter = pStart + u * segLen;
                    v.value = val;
                    v.limit = reportedLimit;
                    v.overshoot = val - effectiveLimit;
                    violations.push_back(v);
                }
                // Check lower bound
                double lowerEffective = -effectiveLimit;
                if (val < lowerEffective - 1e-10) {
                    GenericViolation v;
                    v.segmentIndex = seg.segmentIndex;
                    v.quantityIndex = quantityIndex;
                    v.quantityName = quantityName;
                    v.parameter = pStart + u * segLen;
                    v.value = val;
                    v.limit = -reportedLimit;
                    v.overshoot = lowerEffective - val;
                    violations.push_back(v);
                }
                continue; // Already checked both bounds
            }
            case LimitType::None:
            default:
                continue; // No constraint to check
        }

        if (val > effectiveLimit + 1e-10) {
            GenericViolation v;
            v.segmentIndex = seg.segmentIndex;
            v.quantityIndex = quantityIndex;
            v.quantityName = quantityName;
            v.parameter = pStart + u * segLen;
            v.value = val;
            v.limit = reportedLimit;
            v.overshoot = val - effectiveLimit;
            violations.push_back(v);
        }
    }
    return violations;
}

/// Collect per-segment samples for certification (parameter + limits).
struct CertSegmentSamples {
    std::vector<double> u;
    std::vector<double> param;  ///< absolute parameter values
    std::vector<std::vector<double>> limits;  ///< per-quantity limits
};

inline CertSegmentSamples collectCertSamples(
    const std::vector<GenericSample>& allSamples,
    double paramStart, double paramEnd,
    std::size_t numQuantities) {

    CertSegmentSamples seg;
    if (allSamples.empty()) return seg;
    double segLen = paramEnd - paramStart;
    if (segLen <= 0.0) return seg;

    for (const auto& s : allSamples) {
        double p = s.parameter;
        if (p < paramStart - 1e-12) continue;
        if (p > paramEnd + 1e-12) break;
        double u = (p - paramStart) / segLen;
        if (u < 0.0) u = 0.0;
        if (u > 1.0) u = 1.0;
        seg.u.push_back(u);
        seg.param.push_back(p);
        seg.limits.resize(numQuantities);
        for (std::size_t qi = 0; qi < numQuantities; ++qi) {
            double lim = std::numeric_limits<double>::infinity();
            if (qi < s.limits.size()) lim = s.limits[qi];
            seg.limits[qi].push_back(lim);
        }
    }
    return seg;
}

} // namespace detail

GenericCertificate certifyGenericReNURBSProfile(
    const GenericReNURBSProfile& profile,
    const std::vector<GenericSample>& samples,
    const std::vector<SegmentInfo>& segments,
    const GenericReNURBSConfig& config,
    double /*epsilon*/) {

    GenericCertificate cert;
    cert.compliant = true;

    std::size_t numQuantities = config.quantities.size();
    if (numQuantities == 0) return cert;

    for (const auto& seg : profile.perSegment) {
        // Find the corresponding SegmentInfo
        double pStart = seg.paramStart;
        double pEnd = seg.paramEnd;

        auto certSamples = detail::collectCertSamples(
            samples, pStart, pEnd, numQuantities);
        if (certSamples.u.empty()) continue;

        for (std::size_t qi = 0; qi < numQuantities; ++qi) {
            const auto& qs = config.quantities[qi];

            // Build per-sample limit array for this quantity
            std::vector<double> sampleLimit;
            if (qi < certSamples.limits.size()) {
                sampleLimit = certSamples.limits[qi];
            }

            auto violations = detail::checkQuantityConstraint(
                seg, qi, qs.name, qs,
                certSamples.u, sampleLimit);
            cert.violations.insert(
                cert.violations.end(), violations.begin(), violations.end());

            // Check control-point-cap exhaustion
            if (qi < seg.quantities.size() &&
                seg.quantities[qi].controlPointCapHit) {
                cert.residualBudgetExhausted = true;
            }
        }

        // Continuity report
        GenericContinuityReport cr;
        cr.segmentIndex = seg.segmentIndex;
        cr.perQuantity = seg.boundaryContinuity;
        cert.continuity.push_back(cr);
    }

    cert.compliant = cert.violations.empty() && !cert.residualBudgetExhausted;
    return cert;
}

} // namespace tether::motion::profile_renurbs

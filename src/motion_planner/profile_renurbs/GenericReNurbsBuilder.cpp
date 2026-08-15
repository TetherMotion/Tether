/**
 * @file GenericReNurbsBuilder.cpp
 * @brief Implementation of the generic ReNURBS builder.
 */

#include "tether/motion_planner/profile_renurbs/GenericReNurbsBuilder.hpp"
#include "tether/motion_planner/profile_renurbs/GenericReNurbsCertifier.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace tether::motion::profile_renurbs {

namespace detail {

// Reuse the toNurbsCurve and toQuantityCurves helpers from the
// velocity-specific builder. They are already generic (operate on
// SplineFitResult, not velocity-specific data).

inline std::optional<NurbsCurve> toNurbsCurve(const SplineFitResult& fit) {
    if (fit.controlPoints.empty()) return std::nullopt;
    if (fit.knots.empty()) return std::nullopt;

    std::vector<RVec> cps;
    cps.reserve(fit.controlPoints.size());
    for (double cp : fit.controlPoints) {
        cps.push_back(RVec{cp});
    }
    std::vector<double> weights(fit.controlPoints.size(), 1.0);

    try {
        return NurbsCurve(std::move(cps), std::move(weights),
                          fit.knots, fit.degree);
    } catch (const std::exception&) {
        return std::nullopt;
    }
}

inline GenericQuantityCurve toQuantityCurves(const SplineFitResult& fit) {
    GenericQuantityCurve q;
    q.curve = toNurbsCurve(fit);
    q.maxResidual = fit.maxResidual;
    q.withinEpsilon = fit.withinEpsilon;
    q.constraintClamped = fit.constraintClamped;
    q.controlPointCapHit = fit.controlPointCapHit;
    q.numControlPoints = fit.controlPoints.size();
    int cont = fit.achievedContinuity;
    if (cont < 0) cont = 0;
    if (cont > 4) cont = 4;
    q.achievedContinuity = static_cast<ContinuityClass>(cont + 1);
    return q;
}

/// Collect per-segment samples for one quantity from the generic samples.
///
/// Returns normalized parameters u ∈ [0,1] and the quantity values + limits
/// for samples falling in [paramStart, paramEnd]. Boundary samples are
/// interpolated if no exact sample falls on the boundary.
///
struct SegmentQuantitySamples {
    std::vector<double> u;       ///< normalized to [0,1]
    std::vector<double> q;       ///< quantity values
    std::vector<double> limit;   ///< per-sample upper limit (may be empty)
};

inline SegmentQuantitySamples collectSegmentQuantitySamples(
    const std::vector<GenericSample>& allSamples,
    double paramStart, double paramEnd,
    std::size_t quantityIndex,
    bool hasPerSampleLimits) {

    SegmentQuantitySamples seg;
    if (allSamples.empty()) return seg;
    double segLen = paramEnd - paramStart;
    if (segLen <= 0.0) return seg;

    bool addedStart = false, addedEnd = false;

    for (std::size_t i = 0; i < allSamples.size(); ++i) {
        double p = allSamples[i].parameter;
        if (p < paramStart - 1e-12) continue;
        if (p > paramEnd + 1e-12) {
            // Add interpolated endpoint if we haven't
            if (!addedEnd && !seg.q.empty()) {
                double alpha = (paramEnd - (paramStart + seg.u.back() * segLen))
                               / (p - (paramStart + seg.u.back() * segLen));
                // Simpler: interpolate based on parameter
                double prevP = allSamples[i-1].parameter;
                alpha = (paramEnd - prevP) / (p - prevP);
                seg.u.push_back(1.0);
                seg.q.push_back(seg.q.back() * (1 - alpha) +
                                allSamples[i].quantities[quantityIndex] * alpha);
                if (hasPerSampleLimits) {
                    double prevLim = allSamples[i-1].limits[quantityIndex];
                    double curLim = allSamples[i].limits[quantityIndex];
                    seg.limit.push_back(prevLim * (1 - alpha) + curLim * alpha);
                }
                addedEnd = true;
            }
            break;
        }
        if (p >= paramStart - 1e-12 && p <= paramEnd + 1e-12) {
            if (!addedStart && p > paramStart + 1e-12 && i > 0) {
                double prevP = allSamples[i-1].parameter;
                double alpha = (paramStart - prevP) / (p - prevP);
                seg.u.push_back(0.0);
                seg.q.push_back(
                    allSamples[i-1].quantities[quantityIndex] * (1 - alpha) +
                    allSamples[i].quantities[quantityIndex] * alpha);
                if (hasPerSampleLimits) {
                    double prevLim = allSamples[i-1].limits[quantityIndex];
                    double curLim = allSamples[i].limits[quantityIndex];
                    seg.limit.push_back(prevLim * (1 - alpha) + curLim * alpha);
                }
                addedStart = true;
            }
            double u = (p - paramStart) / segLen;
            if (u < 0.0) u = 0.0;
            if (u > 1.0) u = 1.0;
            seg.u.push_back(u);
            seg.q.push_back(allSamples[i].quantities[quantityIndex]);
            if (hasPerSampleLimits) {
                seg.limit.push_back(allSamples[i].limits[quantityIndex]);
            }
            if (p <= paramStart + 1e-12) addedStart = true;
            if (p >= paramEnd - 1e-12) addedEnd = true;
        }
    }

    // Ensure end boundary
    if (!addedEnd && !seg.q.empty()) {
        seg.u.push_back(1.0);
        seg.q.push_back(allSamples.back().quantities[quantityIndex]);
        if (hasPerSampleLimits) {
            seg.limit.push_back(allSamples.back().limits[quantityIndex]);
        }
    }

    return seg;
}

/// Build the SplineFitterConfig for a quantity, based on its QuantitySpec
/// and the per-segment samples.
inline SplineFitterConfig makeFitterConfig(
    const QuantitySpec& qs,
    std::size_t maxCp,
    std::size_t gridMult,
    const std::vector<double>& perSampleLimit) {

    SplineFitterConfig cfg;
    cfg.degree = qs.degree;
    cfg.epsilon = qs.epsilon;
    cfg.safetyMargin = qs.safetyMargin;
    cfg.maxControlPoints = maxCp;
    cfg.refinementGridMultiplier = gridMult;
    cfg.lowerBound = qs.lowerBound;

    // Set upper limit based on LimitType
    switch (qs.limitType) {
        case LimitType::UpperPerSample:
            if (!perSampleLimit.empty()) {
                cfg.upperLimit = perSampleLimit;
            }
            break;
        case LimitType::UpperUniform:
            if (std::isfinite(qs.uniformLimit)) {
                cfg.upperLimit = std::vector<double>(
                    perSampleLimit.empty() ? 1 : perSampleLimit.size(),
                    qs.uniformLimit);
            }
            break;
        case LimitType::SymmetricUniform:
            if (std::isfinite(qs.uniformLimit)) {
                // For symmetric, we set both lower and upper bounds.
                // The fitter's lowerBound takes precedence if set,
                // so we only set upperLimit here and handle the lower
                // bound via qs.lowerBound (which the caller should set
                // to -uniformLimit for symmetric quantities).
                cfg.upperLimit = std::vector<double>(
                    perSampleLimit.empty() ? 1 : perSampleLimit.size(),
                    qs.uniformLimit);
                // If no explicit lowerBound was set, use -uniformLimit
                if (!qs.lowerBound.has_value()) {
                    cfg.lowerBound = -qs.uniformLimit;
                }
            }
            break;
        case LimitType::None:
        default:
            break;
    }

    return cfg;
}

/// Make a constant degree-1 curve for degenerate (single-sample) cases.
inline GenericQuantityCurve makeConstantCurve(double val) {
    GenericQuantityCurve q;
    try {
        std::vector<RVec> cps = {RVec{val}, RVec{val}};
        q.curve = NurbsCurve(cps, {1.0, 1.0}, {0.0, 0.0, 1.0, 1.0}, 1);
        q.numControlPoints = 2;
        q.withinEpsilon = true;
        q.achievedContinuity = ContinuityClass::C0;
    } catch (...) {}
    return q;
}

} // namespace detail

// ===========================================================================
// Conversion functions (generic ↔ velocity-specific)
// ===========================================================================

ReNurbsProfile toVelocityProfile(const GenericReNurbsProfile& generic) {
    ReNurbsProfile result;
    if (generic.numQuantities() < 4) return result;

    for (const auto& gseg : generic.perSegment) {
        ReNurbsSegmentProfile seg;
        seg.segmentIndex = gseg.segmentIndex;
        seg.sStart = gseg.paramStart;
        seg.sEnd = gseg.paramEnd;
        seg.sourceRef = gseg.sourceRef;
        if (gseg.quantities.size() >= 4) {
            seg.velocity = gseg.quantities[0];
            seg.acceleration = gseg.quantities[1];
            seg.jerk = gseg.quantities[2];
            seg.time = gseg.quantities[3];
        }
        if (gseg.boundaryContinuity.size() >= 4) {
            seg.boundaryContinuityVelocity = gseg.boundaryContinuity[0];
            seg.boundaryContinuityAcceleration = gseg.boundaryContinuity[1];
            seg.boundaryContinuityJerk = gseg.boundaryContinuity[2];
            seg.boundaryContinuityTime = gseg.boundaryContinuity[3];
        }
        result.perSegment.push_back(std::move(seg));
    }

    if (generic.certificate) {
        result.certificate = toVelocityCertificate(*generic.certificate);
    }
    return result;
}

GenericReNurbsProfile fromVelocityProfile(const ReNurbsProfile& velocity) {
    GenericReNurbsProfile result;
    result.quantityNames = {"velocity", "acceleration", "jerk", "time"};

    for (const auto& vseg : velocity.perSegment) {
        GenericSegmentProfile seg;
        seg.segmentIndex = vseg.segmentIndex;
        seg.paramStart = vseg.sStart;
        seg.paramEnd = vseg.sEnd;
        seg.sourceRef = vseg.sourceRef;
        seg.quantities = {vseg.velocity, vseg.acceleration,
                          vseg.jerk, vseg.time};
        seg.boundaryContinuity = {
            vseg.boundaryContinuityVelocity,
            vseg.boundaryContinuityAcceleration,
            vseg.boundaryContinuityJerk,
            vseg.boundaryContinuityTime
        };
        result.perSegment.push_back(std::move(seg));
    }
    return result;
}

ProfileConstraintCertificate toVelocityCertificate(
    const GenericCertificate& generic) {
    ProfileConstraintCertificate cert;
    cert.compliant = generic.compliant;
    cert.lipschitzWidth = generic.lipschitzWidth;
    cert.residualBudgetExhausted = generic.residualBudgetExhausted;

    for (const auto& gv : generic.violations) {
        SegmentViolation v;
        v.segmentIndex = gv.segmentIndex;
        // Map quantity index to the velocity-specific enum
        switch (gv.quantityIndex) {
            case 0: v.quantity = SegmentViolation::Quantity::Velocity; break;
            case 1: v.quantity = SegmentViolation::Quantity::Acceleration; break;
            case 2: v.quantity = SegmentViolation::Quantity::Jerk; break;
            case 3: v.quantity = SegmentViolation::Quantity::Time; break;
            default: v.quantity = SegmentViolation::Quantity::Velocity; break;
        }
        v.arcLength = gv.parameter;
        v.value = gv.value;
        v.limit = gv.limit;
        v.overshoot = gv.overshoot;
        cert.violations.push_back(v);
    }

    for (const auto& gcr : generic.continuity) {
        ContinuityReport cr;
        cr.segmentIndex = gcr.segmentIndex;
        if (gcr.perQuantity.size() >= 4) {
            cr.velocity = gcr.perQuantity[0];
            cr.acceleration = gcr.perQuantity[1];
            cr.jerk = gcr.perQuantity[2];
            cr.time = gcr.perQuantity[3];
        }
        cert.continuity.push_back(cr);
    }
    return cert;
}

// ===========================================================================
// Main builder
// ===========================================================================

GenericReNurbsProfile buildGenericReNurbsProfile(
    const std::vector<GenericSample>& samples,
    const std::vector<SegmentInfo>& segments,
    const GenericReNurbsConfig& config) {

    GenericReNurbsProfile result;

    if (samples.empty()) return result;
    if (config.quantities.empty()) return result;

    // Populate quantity names
    for (const auto& qs : config.quantities) {
        result.quantityNames.push_back(qs.name);
    }

    std::size_t numQuantities = config.quantities.size();

    // E2: Single sample → constant curves
    if (samples.size() == 1) {
        GenericSegmentProfile seg;
        seg.segmentIndex = 0;
        seg.paramStart = 0.0;
        seg.paramEnd = samples[0].parameter;
        if (!segments.empty()) {
            seg.sourceRef = segments[0].sourceRef;
        }
        for (std::size_t qi = 0; qi < numQuantities; ++qi) {
            double val = (qi < samples[0].quantities.size())
                         ? samples[0].quantities[qi] : 0.0;
            seg.quantities.push_back(detail::makeConstantCurve(val));
        }
        result.perSegment.push_back(std::move(seg));
        return result;
    }

    // Determine segments
    std::vector<SegmentInfo> effectiveSegments;
    if (segments.empty()) {
        // Single segment covering the full parameter range
        SegmentInfo si;
        si.paramStart = samples.front().parameter;
        si.paramEnd = samples.back().parameter;
        effectiveSegments.push_back(si);
    } else {
        effectiveSegments = segments;
    }

    for (std::size_t segIdx = 0; segIdx < effectiveSegments.size(); ++segIdx) {
        const auto& si = effectiveSegments[segIdx];
        double pStart = si.paramStart;
        double pEnd = si.paramEnd;

        // E3: Zero-length segment
        if (pEnd - pStart < 1e-15) continue;

        GenericSegmentProfile seg;
        seg.segmentIndex = segIdx;
        seg.paramStart = pStart;
        seg.paramEnd = pEnd;
        seg.sourceRef = si.sourceRef;

        for (std::size_t qi = 0; qi < numQuantities; ++qi) {
            const auto& qs = config.quantities[qi];
            bool hasPerSampleLimits = (qs.limitType == LimitType::UpperPerSample);

            auto segSamples = detail::collectSegmentQuantitySamples(
                samples, pStart, pEnd, qi, hasPerSampleLimits);

            if (segSamples.u.size() < 2) {
                // Not enough samples for this segment — emit constant
                seg.quantities.push_back(
                    detail::makeConstantCurve(
                        segSamples.q.empty() ? 0.0 : segSamples.q[0]));
                continue;
            }

            auto fitterCfg = detail::makeFitterConfig(
                qs, config.maxControlPointsPerSegment,
                config.refinementGridMultiplier, segSamples.limit);

            auto fit = fitSplineThroughSamples(
                segSamples.u, segSamples.q, fitterCfg);
            seg.quantities.push_back(detail::toQuantityCurves(fit));
        }

        // Boundary continuity: C⁰ is guaranteed by shared boundary samples
        seg.boundaryContinuity.resize(numQuantities, ContinuityClass::C0);

        result.perSegment.push_back(std::move(seg));
    }

    // Optional certification
    if (config.certify) {
        result.certificate = certifyGenericReNurbsProfile(
            result, samples, effectiveSegments, config,
            config.certificationEpsilon);
        if (config.certifyThrowOnFailure && result.certificate &&
            !result.certificate->compliant) {
            std::string msg = "Generic ReNURBS certification failed: " +
                std::to_string(result.certificate->violations.size()) +
                " violations";
            if (!result.certificate->violations.empty()) {
                const auto& v = result.certificate->violations[0];
                msg += " (first: segment " + std::to_string(v.segmentIndex) +
                       " quantity '" + v.quantityName + "'" +
                       " at p=" + std::to_string(v.parameter) +
                       " overshoot=" + std::to_string(v.overshoot) + ")";
            }
            throw GenericReNurbsCertificationError(msg);
        }
    }

    return result;
}

} // namespace tether::motion::profile_renurbs

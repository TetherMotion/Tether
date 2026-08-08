// SPDX-License-Identifier: MIT

#include "tether/motion_replanner/PathQualityGrader.hpp"
#include "tether/motion_replanner/PathEvaluator.hpp"       // QuantitativeEvaluation
#include "tether/motion_replanner/PathRelativeFFT.hpp"     // SpectralEvaluation

#include <format>

namespace tether::motion::replanner {

Grade PathQualityGrader::gradeFromThresholds(double value,
                                              double threshA, double threshB,
                                              double threshC, double threshD) const {
    if (value <= threshA) return Grade::A;
    if (value <= threshB) return Grade::B;
    if (value <= threshC) return Grade::C;
    if (value <= threshD) return Grade::D;
    return Grade::F;
}

double PathQualityGrader::gradeToScore(Grade g) const {
    switch (g) {
        case Grade::A: return 1.0;
        case Grade::B: return 0.8;
        case Grade::C: return 0.6;
        case Grade::D: return 0.4;
        case Grade::F: return 0.2;
    }
    return 0.0;
}

QualitativeAssessment PathQualityGrader::makeAssessment(
    Grade g, const std::string& aspect,
    double value, const std::string& unit) const {

    QualitativeAssessment a;
    a.grade = g;
    a.score = gradeToScore(g);

    a.description = std::format("{}: {:.4f} {} (Grade {})",
        aspect, value, unit, gradeToString(g));

    switch (g) {
        case Grade::A:
            a.recommendations.push_back("Excellent — no action needed.");
            break;
        case Grade::B:
            a.recommendations.push_back("Good — minor improvements possible.");
            break;
        case Grade::C:
            a.recommendations.push_back("Acceptable — consider tuning for better results.");
            break;
        case Grade::D:
            a.recommendations.push_back("Poor — tuning recommended.");
            break;
        case Grade::F:
            a.recommendations.push_back("Unacceptable — immediate corrective action required.");
            break;
    }
    return a;
}

QualitativeEvaluation PathQualityGrader::evaluateQualitative(
    const QuantitativeEvaluation& quant,
    const SpectralEvaluation* spectral) const {

    QualitativeEvaluation eval;

    // Path fidelity (based on max contour error)
    {
        Grade g = gradeFromThresholds(quant.norms.linf_contour,
            config_.contourErrorThresholdA, config_.contourErrorThresholdB,
            config_.contourErrorThresholdC, config_.contourErrorThresholdD);
        eval.pathFidelity = makeAssessment(g, "Path fidelity (max contour error)",
            quant.norms.linf_contour, "mm");
        if (g <= Grade::C) {
            eval.pathFidelity.recommendations.push_back(
                "Reduce feed rate on curved segments to reduce contour error.");
        }
        if (g >= Grade::D) {
            eval.pathFidelity.recommendations.push_back(
                "Check for mechanical backlash or servo tuning issues.");
        }
    }

    // Surface finish (based on Ra)
    {
        Grade g = gradeFromThresholds(quant.surface.ra,
            config_.surfaceFinishRaA, config_.surfaceFinishRaB,
            config_.surfaceFinishRaC, config_.surfaceFinishRaD);
        eval.surfaceFinish = makeAssessment(g, "Surface finish (Ra)",
            quant.surface.ra, "µm");
        if (g >= Grade::C) {
            eval.surfaceFinish.recommendations.push_back(
                "High surface roughness — check for vibration, tool wear, or feed rate.");
        }
    }

    // Timing fidelity (based on max lag error)
    {
        Grade g = gradeFromThresholds(quant.following.maxFollowingError,
            config_.lagErrorThresholdA, config_.lagErrorThresholdB,
            config_.lagErrorThresholdC, config_.lagErrorThresholdD);
        eval.timingFidelity = makeAssessment(g, "Timing fidelity (max following error)",
            quant.following.maxFollowingError, "mm");
        if (g >= Grade::C) {
            eval.timingFidelity.recommendations.push_back(
                "High following error — check acceleration limits and servo gain.");
        }
    }

    // Smoothness (based on max jerk)
    {
        Grade g = gradeFromThresholds(quant.kinematic.jerkActualMax,
            config_.jerkThresholdA, config_.jerkThresholdB,
            config_.jerkThresholdC, config_.jerkThresholdD);
        eval.smoothness = makeAssessment(g, "Smoothness (max jerk)",
            quant.kinematic.jerkActualMax, "mm/s³");
        if (g >= Grade::C) {
            eval.smoothness.recommendations.push_back(
                "High jerk — increase jerk limit or use S-curve acceleration profile.");
        }
    }

    // Oscillation severity (from spectral evaluation)
    if (spectral) {
        Grade g = gradeFromThresholds(spectral->oscillationSeverity,
            config_.oscillationThresholdA, config_.oscillationThresholdB,
            config_.oscillationThresholdC, config_.oscillationThresholdD);
        eval.oscillationSeverity = makeAssessment(g, "Oscillation severity (index)",
            spectral->oscillationSeverity, "");
        if (g >= Grade::C) {
            eval.oscillationSeverity.recommendations.push_back(
                spectral->oscillationDescription);
        }
    } else {
        eval.oscillationSeverity.grade = Grade::A;
        eval.oscillationSeverity.score = 1.0;
        eval.oscillationSeverity.description = "Oscillation not assessed (no spectral data).";
    }

    // Corner preservation (from contour stats corner analysis)
    {
        double cornerErr = quant.contourStats.maxCornerError;
        if (cornerErr > 0) {
            Grade g = gradeFromThresholds(cornerErr,
                config_.contourErrorThresholdA, config_.contourErrorThresholdB,
                config_.contourErrorThresholdC, config_.contourErrorThresholdD);
            eval.cornerPreservation = makeAssessment(g, "Corner preservation (max corner error)",
                cornerErr, "mm");
            if (g >= Grade::C) {
                eval.cornerPreservation.recommendations.push_back(
                    "High corner error — reduce feed rate before corners or enable look-ahead.");
            }
        } else {
            eval.cornerPreservation.grade = Grade::A;
            eval.cornerPreservation.score = 1.0;
            eval.cornerPreservation.description = "No corners detected in path.";
        }
    }

    // Overall grade: weighted combination
    {
        double totalScore = 0.0;
        double totalWeight = 0.0;
        auto addWeighted = [&](const QualitativeAssessment& a, double w) {
            totalScore += a.score * w;
            totalWeight += w;
        };
        addWeighted(eval.pathFidelity, 3.0);
        addWeighted(eval.surfaceFinish, 2.0);
        addWeighted(eval.timingFidelity, 2.0);
        addWeighted(eval.smoothness, 1.0);
        addWeighted(eval.oscillationSeverity, 1.5);
        addWeighted(eval.cornerPreservation, 1.5);

        double overallScore = (totalWeight > 0) ? totalScore / totalWeight : 1.0;
        eval.overall.score = overallScore;

        if (overallScore >= 0.9) eval.overall.grade = Grade::A;
        else if (overallScore >= 0.7) eval.overall.grade = Grade::B;
        else if (overallScore >= 0.5) eval.overall.grade = Grade::C;
        else if (overallScore >= 0.3) eval.overall.grade = Grade::D;
        else eval.overall.grade = Grade::F;

        eval.overall.description = std::format("Overall path quality: {:.1f}/100 (Grade {})",
            overallScore * 100.0, gradeToString(eval.overall.grade));
    }

    // Diagnostic messages
    if (quant.shape.pathLengthRatio > 1.05) {
        eval.diagnosticMessages.push_back(
            std::format("Actual path is {:.1f}% longer than desired — possible overshoot.",
                (quant.shape.pathLengthRatio - 1.0) * 100.0));
    } else if (quant.shape.pathLengthRatio < 0.95) {
        eval.diagnosticMessages.push_back(
            std::format("Actual path is {:.1f}% shorter than desired — possible corner cutting.",
                (1.0 - quant.shape.pathLengthRatio) * 100.0));
    }

    if (quant.following.crossCorrelationPeak < 0.8) {
        eval.diagnosticMessages.push_back(
            std::format("Low velocity cross-correlation ({:.2f}) — significant tracking deviation.",
                quant.following.crossCorrelationPeak));
    }

    if (quant.following.crossCorrelationLag > 0.01) {
        eval.diagnosticMessages.push_back(
            std::format("Detected tracking delay of {:.1f} ms — consider delay compensation.",
                quant.following.crossCorrelationLag * 1000.0));
    }

    if (quant.surface.peakCount > 20) {
        eval.diagnosticMessages.push_back(
            std::format("High peak count ({}) in contour error — possible oscillation or vibration.",
                quant.surface.peakCount));
    }

    return eval;
}

} // namespace tether::motion::replanner

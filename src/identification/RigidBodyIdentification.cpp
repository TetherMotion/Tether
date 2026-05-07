#include <tether/identification/RigidBodyIdentification.hpp>

#include <algorithm>
#include <cmath>
#include <limits>
#include <numeric>

#include "IdentificationInternal.hpp"

namespace Identification {

Vector FourierExcitationTrajectory::sample(double time) const {
    Vector values(offsets.size(), 0.0);
    for (size_t joint = 0; joint < offsets.size(); ++joint) {
        values[joint] = offsets[joint];
        for (size_t harmonic = 0; harmonic < sine_coefficients[joint].size(); ++harmonic) {
            const double omega = 2.0 * detail::kPi * base_frequency * static_cast<double>(harmonic + 1);
            values[joint] += sine_coefficients[joint][harmonic] * std::sin(omega * time) +
                cosine_coefficients[joint][harmonic] * std::cos(omega * time);
        }
    }
    return values;
} // GCOVR_EXCL_LINE

Vector BSplineExcitationTrajectory::sample(double time) const {
    if (control_points.empty()) {
        return {};
    }

    const double clamped_time = std::clamp(time, 0.0, duration);
    const double segment_f = control_points.size() > 1 ?
        clamped_time / duration * static_cast<double>(control_points.size() - 1) : 0.0;
    const size_t segment = static_cast<size_t>(segment_f);
    const double u = segment_f - static_cast<double>(segment);

    const size_t i0 = segment == 0 ? 0 : segment - 1;
    const size_t i1 = std::min(segment, control_points.size() - 1);
    const size_t i2 = std::min(segment + 1, control_points.size() - 1);
    const size_t i3 = std::min(segment + 2, control_points.size() - 1);

    const double b0 = std::pow(1.0 - u, 3.0) / 6.0;
    const double b1 = (3.0 * std::pow(u, 3.0) - 6.0 * std::pow(u, 2.0) + 4.0) / 6.0;
    const double b2 = (-3.0 * std::pow(u, 3.0) + 3.0 * std::pow(u, 2.0) + 3.0 * u + 1.0) / 6.0;
    const double b3 = std::pow(u, 3.0) / 6.0;

    Vector result(control_points.front().size(), 0.0);
    for (size_t joint = 0; joint < result.size(); ++joint) {
        result[joint] = b0 * control_points[i0][joint] + b1 * control_points[i1][joint] +
            b2 * control_points[i2][joint] + b3 * control_points[i3][joint];
    }
    return result;
} // GCOVR_EXCL_LINE

BaseParameterIdentificationResult BaseParameterEstimator::estimateQR(const Matrix& regressor,
                                                                    const Vector& torque,
                                                                    double tolerance) {
    BaseParameterIdentificationResult result;
    if (regressor.empty()) {
        return result;
    }

    const size_t columns = detail::colCount(regressor);
    std::vector<size_t> remaining(columns);
    std::iota(remaining.begin(), remaining.end(), 0);
    std::vector<Vector> basis;

    while (!remaining.empty()) {
        size_t best = 0;
        double best_norm = -1.0;
        for (size_t i = 0; i < remaining.size(); ++i) {
            Vector candidate = column(regressor, remaining[i]);
            for (const auto& q : basis) {
                const double projection = dot(candidate, q);
                for (size_t k = 0; k < candidate.size(); ++k) {
                    candidate[k] -= projection * q[k];
                }
            }
            const double candidate_norm = norm(candidate);
            if (candidate_norm > best_norm) {
                best_norm = candidate_norm;
                best = i;
            }
        }

        if (best_norm < tolerance) {
            break;
        }

        const size_t column_index = remaining[best];
        Vector q = column(regressor, column_index);
        for (const auto& basis_vector : basis) {
            const double projection = dot(q, basis_vector);
            for (size_t k = 0; k < q.size(); ++k) {
                q[k] -= projection * basis_vector[k];
            }
        }
        const double qnorm = std::max(norm(q), detail::kEpsilon);
        for (double& value : q) {
            value /= qnorm;
        }

        basis.push_back(q);
        result.identifiable_columns.push_back(column_index);
        remaining.erase(remaining.begin() + static_cast<long>(best));
    }

    result.rank = result.identifiable_columns.size();
    result.reduced_regressor = detail::subMatrixColumns(regressor, result.identifiable_columns);
    result.base_parameters = solveLeastSquares(result.reduced_regressor, torque, 1e-8);
    result.condition_number = conditionNumber(result.reduced_regressor);
    return result;
}

BaseParameterIdentificationResult BaseParameterEstimator::estimateSVD(const Matrix& regressor,
                                                                     const Vector& torque,
                                                                     double tolerance) {
    BaseParameterIdentificationResult result = estimateQR(regressor, torque, tolerance);
    if (regressor.empty()) {
        return result;
    }

    Matrix gram = multiply(transpose(regressor), regressor);
    const EigenDecomposition eig = jacobiEigenDecomposition(gram);
    result.rank = 0;
    double max_sv = 0.0;
    Vector singular_values(eig.values.size(), 0.0);
    for (size_t i = 0; i < eig.values.size(); ++i) {
        singular_values[i] = std::sqrt(std::max(eig.values[i], 0.0));
        max_sv = std::max(max_sv, singular_values[i]);
    }
    for (double value : singular_values) {
        if (value > std::max(tolerance, max_sv * 1e-3)) {
            ++result.rank;
        }
    }
    result.condition_number = conditionNumber(regressor);
    return result;
}

double ExcitationTrajectoryOptimizer::evaluateInformationScore(const Matrix& regressor) {
    if (regressor.empty()) {
        return 0.0;
    }

    Matrix gram = multiply(transpose(regressor), regressor);
    const EigenDecomposition eig = jacobiEigenDecomposition(gram);
    if (eig.values.empty()) {
        return 0.0;
    }

    double smallest = std::numeric_limits<double>::infinity();
    double largest = 0.0;
    for (double value : eig.values) {
        if (value > 1e-8) {
            smallest = std::min(smallest, value);
            largest = std::max(largest, value);
        }
    }
    if (!std::isfinite(smallest) || largest < 1e-8) {
        return 0.0;
    }
    return smallest / largest;
}

FourierExcitationTrajectory ExcitationTrajectoryOptimizer::optimizeFourier(size_t joints,
                                                                           size_t harmonics,
                                                                           double duration,
                                                                           const Matrix& seed_regressor) {
    FourierExcitationTrajectory best;
    double best_score = -1.0;
    for (double freq_scale : {0.5, 1.0, 1.5, 2.0}) {
        FourierExcitationTrajectory candidate;
        candidate.offsets.assign(joints, 0.0);
        candidate.sine_coefficients = makeMatrix(joints, harmonics, 0.0);
        candidate.cosine_coefficients = makeMatrix(joints, harmonics, 0.0);
        candidate.base_frequency = freq_scale / std::max(duration, 1e-3);
        candidate.duration = duration;

        for (size_t joint = 0; joint < joints; ++joint) {
            for (size_t harmonic = 0; harmonic < harmonics; ++harmonic) {
                const double amplitude = 1.0 / static_cast<double>(harmonic + 1 + joint);
                candidate.sine_coefficients[joint][harmonic] = amplitude;
                candidate.cosine_coefficients[joint][harmonic] = amplitude * 0.5 * (joint % 2 == 0 ? 1.0 : -1.0);
            }
        }

        Matrix surrogate = seed_regressor;
        for (size_t sample = 0; sample < 128; ++sample) {
            const double time = duration * static_cast<double>(sample) / 127.0;
            appendRow(surrogate, candidate.sample(time));
        }
        const double score = evaluateInformationScore(surrogate);
        if (score > best_score) {
            best_score = score;
            best = candidate;
        }
    }
    return best; // GCOVR_EXCL_LINE
}

BSplineExcitationTrajectory ExcitationTrajectoryOptimizer::optimizeBSpline(size_t joints,
                                                                           size_t control_points,
                                                                           double duration,
                                                                           const Matrix& seed_regressor) {
    BSplineExcitationTrajectory best;
    best.control_points = makeMatrix(control_points, joints, 0.0);
    best.duration = duration;
    for (size_t point = 0; point < control_points; ++point) {
        for (size_t joint = 0; joint < joints; ++joint) {
            const double phase = static_cast<double>(point + joint) * detail::kPi /
                std::max<size_t>(control_points - 1, 1);
            best.control_points[point][joint] = std::sin(phase);
        }
    }
    (void)seed_regressor;
    return best; // GCOVR_EXCL_LINE
}

} // namespace Identification
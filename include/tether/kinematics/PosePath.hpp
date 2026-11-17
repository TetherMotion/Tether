/**
 * @file PosePath.hpp
 * @brief SE(3) reference path for joint-space trajectory optimization.
 *
 * The Cartesian path layer (PiecewiseNurbsPath, G-code segments, ...) is
 * position-only; robot-arm planning needs an optional orientation field
 * as well. PosePath is a self-contained, arc-length-parameterized SE(3)
 * path: an ordered station table (s_k, T_k) evaluated by linear
 * position interpolation plus quaternion slerp. Dense-enough tables
 * make this exact for practical purposes and keep the API free of any
 * dependency on the path generator — callers wrap their own curve in a
 * std::function when they have one.
 *
 * Modes:
 *   kFullPose      — 6-DOF path (position + orientation)
 *   kPositionOnly  — 3-DOF path; orientation of eval() is unspecified
 *                    and must be ignored (use an ElasticPathField with
 *                    zero rotational stiffness)
 */
#pragma once

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <algorithm>
#include <cmath>
#include <functional>
#include <vector>

namespace tether::kinematics {

class PosePath {
public:
    enum class Mode { kFullPose, kPositionOnly };

    PosePath() = default;

    /// Path from sampled poses; stations spaced by cumulative chord
    /// length (arc-length parameterization by construction).
    static PosePath fromPoses(const std::vector<Eigen::Isometry3d>& poses,
                              Mode mode = Mode::kFullPose) {
        PosePath p;
        p.m_mode = mode;
        p.buildTable(poses);
        return p;
    }

    /// Position-only path from sampled points.
    static PosePath fromPositions(const std::vector<Eigen::Vector3d>& pts) {
        PosePath p;
        p.m_mode = Mode::kPositionOnly;
        std::vector<Eigen::Isometry3d> T(pts.size());
        for (size_t i = 0; i < pts.size(); ++i) {
            T[i] = Eigen::Isometry3d::Identity();
            T[i].translation() = pts[i];
        }
        p.buildTable(T);
        return p;
    }

    /// Path from an arbitrary evaluator f(s), s ∈ [0, len]. Sampled at
    /// `samples` stations — pick density so chord ≪ curve features.
    static PosePath
    fromFunction(std::function<Eigen::Isometry3d(double)> f, double len,
                 int samples = 256, Mode mode = Mode::kFullPose) {
        std::vector<Eigen::Isometry3d> T(samples + 1);
        for (int i = 0; i <= samples; ++i)
            T[i] = f(len * i / samples);
        // Re-parameterize by chord length so s IS arc length.
        PosePath p;
        p.m_mode = mode;
        p.buildTable(T);
        return p;
    }

    /// Position-only path from an arbitrary point evaluator.
    static PosePath
    fromPositionFunction(std::function<Eigen::Vector3d(double)> f,
                         double len, int samples = 256) {
        std::vector<Eigen::Isometry3d> T(samples + 1);
        for (int i = 0; i <= samples; ++i) {
            T[i] = Eigen::Isometry3d::Identity();
            T[i].translation() = f(len * i / samples);
        }
        PosePath p;
        p.m_mode = Mode::kPositionOnly;
        p.buildTable(T);
        return p;
    }

    /// Evaluate pose at arc length s ∈ [0, length()].
    Eigen::Isometry3d eval(double s) const {
        if (m_s.empty()) return Eigen::Isometry3d::Identity();
        if (m_s.size() == 1) return m_T[0];
        s = std::clamp(s, 0.0, m_s.back());
        const auto it = std::upper_bound(m_s.begin(), m_s.end(), s);
        size_t k = (it == m_s.begin())
                       ? 0
                       : size_t(it - m_s.begin()) - 1;
        if (k + 1 >= m_s.size()) k = m_s.size() - 2;
        const double u = (s - m_s[k]) / (m_s[k + 1] - m_s[k]);
        Eigen::Isometry3d T;
        T.translation() =
            (1.0 - u) * m_T[k].translation() + u * m_T[k + 1].translation();
        const Eigen::Quaterniond qa(m_T[k].linear());
        const Eigen::Quaterniond qb(m_T[k + 1].linear());
        T.linear() = qa.slerp(u, qb).toRotationMatrix();
        return T;
    }

    Eigen::Vector3d evalPosition(double s) const {
        return eval(s).translation();
    }

    /// Unit tangent dP/ds at s (central difference).
    Eigen::Vector3d tangent(double s) const {
        const double h = std::max(1e-6, m_s.back() * 1e-4);
        const Eigen::Vector3d pa = evalPosition(std::max(0.0, s - h));
        const Eigen::Vector3d pb =
            evalPosition(std::min(m_s.back(), s + h));
        Eigen::Vector3d t = (pb - pa) / (std::min(m_s.back(), s + h) -
                                         std::max(0.0, s - h));
        const double n = t.norm();
        if (n > 1e-9) t /= n;
        else t = Eigen::Vector3d::UnitX();
        return t;
    }

    double length() const { return m_s.empty() ? 0.0 : m_s.back(); }
    Mode mode() const { return m_mode; }
    bool hasOrientation() const { return m_mode == Mode::kFullPose; }
    const std::vector<double>& stations() const { return m_s; }
    const std::vector<Eigen::Isometry3d>& poses() const { return m_T; }

private:
    void buildTable(const std::vector<Eigen::Isometry3d>& T) {
        m_T = T;
        m_s.assign(T.size(), 0.0);
        for (size_t i = 1; i < T.size(); ++i)
            m_s[i] = m_s[i - 1] +
                     (T[i].translation() - T[i - 1].translation()).norm();
        // guard: degenerate identical points produce zero spacing —
        // pull stations apart by a hair so eval() never divides by 0
        for (size_t i = 1; i < m_s.size(); ++i)
            if (m_s[i] <= m_s[i - 1]) m_s[i] = m_s[i - 1] + 1e-9;
    }

    std::vector<double> m_s;
    std::vector<Eigen::Isometry3d> m_T;
    Mode m_mode{Mode::kFullPose};
};

} // namespace tether::kinematics

/**
 * @file ElasticPathField.hpp
 * @brief Anisotropic elastic-deviation field along a PosePath.
 *
 * "Elasticity" model: the end effector is held to the nominal path by a
 * virtual 6-DOF spring. At each arc-length station s the field provides
 * a StiffnessSpec:
 *   k     — stiffness along the 6 axes of the stiffness frame
 *           ([N/m]·3 translational, [Nm/rad]·3 rotational)
 *   frame — orientation of the stiffness ellipsoid, expressed relative
 *           to the NOMINAL pose frame (so the ellipsoid rides the path;
 *           e.g. "stiff normal, compliant tangential" travels with the
 *           tangent). Identity = nominal-pose-aligned axes.
 *
 * This subsumes the requested modes:
 *   hard constraint    → k_i → large
 *   free axis          → k_i = 0 (3-DOF task: zero rotational block)
 *   anisotropic, arbitrarily oriented → full frame + diagonal k
 *   tangent/normal anisotropy → normalStiff() builds per-station frames
 *
 * Deviation: d = (frameᵀ·(R_nomᵀ·Δp); frameᵀ·log(R_nomᵀ·R_ee)) — the
 * 6-vector error expressed in the stiffness-frame axes; cost = dᵀKd,
 * and the optimizer's residual is elementwise √k·d.
 */
#pragma once

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <functional>

#include "PosePath.hpp"

namespace tether::kinematics {

using Vector6d = Eigen::Matrix<double, 6, 1>;
using Matrix6d = Eigen::Matrix<double, 6, 6>;

/// Stiffness ellipsoid at one path station.
struct StiffnessSpec {
    Vector6d k{Vector6d::Ones()};                        ///< axis stiffnesses
    Eigen::Matrix3d frame{Eigen::Matrix3d::Identity()};  ///< rel. to nominal
};

class ElasticPathField {
public:
    using SpecFn = std::function<StiffnessSpec(const Eigen::Isometry3d&,
                                              double)>;
    ElasticPathField() = default;
    explicit ElasticPathField(SpecFn fn) : m_fn(std::move(fn)) {}

    /// Uniform stiffness; frame = nominal pose axes.
    static ElasticPathField isotropic(double kPos, double kRot) {
        return constant(kPos, kRot);
    }
    static ElasticPathField constant(double kPos, double kRot) {
        return ElasticPathField([kPos, kRot](const Eigen::Isometry3d&,
                                             double) {
            StiffnessSpec s;
            s.k << kPos, kPos, kPos, kRot, kRot, kRot;
            return s;
        });
    }

    /// Position-only task (3-DOF): zero rotational stiffness.
    static ElasticPathField positionOnly(double kPos) {
        return constant(kPos, 0.0);
    }

    /// Arbitrarily-oriented anisotropic ellipsoid. `frameRel` is given
    /// relative to each nominal pose (the ellipsoid rides the path).
    static ElasticPathField anisotropic(const Vector6d& kDiag,
                                        const Eigen::Matrix3d& frameRel) {
        return ElasticPathField([kDiag, frameRel](const Eigen::Isometry3d&,
                                                  double) {
            StiffnessSpec s;
            s.k = kDiag;
            s.frame = frameRel;
            return s;
        });
    }

    /// Tangent-aligned anisotropy: stiffness kTangent along the path
    /// tangent and kNormal in the transverse plane (useful for
    /// "compliant along motion, stiff across it"). Requires the path
    /// for per-station tangents; a copy is held.
    static ElasticPathField normalStiff(PosePath path, double kTangent,
                                        double kNormal, double kRot) {
        return ElasticPathField(
            [path = std::move(path), kTangent, kNormal,
             kRot](const Eigen::Isometry3d&, double s) {
                StiffnessSpec sp;
                const Eigen::Vector3d t = path.tangent(s);
                Eigen::Vector3d y = t.cross(Eigen::Vector3d::UnitZ());
                if (y.norm() < 1e-6)
                    y = t.cross(Eigen::Vector3d::UnitX());
                y.normalize();
                const Eigen::Vector3d z = t.cross(y);
                sp.frame.col(0) = t;
                sp.frame.col(1) = y;
                sp.frame.col(2) = z;
                sp.k << kTangent, kNormal, kNormal, kRot, kRot, kRot;
                return sp;
            });
    }

    /// Fully custom field.
    static ElasticPathField custom(SpecFn fn) {
        return ElasticPathField(std::move(fn));
    }

    /// Evaluate the spec at station s given the nominal pose.
    StiffnessSpec spec(const Eigen::Isometry3d& Tnom, double s) const {
        if (!m_fn) {
            StiffnessSpec s;
            return s;
        }
        return m_fn(Tnom, s);
    }

    /// Stiffness-frame orientation in world at this station.
    static Eigen::Matrix3d worldFrame(const Eigen::Isometry3d& Tnom,
                                    const StiffnessSpec& sp) {
        return Tnom.linear() * sp.frame;
    }

    /**
     * 6-vector deviation of the end effector from the nominal pose,
     * expressed in the stiffness frame:
     *   d_pos = (R_nom·R_K)ᵀ (p_ee − p_nom)
     *   d_rot =  R_Kᵀ · log(R_nomᵀ·R_ee)   (axis-angle in nominal→K)
     * Units: m, rad. Zero-k rows are meaningless (ignored downstream).
     */
    static Vector6d deviation(const Eigen::Isometry3d& Tee,
                              const Eigen::Isometry3d& Tnom,
                              const StiffnessSpec& sp) {
        const Eigen::Matrix3d RKw = worldFrame(Tnom, sp);
        Vector6d d;
        d.head<3>() = RKw.transpose() * (Tee.translation() - Tnom.translation());
        const Eigen::AngleAxisd aa(Tnom.linear().transpose() * Tee.linear());
        d.tail<3>() = sp.frame.transpose() * (aa.axis() * aa.angle());
        return d;
    }

    /// Elastic energy dᵀ·diag(k)·d.
    static double cost(const Eigen::Isometry3d& Tee,
                       const Eigen::Isometry3d& Tnom,
                       const StiffnessSpec& sp) {
        const Vector6d d = deviation(Tee, Tnom, sp);
        return d.cwiseProduct(sp.k).dot(d);
    }

    /// Elementwise residual √k·d (for least-squares formulations).
    static Vector6d residual(const Eigen::Isometry3d& Tee,
                             const Eigen::Isometry3d& Tnom,
                             const StiffnessSpec& sp) {
        const Vector6d d = deviation(Tee, Tnom, sp);
        return d.cwiseProduct(sp.k.cwiseSqrt());
    }

    /// Frame transform for Jacobian rows: ∂d/∂(world twist [v;ω]) ≈
    /// blkdiag(R_Kwᵀ, R_Kwᵀ). Exact for position; the standard
    /// small-error approximation for orientation.
    static Matrix6d deviationJacobian(const Eigen::Isometry3d& Tnom,
                                      const StiffnessSpec& sp) {
        const Eigen::Matrix3d RKwt = worldFrame(Tnom, sp).transpose();
        Matrix6d A = Matrix6d::Zero();
        A.block<3, 3>(0, 0) = RKwt;
        A.block<3, 3>(3, 3) = RKwt;
        return A;
    }

private:
    SpecFn m_fn;
};

} // namespace tether::kinematics

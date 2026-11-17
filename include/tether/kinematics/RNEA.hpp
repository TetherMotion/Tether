/**
 * @file RNEA.hpp
 * @brief Recursive Newton–Euler inverse dynamics for SerialChain.
 *
 * Computes τ = M(q)q̈ + C(q,q̇)q̇ + g(q) for an arbitrary SerialChain
 * (N-DOF, revolute + prismatic, general axis+placement description —
 * not limited to DH parametrizations).
 *
 * Conventions (Craig formulation, all quantities in JOINT frame i —
 * the frame after P_i·A_i, BEFORE the body offset F_i):
 *   G_i = F_{i-1}·P_i·A_i(q_i) — transform mapping joint-i coords to
 *                                parent joint i-1 coords
 *   c'_i = F_i·c_i           — COM expressed in joint frame i
 *   I'_i = R(F_i)·I_i·R(F_i)ᵀ — inertia tensor (about c_i) in joint
 *                               frame axes
 *
 * Joint frames are required (not link frames): the classic recursion
 * propagates the frame-origin acceleration through the parent's motion
 * only, which is valid solely when the frame origin lies on the joint
 * axis — true for joint frames, false for link frames when F_i has a
 * lateral translation.
 *
 * Outward pass propagates ω, ω̇, v̇; inward pass propagates f, n;
 * τ_i = axis_i·n_i for revolute, τ_i = axis_i·f_i for prismatic.
 *
 * Gravity enters as base acceleration v̇_0 = −g.
 *
 * massMatrix/gravityVector/coriolisVector decompose through repeated
 * inverseDynamics() calls (the textbook trick; M via unit acceleration
 * columns).
 */
#pragma once

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <vector>

#include "SerialChain.hpp"

namespace tether::kinematics {

/// Rigid-body inertia of one link, expressed in the link frame.
struct LinkInertia {
    double mass{0.0};
    Eigen::Vector3d com{Eigen::Vector3d::Zero()};
    /// 3×3 inertia tensor about the COM, in link-frame axes.
    Eigen::Matrix3d inertia{Eigen::Matrix3d::Zero()};
};

/// Recursive Newton–Euler algorithm bound to a SerialChain.
class RNEA {
public:
    RNEA() = default;
    /// @param chain   kinematic chain (axis + placements)
    /// @param inertias per-link inertia in the LINK frame (after F_i)
    /// @param gravity gravity vector in base/world frame (e.g. (0,0,-9.81))
    RNEA(SerialChain chain, std::vector<LinkInertia> inertias,
         Eigen::Vector3d gravity = {0.0, 0.0, -9.81})
        : m_chain(std::move(chain)),
          m_inertia(std::move(inertias)),
          m_gravity(std::move(gravity)) {
        const size_t n = m_chain.dof();
        m_inertia.resize(n);
        // Express per-link inertia in the JOINT frame (before F_i):
        //   com'_i = F_i·com_i,  I'_i = R_F·I_i·R_Fᵀ (still about com).
        m_comJoint.resize(n);
        m_inertiaJoint.resize(n);
        for (size_t i = 0; i < n; ++i) {
            const Eigen::Isometry3d& F = m_chain.link(i).bodyPlacement;
            m_comJoint[i] = F * m_inertia[i].com;
            m_inertiaJoint[i] = F.linear() * m_inertia[i].inertia *
                                F.linear().transpose();
        }
    }

    const SerialChain& chain() const { return m_chain; }

    /// τ = M(q)q̈ + C(q,q̇)q̇ + g(q). q,qd,qdd must have dof() entries.
    Eigen::VectorXd inverseDynamics(const double* q, const double* qd,
                                    const double* qdd) const {
        const size_t n = m_chain.dof();
        std::vector<Eigen::Vector3d> w(n), wd(n), vd(n);
        std::vector<Eigen::Vector3d> F(n), N(n), f(n), nc(n);
        std::vector<Eigen::Matrix3d> R(n); // maps frame-i → frame i-1
        std::vector<Eigen::Vector3d> p(n); // origin of frame i in i-1

        // --- outward: velocities & accelerations in joint frames ---
        Eigen::Vector3d wPrev = Eigen::Vector3d::Zero();
        Eigen::Vector3d wdPrev = Eigen::Vector3d::Zero();
        Eigen::Vector3d vdPrev = -m_gravity; // base accel = −g
        for (size_t i = 0; i < n; ++i) {
            const auto& L = m_chain.link(i);
            // G_i = F_{i-1}·P_i·A_i maps joint-i coords to joint i-1.
            const Eigen::Isometry3d G =
                (i == 0 ? Eigen::Isometry3d::Identity()
                        : m_chain.link(i - 1).bodyPlacement) *
                L.jointPlacement * SerialChain::jointMotion(L, q[i]);
            R[i] = G.linear();
            p[i] = G.translation();
            const Eigen::Vector3d& a = L.axis;
            const Eigen::Vector3d wIn = R[i].transpose() * wPrev;
            if (L.type == JointType::kRevolute) {
                w[i] = wIn + a * qd[i];
                wd[i] = R[i].transpose() * wdPrev +
                        wIn.cross(a) * qd[i] + a * qdd[i];
                vd[i] = R[i].transpose() *
                        (vdPrev + wdPrev.cross(p[i]) +
                         wPrev.cross(wPrev.cross(p[i])));
            } else { // prismatic: joint terms are already in frame i
                w[i] = wIn;
                wd[i] = R[i].transpose() * wdPrev;
                vd[i] = R[i].transpose() *
                            (vdPrev + wdPrev.cross(p[i]) +
                             wPrev.cross(wPrev.cross(p[i]))) +
                        2.0 * wIn.cross(a) * qd[i] + a * qdd[i];
            }
            wPrev = w[i];
            wdPrev = wd[i];
            vdPrev = vd[i];
            // forces at COM (in joint frame i)
            const Eigen::Vector3d& c = m_comJoint[i];
            const Eigen::Vector3d vc =
                vd[i] + wd[i].cross(c) + w[i].cross(w[i].cross(c));
            F[i] = m_inertia[i].mass * vc;
            const Eigen::Vector3d Iw = m_inertiaJoint[i] * w[i];
            N[i] = m_inertiaJoint[i] * wd[i] + w[i].cross(Iw);
        }

        // --- inward: forces & moments, joint torque projection ---
        Eigen::VectorXd tau(n);
        Eigen::Vector3d fNext = Eigen::Vector3d::Zero();
        Eigen::Vector3d nNext = Eigen::Vector3d::Zero();
        for (size_t i = n; i-- > 0;) {
            if (i + 1 < n) {
                // child joint i+1 quantities rotate into joint frame i
                // via G_{i+1} (R/p built above)
                f[i] = R[i + 1] * fNext;
                nc[i] = R[i + 1] * nNext +
                        p[i + 1].cross(R[i + 1] * fNext);
            } else {
                f[i].setZero();
                nc[i].setZero();
            }
            f[i] += F[i];
            nc[i] += N[i] + m_comJoint[i].cross(F[i]);
            // joint frames: origin lies on the axis — project directly
            const Eigen::Vector3d& a = m_chain.link(i).axis;
            tau[i] = m_chain.link(i).type == JointType::kRevolute
                         ? a.dot(nc[i])
                         : a.dot(f[i]);
            fNext = f[i];
            nNext = nc[i];
        }
        return tau;
    }
    Eigen::VectorXd inverseDynamics(const Eigen::VectorXd& q,
                                    const Eigen::VectorXd& qd,
                                    const Eigen::VectorXd& qdd) const {
        return inverseDynamics(q.data(), qd.data(), qdd.data());
    }

    /// Generalized gravity g(q) = RNEA(q, 0, 0).
    Eigen::VectorXd gravityVector(const Eigen::VectorXd& q) const {
        const Eigen::VectorXd z = Eigen::VectorXd::Zero(q.size());
        return inverseDynamics(q, z, z);
    }

    /// Coriolis/centrifugal term C(q,q̇)q̇ = RNEA(q,q̇,0) − g(q).
    Eigen::VectorXd coriolisVector(const Eigen::VectorXd& q,
                                   const Eigen::VectorXd& qd) const {
        const Eigen::VectorXd z = Eigen::VectorXd::Zero(q.size());
        return inverseDynamics(q, qd, z) - gravityVector(q);
    }

    /// Mass matrix M(q) via unit-acceleration columns (n RNEA calls).
    Eigen::MatrixXd massMatrix(const Eigen::VectorXd& q) const {
        const size_t n = m_chain.dof();
        const Eigen::VectorXd z = Eigen::VectorXd::Zero(n);
        Eigen::MatrixXd M(n, n);
        const Eigen::VectorXd g = gravityVector(q);
        Eigen::VectorXd e = Eigen::VectorXd::Zero(n);
        for (size_t i = 0; i < n; ++i) {
            e[i] = 1.0;
            M.col(i) = inverseDynamics(q, z, e) - g;
            e[i] = 0.0;
        }
        return M;
    }

    /// Forward dynamics: q̈ = M⁻¹(τ − C·q̇ − g).
    Eigen::VectorXd forwardDynamics(const Eigen::VectorXd& q,
                                    const Eigen::VectorXd& qd,
                                    const Eigen::VectorXd& tau) const {
        const Eigen::MatrixXd M = massMatrix(q);
        const Eigen::VectorXd rhs =
            tau - coriolisVector(q, qd) - gravityVector(q);
        return M.ldlt().solve(rhs);
    }

private:
    SerialChain m_chain;
    std::vector<LinkInertia> m_inertia;
    Eigen::Vector3d m_gravity;
    std::vector<Eigen::Vector3d> m_comJoint;      // com in joint frame
    std::vector<Eigen::Matrix3d> m_inertiaJoint;  // I about com, joint axes
};

} // namespace tether::kinematics

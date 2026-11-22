/**
 * @file SerialChain.hpp
 * @brief General serial-link chain with analytic Jacobian, fast IK, and
 *        spherical-wrist decomposition — the "shortcut" layer on top of
 *        the DH-table forward kinematics in ForwardKinematics.hpp.
 *
 * Representation: each link i carries
 *   - P_i : fixed placement of joint i in the parent LINK frame (SE3)
 *   - axis_i : joint axis expressed in the JOINT frame (unit vector)
 *   - type_i : revolute or prismatic
 *   - F_i : fixed placement of the link (inertial) frame in the joint
 *           frame (SE3) — the "body offset"
 *
 * Link transform:  T_i = T_{i-1} · P_i · A_i(q_i) · F_i
 * with A_i(q) = rotation about axis_i (revolute) or translation along
 * axis_i (prismatic). The chain's tool transform T_tool follows link n.
 *
 * A standard-DH chain converts via fromDH(): with
 *   T_DH,i = RotZ(θ+θ0) · TransZ(d) · TransX(a) · RotX(α)
 * we set P_i = TransZ(d)·RotZ(θ0), axis = ẑ, F_i = TransX(a)·RotX(α) so
 * that Π(P_i·A_i·F_i) reproduces the DH product exactly.
 *
 * All computations are double precision (the legacy float classes in
 * ForwardKinematics.hpp are kept untouched for API compatibility).
 */
#pragma once

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <array>
#include <cmath>
#include <optional>
#include <vector>

#include "ForwardKinematics.hpp" // DHParameters

namespace tether::kinematics {

/// Joint primitive type.
enum class JointType { kRevolute, kPrismatic };

/// One link of a SerialChain (see file-level documentation).
struct ChainLink {
    Eigen::Isometry3d jointPlacement{Eigen::Isometry3d::Identity()}; ///< P_i
    Eigen::Isometry3d bodyPlacement{Eigen::Isometry3d::Identity()};  ///< F_i
    Eigen::Vector3d axis{0.0, 0.0, 1.0};                             ///< joint-local
    JointType type{JointType::kRevolute};
};

/**
 * @brief Nullspace objective for redundant chains (dof > task
 * dimension): resolves the extra degrees of freedom by secondary
 * criteria instead of minimum-norm random walk. Three independent
 * terms, summed into one joint-space pull and projected through
 * N = I − J⁺J so the task solution is never perturbed:
 *
 * - `posture`: pull toward a fixed reference configuration. Right
 *   for anchored cyclic tasks where the redundancy should hold a
 *   designed branch (e.g. the Franka "elbow plane on the work
 *   direction" ready pose). Wrong for roaming tasks — it fights the
 *   natural branch adaptation.
 * - `lo`/`hi` + `limitGain`: joint-limit handling. Pull toward
 *   range midpoints weighted (1+4c²) by normalized offset c —
 *   gentle mid-centering (maximum freedom margin) that stiffens
 *   ×5 into a repulsion near the bounds. Direction-free, so it is
 *   the safe default for tasks whose pose roams.
 * - `shoulder`/`elbow`/`wrist` + `elbowDir` + `swivelGain`:
 *   swivel-angle regulation for 7-DOF arms — pulls the elbow's
 *   offset direction about the shoulder→wrist axis toward
 *   `elbowDir` (world). This is the geometric form of "elbow
 *   plane toward the task": e.g. `elbowDir=-z` keeps the elbow
 *   hanging below the shoulder–wrist line (plane contains the
 *   reach direction for wall work), `+z` arches it up for
 *   downward probing. Indices are jointTransform() link indices
 *   (FR3: 2,3,5). All three >= 0 enables.
 */
struct NullspaceRef {
    Eigen::VectorXd posture;      ///< fixed ref (empty = off)
    double postureGain = 0.0;
    Eigen::VectorXd lo, hi;       ///< joint ranges (empty = off)
    double limitGain = 0.0;
    int shoulder = -1, elbow = -1, wrist = -1;
    Eigen::Vector3d elbowDir{0.0, 0.0, -1.0};
    double swivelGain = 0.0;
};

/// Dynamic-size serial chain. All angles in radians, lengths in meters.
class SerialChain {
public:
    SerialChain() = default;
    explicit SerialChain(std::vector<ChainLink> links,
                         Eigen::Isometry3d tool =
                             Eigen::Isometry3d::Identity())
        : m_links(std::move(links)), m_tool(tool) {}

    /// Build from standard DH parameters (matches Serial6DOF exactly).
    static SerialChain fromDH(const DHParameters* dh, size_t n,
                              Eigen::Isometry3d tool =
                                  Eigen::Isometry3d::Identity()) {
        std::vector<ChainLink> links(n);
        for (size_t i = 0; i < n; ++i) {
            auto& L = links[i];
            L.jointPlacement = Eigen::Translation3d(0, 0, dh[i].d) *
                               Eigen::AngleAxisd(dh[i].theta_offset,
                                                 Eigen::Vector3d::UnitZ());
            L.bodyPlacement =
                Eigen::Translation3d(dh[i].a, 0, 0) *
                Eigen::AngleAxisd(dh[i].alpha, Eigen::Vector3d::UnitX());
            L.axis = Eigen::Vector3d::UnitZ();
            L.type = JointType::kRevolute;
        }
        return SerialChain(std::move(links), tool);
    }

    size_t dof() const { return m_links.size(); }
    const ChainLink& link(size_t i) const { return m_links[i]; }
    const Eigen::Isometry3d& tool() const { return m_tool; }
    void setTool(const Eigen::Isometry3d& t) { m_tool = t; }

    /// Joint motion transform A_i(q).
    static Eigen::Isometry3d jointMotion(const ChainLink& L, double q) {
        return L.type == JointType::kRevolute
                   ? Eigen::Isometry3d(Eigen::AngleAxisd(q, L.axis))
                   : Eigen::Isometry3d(Eigen::Translation3d(q * L.axis));
    }

    /// World transform of link frame i (after F_i). q has dof() entries.
    Eigen::Isometry3d linkTransform(const double* q, size_t i) const {
        Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
        for (size_t k = 0; k <= i; ++k)
            T = T * m_links[k].jointPlacement *
                jointMotion(m_links[k], q[k]) * m_links[k].bodyPlacement;
        return T;
    }

    /// World transform of the tool frame (end effector).
    Eigen::Isometry3d forwardKinematics(const double* q) const {
        return linkTransform(q, dof() - 1) * m_tool;
    }
    Eigen::Isometry3d forwardKinematics(
        const Eigen::VectorXd& q) const {
        return forwardKinematics(q.data());
    }

    /// All link transforms, T[0..n-1] (workhorse for Jacobian/RNEA).
    std::vector<Eigen::Isometry3d> linkTransforms(const double* q) const {
        std::vector<Eigen::Isometry3d> T(dof());
        Eigen::Isometry3d acc = Eigen::Isometry3d::Identity();
        for (size_t k = 0; k < dof(); ++k) {
            acc = acc * m_links[k].jointPlacement *
                  jointMotion(m_links[k], q[k]) *
                  m_links[k].bodyPlacement;
            T[k] = acc;
        }
        return T;
    }

    /// World transform of joint i's frame (before its own motion A_i).
    Eigen::Isometry3d jointTransform(const double* q, size_t i) const {
        Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
        for (size_t k = 0; k < i; ++k)
            T = T * m_links[k].jointPlacement *
                jointMotion(m_links[k], q[k]) * m_links[k].bodyPlacement;
        return T * m_links[i].jointPlacement;
    }

    /**
     * @brief Analytic geometric Jacobian (world frame, 6×n).
     *
     * Column i: revolute [z_i × (p_e − o_i); z_i], prismatic [z_i; 0]
     * where z_i is the joint axis in world and o_i the joint origin.
     * Replaces finite-difference Jacobians (~n FK evals per iteration).
     */
    Eigen::Matrix<double, 6, Eigen::Dynamic>
    jacobian(const double* q, const Eigen::Vector3d& pointWorld) const {
        const size_t n = dof();
        Eigen::Matrix<double, 6, Eigen::Dynamic> J(6, n);
        Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
        for (size_t i = 0; i < n; ++i) {
            const auto& L = m_links[i];
            const Eigen::Isometry3d Tj = T * L.jointPlacement;
            const Eigen::Vector3d z = Tj.linear() * L.axis;
            if (L.type == JointType::kRevolute) {
                J.template block<3, 1>(0, i) =
                    z.cross(pointWorld - Tj.translation());
                J.template block<3, 1>(3, i) = z;
            } else {
                J.template block<3, 1>(0, i) = z;
                J.template block<3, 1>(3, i).setZero();
            }
            T = T * L.jointPlacement * jointMotion(L, q[i]) *
                L.bodyPlacement;
        }
        return J;
    }
    /// Jacobian at the tool origin.
    Eigen::Matrix<double, 6, Eigen::Dynamic>
    jacobian(const double* q) const {
        return jacobian(q, forwardKinematics(q).translation());
    }
    Eigen::Matrix<double, 6, Eigen::Dynamic>
    jacobian(const Eigen::VectorXd& q) const {
        return jacobian(q.data());
    }

    /// Damping used for the nullspace projector N = I − J⁺J in the
    /// posture pulls. Deliberately tiny and independent of the solver
    /// damping: the projector is bounded (eigenvalues ∈ [0,1]) for any
    /// λ, and a small λ keeps the pull's task-space leak ~(λ/σ)² far
    /// below the IK tolerances (λ=1e-4 would leak ~1e-6 > tolPos).
    static constexpr double kNullDamp = 1e-8;

    /// Damped least-squares right pseudoinverse of a 6×n J.
    static Eigen::MatrixXd dampedPinv(
        const Eigen::Matrix<double, 6, Eigen::Dynamic>& J,
        double lambda) {
        const size_t n = (size_t)J.cols();
        Eigen::Matrix<double, 6, 6> JJt =
            J * J.transpose() +
            lambda * lambda * Eigen::Matrix<double, 6, 6>::Identity();
        return J.transpose() * JJt.ldlt().solve(
            Eigen::Matrix<double, 6, 6>::Identity());
    }

    /// Resolved-rate IK: q̇ = J⁺_λ ẋ (world-frame twist [v; ω]).
    /// `ns` adds the NullspaceRef pull projected through
    /// N = I − J⁺J (units 1/s × rad → rad/s) so redundant chains
    /// hold their objective at velocity level too instead of
    /// random-walking with the minimum-norm solution.
    Eigen::VectorXd resolvedRate(
        const double* q, const Eigen::Matrix<double, 6, 1>& xdot,
        double lambda = 1e-3,
        const NullspaceRef& ns = NullspaceRef()) const {
        const Eigen::Matrix<double, 6, Eigen::Dynamic> J = jacobian(q);
        const Eigen::MatrixXd Jpinv = dampedPinv(J, lambda);
        Eigen::VectorXd qdot = Jpinv * xdot;
        const size_t n = dof();
        const Eigen::VectorXd dp = nullspacePull(q, ns);
        if (dp.norm() > 0.0) {
            const Eigen::MatrixXd N =
                Eigen::MatrixXd::Identity(n, n) -
                dampedPinv(J, kNullDamp) * J;
            qdot += N * dp;
        }
        return qdot;
    }
    Eigen::VectorXd resolvedRate(
        const Eigen::VectorXd& q,
        const Eigen::Matrix<double, 6, 1>& xdot,
        double lambda = 1e-3,
        const NullspaceRef& ns = NullspaceRef()) const {
        return resolvedRate(q.data(), xdot, lambda, ns);
    }

    struct IKOptions {
        int maxIter{150};       ///< DLS iterations
        double tolPos{1e-8};    ///< position residual [m]
        double tolOri{1e-7};    ///< orientation residual [rad]
        double lambda{1e-4};    ///< Levenberg damping
        double maxStep{0.5};    ///< per-joint step clamp [rad|m]
        /// Convergence also waits for the NullspaceRef pull to settle
        /// (|N·dp|∞ < 1e-3): right for one-shot solves where the
        /// redundant DOF should be resolved now. Per-cycle replanners
        /// should pass false — the pull then acts as a bounded nudge
        /// per call and the DOF relaxes toward the objective over
        /// many cycles without disturbing the task.
        bool nsSettle{true};
    };

    /**
     * @brief Iterative 6-DOF pose IK (DLS over the analytic Jacobian).
     *
     * Error: position difference + world-frame rotation-vector
     * (angle-axis of R_curᵀ·R_tgt mapped through R_cur). Warm-start via
     * q (in/out); returns residual norm and success flag.
     *
     * `ns` (optional) resolves the redundant DOF of chains with
     * dof>6: the NullspaceRef pull projected through N = I − J⁺J is
     * added to the DLS step each iteration. Without it the extra DOF
     * random-walks with the minimum-norm steps and the arm can drift
     * into a folded/elbow-in branch (low manipulability, branch-jump
     * transients).
     */
    bool poseIK(const Eigen::Isometry3d& target, Eigen::VectorXd& q,
                double* residual = nullptr) const {
        return poseIK(target, q, IKOptions(), residual);
    }
    bool poseIK(const Eigen::Isometry3d& target, Eigen::VectorXd& q,
                const IKOptions& opt, double* residual = nullptr,
                const NullspaceRef& ns = NullspaceRef()) const {
        const size_t n = dof();
        for (int it = 0; it < opt.maxIter; ++it) {
            const Eigen::Isometry3d cur = forwardKinematics(q);
            Eigen::Matrix<double, 6, 1> e;
            e.head<3>() = target.translation() - cur.translation();
            const Eigen::AngleAxisd aa(cur.rotation().transpose() *
                                       target.rotation());
            e.tail<3>() = cur.rotation() * (aa.axis() * aa.angle());
            const double rn = e.norm();
            const Eigen::MatrixXd J = jacobian(q);
            const Eigen::MatrixXd Jpinv = dampedPinv(J, opt.lambda);
            Eigen::VectorXd dq = Jpinv * e;
            Eigen::VectorXd dns = Eigen::VectorXd::Zero(n);
            const Eigen::VectorXd dp = nullspacePull(q.data(), ns);
            if (dp.norm() > 0.0) {
                const Eigen::MatrixXd N =
                    Eigen::MatrixXd::Identity(n, n) -
                    dampedPinv(J, kNullDamp) * J;
                dns = N * dp;
                // The pull shares the maxStep budget with the
                // Cartesian correction; cap it so a large pull
                // (unreachable swivel direction, ×5 limit repulsion)
                // can't starve convergence.
                const double nsStep = dns.cwiseAbs().maxCoeff();
                if (nsStep > 0.5 * opt.maxStep)
                    dns *= 0.5 * opt.maxStep / nsStep;
            }
            // Convergence needs the task AND (with nsSettle) a settled
            // pull: exiting as soon as e < tol would strand a one-shot
            // secondary objective wherever the first few steps left
            // it. A pull at its optimum — or balanced against a joint
            // limit — decays below this threshold; an unreachable one
            // never does and falls through to the DLS polish below.
            if (e.head<3>().norm() < opt.tolPos &&
                e.tail<3>().norm() < opt.tolOri &&
                (!opt.nsSettle ||
                 dns.cwiseAbs().maxCoeff() < 1e-3)) {
                if (residual) *residual = rn;
                return true;
            }
            dq += dns;
            const double step = dq.cwiseAbs().maxCoeff();
            if (step > opt.maxStep) dq *= opt.maxStep / step;
            q += dq;
            for (Eigen::Index i = 0; i < q.size(); ++i)
                if (m_links[i].type == JointType::kRevolute)
                    q[i] = std::remainder(q[i], 2.0 * M_PI);
        }
        if (residual)
            *residual = (forwardKinematics(q).translation() -
                         target.translation())
                            .norm();
        // Polish: an undecayed pull (unreachable swivel direction or
        // limit-vs-objective equilibrium) injects ~(dq_ns)²/radius of
        // task-space error per step and blocks the strict tolerance
        // above. Finish with pure DLS — it removes the residual
        // without undoing the nullspace offset the pull achieved.
        const bool pulled = ns.postureGain != 0.0 ||
                            ns.limitGain != 0.0 || ns.swivelGain != 0.0;
        for (int it = 0; pulled && it < 40; ++it) {
            const Eigen::Isometry3d cur = forwardKinematics(q);
            Eigen::Matrix<double, 6, 1> e;
            e.head<3>() = target.translation() - cur.translation();
            const Eigen::AngleAxisd aa(cur.rotation().transpose() *
                                       target.rotation());
            e.tail<3>() = cur.rotation() * (aa.axis() * aa.angle());
            if (e.head<3>().norm() < opt.tolPos &&
                e.tail<3>().norm() < opt.tolOri) {
                if (residual) *residual = e.norm();
                return true;
            }
            const Eigen::MatrixXd J = jacobian(q);
            Eigen::VectorXd dq = dampedPinv(J, opt.lambda) * e;
            const double step = dq.cwiseAbs().maxCoeff();
            if (step > opt.maxStep) dq *= opt.maxStep / step;
            q += dq;
            for (Eigen::Index i = 0; i < q.size(); ++i)
                if (m_links[i].type == JointType::kRevolute)
                    q[i] = std::remainder(q[i], 2.0 * M_PI);
        }
        if (residual)
            *residual = (forwardKinematics(q).translation() -
                         target.translation())
                            .norm();
        return false;
    }

    /**
     * @brief Position-only IK (3-DOF task on an N-DOF chain).
     *
     * DLS on the translational rows of the analytic Jacobian plus a
     * nullspace pull (NullspaceRef `ns`; a zero joint vector is used
     * when it carries no posture). The nullspace term both regularizes
     * redundant chains and resolves the extra DOF deterministically.
     * Warm-start via q (in/out).
     */
    bool positionIK(const Eigen::Vector3d& target, Eigen::VectorXd& q,
                    const IKOptions& opt, double* residual = nullptr,
                    const NullspaceRef& ns = NullspaceRef()) const {
        const size_t n = dof();
        const bool nsActive =
            ns.posture.size() == (Eigen::Index)n ||
            ns.limitGain != 0.0 || ns.swivelGain != 0.0;
        for (int it = 0; it < opt.maxIter; ++it) {
            const Eigen::Isometry3d cur = forwardKinematics(q);
            const Eigen::Vector3d e =
                target - cur.translation();
            if (!nsActive && e.norm() < opt.tolPos) {
                if (residual) *residual = e.norm();
                return true;
            }
            // Adaptive damping: shrink λ with the residual so the
            // DLS floor (≈λ²) doesn't sit just above tolPos.
            const double lam =
                opt.lambda * std::clamp(e.norm(), 1e-4, 1.0);
            const Eigen::MatrixXd Jp = jacobian(q).topRows(3);
            // DLS pseudoinverse of the 3×n position Jacobian
            const Eigen::Matrix3d JJt =
                Jp * Jp.transpose() +
                lam * lam * Eigen::Matrix3d::Identity();
            const Eigen::MatrixXd Jpinv =
                Jp.transpose() * JJt.ldlt()
                                     .solve(Eigen::Matrix3d::Identity());
            // Projector with its own tiny damping (see kNullDamp):
            // reusing the solver pinv would leak ~(lam/σ)² of the
            // posture pull into the task direction.
            const Eigen::Matrix3d JJtN =
                Jp * Jp.transpose() +
                kNullDamp * kNullDamp * Eigen::Matrix3d::Identity();
            const Eigen::MatrixXd N =
                Eigen::MatrixXd::Identity(n, n) -
                Jp.transpose() *
                    JJtN.ldlt().solve(Eigen::Matrix3d::Identity()) *
                    Jp;
            // Nullspace pull: the NullspaceRef terms, or the historic
            // default zero-pull regularizer when `ns` carries no
            // posture (keeps the solve deterministic on redundant
            // chains — the legacy default).
            Eigen::VectorXd dq = Jpinv * e;
            if (nsActive) {
                NullspaceRef eff = ns;
                if (eff.posture.size() != (Eigen::Index)n &&
                    eff.postureGain != 0.0)
                    eff.posture = Eigen::VectorXd::Zero(n);
                Eigen::VectorXd dns =
                    N * nullspacePull(q.data(), eff);
                // Convergence needs task AND settled pull — see
                // poseIK. The pull shares the maxStep budget with
                // the Cartesian correction.
                const double nsStep = dns.cwiseAbs().maxCoeff();
                if (e.norm() < opt.tolPos &&
                    (!opt.nsSettle || nsStep < 1e-3)) {
                    if (residual) *residual = e.norm();
                    return true;
                }
                if (nsStep > 0.5 * opt.maxStep)
                    dns *= 0.5 * opt.maxStep / nsStep;
                dq += dns;
            } else {
                Eigen::VectorXd dp = -q;
                for (Eigen::Index i = 0; i < dp.size(); ++i)
                    if (m_links[i].type == JointType::kRevolute)
                        dp[i] = std::remainder(dp[i], 2.0 * M_PI);
                dq += 0.2 * (N * dp);
            }
            const double step = dq.cwiseAbs().maxCoeff();
            if (step > opt.maxStep) dq *= opt.maxStep / step;
            q += dq;
            for (Eigen::Index i = 0; i < q.size(); ++i)
                if (m_links[i].type == JointType::kRevolute)
                    q[i] = std::remainder(q[i], 2.0 * M_PI);
        }
        if (residual)
            *residual = (forwardKinematics(q).translation() - target)
                            .norm();
        return false;
    }

    // ------------------------------------------------------------------
    // Spherical-wrist analytic IK (6-DOF, axes 2∥3, axes 4/5/6 concurrent)
    // ------------------------------------------------------------------

    /// Result of analyticIK(): up to 8 solutions in `solutions`.
    struct AnalyticIKSolution {
        std::vector<Eigen::VectorXd> solutions;
        double residual{0.0}; ///< worst FK error across solutions [m]
    };

    /**
     * @brief True iff the chain matches the decomposable structure:
     * 6 revolute joints, axes 2 ∥ 3 (the planar arm), and axes 4,5,6
     * concurrent at a single point (spherical wrist). Axis 1 must be
     * perpendicular to the arm plane normal (standard anthropomorphic
     * geometry — UR5, Kuka KR6, ...).
     */
    bool hasAnalyticIK(double eps = 1e-6) const {
        if (dof() != 6) return false;
        for (const auto& L : m_links)
            if (L.type != JointType::kRevolute) return false;
        Eigen::VectorXd q0 = Eigen::VectorXd::Zero(6);
        // axes 2 ∥ 3 in world
        const Eigen::Vector3d a2 = jointTransform(q0.data(), 1).linear() *
                                   m_links[1].axis;
        const Eigen::Vector3d a3 = jointTransform(q0.data(), 2).linear() *
                                   m_links[2].axis;
        if (a2.cross(a3).norm() > eps) return false;
        // wrist concurrency: pairwise axis intersection points agree
        const auto axP = [&](const Eigen::VectorXd& q, size_t i,
                             Eigen::Vector3d& p, Eigen::Vector3d& z) {
            const Eigen::Isometry3d Tj = jointTransform(q.data(), i);
            p = Tj.translation();
            z = (Tj.linear() * m_links[i].axis).normalized();
        };
        // wrist concurrency: the intersection points of axis pairs
        // (4,5) and (5,6) must coincide — evaluated at non-degenerate
        // configurations (two wrist axes can be parallel at q=0).
        const auto closest = [](const Eigen::Vector3d& p1,
                                const Eigen::Vector3d& z1,
                                const Eigen::Vector3d& p2,
                                const Eigen::Vector3d& z2,
                                Eigen::Vector3d& mid, double& dev) {
            const Eigen::Vector3d n = z1.cross(z2);
            if (n.norm() < 1e-3) return false; // parallel — no unique point
            const Eigen::Vector3d d = p2 - p1;
            const double t1 = d.dot(z2.cross(n)) / n.squaredNorm();
            const double t2 = d.dot(z1.cross(n)) / n.squaredNorm();
            mid = 0.5 * (p1 + t1 * z1 + p2 + t2 * z2);
            dev = (p1 + t1 * z1 - (p2 + t2 * z2)).norm();
            return true;
        };
        bool concurrent = false;
        for (double seed : {0.35, -0.7, 1.1}) {
            Eigen::VectorXd qc = Eigen::VectorXd::Constant(6, seed);
            Eigen::Vector3d p4, z4, p5, z5, p6, z6;
            axP(qc, 3, p4, z4);
            axP(qc, 4, p5, z5);
            axP(qc, 5, p6, z6);
            Eigen::Vector3d w45, w56;
            double dev45, dev56;
            if (!closest(p4, z4, p5, z5, w45, dev45)) continue;
            if (!closest(p5, z5, p6, z6, w56, dev56)) continue;
            if (dev45 > eps || dev56 > eps) return false; // skew axes
            if ((w45 - w56).norm() > 1e-5) return false;  // not one point
            concurrent = true;
            break;
        }
        if (!concurrent) return false;
        // axis 1 ⊥ arm-plane normal (axis2): shoulder pivot resolvable
        const Eigen::Vector3d a1 =
            m_links[0].jointPlacement.linear() * m_links[0].axis;
        if (std::abs(a1.normalized().dot(a2.normalized())) > 1e-3)
            return false;
        return true;
    }

    /**
     * @brief Wrist-center point: the concurrency point of axes 4,5,6 in
     * world at configuration q (only meaningful if hasAnalyticIK()).
     * Computed as the midpoint of the closest approach of axes 5 and 6.
     */
    Eigen::Vector3d wristCenter(const Eigen::VectorXd& q) const {
        const Eigen::Isometry3d T5 = jointTransform(q.data(), 4);
        const Eigen::Isometry3d T6 = jointTransform(q.data(), 5);
        const Eigen::Vector3d p1 = T5.translation();
        const Eigen::Vector3d z1 =
            (T5.linear() * m_links[4].axis).normalized();
        const Eigen::Vector3d p2 = T6.translation();
        const Eigen::Vector3d z2 =
            (T6.linear() * m_links[5].axis).normalized();
        // Closest-point parameters on the two axis lines:
        //   t_i = d·(z_j×n)/|n|²  with n = z1×z2 (unnormalized).
        const Eigen::Vector3d n = z1.cross(z2);
        const Eigen::Vector3d d = p2 - p1;
        const double t1 = d.dot(z2.cross(n)) / n.squaredNorm();
        const double t2 = d.dot(z1.cross(n)) / n.squaredNorm();
        return 0.5 * ((p1 + t1 * z1) + (p2 + t2 * z2));
    }

    /**
     * @brief Analytic IK by arm/wrist decomposition.
     *
     * Position part (fully analytic): wrist center wc = p_ee − R_ee·v_wc
     * (v_wc constant in tool frame). θ1 from the shoulder-plane offset,
     * (θ2,θ3) from the planar 2R triangle. Orientation part: R36 =
     * R03ᵀ·R_ee is decomposed for (θ4,θ5,θ6) by a seeded 3-variable
     * Newton polish on the world-frame wrist Jacobian — deterministic,
     * exact at convergence, and generic over wrist conventions.
     *
     * @returns solutions with FK(q) == pose up to tolPos/tolOri.
     */
    AnalyticIKSolution analyticIK(const Eigen::Isometry3d& pose) const {
        return analyticIK(pose, IKOptions());
    }
    AnalyticIKSolution analyticIK(
        const Eigen::Isometry3d& pose, const IKOptions& opt) const {
        AnalyticIKSolution out;
        if (!hasAnalyticIK()) return out;

        const Eigen::VectorXd q0 = Eigen::VectorXd::Zero(6);
        // Constants measured once at q=0:
        const Eigen::Isometry3d T_tool0 = forwardKinematics(q0);
        const Eigen::Vector3d w0 = wristCenter(q0);
        // v_wc: wrist center in tool frame (constant on axes 4,5,6)
        const Eigen::Vector3d v_wc =
            T_tool0.rotation().transpose() * (w0 - T_tool0.translation());
        // w in link-3 frame (constant — wrist center rides on frame 3)
        const Eigen::Isometry3d T30 = linkTransform(q0.data(), 2);
        const Eigen::Vector3d w3 =
            T30.rotation().transpose() * (w0 - T30.translation());
        // axis-2 direction expressed in frame 3 (constant)
        const Eigen::Vector3d a2w =
            (jointTransform(q0.data(), 1).linear() * m_links[1].axis)
                .normalized();
        const Eigen::Vector3d a2_in3 =
            T30.rotation().transpose() * a2w;
        // axis 1 direction/origin in world (constant)
        const Eigen::Isometry3d Tj1 = m_links[0].jointPlacement;
        const Eigen::Vector3d z1 =
            (Tj1.linear() * m_links[0].axis).normalized();
        const Eigen::Vector3d o1 = Tj1.translation();

        // Joint-3 axis in link-3 coordinates: the line passes through
        // the joint origin o3f = F3⁻¹·0 (offset by the link transform
        // — NOT the link origin) along the axis direction.
        const Eigen::Vector3d o3f =
            m_links[2].bodyPlacement.inverse().translation();
        const Eigen::Vector3d w3rel = w3 - o3f;
        // Signed offset of w from the shoulder plane (const under 2,3)
        const double s = w3rel.dot(a2_in3);
        // In-plane distance from joint-3 axis to w (const under 2,3)
        const double rho3 = (w3rel - s * a2_in3).norm();

        // Desired wrist center: W = t_ee + R_ee·v_wc
        const Eigen::Vector3d wc =
            pose.translation() + pose.rotation() * v_wc;
        const Eigen::Vector3d wrel = wc - o1;
        const double h = wrel.dot(z1);
        const Eigen::Vector3d rvec = wrel - h * z1;
        const double r = rvec.norm();
        if (r < std::abs(s) + 1e-9) return out; // unreachable offset

        const double phi = std::atan2(rvec.y(), rvec.x());
        // θ1 candidates: arm plane through z1 rotated so w's
        // perpendicular offset equals s.  The signed offset enters as
        // sin(θ1−φ) = s/r; generate all four sign/mirror branches —
        // wrong ones are rejected cheaply by the reach test below.
        std::vector<double> theta1;
        {
            const double sn =
                std::clamp(s / r, -1.0, 1.0);
            const double a = std::asin(sn);
            theta1.push_back(phi + a);
            theta1.push_back(phi - a);
            theta1.push_back(phi + M_PI - a);
            theta1.push_back(phi - M_PI + a);
        }

        for (double t1 : theta1) {
            // Shoulder-plane solve at this θ1 (θ2=θ3=0 reference).
            Eigen::VectorXd qr = q0;
            qr[0] = t1;
            const Eigen::Isometry3d Tj2 = jointTransform(qr.data(), 1);
            const Eigen::Vector3d o2 = Tj2.translation();
            const Eigen::Vector3d n2 =
                (Tj2.linear() * m_links[1].axis).normalized();
            const Eigen::Isometry3d Tj3r = jointTransform(qr.data(), 2);
            const Eigen::Vector3d o3r = Tj3r.translation();
            // L2: shoulder→elbow distance in the plane ⊥ n2.
            const double L2 =
                (o3r - o2 - ((o3r - o2).dot(n2)) * n2).norm();
            if (L2 < 1e-9) continue;
            // In-plane wrist target: w projected ⊥ n2 into the plane
            // through o2. The elbow point o3 moves on a circle of
            // radius L2 about o2; |w−o3|in-plane = rho3 (constant).
            const Eigen::Vector3d w2d =
                wc - ((wc - o2).dot(n2)) * n2;
            const double dp = (w2d - o2).norm();
            if (dp < 1e-9) continue; // singular: wrist center on shoulder
            const Eigen::Vector3d uRef =
                (o3r - o2 - ((o3r - o2).dot(n2)) * n2).normalized();

            if (rho3 < 1e-9) {
                // Degenerate wrist center on the elbow axis (UR-style
                // wrist with no out-of-plane offset): W's in-plane
                // position equals o3's, so dp must equal L2 exactly and
                // θ3 is free — recovered by the orientation polish.
                if (std::abs(dp - L2) > 1e-6) continue;
                const Eigen::Vector3d u2 = (w2d - o2).normalized();
                const double t2 = signedAngle(uRef, u2, n2);
                Eigen::VectorXd qArm = qr;
                qArm[1] = t2;
                if ((wristCenter(qArm) - wc).norm() > 1e-6) continue;
                polishOrientation(pose, qArm, 2, opt, out.solutions);
                continue;
            }

            if (dp > L2 + rho3 + 1e-9 || dp < std::abs(L2 - rho3) - 1e-9)
                continue;

            // 2D solve in plane basis (u_w, e⊥ = n2×u_w), o2 at origin:
            //   o3 = x·u_w + y·e⊥,  x = (L2²+dp²−ρ3²)/(2dp), y = ±√(L2²−x²)
            const Eigen::Vector3d uW = (w2d - o2).normalized();
            const Eigen::Vector3d ePerp = n2.cross(uW);
            const double x = (L2 * L2 + dp * dp - rho3 * rho3) /
                             (2.0 * dp);
            const double y2 = L2 * L2 - x * x;
            if (y2 < -1e-9) continue;
            const double y = std::sqrt(std::max(0.0, y2));
            for (double sg : {1.0, -1.0}) {
                const Eigen::Vector3d o3 = o2 + x * uW + sg * y * ePerp;
                const Eigen::Vector3d u2 = (o3 - o2).normalized();
                const Eigen::Vector3d uB = (w2d - o3).normalized();
                // θ2: signed angle from the reference (o3r−o2) direction
                const double t2 = signedAngle(uRef, u2, n2);
                // θ3: signed angle from (o3−o2) to (w−o3), minus the
                // same angle measured at the reference (θ3 = 0).
                const Eigen::Vector3d wRef = wristCenter(qr);
                const Eigen::Vector3d uBr =
                    (wRef - o3r - ((wRef - o3r).dot(n2)) * n2)
                        .normalized();
                const double d0 = signedAngle(uRef, uBr, n2);
                const double t3 =
                    signedAngle(u2, uB, n2) - d0;

                Eigen::VectorXd qArm = qr;
                qArm[1] = t2;
                qArm[2] = t3;
                // Verify the arm solve actually places the wrist center.
                if ((wristCenter(qArm) - wc).norm() > 1e-6) continue;
                polishOrientation(pose, qArm, 3, opt, out.solutions);
            }
        }
        // keep unique solutions, measure worst residual
        dedup(out.solutions);
        for (const auto& qs : out.solutions) {
            const Eigen::Isometry3d f = forwardKinematics(qs);
            const double rr = (f.translation() - pose.translation()).norm();
            out.residual = std::max(out.residual, rr);
        }
        return out;
    }

private:
    /**
     * Joint-space pull assembled from a NullspaceRef. Callers project
     * it through N = I − J⁺J, so every term only ever moves the
     * redundant directions. Units: joint deltas per gain (the IK
     * applies them per step, resolvedRate per second).
     */
    Eigen::VectorXd nullspacePull(const double* q,
                                  const NullspaceRef& ns) const {
        const size_t n = dof();
        Eigen::VectorXd dp = Eigen::VectorXd::Zero(n);
        if (ns.postureGain != 0.0 &&
            ns.posture.size() == (Eigen::Index)n) {
            for (size_t i = 0; i < n; ++i) {
                double d = ns.posture[i] - q[i];
                if (m_links[i].type == JointType::kRevolute)
                    d = std::remainder(d, 2.0 * M_PI);
                dp[i] += ns.postureGain * d;
            }
        }
        if (ns.limitGain != 0.0 &&
            ns.lo.size() == (Eigen::Index)n &&
            ns.hi.size() == (Eigen::Index)n) {
            for (size_t i = 0; i < n; ++i) {
                const double half = 0.5 * (ns.hi[i] - ns.lo[i]);
                if (half < 1e-9) continue;
                const double mid = 0.5 * (ns.hi[i] + ns.lo[i]);
                const double c =
                    std::clamp((q[i] - mid) / half, -1.0, 1.0);
                // Mid-center pull ×(1+4c²): gentle centering far from
                // the bounds, ×5 repulsion at them.
                dp[i] += ns.limitGain * (mid - q[i]) * (1.0 + 4 * c * c);
            }
        }
        if (ns.swivelGain != 0.0 && ns.shoulder >= 0 && ns.elbow >= 0 &&
            ns.wrist >= 0 && ns.shoulder < (int)n &&
            ns.elbow < (int)n && ns.wrist < (int)n) {
            const Eigen::Vector3d S =
                jointTransform(q, (size_t)ns.shoulder).translation();
            const Eigen::Vector3d E =
                jointTransform(q, (size_t)ns.elbow).translation();
            const Eigen::Vector3d W =
                jointTransform(q, (size_t)ns.wrist).translation();
            const Eigen::Vector3d sw = W - S;
            const double swLen = sw.norm();
            if (swLen > 1e-6) {
                const Eigen::Vector3d a = sw / swLen;
                // Elbow offset off the shoulder→wrist axis, and the
                // same-axis projection of the preferred direction.
                const Eigen::Vector3d r = (E - S) - a * a.dot(E - S);
                const Eigen::Vector3d rd =
                    ns.elbowDir - a * a.dot(ns.elbowDir);
                const double rr = r.norm(), rdr = rd.norm();
                if (rr > 1e-4 && rdr > 1e-4) {
                    // Swivel-angle error about the SW axis and the
                    // orbit-tangent elbow correction (t̂·eψ·|r| — the
                    // pull must run around the axis, not through it:
                    // the straight-line pull projects to ~0 in the
                    // nullspace for antipodal offsets).
                    const Eigen::Vector3d rh = r / rr;
                    const Eigen::Vector3d rdh = rd / rdr;
                    const double ePsi = std::atan2(
                        rh.cross(rdh).dot(a), rh.dot(rdh));
                    const Eigen::Vector3d vE =
                        ePsi * rr * a.cross(rh);
                    // Elbow-point Jacobian; columns ≥ elbow are zeroed
                    // because the hinge and downstream joints cannot
                    // move their own origin.
                    Eigen::MatrixXd Je = jacobian(q, E).topRows(3);
                    Je.rightCols(n - (size_t)ns.elbow).setZero();
                    dp += ns.swivelGain * (Je.transpose() * vE);
                }
            }
        }
        return dp;
    }

    static double signedAngle(const Eigen::Vector3d& a,
                              const Eigen::Vector3d& b,
                              const Eigen::Vector3d& n) {
        return std::atan2(a.cross(b).dot(n), a.dot(b));
    }

    /**
     * Damped-Newton polish for the orientation solve. Free joints are
     * indices firstFree..5 (θ4..θ6 normally; θ3..θ6 for the degenerate
     * wrist-center-on-elbow-axis case, which is underdetermined —
     * hence the damped pseudoinverse). Orientation residual e =
     * rotation vector of R_curᵀR_tgt mapped to world; Jacobian column
     * k = world axis of free joint k. A small deterministic seed grid
     * covers the usual multiple wrist solutions.
     */
    void polishOrientation(const Eigen::Isometry3d& pose,
                           const Eigen::VectorXd& qArm, size_t firstFree,
                           const IKOptions& opt,
                           std::vector<Eigen::VectorXd>& sols) const {
        static const double seeds[] = {0.0, M_PI_2, -M_PI_2, M_PI};
        const int nf = static_cast<int>(6 - firstFree);
        const int nCombo = 1 << (2 * nf); // 4^nf seed combinations
        for (int combo = 0; combo < nCombo; ++combo) {
            Eigen::VectorXd q = qArm;
            for (int k = 0, c = combo; k < nf; ++k, c >>= 2)
                q[firstFree + k] = seeds[c & 3];
            bool conv = false;
            for (int it = 0; it < 60; ++it) {
                const Eigen::Isometry3d cur = forwardKinematics(q);
                const Eigen::AngleAxisd aa(cur.rotation().transpose() *
                                           pose.rotation());
                Eigen::Vector3d e =
                    cur.rotation() * (aa.axis() * aa.angle());
                if (e.norm() < opt.tolOri) {
                    conv = true;
                    break;
                }
                // wrist Jacobian: world axes of the free joints (3×nf)
                Eigen::MatrixXd Jw(3, nf);
                for (int k = 0; k < nf; ++k) {
                    const Eigen::Isometry3d Tj =
                        jointTransform(q.data(), firstFree + k);
                    Jw.col(k) = Tj.linear() * m_links[firstFree + k].axis;
                }
                // damped pseudoinverse step dq = Jᵀ(JJᵀ+λ²I)⁻¹e
                const Eigen::Matrix3d JJt =
                    Jw * Jw.transpose() +
                    opt.lambda * opt.lambda * Eigen::Matrix3d::Identity();
                Eigen::VectorXd dq =
                    Jw.transpose() * JJt.ldlt().solve(e);
                if (!dq.allFinite()) break;
                const double mx = dq.cwiseAbs().maxCoeff();
                if (mx > 0.5) dq *= 0.5 / mx;
                q.tail(nf) += dq;
            }
            if (conv) {
                for (int k = 0; k < 6; ++k)
                    q[k] = std::remainder(q[k], 2.0 * M_PI);
                sols.push_back(q);
            }
        }
    }

    static void dedup(std::vector<Eigen::VectorXd>& sols,
                      double tol = 1e-5) {
        std::vector<Eigen::VectorXd> uniq;
        for (auto& s : sols) {
            bool dup = false;
            for (auto& u : uniq)
                if ((s - u).cwiseAbs().maxCoeff() < tol) {
                    dup = true;
                    break;
                }
            if (!dup) uniq.push_back(s);
        }
        sols = std::move(uniq);
    }

    std::vector<ChainLink> m_links;
    Eigen::Isometry3d m_tool{Eigen::Isometry3d::Identity()};
};

} // namespace tether::kinematics

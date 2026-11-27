/**
 * @file test_kinematics_pinocchio.cpp
 * @brief Integration test: SerialChain + RNEA against Pinocchio.
 *
 * Cross-validates the analytic shortcuts (Jacobian, RNEA inverse
 * dynamics, mass matrix, gravity, IK) against Pinocchio 4.x on
 * identical models:
 *   - UR5 built from the repo's DH table → programmatic pinocchio model
 *   - Panda loaded from example-robot-data URDF → SerialChain built
 *     from pinocchio's extracted joint placements/axes/inertias
 *
 * Test-only dependency — skipped entirely when pinocchio is absent.
 */
#include <gtest/gtest.h>

#include <pinocchio/multibody/model.hpp>
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/rnea.hpp>
#include <pinocchio/algorithm/crba.hpp>
#include <pinocchio/algorithm/model.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/parsers/urdf.hpp>

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <cmath>
#include <random>
#include <string>
#include <vector>

#include "tether/kinematics/SerialChain.hpp"
#include "tether/kinematics/RNEA.hpp"
#include "tether/kinematics/ForwardKinematics.hpp"

using namespace tether::kinematics;
namespace pin = pinocchio;

namespace {

std::mt19937 rng(42);

pin::SE3 toPin(const Eigen::Isometry3d& T) {
    return pin::SE3(T.linear(), T.translation());
}
Eigen::Isometry3d toIso(const pin::SE3& s) {
    Eigen::Isometry3d T;
    T.linear() = s.rotation();
    T.translation() = s.translation();
    return T;
}

Eigen::VectorXd randQ(int n, double lo = -2.5, double hi = 2.5) {
    std::uniform_real_distribution<double> d(lo, hi);
    Eigen::VectorXd q(n);
    for (int i = 0; i < n; ++i) q[i] = d(rng);
    return q;
}

/// UR5 DH table (matches Serial6DOF::setUR5Parameters()).
void ur5DH(DHParameters* dh) {
    dh[0] = {0.f, 0.089159f, 0.f, (float)M_PI_2};
    dh[1] = {0.f, 0.f, -0.42500f, 0.f};
    dh[2] = {0.f, 0.f, -0.39225f, 0.f};
    dh[3] = {0.f, 0.10915f, 0.f, (float)M_PI_2};
    dh[4] = {0.f, 0.09465f, 0.f, (float)-M_PI_2};
    dh[5] = {0.f, 0.0823f, 0.f, 0.f};
}

/// Build a pinocchio model mirroring a SerialChain exactly.
/// Joint i: revolute about local z if axis==z else generic via the
/// axis vector — we only support revolute about x/y/z here (enough for
/// DH chains and URDF-extracted joints).
pin::Model buildModel(const SerialChain& chain,
                      const std::vector<LinkInertia>* inertias) {
    pin::Model model;
    pin::JointIndex parent = 0;
    Eigen::Isometry3d prevBody = Eigen::Isometry3d::Identity();
    for (size_t i = 0; i < chain.dof(); ++i) {
        const auto& L = chain.link(i);
        const Eigen::Isometry3d place = prevBody * L.jointPlacement;
        const Eigen::Vector3d a = L.axis;
        pin::JointIndex jid;
        const std::string name = "j" + std::to_string(i);
        if ((a - Eigen::Vector3d::UnitZ()).norm() < 1e-12)
            jid = model.addJoint(parent, pin::JointModel(pin::JointModelRZ()),
                                 toPin(place), name);
        else if ((a - Eigen::Vector3d::UnitY()).norm() < 1e-12)
            jid = model.addJoint(parent, pin::JointModel(pin::JointModelRY()),
                                 toPin(place), name);
        else if ((a - Eigen::Vector3d::UnitX()).norm() < 1e-12)
            jid = model.addJoint(parent, pin::JointModel(pin::JointModelRX()),
                                 toPin(place), name);
        else
            jid = model.addJoint(
                parent, pin::JointModel(pin::JointModelRUBX()),
                toPin(place), name);
        if (inertias) {
            const auto& I = (*inertias)[i];
            model.appendBodyToJoint(
                jid, pin::Inertia(I.mass, I.com, I.inertia),
                toPin(L.bodyPlacement));
        }
        parent = jid;
        prevBody = L.bodyPlacement; // next joint sits in link frame i
    }
    return model;
}

/// Extract a SerialChain (joint placements, axes, inertias) from a
/// pinocchio model. Joint frames == link frames (URDF-style).
struct Extracted {
    SerialChain chain;
    std::vector<LinkInertia> inertias;
    std::vector<pin::JointIndex> jids;
};

Extracted extractChain(const pin::Model& model, pin::Data& data) {
    Extracted ex;
    std::vector<ChainLink> links;
    for (pin::JointIndex jid = 1; jid < model.njoints; ++jid) {
        if (model.joints[jid].nq() == 0) continue; // fixed
        const std::string& jn = model.names[jid];
        // Arm joints only: panda.urdf adds movable finger/hand joints.
        if (jn.find("finger") != std::string::npos ||
            jn.find("hand") != std::string::npos ||
            jn.find("tcp") != std::string::npos)
            continue;
        ChainLink L;
        L.jointPlacement = toIso(model.jointPlacements[jid]);
        // axis in joint frame: angular column of the joint motion
        // subspace S (6×nv) at q=0 — compute via the world Jacobian:
        //   z_world = J_ang[:, v_idx]; axis = R_jiᵀ z_world.
        Eigen::VectorXd q0 = pin::neutral(model);
        data = pin::Data(model);
        pin::computeJointJacobians(model, data, q0);
        pin::forwardKinematics(model, data, q0);
        const Eigen::Isometry3d Tj = toIso(data.oMi[jid]);
        Eigen::MatrixXd Jw =
            pin::getJointJacobian(model, data, jid, pin::LOCAL_WORLD_ALIGNED);
        const int vidx = model.joints[jid].idx_v();
        const Eigen::Vector3d zWorld = Jw.block<3, 1>(3, vidx);
        L.axis = (Tj.linear().transpose() * zWorld).normalized();
        L.type = JointType::kRevolute; // URDF robots used here
        links.push_back(L);
        ex.jids.push_back(jid);
        const auto& I = model.inertias[jid];
        LinkInertia li;
        li.mass = I.mass();
        li.com = I.lever();
        li.inertia = I.inertia().matrix();
        ex.inertias.push_back(li);
    }
    ex.chain = SerialChain(std::move(links));
    return ex;
}

/// Embed the chain-local q into pinocchio's full configuration vector
/// (panda.urdf carries extra movable finger joints → nq > chain dof).
Eigen::VectorXd embedQ(const pin::Model& model,
                     const std::vector<pin::JointIndex>& jids,
                     const Eigen::VectorXd& qOurs) {
    Eigen::VectorXd q = pin::neutral(model);
    for (size_t k = 0; k < jids.size(); ++k)
        q[model.joints[jids[k]].idx_q()] = qOurs[k];
    return q;
}

/// Compare our post-motion joint frames against pinocchio oMi:
/// oMi[jid] = placement-chain · jointMotion — our jointTransform·A_i.
void expectFK(const SerialChain& chain, const pin::Model& model,
              pin::Data& data, const std::vector<pin::JointIndex>& jids,
              const Eigen::VectorXd& qOurs, double tol) {
    const Eigen::VectorXd qPin = embedQ(model, jids, qOurs);
    pin::forwardKinematics(model, data, qPin);
    for (size_t k = 0; k < jids.size(); ++k) {
        const Eigen::Isometry3d ours =
            chain.jointTransform(qOurs.data(), k) *
            SerialChain::jointMotion(chain.link(k), qOurs[k]);
        const Eigen::Isometry3d ref = toIso(data.oMi[jids[k]]);
        EXPECT_LE((ours.translation() - ref.translation()).norm(), tol)
            << "joint " << k;
        const double ang =
            Eigen::AngleAxisd(ours.linear().transpose() * ref.linear())
                .angle();
        EXPECT_LE(ang, tol) << "joint " << k;
    }
}

} // namespace

// ---------------------------------------------------------------------------
// UR5: repo DH table → SerialChain + programmatic pinocchio model
// ---------------------------------------------------------------------------

class Ur5Test : public ::testing::Test {
protected:
    void SetUp() override {
        ur5DH(dh);
        chain = SerialChain::fromDH(dh, 6);
        // Inertial params (synthetic but consistent both sides)
        inertias.resize(6);
        for (int i = 0; i < 6; ++i) {
            inertias[i].mass = 2.0 + i;
            inertias[i].com = Eigen::Vector3d(0.01 * i, -0.02, 0.03);
            inertias[i].inertia << 0.02 + 0.005 * i, 0.001, 0.0, //
                0.001, 0.02 + 0.004 * i, 0.0005,                 //
                0.0, 0.0005, 0.015 + 0.003 * i;
        }
        model = buildModel(chain, &inertias);
        data = pin::Data(model);
        rnea = RNEA(chain, inertias);
    }
    DHParameters dh[6];
    SerialChain chain;
    std::vector<LinkInertia> inertias;
    pin::Model model;
    pin::Data data;
    RNEA rnea;
};

TEST_F(Ur5Test, ForwardKinematicsMatchesPinocchio) {
    for (int t = 0; t < 20; ++t)
        expectFK(chain, model, data,
                 {1, 2, 3, 4, 5, 6}, randQ(6), 1e-10);
}

TEST_F(Ur5Test, AnalyticJacobianMatchesPinocchio) {
    for (int t = 0; t < 20; ++t) {
        const Eigen::VectorXd q = randQ(6);
        pin::computeJointJacobians(model, data, q);
        pin::forwardKinematics(model, data, q);
        const Eigen::MatrixXd Jref =
            pin::getJointJacobian(model, data, 6, pin::LOCAL_WORLD_ALIGNED);
        // world J of last joint is w.r.t. the JOINT frame origin —
        // our tool==link frame for DH (F6=I) so origins coincide.
        const Eigen::MatrixXd Jours = chain.jacobian(q);
        ASSERT_EQ(Jref.cols(), 6);
        EXPECT_LE((Jours - Jref).cwiseAbs().maxCoeff(), 1e-9);
    }
}

TEST_F(Ur5Test, RneaMatchesPinocchio) {
    for (int t = 0; t < 20; ++t) {
        const Eigen::VectorXd q = randQ(6);
        const Eigen::VectorXd qd = randQ(6, -2, 2);
        const Eigen::VectorXd qdd = randQ(6, -3, 3);
        const Eigen::VectorXd tref = pin::rnea(model, data, q, qd, qdd);
        const Eigen::VectorXd tours = rnea.inverseDynamics(q, qd, qdd);
        EXPECT_LE((tours - tref).cwiseAbs().maxCoeff(), 1e-8)
            << "ours=" << tours.transpose() << "\nref=" << tref.transpose();
    }
}

TEST_F(Ur5Test, MassMatrixAndGravityMatchPinocchio) {
    for (int t = 0; t < 10; ++t) {
        const Eigen::VectorXd q = randQ(6);
        Eigen::MatrixXd Mref = pin::crba(model, data, q);
        // pinocchio fills only the upper triangle of data.M
        Mref.triangularView<Eigen::StrictlyLower>() =
            Mref.triangularView<Eigen::StrictlyUpper>().transpose();
        const Eigen::MatrixXd Mours = rnea.massMatrix(q);
        EXPECT_LE((Mours - Mref).cwiseAbs().maxCoeff(), 1e-8);
        const Eigen::VectorXd qd = Eigen::VectorXd::Zero(6);
        const Eigen::VectorXd gref =
            pin::rnea(model, data, q, qd, qd);
        EXPECT_LE((rnea.gravityVector(q) - gref).cwiseAbs().maxCoeff(),
                  1e-9);
    }
}

TEST_F(Ur5Test, PoseIKConvergesFromSeed) {
    // Iterative DLS pose IK on the REAL UR5 (no spherical wrist).
    SerialChain::IKOptions opt;
    opt.maxIter = 200;
    int ok = 0;
    for (int t = 0; t < 30; ++t) {
        const Eigen::VectorXd qTrue = randQ(6, -1.5, 1.5);
        const Eigen::Isometry3d target = chain.forwardKinematics(qTrue);
        Eigen::VectorXd q = qTrue + randQ(6, -0.4, 0.4);
        double res;
        if (chain.poseIK(target, q, opt, &res)) {
            const double err =
                (chain.forwardKinematics(q).translation() -
                 target.translation())
                    .norm();
            EXPECT_LT(err, 1e-6);
            ++ok;
        }
    }
    EXPECT_GE(ok, 25); // allow a few near-singular failures
}

TEST_F(Ur5Test, ResolvedRateMatchesJacobian) {
    const Eigen::VectorXd q = randQ(6, -1.0, 1.0);
    Eigen::Matrix<double, 6, 1> xdot;
    xdot << 0.1, -0.2, 0.05, 0.3, -0.1, 0.2;
    const Eigen::VectorXd qd = chain.resolvedRate(q, xdot, 1e-4);
    const Eigen::Matrix<double, 6, 1> recovered =
        chain.jacobian(q) * qd;
    EXPECT_LE((recovered - xdot).norm(), 1e-3); // damped → near-exact
}

// ---------------------------------------------------------------------------
// Analytic IK on a TRUE spherical-wrist chain (UR5 with d5 = 0 makes
// axes 4,5,6 concur — verifies hasAnalyticIK + solution quality).
// ---------------------------------------------------------------------------

TEST(AnalyticIK, SphericalWristVariant) {
    DHParameters dh[6];
    ur5DH(dh);
    dh[4].d = 0.0f; // collapse the wrist offset → true concurrency
    SerialChain chain = SerialChain::fromDH(dh, 6);
    ASSERT_TRUE(chain.hasAnalyticIK());
    int solved = 0;
    for (int t = 0; t < 15; ++t) {
        const Eigen::VectorXd qTrue = randQ(6, -1.5, 1.5);
        const Eigen::Isometry3d target = chain.forwardKinematics(qTrue);
        auto res = chain.analyticIK(target);
        ASSERT_FALSE(res.solutions.empty()) << "no solutions, t=" << t;
        bool found = false;
        for (const auto& qs : res.solutions) {
            const Eigen::Isometry3d f = chain.forwardKinematics(qs);
            const double perr =
                (f.translation() - target.translation()).norm();
            const double rerr =
                Eigen::AngleAxisd(f.linear().transpose() *
                                  target.linear())
                    .angle();
            EXPECT_LT(perr, 1e-6);
            EXPECT_LT(rerr, 1e-5);
            if ((qs - qTrue).cwiseAbs().maxCoeff() < 1e-4) found = true;
        }
        // The true configuration must appear among the solution set.
        if (found) ++solved;
        else {
            // joint-angle wraparound equivalence also acceptable
            bool wrap = false;
            for (const auto& qs : res.solutions) {
                bool same = true;
                for (int k = 0; k < 6; ++k) {
                    double d =
                        std::remainder(qs[k] - qTrue[k], 2 * M_PI);
                    if (std::abs(d) > 1e-4) same = false;
                }
                if (same) wrap = true;
            }
            EXPECT_TRUE(wrap);
            if (wrap) ++solved;
        }
    }
    EXPECT_EQ(solved, 15);
}

// ---------------------------------------------------------------------------
// Panda via URDF: extract SerialChain from pinocchio's model and verify
// FK/J/RNEA on a real 7-DOF non-DH robot.
// ---------------------------------------------------------------------------

static const char* kPandaUrdf =
    "/opt/openrobots/share/example-robot-data/robots/"
    "panda_description/urdf/panda.urdf";

class PandaTest : public ::testing::Test {
protected:
    void SetUp() override {
        if (!urdfOk) return;
        pin::Model full;
        pin::urdf::buildModel(kPandaUrdf, full);
        // panda.urdf has movable finger/hand joints past the 7 arm
        // joints. Our chain drops them — freeze them in a reduced
        // oracle model so their link inertias merge into link 7
        // (buildReducedModel appends locked-subtree inertia to the
        // parent). The extracted inertias then match the oracle.
        std::vector<pin::JointIndex> lock;
        for (pin::JointIndex jid = 1; jid < full.njoints; ++jid) {
            const std::string& jn = full.names[jid];
            if (full.joints[jid].nq() > 0 &&
                (jn.find("finger") != std::string::npos ||
                 jn.find("hand") != std::string::npos ||
                 jn.find("tcp") != std::string::npos))
                lock.push_back(jid);
        }
        model = pin::buildReducedModel(full, lock, pin::neutral(full));
        data = pin::Data(model);
        ex = extractChain(model, data);
        rnea = RNEA(ex.chain, ex.inertias);
    }
    bool urdfOk = std::string(kPandaUrdf).size() > 0;
    pin::Model model;
    pin::Data data;
    Extracted ex;
    RNEA rnea;
};

TEST_F(PandaTest, ForwardKinematicsMatchesPinocchio) {
    ASSERT_EQ(ex.chain.dof(), 7);
    for (int t = 0; t < 15; ++t)
        expectFK(ex.chain, model, data, ex.jids, randQ(7), 1e-9);
}

TEST_F(PandaTest, JacobianMatchesPinocchio) {
    for (int t = 0; t < 15; ++t) {
        const Eigen::VectorXd q = randQ(7);
        const Eigen::VectorXd qPin = embedQ(model, ex.jids, q);
        pin::computeJointJacobians(model, data, qPin);
        const Eigen::MatrixXd Jours = ex.chain.jacobian(q);
        const pin::JointIndex last = ex.jids.back();
        const Eigen::MatrixXd Jref =
            pin::getJointJacobian(model, data, last, pin::LOCAL_WORLD_ALIGNED);
        // extract the 7 arm columns (pinocchio J is over nv incl fingers)
        Eigen::MatrixXd Jarm(6, 7);
        for (size_t k = 0; k < ex.jids.size(); ++k)
            Jarm.col(k) = Jref.col(model.joints[ex.jids[k]].idx_v());
        EXPECT_LE((Jours - Jarm).cwiseAbs().maxCoeff(), 1e-9);
    }
}

TEST_F(PandaTest, RneaAndGravityMatchPinocchio) {
    for (int t = 0; t < 15; ++t) {
        const Eigen::VectorXd q = randQ(7);
        const Eigen::VectorXd qd = randQ(7, -2, 2);
        const Eigen::VectorXd qdd = randQ(7, -3, 3);
        const Eigen::VectorXd qPin = embedQ(model, ex.jids, q);
        Eigen::VectorXd vPin = Eigen::VectorXd::Zero(model.nv);
        Eigen::VectorXd aPin = Eigen::VectorXd::Zero(model.nv);
        for (size_t k = 0; k < ex.jids.size(); ++k) {
            const int v = model.joints[ex.jids[k]].idx_v();
            vPin[v] = qd[k];
            aPin[v] = qdd[k];
        }
        const Eigen::VectorXd tFull =
            pin::rnea(model, data, qPin, vPin, aPin);
        const Eigen::VectorXd tours = rnea.inverseDynamics(q, qd, qdd);
        for (size_t k = 0; k < ex.jids.size(); ++k)
            EXPECT_LE(std::abs(tours[k] - tFull[model.joints[ex.jids[k]].idx_v()]),
                      1e-8)
                << "joint " << k << " ours=" << tours[k]
                << " ref=" << tFull[model.joints[ex.jids[k]].idx_v()];
        Eigen::MatrixXd Mref = pin::crba(model, data, qPin);
        Mref.triangularView<Eigen::StrictlyLower>() =
            Mref.triangularView<Eigen::StrictlyUpper>().transpose();
        const Eigen::MatrixXd Mours = rnea.massMatrix(q);
        for (size_t i = 0; i < 7; ++i)
            for (size_t j = 0; j < 7; ++j)
                EXPECT_LE(
                    std::abs(Mours(i, j) -
                             Mref(model.joints[ex.jids[i]].idx_v(),
                                  model.joints[ex.jids[j]].idx_v())),
                    1e-8)
                    << "M[" << i << "," << j << "]";
    }
}

TEST_F(PandaTest, PoseIKPlateauExitBoundsRealtimeWork) {
    // A far-outside-workspace target can never satisfy the strict IK
    // tolerance. Real-time callers opt into plateau detection so the
    // solve stops at the stationary point instead of burning the whole
    // control period in every cycle.
    Eigen::VectorXd q0 = Eigen::VectorXd::Zero(7);
    Eigen::Isometry3d target = ex.chain.forwardKinematics(q0);
    target.translation().x() += 2.0;
    NullspaceRef ns;
    ns.posture = q0;
    ns.postureGain = 0.2;

    SerialChain::IKOptions bounded;
    bounded.maxIter = 60;
    bounded.nsSettle = false;
    bounded.plateauIters = 6;
    int boundedIters = 0;
    bounded.iterationCount = &boundedIters;
    Eigen::VectorXd qBounded = q0;
    double resBounded = 0.0;
    EXPECT_FALSE(ex.chain.poseIK(target, qBounded, bounded,
                                 &resBounded, ns));
    EXPECT_LT(boundedIters, 40);
    EXPECT_TRUE(std::isfinite(resBounded));

    SerialChain::IKOptions exact = bounded;
    exact.plateauIters = 0;
    int exactIters = 0;
    exact.iterationCount = &exactIters;
    Eigen::VectorXd qExact = q0;
    double resExact = 0.0;
    EXPECT_FALSE(ex.chain.poseIK(target, qExact, exact,
                                 &resExact, ns));
    EXPECT_GT(exactIters, boundedIters);
}

TEST_F(PandaTest, PoseIKOnSevenDof) {
    SerialChain::IKOptions opt;
    opt.maxIter = 250;
    int ok = 0;
    for (int t = 0; t < 20; ++t) {
        const Eigen::VectorXd qTrue = randQ(7, -1.0, 1.0);
        const Eigen::Isometry3d target =
            ex.chain.forwardKinematics(qTrue);
        Eigen::VectorXd q = qTrue + randQ(7, -0.3, 0.3);
        if (ex.chain.poseIK(target, q, opt)) {
            const double err =
                (ex.chain.forwardKinematics(q).translation() -
                 target.translation())
                    .norm();
            EXPECT_LT(err, 1e-6);
            ++ok;
        }
    }
    EXPECT_GE(ok, 16);
}

// ---------------------------------------------------------------------------
// NullspaceRef: the redundant-DOF objective must only move joints along
// N = I - J⁺J (task preserved), be wrap-aware, repel at limits, and orbit
// the elbow about the shoulder→wrist axis for the swivel term.
// ---------------------------------------------------------------------------

TEST_F(PandaTest, NullspacePullPreservesTask) {
    // With xdot = 0, resolvedRate must return a pure nullspace
    // velocity: the pull moves joints but J·qdot stays ~0.
    for (int t = 0; t < 10; ++t) {
        const Eigen::VectorXd q = randQ(7, -1.5, 1.5);
        NullspaceRef ns;
        ns.posture = randQ(7, -2.0, 2.0);
        ns.postureGain = 0.8;
        const Eigen::VectorXd qd = ex.chain.resolvedRate(
            q, Eigen::Matrix<double, 6, 1>::Zero(), 1e-3, ns);
        const Eigen::VectorXd taskVel = ex.chain.jacobian(q) * qd;
        EXPECT_LT(taskVel.norm(), 1e-5)
            << "pull leaked into task space: " << taskVel.transpose();
        EXPECT_GT(qd.norm(), 1e-4);  // pull is nonzero
    }
}

TEST_F(PandaTest, NullspacePosturePullIsWrapAware) {
    // q = -3.0, pref = +3.0: naive difference is +6.0 rad; the wrapped
    // short way is -0.283 (crossing ±π). The pull must command the
    // small step, never a ~2π kick.
    Eigen::VectorXd q = Eigen::VectorXd::Zero(7);
    q[2] = -3.0;
    NullspaceRef ns;
    ns.posture = Eigen::VectorXd::Zero(7);
    ns.posture[2] = 3.0;
    ns.postureGain = 1.0;
    const Eigen::VectorXd qd = ex.chain.resolvedRate(
        q, Eigen::Matrix<double, 6, 1>::Zero(), 1e-3, ns);
    EXPECT_LT(qd[2], 0.0);         // short way decreases q[2]
    EXPECT_GT(qd[2], -0.4);        // |pull| ≈ 0.283 projected, not 6.0
}

TEST_F(PandaTest, NullspaceLimitRepulsion) {
    Eigen::VectorXd lo(7), hi(7);
    for (int i = 0; i < 7; ++i) {
        lo[i] = -2.0;
        hi[i] = 2.0;
    }
    NullspaceRef ns;
    ns.lo = lo;
    ns.hi = hi;
    ns.limitGain = 0.5;
    // Near the upper bound the pull pushes back down; near the lower,
    // up; at mid-range it is ~0.
    Eigen::VectorXd q = Eigen::VectorXd::Zero(7);
    q[4] = 1.9;
    const Eigen::VectorXd qdHi = ex.chain.resolvedRate(
        q, Eigen::Matrix<double, 6, 1>::Zero(), 1e-3, ns);
    EXPECT_LT(qdHi[4], 0.0);
    q[4] = -1.9;
    const Eigen::VectorXd qdLo = ex.chain.resolvedRate(
        q, Eigen::Matrix<double, 6, 1>::Zero(), 1e-3, ns);
    EXPECT_GT(qdLo[4], 0.0);
    // And the repulsion is stiffer near the bound than mid-range.
    Eigen::VectorXd qMid = Eigen::VectorXd::Zero(7);
    qMid[4] = 0.5;
    const Eigen::VectorXd qdMid = ex.chain.resolvedRate(
        qMid, Eigen::Matrix<double, 6, 1>::Zero(), 1e-3, ns);
    EXPECT_LT(std::abs(qdMid[4]), std::abs(qdHi[4]));
}

TEST_F(PandaTest, NullspaceSwivelOrbitsElbow) {
    // Same task pose, two opposite swivel preferences → the elbow must
    // land at different points on the swivel orbit while both solves
    // converge to the target. elbowDir is derived from the seed's own
    // geometry: the orbit tangent t̂ = a×r̂ is ⊥a by construction, so
    // the preference is never projected away.
    const Eigen::VectorXd qSeed = [] {
        Eigen::VectorXd q(7);
        q << 0.6, -0.7, 0.3, -1.5, -0.4, 1.8, 0.5;
        return q;
    }();
    const Eigen::Vector3d S =
        ex.chain.jointTransform(qSeed.data(), 2).translation();
    const Eigen::Vector3d E =
        ex.chain.jointTransform(qSeed.data(), 3).translation();
    const Eigen::Vector3d W =
        ex.chain.jointTransform(qSeed.data(), 5).translation();
    const Eigen::Vector3d a = (W - S).normalized();
    const Eigen::Vector3d r = (E - S) - a * a.dot(E - S);
    ASSERT_GT(r.norm(), 1e-3);  // needs a real orbit radius
    const Eigen::Vector3d tHat = a.cross(r.normalized());

    const Eigen::Isometry3d target =
        ex.chain.forwardKinematics(qSeed);
    SerialChain::IKOptions opt;
    opt.maxIter = 250;
    NullspaceRef ns;
    ns.shoulder = 2;
    ns.elbow = 3;
    ns.wrist = 5;
    ns.swivelGain = 0.4;
    const Eigen::VectorXd seed = qSeed + randQ(7, -0.15, 0.15);

    Eigen::VectorXd qFwd = seed;
    ns.elbowDir = tHat;
    ASSERT_TRUE(ex.chain.poseIK(target, qFwd, opt, nullptr, ns));
    Eigen::VectorXd qBwd = seed;
    ns.elbowDir = -tHat;
    ASSERT_TRUE(ex.chain.poseIK(target, qBwd, opt, nullptr, ns));
    const Eigen::Vector3d eFwd =
        ex.chain.jointTransform(qFwd.data(), 3).translation();
    const Eigen::Vector3d eBwd =
        ex.chain.jointTransform(qBwd.data(), 3).translation();
    EXPECT_GT((eFwd - eBwd).norm(), 5e-3)
        << "swivel direction had no effect on the elbow orbit";
    // Both solutions must still be on-task.
    EXPECT_LT((ex.chain.forwardKinematics(qFwd).translation() -
               target.translation())
                  .norm(),
              1e-6);
    EXPECT_LT((ex.chain.forwardKinematics(qBwd).translation() -
               target.translation())
                  .norm(),
              1e-6);
}

TEST_F(PandaTest, NullspaceSwivelDegenerate) {
    // shoulder == elbow collapses the offset (r = 0): the term must
    // skip cleanly — finite output, no NaN/instability.
    const Eigen::VectorXd q = randQ(7, -1.0, 1.0);
    NullspaceRef ns;
    ns.shoulder = 2;
    ns.elbow = 2;
    ns.wrist = 5;
    ns.swivelGain = 1.0;
    const Eigen::VectorXd qd = ex.chain.resolvedRate(
        q, Eigen::Matrix<double, 6, 1>::Zero(), 1e-3, ns);
    EXPECT_TRUE(qd.allFinite());
    EXPECT_LE(qd.norm(), 1e-9);
}

TEST_F(PandaTest, NullspacePosturePinsIkBranch) {
    // A posture pull must move the converged solution toward the
    // reference on the redundant direction while staying on-task.
    const Eigen::VectorXd qTrue = randQ(7, -0.8, 0.8);
    const Eigen::Isometry3d target =
        ex.chain.forwardKinematics(qTrue);
    SerialChain::IKOptions opt;
    opt.maxIter = 250;
    const Eigen::VectorXd seed = qTrue + randQ(7, -0.25, 0.25);

    Eigen::VectorXd qFree = seed;
    ASSERT_TRUE(ex.chain.poseIK(target, qFree, opt));

    NullspaceRef ns;
    ns.posture = Eigen::VectorXd::Zero(7);
    ns.postureGain = 0.4;
    Eigen::VectorXd qPin = seed;
    ASSERT_TRUE(ex.chain.poseIK(target, qPin, opt, nullptr, ns));

    EXPECT_LT(qPin.norm(), qFree.norm());
    EXPECT_LT((ex.chain.forwardKinematics(qPin).translation() -
               target.translation())
                  .norm(),
              1e-6);
}

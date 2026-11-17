#include "tether/control/thermal/OvenGradientLQGController.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <numbers>

namespace tether::control::thermal {

namespace {
constexpr size_t kMaxZones = MAX_CONTROL_DIM;   // 6 (control-dim cap)

// Row-major copy of an Eigen matrix for the raw-pointer LQI/discretize APIs.
std::vector<double> toRowMajor(const Eigen::MatrixXd& M) {
    std::vector<double> v(static_cast<size_t>(M.rows()) * M.cols());
    for (Eigen::Index i = 0; i < M.rows(); ++i)
        for (Eigen::Index j = 0; j < M.cols(); ++j)
            v[static_cast<size_t>(i) * M.cols() + j] = M(i, j);
    return v;
}
} // namespace

OvenGradientLQGController::OvenGradientLQGController(
    const GradientLQGConfig& cfg)
    : m_cfg(cfg) {}

void OvenGradientLQGController::addZone(const GradientZoneConfig& cfg) {
    auto z = std::make_unique<Zone>();
    z->cfg = cfg;
    const auto& m = cfg.controller.model;
    z->uMin = m.uMin;
    z->uMax = m.uMax;
    z->alpha = m.alpha;
    z->beta = m.beta;
    z->delay = m.delay;
    z->ambient = m.ambient;
    z->disturbance = m.disturbance;
    z->couplingLeft = cfg.couplingLeft;
    z->couplingRight = cfg.couplingRight;
    z->setpoint = m.ambient;

    z->ffModel.setParams(m);

    GovernorConfig gc;
    gc.alpha = m.alpha;
    gc.beta = m.beta;
    gc.uMin = m.uMin;
    gc.uMax = m.uMax;
    gc.ambient = m.ambient;
    gc.disturbance = m.disturbance;
    gc.trackingGain = cfg.controller.governor.trackingGain;
    gc.previewTime = cfg.controller.governor.previewTime;
    z->gov.setConfig(gc);

    m_zones.push_back(std::move(z));
    m_lastU.push_back(0.0);
    m_lastMeasured.push_back(m.ambient);
    m_designed = false;   // zone set changed -> redesign on next compute
}

void OvenGradientLQGController::setZoneProfile(size_t zone,
                                               const TemperatureProfile& p) {
    Zone& z = *m_zones.at(zone);
    z.profile = p;
    z.hasProfile = !p.empty();
    if (z.hasProfile) z.gov.reset(p.value(0.0));
}

void OvenGradientLQGController::setZoneSetpoint(size_t zone, double tempC) {
    Zone& z = *m_zones.at(zone);
    z.setpoint = tempC;
    z.hasProfile = false;
    z.profile.clear();
}

void OvenGradientLQGController::setZoneSetpoints(
    const std::vector<double>& tempsC, double maxDeltaC) {
    const size_t n = std::min(tempsC.size(), m_zones.size());
    std::vector<double> sp = tempsC;
    if (maxDeltaC > 0.0) {
        for (size_t i = 1; i < n; ++i) {
            sp[i] = std::clamp(sp[i], sp[i - 1] - maxDeltaC,
                               sp[i - 1] + maxDeltaC);
        }
    }
    for (size_t i = 0; i < n; ++i) setZoneSetpoint(i, sp[i]);
}

bool OvenGradientLQGController::setGradientProfile(const GradientSpec& spec) {
    if (m_zones.empty() || spec.base.empty()) return false;

    const size_t n = m_zones.size();
    std::vector<double> off(n, 0.0), shift(n, 0.0);
    for (size_t i = 0; i < n; ++i) {
        if (i < spec.tempOffsetC.size()) off[i] = spec.tempOffsetC[i];
        if (i < spec.timeShiftS.size()) shift[i] = spec.timeShiftS[i];
    }

    // Clamp adjacent offset deltas (same forward-pass semantics as
    // OvenGradientController::setGradientProfile).
    if (spec.maxZoneDeltaC > 0.0) {
        for (size_t i = 1; i < n; ++i) {
            off[i] = std::clamp(off[i], off[i - 1] - spec.maxZoneDeltaC,
                                off[i - 1] + spec.maxZoneDeltaC);
        }
    }

    for (size_t i = 0; i < n; ++i) {
        setZoneProfile(i, spec.base.transformed(off[i], shift[i]));
    }
    return true;
}

bool OvenGradientLQGController::solveModel() {
    const int n = static_cast<int>(m_zones.size());
    if (n < 1 || n > static_cast<int>(kMaxZones)) {
        m_designed = false;
        return false;
    }
    m_n = n;
    m_maxDelay = 0.0;
    for (const auto& zp : m_zones) m_maxDelay = std::max(m_maxDelay, zp->delay);
    const double dt = std::max(m_cfg.dtDesign, 1e-3);

    // ---- Continuous MIMO model: dx/dt = A x + B u + c ----
    // Zone i: dT_i = alpha_i u_i - beta_i (T_i - amb_i)
    //          + cL_i (T_{i-1} - T_i) + cR_i (T_{i+1} - T_i) + d_i
    // Boundary zones have only the inward neighbor (matches the
    // decentralized coupling semantics).
    Eigen::MatrixXd A = Eigen::MatrixXd::Zero(n, n);
    Eigen::MatrixXd B = Eigen::MatrixXd::Zero(n, n);
    Eigen::VectorXd c = Eigen::VectorXd::Zero(n);
    for (int i = 0; i < n; ++i) {
        const Zone& z = *m_zones[i];
        const double cl = (i > 0) ? z.couplingLeft : 0.0;
        const double cr = (i + 1 < n) ? z.couplingRight : 0.0;
        A(i, i) = -(z.beta + cl + cr);
        if (i > 0) A(i, i - 1) += cl;
        if (i + 1 < n) A(i, i + 1) += cr;
        B(i, i) = z.alpha;
        c(i) = z.beta * z.ambient + z.disturbance;
    }

    // Augment B with the constant affine column c so one discretization
    // yields both the input response Bd and the constant response gd.
    Eigen::MatrixXd Baug(n, n + 1);
    Baug.leftCols(n) = B;
    Baug.rightCols(1) = c;

    std::vector<double> AdV(static_cast<size_t>(n) * n, 0.0);
    std::vector<double> BdV(static_cast<size_t>(n) * (n + 1), 0.0);
    {
        const auto A_rm = toRowMajor(A);
        const auto Baug_rm = toRowMajor(Baug);
        StateSpace::discretize(A_rm.data(), Baug_rm.data(), dt,
                               AdV.data(), BdV.data(), n, n + 1);
    }
    Eigen::Map<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic,
                             Eigen::RowMajor>>
        AdMap(AdV.data(), n, n);
    Eigen::Map<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic,
                             Eigen::RowMajor>>
        BdMap(BdV.data(), n, n + 1);
    m_Ad = AdMap;
    m_Bd = BdMap.leftCols(n);       // response to the n real inputs
    m_gd = BdMap.col(n);            // response to the constant "1" input
    m_c = c;

    // ---- LQI regulator: gain design only ----
    // LQIController::computeImpl is scalar-reference, so we use it purely
    // to solve the augmented DARE and read back Kx/Ki; the MIMO loop below
    // implements the per-zone-reference control law ourselves.
    LQIController lqi;
    const auto Ad_rm = toRowMajor(m_Ad);
    const auto Bd_rm = toRowMajor(m_Bd);
    std::vector<double> C_rm(static_cast<size_t>(n) * n, 0.0);
    for (int i = 0; i < n; ++i) C_rm[static_cast<size_t>(i) * n + i] = 1.0;
    lqi.setDiscreteSystemMatrices(Ad_rm.data(), Bd_rm.data(), C_rm.data(),
                                  n, n, n, dt);

    // Qx = qxZone*I + qxGradient*Laplacian. In deviation coordinates
    // z = x̂ - r, z'Lz = sum (z_i - z_{i-1})^2 is the squared gradient
    // tracking error — the quantity decentralized SISO PID cannot weight.
    std::vector<double> Qx_rm(static_cast<size_t>(n) * n, 0.0);
    for (int i = 0; i < n; ++i) {
        for (int j = 0; j < n; ++j) {
            double v = (i == j) ? m_cfg.qxZone : 0.0;
            if (i == j) {
                const double deg = (i > 0 ? 1.0 : 0.0) +
                                   (i + 1 < n ? 1.0 : 0.0);
                v += m_cfg.qxGradient * deg;
            } else if (j == i - 1 || j == i + 1) {
                v -= m_cfg.qxGradient;
            }
            Qx_rm[static_cast<size_t>(i) * n + j] = v;
        }
    }
    std::vector<double> Qi_rm(static_cast<size_t>(n) * n, 0.0);
    std::vector<double> R_rm(static_cast<size_t>(n) * n, 0.0);
    for (int i = 0; i < n; ++i) {
        Qi_rm[static_cast<size_t>(i) * n + i] = m_cfg.qi;
        R_rm[static_cast<size_t>(i) * n + i] = m_cfg.r;
    }
    lqi.setWeights(Qx_rm.data(), Qi_rm.data(), R_rm.data());
    if (!lqi.design()) {
        m_designed = false;
        return false;
    }

    std::vector<double> Kx_rm(static_cast<size_t>(n) * n, 0.0);
    std::vector<double> Ki_rm(static_cast<size_t>(n) * n, 0.0);
    lqi.getStateGain(Kx_rm.data());
    lqi.getIntegralGain(Ki_rm.data());
    Eigen::Map<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic,
                             Eigen::RowMajor>>
        KxMap(Kx_rm.data(), n, n);
    Eigen::Map<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic,
                             Eigen::RowMajor>>
        KiMap(Ki_rm.data(), n, n);
    m_Kx = KxMap;
    m_Ki = KiMap;

    // Per-zone integrator clamp: explicit limit, else auto-scale so
    // |Ki_ii * xi| can demand ~1.2x the zone's authority (anti-windup).
    m_xiLim.assign(static_cast<size_t>(n), 1e6);
    for (int i = 0; i < n; ++i) {
        const Zone& z = *m_zones[i];
        double lim = m_cfg.integralLimit;
        if (lim <= 0.0) {
            const double auth = 1.2 * std::max(
                {std::abs(z.uMax), std::abs(z.uMin), 1e-9});
            lim = auth / std::max(std::abs(m_Ki(i, i)), 1e-9);
            lim = std::clamp(lim, 1.0, 1e6);
        }
        m_xiLim[static_cast<size_t>(i)] = lim;
    }

    // ---- LQG observer: steady-state Kalman on (Ad, [Bd|gd], I) ----
    // The last input column carries the constant affine response gd, so the
    // observer predicts with the known ambient+disturbance drive folded in.
    Eigen::MatrixXd Bobs(n, n + 1);
    Bobs.leftCols(n) = m_Bd;
    Bobs.rightCols(1) = m_gd;
    const Eigen::MatrixXd Cobs = Eigen::MatrixXd::Identity(n, n);
    m_kf.setSystemMatrices(m_Ad, Bobs, Cobs);
    m_kf.setNoiseCovariances(Eigen::MatrixXd::Identity(n, n) *
                                 (m_cfg.qProc * dt),
                             Eigen::MatrixXd::Identity(n, n) * m_cfg.rMeas);
    if (!m_kf.computeGain()) {
        m_designed = false;
        return false;
    }
    // Seed the estimate with the first measurement on the next compute;
    // setInitialState keeps the converged steady-state covariance.
    m_obsInit = false;

    m_z.resize(n);
    m_uAug.resize(n + 1);
    m_y.resize(n);
    m_ufb.resize(n);
    m_xHat.resize(n);
    m_xi.conservativeResize(n);   // preserves integrators across retune()
    m_uDelayed.resize(static_cast<size_t>(n));
    m_ambient.resize(static_cast<size_t>(n));
    m_designed = true;
    return true;
}

bool OvenGradientLQGController::design() {
    if (!solveModel()) return false;
    // Fresh-start dynamic state (zone set changed / first compute).
    m_xi.setZero();
    m_xHat.setZero();
    m_obsInit = false;
    return true;
}

bool OvenGradientLQGController::retune() {
    // Re-solve model + gains. Integrators, clock and input history are
    // untouched -> no bump. The observer deliberately does NOT preserve
    // its estimate: the sensors are collocated (C = I), so the raw
    // measurement IS the state, and re-seeding to it at each retune
    // clears any estimate bias accumulated under the previous model —
    // important while adaptation is actively changing A/B. (Preserving
    // the filtered state measurably degraded gradient tracking.)
    return solveModel();
}

bool OvenGradientLQGController::syncParamsFromEstimator() {
    const size_t n = m_zones.size();
    bool changed = false;
    const double minD = m_cfg.adaptRetuneMinDelta;
    const auto moved = [minD](double a, double b) {
        return std::abs(a - b) >
               minD * std::max({1e-3, std::abs(a), std::abs(b)});
    };
    for (size_t i = 0; i < n; ++i) {
        Zone& z = *m_zones[i];
        const CoupledZoneEstimate e = m_est.estimate(i);
        if (moved(z.alpha, e.alpha) || moved(z.beta, e.beta) ||
            moved(z.couplingLeft, e.couplingLeft) ||
            moved(z.couplingRight, e.couplingRight) ||
            moved(z.disturbance, e.disturbance)) {
            changed = true;
        }
        z.alpha = e.alpha;
        z.beta = e.beta;
        z.couplingLeft = e.couplingLeft;
        z.couplingRight = e.couplingRight;
        z.disturbance = e.disturbance;

        // Keep the per-zone SISO feedforward model + governor consistent
        // with the adapted parameters (coupling stays in the MIMO A only).
        OvenModelParams mp;
        mp.alpha = z.alpha; mp.beta = z.beta; mp.delay = z.delay;
        mp.disturbance = z.disturbance; mp.ambient = z.ambient;
        mp.uMin = z.uMin; mp.uMax = z.uMax;
        z.ffModel.setParams(mp);

        GovernorConfig gc;
        gc.alpha = z.alpha; gc.beta = z.beta;
        gc.uMin = z.uMin; gc.uMax = z.uMax;
        gc.ambient = z.ambient; gc.disturbance = z.disturbance;
        gc.trackingGain = z.cfg.controller.governor.trackingGain;
        gc.previewTime = z.cfg.controller.governor.previewTime;
        z.gov.setConfig(gc);
    }
    return changed;
}

void OvenGradientLQGController::maybeAdapt(
    std::span<const double> measured, double dt) {
    if (!m_cfg.adaptParams || m_zones.empty()) return;
    const size_t n = m_zones.size();
    if (!m_estInit) {
        std::vector<CoupledZoneParams> priors(n);
        for (size_t i = 0; i < n; ++i) {
            const Zone& z = *m_zones[i];
            CoupledZoneParams p;
            p.alpha = z.alpha;
            p.beta = z.beta;
            p.couplingLeft = (i > 0) ? z.couplingLeft : 0.0;
            p.couplingRight = (i + 1 < n) ? z.couplingRight : 0.0;
            p.disturbance = z.disturbance;
            p.ambient = z.ambient;
            priors[i] = p;
        }
        m_est.setConfig(m_cfg.estimator);
        m_est.setZones(priors);
        m_estInit = true;
        m_nextAdapt = m_time + std::max(m_cfg.adaptRedesignInterval, 0.5);
    }
    // Delayed applied inputs + ambient for the estimator regressors.
    for (size_t i = 0; i < n; ++i) {
        const Zone& z = *m_zones[i];
        m_uDelayed[i] = delayedInput(i, m_time - z.delay);
        m_ambient[i] = z.ambient;
    }
    m_est.update(m_uDelayed, measured, m_ambient, dt);

    const bool due = (m_cfg.adaptRedesignInterval <= 0.0) ||
                     (m_time >= m_nextAdapt);
    if (due) {
        if (syncParamsFromEstimator()) retune();
        m_nextAdapt = m_time + std::max(m_cfg.adaptRedesignInterval, 0.5);
    }
}

CoupledZoneEstimate OvenGradientLQGController::zoneParameterEstimate(
    size_t zone) const {
    if (!m_estInit || zone >= m_zones.size()) return {};
    return m_est.estimate(zone);
}

bool OvenGradientLQGController::zoneEstimatorExcited(size_t zone) const {
    return m_estInit && zone < m_zones.size() && m_est.lastExcited(zone);
}

double OvenGradientLQGController::delayedInput(size_t zone,
                                               double tLookback) const {
    const auto& h = m_zones[zone]->uHist;
    if (h.empty() || tLookback < h.front().first) return 0.0;  // pre-history: heater off
    // Latest commanded u_i at time <= tLookback (deque is time-ordered).
    for (auto it = h.rbegin(); it != h.rend(); ++it) {
        if (it->first <= tLookback) return it->second;
    }
    return h.front().second;
}

std::vector<double> OvenGradientLQGController::compute(
    std::span<const double> measured, double dt) {
    const size_t n = m_zones.size();
    if (measured.size() != n) return {};
    std::vector<double> u = m_lastU;   // hold last applied on failure
    computeInto(measured, dt, u);
    return u;
}

bool OvenGradientLQGController::computeInto(
    std::span<const double> measured, double dt,
    std::span<double> out) {
    const size_t n = m_zones.size();
    if (measured.size() != n || out.size() != n || n == 0) return false;
    if (!m_designed || static_cast<int>(n) != m_n) {
        if (!design()) return false;
    }
    if (dt <= 0.0) {
        for (size_t i = 0; i < n; ++i) out[i] = m_lastU[i];
        return true;
    }

    // ---- 1. Per-zone governed reference ----
    m_maxRefDelta = 0.0;
    for (size_t i = 0; i < n; ++i) {
        Zone& z = *m_zones[i];
        if (z.hasProfile) {
            z.gov.update(m_time, z.profile, dt);
            z.ref = z.gov.value();
            z.refDot = z.gov.rate();
        } else {
            z.ref = z.setpoint;
            z.refDot = 0.0;
        }
        if (i > 0)
            m_maxRefDelta =
                std::max(m_maxRefDelta, std::abs(z.ref - m_zones[i - 1]->ref));
    }

    // ---- 2. LQG observer: predict with delayed applied input, correct ----
    for (size_t i = 0; i < n; ++i) {
        m_uAug(static_cast<Eigen::Index>(i)) =
            delayedInput(i, m_time - m_zones[i]->delay);
        m_y(static_cast<Eigen::Index>(i)) = measured[i];
    }
    m_uAug(static_cast<Eigen::Index>(n)) = 1.0;   // constant affine column
    if (!m_obsInit) {
        m_kf.setInitialState(m_y);   // keeps converged steady-state P
        m_obsInit = true;
    } else {
        m_kf.predict(m_uAug);
    }
    m_kf.update(m_y);
    m_xHat = m_kf.getStateVector();

    // ---- 3. Deviation states + per-zone integrators ----
    // z_i = x̂_i - r_i ; integrator accumulates e_i = r_i - x̂_i so that
    // xi -> the steady-state input offset (zero tracking error at ss).
    for (size_t i = 0; i < n; ++i) {
        const double xh = m_xHat(static_cast<Eigen::Index>(i));
        const double r = m_zones[i]->ref;
        m_z(static_cast<Eigen::Index>(i)) = xh - r;
        const double e = r - xh;
        m_xi(static_cast<Eigen::Index>(i)) += e * dt;
        const double lim = m_xiLim[i];
        m_xi(static_cast<Eigen::Index>(i)) =
            std::clamp(m_xi(static_cast<Eigen::Index>(i)), -lim, lim);
    }

    // ---- 4. Centralized MIMO feedback: u_fb = -(Kx z + Ki xi) ----
    // Evaluated into the pre-sized member (no per-cycle allocation).
    m_ufb.noalias() = m_Kx * m_z;
    m_ufb.noalias() += m_Ki * m_xi;
    m_ufb *= -1.0;

    // ---- 5. Per-zone previewed FF + sum + clamp + rate-limit ----
    for (size_t i = 0; i < n; ++i) {
        Zone& z = *m_zones[i];
        double rFf, rDotFf;
        if (z.hasProfile) {
            rFf = z.gov.previewValue(z.delay);
            rDotFf = z.refDot;
        } else {
            rFf = z.setpoint;
            rDotFf = 0.0;
        }
        const double uff =
            m_cfg.feedforward
                ? z.ffModel.requiredInputClamped(rFf, rDotFf, z.ambient,
                                                 z.disturbance)
                : 0.0;
        double uDesat = uff + m_ufb(static_cast<Eigen::Index>(i));
        // PE dither for closed-loop identification: a per-zone phase-
        // offset sinusoid decorrelates the coupled estimator's regressors
        // (golden-angle spacing keeps the zones' dither signals distinct).
        if (m_cfg.adaptParams && m_cfg.adaptDitherAmp > 0.0) {
            const double phase =
                static_cast<double>(i) * 2.399963229728653;  // golden angle
            uDesat += m_cfg.adaptDitherAmp *
                std::sin(2.0 * std::numbers::pi * m_time /
                             m_cfg.adaptDitherPeriod +
                         phase);
        }
        double u = uDesat;
        if (m_cfg.uRateLimit > 0.0) {
            const double md = m_cfg.uRateLimit * dt;
            u = m_lastU[i] + std::clamp(u - m_lastU[i], -md, md);
        }
        u = std::clamp(u, z.uMin, z.uMax);

        // Back-calculation anti-windup (same law as LQIController):
        // bleed xi by the authority lost to saturation/rate-limit so a
        // long saturated transient cannot wind the integrator. Uses the
        // diagonal Ki(i,i); residual vanishes when unsaturated.
        if (m_cfg.antiWindupTt > 0.0) {
            const double residual = u - uDesat;
            const double ki = m_Ki(i, i);
            if (residual != 0.0 && std::abs(ki) > 1e-12) {
                m_xi(i) -= residual * dt / (ki * m_cfg.antiWindupTt);
                const double lim = m_xiLim[static_cast<size_t>(i)];
                m_xi(i) = std::clamp(m_xi(i), -lim, lim);
            }
        }
        z.u = u;
        out[i] = u;
    }

    // ---- 6. Record applied input for the observer's dead-time line ----
    // Trim history to the dead-time window so the deque stays bounded.
    const double tKeep = m_time - m_maxDelay - 4.0 * dt;
    for (size_t i = 0; i < n; ++i) {
        Zone& z = *m_zones[i];
        z.uHist.emplace_back(m_time, z.u);
        while (!z.uHist.empty() && z.uHist.front().first < tKeep)
            z.uHist.pop_front();
        m_lastU[i] = z.u;
        m_lastMeasured[i] = measured[i];
    }
    maybeAdapt(measured, dt);
    m_time += dt;
    return true;
}

double OvenGradientLQGController::maxMeasuredDelta() const {
    double maxD = 0.0;
    for (size_t i = 1; i < m_lastMeasured.size(); ++i) {
        maxD = std::max(maxD,
                        std::abs(m_lastMeasured[i] - m_lastMeasured[i - 1]));
    }
    return maxD;
}

void OvenGradientLQGController::reset() {
    for (auto& zp : m_zones) {
        Zone& z = *zp;
        const double t0 = z.hasProfile ? z.profile.value(0.0) : z.setpoint;
        z.gov.reset(t0);
        z.uHist.clear();
        z.u = 0.0;
    }
    m_time = 0.0;
    m_maxRefDelta = 0.0;
    m_obsInit = false;
    if (m_estInit) m_est.reset();
    m_nextAdapt = 0.0;
    if (m_xi.size() > 0) m_xi.setZero();
    if (m_xHat.size() > 0) m_xHat.setZero();
    for (size_t i = 0; i < m_lastU.size(); ++i) {
        m_lastU[i] = 0.0;
        m_lastMeasured[i] = m_zones[i]->ambient;
    }
}

} // namespace tether::control::thermal

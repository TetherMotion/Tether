#include "tether/control/motion/ServoAxisLQIBackend.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>

namespace tether::control::motion {

// ---------------------------------------------------------------------------
// ServoAxisObserver — steady-state Kalman filter, C = [1,0,0]
// ---------------------------------------------------------------------------

void ServoAxisObserver::design(const ServoAxisParams& plant,
                               double dtDesign, double qAccel,
                               double qDist, double rMeas) {
    const double J = std::max(plant.inertia, 1e-9);
    const double b = std::max(plant.damping, 0.0);
    const double kt = std::max(std::abs(plant.torqueConstant), 1e-9);

    m_bOverJ = b / J;
    m_ktOverJ = kt / J;
    m_invJ = 1.0 / J;

    // Discrete model at the design step (Euler — servo cycle times are
    // well below the mechanical dynamics' fastest timescale).
    //   A_c = [[0, 1, 0], [0, -b/J, -1/J], [0, 0, 0]]  (τ_d random walk)
    const double dt = std::max(dtDesign, 1e-5);
    const double ad00 = 1.0, ad01 = dt, ad02 = 0.0;
    const double ad10 = 0.0, ad11 = 1.0 - m_bOverJ * dt, ad12 = -dt * m_invJ;
    const double ad20 = 0.0, ad21 = 0.0,                 ad22 = 1.0;

    // Discrete noise covariances: position is driven (no direct noise),
    // velocity integrates the accel noise, τ_d integrates the walk.
    const double q1 = std::max(qAccel, 0.0) * dt;
    const double q2 = std::max(qDist, 0.0) * dt;
    const double r = std::max(rMeas, 1e-15);

    // Iterate the predictor Riccati equation to steady state:
    //   S  = P00 + R
    //   L  = P·C' / S            (C = [1,0,0] → row 0 of P)
    //   Pp = P - L·(C·P)         (posterior covariance)
    //   P⁻ = Ad·Pp·Ad' + Q
    double P[3][3] = {{q1, 0, 0}, {0, q1, 0}, {0, 0, q2}};
    double L0 = 0.0, L1 = 0.0, L2 = 0.0;
    for (int i = 0; i < 4000; ++i) {
        const double S = P[0][0] + r;
        const double l0 = P[0][0] / S;
        const double l1 = P[1][0] / S;
        const double l2 = P[2][0] / S;

        // Posterior Pp = P - L·row0(P)
        double Pp[3][3];
        const double lrow[3] = {l0, l1, l2};
        for (int a = 0; a < 3; ++a)
            for (int c = 0; c < 3; ++c)
                Pp[a][c] = P[a][c] - lrow[a] * P[0][c];

        // A-priori P⁻ = Ad·Pp·Ad' + Q  (Q diagonal)
        const double Ad[3][3] = {{ad00, ad01, ad02},
                                 {ad10, ad11, ad12},
                                 {ad20, ad21, ad22}};
        double AP[3][3], N[3][3];
        for (int a = 0; a < 3; ++a)
            for (int c = 0; c < 3; ++c) {
                AP[a][c] = Ad[a][0] * Pp[0][c] + Ad[a][1] * Pp[1][c] +
                           Ad[a][2] * Pp[2][c];
            }
        for (int a = 0; a < 3; ++a)
            for (int c = 0; c < 3; ++c)
                N[a][c] = AP[a][0] * Ad[c][0] + AP[a][1] * Ad[c][1] +
                          AP[a][2] * Ad[c][2];
        N[1][1] += q1;
        N[2][2] += q2;

        const double err = std::max({std::abs(N[0][0] - P[0][0]),
                                     std::abs(N[0][1] - P[0][1]),
                                     std::abs(N[1][1] - P[1][1]),
                                     std::abs(N[1][2] - P[1][2]),
                                     std::abs(N[2][2] - P[2][2])});
        std::memcpy(P, N, sizeof(P));
        L0 = l0; L1 = l1; L2 = l2;
        if (err < 1e-13 * (1.0 + P[0][0] + P[1][1] + P[2][2])) break;
    }
    m_L0 = L0;
    m_L1 = L1;
    m_L2 = L2;
}

void ServoAxisObserver::reset(double theta0) {
    m_x0 = theta0;
    m_x1 = 0.0;
    m_x2 = 0.0;
}

void ServoAxisObserver::predict(double uDelayed, double dt) {
    if (dt <= 0.0) return;
    const double x0 = m_x0, x1 = m_x1, x2 = m_x2;
    m_x0 = x0 + dt * x1;
    m_x1 = x1 + dt * (m_ktOverJ * uDelayed - m_bOverJ * x1 - m_invJ * x2);
    // τ_d random walk: m_x2 unchanged
}

void ServoAxisObserver::correct(double y) {
    const double innov = y - m_x0;
    m_x0 += m_L0 * innov;
    m_x1 += m_L1 * innov;
    m_x2 += m_L2 * innov;
}

// ---------------------------------------------------------------------------
// ServoAxisLQIBackend
// ---------------------------------------------------------------------------

ServoAxisLQIBackend::ServoAxisLQIBackend(const ServoAxisParams& plant,
                                       const ServoAxisTuning& tuning,
                                       double dtDesign)
    : m_plant(plant), m_tuning(tuning), m_dtDesign(dtDesign) {
    redesign(dtDesign);
}

void ServoAxisLQIBackend::redesign(double dt) {
    m_dtDesign = std::max(dt, 1e-5);

    const double J = std::max(m_plant.inertia, 1e-9);
    const double b = std::max(m_plant.damping, 0.0);
    const double kt = std::max(std::abs(m_plant.torqueConstant), 1e-9);

    // Regulated plant: deviation state z = [θ−r, ω−ω_ref].
    //   ż = A_c z + B_c u_fb   (disturbance handled by DOB ff + integrator)
    //   A_c = [[0,1],[0,−b/J]], B_c = [0, Kt/J]ᵀ, C = [1,0]
    const double d = m_dtDesign;
    double Ad[4] = {1.0, d, 0.0, 1.0 - (b / J) * d};
    double Bd[2] = {0.0, (kt / J) * d};
    double C[2]  = {1.0, 0.0};

    // Discrete design consistent with computeImpl()'s xi += e·dt update.
    m_lqi.setDiscreteSystemMatrices(Ad, Bd, C, 2, 1, 1, m_dtDesign);

    double Qx[4] = {m_tuning.qPos, 0.0, 0.0, m_tuning.qVel};
    double Qi[1] = {m_tuning.qInt};
    double R[1]  = {m_tuning.r};
    m_lqi.setWeights(Qx, Qi, R);

    m_designed = m_lqi.design();

    // Observer gain from the same physical model.
    m_obs.design(m_plant, m_dtDesign, m_tuning.qAccel, m_tuning.qDist,
                 m_tuning.rMeas);

    // Bound the integrator so |Ki·xi| can never demand more than ~1.2x
    // the available authority — anti-windup without a tracking-time
    // constant. xi_ss ≈ u*/|Ki| (~uMax/|Ki|) so the bound must exceed the
    // steady-state requirement or the clamp becomes a static offset.
    if (m_designed) {
        double Ki[1] = {0.0};
        m_lqi.getIntegralGain(Ki);
        const double auth = 1.2 * std::max({std::abs(m_plant.uMax),
                                            std::abs(m_plant.uMin), 1e-9});
        double xiMax = auth / std::max(std::abs(Ki[0]), 1e-9);
        xiMax = std::clamp(xiMax, 1.0, 1e9);
        m_lqi.setIntegralLimits(-xiMax, xiMax);
    }
}

bool ServoAxisLQIBackend::updateAxisParameters(double inertia,
                                               double damping,
                                               double torqueConstant,
                                               double coulombFriction) {
    ServoAxisParams p = m_plant;
    p.inertia = inertia;
    p.damping = damping;
    p.torqueConstant = torqueConstant;
    p.coulombFriction = coulombFriction;
    if (!p.valid()) return false;
    m_plant = p;
    redesign(m_dtDesign);
    return true;
}

bool ServoAxisLQIBackend::setPlant(const ServoAxisParams& plant) {
    if (!plant.valid()) return false;
    m_plant = plant;
    redesign(m_dtDesign);
    return true;
}

void ServoAxisLQIBackend::setTuning(const ServoAxisTuning& tuning) {
    m_tuning = tuning;
    redesign(m_dtDesign);
}

void ServoAxisLQIBackend::notifyAppliedInput(double u) {
    m_uApplied = u;
    m_notified = true;
}

void ServoAxisLQIBackend::pushApplied(double u) {
    m_uBuf[m_uHead] = u;
    m_uHead = (m_uHead + 1) % kDelayBufSize;
    if (m_uCount < kDelayBufSize) ++m_uCount;
}

double ServoAxisLQIBackend::delayedApplied(double theta) const {
    if (theta <= 0.0 || m_uCount == 0) {
        // No delay (or no history yet): the plant sees the most recent
        // applied input — before any output, that's 0.
        return (m_uCount > 0) ? m_uBuf[(m_uHead + kDelayBufSize - 1) % kDelayBufSize]
                              : 0.0;
    }
    int taps = static_cast<int>(std::lround(theta / m_dtDesign));
    taps = std::clamp(taps, 1, m_uCount);
    const int idx = (m_uHead + kDelayBufSize - taps) % kDelayBufSize;
    return m_uBuf[idx];
}

double ServoAxisLQIBackend::frictionFeedforward(double velRef) const {
    // Coulomb compensation on the *reference* velocity (not the noisy
    // measured ω̂): tanh deadband smooths through zero-crossings without
    // chatter. Physical parameter τ_c — not a gain.
    if (m_plant.coulombFriction <= 0.0) return 0.0;
    const double veps = std::max(m_velEps, 1e-9);
    return (m_plant.coulombFriction * std::tanh(velRef / veps)) /
           std::max(std::abs(m_plant.torqueConstant), 1e-9);
}

ControllerOutput ServoAxisLQIBackend::computeImpl(const ControllerInput& input) {
    ControllerOutput out;
    if (!m_designed) {
        out.control = m_uLast;
        return out;
    }
    const double dt = (input.dt > 0.0) ? input.dt : m_dtDesign;

    // Lazy observer init on first measurement.
    if (!m_obsInit) {
        m_obs.reset(input.measured);
        m_obsInit = true;
    } else {
        // Observer sees the TOTAL applied input — via notifyAppliedInput
        // when a wrapper reports it, else the backend's own last output.
        const double uObs = m_notified ? m_uApplied : m_uLast;
        m_notified = false;
        pushApplied(uObs);
        m_obs.predict(delayedApplied(m_plant.delay), dt);
    }
    m_obs.correct(input.measured);

    // Q-filter the disturbance estimate before using it as feedforward:
    // a raw −τ̂_d/Kt would inject observer convergence transients into
    // the actuator. First-order low-pass at the DOB bandwidth.
    {
        const double a = std::min(1.0, m_dobBw * dt);
        m_dobTau += (m_obs.disturbanceTorque() - m_dobTau) * a;
    }

    const double J = std::max(m_plant.inertia, 1e-9);
    const double kt = std::max(std::abs(m_plant.torqueConstant), 1e-9);

    // Model-based feedforward in command units:
    //   u_ff = (J·α_ref + b·ω_ref)/Kt + τ_c·tanh(ω_ref/vε)/Kt + u_ff_ext
    //        + τ̂_d/Kt                                    (disturbance FF)
    // τ̂_d estimates the *decelerating* disturbance torque (the observer
    // predicts ω̇ = Kt·u/J − b·ω/J − τ̂_d/J), so cancellation ADDS +τ̂_d/Kt
    // to the command — subtracting would double the disturbance.
    // All physical parameters; τ̂_d cancels load torque without waiting
    // for the integrator.
    const double r     = input.reference;
    const double wRef  = input.referenceDerivative;
    const double alRef = input.referenceAccel;
    double u_ff = (J * alRef + m_plant.damping * wRef) / kt +
                frictionFeedforward(wRef) + input.feedforward;
    if (m_dobFf) u_ff += m_dobTau / kt;

    // LQI on deviation states: the integrator sees xi += (0 − C·z)·dt =
    // (r − θ̂)·dt — the physical position error.
    //
    // Anti-windup (see ServoAxisAWMode):
    // - kConditional: pass dt=0 to freeze the integrator while the loop
    //   is saturated in the error's direction (LQIController uses dt
    //   only for xi accumulation — state feedback stays live).
    // - kBackCalculation: LQIController's native tracking term
    //   xi -= (u_sat − u_desat)·dt/(Ki·Tt), applied inside compute.
    const double e = r - m_obs.position();
    const bool pushingIntoSat =
        (m_saturatedHigh && e > 0.0) || (m_saturatedLow && e < 0.0);
    double liDt = dt;
    switch (m_awMode) {
        case ServoAxisAWMode::kConditional:
            if (pushingIntoSat) liDt = 0.0;
            break;
        case ServoAxisAWMode::kBackCalculation:
        case ServoAxisAWMode::kNone:
            break;
    }
    m_lqi.setBackCalculationTime(
        m_awMode == ServoAxisAWMode::kBackCalculation ? m_awTt : 0.0);
    ControllerInput li;
    li.state[0] = m_obs.position() - r;
    li.state[1] = m_obs.velocity() - wRef;
    li.stateDim = 2;
    li.reference = 0.0;
    li.measured = input.measured;
    li.dt = liDt;

    SaturationLimits ll;
    ll.outputMin = m_plant.uMin - u_ff;
    ll.outputMax = m_plant.uMax - u_ff;
    m_lqi.setSaturationLimits(ll);

    ControllerOutput o = m_lqi.compute(li);
    const double u_fb = o.control;

    out.control = u_ff + u_fb;

    // Optional output slew-rate limiting (actuator protection). Applies
    // to the TOTAL command so feedforward steps cannot outrun the drive.
    // A slew limit tighter than the closed loop's demand is itself a
    // saturation — it must flag for conditional integration below.
    if (m_rateLimit > 0.0) {
        const double duMax = m_rateLimit * dt;
        out.control =
            std::clamp(out.control, m_uLast - duMax, m_uLast + duMax);
    }
    out.control = std::clamp(out.control, m_plant.uMin, m_plant.uMax);

    // Track saturation direction for next cycle's conditional
    // integration — both the LQI's remaining-authority rail AND any
    // clipping applied to the total command (slew/absolute limits).
    m_saturatedHigh =
        (std::isfinite(ll.outputMax) && u_fb >= ll.outputMax - 1e-12) ||
        out.control < u_ff + u_fb - 1e-12;
    m_saturatedLow =
        (std::isfinite(ll.outputMin) && u_fb <= ll.outputMin + 1e-12) ||
        out.control > u_ff + u_fb + 1e-12;

    out.feedforward = u_ff;
    out.error = r - m_obs.position();
    out.integral = o.integral;
    out.saturated = m_saturatedHigh || m_saturatedLow;
    m_uLast = out.control;
    return out;
}

void ServoAxisLQIBackend::resetImpl() {
    m_obs.reset(0.0);
    m_obsInit = false;
    m_uHead = 0;
    m_uCount = 0;
    m_uApplied = 0.0;
    m_uLast = 0.0;
    m_notified = false;
    m_saturatedHigh = false;
    m_saturatedLow = false;
    m_dobTau = 0.0;
    m_lqi.reset();
}

bool ServoAxisLQIBackend::getGain(double* Kx, double* Ki) const {
    if (!m_designed) return false;
    m_lqi.getStateGain(Kx);
    m_lqi.getIntegralGain(Ki);
    return true;
}

} // namespace tether::control::motion

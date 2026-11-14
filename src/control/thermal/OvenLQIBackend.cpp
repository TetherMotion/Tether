#include "tether/control/thermal/OvenLQIBackend.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>

namespace tether::control::thermal {

// ---------------------------------------------------------------------------
// OvenThermalObserver — steady-state Kalman filter, C = [1, 0]
// ---------------------------------------------------------------------------

void OvenThermalObserver::design(const OvenCoupledParams& plant,
                                 double dtDesign, double qProc,
                                 double rMeas) {
    const double bs = std::max(plant.betaSensor, 1e-9);
    const double bp = std::max(plant.betaProduct, 1e-9);
    const double gsp = std::max(plant.gammaSP, 0.0);
    const double gps = std::max(plant.gammaPS, 0.0);

    m_a11 = -(bs + gsp);
    m_a12 = gsp;
    m_a21 = gps;
    m_a22 = -(gps + bp);
    m_b1 = std::max(plant.alpha, 1e-9);
    m_c1 = bs * plant.ambient;
    m_c2 = bp * plant.ambient + plant.disturbance;
    m_ambient = plant.ambient;

    // Discrete model at the design step (Euler — exact enough for thermal
    // timescales where dt << tau).
    const double dt = std::max(dtDesign, 1e-3);
    const double ad11 = 1.0 + m_a11 * dt, ad12 = m_a12 * dt;
    const double ad21 = m_a21 * dt,     ad22 = 1.0 + m_a22 * dt;

    // Noise covariances (discrete white-noise acceleration form)
    const double q = std::max(qProc, 0.0) * dt;
    const double r = std::max(rMeas, 1e-12);

    // Iterate the predictor Riccati equation to steady state:
    //   S  = P00 + R
    //   L  = P·C' / S            (C = [1,0] → column 0 of P)
    //   Pp = P - L·(C·P)         (posterior covariance)
    //   P⁻ = Ad·Pp·Ad' + Q
    double p00 = q, p01 = 0.0, p11 = q;
    double L1 = 0.0, L2 = 0.0;
    for (int i = 0; i < 2000; ++i) {
        const double S = p00 + r;
        const double l1 = p00 / S;
        const double l2 = p01 / S;   // P is symmetric: P[1][0] = p01

        // Posterior covariance Pp = P - L * row0(P)
        const double pp00 = p00 - l1 * p00;
        const double pp01 = p01 - l1 * p01;
        const double pp11 = p11 - l2 * p01;

        // A-priori: P⁻ = Ad·Pp·Ad' + Q
        // APp = Ad·Pp
        const double ap00 = ad11 * pp00 + ad12 * pp01;
        const double ap01 = ad11 * pp01 + ad12 * pp11;
        const double ap10 = ad21 * pp00 + ad22 * pp01;
        const double ap11 = ad21 * pp01 + ad22 * pp11;
        // P⁻ = APp·Ad'
        const double n00 = ap00 * ad11 + ap01 * ad12 + q;
        const double n01 = ap00 * ad21 + ap01 * ad22;
        const double n11 = ap10 * ad21 + ap11 * ad22 + q;

        const double err = std::max({std::abs(n00 - p00),
                                     std::abs(n01 - p01),
                                     std::abs(n11 - p11)});
        p00 = n00; p01 = n01; p11 = n11;
        L1 = l1; L2 = l2;
        if (err < 1e-13 * (1.0 + p00 + p11)) break;
    }
    m_L1 = L1;
    m_L2 = L2;
}

void OvenThermalObserver::reset(double y0) {
    m_x1 = y0;
    m_x2 = y0;
}

void OvenThermalObserver::predict(double uDelayed, double dt) {
    if (dt <= 0.0) return;
    const double x1o = m_x1, x2o = m_x2;
    m_x1 = x1o + dt * (m_a11 * x1o + m_a12 * x2o + m_b1 * uDelayed + m_c1);
    m_x2 = x2o + dt * (m_a21 * x1o + m_a22 * x2o + m_c2);
}

void OvenThermalObserver::correct(double y) {
    const double innov = y - m_x1;
    m_x1 += m_L1 * innov;
    m_x2 += m_L2 * innov;
}

// ---------------------------------------------------------------------------
// OvenLQIBackend
// ---------------------------------------------------------------------------

OvenLQIBackend::OvenLQIBackend(const OvenCoupledParams& plant,
                               double qxSensor, double qxProduct,
                               double qi, double r,
                               double qProc, double rMeas,
                               double dtDesign)
    : m_plant(plant),
      m_qxSensor(qxSensor), m_qxProduct(qxProduct),
      m_qi(qi), m_r(r),
      m_qProc(qProc), m_rMeas(rMeas), m_dtDesign(dtDesign) {
    redesign(dtDesign);
}

void OvenLQIBackend::redesign(double dt) {
    m_dtDesign = std::max(dt, 1e-3);

    const double bs = std::max(m_plant.betaSensor, 1e-9);
    const double bp = std::max(m_plant.betaProduct, 1e-9);
    const double gsp = std::max(m_plant.gammaSP, 0.0);
    const double gps = std::max(m_plant.gammaPS, 0.0);
    const double alpha = std::max(m_plant.alpha, 1e-9);

    // Continuous A, B → Euler-discretized Ad, Bd at the design step.
    const double a11 = -(bs + gsp), a12 = gsp;
    const double a21 = gps,            a22 = -(gps + bp);
    double Ad[4] = {1.0 + a11 * m_dtDesign, a12 * m_dtDesign,
                    a21 * m_dtDesign,     1.0 + a22 * m_dtDesign};
    double Bd[2] = {alpha * m_dtDesign, 0.0};
    // Controlled output: product-zone temperature → C = [0, 1]
    double C[2] = {0.0, 1.0};

    m_lqi.setSystemMatrices(Ad, Bd, C, 2, 1, 1);

    double Qx[4] = {m_qxSensor, 0.0, 0.0, m_qxProduct};
    double Qi[1] = {m_qi};
    double R[1]  = {m_r};
    m_lqi.setWeights(Qx, Qi, R);

    m_designed = m_lqi.design();

    // Observer gain from the same physical model.
    m_obs.design(m_plant, m_dtDesign, m_qProc, m_rMeas);

    // Bound the integrator so |Ki·xᵢ| can never demand more than ~1.2x the
    // available authority — anti-windup without a tracking-time constant.
    // NOTE: the bound must exceed the steady-state requirement
    // xi_ss ≈ u*/|Ki| (~uMax/|Ki|), otherwise the clamp itself becomes a
    // steady-state offset.
    if (m_designed) {
        double Ki[1] = {0.0};
        m_lqi.getIntegralGain(Ki);
        const double auth = 1.2 * std::max({std::abs(m_plant.uMax),
                                            std::abs(m_plant.uMin), 1e-9});
        double xiMax = auth / std::max(std::abs(Ki[0]), 1e-9);
        xiMax = std::clamp(xiMax, 1.0, 1e6);
        m_lqi.setIntegralLimits(-xiMax, xiMax);
    }
}

void OvenLQIBackend::updateThermalParameters(double alpha, double beta,
                                             double disturbance) {
    // The estimator identifies the sensor-path FOPDT parameters. Map them
    // onto the coupled model: alpha and the dominant loss carry to zone 1,
    // the product-zone loss keeps its configured ratio to betaSensor.
    // Coupling rates and the loss split are structural (capacitance ratio)
    // — kept fixed; refine via setPlant()/recommissioning if they drift.
    //
    // NOTE: `disturbance` is deliberately NOT mapped. The FOPDT d-hat
    // conflates true exogenous loads with structural mismatch between the
    // single-state regression model and the coupled plant (product-zone
    // losses appear as a phantom endothermic load). Injecting it here
    // double-counts zone-2 losses and corrupts the observer. True load
    // disturbances are rejected by the LQI integral action instead —
    // which is precisely what it is for. m_plant.disturbance remains a
    // commissioning-set prior for KNOWN persistent loads.
    (void)disturbance;
    const double ratio = (m_plant.betaSensor > 1e-12)
        ? m_plant.betaProduct / m_plant.betaSensor : 0.3;
    m_plant.alpha = std::max(alpha, 1e-9);
    m_plant.betaSensor = std::max(beta, 1e-9);
    m_plant.betaProduct = m_plant.betaSensor * ratio;
    redesign(m_dtDesign);
}

void OvenLQIBackend::notifyAppliedInput(double u) {
    m_uApplied = u;
}

void OvenLQIBackend::pushApplied(double u) {
    m_uBuf[m_uHead] = u;
    m_uHead = (m_uHead + 1) % kDelayBufSize;
    if (m_uCount < kDelayBufSize) ++m_uCount;
}

double OvenLQIBackend::delayedApplied(double theta) const {
    if (theta <= 0.0 || m_uCount == 0) {
        // No delay (or no history yet): the plant sees the most recent
        // applied input — but before any notify, that's the initial 0.
        return (m_uCount > 0) ? m_uBuf[(m_uHead + kDelayBufSize - 1) % kDelayBufSize]
                              : 0.0;
    }
    // Approximate taps from the design step; exact enough for thermal dt.
    int taps = static_cast<int>(std::lround(theta / m_dtDesign));
    taps = std::clamp(taps, 1, m_uCount);
    const int idx = (m_uHead + kDelayBufSize - taps) % kDelayBufSize;
    return m_uBuf[idx];
}

ControllerOutput OvenLQIBackend::computeImpl(const ControllerInput& input) {
    ControllerOutput out;
    if (!m_designed) {
        out.control = m_uFbLast;
        return out;
    }
    const double dt = (input.dt > 0.0) ? input.dt : m_dtDesign;

    // Lazy observer init on first measurement.
    if (!m_obsInit) {
        m_obs.reset(input.measured);
        m_obsInit = true;
    } else {
        // Observer sees the TOTAL applied input (FF+FB+dither, post-clamp)
        // delayed by the plant dead time — feedforward is therefore not
        // misinterpreted as process noise.
        pushApplied(m_uApplied);
        m_obs.predict(delayedApplied(m_plant.delay), dt);
    }
    m_obs.correct(input.measured);

    // Feed the LQI DEVIATION states z = x̂ - x_ss(r), not absolute
    // temperatures. Tether's LQI penalizes absolute state in its ARE, so
    // feeding ~100 °C absolutes would produce a ~10^4-unit bias that the
    // integrator can never cancel (output pinned at the rail). With z the
    // regulated variable is the tracking error: the integrator sees
    // xi = ∫(0 - C·z) = ∫(r - x̂_p) — exactly the product-temperature
    // error — and Kx·z supplies corrective action on small deviations.
    //
    // x_ss(r): zone-2 steady-state balance gives the sensor-product
    // gradient; zone-2 itself sits at the reference by definition.
    //   0 = γPS·(x1-x2) - βP·(x2-Tamb) + d   →
    //   x1 - x2 = (βP·(x2-Tamb) - d)/γPS,   x2 = r
    const double r = input.reference;
    const double bp = std::max(m_plant.betaProduct, 1e-9);
    const double gps = std::max(m_plant.gammaPS, 1e-6);
    double grad = (bp * (r - m_plant.ambient) - m_plant.disturbance) / gps;
    grad = std::clamp(grad, -200.0, 200.0);   // guard degenerate coupling
    const double x1ss = r + grad;

    ControllerInput li;
    li.state[0] = m_obs.xSensor() - x1ss;
    li.state[1] = m_obs.xProduct() - r;
    li.reference = 0.0;          // deviation coordinates: regulate z -> 0
    li.measured = input.measured;
    li.dt = dt;
    li.feedforward = input.feedforward;   // u_ff — used for dynamic limits

    // Mirror the PI backend's remaining-authority limits so the LQI
    // output is clamped to [uMin-u_ff, uMax-u_ff] before the outer sum.
    SaturationLimits ll;
    ll.outputMin = m_plant.uMin - input.feedforward;
    ll.outputMax = m_plant.uMax - input.feedforward;
    m_lqi.setSaturationLimits(ll);

    ControllerOutput o = m_lqi.compute(li);
    m_uFbLast = o.control;
    out.control = o.control;
    out.error = input.reference - m_obs.xProduct();
    out.integral = o.integral;
    out.saturated = o.saturated;
    return out;
}

void OvenLQIBackend::resetImpl() {
    m_obs.reset(m_plant.ambient);
    m_obsInit = false;
    m_uHead = 0;
    m_uCount = 0;
    m_uApplied = 0.0;
    m_uFbLast = 0.0;
    m_lqi.reset();
}

bool OvenLQIBackend::getGain(double* Kx, double* Ki) const {
    if (!m_designed) return false;
    m_lqi.getStateGain(Kx);
    m_lqi.getIntegralGain(Ki);
    return true;
}

} // namespace tether::control::thermal

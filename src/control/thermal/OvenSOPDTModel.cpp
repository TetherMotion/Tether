#include "tether/control/thermal/OvenSOPDTModel.hpp"

#include <algorithm>
#include <cmath>
#include <utility>

namespace tether::control::thermal {

namespace {

constexpr double kZohInt = 1e-9;

} // namespace

void detail::CascadeZoh::init(double beta, double tau2, double dt) {
    if (tau2 <= kZohInt) {
        // Locked stages: single FOPDT mass, y2 = y1.
        if (beta > kZohInt) {
            m_a1 = std::exp(-beta * dt);
            m_cv1 = (1.0 - m_a1) / beta;
        } else {
            m_a1 = 1.0;
            m_cv1 = dt;
        }
        m_e21 = m_a1;
        m_a2 = 0.0;
        m_cv2 = m_cv1;
        return;
    }

    if (beta <= kZohInt) {
        // Stage-1 pure integrator: y1 ramps at rate v; y2 sees a ramp.
        const double a2 = std::exp(-dt / tau2);
        m_a1 = 1.0;
        m_cv1 = dt;
        m_e21 = 1.0 - a2;
        m_a2 = a2;
        m_cv2 = dt - tau2 * (1.0 - a2);
        return;
    }

    const double lam2 = 1.0 / tau2;
    const double a1 = std::exp(-beta * dt);
    const double a2 = std::exp(-lam2 * dt);

    if (std::abs(lam2 - beta) < 1e-7 * std::max(beta, lam2)) {
        // Resonant case beta == 1/tau2 = lambda.
        // e21(t) = lambda*t*exp(-lambda*t); the input convolution is
        // int_0^dt lambda*(dt-s)*exp(-lambda*(dt-s)) ds
        //         = int_0^dt lambda*u*exp(-lambda*u) du
        //         = (1 - exp(-lambda*dt)*(1 + lambda*dt)) / lambda
        const double lam = 0.5 * (beta + lam2);
        const double al = std::exp(-lam * dt);
        m_a1 = al;
        m_cv1 = (1.0 - al) / lam;
        m_e21 = lam * dt * al;
        m_a2 = a2;
        m_cv2 = (1.0 - al * (1.0 + lam * dt)) / lam;
        return;
    }

    // General case.
    m_a1 = a1;
    m_cv1 = (1.0 - a1) / beta;
    m_e21 = lam2 * (a1 - a2) / (lam2 - beta);
    m_a2 = a2;
    // conv = int_0^dt (1/tau2)(e^{-beta s}-e^{-s/tau2})/(1/tau2-beta) * v ds
    //      = v * (i1 - i2) / (1 - beta*tau2)
    const double i1 = (1.0 - a1) / beta;          // int_0^dt e^{-beta s} ds
    const double i2 = tau2 * (1.0 - a2);          // int_0^dt e^{-s/tau2} ds
    m_cv2 = (i1 - i2) / (1.0 - beta * tau2);
}

namespace {

/**
 * @brief Exact zero-order-hold step of the cascade in deviation
 *        coordinates y_i = T_i - T_amb (see detail::CascadeZoh).
 */
std::pair<double, double> cascadeZohStep(double y1, double y2, double v,
                                         double dt, double beta,
                                         double tau2) {
    detail::CascadeZoh z;
    z.init(beta, tau2, dt);
    return z.step(y1, y2, v);
}

} // namespace

double OvenThermalModelSOPDT::step(double u, double dt) {
    if (dt <= 0.0) return m_T2;

    m_delayLine.push(m_time, u);
    const double uDelayed = m_delayLine.value(m_time - m_params.delay);
    m_time += dt;

    const double v = m_params.alpha * uDelayed + m_params.disturbance;
    const double amb = m_params.ambient;

    auto [y1, y2] = cascadeZohStep(m_T1 - amb, m_T2 - amb, v, dt,
                                   m_params.beta, m_params.tau2);
    m_T1 = amb + y1;
    m_T2 = amb + y2;
    return m_T2;
}

std::vector<double> OvenThermalModelSOPDT::simulate(double uConst, double dt,
                                                  int steps, double tAmb,
                                                  double d) const {
    std::vector<double> out;
    out.reserve(static_cast<size_t>(steps) + 1);
    out.push_back(m_T2);

    const int nDelay = m_params.delay > 0.0 && dt > 0.0
        ? static_cast<int>(std::lround(m_params.delay / dt)) : 0;
    std::deque<double> uHist(static_cast<size_t>(nDelay) + 1, 0.0);

    double y1 = m_T1 - tAmb, y2 = m_T2 - tAmb;
    for (int k = 0; k < steps; ++k) {
        double uEff = uHist.front();
        uHist.pop_front();
        uHist.push_back(uConst);
        const double v = m_params.alpha * uEff + d;
        auto [n1, n2] = cascadeZohStep(y1, y2, v, dt, m_params.beta,
                                       m_params.tau2);
        y1 = n1;
        y2 = n2;
        out.push_back(tAmb + y2);
    }
    return out;
}

double OvenThermalModelSOPDT::requiredInput(double tRef, double tRefDot,
                                            double tRefAcc, double ambient,
                                            double disturb) const {
    // Stage-2 tracking r requires stage 1 to follow T1_req = r + τ2·ṙ.
    // Stage-1 (FOPDT) inverse at (T1_req, dT1_req/dt):
    //   α·u = dT1/dt + β·(T1 − T_amb) − d
    //   dT1/dt = ṙ + τ2·r̈
    const double alpha = m_params.alpha;
    if (alpha <= 1e-12) return m_params.uMin;

    const double tau2 = std::max(m_params.tau2, 0.0);
    const double t1Req = tRef + tau2 * tRefDot;
    const double dt1Req = tRefDot + tau2 * tRefAcc;
    const double loss = m_params.beta * (t1Req - ambient);
    return (dt1Req + loss - disturb) / alpha;
}

double OvenThermalModelSOPDT::requiredInputClamped(double tRef,
                                                   double tRefDot,
                                                   double tRefAcc,
                                                   double ambient,
                                                   double disturb) const {
    return std::clamp(requiredInput(tRef, tRefDot, tRefAcc, ambient, disturb),
                      m_params.uMin, m_params.uMax);
}

double OvenThermalModelSOPDT::requiredInputDominantLag(double tRef,
                                                       double tRefDot,
                                                       double ambient,
                                                       double disturb) const {
    // Ignore the τ2 compensation: invert stage 1 as if the output were
    // T1. Produces a feasible but laggier command — feedback trims.
    const double alpha = m_params.alpha;
    if (alpha <= 1e-12) return m_params.uMin;
    const double loss = m_params.beta * (tRef - ambient);
    return (tRefDot + loss - disturb) / alpha;
}

} // namespace tether::control::thermal

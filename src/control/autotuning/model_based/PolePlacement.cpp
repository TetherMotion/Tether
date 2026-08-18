#include "tether/control/autotuning/model_based/PolePlacement.hpp"
#include <cmath>
#include <algorithm>

namespace tether::control {
namespace Autotuning {

bool PolePlacement::isCompatible(const TunableController& controller) const {
    auto name = controller.getControllerTypeName();
    return name.find("State") != std::string::npos ||
           name.find("PID") != std::string::npos ||
           name.find("LQR") != std::string::npos;
}

TuningResult PolePlacement::tune(TunableController& controller,
                                 const ProcessModel* model) {
    TuningResult result;

    if (m_desiredPoles.empty()) {
        result.success = false;
        result.message = "No desired poles specified";
        return result;
    }

    if (model && m_A.empty()) {
        auto fopdt = model->toFOPDT();
        m_n = 2;
        m_m = 1;
        m_A.resize(m_n * m_n);
        m_B.resize(m_n * m_m);

        double tau = fopdt.tau;
        double K = fopdt.K;
        double L = fopdt.L;

        m_A[0] = 0.0;
        m_A[1] = 1.0;
        m_A[2] = -2.0 / (tau * L + 0.01);
        m_A[3] = -(2.0*tau + L) / (tau * L + 0.01);

        m_B[0] = 0.0;
        m_B[1] = 2.0 * K / (tau * L + 0.01);
    }

    if (m_A.empty() || m_B.empty()) {
        result.success = false;
        result.message = "No system matrices specified";
        return result;
    }

    std::vector<double> K_fb = computeGain(m_A.data(), m_B.data(), 
                                           m_n, m_desiredPoles);

    if (K_fb.empty()) {
        result.success = false;
        result.message = "Pole placement computation failed";
        return result;
    }

    PIDGains gains = stateFeedbackToPID(K_fb, m_n);
    double Ki = (gains.Ti > 0) ? gains.Kp / gains.Ti : 0.0;
    double Kd = gains.Kp * gains.Td;

    result.parameters = {gains.Kp, Ki, Kd};
    result.success = true;
    result.message = "Pole placement complete";

    controller.setParameters(result.parameters);

    return result;
}

void PolePlacement::setDesiredPoles(const std::vector<std::complex<double>>& poles) {
    m_desiredPoles = poles;
}

void PolePlacement::setPolesFromSpecs(double settlingTime, double overshoot) {
    double zeta;
    if (overshoot <= 0) {
        zeta = 1.0;
    } else {
        zeta = 0.5;
        for (int i = 0; i < 20; i++) {
            double mp_calc = std::exp(-M_PI * zeta / std::sqrt(1.0 - zeta*zeta + 0.001));
            double error = mp_calc - overshoot;
            zeta += 0.05 * error;
            zeta = std::clamp(zeta, 0.1, 0.999);
        }
    }

    double wn = 4.0 / (zeta * settlingTime);
    double realPart = -zeta * wn;
    double imagPart = wn * std::sqrt(1.0 - zeta*zeta);

    m_desiredPoles.clear();
    m_desiredPoles.push_back({realPart, imagPart});
    m_desiredPoles.push_back({realPart, -imagPart});
}

std::vector<double> PolePlacement::computeGain(const double* A, const double* B,
                                                int n, const std::vector<std::complex<double>>& poles) {
    if (n <= 0 || poles.size() < static_cast<size_t>(n)) {
        return {};
    }

    if (n != 2) {
        return {};
    }

    double b1 = B[0], b2 = B[1];
    double a11 = A[0], a12 = A[1], a21 = A[2], a22 = A[3];

    double Wc[4];
    Wc[0] = b1;
    Wc[1] = a11*b1 + a12*b2;
    Wc[2] = b2;
    Wc[3] = a21*b1 + a22*b2;

    double det = Wc[0]*Wc[3] - Wc[1]*Wc[2];
    if (std::abs(det) < 1e-10) {
        return {};
    }

    double WcInv[4];
    WcInv[0] = Wc[3] / det;
    WcInv[1] = -Wc[1] / det;
    WcInv[2] = -Wc[2] / det;
    WcInv[3] = Wc[0] / det;

    std::complex<double> sum = poles[0] + poles[1];
    std::complex<double> prod = poles[0] * poles[1];
    double alpha1 = -sum.real();
    double alpha0 = prod.real();

    double A2[4];
    A2[0] = a11*a11 + a12*a21;
    A2[1] = a11*a12 + a12*a22;
    A2[2] = a21*a11 + a22*a21;
    A2[3] = a21*a12 + a22*a22;

    double alphaA[4];
    alphaA[0] = A2[0] + alpha1*a11 + alpha0;
    alphaA[1] = A2[1] + alpha1*a12;
    alphaA[2] = A2[2] + alpha1*a21;
    alphaA[3] = A2[3] + alpha1*a22 + alpha0;

    double k1 = WcInv[2]*alphaA[0] + WcInv[3]*alphaA[2];
    double k2 = WcInv[2]*alphaA[1] + WcInv[3]*alphaA[3];

    return {k1, k2};
}

PIDGains PolePlacement::stateFeedbackToPID(const std::vector<double>& K, int order) {
    PIDGains gains;

    if (order == 2 && K.size() >= 2) {
        gains.Kp = K[0];
        gains.Ti = 0.0;
        gains.Td = K[1] / (K[0] + 0.01);
        gains.Ti = 5.0 / K[0];
    }

    return gains;
}

void PolePlacement::setSystemMatrices(const double* A, const double* B, int n, int m) {
    m_n = n;
    m_m = m;
    m_A.assign(A, A + n*n);
    m_B.assign(B, B + n*m);
}

} // namespace Autotuning
} // namespace tether::control

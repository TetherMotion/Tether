#include "tether/control/autotuning/model_based/LoopShaping.hpp"
#include <cmath>

namespace {

double clamp_double(double v, double lo, double hi) {
    if (v < lo) {
        return lo;
    }
    if (v > hi) {
        return hi;
    }
    return v;
}

} // namespace

namespace tether::control {
namespace Autotuning {

bool LoopShaping::isCompatible(const TunableController& controller) const {
    auto name = controller.getControllerTypeName();
    return name.find("PID") != std::string::npos ||
           name.find("Lead") != std::string::npos ||
           name.find("Lag") != std::string::npos;
}

TuningResult LoopShaping::tune(TunableController& controller,
                               const ProcessModel* model) {
    TuningResult result;
    if (!model) {
        result.success = false;
        result.message = "Process model required for loop shaping";
        return result;
    }

    const double gainMarginScale = clamp_double(
        std::pow(10.0, -(m_gainMargin - 6.0) / 20.0), 0.1, 10.0);
    const double lowFreqSlopeScale = clamp_double(
        1.0 + 0.1 * (static_cast<double>(m_lowFreqSlope) + 1.0), 0.1, 10.0);
    const double highFreqRolloffScale = clamp_double(
        1.0 + 0.05 * ((-static_cast<double>(m_highFreqRolloff)) - 2.0), 0.1, 10.0);

    auto fopdt = model->toFOPDT();
    double plantPhase = -std::atan(m_omegaC * fopdt.tau) - m_omegaC * fopdt.L;
    plantPhase *= 180.0 / M_PI;
    double phaseBoost = m_phaseMargin - (180.0 + plantPhase);

    double maxBoostRad = phaseBoost * M_PI / 180.0;
    if (maxBoostRad > 0) {
        double sinPhi = std::sin(maxBoostRad);
        double alpha = (1.0 + sinPhi) / (1.0 - sinPhi + 0.01);
        double omegaZ = m_omegaC / std::sqrt(alpha);
        double omegaP = m_omegaC * std::sqrt(alpha);

        double plantMag = fopdt.K / std::sqrt(1.0 + std::pow(m_omegaC * fopdt.tau, 2));
        double compMag = std::sqrt((1.0 + std::pow(m_omegaC/omegaZ, 2)) / 
                                   (1.0 + std::pow(m_omegaC/omegaP, 2)));
        double Kc = (1.0 / (plantMag * compMag)) * gainMarginScale * lowFreqSlopeScale * highFreqRolloffScale;

        double Kp = Kc;
        double Td = 1.0 / omegaZ - 1.0 / omegaP;
        double Ti = 10.0 / m_omegaC;

        double Ki = Kp / Ti;
        double Kd_val = Kp * Td;

        result.parameters = {Kp, Ki, Kd_val};
        result.success = true;
        result.message = "Loop shaping complete";
        result.phaseMargin = m_phaseMargin;

        controller.setParameters(result.parameters);
    } else {
        double Kp = (1.0 / fopdt.K) * gainMarginScale * lowFreqSlopeScale * highFreqRolloffScale;
        double Ti = 10.0 / m_omegaC;

        result.parameters = {Kp, Kp/Ti, 0.0};
        result.success = true;
        result.message = "Loop shaping (lag only)";

        controller.setParameters(result.parameters);
    }

    return result;
}

TransferFunction LoopShaping::designLeadLag(const ProcessModel& plant,
                                            double omegaC, double phaseMargin) {
    TransferFunction C;
    auto fopdt = plant.toFOPDT();
    double plantPhase = -std::atan(omegaC * fopdt.tau) - omegaC * fopdt.L;
    plantPhase *= 180.0 / M_PI;
    double phaseBoost = phaseMargin - (180.0 + plantPhase);

    if (phaseBoost > 0) {
        double maxBoostRad = phaseBoost * M_PI / 180.0;
        double sinPhi = std::sin(maxBoostRad);
        double alpha = (1.0 + sinPhi) / (1.0 - sinPhi + 0.01);

        double omegaZ = omegaC / std::sqrt(alpha);
        double omegaP = omegaC * std::sqrt(alpha);

        double plantMag = fopdt.K / std::sqrt(1.0 + std::pow(omegaC * fopdt.tau, 2));
        double compMag = std::sqrt((1.0 + std::pow(omegaC/omegaZ, 2)) / 
                                   (1.0 + std::pow(omegaC/omegaP, 2)));
        double Kc = 1.0 / (plantMag * compMag);

        C.num = {Kc / omegaZ, Kc};
        C.den = {1.0 / omegaP, 1.0};
    } else {
        C.num = {1.0};
        C.den = {1.0};
    }

    return C;
}

} // namespace Autotuning
} // namespace tether::control

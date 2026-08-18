#include "tether/control/autotuning/model_based/DirectSynthesis.hpp"

namespace tether::control {
namespace Autotuning {

bool DirectSynthesis::isCompatible(const TunableController& controller) const {
    auto name = controller.getControllerTypeName();
    return name.find("PID") != std::string::npos;
}

TuningResult DirectSynthesis::tune(TunableController& controller,
                                   const ProcessModel* model) {
    TuningResult result;
    if (!model) {
        result.success = false;
        result.message = "Process model required for direct synthesis";
        return result;
    }

    auto fopdt = model->toFOPDT();
    // Even for FOPDT models, allow the configured closed-loop damping to
    // influence the aggressiveness of the design. This keeps the API behavior
    // consistent for callers who set a damping target.
    const double zeta = (m_zetaCL > 1e-6) ? m_zetaCL : 1e-6;
    const double tauCL_eff = m_tauCL * std::clamp(1.0 / zeta, 0.5, 2.0);
    PIDGains gains = designForFOPDT(fopdt, tauCL_eff);

    double Ki = (gains.Ti > 0) ? gains.Kp / gains.Ti : 0.0;
    double Kd = gains.Kp * gains.Td;

    result.parameters = {gains.Kp, Ki, Kd};
    result.success = true;
    result.message = "Direct synthesis complete";
    result.settlingTime = 4.0 * tauCL_eff;

    controller.setParameters(result.parameters);

    return result;
}

PIDGains DirectSynthesis::designForFOPDT(const FOPDTModel& model, double tauCL) {
    PIDGains gains;
    double K = model.K;
    double tau = model.tau;
    double L = model.L;

    gains.Kp = tau / (K * (tauCL + L/2));
    gains.Ti = tau;
    gains.Td = L / 2;

    return gains;
}

PIDGains DirectSynthesis::designForSOPDT(const SOPDTModel& model, 
                                          double tauCL, double zetaCL) {
    PIDGains gains;
    double K = model.K;
    double tau1 = model.tau1;
    double tau2 = model.tau2;
    // double L = model.L; // Not used

    double tauSum = tau1 + tau2;
    double tauProd = tau1 * tau2;

    gains.Kp = tauSum / (K * 2 * zetaCL * tauCL);
    gains.Ti = tauSum;
    gains.Td = tauProd / tauSum;

    return gains;
}

} // namespace Autotuning
} // namespace tether::control

#include "tether/control/autotuning/classical/SIMCMethod.hpp"
#include <algorithm>

namespace Control {
namespace Autotuning {

std::string SIMCMethod::getName() const { return "SIMC (Skogestad)"; }
std::string SIMCMethod::getDescription() const { return "Simple IMC rules with tuning parameter tauC."; }
AutotuningMode SIMCMethod::getMode() const { return AutotuningMode::Offline; }

bool SIMCMethod::isCompatible(const TunableController& controller) const { return !controller.getParameterDescriptors().empty(); }

TuningResult SIMCMethod::tune(TunableController& controller, const ProcessModel* model) {
    TuningResult result; result.success = false;
    FOPDTModel processModel = m_model;
    if (!processModel.isValid() && model) processModel = model->toFOPDT();
    if (!processModel.isValid()) { result.message = "Process model required for SIMC"; return result; }
    PIDGains gains = calculateGains(processModel, m_tauC);
    result.parameters = {gains.Kp, gains.Ki, gains.Kd};
    result.success = controller.setParameters(result.parameters);
    result.message = result.success ? "SIMC tuning successful" : "Failed to set parameters";
    return result;
}

PIDGains SIMCMethod::calculateGains(const FOPDTModel& model, double tauC) {
    PIDGains gains; double K = model.K, tau = model.tau, L = model.L; if (K == 0) return gains;
    gains.Kp = tau / (K * (tauC + L));
    gains.Ti = std::min(tau, 4.0 * (tauC + L));
    if (gains.Ti > 0) gains.Ki = gains.Kp / gains.Ti;
    return gains;
}

PIDGains SIMCMethod::calculateGains(const SOPDTModel& model, double tauC) {
    PIDGains gains; double K = model.K, tau1 = model.tau1, tau2 = model.tau2, L = model.L; if (K == 0) return gains;
    gains.Kp = tau1 / (K * (tauC + L));
    gains.Ti = std::min(tau1, 4.0 * (tauC + L));
    gains.Td = tau2;
    if (gains.Ti > 0) gains.Ki = gains.Kp / gains.Ti;
    gains.Kd = gains.Kp * gains.Td;
    return gains;
}

} // namespace Autotuning
} // namespace Control

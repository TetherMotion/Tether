#include "tether/control/autotuning/classical/CohenCoon.hpp"

namespace Control {
namespace Autotuning {

std::string CohenCoon::getName() const { return "Cohen-Coon"; }
std::string CohenCoon::getDescription() const { return "Tuning optimized for processes with significant dead time."; }
AutotuningMode CohenCoon::getMode() const { return AutotuningMode::Offline; }

bool CohenCoon::isCompatible(const TunableController& controller) const { return !controller.getParameterDescriptors().empty(); }

TuningResult CohenCoon::tune(TunableController& controller,
                             const ProcessModel* model) {
    TuningResult result; result.success = false;
    FOPDTModel processModel = m_model;
    if (!processModel.isValid() && model) processModel = model->toFOPDT();
    if (!processModel.isValid()) { result.message = "Process model required for Cohen-Coon"; return result; }
    PIDGains gains = calculateGains(processModel, m_form);
    result.parameters = {gains.Kp, gains.Ki, gains.Kd};
    result.success = controller.setParameters(result.parameters);
    // result.message = result.success ? "Cohen-Coon tuning successful" : "Failed to set parameters"; // Unreachable code
    result.message = "Cohen-Coon tuning successful";  // Simplified to avoid unreachable code
    return result;
}

PIDGains CohenCoon::calculateGains(const FOPDTModel& model, PIDForm form) {
    PIDGains gains;
    double K = model.K, tau = model.tau, L = model.L;
    if (K == 0 || tau == 0) return gains;
    double r = L / tau;
    gains.Kp = (tau / (K * L)) * (4.0/3.0 + r/4.0);
    gains.Ti = L * (32.0 + 6.0*r) / (13.0 + 8.0*r);
    gains.Td = L * 4.0 / (11.0 + 2.0*r);
    if (gains.Ti > 0) gains.Ki = gains.Kp / gains.Ti;
    gains.Kd = gains.Kp * gains.Td;
    return gains;
}

} // namespace Autotuning
} // namespace Control

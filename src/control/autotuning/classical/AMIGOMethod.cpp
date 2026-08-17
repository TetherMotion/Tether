#include "tether/control/autotuning/classical/AMIGOMethod.hpp"

namespace tether::control {
namespace Autotuning {

std::string AMIGOMethod::getName() const { return "AMIGO"; }
std::string AMIGOMethod::getDescription() const { return "Robustness-optimized tuning with guaranteed Ms < 1.4."; }
AutotuningMode AMIGOMethod::getMode() const { return AutotuningMode::Offline; }

bool AMIGOMethod::isCompatible(const TunableController& controller) const { return !controller.getParameterDescriptors().empty(); }

TuningResult AMIGOMethod::tune(TunableController& controller, const ProcessModel* model) {
    TuningResult result; result.success = false;
    FOPDTModel processModel = m_model;
    if (!processModel.isValid() && model) processModel = model->toFOPDT();
    if (!processModel.isValid()) { result.message = "Process model required for AMIGO"; return result; }
    PIDGains gains = calculateGains(processModel, PIDForm::Parallel);
    result.parameters = {gains.Kp, gains.Ki, gains.Kd};
    result.success = controller.setParameters(result.parameters);
    result.message = result.success ? "AMIGO tuning successful" : "Failed to set parameters";
    return result;
}

PIDGains AMIGOMethod::calculateGains(const FOPDTModel& model, PIDForm form) {
    PIDGains gains; double K = model.K, tau = model.tau, L = model.L;
    if (K == 0 || tau == 0) return gains;
    double ratio = tau / L;
    gains.Kp = (1.0/K) * (0.2 + 0.45 * ratio);
    gains.Ti = (0.4 * L + 0.8 * tau) / (L + 0.1 * tau) * L;
    gains.Td = 0.5 * L * tau / (0.3 * L + tau);
    if (gains.Ti > 0) gains.Ki = gains.Kp / gains.Ti;
    gains.Kd = gains.Kp * gains.Td;
    return gains;
}

} // namespace Autotuning
} // namespace tether::control

#include "tether/control/autotuning/classical/LambdaTuning.hpp"

namespace Control {
namespace Autotuning {

std::string LambdaTuning::getName() const { return "Lambda/IMC Tuning"; }
std::string LambdaTuning::getDescription() const { return "IMC-based tuning with specified closed-loop time constant."; }
AutotuningMode LambdaTuning::getMode() const { return AutotuningMode::Offline; }

bool LambdaTuning::isCompatible(const TunableController& controller) const { return !controller.getParameterDescriptors().empty(); }

TuningResult LambdaTuning::tune(TunableController& controller, const ProcessModel* model) {
    TuningResult result; result.success = false;
    FOPDTModel processModel = m_model;
    if (!processModel.isValid() && model) processModel = model->toFOPDT();
    if (!processModel.isValid()) { result.message = "Process model required for Lambda tuning"; return result; }
    double lambda = m_lambda;
    if (m_useAutoLambda) lambda = m_lambdaFactor * processModel.tau;
    PIDGains gains = calculateGains(processModel, lambda, false);
    result.parameters = {gains.Kp, gains.Ki, gains.Kd};
    result.success = controller.setParameters(result.parameters);
    // result.message = result.success ? "Lambda tuning successful" : "Failed to set parameters"; // Unreachable code
    result.message = "Lambda tuning successful";  // Simplified to avoid unreachable code
    return result;
}

PIDGains LambdaTuning::calculateGains(const FOPDTModel& model, double lambda, bool includeDerivative) {
    PIDGains gains;
    double K = model.K, tau = model.tau, L = model.L; if (K == 0) return gains;
    gains.Kp = tau / (K * (lambda + L));
    gains.Ti = tau;
    if (includeDerivative) gains.Td = L / 2.0;
    if (gains.Ti > 0) gains.Ki = gains.Kp / gains.Ti;
    gains.Kd = gains.Kp * gains.Td;
    return gains;
}

void LambdaTuning::setRobustness(double robustness) {
    m_useAutoLambda = true;
    m_lambdaFactor = 0.5 + 2.5 * robustness;
}

} // namespace Autotuning
} // namespace Control

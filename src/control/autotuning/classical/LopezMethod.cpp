#include "tether/control/autotuning/classical/LopezMethod.hpp"
#include <cmath>

namespace tether::control {
namespace Autotuning {

std::string LopezMethod::getName() const { return "Lopez (ITAE/IAE/ISE)"; }
std::string LopezMethod::getDescription() const { return "Optimal tuning minimizing integral error criteria."; }
AutotuningMode LopezMethod::getMode() const { return AutotuningMode::Offline; }

bool LopezMethod::isCompatible(const TunableController& controller) const { return !controller.getParameterDescriptors().empty(); }

TuningResult LopezMethod::tune(TunableController& controller, const ProcessModel* model) {
    TuningResult result; result.success = false;
    FOPDTModel processModel = m_model;
    if (!processModel.isValid() && model) processModel = model->toFOPDT();
    if (!processModel.isValid()) { result.message = "Process model required for Lopez method"; return result; }
    PIDGains gains = calculateGains(processModel, m_form, m_criterion, m_responseType);
    result.parameters = {gains.Kp, gains.Ki, gains.Kd};
    result.success = controller.setParameters(result.parameters);
    result.message = result.success ? "Lopez tuning successful" : "Failed to set parameters";
    return result;
}

PIDGains LopezMethod::calculateGains(const FOPDTModel& model, PIDForm form, Criterion criterion, ResponseType response) {
    PIDGains gains;
    double K = model.K, tau = model.tau, L = model.L;
    if (K == 0 || tau == 0) return gains;
    double A, B, C, D, E, F;
    double ratio = L / tau;
    if (response == ResponseType::Setpoint) {
        switch (criterion) {
            case Criterion::IAE: A = 0.758; B = 0.861; C = 1.02; D = -0.323; break;
            case Criterion::ITAE: A = 0.586; B = 0.916; C = 1.03; D = -0.165; break;
            case Criterion::ISE: default: A = 1.048; B = 0.897; C = 1.195; D = -0.368; break;
        }
    } else {
        switch (criterion) {
            case Criterion::IAE: A = 0.984; B = 0.986; C = 0.608; D = 0.707; break;
            case Criterion::ITAE: A = 1.357; B = 0.947; C = 0.842; D = 0.738; break;
            case Criterion::ISE: default: A = 1.495; B = 0.945; C = 1.101; D = 0.771; break;
        }
    }
    gains.Kp = (1.0 / K) * A * std::pow(tau / L, B);
    gains.Ti = tau / (C * std::pow(ratio, D));
    if (gains.Ti > 0) gains.Ki = gains.Kp / gains.Ti;
    return gains;
}

} // namespace Autotuning
} // namespace tether::control

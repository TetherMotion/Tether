#include "tether/control/autotuning/classical/ChienHronesReswick.hpp"

namespace tether::control {
namespace Autotuning {

std::string ChienHronesReswick::getName() const { return "Chien-Hrones-Reswick"; }
std::string ChienHronesReswick::getDescription() const { return "Tuning with selectable overshoot and setpoint/regulator focus."; }
AutotuningMode ChienHronesReswick::getMode() const { return AutotuningMode::Offline; }

bool ChienHronesReswick::isCompatible(const TunableController& controller) const { return !controller.getParameterDescriptors().empty(); }

TuningResult ChienHronesReswick::tune(TunableController& controller, const ProcessModel* model) {
    TuningResult result; result.success = false;
    FOPDTModel processModel = m_model;
    if (!processModel.isValid() && model) processModel = model->toFOPDT();
    if (!processModel.isValid()) { result.message = "Process model required for CHR"; return result; }
    PIDGains gains = calculateGains(processModel, m_form, m_mode);
    result.parameters = {gains.Kp, gains.Ki, gains.Kd};
    result.success = controller.setParameters(result.parameters);
    result.message = result.success ? "CHR tuning successful" : "Failed to set parameters";
    return result;
}

PIDGains ChienHronesReswick::calculateGains(const FOPDTModel& model, PIDForm form, Mode mode) {
    PIDGains gains; double K = model.K, tau = model.tau, L = model.L;
    if (K == 0 || L == 0) return gains;
    switch (mode) {
        case Mode::SetpointNoOvershoot:
            gains.Kp = 0.6 * tau / (K * L);
            gains.Ti = tau; gains.Td = 0.5 * L; break;
        case Mode::Setpoint20Overshoot:
            gains.Kp = 0.95 * tau / (K * L);
            gains.Ti = 1.357 * tau; gains.Td = 0.473 * L; break;
        case Mode::RegulatorNoOvershoot:
            gains.Kp = 0.95 * tau / (K * L);
            gains.Ti = 2.375 * tau; gains.Td = 0.421 * L; break;
        case Mode::Regulator20Overshoot:
            gains.Kp = 1.2 * tau / (K * L);
            gains.Ti = 2.0 * tau; gains.Td = 0.42 * L; break;
    }
    if (gains.Ti > 0) gains.Ki = gains.Kp / gains.Ti;
    gains.Kd = gains.Kp * gains.Td;
    return gains;
}

} // namespace Autotuning
} // namespace tether::control

#include "tether/control/autotuning/classical/ZieglerNicholsUltimateCycle.hpp"

namespace Control {
namespace Autotuning {

std::string ZieglerNicholsUltimateCycle::getName() const { return "Ziegler-Nichols Ultimate Cycle"; }
std::string ZieglerNicholsUltimateCycle::getDescription() const { return "Classic closed-loop tuning using critical gain and period."; }
AutotuningMode ZieglerNicholsUltimateCycle::getMode() const { return AutotuningMode::Offline; }

bool ZieglerNicholsUltimateCycle::isCompatible(const TunableController& controller) const {
    return !controller.getParameterDescriptors().empty();
}

TuningResult ZieglerNicholsUltimateCycle::tune(TunableController& controller,
                                               const ProcessModel* model) {
    TuningResult result;
    result.success = false;

    double Ku = m_Ku;
    double Tu = m_Tu;
    if ((Ku <= 0 || Tu <= 0) && model) {
        auto [kuEst, tuEst] = model->getUltimateParams();
        Ku = kuEst; Tu = tuEst;
    }
    if (Ku <= 0 || Tu <= 0) {
        result.message = "Ultimate gain and period must be set first";
        return result;
    }

    PIDGains gains = calculateGains(Ku, Tu, m_form);
    result.parameters = {gains.Kp, gains.Ki, gains.Kd};
    result.success = controller.setParameters(result.parameters);
    result.message = result.success ? "Ziegler-Nichols ultimate cycle tuning successful" : "Failed to set parameters";
    return result;
}

PIDGains ZieglerNicholsUltimateCycle::calculateGains(double Ku, double Tu, PIDForm form) {
    PIDGains gains;
    gains.Kp = 0.6 * Ku;
    gains.Ti = Tu / 2.0;
    gains.Td = Tu / 8.0;
    gains.Ki = gains.Ti > 0 ? gains.Kp / gains.Ti : 0.0;
    gains.Kd = gains.Kp * gains.Td;
    return gains;
}

void ZieglerNicholsUltimateCycle::setUltimateParameters(double Ku, double Tu) {
    m_Ku = Ku; m_Tu = Tu;
}

} // namespace Autotuning
} // namespace Control

#include "tether/control/autotuning/classical/TyreusLuyben.hpp"

namespace tether::control {
namespace Autotuning {

std::string TyreusLuyben::getName() const { return "Tyreus-Luyben"; }
std::string TyreusLuyben::getDescription() const { return "Conservative tuning for chemical processes."; }
AutotuningMode TyreusLuyben::getMode() const { return AutotuningMode::Offline; }

bool TyreusLuyben::isCompatible(const TunableController& controller) const {
    return !controller.getParameterDescriptors().empty();
}

TuningResult TyreusLuyben::tune(TunableController& controller,
                                const ProcessModel* model) {
    TuningResult result; result.success = false;

    double Ku = m_Ku, Tu = m_Tu;
    if ((Ku <= 0 || Tu <= 0) && model) {
        auto [kuEst, tuEst] = model->getUltimateParams(); Ku = kuEst; Tu = tuEst;
    }
    if (Ku <= 0 || Tu <= 0) { result.message = "Ultimate gain and period must be set first"; return result; }

    PIDGains gains = calculateGains(Ku, Tu, false);
    result.parameters = {gains.Kp, gains.Ki, gains.Kd};
    result.success = controller.setParameters(result.parameters);
    result.message = result.success ? "Tyreus-Luyben tuning successful" : "Failed to set parameters";
    return result;
}

PIDGains TyreusLuyben::calculateGains(double Ku, double Tu, bool includeDterm) {
    PIDGains gains;
    gains.Kp = Ku / 3.2;
    gains.Ti = 2.2 * Tu;
    if (includeDterm) gains.Td = Tu / 6.3;
    if (gains.Ti > 0) gains.Ki = gains.Kp / gains.Ti;
    gains.Kd = gains.Kp * gains.Td;
    return gains;
}

} // namespace Autotuning
} // namespace tether::control

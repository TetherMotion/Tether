#include "tether/control/autotuning/classical/ZieglerNicholsStepResponse.hpp"
#include <algorithm>

namespace Control {
namespace Autotuning {

std::string ZieglerNicholsStepResponse::getName() const { return "Ziegler-Nichols Step Response"; }
std::string ZieglerNicholsStepResponse::getDescription() const {
    return "Classic open-loop tuning using step response. Identifies FOPDT model and applies Z-N rules.";
}
AutotuningMode ZieglerNicholsStepResponse::getMode() const { return AutotuningMode::Offline; }

bool ZieglerNicholsStepResponse::isCompatible(const TunableController& controller) const {
    auto descriptors = controller.getParameterDescriptors();
    return !descriptors.empty();
}

TuningResult ZieglerNicholsStepResponse::tune(TunableController& controller,
                                              const ProcessModel* model) {
    // Implementation adapted from the monolithic source
    TuningResult result;
    result.success = false;

    FOPDTModel processModel;
    if (!m_time.empty() && !m_response.empty()) {
        processModel = identifyModel(m_time, m_response, m_stepSize, m_initialValue);
    } else if (model) {
        processModel = model->toFOPDT();
    } else {
        result.message = "Process model required for Z-N step response";
        return result;
    }

    if (!processModel.isValid()) {
        result.message = "Invalid model parameters";
        return result;
    }

    PIDGains gains = calculateGains(processModel, m_form);
    result.parameters = {gains.Kp, gains.Ki, gains.Kd};
    result.success = controller.setParameters(result.parameters);
    result.message = result.success ? "Ziegler-Nichols step response tuning successful" : "Failed to set parameters";
    result.cost = 0.0;
    return result;
}

FOPDTModel ZieglerNicholsStepResponse::identifyModel(const std::vector<double>& time,
                                                      const std::vector<double>& response,
                                                      double stepSize,
                                                      double initialValue) {
    FOPDTModel model;
    if (time.size() < 3 || response.size() < 3) return model;

    double finalValue = response.back();
    model.K = (finalValue - initialValue) / stepSize;

    double maxSlope = 0;
    size_t inflectionIdx = 1;
    for (size_t i = 1; i < response.size() - 1; ++i) {
        double slope = (response[i + 1] - response[i - 1]) / (time[i + 1] - time[i - 1]);
        if (slope > maxSlope) {
            maxSlope = slope;
            inflectionIdx = i;
        }
    }

    double tInfl = time[inflectionIdx];
    double yInfl = response[inflectionIdx];

    model.L = tInfl - (yInfl - initialValue) / maxSlope;
    if (model.L < 0) model.L = 0;

    double tauPlusL = tInfl + (finalValue - yInfl) / maxSlope;
    model.tau = tauPlusL - model.L;
    if (model.tau <= 0) model.tau = 1.0;

    return model;
}

PIDGains ZieglerNicholsStepResponse::calculateGains(const FOPDTModel& model, PIDForm form) {
    PIDGains gains;
    double K = model.K;
    double tau = model.tau;
    double L = model.L;
    if (K == 0 || L == 0) return gains;

    gains.Kp = 1.2 * tau / (K * L);
    gains.Ti = 2.0 * L;
    gains.Td = 0.5 * L;

    if (gains.Ti > 0) gains.Ki = gains.Kp / gains.Ti;
    gains.Kd = gains.Kp * gains.Td;

    return gains;
}

void ZieglerNicholsStepResponse::setStepResponseData(const std::vector<double>& time,
                                                     const std::vector<double>& response,
                                                     double stepSize, double initialValue) {
    m_time = time;
    m_response = response;
    m_stepSize = stepSize;
    m_initialValue = initialValue;
}

} // namespace Autotuning
} // namespace Control

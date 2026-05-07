#include "tether/control/autotuning/model_based/SmithPredictor.hpp"

namespace Control {
namespace Autotuning {

bool SmithPredictor::isCompatible(const TunableController& controller) const {
    auto name = controller.getControllerTypeName();
    return name.find("PID") != std::string::npos ||
           name.find("Smith") != std::string::npos;
}

TuningResult SmithPredictor::tune(TunableController& controller,
                                  const ProcessModel* model) {
    TuningResult result;

    FOPDTModel fopdt;
    if (model) {
        fopdt = model->toFOPDT();
    } else {
        fopdt = m_model;
    }

    if (fopdt.K == 0 || fopdt.tau <= 0) {
        result.success = false;
        result.message = "Invalid model parameters";
        return result;
    }

    auto sp = design(fopdt, m_lambda);
    double Ki = (sp.innerController.Ti > 0) ? 
                sp.innerController.Kp / sp.innerController.Ti : 0.0;
    double Kd = sp.innerController.Kp * sp.innerController.Td;

    result.parameters = {sp.innerController.Kp, Ki, Kd};
    result.success = true;
    result.message = "Smith predictor design complete";
    result.settlingTime = 4.0 * m_lambda;

    controller.setParameters(result.parameters);

    return result;
}

SmithPredictor::SmithPredictorStructure SmithPredictor::design(
    const FOPDTModel& model, double lambda) {

    SmithPredictorStructure sp;
    sp.processModel = model;
    sp.delay = model.L;

    sp.innerController.Kp = model.tau / (model.K * lambda);
    sp.innerController.Ti = model.tau;
    sp.innerController.Td = 0.0;

    return sp;
}

} // namespace Autotuning
} // namespace Control

#include "tether/control/autotuning/model_based/DahlinAlgorithm.hpp"
#include <cmath>

namespace Control {
namespace Autotuning {

bool DahlinAlgorithm::isCompatible(const TunableController& controller) const {
    auto name = controller.getControllerTypeName();
    return name.find("Digital") != std::string::npos ||
           name.find("PID") != std::string::npos ||
           name.find("Discrete") != std::string::npos;
}

TuningResult DahlinAlgorithm::tune(TunableController& controller,
                                   const ProcessModel* model) {
    TuningResult result;

    if (!model) {
        result.success = false;
        result.message = "Process model required for Dahlin's algorithm";
        return result;
    }

    auto fopdt = model->toFOPDT();
    auto coeffs = designDigital(fopdt, m_Ts, m_lambda);

    double b0 = coeffs[0];
    double b1 = coeffs[1];
    double a1 = coeffs[2];

    double Kp = b0;
    double Ki = (b0 + b1) / m_Ts;
    double Kd = 0.0;

    result.parameters = {Kp, Ki, Kd};
    result.success = true;
    result.message = "Dahlin controller design complete";
    result.settlingTime = 4.0 * m_lambda;

    controller.setParameters(result.parameters);

    return result;
}

std::array<double, 3> DahlinAlgorithm::designDigital(const FOPDTModel& model,
                                                      double Ts, double lambda) {
    double K = model.K;
    double tau = model.tau;
    double L = model.L;

    double a = std::exp(-Ts / tau);
    int N = static_cast<int>(L / Ts);
    double alpha = std::exp(-Ts / lambda);

    double num0 = (1 - alpha) * 1.0;
    double num1 = (1 - alpha) * (-a);
    double den0 = K * (1 - a) * 1.0 - (1 - alpha) * 1.0;
    double den1 = K * (1 - a) * (-alpha) - (1 - alpha) * (-a);

    double b0 = num0 / (den0 + 0.001);
    double b1 = num1 / (den0 + 0.001);
    double a1 = den1 / (den0 + 0.001);

    return {b0, b1, a1};
}

} // namespace Autotuning
} // namespace Control

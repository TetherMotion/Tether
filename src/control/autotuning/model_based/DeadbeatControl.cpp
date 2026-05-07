#include "tether/control/autotuning/model_based/DeadbeatControl.hpp"
#include <cmath>

namespace Control {
namespace Autotuning {

bool DeadbeatControl::isCompatible(const TunableController& controller) const {
    auto name = controller.getControllerTypeName();
    return name.find("Digital") != std::string::npos ||
           name.find("Discrete") != std::string::npos ||
           name.find("State") != std::string::npos;
}

TuningResult DeadbeatControl::tune(TunableController& controller,
                                   const ProcessModel* model) {
    TuningResult result;

    if (!model) {
        result.success = false;
        result.message = "Process model required for deadbeat control";
        return result;
    }

    auto fopdt = model->toFOPDT();

    double a = std::exp(-m_Ts / fopdt.tau);
    double b = fopdt.K * (1 - a);

    double K_db = (1.0 - 0.0) / (b + 0.001);

    if (m_settlingN > 0) {
        double desiredPole = std::pow(0.1, 1.0 / m_settlingN);
        K_db = (a - desiredPole) / (b + 0.001);
    }

    double Kp = K_db;
    double Ki = K_db / fopdt.tau;

    result.parameters = {Kp, Ki, 0.0};
    result.success = true;
    result.message = "Deadbeat control design complete";
    result.settlingTime = m_Ts * (m_settlingN > 0 ? m_settlingN : 1);

    controller.setParameters(result.parameters);

    return result;
}

std::vector<double> DeadbeatControl::design(const double* A, const double* B,
                                            const double* C, int n, int m, int p,
                                            double Ts, int settlingN) {
    std::vector<std::complex<double>> desiredPoles(n);
    double poleLocation = (settlingN > 0) ? std::pow(0.1, 1.0 / settlingN) : 0.0;
    for (int i = 0; i < n; i++) {
        desiredPoles[i] = {poleLocation, 0.0};
    }
    return PolePlacement::computeGain(A, B, n, desiredPoles);
}

} // namespace Autotuning
} // namespace Control

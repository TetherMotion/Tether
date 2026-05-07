#include "tether/control/autotuning/model_based/MinimumVarianceControl.hpp"
#include <cmath>

namespace Control {
namespace Autotuning {

bool MinimumVarianceControl::isCompatible(const TunableController& controller) const {
    auto name = controller.getControllerTypeName();
    return name.find("Digital") != std::string::npos ||
           name.find("MVC") != std::string::npos ||
           name.find("Stochastic") != std::string::npos;
}

TuningResult MinimumVarianceControl::tune(TunableController& controller,
                                          const ProcessModel* /*model*/) {
    TuningResult result;

    if (m_A.empty() || m_B.empty() || m_C.empty()) {
        result.success = false;
        result.message = "ARMAX model required. Call setARMAXModel() first.";
        return result;
    }

    double a1 = m_A.size() > 1 ? m_A[1] : 0.0;

    // ARMAX B often includes leading zeros for input delay k.
    // Use the coefficient at the delay index when available.
    size_t bIndex = 0;
    if (m_k > 0) {
        bIndex = static_cast<size_t>(m_k);
        if (bIndex >= m_B.size()) {
            bIndex = m_B.size() - 1;
        }
    }
    double b = m_B.empty() ? 0.0 : m_B[bIndex];

    double c1 = m_C.size() > 1 ? m_C[1] : 0.0;

    double s0 = a1 - c1;
    double denom = b + m_lambda * std::abs(b);
    if (std::abs(denom) < 1e-12) {
        result.success = false;
        result.message = "Invalid ARMAX B coefficient (division by zero)";
        return result;
    }
    double K_mvc = s0 / denom;
    if (!std::isfinite(K_mvc)) {
        result.success = false;
        result.message = "MVC computation produced non-finite gain";
        return result;
    }

    double Kp = K_mvc;
    double Ki = K_mvc / 10.0;  // approximate

    result.parameters = {Kp, Ki, 0.0};
    result.success = true;
    result.message = "Minimum variance control design complete";

    controller.setParameters(result.parameters);

    return result;
}

void MinimumVarianceControl::setARMAXModel(const std::vector<double>& A,
                                            const std::vector<double>& B,
                                            const std::vector<double>& C,
                                            int k) {
    m_A = A;
    m_B = B;
    m_C = C;
    m_k = k;
}

} // namespace Autotuning
} // namespace Control

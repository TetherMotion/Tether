#include "tether/control/autotuning/model_based/IMCDesign.hpp"
#include <cmath>

namespace tether::control {
namespace Autotuning {

bool IMCDesign::isCompatible(const TunableController& controller) const {
    auto name = controller.getControllerTypeName();
    return name.find("PID") != std::string::npos ||
           name.find("PI") != std::string::npos ||
           name.find("IMC") != std::string::npos;
}

TuningResult IMCDesign::tune(TunableController& controller,
                             const ProcessModel* model) {
    TuningResult result;
    FOPDTModel fopdt;

    if (model) {
        fopdt = model->toFOPDT();
    } else if (m_useFOPDT) {
        fopdt = m_fopdtModel;
    } else {
        fopdt.K = m_sopdtModel.K;
        fopdt.tau = m_sopdtModel.tau1 + m_sopdtModel.tau2;
        fopdt.L = m_sopdtModel.L;
    }

    if (fopdt.K == 0 || fopdt.tau <= 0) {
        result.success = false;
        result.message = "Invalid model parameters";
        return result;
    }

    PIDGains gains;
    if (m_useFOPDT || model) {
        gains = designForFOPDT(fopdt, m_lambda, m_filterOrder);
    } else {
        gains = designForSOPDT(m_sopdtModel, m_lambda);
    }

    double Ki = (gains.Ti > 0) ? gains.Kp / gains.Ti : 0.0;
    double Kd = gains.Kp * gains.Td;

    result.parameters = {gains.Kp, Ki, Kd};
    result.success = true;
    result.message = "IMC design complete";
    result.settlingTime = m_lambda * 4;

    controller.setParameters(result.parameters);

    return result;
}

PIDGains IMCDesign::designForFOPDT(const FOPDTModel& model, 
                                   double lambda, int filterOrder) {
    PIDGains gains;
    double K = model.K;
    double tau = model.tau;
    double L = model.L;

    if (filterOrder == 1) {
        double lambdaEff = lambda + L/2;
        gains.Kp = tau / (K * lambdaEff);
        gains.Ti = tau;
        gains.Td = L / 2;
    } else {
        double lambdaEff = lambda;
        gains.Kp = (tau + L/2) / (K * (lambdaEff + L/2));
        gains.Ti = tau + L/2;
        gains.Td = tau * L / (2*tau + L);
    }

    return gains;
}

PIDGains IMCDesign::designForSOPDT(const SOPDTModel& model, double lambda) {
    PIDGains gains;
    double K = model.K;
    double tau1 = model.tau1;
    double tau2 = model.tau2;
    double L = model.L;

    double tauSum = tau1 + tau2;
    double tauProd = tau1 * tau2;

    gains.Kp = tauSum / (K * (lambda + L));
    gains.Ti = tauSum;
    gains.Td = tauProd / tauSum;

    return gains;
}

PIDGains IMCDesign::designForIPDT(const IPDTModel& model, double lambda) {
    PIDGains gains;
    double K = model.K;
    double L = model.L;

    gains.Kp = 2 / (K * (2*lambda + L));
    gains.Ti = 2*lambda + L;
    gains.Td = 0.0;

    return gains;
}

} // namespace Autotuning
} // namespace tether::control

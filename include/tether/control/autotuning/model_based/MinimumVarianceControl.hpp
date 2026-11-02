/**
 * @file MinimumVarianceControl.hpp
 * @brief Minimum Variance Control autotuner (stochastic optimal control).
 *
 * @details
 * Designs controllers that minimize output variance for systems modelled
 * with ARMAX noise descriptions. A penalty on control variance can be
 * specified to trade performance for control effort.
 */

#pragma once

#include "tether/control/autotuning/AutotuningFramework.hpp"
#include <vector>

namespace tether::control {
namespace Autotuning {

/**
 * @class MinimumVarianceControl
 * @brief Minimum Variance Control autotuner (stochastic optimal control).
 *
 * @details
 * Designs controllers that minimize output variance for systems modelled
 * with ARMAX noise descriptions; a control-penalty lambda may be applied.
 */
class MinimumVarianceControl : public AutotunerBase {
public:
    std::string getName() const override { return "Minimum Variance Control"; }
    std::string getDescription() const override {
        return "Minimizes output variance for stochastic systems. "
               "Requires ARMAX noise model.";
    }
    AutotuningMode getMode() const override { return AutotuningMode::Offline; }

    /**
     * @brief Check compatibility for minimum-variance design (controller supports ARMAX).
     * @param controller  Controller whose structure is validated.
     * @return true when MVC coefficients can be applied.
     */
    bool isCompatible(const TunableController& controller) const override;
    
    /**
     * @brief Design an MVC controller from ARMAX model and apply parameters.
     * @param controller  Controller receiving the coefficients.
     * @param model       Optional process model.
     * @return TuningResult or an error.
     */
    TuningResult tune(TunableController& controller,
                     const ProcessModel* model = nullptr) override;
    
    /**
     * @brief Set ARMAX model polynomials
     * y(t) = B(q)/A(q) * u(t-k) + C(q)/A(q) * e(t)
     *
     * @param A,B,C  ARMAX polynomials.
     * @param k      Input delay in samples.
     */
    void setARMAXModel(const std::vector<double>& A,
                      const std::vector<double>& B,
                      const std::vector<double>& C,
                      int k);
    
    /** @brief Set penalty on control variance (generalized MVC)
     *  J = E[(y - r)² + λu²]
     *
     * @param lambda  Control-variance penalty weight.
     */
    void setControlWeight(double lambda) { m_lambda = lambda; }
    
private:
    std::vector<double> m_A, m_B, m_C;
    int m_k{1};
    double m_lambda{0.0};
};

} // namespace Autotuning
} // namespace tether::control

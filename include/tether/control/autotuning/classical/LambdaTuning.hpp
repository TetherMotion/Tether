/**
 * @file LambdaTuning.hpp
 * @brief IMC-style lambda tuning for closed-loop time-constant specification.
 *
 * @details
 * Design method that selects controller to achieve a desired closed-loop
 * time constant (lambda), trading performance against robustness.
 */

#pragma once

#include "tether/control/autotuning/classical/Common.hpp"
#include "tether/control/autotuning/AutotuningFramework.hpp"

namespace tether::control {
namespace Autotuning {

/**
 * @class LambdaTuning
 * @brief IMC-style lambda tuning.
 *
 * @details
 * Designs controllers to achieve a desired closed-loop time constant
 * (lambda), trading performance for robustness.
 */
class LambdaTuning : public AutotunerBase {
public:
    /** @brief Human-readable name of the tuner. */
    std::string getName() const override;

    /** @brief Short description for UI or logs. */
    std::string getDescription() const override;

    /** @brief Offline/online mode of operation. */
    AutotuningMode getMode() const override;

    /** @brief Whether the given controller is compatible with this tuner. */
    bool isCompatible(const TunableController& controller) const override;

    /**
     * @brief Execute the tuner and apply computed parameters to @p controller.
     * @param controller Controller to update with computed parameters.
     * @param model Optional process model to base the design on.
     */
    TuningResult tune(TunableController& controller, const ProcessModel* model = nullptr) override;

    /** @brief Compute PID gains using IMC-style lambda tuning. */
    static PIDGains calculateGains(const FOPDTModel& model, double lambda, bool includeDerivative = false);

    /** @brief Set desired closed-loop time constant (lambda). */
    void setLambda(double lambda) { m_lambda = lambda; }

    /** @brief Scale the computed lambda (helper). */
    void setLambdaFactor(double factor) { m_lambdaFactor = factor; }

    /** @brief Adjust robustness parameter used to compute lambda. */
    void setRobustness(double robustness);

    /** @brief Provide the FOPDT model used for design. */
    void setModel(const FOPDTModel& model) { m_model = model; }

private:
    FOPDTModel m_model;
    double m_lambda{1.0};
    double m_lambdaFactor{1.0};
    bool m_useAutoLambda{false};
};

} // namespace Autotuning
} // namespace tether::control

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
    /** @brief Human-readable name of the tuner.
     *  @return Display name ("Lambda (IMC) Tuning")
     */
    std::string getName() const override;

    /** @brief Short description for UI or logs.
     *  @return One-line description of the lambda-tuning method
     */
    std::string getDescription() const override;

    /** @brief Offline/online mode of operation.
     *  @return AutotuningMode::Offline — lambda tuning needs a model
     */
    AutotuningMode getMode() const override;

    /** @brief Whether the given controller is compatible with this tuner.
     *  @param controller Controller to check
     *  @return `true` if the tuner can produce gains for `controller`
     */
    bool isCompatible(const TunableController& controller) const override;

    /**
     * @brief Execute the tuner and apply computed parameters to @p controller.
     * @param controller Controller to update with computed parameters.
     * @param model Optional process model to base the design on; falls back
     *        to the model previously set via setModel() when nullptr
     * @return Tuning outcome including the computed gains and status
     */
    TuningResult tune(TunableController& controller, const ProcessModel* model = nullptr) override;

    /** @brief Compute PID gains using IMC-style lambda tuning.
     *  @param model Identified FOPDT process model
     *  @param lambda Desired closed-loop time constant [s]; smaller is
     *         faster but less robust
     *  @param includeDerivative `true` to include a D term
     *  @return Computed PID gains
     */
    static PIDGains calculateGains(const FOPDTModel& model, double lambda, bool includeDerivative = false);

    /** @brief Set desired closed-loop time constant (lambda).
     *  @param lambda Closed-loop target time constant [s]
     */
    void setLambda(double lambda) { m_lambda = lambda; }

    /** @brief Scale the computed lambda (helper).
     *  @param factor Multiplier applied to the auto-computed lambda
     */
    void setLambdaFactor(double factor) { m_lambdaFactor = factor; }

    /** @brief Adjust robustness parameter used to compute lambda.
     *  @param robustness Robustness weight; higher values pick a larger
     *         lambda (slower, more robust response)
     */
    void setRobustness(double robustness);

    /** @brief Provide the FOPDT model used for design.
     *  @param model Identified process model the design is based on
     */
    void setModel(const FOPDTModel& model) { m_model = model; }

private:
    FOPDTModel m_model;
    double m_lambda{1.0};
    double m_lambdaFactor{1.0};
    bool m_useAutoLambda{false};
};

} // namespace Autotuning
} // namespace tether::control

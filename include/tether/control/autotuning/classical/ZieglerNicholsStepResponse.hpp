/**
 * @file ZieglerNicholsStepResponse.hpp
 * @brief Ziegler-Nichols step-response autotuner (open-loop).
 *
 * @details
 * Estimates a first-order-plus-dead-time (FOPDT) model from an open-loop
 * step response using the tangent method and computes PID gains according
 * to Ziegler-Nichols step-response tuning rules.
 */

#pragma once

#include "tether/control/autotuning/classical/Common.hpp"
#include "tether/control/autotuning/AutotuningFramework.hpp"
#include <vector>

namespace tether::control {
namespace Autotuning {

/**
 * @class ZieglerNicholsStepResponse
 * @brief Ziegler-Nichols step-response autotuner (open-loop).
 *
 * @details
 * Estimates a first-order-plus-dead-time (FOPDT) model using the tangent
 * method from an open-loop step response and computes PID gains according
 * to Ziegler-Nichols step-response rules.
 */
class ZieglerNicholsStepResponse : public AutotunerBase {
public:
    /**
     * @brief Human-readable name of the tuner.
     * @return Display name ("Ziegler-Nichols Step Response")
     */
    std::string getName() const override;

    /**
     * @brief Short description for UI or logs.
     * @return One-line description of the step-response method
     */
    std::string getDescription() const override;

    /**
     * @brief Offline/online mode of operation.
     * @return AutotuningMode::Offline — needs recorded step data
     */
    AutotuningMode getMode() const override;

    /**
     * @brief Whether the given controller is compatible with this tuner.
     * @param controller Controller to check
     * @return `true` if the tuner can produce gains for `controller`
     */
    bool isCompatible(const TunableController& controller) const override;

    /**
     * @brief Execute tuning and apply parameters to the controller.
     * @param controller Controller whose parameters will be set.
     * @param model Optional process model to use for tuning.
     * @return TuningResult describing success and resulting parameters.
     */
    TuningResult tune(TunableController& controller,
                      const ProcessModel* model = nullptr) override;

    /**
     * @brief Identify a FOPDT model from step response data using the tangent method.
     * @param time Sample times corresponding to response samples.
     * @param response Measured step response values.
     * @param stepSize Input step size.
     * @param initialValue Initial output value before step.
     * @return Estimated FOPDTModel
     */
    static FOPDTModel identifyModel(const std::vector<double>& time,
                                    const std::vector<double>& response,
                                    double stepSize,
                                    double initialValue);

    /**
     * @brief Compute PID gains using Ziegler-Nichols step-response rules.
     * @param model FOPDT model used for calculation.
     * @param form Desired PID form (Parallel/Standard/Series).
     * @return Computed PID gains
     */
    static PIDGains calculateGains(const FOPDTModel& model, PIDForm form);

    /**
     * @brief Provide step response data for offline identification.
     * @param time Sample times [s]
     * @param response Measured output values
     * @param stepSize Input step amplitude
     * @param initialValue Output value before the step
     */
    void setStepResponseData(const std::vector<double>& time,
                             const std::vector<double>& response,
                             double stepSize, double initialValue);

    /**
     * @brief Select the controller PID form to compute gains for.
     * @param form Gain form (e.g. Parallel, Standard)
     */
    void setControllerForm(PIDForm form) { m_form = form; }

private:
    std::vector<double> m_time;
    std::vector<double> m_response;
    double m_stepSize{1.0};
    double m_initialValue{0.0};
    PIDForm m_form{PIDForm::Parallel};
};

} // namespace Autotuning
} // namespace tether::control

/**
 * @file AMIGOMethod.hpp
 * @brief AMIGO tuning method (robustness-optimized).
 *
 * @details
 * AMIGO provides tuning rules aimed at improved robustness and disturbance
 * rejection, using simplified model-based approximations.
 */

#pragma once

#include "tether/control/autotuning/classical/Common.hpp"
#include "tether/control/autotuning/AutotuningFramework.hpp"

namespace tether::control {
namespace Autotuning {

/**
 * @class AMIGOMethod
 * @brief AMIGO tuning method (robustness-optimized).
 *
 * @details
 * Heuristic rules focused on robustness and disturbance rejection using
 * simplified model-based approximations.
 */
class AMIGOMethod : public AutotunerBase {
public:
    /** @brief Human-readable name of the tuner.
     *  @return Display name ("AMIGO")
     */
    std::string getName() const override;

    /** @brief Short description for UI or logs.
     *  @return One-line description of the AMIGO method
     */
    std::string getDescription() const override;

    /** @brief Offline/online mode of operation.
     *  @return AutotuningMode::Offline — AMIGO needs a FOPDT model
     */
    AutotuningMode getMode() const override;

    /** @brief Whether the given controller is compatible with this tuner.
     *  @param controller Controller to check
     *  @return `true` if the tuner can produce gains for `controller`
     */
    bool isCompatible(const TunableController& controller) const override;

    /** @brief Run AMIGO-style tuning using a FOPDT model.
     *  @param controller Controller to update with computed parameters
     *  @param model Optional process model; falls back to the model set
     *         via setModel() when nullptr
     *  @return Tuning outcome including the computed gains and status
     */
    TuningResult tune(TunableController& controller, const ProcessModel* model = nullptr) override;

    /** @brief Compute AMIGO-based PID gains for the provided FOPDT model.
     *  @param model Identified FOPDT process model
     *  @param form PID gain form the result is expressed in
     *  @return Computed PID gains
     */
    static PIDGains calculateGains(const FOPDTModel& model, PIDForm form);

    /** @brief Provide the FOPDT model for design calculations.
     *  @param model Identified process model the design is based on
     */
    void setModel(const FOPDTModel& model) { m_model = model; }

private:
    FOPDTModel m_model;
};

} // namespace Autotuning
} // namespace tether::control

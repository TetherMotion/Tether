/**
 * @file ZieglerNicholsUltimateCycle.hpp
 * @brief Ziegler-Nichols ultimate cycle (closed-loop relay) autotuner.
 *
 * @details
 * Uses relay-feedback or closed-loop oscillation to estimate the critical
 * gain (Ku) and period (Tu) and applies Ziegler-Nichols ultimate-cycle
 * tuning rules to compute PID gains.
 */

#pragma once

#include "tether/control/autotuning/classical/Common.hpp"
#include "tether/control/autotuning/AutotuningFramework.hpp"

namespace tether::control {
namespace Autotuning {

/**
 * @class ZieglerNicholsUltimateCycle
 * @brief Ziegler-Nichols ultimate-cycle autotuner (closed-loop).
 *
 * @details
 * Estimates the critical gain (Ku) and period (Tu) from closed-loop
 * oscillation and applies ultimate-cycle tuning rules to compute PID gains.
 */
class ZieglerNicholsUltimateCycle : public AutotunerBase {
public:
    /** @brief Human-readable name of the tuner.
     *  @return Display name ("Ziegler-Nichols Ultimate Cycle")
     */
    std::string getName() const override;

    /** @brief Short description for UI or logs.
     *  @return One-line description of the ultimate-cycle method
     */
    std::string getDescription() const override;

    /** @brief Offline/online mode of operation.
     *  @return AutotuningMode::Online — drives the closed loop to its
     *          stability limit to measure Ku/Tu
     */
    AutotuningMode getMode() const override;

    /** @brief Whether the given controller is compatible with this tuner.
     *  @param controller Controller to check
     *  @return `true` if the tuner can produce gains for `controller`
     */
    bool isCompatible(const TunableController& controller) const override;

    /**
     * @brief Execute closed-loop ultimate-cycle tuning and apply parameters.
     * @param controller Controller to tune.
     * @param model Optional process model (may provide initial estimates).
     * @return Tuning outcome including the computed gains and status
     */
    TuningResult tune(TunableController& controller,
                      const ProcessModel* model = nullptr) override;

    /** @brief Compute PID gains from Ku and Tu for a given PID form.
     *  @param Ku Ultimate gain at the stability limit
     *  @param Tu Ultimate oscillation period [s]
     *  @param form PID gain form the result is expressed in
     *  @return Computed PID gains
     */
    static PIDGains calculateGains(double Ku, double Tu, PIDForm form);

    /** @brief Set measured or estimated ultimate gain and period.
     *  @param Ku Ultimate gain at the stability limit
     *  @param Tu Ultimate oscillation period [s]
     */
    void setUltimateParameters(double Ku, double Tu);

    /** @brief Set the controller form to compute gains for.
     *  @param form Gain form (e.g. Parallel, Standard)
     */
    void setControllerForm(PIDForm form) { m_form = form; }

    /** @brief Set an implementation variant string (informational).
     *  @param variant Variant label recorded with the tuning result
     */
    void setVariant(const std::string& variant) { m_variant = variant; }

private:
    double m_Ku{0.0};
    double m_Tu{0.0};
    PIDForm m_form{PIDForm::Parallel};
    std::string m_variant{"original"};
};

} // namespace Autotuning
} // namespace tether::control

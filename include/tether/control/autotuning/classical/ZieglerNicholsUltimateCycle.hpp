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

namespace Control {
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
    /** @brief Human-readable name of the tuner. */
    std::string getName() const override;

    /** @brief Short description for UI or logs. */
    std::string getDescription() const override;

    /** @brief Offline/online mode of operation. */
    AutotuningMode getMode() const override;

    /** @brief Whether the given controller is compatible with this tuner. */
    bool isCompatible(const TunableController& controller) const override;

    /**
     * @brief Execute closed-loop ultimate-cycle tuning and apply parameters.
     * @param controller Controller to tune.
     * @param model Optional process model (may provide initial estimates).
     */
    TuningResult tune(TunableController& controller,
                      const ProcessModel* model = nullptr) override;

    /** @brief Compute PID gains from Ku and Tu for a given PID form. */
    static PIDGains calculateGains(double Ku, double Tu, PIDForm form);

    /** @brief Set measured or estimated ultimate gain and period. */
    void setUltimateParameters(double Ku, double Tu);

    /** @brief Set the controller form to compute gains for. */
    void setControllerForm(PIDForm form) { m_form = form; }

    /** @brief Set an implementation variant string (informational). */
    void setVariant(const std::string& variant) { m_variant = variant; }

private:
    double m_Ku{0.0};
    double m_Tu{0.0};
    PIDForm m_form{PIDForm::Parallel};
    std::string m_variant{"original"};
};

} // namespace Autotuning
} // namespace Control

/**
 * @file TyreusLuyben.hpp
 * @brief Tyreus-Luyben tuning method (conservative, for chemical processes).
 *
 * @details
 * Conservative modification of Ziegler-Nichols ultimate-cycle tuning intended
 * for slower chemical processes with reduced aggressiveness to improve stability.
 */

#pragma once

#include "tether/control/autotuning/classical/Common.hpp"
#include "tether/control/autotuning/AutotuningFramework.hpp"

namespace tether::control {
namespace Autotuning {

/**
 * @class TyreusLuyben
 * @brief Tyreus-Luyben tuning method (conservative variant of Ziegler-Nichols).
 *
 * @details
 * Conservative modification of Ziegler-Nichols ultimate-cycle tuning intended
 * for slower processes. Uses ultimate gain Ku and period Tu (e.g., with D-term
 * disabled: Kp = Ku / 3.2). See `TyreusLuyben::calculateGains` for implementation.
 */
class TyreusLuyben : public AutotunerBase {
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
     * @brief Tune using Tyreus-Luyben rules based on ultimate gain/period.
     * @param controller Controller to update.
     * @param model Optional process model to provide Ku/Tu estimates.
     */
    TuningResult tune(TunableController& controller,
                      const ProcessModel* model = nullptr) override;

    /** @brief Compute PID gains from Ku and Tu (optionally include D-term). */
    static PIDGains calculateGains(double Ku, double Tu, bool includeDterm = false);

    /** @brief Set measured or estimated ultimate gain and period. */
    void setUltimateParameters(double Ku, double Tu) { m_Ku = Ku; m_Tu = Tu; }

private:
    double m_Ku{0.0};
    double m_Tu{0.0};
};

} // namespace Autotuning
} // namespace tether::control

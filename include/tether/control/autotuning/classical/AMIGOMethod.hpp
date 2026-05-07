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

namespace Control {
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
    /** @brief Human-readable name of the tuner. */
    std::string getName() const override;

    /** @brief Short description for UI or logs. */
    std::string getDescription() const override;

    /** @brief Offline/online mode of operation. */
    AutotuningMode getMode() const override;

    /** @brief Whether the given controller is compatible with this tuner. */
    bool isCompatible(const TunableController& controller) const override;

    /** @brief Run AMIGO-style tuning using a FOPDT model. */
    TuningResult tune(TunableController& controller, const ProcessModel* model = nullptr) override;

    /** @brief Compute AMIGO-based PID gains for the provided FOPDT model. */
    static PIDGains calculateGains(const FOPDTModel& model, PIDForm form);

    /** @brief Provide the FOPDT model for design calculations. */
    void setModel(const FOPDTModel& model) { m_model = model; }

private:
    FOPDTModel m_model;
};

} // namespace Autotuning
} // namespace Control

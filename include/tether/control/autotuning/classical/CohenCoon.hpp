/**
 * @file CohenCoon.hpp
 * @brief Cohen-Coon tuning method.
 *
 * @details
 * Provides tuning rules that are often better for processes with significant
 * dead time compared to basic Ziegler-Nichols rules.
 */

#pragma once

#include "tether/control/autotuning/classical/Common.hpp"
#include "tether/control/autotuning/AutotuningFramework.hpp"

namespace Control {
namespace Autotuning {

/**
 * @class CohenCoon
 * @brief Cohen-Coon tuning method.
 *
 * @details
 * Tuning rules tailored for processes with significant dead time and
 * asymmetry in dynamics; often preferable to basic Ziegler-Nichols for
 * long-delay systems.
 */
class CohenCoon : public AutotunerBase {
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
     * @brief Tune using Cohen-Coon rules based on a FOPDT model.
     * @param controller Controller to update.
     * @param model Optional process model to use for tuning.
     */
    TuningResult tune(TunableController& controller,
                      const ProcessModel* model = nullptr) override;

    /** @brief Compute Cohen-Coon PID gains from a FOPDT model. */
    static PIDGains calculateGains(const FOPDTModel& model, PIDForm form);

    /** @brief Provide the FOPDT process model to base calculations on. */
    void setModel(const FOPDTModel& model) { m_model = model; }

    /** @brief Set the PID representation to compute gains for. */
    void setControllerForm(PIDForm form) { m_form = form; }

private:
    FOPDTModel m_model;
    PIDForm m_form{PIDForm::Parallel};
};

} // namespace Autotuning
} // namespace Control

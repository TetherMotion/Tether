/**
 * @file ChienHronesReswick.hpp
 * @brief Chien-Hrones-Reswick (CHR) tuning method family.
 *
 * @details
 * CHR provides variants for setpoint or regulator tuning with or without
 * overshoot specifications (no overshoot or ~20% overshoot targets).
 */

#pragma once

#include "tether/control/autotuning/classical/Common.hpp"
#include "tether/control/autotuning/AutotuningFramework.hpp"

namespace tether::control {
namespace Autotuning {

/**
 * @class ChienHronesReswick
 * @brief Chien-Hrones-Reswick (CHR) tuning family.
 *
 * @details
 * Provides tuning variants for setpoint or regulator control with options
 * for no overshoot or approximately 20% overshoot targets.
 */
class ChienHronesReswick : public AutotunerBase {
public:
    enum class Mode { SetpointNoOvershoot, Setpoint20Overshoot, RegulatorNoOvershoot, Regulator20Overshoot };

    /** @brief Human-readable name of the tuner. */
    std::string getName() const override;

    /** @brief Short description for UI or logs. */
    std::string getDescription() const override;

    /** @brief Offline/online mode of operation. */
    AutotuningMode getMode() const override;

    /** @brief Whether the given controller is compatible with this tuner. */
    bool isCompatible(const TunableController& controller) const override;

    /**
     * @brief Tune with CHR variant specified by @p mode using a FOPDT model.
     */
    TuningResult tune(TunableController& controller,
                      const ProcessModel* model = nullptr) override;

    /** @brief Compute CHR gains for a given mode and FOPDT model. */
    static PIDGains calculateGains(const FOPDTModel& model, PIDForm form, Mode mode);

    /** @brief Select the CHR tuning mode (setpoint/regulator, overshoot settings). */
    void setTuningMode(Mode mode) { m_mode = mode; }

    /** @brief Provide the FOPDT process model to base calculations on. */
    void setModel(const FOPDTModel& model) { m_model = model; }

    /** @brief Set PID form for returned gains. */
    void setControllerForm(PIDForm form) { m_form = form; }

private:
    FOPDTModel m_model;
    PIDForm m_form{PIDForm::Parallel};
    Mode m_mode{Mode::SetpointNoOvershoot};
};

} // namespace Autotuning
} // namespace tether::control

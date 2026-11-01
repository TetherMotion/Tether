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

    /** @brief Human-readable name of the tuner.
     *  @return Display name ("Chien-Hrones-Reswick")
     */
    std::string getName() const override;

    /** @brief Short description for UI or logs.
     *  @return One-line description of the CHR tuning family
     */
    std::string getDescription() const override;

    /** @brief Offline/online mode of operation.
     *  @return AutotuningMode::Offline — CHR needs a FOPDT model
     */
    AutotuningMode getMode() const override;

    /** @brief Whether the given controller is compatible with this tuner.
     *  @param controller Controller to check
     *  @return `true` if the tuner can produce gains for `controller`
     */
    bool isCompatible(const TunableController& controller) const override;

    /**
     * @brief Tune with CHR variant specified by @p mode using a FOPDT model.
     * @param controller Controller to update with computed parameters
     * @param model Optional process model; falls back to the model set via
     *        setModel() when nullptr
     * @return Tuning outcome including the computed gains and status
     */
    TuningResult tune(TunableController& controller,
                      const ProcessModel* model = nullptr) override;

    /** @brief Compute CHR gains for a given mode and FOPDT model.
     *  @param model Identified FOPDT process model
     *  @param form PID gain form the result is expressed in
     *  @param mode CHR variant (setpoint/regulator, with or without overshoot)
     *  @return Computed PID gains
     */
    static PIDGains calculateGains(const FOPDTModel& model, PIDForm form, Mode mode);

    /** @brief Select the CHR tuning mode (setpoint/regulator, overshoot settings).
     *  @param mode CHR variant to use
     */
    void setTuningMode(Mode mode) { m_mode = mode; }

    /** @brief Provide the FOPDT process model to base calculations on.
     *  @param model Identified process model the design is based on
     */
    void setModel(const FOPDTModel& model) { m_model = model; }

    /** @brief Set PID form for returned gains.
     *  @param form Gain form (e.g. Parallel, Standard) used for the result
     */
    void setControllerForm(PIDForm form) { m_form = form; }

private:
    FOPDTModel m_model;
    PIDForm m_form{PIDForm::Parallel};
    Mode m_mode{Mode::SetpointNoOvershoot};
};

} // namespace Autotuning
} // namespace tether::control

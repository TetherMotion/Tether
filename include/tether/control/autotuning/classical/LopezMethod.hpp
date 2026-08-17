/**
 * @file LopezMethod.hpp
 * @brief Lopez optimized tuning methods (ITAE/IAE/ISE criteria).
 *
 * @details
 * Computes PID gains by optimizing integral performance criteria (ITAE, IAE,
 * ISE) for either setpoint or disturbance response using FOPDT models.
 */

#pragma once

#include "tether/control/autotuning/classical/Common.hpp"
#include "tether/control/autotuning/AutotuningFramework.hpp"

namespace tether::control {
namespace Autotuning {

/**
 * @class LopezMethod
 * @brief Lopez optimized tuning methods (ITAE/IAE/ISE criteria).
 *
 * @details
 * Computes PID gains by optimizing integral performance criteria for
 * setpoint or disturbance responses using a FOPDT process model.
 */
class LopezMethod : public AutotunerBase {
public:
    enum class Criterion { ITAE, IAE, ISE };
    enum class ResponseType { Setpoint, Disturbance };

    /** @brief Human-readable name of the tuner. */
    std::string getName() const override;

    /** @brief Short description for UI or logs. */
    std::string getDescription() const override;

    /** @brief Offline/online mode of operation. */
    AutotuningMode getMode() const override;

    /** @brief Whether the given controller is compatible with this tuner. */
    bool isCompatible(const TunableController& controller) const override;

    /**
     * @brief Execute the tuner and apply computed parameters to @p controller.
     * @param controller Controller to update with computed parameters.
     * @param model Optional process model to base the design on.
     */
    TuningResult tune(TunableController& controller, const ProcessModel* model = nullptr) override;

    /** @brief Compute gains by optimizing the selected criterion for response type. */
    static PIDGains calculateGains(const FOPDTModel& model, PIDForm form, Criterion criterion, ResponseType response);

    /** @brief Choose the optimization criterion (ITAE, IAE, ISE). */
    void setCriterion(Criterion criterion) { m_criterion = criterion; }

    /** @brief Select setpoint or disturbance response variant. */
    void setResponseType(ResponseType type) { m_responseType = type; }

    /** @brief Provide the FOPDT model used for optimization. */
    void setModel(const FOPDTModel& model) { m_model = model; }

private:
    FOPDTModel m_model;
    Criterion m_criterion{Criterion::ITAE};
    ResponseType m_responseType{ResponseType::Setpoint};
    PIDForm m_form{PIDForm::Parallel};
};

} // namespace Autotuning
} // namespace tether::control

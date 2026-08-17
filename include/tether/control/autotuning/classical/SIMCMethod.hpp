/**
 * @file SIMCMethod.hpp
 * @brief SIMC (Skogestad) tuning rules.
 *
 * @details
 * Simple IMC-based tuning rules that provide straightforward tuning for
 * FOPDT and SOPDT models using a user-specified closed-loop time constant.
 */

#pragma once

#include "tether/control/autotuning/classical/Common.hpp"
#include "tether/control/autotuning/AutotuningFramework.hpp"

namespace tether::control {
namespace Autotuning {

/**
 * @class SIMCMethod
 * @brief SIMC (Skogestad) tuning rules.
 *
 * @details
 * Simple IMC-based rules providing practical tuning for FOPDT/SOPDT models
 * using a user-specified closed-loop time constant (tauC).
 */
class SIMCMethod : public AutotunerBase {
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
     * @brief Execute the tuner and apply computed parameters to @p controller.
     * @param controller Controller to update with computed parameters.
     * @param model Optional process model to base the design on.
     */
    TuningResult tune(TunableController& controller, const ProcessModel* model = nullptr) override;

    /** @brief Compute SIMC PID gains for FOPDT or SOPDT models. */
    static PIDGains calculateGains(const FOPDTModel& model, double tauC);
    static PIDGains calculateGains(const SOPDTModel& model, double tauC);

    /** @brief Set the closed-loop time constant parameter used by SIMC. */
    void setTauC(double tauC) { m_tauC = tauC; }

    /** @brief Provide the FOPDT model for design calculations. */
    void setModel(const FOPDTModel& model) { m_model = model; }

private:
    FOPDTModel m_model;
    double m_tauC{1.0};
};

} // namespace Autotuning
} // namespace tether::control

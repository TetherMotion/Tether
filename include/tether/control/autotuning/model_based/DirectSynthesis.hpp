/**
 * @file DirectSynthesis.hpp
 * @brief Direct synthesis autotuner (specify closed-loop behavior).
 *
 * @details
 * Computes a controller that achieves a specified closed-loop response by
 * directly solving for the controller from the desired transfer function.
 */

#pragma once

#include "tether/control/autotuning/AutotuningFramework.hpp"
#include "tether/control/autotuning/ClassicalTuningMethods.hpp"

namespace tether::control {
namespace Autotuning {

/**
 * @class DirectSynthesis
 * @brief Direct synthesis autotuner (specify closed-loop behavior).
 *
 * @details
 * Computes a controller achieving a desired closed-loop response by directly
 * solving for the controller from the specified target transfer function.
 */
class DirectSynthesis : public AutotunerBase {
public:
    std::string getName() const override { return "Direct Synthesis"; }
    std::string getDescription() const override {
        return "Specify desired closed-loop response, derive controller. "
               "C = T_d / (G * (1 - T_d))";
    }
    AutotuningMode getMode() const override { return AutotuningMode::Offline; }

    /**
     * @brief Check whether the controller supports direct synthesis design.
     * @param controller  Controller whose structure is validated.
     * @return true when the controller can take direct-synthesis PID gains.
     */
    bool isCompatible(const TunableController& controller) const override;

    /**
     * @brief Design controller via direct synthesis and apply parameters.
     * @param controller  Controller receiving the computed gains.
     * @param model       Process model (FOPDT or SOPDT) to design against.
     * @return TuningResult with applied gains or an error.
     */
    TuningResult tune(TunableController& controller,
                     const ProcessModel* model = nullptr) override;

    /**
     * @brief Set desired closed-loop time constant.
     * @param tau  Desired closed-loop time constant in seconds.
     */
    void setClosedLoopTimeConstant(double tau) { m_tauCL = tau; }

    /**
     * @brief Set closed-loop damping ratio used for second-order specs.
     * @param zeta  Damping ratio (0.707 ≈ Butterworth response).
     */
    void setClosedLoopDamping(double zeta) { m_zetaCL = zeta; }

    /**
     * @brief Design PID for a FOPDT target closed-loop time constant.
     * @param model   First-order-plus-dead-time process model.
     * @param tauCL   Desired closed-loop time constant in seconds.
     * @return PIDGains achieving the requested closed-loop response.
     */
    static PIDGains designForFOPDT(const FOPDTModel& model, double tauCL);

    /**
     * @brief Design PID for a SOPDT target closed-loop response.
     * @param model    Second-order-plus-dead-time process model.
     * @param tauCL    Desired closed-loop time constant in seconds.
     * @param zetaCL   Desired closed-loop damping ratio.
     * @return PIDGains achieving the requested closed-loop response.
     */
    static PIDGains designForSOPDT(const SOPDTModel& model, double tauCL, double zetaCL);

private:
    double m_tauCL{1.0};
    double m_zetaCL{0.707};
};

} // namespace Autotuning
} // namespace tether::control

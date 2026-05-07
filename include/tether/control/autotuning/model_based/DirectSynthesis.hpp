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

namespace Control {
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

    /** @brief Check whether the controller supports direct synthesis design. */
    bool isCompatible(const TunableController& controller) const override;

    /** @brief Design controller via direct synthesis and apply parameters. */
    TuningResult tune(TunableController& controller,
                     const ProcessModel* model = nullptr) override;

    /** @brief Set desired closed-loop time constant. */
    void setClosedLoopTimeConstant(double tau) { m_tauCL = tau; }

    /** @brief Set closed-loop damping ratio used for second-order specs. */
    void setClosedLoopDamping(double zeta) { m_zetaCL = zeta; }

    /** @brief Design PID for a FOPDT target closed-loop time constant. */
    static PIDGains designForFOPDT(const FOPDTModel& model, double tauCL);

    /** @brief Design PID for a SOPDT target closed-loop response. */
    static PIDGains designForSOPDT(const SOPDTModel& model, double tauCL, double zetaCL);

private:
    double m_tauCL{1.0};
    double m_zetaCL{0.707};
};

} // namespace Autotuning
} // namespace Control

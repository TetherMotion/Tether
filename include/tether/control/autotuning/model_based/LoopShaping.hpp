/**
 * @file LoopShaping.hpp
 * @brief Loop-shaping autotuner (frequency domain design).
 *
 * @details
 * Shapes the open-loop frequency response to meet bandwidth and robustness
 * objectives using tools from QFT and classical frequency-domain design.
 */

#pragma once

#include "tether/control/autotuning/AutotuningFramework.hpp"
#include "tether/control/autotuning/QFT.hpp"
#include <cmath>

namespace tether::control {
namespace Autotuning {

/**
 * @class LoopShaping
 * @brief Loop-shaping autotuner (frequency domain design).
 *
 * @details
 * Shapes the open-loop frequency response to meet bandwidth and robustness
 * objectives using frequency-domain design techniques.
 */
class LoopShaping : public AutotunerBase {
public:
    std::string getName() const override { return "Loop Shaping"; }
    std::string getDescription() const override {
        return "Frequency-domain controller design. "
               "Shape open-loop to achieve desired bandwidth and margins.";
    }
    AutotuningMode getMode() const override { return AutotuningMode::Offline; }

    /** @brief Check compatibility for frequency-domain design. */
    /**
     * @brief Whether this tuner can drive the given controller type.
     *
     * Loop shaping requires a controller whose structure can express
     * lead/lag compensation.
     * @param controller  Controller descriptor to check.
     * @return true if the controller type is supported.
     */
    bool isCompatible(const TunableController& controller) const override;
    
    /**
     * @brief Perform loop-shaping design and convert to controller parameters.
     *
     * Shapes L(jw) = C(jw)P(jw) to meet the configured crossover,
     * margins, and slope/rolloff targets, then maps the compensator onto
     * the controller's parameter set.
     * @param controller  Controller to tune; parameters overwritten on
     *                    success.
     * @param model       Optional process model; null uses the tuner's
     *                    identified model.
     * @return TuningResult with parameters and achieved margins;
     *         `success == false` when no feasible design met targets.
     */
    TuningResult tune(TunableController& controller,
                     const ProcessModel* model = nullptr) override;
    
    /**
     * @brief Set the desired gain crossover frequency.
     * @param omegaC  Target |L(jw)|=1 frequency [rad/s].
     */
    void setCrossoverFrequency(double omegaC) { m_omegaC = omegaC; }

    /**
     * @brief Set the required phase margin.
     * @param PM  Target phase margin [deg]; typical 30-60.
     */
    void setPhaseMargin(double PM) { m_phaseMargin = PM; }

    /**
     * @brief Set the required gain margin.
     * @param GM  Target gain margin [dB]; typical >= 6.
     */
    void setGainMargin(double GM) { m_gainMargin = GM; }

    /**
     * @brief Set the low-frequency asymptote slope.
     * @param slope  Slope in decades per decade (-1 = integrator,
     *               -2 = double integrator).
     */
    void setLowFreqSlope(int slope) { m_lowFreqSlope = slope; }

    /**
     * @brief Set the high-frequency rolloff rate.
     * @param rolloff  Asymptotic rolloff past the plant poles
     *                 (-1 or -2 typical).
     */
    void setHighFreqRolloff(int rolloff) { m_highFreqRolloff = rolloff; }

    /**
     * @brief Design a lead/lag compensator meeting crossover and margin.
     *
     * Solves for C(s) so that |L(jw)| = 1 at @p omegaC with the requested
     * phase margin, given the plant model.
     * @param plant        Process model the loop is shaped around.
     * @param omegaC       Desired gain crossover frequency [rad/s].
     * @param phaseMargin  Desired phase margin [deg].
     * @return Compensator transfer function C(s).
     */
    static TransferFunction designLeadLag(const ProcessModel& plant,
                                         double omegaC, double phaseMargin);

private:
    double m_omegaC{1.0};
    double m_phaseMargin{45.0};
    double m_gainMargin{6.0};
    int m_lowFreqSlope{-1};
    int m_highFreqRolloff{-2};
};

} // namespace Autotuning
} // namespace tether::control

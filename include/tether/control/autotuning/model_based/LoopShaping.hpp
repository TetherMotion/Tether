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
    bool isCompatible(const TunableController& controller) const override;
    
    /** @brief Perform loop-shaping design and convert to controller parameters. */
    TuningResult tune(TunableController& controller,
                     const ProcessModel* model = nullptr) override;
    
    /** @brief Set target crossover frequency for loop shaping. */
    void setCrossoverFrequency(double omegaC) { m_omegaC = omegaC; }

    /** @brief Set desired phase margin in degrees. */
    void setPhaseMargin(double PM) { m_phaseMargin = PM; }

    /** @brief Set desired gain margin in dB. */
    void setGainMargin(double GM) { m_gainMargin = GM; }

    /** @brief Set low frequency slope target (e.g., -1, -2). */
    void setLowFreqSlope(int slope) { m_lowFreqSlope = slope; }

    /** @brief Set high frequency rolloff target (negative integer). */
    void setHighFreqRolloff(int rolloff) { m_highFreqRolloff = rolloff; }

    /** @brief Design a lead/lag TransferFunction to meet crossover/PM specs. */
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

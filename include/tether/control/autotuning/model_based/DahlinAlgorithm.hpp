/**
 * @file DahlinAlgorithm.hpp
 * @brief Dahlin's algorithm autotuner (digital controller for first-order responses).
 *
 * @details
 * Discrete-time design method for achieving a desired closed-loop step
 * response for first-order processes with or without delay.
 */

#pragma once

#include "tether/control/autotuning/AutotuningFramework.hpp"
#include <array>

namespace tether::control {
namespace Autotuning {

/**
 * @class DahlinAlgorithm
 * @brief Dahlin's algorithm autotuner (digital controller for first-order responses).
 *
 * @details
 * Discrete-time design method for achieving a desired closed-loop step
 * response for first-order processes with optional dead time handling.
 */
class DahlinAlgorithm : public AutotunerBase {
public:
    std::string getName() const override { return "Dahlin's Algorithm"; }
    std::string getDescription() const override {
        return "Digital controller for first-order closed-loop response. "
               "Accounts for process dead time in discrete design.";
    }
    AutotuningMode getMode() const override { return AutotuningMode::Offline; }

    /**
     * @brief Check compatibility for Dahlin's discrete-time design.
     * @param controller  Controller whose structure is validated.
     * @return true when the controller accepts discrete Dahlin gains.
     */
    bool isCompatible(const TunableController& controller) const override;
    
    /**
     * @brief Design digital controller and apply discrete gains to controller.
     * @param controller  Controller receiving the computed gains.
     * @param model       Optional process model to design against.
     * @return TuningResult with applied gains or an error.
     */
    TuningResult tune(TunableController& controller,
                     const ProcessModel* model = nullptr) override;
    
    /**
     * @brief Set sample time used for discrete design.
     * @param Ts  Sampling period in seconds.
     */
    void setSampleTime(double Ts) { m_Ts = Ts; }

    /**
     * @brief Set lambda parameter used in discrete design mapping.
     * @param lambda  Desired closed-loop time constant (seconds).
     */
    void setLambda(double lambda) { m_lambda = lambda; }

    /** @brief Return digital controller coefficients (e.g., numerator array).
     *  
     *  @param model   FOPDT process model.
     *  @param Ts      Sampling period in seconds.
     *  @param lambda  Desired closed-loop time constant (seconds).
     *  @return Array of 3 coefficients usable in discrete controller realization.
     */
    static std::array<double, 3> designDigital(const FOPDTModel& model, 
                                               double Ts, double lambda);

private:
    double m_Ts{0.01};
    double m_lambda{1.0};
};

} // namespace Autotuning
} // namespace tether::control

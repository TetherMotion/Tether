/**
 * @file DeadbeatControl.hpp
 * @brief Deadbeat control autotuner (finite settling time).
 *
 * @details
 * Designs digital controllers that achieve zero error in a finite number
 * of sampling periods for systems that admit deadbeat control solutions.
 */

#pragma once

#include "tether/control/autotuning/AutotuningFramework.hpp"
#include "tether/control/autotuning/model_based/PolePlacement.hpp"
#include <vector>

namespace Control {
namespace Autotuning {

/**
 * @class DeadbeatControl
 * @brief Deadbeat control autotuner (finite settling time).
 *
 * @details
 * Designs digital controllers that achieve zero error in a finite number
 * of sampling periods for systems that admit deadbeat solutions.
 */
class DeadbeatControl : public AutotunerBase {
public:
    std::string getName() const override { return "Deadbeat Control"; }
    std::string getDescription() const override {
        return "Finite settling time controller. Output reaches setpoint "
               "in minimum samples. Warning: can have large control effort.";
    }
    AutotuningMode getMode() const override { return AutotuningMode::Offline; }

    /** @brief Check compatibility for deadbeat controller design. */
    bool isCompatible(const TunableController& controller) const override;
    
    /** @brief Compute deadbeat controller coefficients and apply to controller. */
    TuningResult tune(TunableController& controller,
                     const ProcessModel* model = nullptr) override;
    
    /** @brief Set sample time for discrete deadbeat design. */
    void setSampleTime(double Ts) { m_Ts = Ts; }

    /** @brief Set desired number of settling samples for deadbeat design. */
    void setSettlingSamples(int N) { m_settlingN = N; }

    /**
     * @brief Compute deadbeat controller coefficients for given plant polynomials.
     * @return Vector of controller coefficients.
     */
    static std::vector<double> design(const double* A, const double* B,
                                     const double* C, int n, int m, int p,
                                     double Ts, int settlingN = 0);

private:
    double m_Ts{0.01};
    int m_settlingN{0};
};

} // namespace Autotuning
} // namespace Control

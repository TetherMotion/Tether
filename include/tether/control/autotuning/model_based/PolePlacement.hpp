/**
 * @file PolePlacement.hpp
 * @brief Pole placement based autotuner.
 *
 * @details
 * Designs state-feedback controllers by placing closed-loop poles at
 * desired locations to achieve specified transient response characteristics.
 */

#pragma once

#include "tether/control/autotuning/AutotuningFramework.hpp"
#include "tether/control/autotuning/ClassicalTuningMethods.hpp"
#include <complex>
#include <vector>

namespace tether::control {
namespace Autotuning {

/**
 * @class PolePlacement
 * @brief Pole placement based autotuner.
 *
 * @details
 * Designs state-feedback controllers by placing closed-loop poles at
 * desired locations to achieve specified transient response characteristics.
 */
class PolePlacement : public AutotunerBase {
public:
    std::string getName() const override { return "Pole Placement"; }
    std::string getDescription() const override {
        return "State feedback design with specified closed-loop poles. "
               "Achieves desired transient response characteristics.";
    }
    AutotuningMode getMode() const override { return AutotuningMode::Offline; }

    /** @brief Check controller compatibility for pole-placement approaches. */
    bool isCompatible(const TunableController& controller) const override;
    
    /**
     * @brief Compute gains via pole placement and convert to controller parameters.
     */
    TuningResult tune(TunableController& controller,
                     const ProcessModel* model = nullptr) override;
    
    /** @brief Supply desired closed-loop poles explicitly. */
    void setDesiredPoles(const std::vector<std::complex<double>>& poles);

    /** @brief Compute pole locations from desired settling time and overshoot. */
    void setPolesFromSpecs(double settlingTime, double overshoot);

    /**
     * @brief Compute state-feedback gain for linear system (A,B) and desired poles.
     * @return Vector of gains K.
     */
    static std::vector<double> computeGain(const double* A, const double* B,
                                          int n, const std::vector<std::complex<double>>& poles);

    /** @brief Convert state-feedback gains into PID gains for a controller of given order. */
    static PIDGains stateFeedbackToPID(const std::vector<double>& K, int order);

    /** @brief Provide system matrices for state-space-based design. */
    void setSystemMatrices(const double* A, const double* B, int n, int m);
    
private:
    std::vector<std::complex<double>> m_desiredPoles;
    std::vector<double> m_A;
    std::vector<double> m_B;
    int m_n{0}, m_m{0};
};

} // namespace Autotuning
} // namespace tether::control

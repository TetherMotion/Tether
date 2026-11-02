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

    /**
     * @brief Check controller compatibility for pole-placement approaches.
     * @param controller  Controller whose structure is validated.
     * @return true when the controller accepts PID gains derived from
     *         state feedback.
     */
    bool isCompatible(const TunableController& controller) const override;
    
    /**
     * @brief Compute gains via pole placement and convert to controller parameters.
     * @param controller  Controller receiving the computed gains.
     * @param model       Optional process model used to derive system
     *                    matrices when none were set explicitly.
     * @return TuningResult with applied gains or an error.
     */
    TuningResult tune(TunableController& controller,
                     const ProcessModel* model = nullptr) override;
    
    /**
     * @brief Supply desired closed-loop poles explicitly.
     * @param poles  Desired pole locations (s-plane); count must match
     *               the system order used in tune().
     */
    void setDesiredPoles(const std::vector<std::complex<double>>& poles);

    /**
     * @brief Compute pole locations from desired settling time and overshoot.
     * @param settlingTime  2% settling time in seconds.
     * @param overshoot     Fractional overshoot (0..1, e.g. 0.1 = 10%).
     */
    void setPolesFromSpecs(double settlingTime, double overshoot);

    /**
     * @brief Compute state-feedback gain for linear system (A,B) and desired poles.
     * @param A      n×n system matrix (row-major).
     * @param B      n×1 input matrix.
     * @param n      System order.
     * @param poles  Desired pole locations.
     * @return Vector of gains K.
     */
    static std::vector<double> computeGain(const double* A, const double* B,
                                          int n, const std::vector<std::complex<double>>& poles);

    /**
     * @brief Convert state-feedback gains into PID gains for a controller of given order.
     * @param K      State-feedback gain vector from computeGain().
     * @param order  Target controller order (2 = PI, 3 = PID).
     * @return equivalent PIDGains.
     */
    static PIDGains stateFeedbackToPID(const std::vector<double>& K, int order);

    /**
     * @brief Provide system matrices for state-space-based design.
     * @param A  n×n system matrix (row-major, copied).
     * @param B  n×m input matrix (row-major, copied).
     * @param n  State dimension.
     * @param m  Input dimension.
     */
    void setSystemMatrices(const double* A, const double* B, int n, int m);
    
private:
    std::vector<std::complex<double>> m_desiredPoles;
    std::vector<double> m_A;
    std::vector<double> m_B;
    int m_n{0}, m_m{0};
};

} // namespace Autotuning
} // namespace tether::control

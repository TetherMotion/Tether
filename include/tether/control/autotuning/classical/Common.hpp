/**
 * @file Common.hpp
 * @brief Shared utilities and types for classical autotuning methods.
 *
 * @details
 * Defines common structures such as PIDGains and ProcessIdentification
 * helpers used by the classical autotuning algorithms.
 */

#pragma once

#include "tether/control/autotuning/AutotuningFramework.hpp"
#include <vector>

namespace tether::control {
namespace Autotuning {

/**
 * @brief Common PID gains used by classical methods.
 *
 * Members include both parallel and standard-form parameters. Helper
 * methods convert between forms or validate the gains.
 */
struct PIDGains {
    double Kp{0.0};   ///< Proportional gain
    double Ki{0.0};   ///< Integral gain (1/Ti form or direct Ki)
    double Kd{0.0};   ///< Derivative gain (Kd)
    double Ti{0.0};   ///< Integral time constant
    double Td{0.0};   ///< Derivative time constant
    double Tf{0.0};   ///< Filter time constant for derivative action

    /** @brief Convert stored gains to standard (series) form. */
    void toStandardForm();

    /** @brief Convert stored gains to parallel form. */
    void toParallelForm();

    /** @brief Return true if the gains represent a valid controller. */
    bool isValid() const;
};

/**
 * @brief Process identification helper methods.
 *
 * Implements common FOPDT/SOPDT identification routines used by classical
 * autotuners (tangent, area, two-point, least-squares, etc.).
 */
class ProcessIdentification {
public:
    /** @brief Tangent (inflection) method for FOPDT estimation. */
    static FOPDTModel tangentMethod(const std::vector<double>& time,
                                    const std::vector<double>& response,
                                    double stepSize);

    /** @brief Area method for FOPDT estimation (integral approach). */
    static FOPDTModel areaMethod(const std::vector<double>& time,
                                 const std::vector<double>& response,
                                 double stepSize);

    /** @brief Two-point method using fractional time constants. */
    static FOPDTModel twoPointMethod(const std::vector<double>& time,
                                     const std::vector<double>& response,
                                     double stepSize,
                                     double t1Fraction = 0.283,
                                     double t2Fraction = 0.632);

    /** @brief Least-squares fit of a FOPDT model to step response data. */
    static FOPDTModel leastSquaresFit(const std::vector<double>& time,
                                      const std::vector<double>& response,
                                      double stepSize);

    /** @brief Identify a SOPDT model from step response data. */
    static SOPDTModel identifySOPDT(const std::vector<double>& time,
                                    const std::vector<double>& response,
                                    double stepSize);

    /** @brief Estimate ultimate gain and period from a FOPDT model. */
    static std::pair<double, double> estimateUltimate(const FOPDTModel& model);
};

} // namespace Autotuning
} // namespace tether::control

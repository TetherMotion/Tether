#pragma once

#include <cstddef>
#include <vector>

#include <tether/identification/DenseLinearAlgebra.hpp>

namespace Identification {

/**
 * @brief Results representing the minimal physically identifiable set of rigid body parameters.
 *
 * In mechanical systems, many parameters (like link inertias Ixx, Iyy, Izz) often appear 
 * linearly dependent in the equations of motion (EOM). A base parameter set is the minimal 
 * subset or linear combination of physical parameters that can actually be excited and 
 * thus uniquely estimated from joint torque/motion data via regression.
 */
struct BaseParameterIdentificationResult {
    Vector base_parameters;                  ///< The aggregated parameter values mapping to physical quantities (like mass*length).
    std::vector<size_t> identifiable_columns;///< Indices of the column space (regressor columns) that are mathematically independent.
    Matrix reduced_regressor;                ///< The pruned data matrix W_base mapping motion to torque: Torque = W_base * base_parameters.
    size_t rank{0};                          ///< The numerical rank found (the number of independent base parameters).
    double condition_number{0.0};            ///< Condition number of the reduced regressor. Values > 100 predict extreme sensitivity to sensor noise.
};

/**
 * @brief Parameterization for a periodic excitation trajectory defined by fundamental harmonics.
 *
 * Mechanical linkages require "rich" trajectories (Excitation Trajectories) 
 * vibrating across many frequencies to expose inertia, Coriolis, and gravity parameters distinctively.
 * A finite Fourier series natively satisfies velocity/acceleration boundary conditions (zero at start/end) 
 * ensuring smooth physical actuation.
 */
struct FourierExcitationTrajectory {
    Vector offsets;               ///< DC offsets forming the mean joint position configuration.
    Matrix sine_coefficients;     ///< Fourier sine amplitudes driving each joint harmonic.
    Matrix cosine_coefficients;   ///< Fourier cosine amplitudes driving each joint harmonic.
    double base_frequency{1.0};   ///< The fundamental period (1/period) of the movement [Hz].
    double duration{1.0};         ///< The total experiment time in seconds.

    /**
     * @brief Computes the trajectory position evaluated at a given time tick.
     * @param time The elapsed time [s].
     * @return A vector output representing the joint positions [rad or m] at the tick.
     */
    Vector sample(double time) const;
};

/**
 * @brief Non-periodic smooth point-to-point motion mapping defined over a normalized elapsed time.
 * 
 * BSplines provide localized control over trajectory segments, enabling 
 * the optimizer to respect strict joint limits and workspace boundaries better than Fourier series.
 */
struct BSplineExcitationTrajectory {
    Matrix control_points; ///< Discretized anchor points dictating the B-Spline shape.
    double duration{1.0};  ///< The total experiment time mapping out the spline curve.

    /**
     * @brief Resolves the B-Spline value at physical time t. 
     * @param time The elapsed time [s].
     * @return Evaluating the point on the spline for all joints at `time`.
     */
    Vector sample(double time) const;
};

/**
 * @brief Estimator solving the linear system Torque = Regressor * Parameters.
 * 
 * Takes the overdetermined system of equations sampled during the experiment and 
 * isolates the minimal base parameter combinations that characterize the mechanical plant.
 */
class BaseParameterEstimator {
public:
    /**
     * @brief Reconstructs parameters using Singular Value Decomposition (SVD).
     * 
     * Uses SVD to extract the condition number and strictly define the null space.
     * While computationally slower than QR, SVD is the canonical, numerically bulletproof 
     * approach for rank-deficient observation matrices found in mechanical identification.
     * 
     * @param regressor The full kinematic/dynamic regression matrix $W$.
     * @param torque The measured joint torques corresponding to the regressor timeseries.
     * @param tolerance Threshold below which a singular value indicates an unobservable parameter dependency.
     * @return A consolidated estimation struct isolating base parameters.
     */
    static BaseParameterIdentificationResult estimateSVD(const Matrix& regressor,
                                                         const Vector& torque,
                                                         double tolerance = 1e-6);

    /**
     * @brief Reconstructs parameters using QR factorization with column pivoting.
     * 
     * A massively faster counterpart to SVD using orthogonal triangular decomposition. 
     * The permutation matrix $P$ inherently sorts equations by observability significance.
     * 
     * @param regressor The full dynamic regression matrix $W$.
     * @param torque Measured joint exertion effort.
     * @param tolerance Threshold detecting numerical column dependency during orthogonalization.
     */
    static BaseParameterIdentificationResult estimateQR(const Matrix& regressor,
                                                        const Vector& torque,
                                                        double tolerance = 1e-6);
};

/**
 * @brief Tools validating and generating optimal trajectories to map a robot's parameter equations.
 */
class ExcitationTrajectoryOptimizer {
public:
    static FourierExcitationTrajectory optimizeFourier(size_t joints,
                                                       size_t harmonics,
                                                       double duration,
                                                       const Matrix& seed_regressor = {});
    static BSplineExcitationTrajectory optimizeBSpline(size_t joints,
                                                       size_t control_points,
                                                       double duration,
                                                       const Matrix& seed_regressor = {});
    
    /**
     * @brief Grades a trajectory by analyzing its informational Fisher matrix $(W^T W)$.
     * 
     * A larger score (typically tied to the condition number or trace of the covariance)
     * indicates the trajectory exercises the mechanics independently, rejecting 
     * sensor noise mapping to the identified parameters.
     * 
     * @param regressor The motion-generated kinematic observation matrix.
     * @return Quality scaler (lower is theoretically worse conditioning).
     */
    static double evaluateInformationScore(const Matrix& regressor);
};

} // namespace Identification
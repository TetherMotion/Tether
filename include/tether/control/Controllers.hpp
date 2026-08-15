/**
 * @file Controllers.hpp
 * @brief Unified Controller Framework - Include All Controllers
 * 
 * @details
 * This is the main entry point for the controller framework.
 * Include this file to access all controller types.
 * 
 * ## Controller Framework Overview
 * 
 * This framework provides a comprehensive collection of control algorithms
 * suitable for embedded motion control applications.
 * 
 * ### Controller Categories
 * 
 * | Category | Controllers | Use Case |
 * |----------|-------------|----------|
 * | Classical PID | P, PD, PI, PID, PID-2DOF | General purpose |
 * | Advanced PID | Bang-Bang, PD+, Dual-Loop | Special requirements |
 * | Fractional | FOPID (PI^λD^μ) | Robust tuning |
 * | Optimal | LQR, LQG, LQI | State-space systems |
 * | Robust | H2, H∞ | Uncertain systems |
 * | Learning | ILC, Repetitive | Repetitive tasks |
 * | Composite | Cascade, Parallel, Switching | Combined control |
 * 
 * ### Quick Start
 * 
 * ```cpp
 * #include "control/Controllers.hpp"
 * using namespace Control;
 * 
 * // Create a PID controller
 * PIDController pid;
 * pid.setGains(1.0, 0.1, 0.05);
 * pid.setOutputLimits(-100, 100);
 * pid.setAntiWindupMethod(AntiWindupMethod::BackCalculation);
 * 
 * // Control loop
 * while (running) {
 *     ControllerInput input;
 *     input.reference = setpoint;
 *     input.measured = sensor.read();
 *     input.dt = loopTime;
 *     
 *     auto output = pid.compute(input);
 *     actuator.set(output.control);
 * }
 * ```
 * 
 * ### Using Controllers as Backends
 * 
 * Controllers can be composed and used as backends for motion controllers:
 * 
 * ```cpp
 * // Position control with LQR backend
 * LQRController lqr;
 * lqr.setWeights(Q, R, n);
 * lqr.setSystemMatrices(A, B, n, m);
 * lqr.design();
 * 
 * PositionController posCtrl;
 * posCtrl.setBackend(&lqr);
 * 
 * // Or cascade control: position → velocity → torque
 * CascadeController cascade;
 * cascade.setOuterController(&positionPID);
 * cascade.setInnerController(&velocityPID);
 * 
 * TorqueController torqueCtrl;
 * torqueCtrl.setBackend(&cascade);
 * ```
 * 
 * ### Anti-Windup Methods
 * 
 * For PID-like controllers:
 * ```cpp
 * pid.setAntiWindupMethod(AntiWindupMethod::None);          // No anti-windup
 * pid.setAntiWindupMethod(AntiWindupMethod::Clamping);      // Simple clamp
 * pid.setAntiWindupMethod(AntiWindupMethod::BackCalculation);  // Recommended
 * pid.setAntiWindupMethod(AntiWindupMethod::Conditional);   // Conditional integration
 * pid.setAntiWindupMethod(AntiWindupMethod::Tracking);      // Tracking anti-windup
 * ```
 * 
 * ### Derivative Filters
 * 
 * ```cpp
 * pid.setDerivativeFilter(DerivativeFilter::FirstOrder);
 * pid.setDerivativeFilterCoeff(0.1);  // N = 10
 * ```
 * 
 * ### Auto-Tuning
 * 
 * ```cpp
 * // Auto-tune PID using Ziegler-Nichols
 * pid.autoTune(TuningMethod::ZieglerNichols, Ku, Tu);
 * 
 * // Or use other methods
 * pid.autoTune(TuningMethod::CohenCoon, ...);
 * pid.autoTune(TuningMethod::SIMC, ...);
 * ```
 * 
 * @author ESP32EtherCAT Project
 * @version 1.0
 * @date 2024
 * 
 * @see ControllerBase.hpp
 * @see PIDControllers.hpp
 * @see FractionalPID.hpp
 * @see StateSpaceControllers.hpp
 * @see RobustControllers.hpp
 * @see LearningControllers.hpp
 * @see CompositeControllers.hpp
 */

#pragma once

// Base interface and common types
#include "control/ControllerBase.hpp"

// Classical and advanced PID controllers
#include "control/PIDControllers.hpp"

// Fractional-order PID
#include "control/FractionalPID.hpp"

// State-space optimal controllers (LQR, LQG, LQI)
#include "control/StateSpaceControllers.hpp"

// Kalman filters (linear and extended)
#include "control/KalmanFilter.hpp"
#include "control/ExtendedKalmanFilter.hpp"

// Robust controllers (H2, H∞)
#include "control/RobustControllers.hpp"

// Learning-based controllers (ILC, Repetitive)
#include "control/LearningControllers.hpp"

// Composite controllers (Cascade, Parallel, Switching, Wrappers)
#include "control/CompositeControllers.hpp"

/**
 * @namespace Control
 * @brief Control algorithm namespace
 * 
 * Contains all controller classes and utilities.
 * 
 * ## Key Classes
 * 
 * ### Base Interface
 * - ControllerBase - Abstract base for all controllers
 * - StateEstimator - Interface for state estimators (Kalman, etc.)
 * - SystemModel - Interface for plant models
 * 
 * ### Classical PID Family
 * - PController - Proportional only
 * - PDController - Proportional + Derivative
 * - PIController - Proportional + Integral
 * - PIDController - Full PID with tuning
 * - PID2DOFController - Two-degree-of-freedom PID
 * - BangBangController - On-off control
 * - PDPlusController - PD with gravity/feedforward
 * - DualLoopPIDController - Cascade position/velocity
 * 
 * ### Fractional-Order
 * - FractionalPIDController - PI^λD^μ controller
 * 
 * ### Optimal Control
 * - LQRController - Linear Quadratic Regulator
 * - KalmanFilter - Linear state estimator (Eigen-based)
 * - ExtendedKalmanFilter - Nonlinear state estimator (Eigen-based)
 * - LQGController - LQR + Kalman Filter
 * - LQIController - LQR with integral action
 * 
 * ### Robust Control
 * - H2Controller - H2 optimal control
 * - HInfinityController - H∞ robust control
 * - MuSynthesisFramework - μ-synthesis support
 * 
 * ### Learning Control
 * - PTypeILC - P-type Iterative Learning Control
 * - PDTypeILC - PD-type ILC
 * - PhaseLeadILC - Phase-lead ILC
 * - NormOptimalILC - Norm-optimal ILC
 * - CurrentIterationLearning - ILC + feedback
 * - RepetitiveController - Repetitive control
 * 
 * ### Composite Controllers
 * - CascadeController - Master-slave cascade
 * - FeedforwardController - FF + FB combination
 * - ParallelController - Weighted parallel sum
 * - SwitchingController - Gain scheduling/switching
 * - RateLimiterWrapper - Rate limiting
 * - DeadbandWrapper - Deadband/hysteresis
 * - FilterWrapper - Output filtering
 * 
 * ### Utilities
 * - ControllerFactory - Create controllers by type/name
 * - StateSpace namespace - Matrix operations
 */
namespace Control {

/**
 * @brief Controller version information
 */
constexpr const char* CONTROLLER_VERSION = "1.0.0";

/**
 * @brief Get controller framework version
 */
inline const char* getVersion() { return CONTROLLER_VERSION; }

} // namespace Control

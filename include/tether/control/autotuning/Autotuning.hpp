/**
 * @file Autotuning.hpp
 * @brief Main include header for Controller Autotuning Framework
 * 
 * @details
 * This header includes all autotuning-related components. Include this
 * single header to access the complete autotuning framework.
 * 
 * ## Framework Overview
 * 
 * The autotuning framework provides comprehensive controller tuning
 * capabilities organized into several categories:
 * 
 * ### Core Framework (AutotuningFramework.hpp)
 * - Base classes: AutotunerBase, TunableController
 * - Process models: FOPDTModel, SOPDTModel, IPDTModel
 * - Cost functions: ISE, IAE, ITAE, ITSE
 * 
 * ### Optimization (OptimizationAlgorithms.hpp)
 * - Gradient-based: GradientDescent, BFGS, Powell
 * - Evolutionary: GA, PSO, DE, ACO
 * - Direct search: Nelder-Mead, Simulated Annealing
 * - Bayesian: Gaussian Process optimization
 * 
 * ### Robust Control (MuSynthesis.hpp, QFT.hpp, SlidingModeControl.hpp)
 * - μ-Synthesis with D-K iteration
 * - Quantitative Feedback Theory
 * - Sliding Mode Control variants
 * 
 * ### Classical Methods (ClassicalTuningMethods.hpp)
 * - Ziegler-Nichols (step response & ultimate cycle)
 * - Cohen-Coon, CHR, Tyreus-Luyben
 * - SIMC, AMIGO, Lambda tuning
 * 
 * ### Model-Based Methods (ModelBasedMethods.hpp)
 * - IMC-based PID design
 * - Pole placement, Loop shaping
 * - Smith Predictor, Dahlin, Deadbeat
 * 
 * ### Adaptive Methods (AdaptiveMethods.hpp)
 * - Gain scheduling
 * - MRAC, STR, ESC
 * - Fuzzy logic, Neural networks
 * - MMAC
 * 
 * ### LQR-Based Methods (LQRTuning.hpp)
 * - Q/R optimization
 * - Loop Transfer Recovery
 * - IFT, VRFT, FRIT
 * 
 * ### Hybrid Methods (HybridMethods.hpp)
 * - Z-N + Optimization
 * - GA-PID, PSO-PID
 * - Neural PID, Fuzzy PID
 * 
 * ### Industrial Autotuners (IndustrialAutotuners.hpp)
 * - Relay feedback autotuner
 * - Step response autotuner
 * - Pattern recognition autotuner
 * 
 * ## Usage Example
 * 
 * ```cpp
 * #include <tether/control/autotuning/Autotuning.hpp>
 * 
 * using namespace Control::Autotuning;
 * 
 * // Create a tunable PID controller
 * class MyPIDController : public TunableController {
 *     // ... implementation
 * };
 * 
 * // Create an autotuner
 * RelayFeedbackAutotuner tuner;
 * tuner.setAmplitude(5.0);
 * tuner.setTuningRule(RelayFeedbackAutotuner::TuningRule::IMC_Moderate);
 * 
 * // Start tuning
 * MyPIDController controller;
 * tuner.start();
 * 
 * while (!tuner.isComplete()) {
 *     double y = readSensor();
 *     double r = setpoint;
 *     double u = tuner.update(y, r, lastControl, dt);
 *     writeOutput(u);
 * }
 * 
 * TuningResult result = tuner.tune(controller);
 * if (result.success) {
 *     // Controller is now tuned
 * }
 * ```
 * 
 * @author ESP32EtherCAT Project
 * @version 2.0
 */

#pragma once

// Core framework
#include "AutotuningFramework.hpp"

// Optimization algorithms
#include "OptimizationAlgorithms.hpp"

// Robust control methods
#include "MuSynthesis.hpp"
#include "QFT.hpp"
#include "SlidingModeControl.hpp"

// Classical tuning methods
#include "ClassicalTuningMethods.hpp"

// Model-based methods
#include "ModelBasedMethods.hpp"

// Adaptive methods
#include "AdaptiveMethods.hpp"

// LQR-based tuning
#include "LQRTuning.hpp"

// Hybrid methods
#include "HybridMethods.hpp"

// Industrial autotuners
#include "IndustrialAutotuners.hpp"

namespace Control {
namespace Autotuning {

/**
 * @brief Framework version information
 */
struct Version {
    static constexpr int major = 2;
    static constexpr int minor = 0;
    static constexpr int patch = 0;
    
    static constexpr const char* string() { return "2.0.0"; }
};

/**
 * @brief Create an autotuner by name
 * 
 * Factory function to create autotuners from string names.
 * Useful for configuration-based selection.
 * 
 * @param name Autotuner name (e.g., "RelayFeedback", "ZieglerNichols")
 * @return unique_ptr to autotuner, or nullptr if not found
 */
std::unique_ptr<OnlineAutotuner> createOnlineAutotuner(const std::string& name);
std::unique_ptr<OfflineAutotuner> createOfflineAutotuner(const std::string& name);

/**
 * @brief List available autotuners
 */
std::vector<std::string> listOnlineAutotuners();
std::vector<std::string> listOfflineAutotuners();

/**
 * @brief Get autotuner description
 */
std::string getAutotunerDescription(const std::string& name);

} // namespace Autotuning
} // namespace Control

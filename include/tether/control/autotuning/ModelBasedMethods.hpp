/**
 * @file ModelBasedMethods.hpp
 * @brief Model-Based Controller Tuning Methods
 * 
 * @details
 * This file implements model-based controller tuning methods that use
 * explicit process models for controller design.
 * 
 * ## Implemented Methods
 * 
 * ### Internal Model Control (IMC)
 * Controller designed as inverse of process model with filter
 * 
 * ### Pole Placement
 * Place closed-loop poles at desired locations
 * 
 * ### Loop Shaping
 * Shape loop transfer function in frequency domain
 * 
 * ### Direct Synthesis
 * Specify desired closed-loop and solve for controller
 * 
 * ### Smith Predictor
 * Dead-time compensation for processes with delay
 * 
 * ### Dahlin's Algorithm
 * Digital controller design for first-order response
 * 
 * ### Deadbeat Control
 * Finite settling time controller
 * 
 * ### Minimum Variance Control
 * Minimize output variance for stochastic systems
 * 
 * @author ESP32EtherCAT Project
 * @version 2.0
 */

#pragma once

#include "AutotuningFramework.hpp"
#include "ClassicalTuningMethods.hpp"
#include "QFT.hpp"  // For TransferFunction
#include <complex>
#include <vector>

// Individual model-based autotuning classes are defined in the
// `tether/control/autotuning/model_based` subdirectory. Include the
// specific headers below for convenience.

#include "tether/control/autotuning/model_based/IMCDesign.hpp"
#include "tether/control/autotuning/model_based/PolePlacement.hpp"
#include "tether/control/autotuning/model_based/LoopShaping.hpp"
#include "tether/control/autotuning/model_based/DirectSynthesis.hpp"
#include "tether/control/autotuning/model_based/SmithPredictor.hpp"
#include "tether/control/autotuning/model_based/DahlinAlgorithm.hpp"
#include "tether/control/autotuning/model_based/DeadbeatControl.hpp"
#include "tether/control/autotuning/model_based/MinimumVarianceControl.hpp"

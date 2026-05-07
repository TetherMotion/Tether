/**
 * @file ClassicalTuningMethods.hpp
 * @brief Classical and Heuristic Controller Tuning Methods
 * 
 * @details
 * This file implements classical controller tuning methods that have been
 * developed and refined over decades of industrial practice. These methods
 * provide reliable tuning for PID-type controllers based on process models
 * or closed-loop experiments.
 * 
 * ## Implemented Methods
 * 
 * ### Ziegler-Nichols Methods
 * - **Step Response (Open-Loop)**: Uses tangent line method
 * - **Ultimate Cycle (Closed-Loop)**: Uses critical gain and period
 * 
 * ### Optimized Methods
 * - **Cohen-Coon**: Better for processes with significant dead time
 * - **ITAE, IAE, ISE**: Minimize integral error criteria
 * - **Tyreus-Luyben**: Conservative tuning for chemical processes
 * 
 * ### IMC-Based Methods
 * - **Lambda Tuning**: Specify closed-loop time constant
 * - **SIMC (Skogestad)**: Simple rules with tuning parameter
 * - **AMIGO**: Robustness-optimized tuning
 * 
 * ### Relay Feedback
 * - **Åström-Hägglund**: Auto-identification of critical point
 * 
 * @author ESP32EtherCAT Project
 * @version 2.0
 */

#pragma once

// Aggregator header for classical tuning methods.
// Individual controllers have been split into per-class headers under
// include/tether/control/autotuning/classical/. This header remains as a
// compatibility shim that includes the common types and all classical tuner
// declarations.

#include "tether/control/autotuning/classical/Common.hpp"
#include "tether/control/autotuning/classical/ZieglerNicholsStepResponse.hpp"
#include "tether/control/autotuning/classical/ZieglerNicholsUltimateCycle.hpp"
#include "tether/control/autotuning/classical/TyreusLuyben.hpp"
#include "tether/control/autotuning/classical/CohenCoon.hpp"
#include "tether/control/autotuning/classical/ChienHronesReswick.hpp"
#include "tether/control/autotuning/classical/AstromHagglundRelay.hpp"
#include "tether/control/autotuning/classical/LopezMethod.hpp"
#include "tether/control/autotuning/classical/LambdaTuning.hpp"
#include "tether/control/autotuning/classical/SIMCMethod.hpp"
#include "tether/control/autotuning/classical/AMIGOMethod.hpp"

#include "tether/control/autotuning/AutotuningFramework.hpp"

#include <memory>
#include <vector>

namespace Control {
namespace Autotuning {

class ClassicalTuningFactory {
public:
    enum class Method {
        ZieglerNicholsStep,
        ZieglerNicholsUltimate,
        TyreusLuyben,
        CohenCoon,
        CHR_SetpointNoOS,
        CHR_Setpoint20OS,
        CHR_RegulatorNoOS,
        CHR_Regulator20OS,
        LopezITAE,
        LopezIAE,
        LopezISE,
        Lambda,
        SIMC,
        AMIGO,
        RelayFeedback
    };

    static std::unique_ptr<AutotunerBase> create(Method method);
    static std::vector<Method> getAvailableMethods();
    static std::string getMethodName(Method method);
};

} // namespace Autotuning
} // namespace Control



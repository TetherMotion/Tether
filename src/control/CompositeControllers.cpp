/**
 * @file CompositeControllers.cpp
 * @brief Implementation of Composite Controllers
 */

#include "control/CompositeControllers.hpp"
#include "control/PIDControllers.hpp"
#include "control/FractionalPID.hpp"
#include "control/StateSpaceControllers.hpp"
#include "control/RobustControllers.hpp"
#include "control/LearningControllers.hpp"
#include <cmath>
#include <algorithm>
#include <cstring>

namespace tether::control {

// ============================================================================
// Cascade Controller Implementation
// ============================================================================

void CascadeController::setInnerReferenceLimits(double min, double max) {
    m_innerRefMin = min;
    m_innerRefMax = max;
}

ControllerOutput CascadeController::computeImpl(const ControllerInput& input) {
    ControllerOutput output;
    
    if (!m_outer || !m_inner) {
        output.error = input.reference - input.measured;
        output.control = 0.0;
        return output;
    }
    
    // Outer loop: generates reference for inner loop
    auto outerOutput = m_outer->compute(input);
    
    // Limit inner reference
    double innerRef = std::clamp(outerOutput.control, m_innerRefMin, m_innerRefMax);
    
    // Inner loop: uses outer output as reference, state[0] as measurement
    ControllerInput innerInput = input;
    innerInput.reference = innerRef;
    innerInput.measured = input.state[0];  // Inner measurement from state vector
    
    auto innerOutput = m_inner->compute(innerInput);
    
    // Output is from inner loop
    output = innerOutput;
    output.error = input.reference - input.measured;  // Outer error for diagnostics
    
    return output;
}

void CascadeController::resetImpl() {
    if (m_outer) m_outer->reset();
    if (m_inner) m_inner->reset();
}

// ============================================================================
// Feedforward Controller Implementation
// ============================================================================

ControllerOutput FeedforwardController::computeImpl(const ControllerInput& input) {
    ControllerOutput output;
    output.error = input.reference - input.measured;
    
    // Calculate feedforward
    double ff = 0.0;
    
    if (m_ffFunc) {
        ff = m_ffFunc(input);
    } else if (m_useSimpleFF) {
        ff = m_ffGain * input.reference;
        ff += m_velFFGain * input.referenceDerivative;
        ff += m_accelFFGain * input.referenceAccel;
    }
    
    // Calculate feedback
    double fb = 0.0;
    if (m_feedback) {
        auto fbOutput = m_feedback->compute(input);
        fb = fbOutput.control;
    }
    
    // Combine based on mixing ratio
    output.control = m_mixRatio * ff + (1.0 - m_mixRatio) * fb;
    output.feedforward = ff;
    
    return output;
}

void FeedforwardController::resetImpl() {
    if (m_feedback) m_feedback->reset();
}

// ============================================================================
// Parallel Controller Implementation
// ============================================================================

void ParallelController::addController(ControllerBase* controller, double weight) {
    m_controllers.push_back({controller, weight});
}

void ParallelController::clearControllers() {
    m_controllers.clear();
}

void ParallelController::setWeight(size_t index, double weight) {
    if (index < m_controllers.size()) {
        m_controllers[index].weight = weight;
    }
}

ControllerOutput ParallelController::computeImpl(const ControllerInput& input) {
    ControllerOutput output;
    output.error = input.reference - input.measured;
    output.control = 0.0;
    output.feedforward = 0.0;
    
    for (const auto& entry : m_controllers) {
        if (entry.controller) {
            auto ctrlOutput = entry.controller->compute(input);
            output.control += entry.weight * ctrlOutput.control;
            output.feedforward += entry.weight * ctrlOutput.feedforward;
        }
    }
    
    return output;
}

void ParallelController::resetImpl() {
    for (auto& entry : m_controllers) {
        if (entry.controller) {
            entry.controller->reset();
        }
    }
}

// ============================================================================
// Switching Controller Implementation
// ============================================================================

void SwitchingController::addController(ControllerBase* controller, 
                                         SelectionFunction condition, 
                                         int priority) {
    m_controllers.push_back({controller, condition, priority});
    
    // Sort by priority (highest first)
    std::sort(m_controllers.begin(), m_controllers.end(),
              [](const ControllerEntry& a, const ControllerEntry& b) {
                  return a.priority > b.priority;
              });
}

ControllerOutput SwitchingController::computeImpl(const ControllerInput& input) {
    ControllerOutput output;
    output.error = input.reference - input.measured;
    
    // Find active controller
    m_prevActiveIndex = m_activeIndex;
    m_activeIndex = -1;
    ControllerBase* activeController = nullptr;
    
    for (size_t i = 0; i < m_controllers.size(); ++i) {
        if (m_controllers[i].condition(input)) {
            m_activeIndex = static_cast<int>(i);
            activeController = m_controllers[i].controller;
            break;
        }
    }
    
    // Use default if no condition matched
    if (!activeController && m_default) {
        activeController = m_default;
        m_activeIndex = static_cast<int>(m_controllers.size());
    }
    
    if (!activeController) {
        output.control = 0.0;
        return output;
    }
    
    // Compute output
    auto ctrlOutput = activeController->compute(input);
    output = ctrlOutput;
    
    // Bumpless transfer
    if (m_bumpless && m_activeIndex != m_prevActiveIndex && m_prevActiveIndex >= 0) {
        // Blend from previous output to new output
        if (m_bumplessTau > 0 && input.dt > 0) {
            double alpha = 1.0 - std::exp(-input.dt / m_bumplessTau);
            m_blendFactor = std::min(1.0, m_blendFactor + alpha);
            output.control = m_blendFactor * ctrlOutput.control + 
                            (1.0 - m_blendFactor) * m_prevOutput;
        }
    } else {
        m_blendFactor = 1.0;
    }
    
    m_prevOutput = output.control;
    
    return output;
}

void SwitchingController::resetImpl() {
    for (auto& entry : m_controllers) {
        if (entry.controller) {
            entry.controller->reset();
        }
    }
    if (m_default) {
        m_default->reset();
    }
    m_activeIndex = -1;
    m_prevActiveIndex = -1;
    m_blendFactor = 1.0;
    m_prevOutput = 0.0;
}

// ============================================================================
// Rate Limiter Wrapper Implementation
// ============================================================================

void RateLimiterWrapper::setRateLimits(double minRate, double maxRate) {
    m_minRate = minRate;
    m_maxRate = maxRate;
}

ControllerOutput RateLimiterWrapper::computeImpl(const ControllerInput& input) {
    ControllerOutput output;
    
    if (!m_inner) {
        output.error = input.reference - input.measured;
        output.control = 0.0;
        return output;
    }
    
    auto innerOutput = m_inner->compute(input);
    output = innerOutput;
    
    if (m_firstSample) {
        m_prevOutput = innerOutput.control;
        m_firstSample = false;
    } else if (input.dt > 0) {
        double rate = (innerOutput.control - m_prevOutput) / input.dt;
        rate = std::clamp(rate, m_minRate, m_maxRate);
        output.control = m_prevOutput + rate * input.dt;
    }
    
    m_prevOutput = output.control;
    
    return output;
}

void RateLimiterWrapper::resetImpl() {
    if (m_inner) m_inner->reset();
    m_prevOutput = 0.0;
    m_firstSample = true;
}

// ============================================================================
// Deadband Wrapper Implementation
// ============================================================================

ControllerOutput DeadbandWrapper::computeImpl(const ControllerInput& input) {
    ControllerOutput output;
    
    // Apply error deadband
    ControllerInput modifiedInput = input;
    double error = input.reference - input.measured;
    
    if (std::fabs(error) < m_errorDeadband) {
        modifiedInput.reference = input.measured;  // Zero error
    }
    
    if (!m_inner) {
        output.error = error;
        output.control = 0.0;
        return output;
    }
    
    auto innerOutput = m_inner->compute(modifiedInput);
    output = innerOutput;
    output.error = error;  // Report original error
    
    // Apply output deadband with hysteresis
    if (std::fabs(output.control) < m_outputDeadband) {
        // Check hysteresis condition
        bool shouldZero = true;
        if (m_hysteresis > 0) {
            // Only zero if we're within hysteresis band
            if (m_prevOutput != 0) {
                double threshold = m_outputDeadband + m_hysteresis;
                shouldZero = std::fabs(output.control) < threshold;
            }
        }
        
        if (shouldZero) {
            output.control = 0.0;
        }
    }
    
    m_prevOutput = output.control;
    
    return output;
}

void DeadbandWrapper::resetImpl() {
    if (m_inner) m_inner->reset();
    m_prevOutput = 0.0;
}

// ============================================================================
// Filter Wrapper Implementation
// ============================================================================

ControllerOutput FilterWrapper::computeImpl(const ControllerInput& input) {
    ControllerOutput output;
    
    if (!m_inner) {
        output.error = input.reference - input.measured;
        output.control = 0.0;
        return output;
    }
    
    auto innerOutput = m_inner->compute(input);
    output = innerOutput;
    
    // Apply low-pass filter
    if (input.dt > 0 && m_cutoff > 0) {
        double omega = 2.0 * M_PI * m_cutoff;
        double alpha = omega * input.dt / (1.0 + omega * input.dt);
        
        if (m_order == 1) {
            m_state1 = alpha * innerOutput.control + (1.0 - alpha) * m_state1;
            output.control = m_state1;
        } else {
            // Second-order (two cascaded first-order)
            m_state1 = alpha * innerOutput.control + (1.0 - alpha) * m_state1;
            m_state2 = alpha * m_state1 + (1.0 - alpha) * m_state2;
            output.control = m_state2;
        }
    }
    
    return output;
}

void FilterWrapper::resetImpl() {
    if (m_inner) m_inner->reset();
    m_state1 = 0.0;
    m_state2 = 0.0;
}

// ============================================================================
// Controller Factory Implementation
// ============================================================================

std::unique_ptr<ControllerBase> ControllerFactory::create(ControllerType type) {
    switch (type) {
        case ControllerType::P:
            return std::make_unique<PController>();
        case ControllerType::PD:
            return std::make_unique<PDController>();
        case ControllerType::PI:
            return std::make_unique<PIController>();
        case ControllerType::PID:
            return std::make_unique<PIDController>();
        case ControllerType::PID2DOF:
            return std::make_unique<PID2DOFController>();
        case ControllerType::BangBang:
            return std::make_unique<BangBangController>();
        case ControllerType::PDPlus:
            return std::make_unique<PDPlusController>();
        case ControllerType::FractionalPID:
            return std::make_unique<FractionalPIDController>();
        case ControllerType::LQR:
            return std::make_unique<LQRController>();
        case ControllerType::LQG:
            return std::make_unique<LQGController>();
        case ControllerType::LQI:
            return std::make_unique<LQIController>();
        case ControllerType::HInfinity:
            return std::make_unique<HInfinityController>();
        case ControllerType::ILC:
            return std::make_unique<PTypeILC>();
        case ControllerType::Cascade:
            return std::make_unique<CascadeController>();
        default:
            return nullptr;
    }
}

std::unique_ptr<ControllerBase> ControllerFactory::create(const char* name) {
    if (!name) return nullptr;
    
    if (std::strcmp(name, "P") == 0) return std::make_unique<PController>();
    if (std::strcmp(name, "PD") == 0) return std::make_unique<PDController>();
    if (std::strcmp(name, "PI") == 0) return std::make_unique<PIController>();
    if (std::strcmp(name, "PID") == 0) return std::make_unique<PIDController>();
    if (std::strcmp(name, "PID-2DOF") == 0) return std::make_unique<PID2DOFController>();
    if (std::strcmp(name, "Bang-Bang") == 0) return std::make_unique<BangBangController>();
    if (std::strcmp(name, "PD+") == 0) return std::make_unique<PDPlusController>();
    if (std::strcmp(name, "Dual-Loop PID") == 0) return std::make_unique<DualLoopPIDController>();
    if (std::strcmp(name, "FOPID") == 0) return std::make_unique<FractionalPIDController>();
    if (std::strcmp(name, "LQR") == 0) return std::make_unique<LQRController>();
    if (std::strcmp(name, "LQG") == 0) return std::make_unique<LQGController>();
    if (std::strcmp(name, "LQI") == 0) return std::make_unique<LQIController>();
    if (std::strcmp(name, "H2") == 0) return std::make_unique<H2Controller>();
    if (std::strcmp(name, "H-Infinity") == 0) return std::make_unique<HInfinityController>();
    if (std::strcmp(name, "P-Type ILC") == 0) return std::make_unique<PTypeILC>();
    if (std::strcmp(name, "PD-Type ILC") == 0) return std::make_unique<PDTypeILC>();
    if (std::strcmp(name, "Phase-Lead ILC") == 0) return std::make_unique<PhaseLeadILC>();
    if (std::strcmp(name, "Norm-Optimal ILC") == 0) return std::make_unique<NormOptimalILC>();
    if (std::strcmp(name, "Repetitive") == 0) return std::make_unique<RepetitiveController>();
    if (std::strcmp(name, "Cascade") == 0) return std::make_unique<CascadeController>();
    if (std::strcmp(name, "Parallel") == 0) return std::make_unique<ParallelController>();
    if (std::strcmp(name, "Switching") == 0) return std::make_unique<SwitchingController>();
    
    return nullptr;
}

} // namespace tether::control

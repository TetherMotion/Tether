/**
 * @file ControllerBase.cpp
 * @brief Implementation of base controller functionality
 */

#include "control/ControllerBase.hpp"
#include "tether/platform/EspCompat.hpp"

namespace Control {

// ============================================================================
// ControllerBase Implementation
// ============================================================================

ControllerOutput ControllerBase::compute(const ControllerInput& input) {
    int64_t startTime = esp_timer_get_time();
    
    ControllerOutput output;
    
    // Handle reset request
    if (input.reset) {
        reset();
    }
    
    // Handle different modes
    switch (m_mode) {
        case ControllerMode::Disabled:
            output.control = 0.0;
            break;
            
        case ControllerMode::Manual:
            output.control = m_manualOutput;
            break;
            
        case ControllerMode::Hold:
            output.control = m_lastOutputValue;
            break;
            
        case ControllerMode::Automatic:
        case ControllerMode::Tracking:
        default:
            if (input.enable) {
                output = computeImpl(input);
            }
            break;
    }
    
    // Apply saturation
    double unsaturated = output.control;
    output.control = saturate(output.control);
    output.saturated = (output.control != unsaturated);
    
    // Apply rate limiting
    output.control = rateLimit(output.control, input.dt);
    
    // Store for next cycle
    m_lastOutputValue = output.control;
    m_lastOutput = output;
    
    // Update diagnostics
    updateDiagnostics(input, output);
    
    // Record compute time
    m_diagnostics.computeTimeUs = static_cast<double>(
        esp_timer_get_time() - startTime);
    
    return output;
}

void ControllerBase::reset() {
    m_lastOutputValue = 0.0;
    m_lastOutput = ControllerOutput{};
    resetDiagnostics();  // Also reset diagnostics
    resetImpl();
}

void ControllerBase::resetDiagnostics() {
    m_diagnostics = ControllerDiagnostics{};
}

double ControllerBase::saturate(double value) const {
    return clamp(value, m_limits.outputMin, m_limits.outputMax);
}

double ControllerBase::rateLimit(double value, double dt) {
    if (m_limits.rateLimit >= std::numeric_limits<double>::max()) {
        return value;
    }
    
    double maxChange = m_limits.rateLimit * dt;
    double change = value - m_lastOutputValue;
    
    if (std::abs(change) > maxChange) {
        return m_lastOutputValue + sign(change) * maxChange;
    }
    
    return value;
}

void ControllerBase::updateDiagnostics(const ControllerInput& input,
                                       const ControllerOutput& output) {
    m_diagnostics.cycleCount++;
    
    double absError = std::abs(output.error);
    if (absError > m_diagnostics.maxError) {
        m_diagnostics.maxError = absError;
    }
    
    // Exponential moving average for RMS error
    double alpha = 0.01;
    m_diagnostics.rmsError = std::sqrt(
        alpha * output.error * output.error + 
        (1.0 - alpha) * m_diagnostics.rmsError * m_diagnostics.rmsError);
    
    m_diagnostics.integralValue = output.integral;
    m_diagnostics.lastDerivative = output.derivative;
    
    if (output.saturated) {
        m_diagnostics.saturationCount++;
    }
}

} // namespace Control

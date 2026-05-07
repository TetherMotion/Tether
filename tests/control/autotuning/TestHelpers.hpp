/**
 * @file TestHelpers.hpp
 * @brief Common test utilities and mock controllers for autotuning tests
 */

#pragma once

#include <gtest/gtest.h>
#include <memory>
#include <vector>
#include <cmath>
#include <complex>
#include <algorithm>
#include <numeric>
#include "tether/control/autotuning/AutotuningFramework.hpp"

// Use existing types from AutotuningFramework
using Control::Autotuning::FOPDTModel;
using Control::Autotuning::ProcessModel;
using Control::Autotuning::ProcessModelType;
using Control::Autotuning::TunableController;
using Control::Autotuning::CostFunction;
using Control::Autotuning::ParameterDescriptors;
using Control::Autotuning::ParameterVector;
using Control::Autotuning::PIDForm;

namespace Control {
namespace Autotuning {
namespace Testing {

/**
 * @brief Concrete test controller implementing all TunableController requirements
 * 
 * This is a simple PID controller for testing purposes.
 */
class TestPIDController : public TunableController {
public:
    TestPIDController() : m_Kp(1.0), m_Ki(0.1), m_Kd(0.01) {}
    
    TestPIDController(double Kp, double Ki, double Kd) 
        : m_Kp(Kp), m_Ki(Ki), m_Kd(Kd) {}
    
    ParameterDescriptors getParameterDescriptors() const override {
        return {
            {"Kp", m_Kp, {0.001, 100.0}, 1.0, true},
            {"Ki", m_Ki, {0.0, 50.0}, 1.0, false},
            {"Kd", m_Kd, {0.0, 10.0}, 1.0, false}
        };
    }
    
    ParameterVector getParameters() const override {
        return {m_Kp, m_Ki, m_Kd};
    }
    
    bool setParameters(const ParameterVector& params) override {
        if (params.size() < 3) return false;
        if (params[0] <= 0) return false;
        m_Kp = params[0];
        m_Ki = params[1];
        m_Kd = params[2];
        return true;
    }
    
    std::string getControllerTypeName() const override {
        return "PID";
    }
    
    std::vector<double> evaluate(
        const std::vector<double>& input,
        const std::vector<double>& reference,
        double dt) override {
        
        reset();
        std::vector<double> output;
        output.reserve(input.size());
        
        for (size_t i = 0; i < input.size(); ++i) {
            double error = (i < reference.size()) ? (reference[i] - input[i]) : 0.0;
            output.push_back(compute(error, dt));
        }
        
        return output;
    }
    
    void reset() override {
        m_integral = 0.0;
        m_prevError = 0.0;
        m_prevDeriv = 0.0;
    }
    
    std::shared_ptr<TunableController> clone() const override {
        auto copy = std::make_shared<TestPIDController>(m_Kp, m_Ki, m_Kd);
        return copy;
    }
    
    double compute(double error, double dt) {
        m_integral += error * dt;
        
        // Filtered derivative
        double deriv = (error - m_prevError) / (dt + 1e-10);
        double alpha = 0.1;
        deriv = alpha * deriv + (1.0 - alpha) * m_prevDeriv;
        m_prevDeriv = deriv;
        m_prevError = error;
        
        return m_Kp * error + m_Ki * m_integral + m_Kd * deriv;
    }
    
    /**
     * @brief Compute control output for setpoint tracking
     * @param setpoint Desired value
     * @param measurement Current process output
     * @param dt Time step
     * @return Control signal
     */
    double computeSetpoint(double setpoint, double measurement, double dt = 0.01) {
        double error = setpoint - measurement;
        return compute(error, dt);
    }
    
    // Getters for gains
    double getKp() const { return m_Kp; }
    double getKi() const { return m_Ki; }
    double getKd() const { return m_Kd; }
    
private:
    double m_Kp, m_Ki, m_Kd;
    double m_integral = 0.0;
    double m_prevError = 0.0;
    double m_prevDeriv = 0.0;
};

/**
 * @brief Concrete ProcessModel implementation for testing using FOPDT
 * 
 * This class provides both a process model interface and a simulatable
 * process that can be stepped in time for closed-loop testing.
 */
class TestFOPDTProcessModel : public ProcessModel {
public:
    TestFOPDTProcessModel(double K = 1.0, double tau = 10.0, double theta = 1.0) 
        : m_output(0.0), m_disturbance(0.0), m_time(0.0) {
        m_model.K = K;
        m_model.tau = tau;
        m_model.L = theta;
        m_delayBuffer.resize(static_cast<size_t>(theta / 0.001) + 1, 0.0);  // 1ms resolution
        m_delayIndex = 0;
    }
    
    ProcessModelType getType() const override { return ProcessModelType::FOPDT; }
    
    std::complex<double> evaluate(double omega) const override {
        // Use the FOPDTModel's evaluate method
        return m_model.evaluate(omega);
    }
    
    std::vector<std::pair<double, double>> stepResponse(
        double stepMagnitude, double dt, double duration) const override {
        
        std::vector<std::pair<double, double>> result;
        int steps = static_cast<int>(duration / dt);
        result.reserve(steps);
        
        double y = 0.0;
        for (int i = 0; i < steps; ++i) {
            double t = i * dt;
            // FOPDT step response: K * (1 - exp(-(t-L)/tau)) for t > L
            if (t > m_model.L) {
                y = m_model.K * stepMagnitude * (1.0 - std::exp(-(t - m_model.L) / m_model.tau));
            }
            result.push_back(std::make_pair(t, y));
        }
        return result;
    }
    
    FOPDTModel toFOPDT() const override {
        return m_model;
    }
    
    std::pair<double, double> getUltimateParams() const override {
        // Estimate Ku and Tu for FOPDT using Ziegler-Nichols approximation
        double Ku = m_model.tau / (m_model.K * m_model.L);
        double Tu = 4.0 * m_model.L;
        
        return {Ku, Tu};
    }
    
    const FOPDTModel& getInternalModel() const { return m_model; }
    
    /**
     * @brief Step the process simulation forward in time
     * @param input Control input signal
     * @param dt Time step
     */
    void step(double input, double dt) {
        // Add input to delay buffer
        if (!m_delayBuffer.empty()) {
            m_delayBuffer[m_delayIndex] = input;
            m_delayIndex = (m_delayIndex + 1) % m_delayBuffer.size();
        }
        
        // Get delayed input
        double delayedInput = input;
        if (!m_delayBuffer.empty() && m_model.L > 0) {
            size_t delaySteps = static_cast<size_t>(m_model.L / dt);
            if (delaySteps < m_delayBuffer.size()) {
                size_t idx = (m_delayIndex + m_delayBuffer.size() - delaySteps) % m_delayBuffer.size();
                delayedInput = m_delayBuffer[idx];
            }
        }
        
        // First-order dynamics: dy/dt = (K*u - y) / tau
        if (m_model.tau > 0) {
            double dOutput = (m_model.K * delayedInput - m_output) / m_model.tau;
            m_output += dOutput * dt;
        } else {
            m_output = m_model.K * delayedInput;
        }
        
        m_time += dt;
    }
    
    /**
     * @brief Get current process output
     * @return Current output value including disturbance
     */
    double getOutput() const {
        return m_output + m_disturbance;
    }
    
    /**
     * @brief Add a disturbance to the process
     * @param disturbance Disturbance magnitude
     */
    void addDisturbance(double disturbance) {
        m_disturbance = disturbance;
    }
    
    /**
     * @brief Reset the process to initial state
     */
    void reset() {
        m_output = 0.0;
        m_disturbance = 0.0;
        m_time = 0.0;
        std::fill(m_delayBuffer.begin(), m_delayBuffer.end(), 0.0);
        m_delayIndex = 0;
    }
    
    /**
     * @brief Get current simulation time
     */
    double getTime() const { return m_time; }
    
private:
    FOPDTModel m_model;
    double m_output;
    double m_disturbance;
    double m_time;
    std::vector<double> m_delayBuffer;
    size_t m_delayIndex;
};

/**
 * @brief Simple PID controller for closed-loop simulation tests
 * 
 * This is a standalone PID controller that supports the compute(setpoint, measurement) API
 * commonly used in closed-loop simulations. It does not inherit from TunableController.
 */
class SimplePIDController {
public:
    SimplePIDController() : m_Kp(1.0), m_Ki(0.1), m_Kd(0.01), 
                           m_integral(0.0), m_prevError(0.0), m_prevDeriv(0.0) {}
    
    SimplePIDController(double Kp, double Ki, double Kd) 
        : m_Kp(Kp), m_Ki(Ki), m_Kd(Kd),
          m_integral(0.0), m_prevError(0.0), m_prevDeriv(0.0) {}
    
    /**
     * @brief Compute control output for setpoint tracking
     * @param setpoint Desired value
     * @param measurement Current process output
     * @param dt Time step (default 0.01)
     * @return Control signal
     */
    double compute(double setpoint, double measurement, double dt = 0.01) {
        double error = setpoint - measurement;
        
        m_integral += error * dt;
        
        // Filtered derivative
        double deriv = (error - m_prevError) / (dt + 1e-10);
        double alpha = 0.1;
        deriv = alpha * deriv + (1.0 - alpha) * m_prevDeriv;
        m_prevDeriv = deriv;
        m_prevError = error;
        
        return m_Kp * error + m_Ki * m_integral + m_Kd * deriv;
    }
    
    void reset() {
        m_integral = 0.0;
        m_prevError = 0.0;
        m_prevDeriv = 0.0;
    }
    
    double getKp() const { return m_Kp; }
    double getKi() const { return m_Ki; }
    double getKd() const { return m_Kd; }
    
private:
    double m_Kp, m_Ki, m_Kd;
    double m_integral;
    double m_prevError;
    double m_prevDeriv;
};

/**
 * @brief Simple quadratic cost function for testing
 */
class SimpleQuadraticCost : public CostFunction {
public:
    double evaluate(const ParameterVector& params) override {
        double sum = 0.0;
        for (double p : params) {
            sum += (p - 1.0) * (p - 1.0);  // Minimum at params = [1, 1, ...]
        }
        return sum;
    }
};

// Tolerance constants
constexpr double TOLERANCE = 1e-6;
constexpr double DT = 0.01;

} // namespace Testing
} // namespace Autotuning
} // namespace Control

// Create namespace alias for tests using the new naming convention
namespace tether {
namespace control {
namespace autotuning {
namespace test {

// Re-export test utilities - use SimplePIDController as TestPIDController for simulation tests
using TestPIDController = Control::Autotuning::Testing::SimplePIDController;
using TestFOPDTProcessModel = Control::Autotuning::Testing::TestFOPDTProcessModel;
using SimpleQuadraticCost = Control::Autotuning::Testing::SimpleQuadraticCost;

// Also export the original TunableController-based version
using TunablePIDController = Control::Autotuning::Testing::TestPIDController;

constexpr double TOLERANCE = Control::Autotuning::Testing::TOLERANCE;
constexpr double DT = Control::Autotuning::Testing::DT;

} // namespace test
} // namespace autotuning
} // namespace control
} // namespace tether

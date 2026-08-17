/**
 * @file AutotuningFrameworkTests.cpp
 * @brief Unit tests for the Autotuning Framework core types
 */

#include <gtest/gtest.h>
#include <cmath>
#include <vector>
#include <complex>

#include "tether/control/autotuning/AutotuningFramework.hpp"
#include "TestHelpers.hpp"

using namespace tether::control::Autotuning;
using namespace tether::control::Autotuning::Testing;

// ============================================================================
// FOPDTModel Tests
// ============================================================================

TEST(FOPDTModelTest, DefaultConstruction) {
    FOPDTModel model;
    EXPECT_DOUBLE_EQ(model.K, 1.0);
    EXPECT_DOUBLE_EQ(model.tau, 1.0);
    EXPECT_DOUBLE_EQ(model.L, 0.0);
}

TEST(FOPDTModelTest, FieldAccess) {
    FOPDTModel model;
    model.K = 2.0;
    model.tau = 5.0;
    model.L = 1.0;
    
    EXPECT_DOUBLE_EQ(model.K, 2.0);
    EXPECT_DOUBLE_EQ(model.tau, 5.0);
    EXPECT_DOUBLE_EQ(model.L, 1.0);
}

TEST(FOPDTModelTest, NormalizedDeadTime) {
    FOPDTModel model;
    model.K = 1.0;
    model.tau = 10.0;
    model.L = 2.0;
    
    EXPECT_DOUBLE_EQ(model.normalizedDeadTime(), 0.2);
}

TEST(FOPDTModelTest, IsValidWithPositiveParams) {
    FOPDTModel model;
    model.K = 1.0;
    model.tau = 1.0;
    model.L = 0.5;
    
    EXPECT_TRUE(model.isValid());
}

TEST(FOPDTModelTest, IsValidWithZeroDelay) {
    FOPDTModel model;
    model.K = 1.0;
    model.tau = 1.0;
    model.L = 0.0;
    
    EXPECT_TRUE(model.isValid());
}

TEST(FOPDTModelTest, IsValidWithNegativeDelay) {
    FOPDTModel model;
    model.K = 1.0;
    model.tau = 1.0;
    model.L = -1.0;
    
    EXPECT_FALSE(model.isValid());
}

TEST(FOPDTModelTest, IsValidWithZeroTau) {
    FOPDTModel model;
    model.K = 1.0;
    model.tau = 0.0;
    model.L = 1.0;
    
    EXPECT_FALSE(model.isValid());
}

TEST(FOPDTModelTest, EvaluateAtZeroFrequency) {
    FOPDTModel model;
    model.K = 2.0;
    model.tau = 5.0;
    model.L = 0.0;
    
    auto response = model.evaluate(0.0);
    EXPECT_NEAR(std::abs(response), 2.0, 1e-6);
}

TEST(FOPDTModelTest, EvaluateAtHighFrequency) {
    FOPDTModel model;
    model.K = 2.0;
    model.tau = 5.0;
    model.L = 0.0;
    
    auto response = model.evaluate(100.0);
    EXPECT_LT(std::abs(response), 0.1);  // Should attenuate at high freq
}

// ============================================================================
// SOPDTModel Tests
// ============================================================================

TEST(SOPDTModelTest, DefaultConstruction) {
    SOPDTModel model;
    EXPECT_DOUBLE_EQ(model.K, 1.0);
    EXPECT_DOUBLE_EQ(model.tau1, 1.0);
    EXPECT_DOUBLE_EQ(model.tau2, 0.5);
    EXPECT_DOUBLE_EQ(model.L, 0.0);
}

TEST(SOPDTModelTest, IsValid) {
    SOPDTModel model;
    model.tau1 = 2.0;
    model.tau2 = 1.0;
    model.L = 0.5;
    
    EXPECT_TRUE(model.isValid());
}

// ============================================================================
// IPDTModel Tests
// ============================================================================

TEST(IPDTModelTest, DefaultConstruction) {
    IPDTModel model;
    EXPECT_DOUBLE_EQ(model.K, 1.0);
    EXPECT_DOUBLE_EQ(model.L, 0.0);
}

TEST(IPDTModelTest, IsValid) {
    IPDTModel model;
    model.K = 1.0;
    model.L = 0.5;
    
    EXPECT_TRUE(model.isValid());
}

// ============================================================================
// ParameterBounds Tests
// ============================================================================

TEST(ParameterBoundsTest, Contains) {
    ParameterBounds bounds{0.0, 10.0};
    
    EXPECT_TRUE(bounds.contains(5.0));
    EXPECT_TRUE(bounds.contains(0.0));
    EXPECT_TRUE(bounds.contains(10.0));
    EXPECT_FALSE(bounds.contains(-1.0));
    EXPECT_FALSE(bounds.contains(11.0));
}

TEST(ParameterBoundsTest, Clamp) {
    ParameterBounds bounds{0.0, 10.0};
    
    EXPECT_DOUBLE_EQ(bounds.clamp(5.0), 5.0);
    EXPECT_DOUBLE_EQ(bounds.clamp(-5.0), 0.0);
    EXPECT_DOUBLE_EQ(bounds.clamp(15.0), 10.0);
}

TEST(ParameterBoundsTest, Range) {
    ParameterBounds bounds{-5.0, 5.0};
    EXPECT_DOUBLE_EQ(bounds.range(), 10.0);
}

TEST(ParameterBoundsTest, Center) {
    ParameterBounds bounds{0.0, 10.0};
    EXPECT_DOUBLE_EQ(bounds.center(), 5.0);
}

// ============================================================================
// ParameterDescriptor Tests
// ============================================================================

TEST(ParameterDescriptorTest, Construction) {
    ParameterDescriptor desc;
    desc.name = "Kp";
    desc.initialValue = 1.0;
    desc.bounds = {0.001, 100.0};
    desc.scale = 1.0;
    desc.logarithmic = true;
    
    EXPECT_EQ(desc.name, "Kp");
    EXPECT_DOUBLE_EQ(desc.initialValue, 1.0);
    EXPECT_TRUE(desc.logarithmic);
}

TEST(ParameterDescriptorTest, LinearNormalize) {
    ParameterDescriptor desc;
    desc.bounds = {0.0, 10.0};
    desc.logarithmic = false;
    
    EXPECT_NEAR(desc.normalize(0.0), 0.0, 1e-6);
    EXPECT_NEAR(desc.normalize(5.0), 0.5, 1e-6);
    EXPECT_NEAR(desc.normalize(10.0), 1.0, 1e-6);
}

TEST(ParameterDescriptorTest, LinearDenormalize) {
    ParameterDescriptor desc;
    desc.bounds = {0.0, 10.0};
    desc.logarithmic = false;
    
    EXPECT_NEAR(desc.denormalize(0.0), 0.0, 1e-6);
    EXPECT_NEAR(desc.denormalize(0.5), 5.0, 1e-6);
    EXPECT_NEAR(desc.denormalize(1.0), 10.0, 1e-6);
}

TEST(ParameterDescriptorTest, LogNormalize) {
    ParameterDescriptor desc;
    desc.bounds = {0.1, 100.0};
    desc.logarithmic = true;
    
    double normalized = desc.normalize(1.0);
    EXPECT_GT(normalized, 0.0);
    EXPECT_LT(normalized, 1.0);
}

// ============================================================================
// TuningResult Tests
// ============================================================================

TEST(TuningResultTest, DefaultConstruction) {
    TuningResult result;
    
    EXPECT_TRUE(result.parameters.empty());
    EXPECT_FALSE(result.success);
    EXPECT_EQ(result.iterations, 0);
    EXPECT_EQ(result.functionEvaluations, 0);
}

TEST(TuningResultTest, SetValues) {
    TuningResult result;
    result.parameters = {1.0, 0.1, 0.01};
    result.cost = 0.05;
    result.success = true;
    result.message = "Converged";
    result.iterations = 100;
    result.functionEvaluations = 500;
    
    EXPECT_EQ(result.parameters.size(), 3);
    EXPECT_DOUBLE_EQ(result.cost, 0.05);
    EXPECT_TRUE(result.success);
    EXPECT_EQ(result.iterations, 100);
}

TEST(TuningResultTest, PerformanceMetrics) {
    TuningResult result;
    result.settlingTime = 5.0;
    result.overshoot = 0.1;
    result.riseTime = 1.0;
    result.steadyStateError = 0.01;
    result.gainMargin = 10.0;
    result.phaseMargin = 45.0;
    
    EXPECT_DOUBLE_EQ(result.settlingTime, 5.0);
    EXPECT_DOUBLE_EQ(result.phaseMargin, 45.0);
}

// ============================================================================
// TestPIDController Tests
// ============================================================================

TEST(TestPIDControllerTest, DefaultConstruction) {
    TestPIDController ctrl;
    
    auto params = ctrl.getParameters();
    EXPECT_EQ(params.size(), 3);
}

TEST(TestPIDControllerTest, ConstructionWithParams) {
    TestPIDController ctrl(2.0, 0.5, 0.1);
    
    auto params = ctrl.getParameters();
    EXPECT_DOUBLE_EQ(params[0], 2.0);
    EXPECT_DOUBLE_EQ(params[1], 0.5);
    EXPECT_DOUBLE_EQ(params[2], 0.1);
}

TEST(TestPIDControllerTest, SetParameters) {
    TestPIDController ctrl;
    EXPECT_TRUE(ctrl.setParameters({3.0, 0.3, 0.03}));
    
    auto params = ctrl.getParameters();
    EXPECT_DOUBLE_EQ(params[0], 3.0);
    EXPECT_DOUBLE_EQ(params[1], 0.3);
    EXPECT_DOUBLE_EQ(params[2], 0.03);
}

TEST(TestPIDControllerTest, SetParametersRejectsInvalid) {
    TestPIDController ctrl;
    EXPECT_FALSE(ctrl.setParameters({1.0, 0.1}));  // Too few
    EXPECT_FALSE(ctrl.setParameters({-1.0, 0.1, 0.01}));  // Negative Kp
}

TEST(TestPIDControllerTest, GetParameterDescriptors) {
    TestPIDController ctrl;
    auto descs = ctrl.getParameterDescriptors();
    
    EXPECT_EQ(descs.size(), 3);
    EXPECT_EQ(descs[0].name, "Kp");
    EXPECT_EQ(descs[1].name, "Ki");
    EXPECT_EQ(descs[2].name, "Kd");
}

TEST(TestPIDControllerTest, GetControllerTypeName) {
    TestPIDController ctrl;
    EXPECT_EQ(ctrl.getControllerTypeName(), "PID");
}

TEST(TestPIDControllerTest, Clone) {
    TestPIDController ctrl(2.0, 0.5, 0.1);
    auto clone = ctrl.clone();
    
    auto params = clone->getParameters();
    EXPECT_DOUBLE_EQ(params[0], 2.0);
    EXPECT_DOUBLE_EQ(params[1], 0.5);
    EXPECT_DOUBLE_EQ(params[2], 0.1);
}

TEST(TestPIDControllerTest, Compute) {
    TestPIDController ctrl(1.0, 0.0, 0.0);  // P-only
    ctrl.reset();
    
    double output = ctrl.compute(1.0, 0.01);  // error = 1
    EXPECT_DOUBLE_EQ(output, 1.0);  // P-only: output = Kp * error
}

TEST(TestPIDControllerTest, Reset) {
    TestPIDController ctrl(1.0, 1.0, 0.0);
    
    // Accumulate integral
    ctrl.compute(1.0, 0.01);
    ctrl.compute(1.0, 0.01);
    
    ctrl.reset();
    
    // After reset, integral should be zero
    double output = ctrl.compute(0.0, 0.01);
    EXPECT_DOUBLE_EQ(output, 0.0);
}

// ============================================================================
// TestFOPDTProcessModel Tests
// ============================================================================

TEST(TestFOPDTProcessModelTest, Construction) {
    TestFOPDTProcessModel model(2.0, 5.0, 1.0);
    
    EXPECT_EQ(model.getType(), ProcessModelType::FOPDT);
}

TEST(TestFOPDTProcessModelTest, ToFOPDT) {
    TestFOPDTProcessModel model(2.0, 5.0, 1.0);
    
    auto fopdt = model.toFOPDT();
    EXPECT_DOUBLE_EQ(fopdt.K, 2.0);
    EXPECT_DOUBLE_EQ(fopdt.tau, 5.0);
    EXPECT_DOUBLE_EQ(fopdt.L, 1.0);
}

TEST(TestFOPDTProcessModelTest, EvaluateAtDCGain) {
    TestFOPDTProcessModel model(2.0, 5.0, 0.0);
    
    auto response = model.evaluate(0.0);
    EXPECT_NEAR(std::abs(response), 2.0, 1e-6);
}

TEST(TestFOPDTProcessModelTest, StepResponse) {
    TestFOPDTProcessModel model(2.0, 5.0, 1.0);
    
    auto response = model.stepResponse(1.0, 0.1, 50.0);
    
    EXPECT_GT(response.size(), 0);
    
    // Check that response starts at 0
    EXPECT_NEAR(response.front().second, 0.0, 1e-6);
    
    // Check that response approaches steady state
    EXPECT_NEAR(response.back().second, 2.0, 0.1);
}

TEST(TestFOPDTProcessModelTest, GetUltimateParams) {
    TestFOPDTProcessModel model(1.0, 10.0, 2.0);
    
    auto [Ku, Tu] = model.getUltimateParams();
    EXPECT_GT(Ku, 0.0);
    EXPECT_GT(Tu, 0.0);
}

// ============================================================================
// ProcessModelType Enum Tests
// ============================================================================

TEST(ProcessModelTypeTest, EnumValues) {
    EXPECT_NE(ProcessModelType::FOPDT, ProcessModelType::SOPDT);
    EXPECT_NE(ProcessModelType::SOPDT, ProcessModelType::IPDT);
    EXPECT_NE(ProcessModelType::IPDT, ProcessModelType::IFOPDT);
}

// ============================================================================
// PerformanceObjective Enum Tests
// ============================================================================

TEST(PerformanceObjectiveTest, EnumValues) {
    EXPECT_NE(PerformanceObjective::SetpointTracking, PerformanceObjective::DisturbanceRejection);
    EXPECT_NE(PerformanceObjective::Balanced, PerformanceObjective::MinimumTime);
}

// ============================================================================
// PIDForm Enum Tests
// ============================================================================

TEST(PIDFormTest, EnumValues) {
    EXPECT_NE(PIDForm::Parallel, PIDForm::Standard);
    EXPECT_NE(PIDForm::Standard, PIDForm::Series);
}
// ============================================================================
// StandardCostFunctions Tests
// ============================================================================

TEST(StandardCostFunctionsTest, ISE_ZeroError) {
    std::vector<double> error = {0.0, 0.0, 0.0, 0.0};
    double dt = 0.1;
    
    double ise = StandardCostFunctions::ISE(error, dt);
    EXPECT_DOUBLE_EQ(ise, 0.0);
}

TEST(StandardCostFunctionsTest, ISE_ConstantError) {
    std::vector<double> error = {1.0, 1.0, 1.0, 1.0};  // 4 samples
    double dt = 0.1;
    
    // ISE = sum(e^2 * dt) = 4 * 1^2 * 0.1 = 0.4
    double ise = StandardCostFunctions::ISE(error, dt);
    EXPECT_NEAR(ise, 0.4, 1e-10);
}

TEST(StandardCostFunctionsTest, ISE_VaryingError) {
    std::vector<double> error = {1.0, 2.0, 3.0};  // 3 samples
    double dt = 0.1;
    
    // ISE = (1^2 + 2^2 + 3^2) * 0.1 = 14 * 0.1 = 1.4
    double ise = StandardCostFunctions::ISE(error, dt);
    EXPECT_NEAR(ise, 1.4, 1e-10);
}

TEST(StandardCostFunctionsTest, IAE_ZeroError) {
    std::vector<double> error = {0.0, 0.0, 0.0, 0.0};
    double dt = 0.1;
    
    double iae = StandardCostFunctions::IAE(error, dt);
    EXPECT_DOUBLE_EQ(iae, 0.0);
}

TEST(StandardCostFunctionsTest, IAE_ConstantError) {
    std::vector<double> error = {1.0, 1.0, 1.0, 1.0};
    double dt = 0.1;
    
    // IAE = sum(|e| * dt) = 4 * 1 * 0.1 = 0.4
    double iae = StandardCostFunctions::IAE(error, dt);
    EXPECT_NEAR(iae, 0.4, 1e-10);
}

TEST(StandardCostFunctionsTest, IAE_NegativeError) {
    std::vector<double> error = {-1.0, -2.0, 1.0, 2.0};
    double dt = 0.1;
    
    // IAE = (1 + 2 + 1 + 2) * 0.1 = 0.6
    double iae = StandardCostFunctions::IAE(error, dt);
    EXPECT_NEAR(iae, 0.6, 1e-10);
}

TEST(StandardCostFunctionsTest, ITAE_ZeroError) {
    std::vector<double> error = {0.0, 0.0, 0.0, 0.0};
    double dt = 0.1;
    
    double itae = StandardCostFunctions::ITAE(error, dt);
    EXPECT_DOUBLE_EQ(itae, 0.0);
}

TEST(StandardCostFunctionsTest, ITAE_ConstantError) {
    std::vector<double> error = {1.0, 1.0, 1.0};
    double dt = 0.1;
    
    // ITAE = t0*|e0| + t1*|e1| + t2*|e2| = 0*1 + 0.1*1 + 0.2*1 * dt
    // = (0 + 0.1 + 0.2) * 0.1 = 0.3 * 0.1 = 0.03
    double itae = StandardCostFunctions::ITAE(error, dt);
    EXPECT_NEAR(itae, 0.03, 1e-10);
}

TEST(StandardCostFunctionsTest, ITSE_ZeroError) {
    std::vector<double> error = {0.0, 0.0, 0.0, 0.0};
    double dt = 0.1;
    
    double itse = StandardCostFunctions::ITSE(error, dt);
    EXPECT_DOUBLE_EQ(itse, 0.0);
}

TEST(StandardCostFunctionsTest, ITSE_ConstantError) {
    std::vector<double> error = {1.0, 1.0, 1.0};
    double dt = 0.1;
    
    // ITSE = t0*e0^2 + t1*e1^2 + t2*e2^2 * dt
    // = (0*1 + 0.1*1 + 0.2*1) * 0.1 = 0.03
    double itse = StandardCostFunctions::ITSE(error, dt);
    EXPECT_NEAR(itse, 0.03, 1e-10);
}

TEST(StandardCostFunctionsTest, CombinedCost_EmptyResponse) {
    std::vector<double> response;
    std::vector<double> reference = {1.0};
    
    double cost = StandardCostFunctions::combinedCost(response, reference, 1.0, 1.0, 1.0, 0.1);
    EXPECT_EQ(cost, std::numeric_limits<double>::max());
}

TEST(StandardCostFunctionsTest, CombinedCost_EmptyReference) {
    std::vector<double> response = {1.0};
    std::vector<double> reference;
    
    double cost = StandardCostFunctions::combinedCost(response, reference, 1.0, 1.0, 1.0, 0.1);
    EXPECT_EQ(cost, std::numeric_limits<double>::max());
}

TEST(StandardCostFunctionsTest, CombinedCost_PerfectTracking) {
    std::vector<double> response = {1.0, 1.0, 1.0, 1.0};
    std::vector<double> reference = {1.0, 1.0, 1.0, 1.0};
    
    double cost = StandardCostFunctions::combinedCost(response, reference, 1.0, 1.0, 0.0, 0.1);
    EXPECT_NEAR(cost, 0.0, 1e-10);
}

TEST(StandardCostFunctionsTest, CombinedCost_WithOvershoot) {
    // Response that overshoots target of 1.0
    std::vector<double> response = {0.0, 0.5, 1.2, 1.1, 1.0};
    std::vector<double> reference = {1.0, 1.0, 1.0, 1.0, 1.0};
    
    double cost = StandardCostFunctions::combinedCost(response, reference, 1.0, 0.0, 0.0, 0.1);
    EXPECT_GT(cost, 0.0);  // Should have ISE and overshoot contribution
}

// ============================================================================
// StepResponseMetrics Tests
// ============================================================================

TEST(AnalyzeStepResponseTest, BasicMetrics) {
    // Simulate a step response that rises to 1.0
    std::vector<double> response;
    double finalValue = 1.0;
    double dt = 0.01;
    
    // Generate an exponential rise response: y(t) = 1 - exp(-t/tau)
    double tau = 0.5;
    for (int i = 0; i < 500; ++i) {
        double t = i * dt;
        response.push_back(finalValue * (1.0 - std::exp(-t / tau)));
    }
    
    auto metrics = analyzeStepResponse(response, finalValue, dt, 0.02);
    
    EXPECT_GT(metrics.riseTime, 0.0);
    EXPECT_GT(metrics.settlingTime, 0.0);
    EXPECT_NEAR(metrics.steadyStateValue, finalValue, 0.1);
}

TEST(AnalyzeStepResponseTest, EmptyResponse) {
    std::vector<double> response;
    auto metrics = analyzeStepResponse(response, 1.0, 0.1, 0.02);
    
    // Should handle empty gracefully
    EXPECT_EQ(metrics.riseTime, 0.0);
}

TEST(AnalyzeStepResponseTest, WithOvershoot) {
    // Create a response with overshoot
    std::vector<double> response;
    double finalValue = 1.0;
    double dt = 0.01;
    
    // Second-order underdamped response: overshoots then settles
    double wn = 10.0;
    double zeta = 0.3;  // Underdamped
    double wd = wn * std::sqrt(1.0 - zeta * zeta);
    
    for (int i = 0; i < 500; ++i) {
        double t = i * dt;
        double y = 1.0 - std::exp(-zeta * wn * t) * 
            (std::cos(wd * t) + (zeta * wn / wd) * std::sin(wd * t));
        response.push_back(y);
    }
    
    auto metrics = analyzeStepResponse(response, finalValue, dt, 0.02);
    
    EXPECT_GT(metrics.overshoot, 0.0);  // Should have overshoot
    EXPECT_GT(metrics.peakTime, 0.0);   // Peak should be after t=0
}

// ============================================================================
// SimulationCostFunction Tests  
// ============================================================================

TEST(SimulationCostFunctionTest, NullController) {
    auto model = std::make_shared<TestFOPDTProcessModel>(1.0, 1.0, 0.0);
    std::vector<double> ref = {1.0, 1.0, 1.0};
    
    SimulationCostFunction costFunc(nullptr, model, ref, 0.1);
    
    ParameterVector params = {1.0, 0.1, 0.01};
    double cost = costFunc.evaluate(params);
    
    EXPECT_EQ(cost, std::numeric_limits<double>::max());
}

TEST(SimulationCostFunctionTest, NullModel) {
    auto controller = std::make_shared<TestPIDController>(1.0, 0.1, 0.01);
    std::vector<double> ref = {1.0, 1.0, 1.0};
    
    SimulationCostFunction costFunc(controller, nullptr, ref, 0.1);
    
    ParameterVector params = {1.0, 0.1, 0.01};
    double cost = costFunc.evaluate(params);
    
    EXPECT_EQ(cost, std::numeric_limits<double>::max());
}

TEST(SimulationCostFunctionTest, SetWeights) {
    auto controller = std::make_shared<TestPIDController>(1.0, 0.1, 0.01);
    auto model = std::make_shared<TestFOPDTProcessModel>(1.0, 1.0, 0.0);
    std::vector<double> ref = {1.0, 1.0, 1.0};
    
    SimulationCostFunction costFunc(controller, model, ref, 0.1);
    costFunc.setWeights(1.0, 0.5, 0.3, 0.2);
    
    // Just verify it doesn't crash
    ParameterVector params = {1.0, 0.1, 0.01};
    double cost = costFunc.evaluate(params);
    EXPECT_TRUE(std::isfinite(cost) || cost == std::numeric_limits<double>::max());
}

// ============================================================================
// FrequencyResponseMetrics Tests
// ============================================================================

TEST(AnalyzeFrequencyResponseTest, BasicAnalysis) {
    TestFOPDTProcessModel model(1.0, 1.0, 0.0);
    TestPIDController controller(1.0, 0.1, 0.01);
    
    auto metrics = analyzeFrequencyResponse(model, controller);
    
    // Should have computed some metrics
    EXPECT_GE(metrics.crossoverFrequency, 0.0);
}
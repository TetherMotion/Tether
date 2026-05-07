// Monolithic classical tuning tests were split into per-controller test files
// under tests/control/autotuning/classical/. This file is retained as a small
// compatibility placeholder to avoid breaking existing test runners.

#include <gtest/gtest.h>
#include "tether/control/autotuning/ClassicalTuningMethods.hpp"

using Control::Autotuning::ClassicalTuningFactory;

TEST(ClassicalTuningMethods_CompatibilityPlaceholder, Compiles) {
    // Ensure factory returns at least one classical method (behavioral check)
    auto methods = ClassicalTuningFactory::getAvailableMethods();
    EXPECT_FALSE(methods.empty());
    for (const auto &m : methods) {
        auto tuner = ClassicalTuningFactory::create(m);
        EXPECT_NE(tuner, nullptr);
        EXPECT_FALSE(tuner->getName().empty());
    }
}


#if 0

TEST(ZNStepResponseCoverage, CalculateGainsZeroL) {
    FOPDTModel model;
    model.K = 1.0;
    model.tau = 10.0;
    model.L = 0.0;  // Zero delay
    
    auto gains = ZieglerNicholsStepResponse::calculateGains(model, PIDForm::Parallel);
    // Should handle gracefully
}

TEST(ZNUltimateCycleCoverage, AllVariants) {
    auto controller = std::make_shared<TestPIDController>(1.0, 0.1, 0.01);
    TestFOPDTProcessModel model(1.0, 10.0, 2.0);
    
    // Test different variants
    ZieglerNicholsUltimateCycle zn;
    zn.setUltimateParameters(4.0, 5.0);  // Ku, Tu
    
    zn.setVariant("original");
    auto result1 = zn.tune(*controller, &model);
    
    zn.setVariant("some");
    auto result2 = zn.tune(*controller, &model);
    
    zn.setVariant("none");
    auto result3 = zn.tune(*controller, &model);
}

TEST(CohenCoonCoverage, DifferentForms) {
    FOPDTModel model;
    model.K = 1.0;
    model.tau = 10.0;
    model.L = 2.0;
    
    // Test parallel form
    auto gains1 = CohenCoon::calculateGains(model, PIDForm::Parallel);
    EXPECT_TRUE(gains1.isValid());
    
    // Test standard form
    auto gains2 = CohenCoon::calculateGains(model, PIDForm::Standard);
    EXPECT_TRUE(gains2.isValid());
}

TEST(CHRCoverage, AllModes) {
    auto controller = std::make_shared<TestPIDController>(1.0, 0.1, 0.01);
    TestFOPDTProcessModel model(1.0, 10.0, 2.0);
    
    ChienHronesReswick chr;
    
    // Test different modes
    chr.setTuningMode(ChienHronesReswick::Mode::SetpointNoOvershoot);
    chr.tune(*controller, &model);
    
    chr.setTuningMode(ChienHronesReswick::Mode::Setpoint20Overshoot);
    chr.tune(*controller, &model);
    
    chr.setTuningMode(ChienHronesReswick::Mode::RegulatorNoOvershoot);
    chr.tune(*controller, &model);
    
    chr.setTuningMode(ChienHronesReswick::Mode::Regulator20Overshoot);
    chr.tune(*controller, &model);
}

TEST(RelayCoverage, UpdateSequence) {
    AstromHagglundRelay relay;
    
    AstromHagglundRelay::Config config;
    config.relayAmplitude = 2.0;
    config.hysteresis = 0.1;
    config.minCycles = 3;
    relay.setConfig(config);
    
    auto controller = std::make_shared<TestPIDController>(1.0, 0.1, 0.01);
    EXPECT_TRUE(relay.isCompatible(*controller));
    
    relay.start();
    
    // Simulate oscillation
    for (int i = 0; i < 500; ++i) {
        double t = i * 0.1;
        double measured = std::sin(t) + 0.01 * std::sin(5*t);
        double result = relay.update(measured, 0.0, 0.0, 0.1);
        EXPECT_TRUE(std::isfinite(result));
        
        if (relay.isComplete()) break;
    }
    
    relay.stop();
    
    double Ku = relay.getUltimateGain();
    double Tu = relay.getUltimatePeriod();
    EXPECT_TRUE(std::isfinite(Ku));
    EXPECT_TRUE(std::isfinite(Tu));
}

TEST(LopezMethodCoverage, DifferentCriteria) {
    auto controller = std::make_shared<TestPIDController>(1.0, 0.1, 0.01);
    TestFOPDTProcessModel model(1.0, 10.0, 2.0);
    
    LopezMethod lopez;
    
    // Test different criteria
    lopez.setCriterion(LopezMethod::Criterion::IAE);
    lopez.tune(*controller, &model);
    
    lopez.setCriterion(LopezMethod::Criterion::ITAE);
    lopez.tune(*controller, &model);
    
    lopez.setCriterion(LopezMethod::Criterion::ISE);
    lopez.tune(*controller, &model);
}

TEST(LambdaTuningCoverage, AllConfigurations) {
    auto controller = std::make_shared<TestPIDController>(1.0, 0.1, 0.01);
    TestFOPDTProcessModel model(1.0, 10.0, 2.0);
    
    LambdaTuning lambda;
    
    // Test different lambda values
    lambda.setLambda(5.0);
    lambda.tune(*controller, &model);
    
    lambda.setLambda(20.0);
    lambda.tune(*controller, &model);
    
    // Test with PI
    lambda.setLambda(10.0);
    auto result = lambda.tune(*controller, &model);
}

TEST(SIMCCoverage, AllConfigurations) {
    auto controller = std::make_shared<TestPIDController>(1.0, 0.1, 0.01);
    TestFOPDTProcessModel model(1.0, 10.0, 2.0);
    
    SIMCMethod simc;
    
    // Test tight tuning (small tauC)
    simc.setTauC(1.0);
    simc.tune(*controller, &model);
    
    // Test conservative tuning (large tauC)
    simc.setTauC(5.0);
    simc.tune(*controller, &model);
}

TEST(AMIGOCoverage, AllConfigurations) {
    auto controller = std::make_shared<TestPIDController>(1.0, 0.1, 0.01);
    TestFOPDTProcessModel model(1.0, 10.0, 2.0);
    
    AMIGOMethod amigo;
    
    EXPECT_EQ(amigo.getName(), "AMIGO");
    EXPECT_FALSE(amigo.getDescription().empty());
    
    auto result = amigo.tune(*controller, &model);
}

TEST(ProcessIdentificationCoverage, EstimateUltimateFromModel) {
    FOPDTModel model;
    model.K = 2.0;
    model.tau = 5.0;
    model.L = 1.0;
    
    auto [Ku, Tu] = ProcessIdentification::estimateUltimate(model);
    EXPECT_TRUE(std::isfinite(Ku));
    EXPECT_TRUE(std::isfinite(Tu));
    EXPECT_GT(Ku, 0.0);
    EXPECT_GT(Tu, 0.0);
}

TEST(ClassicalTuningFactoryCoverage, CreateAllMethods) {
    // Test creating all methods via factory
    auto methods = ClassicalTuningFactory::getAvailableMethods();
    
    for (const auto& method : methods) {
        auto tuner = ClassicalTuningFactory::create(method);
        EXPECT_NE(tuner, nullptr);
        EXPECT_FALSE(tuner->getName().empty());
    }
}

TEST(PIDGainsCoverage, FormConversions) {
    PIDGains gains;
    gains.Kp = 2.0;
    gains.Ki = 0.5;
    gains.Kd = 0.1;
    
    // toStandardForm modifies in place
    PIDGains standard = gains;  // Copy
    standard.toStandardForm();
    
    // toParallelForm also modifies in place
    PIDGains parallel = standard;  // Copy
    parallel.toParallelForm();
    
    // Round-trip should give similar results
    EXPECT_NEAR(parallel.Kp, gains.Kp, 0.01);
}

// ============================================================================
// Additional Coverage Tests for ClassicalTuningMethods
// ============================================================================

TEST(ZNStepResponseCoverage, WithStepResponseData) {
    ZieglerNicholsStepResponse tuner;
    
    // Generate synthetic step response data
    std::vector<double> time, response;
    double stepSize = 10.0;
    double K = 1.5;
    double tau = 5.0;
    double L = 0.8;
    
    for (int i = 0; i < 200; ++i) {
        double t = i * 0.1;
        time.push_back(t);
        
        double y = 0.0;
        if (t > L) {
            y = K * stepSize * (1.0 - std::exp(-(t - L) / tau));
        }
        response.push_back(y);
    }
    
    tuner.setStepResponseData(time, response, stepSize, 0.0);
    
    auto controller = std::make_shared<TestPIDController>(1.0, 0.1, 0.01);
    auto result = tuner.tune(*controller, nullptr);
    
    EXPECT_TRUE(result.success);
}

TEST(ZNStepResponseCoverage, IdentifyModelStatic) {
    std::vector<double> time, response;
    double stepSize = 10.0;
    double K = 2.0;
    double tau = 4.0;
    double L = 1.0;
    
    for (int i = 0; i < 200; ++i) {
        double t = i * 0.1;
        time.push_back(t);
        
        double y = 0.0;
        if (t > L) {
            y = K * stepSize * (1.0 - std::exp(-(t - L) / tau));
        }
        response.push_back(y);
    }
    
    auto model = ZieglerNicholsStepResponse::identifyModel(time, response, stepSize, 0.0);
    
    EXPECT_NEAR(model.K, K, 0.5);
    EXPECT_GT(model.tau, 0.0);
}

TEST(ZNStepResponseCoverage, AllPIDForms) {
    FOPDTModel model;
    model.K = 1.0;
    model.tau = 10.0;
    model.L = 2.0;
    
    auto gainsParallel = ZieglerNicholsStepResponse::calculateGains(model, PIDForm::Parallel);
    auto gainsStandard = ZieglerNicholsStepResponse::calculateGains(model, PIDForm::Standard);
    auto gainsSeries = ZieglerNicholsStepResponse::calculateGains(model, PIDForm::Series);
    
    EXPECT_TRUE(gainsParallel.isValid());
    EXPECT_TRUE(gainsStandard.isValid());
    EXPECT_TRUE(gainsSeries.isValid());
}

TEST(ZNStepResponseCoverage, InvalidModel) {
    ZieglerNicholsStepResponse tuner;
    
    auto controller = std::make_shared<TestPIDController>(1.0, 0.1, 0.01);
    auto result = tuner.tune(*controller, nullptr);
    
    // Should fail without model
    EXPECT_FALSE(result.success);
}

TEST(ZNUltimateCycleCoverage, AllPIDForms) {
    double Ku = 5.0;
    double Tu = 2.0;
    
    auto gainsParallel = ZieglerNicholsUltimateCycle::calculateGains(Ku, Tu, PIDForm::Parallel);
    auto gainsStandard = ZieglerNicholsUltimateCycle::calculateGains(Ku, Tu, PIDForm::Standard);
    auto gainsSeries = ZieglerNicholsUltimateCycle::calculateGains(Ku, Tu, PIDForm::Series);
    
    EXPECT_TRUE(gainsParallel.isValid());
    EXPECT_TRUE(gainsStandard.isValid());
    EXPECT_TRUE(gainsSeries.isValid());
}

TEST(CohenCoonCoverage, AllPIDForms) {
    FOPDTModel model;
    model.K = 1.0;
    model.tau = 10.0;
    model.L = 2.0;
    
    auto gainsParallel = CohenCoon::calculateGains(model, PIDForm::Parallel);
    auto gainsStandard = CohenCoon::calculateGains(model, PIDForm::Standard);
    auto gainsSeries = CohenCoon::calculateGains(model, PIDForm::Series);
    
    EXPECT_TRUE(gainsParallel.isValid());
    EXPECT_TRUE(gainsStandard.isValid());
    EXPECT_TRUE(gainsSeries.isValid());
}

TEST(CHRCoverage, AllPIDForms) {
    FOPDTModel model;
    model.K = 1.0;
    model.tau = 10.0;
    model.L = 2.0;
    
    std::vector<ChienHronesReswick::Mode> modes = {
        ChienHronesReswick::Mode::SetpointNoOvershoot,
        ChienHronesReswick::Mode::Setpoint20Overshoot,
        ChienHronesReswick::Mode::RegulatorNoOvershoot,
        ChienHronesReswick::Mode::Regulator20Overshoot
    };
    
    for (auto mode : modes) {
        auto gainsParallel = ChienHronesReswick::calculateGains(model, PIDForm::Parallel, mode);
        auto gainsStandard = ChienHronesReswick::calculateGains(model, PIDForm::Standard, mode);
        
        EXPECT_TRUE(gainsParallel.isValid());
        EXPECT_TRUE(gainsStandard.isValid());
    }
}

TEST(AstromHagglundRelayCoverage, AllTuningRules) {
    std::vector<AstromHagglundRelay::TuningRule> rules = {
        AstromHagglundRelay::TuningRule::ZieglerNichols,
        AstromHagglundRelay::TuningRule::TyreusLuyben,
        AstromHagglundRelay::TuningRule::AMIGO
    };
    
    for (auto rule : rules) {
        AstromHagglundRelay relay;
        
        AstromHagglundRelay::Config config;
        config.relayAmplitude = 5.0;
        config.hysteresis = 0.2;
        config.minCycles = 3;
        config.maxCycles = 20;
        relay.setConfig(config);
        relay.setTuningRule(rule);
        
        relay.start();
        
        // Generate oscillation that completes
        for (int i = 0; i < 600; ++i) {
            double t = i * 0.01;
            double measured = 5.0 * std::sin(2.0 * M_PI * t);  // Oscillation
            relay.update(measured, 0.0, 0.0, 0.01);
            
            if (relay.isComplete()) break;
        }
        
        relay.stop();
        
        auto Ku = relay.getUltimateGain();
        auto Tu = relay.getUltimatePeriod();
    }
}

TEST(AstromHagglundRelayCoverage, GetIntermediateResult) {
    AstromHagglundRelay relay;
    
    AstromHagglundRelay::Config config;
    config.relayAmplitude = 5.0;
    config.minCycles = 5;
    relay.setConfig(config);
    
    relay.start();
    
    // Run for a few cycles
    for (int i = 0; i < 200; ++i) {
        double t = i * 0.01;
        relay.update(5.0 * std::sin(2.0 * M_PI * t), 0.0, 0.0, 0.01);
    }
    
    auto result = relay.getIntermediateResult();
    // Result may or may not be complete
    
    relay.stop();
}

TEST(LopezMethodCoverage, AllCombinations) {
    FOPDTModel model;
    model.K = 1.0;
    model.tau = 10.0;
    model.L = 2.0;
    
    std::vector<LopezMethod::Criterion> criteria = {
        LopezMethod::Criterion::IAE,
        LopezMethod::Criterion::ITAE,
        LopezMethod::Criterion::ISE
    };
    
    std::vector<LopezMethod::ResponseType> responses = {
        LopezMethod::ResponseType::Setpoint,
        LopezMethod::ResponseType::Disturbance
    };
    
    for (auto criterion : criteria) {
        for (auto response : responses) {
            auto gains = LopezMethod::calculateGains(model, PIDForm::Parallel, criterion, response);
            EXPECT_TRUE(gains.isValid());
        }
    }
}

TEST(LambdaTuningCoverage, WithRobustness) {
    auto controller = std::make_shared<TestPIDController>(1.0, 0.1, 0.01);
    TestFOPDTProcessModel model(1.0, 10.0, 2.0);
    
    LambdaTuning tuner;
    
    // Test with robustness settings
    tuner.setRobustness(0.0);  // Aggressive
    tuner.tune(*controller, &model);
    
    tuner.setRobustness(0.5);  // Moderate
    tuner.tune(*controller, &model);
    
    tuner.setRobustness(1.0);  // Conservative
    tuner.tune(*controller, &model);
}

TEST(LambdaTuningCoverage, CalculateGainsWithDerivative) {
    FOPDTModel model;
    model.K = 1.0;
    model.tau = 10.0;
    model.L = 2.0;
    
    auto gainsWithD = LambdaTuning::calculateGains(model, 5.0, true);
    auto gainsNoD = LambdaTuning::calculateGains(model, 5.0, false);
    
    EXPECT_TRUE(gainsWithD.isValid());
    EXPECT_TRUE(gainsNoD.isValid());
    EXPECT_GT(gainsWithD.Kd, 0.0);
}

TEST(SIMCMethodCoverage, AllConfigurations) {
    auto controller = std::make_shared<TestPIDController>(1.0, 0.1, 0.01);
    TestFOPDTProcessModel model(1.0, 10.0, 2.0);
    
    SIMCMethod simc;
    
    // Test static calculation
    FOPDTModel fopdt;
    fopdt.K = 1.0;
    fopdt.tau = 10.0;
    fopdt.L = 2.0;
    
    auto gains = SIMCMethod::calculateGains(fopdt, 2.0);
    EXPECT_TRUE(gains.isValid());
}

TEST(ProcessIdentificationCoverage, AreaMethod) {
    std::vector<double> time, response;
    double stepSize = 10.0;
    double K = 1.0;
    double tau = 5.0;
    double L = 1.0;
    
    for (int i = 0; i < 200; ++i) {
        double t = i * 0.1;
        time.push_back(t);
        
        double y = 0.0;
        if (t > L) {
            y = K * stepSize * (1.0 - std::exp(-(t - L) / tau));
        }
        response.push_back(y);
    }
    
    auto model = ProcessIdentification::areaMethod(time, response, stepSize);
    
    EXPECT_NEAR(model.K, K, 0.2);
    EXPECT_GT(model.tau, 0.0);
}

TEST(ProcessIdentificationCoverage, TwoPointMethod) {
    std::vector<double> time, response;
    double stepSize = 10.0;
    double K = 1.0;
    double tau = 5.0;
    double L = 1.0;
    
    for (int i = 0; i < 200; ++i) {
        double t = i * 0.1;
        time.push_back(t);
        
        double y = 0.0;
        if (t > L) {
            y = K * stepSize * (1.0 - std::exp(-(t - L) / tau));
        }
        response.push_back(y);
    }
    
    auto model = ProcessIdentification::twoPointMethod(time, response, stepSize);
    
    EXPECT_NEAR(model.K, K, 0.2);
    EXPECT_GT(model.tau, 0.0);
}

TEST(ProcessIdentificationCoverage, LeastSquaresFit) {
    std::vector<double> time, response;
    double stepSize = 10.0;
    double K = 1.0;
    double tau = 5.0;
    double L = 1.0;
    
    for (int i = 0; i < 200; ++i) {
        double t = i * 0.1;
        time.push_back(t);
        
        double y = 0.0;
        if (t > L) {
            y = K * stepSize * (1.0 - std::exp(-(t - L) / tau));
        }
        // Add some noise
        y += 0.1 * std::sin(10.0 * t);
        response.push_back(y);
    }
    
    auto model = ProcessIdentification::leastSquaresFit(time, response, stepSize);
    
    EXPECT_NEAR(model.K, K, 0.3);
    EXPECT_GT(model.tau, 0.0);
}

TEST(ProcessIdentificationCoverage, IdentifySOPDT) {
    std::vector<double> time, response;
    double stepSize = 10.0;
    double K = 1.0;
    double tau1 = 5.0;
    double tau2 = 2.0;
    double L = 1.0;
    
    for (int i = 0; i < 200; ++i) {
        double t = i * 0.1;
        time.push_back(t);
        
        double y = 0.0;
        if (t > L) {
            // SOPDT response approximation
            double tEff = t - L;
            y = K * stepSize * (1.0 - (tau1*std::exp(-tEff/tau1) - tau2*std::exp(-tEff/tau2))/(tau1-tau2));
        }
        response.push_back(y);
    }
    
    auto model = ProcessIdentification::identifySOPDT(time, response, stepSize);
    
    EXPECT_NEAR(model.K, K, 0.2);
    EXPECT_GT(model.tau1, 0.0);
    EXPECT_GT(model.tau2, 0.0);
}

TEST(ClassicalTuningFactoryCoverage, CreateAllMethods2) {
    // Test all methods through the factory
    std::vector<ClassicalTuningFactory::Method> methods = {
        ClassicalTuningFactory::Method::ZieglerNicholsStep,
        ClassicalTuningFactory::Method::ZieglerNicholsUltimate,
        ClassicalTuningFactory::Method::TyreusLuyben,
        ClassicalTuningFactory::Method::CohenCoon,
        ClassicalTuningFactory::Method::CHR_SetpointNoOS,
        ClassicalTuningFactory::Method::CHR_Setpoint20OS,
        ClassicalTuningFactory::Method::CHR_RegulatorNoOS,
        ClassicalTuningFactory::Method::CHR_Regulator20OS,
        ClassicalTuningFactory::Method::RelayFeedback,
        ClassicalTuningFactory::Method::LopezIAE,
        ClassicalTuningFactory::Method::LopezITAE,
        ClassicalTuningFactory::Method::LopezISE,
        ClassicalTuningFactory::Method::Lambda,
        ClassicalTuningFactory::Method::SIMC,
        ClassicalTuningFactory::Method::AMIGO
    };
    
    for (auto method : methods) {
        auto tuner = ClassicalTuningFactory::create(method);
        EXPECT_NE(tuner, nullptr);
        EXPECT_FALSE(tuner->getName().empty());
        EXPECT_FALSE(tuner->getDescription().empty());
    }
}

TEST(AMIGOCoverage, CalculateGainsStatic) {
    FOPDTModel model;
    model.K = 1.0;
    model.tau = 10.0;
    model.L = 2.0;
    
    auto gainsParallel = AMIGOMethod::calculateGains(model, PIDForm::Parallel);
    auto gainsStandard = AMIGOMethod::calculateGains(model, PIDForm::Standard);
    
    EXPECT_TRUE(gainsParallel.isValid());
    EXPECT_TRUE(gainsStandard.isValid());
}

TEST(ClassicalEdgeCases, EmptyData) {
    std::vector<double> time, response;
    
    // Empty data - these may return default models
    // Just verify they don't crash
    auto model1 = ProcessIdentification::areaMethod(time, response, 10.0);
    auto model2 = ProcessIdentification::twoPointMethod(time, response, 10.0);
    
    // Very small data
    time = {0.0, 0.1};
    response = {0.0, 0.1};
    auto model3 = ProcessIdentification::tangentMethod(time, response, 10.0);
}

TEST(ClassicalEdgeCases, ZeroGainModel) {
    FOPDTModel model;
    model.K = 0.0;  // Zero gain
    model.tau = 10.0;
    model.L = 2.0;
    
    auto gains = ZieglerNicholsStepResponse::calculateGains(model, PIDForm::Parallel);
    // Should return default/zero gains for invalid model
}

TEST(ClassicalEdgeCases, ZeroDeadTimeModel) {
    FOPDTModel model;
    model.K = 1.0;
    model.tau = 10.0;
    model.L = 0.0;  // Zero dead time
    
    auto gains = ZieglerNicholsStepResponse::calculateGains(model, PIDForm::Parallel);
    // Should handle gracefully
}

#endif

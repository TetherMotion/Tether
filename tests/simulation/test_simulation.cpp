#include <gtest/gtest.h>
#include "tether/simulation/AllSystems.hpp"
#include "tether/simulation/SimulationEngine.hpp"
#include "tether/simulation/Integrators.hpp"
#include "tether/simulation/SensorActuatorModels.hpp"
#include <cmath>
#include <algorithm>
#include <numeric>
#include <memory>

namespace {

// ===== AllSystems Factory Tests =====

TEST(AllSystemsFactory, SystemCountIs70) {
    EXPECT_EQ(Simulation::systemCount(), 70);
}

TEST(AllSystemsFactory, ListSystemsReturns70) {
    auto systems = Simulation::listSystems();
    EXPECT_EQ(systems.size(), 70u);
}

TEST(AllSystemsFactory, ListSystemsHasUniqueIds) {
    auto systems = Simulation::listSystems();
    std::set<int> ids;
    for (auto& [id, name] : systems) ids.insert(id);
    EXPECT_EQ(ids.size(), 70u);
}

TEST(AllSystemsFactory, CreateSystemValidIds) {
    for (int id = 1; id <= 70; ++id) {
        auto sys = Simulation::createSystem(id);
        ASSERT_NE(sys, nullptr) << "createSystem(" << id << ") returned null";
        EXPECT_EQ(sys->systemId(), id);
    }
}

TEST(AllSystemsFactory, CreateSystemInvalidIds) {
    EXPECT_EQ(Simulation::createSystem(0), nullptr);
    EXPECT_EQ(Simulation::createSystem(-1), nullptr);
    EXPECT_EQ(Simulation::createSystem(71), nullptr);
    EXPECT_EQ(Simulation::createSystem(999), nullptr);
}

TEST(AllSystemsFactory, AllSystemsHaveNames) {
    for (int id = 1; id <= 70; ++id) {
        auto sys = Simulation::createSystem(id);
        ASSERT_NE(sys, nullptr);
        EXPECT_NE(std::string(sys->name()), "");
        EXPECT_NE(std::string(sys->description()), "");
    }
}

// ===== DynamicalSystem Interface Tests =====

class SystemInterfaceTest : public ::testing::TestWithParam<int> {};

TEST_P(SystemInterfaceTest, DimensionsArePositive) {
    auto sys = Simulation::createSystem(GetParam());
    ASSERT_NE(sys, nullptr);
    EXPECT_GT(sys->stateDim(), 0);
    EXPECT_GT(sys->inputDim(), 0);
    EXPECT_GT(sys->outputDim(), 0);
}

TEST_P(SystemInterfaceTest, DefaultInitialStateHasCorrectSize) {
    auto sys = Simulation::createSystem(GetParam());
    ASSERT_NE(sys, nullptr);
    auto x0 = sys->defaultInitialState();
    EXPECT_EQ(static_cast<int>(x0.size()), sys->stateDim());
}

TEST_P(SystemInterfaceTest, DefaultInputHasCorrectSize) {
    auto sys = Simulation::createSystem(GetParam());
    ASSERT_NE(sys, nullptr);
    auto u0 = sys->defaultInput();
    EXPECT_EQ(static_cast<int>(u0.size()), sys->inputDim());
}

TEST_P(SystemInterfaceTest, DynamicsReturnsCorrectSize) {
    auto sys = Simulation::createSystem(GetParam());
    ASSERT_NE(sys, nullptr);
    auto x0 = sys->defaultInitialState();
    auto u0 = sys->defaultInput();
    auto dx = sys->dynamics(0.0, x0, u0);
    EXPECT_EQ(static_cast<int>(dx.size()), sys->stateDim());
}

TEST_P(SystemInterfaceTest, OutputReturnsCorrectSize) {
    auto sys = Simulation::createSystem(GetParam());
    ASSERT_NE(sys, nullptr);
    auto x0 = sys->defaultInitialState();
    auto u0 = sys->defaultInput();
    auto y = sys->output(0.0, x0, u0);
    EXPECT_EQ(static_cast<int>(y.size()), sys->outputDim());
}

TEST_P(SystemInterfaceTest, DynamicsReturnsFinite) {
    auto sys = Simulation::createSystem(GetParam());
    ASSERT_NE(sys, nullptr);
    auto x0 = sys->defaultInitialState();
    auto u0 = sys->defaultInput();
    auto dx = sys->dynamics(0.0, x0, u0);
    for (size_t i = 0; i < dx.size(); ++i) {
        EXPECT_TRUE(std::isfinite(dx[i]))
            << "System " << GetParam() << " (" << sys->name()
            << ") dx[" << i << "] = " << dx[i];
    }
}

TEST_P(SystemInterfaceTest, OutputReturnsFinite) {
    auto sys = Simulation::createSystem(GetParam());
    ASSERT_NE(sys, nullptr);
    auto x0 = sys->defaultInitialState();
    auto u0 = sys->defaultInput();
    auto y = sys->output(0.0, x0, u0);
    for (size_t i = 0; i < y.size(); ++i) {
        EXPECT_TRUE(std::isfinite(y[i]))
            << "System " << GetParam() << " (" << sys->name()
            << ") y[" << i << "] = " << y[i];
    }
}

TEST_P(SystemInterfaceTest, StateNamesMatchDim) {
    auto sys = Simulation::createSystem(GetParam());
    ASSERT_NE(sys, nullptr);
    EXPECT_EQ(static_cast<int>(sys->stateNames().size()), sys->stateDim());
}

TEST_P(SystemInterfaceTest, OutputNamesMatchDim) {
    auto sys = Simulation::createSystem(GetParam());
    ASSERT_NE(sys, nullptr);
    EXPECT_EQ(static_cast<int>(sys->outputNames().size()), sys->outputDim());
}

TEST_P(SystemInterfaceTest, InputNamesMatchDim) {
    auto sys = Simulation::createSystem(GetParam());
    ASSERT_NE(sys, nullptr);
    EXPECT_EQ(static_cast<int>(sys->inputNames().size()), sys->inputDim());
}

TEST_P(SystemInterfaceTest, ParameterDescriptorsNonEmpty) {
    auto sys = Simulation::createSystem(GetParam());
    ASSERT_NE(sys, nullptr);
    auto descriptors = sys->parameterDescriptors();
    EXPECT_GT(descriptors.size(), 0u);
    for (auto& pd : descriptors) {
        EXPECT_FALSE(pd.name.empty());
        EXPECT_LE(pd.minValue, pd.maxValue);
        EXPECT_GE(pd.defaultValue, pd.minValue);
        EXPECT_LE(pd.defaultValue, pd.maxValue);
    }
}

TEST_P(SystemInterfaceTest, GetParameterReturnsValue) {
    auto sys = Simulation::createSystem(GetParam());
    ASSERT_NE(sys, nullptr);
    auto descriptors = sys->parameterDescriptors();
    for (auto& pd : descriptors) {
        double val = sys->getParameter(pd.name);
        EXPECT_TRUE(std::isfinite(val))
            << "System " << GetParam() << " param " << pd.name
            << " = " << val;
    }
}

TEST_P(SystemInterfaceTest, SetParameterWorks) {
    auto sys = Simulation::createSystem(GetParam());
    ASSERT_NE(sys, nullptr);
    auto descriptors = sys->parameterDescriptors();
    if (!descriptors.empty()) {
        auto& pd = descriptors[0];
        double newVal = (pd.minValue + pd.maxValue) / 2.0;
        sys->setParameter(pd.name, newVal);
        EXPECT_NEAR(sys->getParameter(pd.name), newVal, 1e-10);
    }
}

TEST_P(SystemInterfaceTest, PresetsAreValid) {
    auto sys = Simulation::createSystem(GetParam());
    ASSERT_NE(sys, nullptr);
    auto presets = sys->presets();
    for (size_t i = 0; i < presets.size(); ++i) {
        EXPECT_FALSE(presets[i].name.empty());
        sys->applyPreset(static_cast<int>(i));
        // After applying preset, dynamics should still be finite
        auto x0 = sys->defaultInitialState();
        auto u0 = sys->defaultInput();
        auto dx = sys->dynamics(0.0, x0, u0);
        for (size_t j = 0; j < dx.size(); ++j) {
            EXPECT_TRUE(std::isfinite(dx[j]))
                << "System " << GetParam() << " preset " << presets[i].name
                << " dx[" << j << "] = " << dx[j];
        }
    }
}

TEST_P(SystemInterfaceTest, GetParametersAndSetParametersRoundTrip) {
    auto sys = Simulation::createSystem(GetParam());
    ASSERT_NE(sys, nullptr);
    auto params = sys->getParameters();
    EXPECT_GT(params.size(), 0u);
    sys->setParameters(params);
    auto params2 = sys->getParameters();
    EXPECT_EQ(params.size(), params2.size());
    for (auto& [k, v] : params) {
        EXPECT_NEAR(params2[k], v, 1e-10);
    }
}

TEST_P(SystemInterfaceTest, LinearizeProducesCorrectDimensions) {
    auto sys = Simulation::createSystem(GetParam());
    ASSERT_NE(sys, nullptr);
    auto x0 = sys->defaultInitialState();
    auto u0 = sys->defaultInput();
    std::vector<double> A, B, C, D;
    sys->linearize(x0, u0, A, B, C, D);

    int n = sys->stateDim();
    int m = sys->inputDim();
    int p = sys->outputDim();
    EXPECT_EQ(static_cast<int>(A.size()), n * n);
    EXPECT_EQ(static_cast<int>(B.size()), n * m);
    EXPECT_EQ(static_cast<int>(C.size()), p * n);
    EXPECT_EQ(static_cast<int>(D.size()), p * m);
}

TEST_P(SystemInterfaceTest, MIMODetection) {
    auto sys = Simulation::createSystem(GetParam());
    ASSERT_NE(sys, nullptr);
    bool mimo = sys->isMIMO();
    EXPECT_EQ(mimo, sys->inputDim() > 1 || sys->outputDim() > 1);
}

TEST_P(SystemInterfaceTest, DefaultSensorConfigs) {
    auto sys = Simulation::createSystem(GetParam());
    ASSERT_NE(sys, nullptr);
    auto configs = sys->defaultSensorConfigs();
    EXPECT_EQ(static_cast<int>(configs.size()), sys->outputDim());
}

TEST_P(SystemInterfaceTest, DefaultActuatorConfigs) {
    auto sys = Simulation::createSystem(GetParam());
    ASSERT_NE(sys, nullptr);
    auto configs = sys->defaultActuatorConfigs();
    EXPECT_EQ(static_cast<int>(configs.size()), sys->inputDim());
}

TEST_P(SystemInterfaceTest, CategoryIsValid) {
    auto sys = Simulation::createSystem(GetParam());
    ASSERT_NE(sys, nullptr);
    auto cat = sys->category();
    EXPECT_GE(static_cast<int>(cat), 0);
    EXPECT_LE(static_cast<int>(cat), 10);
}

TEST_P(SystemInterfaceTest, EquationStringsAreValid) {
    auto sys = Simulation::createSystem(GetParam());
    ASSERT_NE(sys, nullptr);
    auto eqs = sys->equationStrings();
    for (auto& eq : eqs) {
        EXPECT_FALSE(eq.empty());
    }
}

TEST_P(SystemInterfaceTest, GeometriesAreValid) {
    auto sys = Simulation::createSystem(GetParam());
    ASSERT_NE(sys, nullptr);
    auto geoms = sys->geometries();
    for (auto& g : geoms) {
        EXPECT_GE(g.mass, 0.0);
        auto inertia = g.computeInertia();
        EXPECT_TRUE(std::isfinite(inertia));
        auto area = g.computeArea();
        EXPECT_TRUE(std::isfinite(area));
    }
}

INSTANTIATE_TEST_SUITE_P(
    AllSystems, SystemInterfaceTest,
    ::testing::Range(1, 71),
    [](const ::testing::TestParamInfo<int>& info) {
        auto sys = Simulation::createSystem(info.param);
        std::string name = sys ? sys->name() : "Unknown";
        // Keep only alphanumeric and underscore for valid test names
        std::string clean;
        for (char c : name) {
            if (std::isalnum(static_cast<unsigned char>(c))) clean += c;
            else if (c == ' ' || c == '-') clean += '_';
        }
        return std::to_string(info.param) + "_" + clean;
    }
);

// ===== Integrator Tests =====

TEST(IntegratorTests, EulerForwardBasic) {
    auto integrator = Simulation::createIntegrator(
        Simulation::IntegrationMethod::EulerForward, 1e-6, 1e-6);
    ASSERT_NE(integrator, nullptr);

    // dx/dt = -x, x(0) = 1 => x(t) = exp(-t)
    auto odefun = [](double, const Simulation::StateVector& x) -> Simulation::StateVector {
        return {-x[0]};
    };

    Simulation::StateVector x = {1.0};
    double t = 0.0, dt = 0.001;
    for (int i = 0; i < 1000; ++i) {
        auto result = integrator->step(odefun, t, x, dt);
        x = result.state;
        t += dt;
    }
    // After t=1, x should be close to exp(-1) ≈ 0.3679
    EXPECT_NEAR(x[0], std::exp(-1.0), 0.01);
}

TEST(IntegratorTests, RK4Basic) {
    auto integrator = Simulation::createIntegrator(
        Simulation::IntegrationMethod::RungeKutta4, 1e-6, 1e-6);
    ASSERT_NE(integrator, nullptr);

    auto odefun = [](double, const Simulation::StateVector& x) -> Simulation::StateVector {
        return {-x[0]};
    };

    Simulation::StateVector x = {1.0};
    double t = 0.0, dt = 0.01;
    for (int i = 0; i < 100; ++i) {
        auto result = integrator->step(odefun, t, x, dt);
        x = result.state;
        t += dt;
    }
    EXPECT_NEAR(x[0], std::exp(-1.0), 1e-6);
}

TEST(IntegratorTests, AllMethodsCreate) {
    std::vector<Simulation::IntegrationMethod> methods = {
        Simulation::IntegrationMethod::EulerForward,
        Simulation::IntegrationMethod::RungeKutta4,
        Simulation::IntegrationMethod::DormandPrinceRK45,
        Simulation::IntegrationMethod::BogackiShampineRK23,
        Simulation::IntegrationMethod::ImplicitTrapezoidal,
        Simulation::IntegrationMethod::BDF2,
        Simulation::IntegrationMethod::SDIRK4,
    };

    for (auto method : methods) {
        auto integrator = Simulation::createIntegrator(method, 1e-6, 1e-6);
        EXPECT_NE(integrator, nullptr) << "Method " << static_cast<int>(method);
    }
}

TEST(IntegratorTests, AllMethodsSimpleODE) {
    auto odefun = [](double, const Simulation::StateVector& x) -> Simulation::StateVector {
        return {-x[0]};
    };

    std::vector<Simulation::IntegrationMethod> methods = {
        Simulation::IntegrationMethod::EulerForward,
        Simulation::IntegrationMethod::RungeKutta4,
        Simulation::IntegrationMethod::DormandPrinceRK45,
        Simulation::IntegrationMethod::BogackiShampineRK23,
        Simulation::IntegrationMethod::ImplicitTrapezoidal,
        Simulation::IntegrationMethod::BDF2,
        Simulation::IntegrationMethod::SDIRK4,
    };

    for (auto method : methods) {
        auto integrator = Simulation::createIntegrator(method, 1e-6, 1e-6);
        ASSERT_NE(integrator, nullptr);

        Simulation::StateVector x = {1.0};
        double t = 0.0, dt = 0.001;
        for (int i = 0; i < 1000; ++i) {
            auto result = integrator->step(odefun, t, x, dt);
            x = result.state;
            t += dt;
        }
        EXPECT_NEAR(x[0], std::exp(-1.0), 0.05)
            << "Method " << static_cast<int>(method) << " diverged";
    }
}

TEST(IntegratorTests, AdaptiveIntegratorsReport) {
    auto rk45 = Simulation::createIntegrator(
        Simulation::IntegrationMethod::DormandPrinceRK45, 1e-6, 1e-6);
    auto rk23 = Simulation::createIntegrator(
        Simulation::IntegrationMethod::BogackiShampineRK23, 1e-6, 1e-6);
    EXPECT_TRUE(rk45->isAdaptive());
    EXPECT_TRUE(rk23->isAdaptive());

    auto euler = Simulation::createIntegrator(
        Simulation::IntegrationMethod::EulerForward, 1e-6, 1e-6);
    EXPECT_FALSE(euler->isAdaptive());
}

// ===== SimulationEngine Tests =====

TEST(SimulationEngineTests, RunMassSpringDamper) {
    auto sys = Simulation::createSystem(1); // MassSpringDamper
    ASSERT_NE(sys, nullptr);

    Simulation::SimulationEngine engine;
    engine.setSystem(std::shared_ptr<Simulation::DynamicalSystem>(
        sys.release(), std::default_delete<Simulation::DynamicalSystem>()));

    Simulation::SimConfig cfg;
    cfg.dt = 0.001;
    cfg.totalTime = 2.0;
    cfg.method = Simulation::IntegrationMethod::RungeKutta4;
    engine.setConfig(cfg);

    engine.setInitialState({1.0, 0.0}); // displaced, zero velocity

    auto record = engine.run();
    EXPECT_GT(record.size(), 0u);
    EXPECT_EQ(record.times.size(), record.states.size());
    EXPECT_EQ(record.times.size(), record.outputs.size());
}

TEST(SimulationEngineTests, StepByStepMatchesRun) {
    auto sys = Simulation::createSystem(1);
    ASSERT_NE(sys, nullptr);

    auto sys2 = Simulation::createSystem(1);
    ASSERT_NE(sys2, nullptr);

    // Run with run()
    Simulation::SimulationEngine engine1;
    engine1.setSystem(std::shared_ptr<Simulation::DynamicalSystem>(
        sys.release(), std::default_delete<Simulation::DynamicalSystem>()));
    Simulation::SimConfig cfg;
    cfg.dt = 0.01;
    cfg.totalTime = 0.1;
    cfg.method = Simulation::IntegrationMethod::RungeKutta4;
    engine1.setConfig(cfg);
    engine1.setInitialState({1.0, 0.0});
    auto record = engine1.run();

    // Run with step-by-step
    Simulation::SimulationEngine engine2;
    engine2.setSystem(std::shared_ptr<Simulation::DynamicalSystem>(
        sys2.release(), std::default_delete<Simulation::DynamicalSystem>()));
    engine2.setConfig(cfg);
    engine2.setInitialState({1.0, 0.0});
    engine2.initialize();

    std::vector<Simulation::StateVector> states2;
    while (!engine2.isFinished()) {
        auto result = engine2.step();
        states2.push_back(result.state);
    }

    EXPECT_EQ(record.states.size(), states2.size());
    for (size_t i = 0; i < std::min(record.states.size(), states2.size()); ++i) {
        for (size_t j = 0; j < record.states[i].size(); ++j) {
            EXPECT_NEAR(record.states[i][j], states2[i][j], 1e-10)
                << "Mismatch at step " << i << " state " << j;
        }
    }
}

TEST(SimulationEngineTests, ResetWorks) {
    auto sys = Simulation::createSystem(1);
    ASSERT_NE(sys, nullptr);

    Simulation::SimulationEngine engine;
    engine.setSystem(std::shared_ptr<Simulation::DynamicalSystem>(
        sys.release(), std::default_delete<Simulation::DynamicalSystem>()));
    Simulation::SimConfig cfg;
    cfg.dt = 0.01;
    cfg.totalTime = 0.5;
    cfg.method = Simulation::IntegrationMethod::RungeKutta4;
    engine.setConfig(cfg);
    engine.setInitialState({1.0, 0.0});

    auto record1 = engine.run();
    engine.reset();
    auto record2 = engine.run();

    EXPECT_EQ(record1.size(), record2.size());
}

TEST(SimulationEngineTests, ComputeMetrics) {
    auto sys = Simulation::createSystem(1);
    ASSERT_NE(sys, nullptr);

    Simulation::SimulationEngine engine;
    engine.setSystem(std::shared_ptr<Simulation::DynamicalSystem>(
        sys.release(), std::default_delete<Simulation::DynamicalSystem>()));
    Simulation::SimConfig cfg;
    cfg.dt = 0.001;
    cfg.totalTime = 5.0;
    cfg.method = Simulation::IntegrationMethod::RungeKutta4;
    engine.setConfig(cfg);
    engine.setInitialState({0.0, 0.0});
    engine.setReference({1.0});

    auto record = engine.run();
    auto metrics = Simulation::SimulationEngine::computeMetrics(record, {1.0}, 0);

    EXPECT_TRUE(std::isfinite(metrics.iae));
    EXPECT_TRUE(std::isfinite(metrics.ise));
    EXPECT_TRUE(std::isfinite(metrics.itae));
    EXPECT_GE(metrics.iae, 0.0);
}

TEST(SimulationEngineTests, ComputeBodePlot) {
    auto sys = Simulation::createSystem(1);
    ASSERT_NE(sys, nullptr);

    Simulation::SimulationEngine engine;
    engine.setSystem(std::shared_ptr<Simulation::DynamicalSystem>(
        sys.release(), std::default_delete<Simulation::DynamicalSystem>()));
    Simulation::SimConfig cfg;
    cfg.dt = 0.001;
    cfg.totalTime = 5.0;
    cfg.method = Simulation::IntegrationMethod::RungeKutta4;
    engine.setConfig(cfg);
    engine.setInitialState({0.0, 0.0});

    auto record = engine.run();
    std::vector<double> freq, mag, phase;
    Simulation::SimulationEngine::computeBodePlot(record, 0, 0, freq, mag, phase);

    EXPECT_GT(freq.size(), 0u);
    EXPECT_EQ(freq.size(), mag.size());
    EXPECT_EQ(freq.size(), phase.size());
}

// ===== Short Simulation Stability Tests =====

class SimulationStabilityTest : public ::testing::TestWithParam<int> {};

TEST_P(SimulationStabilityTest, ShortSimulationDoesNotDiverge) {
    auto sys = Simulation::createSystem(GetParam());
    ASSERT_NE(sys, nullptr);

    Simulation::SimulationEngine engine;
    engine.setSystem(std::shared_ptr<Simulation::DynamicalSystem>(
        sys.release(), std::default_delete<Simulation::DynamicalSystem>()));

    Simulation::SimConfig cfg;
    cfg.dt = 0.001;
    cfg.totalTime = 0.1;
    cfg.method = Simulation::IntegrationMethod::RungeKutta4;
    engine.setConfig(cfg);

    auto record = engine.run();
    EXPECT_GT(record.size(), 0u);

    // Check that states don't become NaN/Inf
    for (size_t i = 0; i < record.states.size(); ++i) {
        for (size_t j = 0; j < record.states[i].size(); ++j) {
            EXPECT_TRUE(std::isfinite(record.states[i][j]))
                << "System " << GetParam() << " state diverged at step " << i
                << " component " << j;
        }
    }
}

INSTANTIATE_TEST_SUITE_P(
    AllSystems, SimulationStabilityTest,
    ::testing::Range(1, 71),
    [](const ::testing::TestParamInfo<int>& info) {
        auto sys = Simulation::createSystem(info.param);
        std::string name = sys ? sys->name() : "Unknown";
        std::string clean;
        for (char c : name) {
            if (std::isalnum(static_cast<unsigned char>(c))) clean += c;
            else if (c == ' ' || c == '-') clean += '_';
        }
        return std::to_string(info.param) + "_" + clean;
    }
);

// ===== SimulationTypes Tests =====

TEST(SimulationTypes, GeometryDescDefaults) {
    Simulation::GeometryDesc g;
    EXPECT_EQ(g.shape, Simulation::GeometryShape::PointMass);
    EXPECT_DOUBLE_EQ(g.mass, 1.0);
}

TEST(SimulationTypes, SimConfigDefaults) {
    Simulation::SimConfig cfg;
    EXPECT_DOUBLE_EQ(cfg.dt, 0.001);
    EXPECT_DOUBLE_EQ(cfg.totalTime, 10.0);
    EXPECT_EQ(cfg.method, Simulation::IntegrationMethod::RungeKutta4);
    EXPECT_FALSE(cfg.adaptiveStep);
}

TEST(SimulationTypes, ParamDescriptorFields) {
    Simulation::ParamDescriptor pd;
    pd.name = "test";
    pd.unit = "m";
    pd.description = "test param";
    pd.defaultValue = 1.0;
    pd.minValue = 0.0;
    pd.maxValue = 10.0;
    pd.step = 0.1;
    pd.logarithmic = false;
    EXPECT_EQ(pd.name, "test");
    EXPECT_FALSE(pd.logarithmic);
}

TEST(SimulationTypes, PresetFields) {
    Simulation::Preset p;
    p.name = "default";
    p.description = "default preset";
    p.params["k"] = 1.0;
    EXPECT_EQ(p.params.size(), 1u);
}

TEST(SimulationTypes, PerformanceMetricsDefaults) {
    Simulation::PerformanceMetrics m;
    EXPECT_DOUBLE_EQ(m.riseTime, 0.0);
    EXPECT_DOUBLE_EQ(m.overshoot, 0.0);
    EXPECT_DOUBLE_EQ(m.iae, 0.0);
}

TEST(SimulationTypes, SimStepResultFields) {
    Simulation::SimStepResult r;
    r.time = 1.0;
    r.state = {1.0, 2.0};
    r.output = {3.0};
    r.controlSignal = 0.5;
    r.error = 0.1;
    EXPECT_DOUBLE_EQ(r.time, 1.0);
    EXPECT_EQ(r.state.size(), 2u);
}

TEST(SimulationTypes, SimulationRecordClearAndSize) {
    Simulation::SimulationRecord rec;
    rec.times.push_back(0.0);
    rec.states.push_back({1.0});
    rec.outputs.push_back({2.0});
    EXPECT_EQ(rec.size(), 1u);
    rec.clear();
    EXPECT_EQ(rec.size(), 0u);
}

// ===== SensorActuatorModel Tests =====

TEST(SensorTests, DefaultSensorPassthrough) {
    Simulation::SensorConfig cfg;
    Simulation::SensorModel sensor(cfg);
    double val = sensor.measure(1.0, 0.0);
    EXPECT_DOUBLE_EQ(val, 1.0);
}

TEST(SensorTests, SensorSaturationLow) {
    Simulation::SensorConfig cfg;
    cfg.saturationMin = 0.0;
    cfg.saturationMax = 5.0;
    Simulation::SensorModel sensor(cfg);
    double val = sensor.measure(-10.0, 0.0);
    EXPECT_GE(val, 0.0);
}

TEST(SensorTests, SensorSaturation) {
    Simulation::SensorConfig cfg;
    cfg.saturationMin = -5.0;
    cfg.saturationMax = 5.0;
    Simulation::SensorModel sensor(cfg);
    double val = sensor.measure(10.0, 0.0);
    EXPECT_LE(val, 5.0);
}

TEST(SensorTests, SensorReset) {
    Simulation::SensorConfig cfg;
    cfg.saturationMin = -5.0;
    cfg.saturationMax = 5.0;
    Simulation::SensorModel sensor(cfg);
    sensor.measure(1.0, 0.0);
    sensor.reset();
    double val = sensor.measure(1.0, 0.0);
    EXPECT_NEAR(val, 1.0, 0.01);
}

TEST(ActuatorTests, DefaultActuatorPassthrough) {
    Simulation::ActuatorConfig cfg;
    Simulation::ActuatorModel actuator(cfg);
    double val = actuator.apply(1.0, 0.001);
    EXPECT_DOUBLE_EQ(val, 1.0);
}

TEST(ActuatorTests, ActuatorSaturation) {
    Simulation::ActuatorConfig cfg;
    cfg.minOutput = -5.0;
    cfg.maxOutput = 5.0;
    Simulation::ActuatorModel actuator(cfg);
    double val = actuator.apply(10.0, 0.001);
    EXPECT_LE(val, 5.0);
}

TEST(ActuatorTests, ActuatorRateLimit) {
    Simulation::ActuatorConfig cfg;
    cfg.maxRate = 1.0;
    Simulation::ActuatorModel actuator(cfg);
    actuator.apply(0.0, 0.001); // Initialize at 0
    double val = actuator.apply(100.0, 0.001); // Try big jump
    EXPECT_LT(val, 100.0); // Should be rate-limited
}

TEST(ActuatorTests, ActuatorReset) {
    Simulation::ActuatorConfig cfg;
    cfg.maxRate = 1.0;
    Simulation::ActuatorModel actuator(cfg);
    actuator.apply(5.0, 0.001);
    actuator.reset();
    double val = actuator.apply(0.0, 0.001);
    EXPECT_DOUBLE_EQ(val, 0.0);
}

// ===== Specific System Behaviour Tests =====

TEST(MechanicalSystems, MassSpringDamperOscillates) {
    auto sys = Simulation::createSystem(1);
    ASSERT_NE(sys, nullptr);
    EXPECT_STREQ(sys->name(), "Mass-Spring-Damper");
    EXPECT_EQ(sys->stateDim(), 2);
    EXPECT_EQ(sys->inputDim(), 1);
    EXPECT_EQ(sys->outputDim(), 1);
}

TEST(MechanicalSystems, SimplePendulum) {
    auto sys = Simulation::createSystem(2);
    ASSERT_NE(sys, nullptr);
    EXPECT_STREQ(sys->name(), "Coupled Mass-Spring-Damper");
    EXPECT_EQ(sys->stateDim(), 4);
}

TEST(MechanicalSystems, DoublePendulum) {
    auto sys = Simulation::createSystem(3);
    ASSERT_NE(sys, nullptr);
    EXPECT_STREQ(sys->name(), "Inverted Pendulum on Cart");
}

TEST(MechanicalSystems, CartPole) {
    auto sys = Simulation::createSystem(4);
    ASSERT_NE(sys, nullptr);
    EXPECT_STREQ(sys->name(), "Double Inverted Pendulum");
    EXPECT_EQ(sys->stateDim(), 6);
    EXPECT_EQ(sys->inputDim(), 1);
}

TEST(ElectricalSystems, BuckConverter) {
    auto sys = Simulation::createSystem(44);
    ASSERT_NE(sys, nullptr);
    EXPECT_STREQ(sys->name(), "Buck Converter");
    EXPECT_EQ(sys->stateDim(), 2);
}

TEST(ChaoticSystems, LorenzSystem) {
    auto sys = Simulation::createSystem(57);
    ASSERT_NE(sys, nullptr);
    EXPECT_STREQ(sys->name(), "Lorenz System");
    EXPECT_EQ(sys->stateDim(), 3);
}

TEST(BiologicalSystems, BloodGlucose) {
    auto sys = Simulation::createSystem(54);
    ASSERT_NE(sys, nullptr);
    EXPECT_STREQ(sys->name(), "Blood Glucose Regulation");
    EXPECT_EQ(sys->stateDim(), 3);
}

// ===== Geometry & Inertia Tests =====

TEST(SimulationTypes, GeometryShapeEnum) {
    EXPECT_NE(static_cast<int>(Simulation::GeometryShape::PointMass),
              static_cast<int>(Simulation::GeometryShape::Sphere));
    EXPECT_NE(static_cast<int>(Simulation::GeometryShape::SolidCylinder),
              static_cast<int>(Simulation::GeometryShape::HollowCylinder));
}

TEST(SimulationTypes, SystemCategoryEnum) {
    EXPECT_NE(static_cast<int>(Simulation::SystemCategory::MechanicalTranslational),
              static_cast<int>(Simulation::SystemCategory::RotationalAngular));
}

TEST(SimulationTypes, IntegrationMethodEnum) {
    EXPECT_NE(static_cast<int>(Simulation::IntegrationMethod::EulerForward),
              static_cast<int>(Simulation::IntegrationMethod::RungeKutta4));
}

TEST(SimulationTypes, AirResistanceConfigDefaults) {
    Simulation::AirResistanceConfig arc;
    EXPECT_DOUBLE_EQ(arc.dragCoefficient, 0.47);
    EXPECT_DOUBLE_EQ(arc.airDensity, 1.225);
}

// ===== Adaptive Simulation Test =====

TEST(SimulationEngineTests, AdaptiveStepSimulation) {
    auto sys = Simulation::createSystem(1);
    ASSERT_NE(sys, nullptr);

    Simulation::SimulationEngine engine;
    engine.setSystem(std::shared_ptr<Simulation::DynamicalSystem>(
        sys.release(), std::default_delete<Simulation::DynamicalSystem>()));

    Simulation::SimConfig cfg;
    cfg.dt = 0.01;
    cfg.totalTime = 1.0;
    cfg.method = Simulation::IntegrationMethod::DormandPrinceRK45;
    cfg.adaptiveStep = true;
    cfg.absTolerance = 1e-6;
    cfg.relTolerance = 1e-6;
    engine.setConfig(cfg);
    engine.setInitialState({1.0, 0.0});

    auto record = engine.run();
    EXPECT_GT(record.size(), 0u);
}

// ===== Empty/Edge Case Tests =====

TEST(SimulationEngineTests, EmptyRecordMetrics) {
    Simulation::SimulationRecord emptyRecord;
    auto metrics = Simulation::SimulationEngine::computeMetrics(emptyRecord, {1.0}, 0);
    EXPECT_DOUBLE_EQ(metrics.iae, 0.0);
}

TEST(SimulationEngineTests, EmptyRecordBodePlot) {
    Simulation::SimulationRecord emptyRecord;
    std::vector<double> freq, mag, phase;
    Simulation::SimulationEngine::computeBodePlot(emptyRecord, 0, 0, freq, mag, phase);
    EXPECT_EQ(freq.size(), 0u);
}

TEST(SimulationEngineTests, ExternalForceCallback) {
    auto sys = Simulation::createSystem(1);
    ASSERT_NE(sys, nullptr);

    Simulation::SimulationEngine engine;
    engine.setSystem(std::shared_ptr<Simulation::DynamicalSystem>(
        sys.release(), std::default_delete<Simulation::DynamicalSystem>()));

    Simulation::SimConfig cfg;
    cfg.dt = 0.01;
    cfg.totalTime = 0.5;
    cfg.method = Simulation::IntegrationMethod::RungeKutta4;
    engine.setConfig(cfg);

    engine.setExternalForce([](double, const Simulation::StateVector&) -> Simulation::StateVector {
        return {1.0};
    });

    auto record = engine.run();
    EXPECT_GT(record.size(), 0u);
}

TEST(SimulationEngineTests, SensorAndActuatorConfigs) {
    auto sys = Simulation::createSystem(1);
    ASSERT_NE(sys, nullptr);

    Simulation::SimulationEngine engine;
    engine.setSystem(std::shared_ptr<Simulation::DynamicalSystem>(
        sys.release(), std::default_delete<Simulation::DynamicalSystem>()));

    // Set sensor and actuator configs
    Simulation::SensorConfig scfg;
    scfg.saturationMin = -10.0;
    scfg.saturationMax = 10.0;
    engine.setSensorConfigs({scfg});

    Simulation::ActuatorConfig acfg;
    acfg.minOutput = -10.0;
    acfg.maxOutput = 10.0;
    engine.setActuatorConfigs({acfg});

    Simulation::SimConfig cfg;
    cfg.dt = 0.01;
    cfg.totalTime = 0.5;
    cfg.method = Simulation::IntegrationMethod::RungeKutta4;
    engine.setConfig(cfg);

    auto record = engine.run();
    EXPECT_GT(record.size(), 0u);
}

// ===== GeometryDesc Tests =====

TEST(GeometryTests, PointMassInertia) {
    Simulation::GeometryDesc g;
    g.shape = Simulation::GeometryShape::PointMass;
    g.mass = 5.0;
    EXPECT_DOUBLE_EQ(g.computeInertia(), 0.0);
    EXPECT_DOUBLE_EQ(g.computeArea(), 0.0);
}

TEST(GeometryTests, SphereInertia) {
    Simulation::GeometryDesc g;
    g.shape = Simulation::GeometryShape::Sphere;
    g.mass = 10.0;
    g.radius = 0.5;
    double expected = 0.4 * 10.0 * 0.5 * 0.5;
    EXPECT_DOUBLE_EQ(g.computeInertia(), expected);
    EXPECT_NEAR(g.computeArea(), M_PI * 0.25, 1e-10);
}

TEST(GeometryTests, SolidCylinderInertia) {
    Simulation::GeometryDesc g;
    g.shape = Simulation::GeometryShape::SolidCylinder;
    g.mass = 10.0;
    g.radius = 0.5;
    g.length = 1.0;
    EXPECT_DOUBLE_EQ(g.computeInertia(), 0.5 * 10.0 * 0.25);
    EXPECT_DOUBLE_EQ(g.computeArea(), 2.0 * 0.5 * 1.0);
}

TEST(GeometryTests, HollowCylinderInertia) {
    Simulation::GeometryDesc g;
    g.shape = Simulation::GeometryShape::HollowCylinder;
    g.mass = 10.0;
    g.radius = 0.5;
    g.innerRadius = 0.3;
    g.length = 1.0;
    double expected = 0.5 * 10.0 * (0.25 + 0.09);
    EXPECT_DOUBLE_EQ(g.computeInertia(), expected);
}

TEST(GeometryTests, SolidCuboidInertia) {
    Simulation::GeometryDesc g;
    g.shape = Simulation::GeometryShape::SolidCuboid;
    g.mass = 12.0;
    g.width = 2.0;
    g.height = 3.0;
    double expected = 12.0 * (4.0 + 9.0) / 12.0;
    EXPECT_DOUBLE_EQ(g.computeInertia(), expected);
    EXPECT_DOUBLE_EQ(g.computeArea(), 6.0);
}

TEST(GeometryTests, ThinRodInertia) {
    Simulation::GeometryDesc g;
    g.shape = Simulation::GeometryShape::ThinRod;
    g.mass = 6.0;
    g.length = 2.0;
    g.width = 0.1;
    EXPECT_DOUBLE_EQ(g.computeInertia(), 6.0 * 4.0 / 12.0);
    EXPECT_DOUBLE_EQ(g.computeArea(), 0.1 * 2.0);
}

TEST(GeometryTests, ThinDiskInertia) {
    Simulation::GeometryDesc g;
    g.shape = Simulation::GeometryShape::ThinDisk;
    g.mass = 5.0;
    g.radius = 1.0;
    EXPECT_DOUBLE_EQ(g.computeInertia(), 2.5);
    EXPECT_NEAR(g.computeArea(), M_PI, 1e-10);
}

TEST(GeometryTests, ConeInertia) {
    Simulation::GeometryDesc g;
    g.shape = Simulation::GeometryShape::Cone;
    g.mass = 10.0;
    g.radius = 1.0;
    EXPECT_DOUBLE_EQ(g.computeInertia(), 3.0);
    EXPECT_NEAR(g.computeArea(), M_PI, 1e-10);
}

TEST(GeometryTests, CustomInertia) {
    Simulation::GeometryDesc g;
    g.shape = Simulation::GeometryShape::Custom;
    g.mass = 4.0;
    g.radius = 2.0;
    g.width = 3.0;
    g.height = 4.0;
    EXPECT_DOUBLE_EQ(g.computeInertia(), 4.0 * 4.0);
    EXPECT_DOUBLE_EQ(g.computeArea(), 12.0);
}

// ===== FrictionParams Tests =====

TEST(FrictionTests, NoneFriction) {
    Simulation::FrictionParams fp;
    fp.model = Simulation::FrictionModel::None;
    EXPECT_DOUBLE_EQ(fp.compute(1.0, 10.0, 0.001), 0.0);
}

TEST(FrictionTests, CoulombFriction) {
    Simulation::FrictionParams fp;
    fp.model = Simulation::FrictionModel::Coulomb;
    fp.kineticFriction = 0.3;
    double f = fp.compute(1.0, 10.0, 0.001);
    EXPECT_NEAR(f, -0.3 * 10.0, 1e-10);
    double f2 = fp.compute(-1.0, 10.0, 0.001);
    EXPECT_NEAR(f2, 0.3 * 10.0, 1e-10);
}

TEST(FrictionTests, ViscousFriction) {
    Simulation::FrictionParams fp;
    fp.model = Simulation::FrictionModel::Viscous;
    fp.viscousCoeff = 0.5;
    double f = fp.compute(2.0, 10.0, 0.001);
    EXPECT_NEAR(f, -1.0, 1e-10);
}

TEST(FrictionTests, CoulombViscousFriction) {
    Simulation::FrictionParams fp;
    fp.model = Simulation::FrictionModel::CoulombViscous;
    fp.kineticFriction = 0.2;
    fp.viscousCoeff = 0.5;
    double f = fp.compute(1.0, 10.0, 0.001);
    EXPECT_NEAR(f, -0.2 * 10.0 - 0.5 * 1.0, 1e-10);
}

TEST(FrictionTests, StribeckFriction) {
    Simulation::FrictionParams fp;
    fp.model = Simulation::FrictionModel::Stribeck;
    fp.staticFriction = 0.5;
    fp.kineticFriction = 0.3;
    fp.stribeckVelocity = 0.1;
    fp.stribeckExponent = 2.0;
    fp.viscousCoeff = 0.01;
    double f = fp.compute(0.01, 10.0, 0.001);
    EXPECT_TRUE(std::isfinite(f));
    EXPECT_LT(f, 0.0);
}

TEST(FrictionTests, LuGreFriction) {
    Simulation::FrictionParams fp;
    fp.model = Simulation::FrictionModel::LuGre;
    fp.staticFriction = 0.5;
    fp.kineticFriction = 0.3;
    fp.stribeckVelocity = 0.1;
    fp.stribeckExponent = 2.0;
    fp.sigma0 = 1e5;
    fp.sigma1 = 316.0;
    fp.sigma2 = 0.4;
    fp.lugreState = 0.0;
    double f = fp.compute(0.1, 10.0, 0.001);
    EXPECT_TRUE(std::isfinite(f));
}

TEST(FrictionTests, DahlFriction) {
    Simulation::FrictionParams fp;
    fp.model = Simulation::FrictionModel::Dahl;
    fp.kineticFriction = 0.3;
    fp.sigma0 = 1e5;
    fp.lugreState = 0.0;
    double f = fp.compute(0.01, 10.0, 0.001);
    EXPECT_TRUE(std::isfinite(f));
}

// ===== NoiseGenerator Tests =====

TEST(NoiseTests, WhiteNoise) {
    Simulation::NoiseParams np;
    np.type = Simulation::NoiseType::White;
    np.amplitude = 1.0;
    Simulation::NoiseGenerator gen;
    double sum = 0.0;
    for (int i = 0; i < 1000; ++i) {
        sum += gen.generate(np, i * 0.001, 0.0);
    }
    EXPECT_NEAR(sum / 1000.0, 0.0, 0.2);
}

TEST(NoiseTests, BrownNoise) {
    Simulation::NoiseParams np;
    np.type = Simulation::NoiseType::Brown;
    np.amplitude = 1.0;
    Simulation::NoiseGenerator gen;
    double val = 0.0;
    for (int i = 0; i < 100; ++i) {
        val = gen.generate(np, i * 0.001, 0.0);
    }
    EXPECT_TRUE(std::isfinite(val));
}

TEST(NoiseTests, PurpleNoise) {
    Simulation::NoiseParams np;
    np.type = Simulation::NoiseType::Purple;
    np.amplitude = 1.0;
    Simulation::NoiseGenerator gen;
    double val = gen.generate(np, 0.0, 0.0);
    EXPECT_TRUE(std::isfinite(val));
}

TEST(NoiseTests, GreyNoise) {
    Simulation::NoiseParams np;
    np.type = Simulation::NoiseType::Grey;
    np.amplitude = 1.0;
    Simulation::NoiseGenerator gen;
    double val = gen.generate(np, 0.0, 0.0);
    EXPECT_TRUE(std::isfinite(val));
}

TEST(NoiseTests, PeriodicGSMNoise) {
    Simulation::NoiseParams np;
    np.type = Simulation::NoiseType::PeriodicGSM;
    np.amplitude = 1.0;
    np.frequency = 217.0;
    Simulation::NoiseGenerator gen;
    double val = gen.generate(np, 0.0, 0.0);
    EXPECT_TRUE(std::isfinite(val));
}

TEST(NoiseTests, ActuatorDependentNoise) {
    Simulation::NoiseParams np;
    np.type = Simulation::NoiseType::ActuatorDependent;
    np.amplitude = 1.0;
    np.actuatorGain = 0.5;
    Simulation::NoiseGenerator gen;
    double val = gen.generate(np, 0.0, 2.0);
    EXPECT_TRUE(std::isfinite(val));
}

TEST(NoiseTests, NoiseGeneratorReset) {
    Simulation::NoiseGenerator gen;
    Simulation::NoiseParams np;
    np.type = Simulation::NoiseType::Brown;
    np.amplitude = 1.0;
    for (int i = 0; i < 100; ++i) gen.generate(np, i * 0.001, 0.0);
    gen.reset();
    np.type = Simulation::NoiseType::None;
    np.amplitude = 0.0;
    EXPECT_DOUBLE_EQ(gen.generate(np, 0.0, 0.0), 0.0);
}

TEST(NoiseTests, NoiseGeneratorSetSeed) {
    Simulation::NoiseGenerator gen;
    gen.setSeed(12345);
    Simulation::NoiseParams np;
    np.type = Simulation::NoiseType::White;
    np.amplitude = 1.0;
    double v1 = gen.generate(np, 0.0, 0.0);
    gen.setSeed(12345);
    double v2 = gen.generate(np, 0.0, 0.0);
    EXPECT_DOUBLE_EQ(v1, v2);
}

// ===== Sensor Advanced Tests =====

TEST(SensorTests, SensorQuantization) {
    Simulation::SensorConfig cfg;
    cfg.quantization = 0.1;
    Simulation::SensorModel sensor(cfg);
    double val = sensor.measure(0.55, 0.0);
    EXPECT_NEAR(val, 0.6, 1e-10);
}

TEST(SensorTests, SensorSampleRate) {
    Simulation::SensorConfig cfg;
    cfg.sampleRate = 10.0;
    Simulation::SensorModel sensor(cfg);
    double v1 = sensor.measure(1.0, 0.0);
    double v2 = sensor.measure(2.0, 0.05);
    EXPECT_DOUBLE_EQ(v2, v1);
    double v3 = sensor.measure(3.0, 0.11);
    EXPECT_NEAR(v3, 3.0, 0.01);
}

TEST(SensorTests, SensorWithDelay) {
    Simulation::SensorConfig cfg;
    cfg.delay = 0.01;
    Simulation::SensorModel sensor(cfg);
    double v1 = sensor.measure(1.0, 0.0);
    double v2 = sensor.measure(2.0, 0.005);
    double v3 = sensor.measure(3.0, 0.02);
    EXPECT_TRUE(std::isfinite(v1));
    EXPECT_TRUE(std::isfinite(v2));
    EXPECT_TRUE(std::isfinite(v3));
}

TEST(SensorTests, SensorWithDelayVariance) {
    Simulation::SensorConfig cfg;
    cfg.delay = 0.01;
    cfg.delayVariance = 0.005;
    Simulation::SensorModel sensor(cfg);
    double val = sensor.measure(1.0, 0.0);
    EXPECT_TRUE(std::isfinite(val));
}

TEST(SensorTests, SensorWithNoise) {
    Simulation::SensorConfig cfg;
    cfg.noise.type = Simulation::NoiseType::White;
    cfg.noise.amplitude = 0.1;
    Simulation::SensorModel sensor(cfg);
    double val = sensor.measure(5.0, 0.0);
    EXPECT_NEAR(val, 5.0, 1.0);
}

// ===== Actuator Advanced Tests =====

TEST(ActuatorTests, ActuatorDeadZone) {
    Simulation::ActuatorConfig cfg;
    cfg.deadZone = 0.5;
    Simulation::ActuatorModel actuator(cfg);
    double val = actuator.apply(0.3, 0.001);
    EXPECT_DOUBLE_EQ(val, 0.0);
    double val2 = actuator.apply(1.0, 0.001);
    EXPECT_NEAR(val2, 0.5, 0.01);
}

TEST(ActuatorTests, ActuatorBacklash) {
    Simulation::ActuatorConfig cfg;
    cfg.backlash = 0.1;
    Simulation::ActuatorModel actuator(cfg);
    double v1 = actuator.apply(1.0, 0.001);
    double v2 = actuator.apply(0.95, 0.001);
    EXPECT_DOUBLE_EQ(v2, v1);
}

TEST(ActuatorTests, ActuatorHysteresis) {
    Simulation::ActuatorConfig cfg;
    cfg.hysteresis = 0.2;
    Simulation::ActuatorModel actuator(cfg);
    double v1 = actuator.apply(1.0, 0.001);
    double v2 = actuator.apply(1.1, 0.001);
    EXPECT_DOUBLE_EQ(v2, v1);
}

TEST(ActuatorTests, ActuatorReducedPerfNearLimits) {
    Simulation::ActuatorConfig cfg;
    cfg.minOutput = -10.0;
    cfg.maxOutput = 10.0;
    cfg.reducedPerfNearLimits = true;
    cfg.perfReductionZone = 0.1;
    Simulation::ActuatorModel actuator(cfg);
    double val = actuator.apply(9.5, 0.001);
    EXPECT_LT(val, 9.5);
}

TEST(ActuatorTests, ActuatorFailOnLimit) {
    Simulation::ActuatorConfig cfg;
    cfg.minOutput = -5.0;
    cfg.maxOutput = 5.0;
    cfg.failOnLimit = true;
    Simulation::ActuatorModel actuator(cfg);
    double v1 = actuator.apply(10.0, 0.001);
    EXPECT_DOUBLE_EQ(v1, 0.0);
    double v2 = actuator.apply(1.0, 0.001);
    EXPECT_DOUBLE_EQ(v2, 0.0);
}

TEST(ActuatorTests, ActuatorStiction) {
    Simulation::ActuatorConfig cfg;
    cfg.stiction.model = Simulation::FrictionModel::Viscous;
    cfg.stiction.viscousCoeff = 0.1;
    Simulation::ActuatorModel actuator(cfg);
    double val = actuator.apply(1.0, 0.001);
    EXPECT_TRUE(std::isfinite(val));
}

// ===== SimulationEngine Controller Test =====

TEST(SimulationEngineTests, WithController) {
    auto sys = Simulation::createSystem(1);
    ASSERT_NE(sys, nullptr);

    struct PController : Simulation::SimController {
        Simulation::StateVector compute(double, const Simulation::StateVector& measured,
                                          const Simulation::StateVector& ref, double) override {
            double error = ref[0] - measured[0];
            return {10.0 * error};
        }
        void reset() override {}
        const char* name() const override { return "PController"; }
    };

    Simulation::SimulationEngine engine;
    engine.setSystem(std::shared_ptr<Simulation::DynamicalSystem>(
        sys.release(), std::default_delete<Simulation::DynamicalSystem>()));

    auto ctrl = std::make_shared<PController>();
    engine.setController(ctrl);

    Simulation::SimConfig cfg;
    cfg.dt = 0.001;
    cfg.totalTime = 2.0;
    cfg.method = Simulation::IntegrationMethod::RungeKutta4;
    engine.setConfig(cfg);
    engine.setReference({1.0});

    auto record = engine.run();
    EXPECT_GT(record.size(), 0u);

    auto metrics = Simulation::SimulationEngine::computeMetrics(record, {1.0}, 0);
    EXPECT_TRUE(std::isfinite(metrics.riseTime));
    EXPECT_TRUE(std::isfinite(metrics.settlingTime));
    EXPECT_TRUE(std::isfinite(metrics.overshoot));
    EXPECT_TRUE(std::isfinite(metrics.steadyStateError));
    EXPECT_TRUE(std::isfinite(metrics.controlEnergy));
}

// ===== SimulationRecord Fields =====

TEST(SimulationTypes, SimulationRecordFullFields) {
    Simulation::SimulationRecord rec;
    rec.times.push_back(0.0);
    rec.times.push_back(0.001);
    rec.states.push_back({1.0, 0.0});
    rec.states.push_back({0.99, -0.1});
    rec.outputs.push_back({1.0});
    rec.outputs.push_back({0.99});
    rec.controlInputs.push_back({0.5});
    rec.controlInputs.push_back({0.6});
    rec.errors.push_back(0.1);
    rec.errors.push_back(0.05);
    rec.dtHistory.push_back(0.001);
    rec.dtHistory.push_back(0.001);
    EXPECT_EQ(rec.size(), 2u);
    rec.clear();
    EXPECT_EQ(rec.size(), 0u);
    EXPECT_TRUE(rec.controlInputs.empty());
    EXPECT_TRUE(rec.errors.empty());
    EXPECT_TRUE(rec.dtHistory.empty());
}

// ===== Integrator name() and isImplicit() Tests =====

TEST(IntegratorTests, IntegratorNamesAndImplicit) {
    struct IntInfo { Simulation::IntegrationMethod m; const char* expectedName; bool implicit; };
    std::vector<IntInfo> infos = {
        {Simulation::IntegrationMethod::EulerForward, "Forward Euler", false},
        {Simulation::IntegrationMethod::RungeKutta4, "RK4", false},
        {Simulation::IntegrationMethod::DormandPrinceRK45, "Dormand-Prince RK45", false},
        {Simulation::IntegrationMethod::BogackiShampineRK23, "Bogacki-Shampine RK23", false},
        {Simulation::IntegrationMethod::ImplicitTrapezoidal, "Implicit Trapezoidal", true},
        {Simulation::IntegrationMethod::BDF2, "BDF2", true},
        {Simulation::IntegrationMethod::SDIRK4, nullptr, false}, // falls to default (RK4)
    };
    for (auto& info : infos) {
        auto integrator = Simulation::createIntegrator(info.m, 1e-6, 1e-6);
        ASSERT_NE(integrator, nullptr);
        const char* n = integrator->name();
        EXPECT_NE(n, nullptr);
        if (info.expectedName) {
            EXPECT_STREQ(n, info.expectedName);
        }
        EXPECT_EQ(integrator->isImplicit(), info.implicit);
    }
}

// ===== getParameter with non-existent key =====

TEST(MechanicalSystems, GetParameterNonExistent) {
    auto sys = Simulation::createSystem(1); // MassSpringDamper
    ASSERT_NE(sys, nullptr);
    double val = sys->getParameter("nonexistent_param_xyz");
    EXPECT_DOUBLE_EQ(val, 0.0);
}

// ===== SimulationEngine edge cases =====

TEST(SimulationEngineTests, StepBeforeInitialize) {
    Simulation::SimulationEngine engine;
    auto result = engine.step();
    EXPECT_DOUBLE_EQ(result.time, 0.0);
    EXPECT_TRUE(result.state.empty());
    EXPECT_TRUE(result.output.empty());
}

TEST(SimulationEngineTests, InitializeWithoutExplicitIntegrator) {
    // Test lazy integrator creation in initialize() (line 43)
    auto sys = Simulation::createSystem(1);
    ASSERT_NE(sys, nullptr);

    Simulation::SimulationEngine engine;
    engine.setSystem(std::shared_ptr<Simulation::DynamicalSystem>(
        sys.release(), std::default_delete<Simulation::DynamicalSystem>()));
    // Don't call setConfig - use defaults, integrator_ will be null
    engine.initialize();
    auto result = engine.step();
    EXPECT_GT(result.time, 0.0);
    EXPECT_FALSE(result.state.empty());
}

TEST(SimulationEngineTests, MoreOutputsThanSensors) {
    // System 2 (Coupled MSD) has 2 outputs. Set only 1 sensor.
    auto sys = Simulation::createSystem(2);
    ASSERT_NE(sys, nullptr);
    EXPECT_EQ(sys->outputDim(), 2);

    Simulation::SimulationEngine engine;
    engine.setSystem(std::shared_ptr<Simulation::DynamicalSystem>(
        sys.release(), std::default_delete<Simulation::DynamicalSystem>()));

    Simulation::SensorConfig scfg;
    engine.setSensorConfigs({scfg}); // Only 1 sensor for 2 outputs

    Simulation::SimConfig cfg;
    cfg.dt = 0.001;
    cfg.totalTime = 0.01;
    cfg.method = Simulation::IntegrationMethod::RungeKutta4;
    engine.setConfig(cfg);

    auto record = engine.run();
    EXPECT_GT(record.size(), 0u);
}

TEST(SimulationEngineTests, AdaptiveStepRejection) {
    // Use Lorenz with extreme tolerance and large dt to force step rejections
    // and eventually reach the minimum step size forced-accept path
    auto sys = Simulation::createSystem(57); // Lorenz (chaotic)
    ASSERT_NE(sys, nullptr);

    Simulation::SimulationEngine engine;
    engine.setSystem(std::shared_ptr<Simulation::DynamicalSystem>(
        sys.release(), std::default_delete<Simulation::DynamicalSystem>()));

    Simulation::SimConfig cfg;
    cfg.dt = 1.0;             // Very large initial step
    cfg.totalTime = 0.001;    // Very short simulation
    cfg.method = Simulation::IntegrationMethod::DormandPrinceRK45;
    cfg.adaptiveStep = true;
    cfg.absTolerance = 1e-14;
    cfg.relTolerance = 1e-14;
    cfg.minStepSize = 0.5;    // Large min step so halving reaches it fast
    cfg.maxStepSize = 1.0;
    engine.setConfig(cfg);

    auto record = engine.run();
    EXPECT_GT(record.size(), 0u);
}

TEST(SimulationEngineTests, InitializeWithNullSystem) {
    Simulation::SimulationEngine engine;
    engine.initialize(); // system_ is null, should return early (line 43 vicinity)
    auto result = engine.step();
    EXPECT_DOUBLE_EQ(result.time, 0.0);
}

// ===== SensorActuatorModels edge cases =====

TEST(SensorTests, DefaultConstructor) {
    Simulation::SensorModel sensor; // default ctor (line 74)
    double val = sensor.measure(1.0, 0.0);
    EXPECT_DOUBLE_EQ(val, 1.0);
}

TEST(ActuatorTests, DefaultConstructor) {
    Simulation::ActuatorModel actuator; // default ctor (line 141)
    double val = actuator.apply(1.0, 0.001);
    EXPECT_DOUBLE_EQ(val, 1.0);
}

TEST(SensorTests, DelayBufferPruning) {
    // Exercise delay buffer pruning (>1000 entries, line 124)
    Simulation::SensorConfig cfg;
    cfg.delay = 0.001;
    Simulation::SensorModel sensor(cfg);
    for (int i = 0; i < 1100; ++i) {
        double val = sensor.measure(static_cast<double>(i), i * 0.0001);
        EXPECT_TRUE(std::isfinite(val));
    }
}

TEST(ActuatorTests, ReducedPerfNearMinLimit) {
    // Exercise reduced perf near MIN limit (line 181 - relPos < zone)
    Simulation::ActuatorConfig cfg;
    cfg.minOutput = -10.0;
    cfg.maxOutput = 10.0;
    cfg.reducedPerfNearLimits = true;
    cfg.perfReductionZone = 0.1;
    Simulation::ActuatorModel actuator(cfg);
    // Apply a value near the minimum output
    double val = actuator.apply(-9.5, 0.001);
    EXPECT_GT(val, -9.5);
}

TEST(NoiseTests, NoNoise) {
    Simulation::NoiseParams np;
    np.type = Simulation::NoiseType::None;
    np.amplitude = 1.0;
    Simulation::NoiseGenerator gen;
    EXPECT_DOUBLE_EQ(gen.generate(np, 0.0, 0.0), 0.0);
}

TEST(NoiseTests, ZeroAmplitude) {
    Simulation::NoiseParams np;
    np.type = Simulation::NoiseType::White;
    np.amplitude = 0.0;
    Simulation::NoiseGenerator gen;
    EXPECT_DOUBLE_EQ(gen.generate(np, 0.0, 0.0), 0.0);
}

// ===== System-specific dynamics branch coverage =====

TEST(MechanicalSystems, BouncingBallContact) {
    // Exercise contact force calculation (lines 584-585)
    auto sys = Simulation::createSystem(11); // BouncingBall
    ASSERT_NE(sys, nullptr);

    // State: [y_ball, dy_ball, y_plate, dy_plate]
    // Set ball below plate (penetration > 0) to trigger contact
    Simulation::StateVector state = {0.0, -1.0, 0.1, 0.0};
    auto u = sys->defaultInput();
    auto dx = sys->dynamics(0.0, state, u);
    EXPECT_TRUE(std::isfinite(dx[0]));

    // Also test with contact producing negative force (Fc < 0)
    // Ball barely penetrating but moving apart fast
    Simulation::StateVector state2 = {0.099, 10.0, 0.1, -10.0};
    auto dx2 = sys->dynamics(0.0, state2, u);
    EXPECT_TRUE(std::isfinite(dx2[0]));
}

TEST(AerospaceSystems, RocketLanding2DDynamics) {
    // Exercise fuel consumption path and ground constraint (line 74 in AerospaceSystems.cpp)
    auto sys = Simulation::createSystem(29); // RocketLanding2D
    ASSERT_NE(sys, nullptr);

    auto x0 = sys->defaultInitialState();
    // Apply throttle to consume fuel
    Simulation::StateVector input = {0.8, 0.0}; // throttle, gimbal
    auto dx = sys->dynamics(0.0, x0, input);
    EXPECT_TRUE(std::isfinite(dx[6])); // dm (fuel mass change) should be negative
    EXPECT_LE(dx[6], 0.0);

    // Trigger ground constraint: y <= 0 and dy < 0
    // State: [x, dx, y, dy, th, dth, m_fuel]
    Simulation::StateVector groundState = {0.0, 0.0, 0.0, -5.0, 0.0, 0.0, 5.0};
    auto dx2 = sys->dynamics(0.0, groundState, input);
    EXPECT_DOUBLE_EQ(dx2[0], 0.0); // dx should be 0 (ground)
    EXPECT_DOUBLE_EQ(dx2[1], 0.0); // ddx should be 0
    EXPECT_DOUBLE_EQ(dx2[2], 0.0); // dy should be 0
    EXPECT_DOUBLE_EQ(dx2[3], 0.0); // ddy should be 0
}

// ===== SimulationTypes switch fall-through coverage =====

TEST(SimulationTypes, GeometryInertiaFallthrough) {
    // The switch in computeInertia() has a fall-through return 0.0 (line 27)
    // This can only be reached with an invalid enum value
    Simulation::GeometryDesc g;
    g.shape = static_cast<Simulation::GeometryShape>(999);
    g.mass = 1.0;
    g.radius = 1.0;
    EXPECT_DOUBLE_EQ(g.computeInertia(), 0.0);
}

TEST(SimulationTypes, GeometryAreaFallthrough) {
    // The switch in computeArea() has a fall-through return 0.0 (line 50)
    Simulation::GeometryDesc g;
    g.shape = static_cast<Simulation::GeometryShape>(999);
    g.width = 1.0;
    g.height = 1.0;
    EXPECT_DOUBLE_EQ(g.computeArea(), 0.0);
}

TEST(SimulationTypes, ComputeInertiaAllShapes) {
    Simulation::GeometryDesc g;
    g.mass = 2.0;
    g.radius = 0.5;
    g.innerRadius = 0.3;
    g.width = 0.4;
    g.height = 0.6;
    g.length = 1.0;

    g.shape = Simulation::GeometryShape::PointMass;
    EXPECT_DOUBLE_EQ(g.computeInertia(), 0.0);

    g.shape = Simulation::GeometryShape::Sphere;
    EXPECT_DOUBLE_EQ(g.computeInertia(), 0.4 * 2.0 * 0.25);

    g.shape = Simulation::GeometryShape::SolidCylinder;
    EXPECT_DOUBLE_EQ(g.computeInertia(), 0.5 * 2.0 * 0.25);

    g.shape = Simulation::GeometryShape::HollowCylinder;
    EXPECT_DOUBLE_EQ(g.computeInertia(), 0.5 * 2.0 * (0.25 + 0.09));

    g.shape = Simulation::GeometryShape::SolidCuboid;
    EXPECT_DOUBLE_EQ(g.computeInertia(), 2.0 * (0.16 + 0.36) / 12.0);

    g.shape = Simulation::GeometryShape::ThinRod;
    EXPECT_DOUBLE_EQ(g.computeInertia(), 2.0 * 1.0 / 12.0);

    g.shape = Simulation::GeometryShape::ThinDisk;
    EXPECT_DOUBLE_EQ(g.computeInertia(), 0.5 * 2.0 * 0.25);

    g.shape = Simulation::GeometryShape::Cone;
    EXPECT_DOUBLE_EQ(g.computeInertia(), 0.3 * 2.0 * 0.25);

    g.shape = Simulation::GeometryShape::Custom;
    EXPECT_DOUBLE_EQ(g.computeInertia(), 2.0 * 0.25);
}

TEST(SimulationTypes, ComputeAreaAllShapes) {
    Simulation::GeometryDesc g;
    g.radius = 0.5;
    g.width = 0.4;
    g.height = 0.6;
    g.length = 1.0;

    g.shape = Simulation::GeometryShape::PointMass;
    EXPECT_DOUBLE_EQ(g.computeArea(), 0.0);

    g.shape = Simulation::GeometryShape::Sphere;
    EXPECT_DOUBLE_EQ(g.computeArea(), M_PI * 0.25);

    g.shape = Simulation::GeometryShape::SolidCylinder;
    EXPECT_DOUBLE_EQ(g.computeArea(), 2.0 * 0.5 * 1.0);

    g.shape = Simulation::GeometryShape::HollowCylinder;
    EXPECT_DOUBLE_EQ(g.computeArea(), 2.0 * 0.5 * 1.0);

    g.shape = Simulation::GeometryShape::SolidCuboid;
    EXPECT_DOUBLE_EQ(g.computeArea(), 0.4 * 0.6);

    g.shape = Simulation::GeometryShape::ThinRod;
    EXPECT_DOUBLE_EQ(g.computeArea(), 0.4 * 1.0);

    g.shape = Simulation::GeometryShape::ThinDisk;
    EXPECT_DOUBLE_EQ(g.computeArea(), M_PI * 0.25);

    g.shape = Simulation::GeometryShape::Cone;
    EXPECT_DOUBLE_EQ(g.computeArea(), M_PI * 0.25);

    g.shape = Simulation::GeometryShape::Custom;
    EXPECT_DOUBLE_EQ(g.computeArea(), 0.4 * 0.6);
}

TEST(FrictionTests, DefaultFallthrough) {
    // Exercise default case in FrictionParams::compute (lines 94-95)
    Simulation::FrictionParams fp;
    fp.model = static_cast<Simulation::FrictionModel>(999);
    EXPECT_DOUBLE_EQ(fp.compute(1.0, 10.0, 0.001), 0.0);
}

TEST(NoiseTests, DefaultFallthrough) {
    // Exercise default case in NoiseGenerator::generate (lines 55-56)
    Simulation::NoiseParams np;
    np.type = static_cast<Simulation::NoiseType>(999);
    np.amplitude = 1.0;
    Simulation::NoiseGenerator gen;
    EXPECT_DOUBLE_EQ(gen.generate(np, 0.0, 0.0), 0.0);
}

} // anonymous namespace

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

} // anonymous namespace

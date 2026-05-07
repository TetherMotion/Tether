#include <gtest/gtest.h>

#include "tether/simulation/AllSystems.hpp"
#include "tether/simulation/SimulationEngine.hpp"

#include <algorithm>
#include <cmath>
#include <memory>
#include <string>

namespace {

constexpr double kAmplitudeEpsilon = 1e-9;
constexpr double kHalfSecondWindow = 0.5;
constexpr double kTimeStep = 0.001;

double max_abs_value(const Simulation::StateVector& values) {
    double max_value = 0.0;
    for (double value : values) {
        max_value = std::max(max_value, std::abs(value));
    }
    return max_value;
}

bool has_nonzero_amplitude_through_half_second(const Simulation::SimulationRecord& record) {
    bool saw_window_end = false;
    for (size_t index = 0; index < record.times.size(); ++index) {
        const double time = record.times[index];
        if (time + (kTimeStep * 0.5) < kHalfSecondWindow) {
            continue;
        }

        saw_window_end = true;
        const double output_amplitude = index < record.outputs.size()
            ? max_abs_value(record.outputs[index])
            : 0.0;
        const double state_amplitude = index < record.states.size()
            ? max_abs_value(record.states[index])
            : 0.0;

        if (std::max(output_amplitude, state_amplitude) > kAmplitudeEpsilon) {
            return true;
        }
    }

    return !saw_window_end ? false : false;
}

std::shared_ptr<Simulation::DynamicalSystem> make_shared_system(int system_id) {
    auto system = Simulation::createSystem(system_id);
    if (!system) {
        return {};
    }
    return std::shared_ptr<Simulation::DynamicalSystem>(system.release());
}

} // namespace

TEST(SimulationParameterMetadata, AllSystemsExposeDetailedParameterDescriptions) {
    for (const auto& [system_id, _system_name] : Simulation::listSystems()) {
        auto system = Simulation::createSystem(system_id);
        ASSERT_NE(system, nullptr) << "Failed to create system " << system_id;

        const auto raw_descriptors = system->parameterDescriptors();
        const auto detailed_descriptors = system->parameterDescriptorsDetailed();
        ASSERT_EQ(raw_descriptors.size(), detailed_descriptors.size())
            << "Descriptor count changed for system " << system_id;

        for (size_t index = 0; index < detailed_descriptors.size(); ++index) {
            const auto& raw = raw_descriptors[index];
            const auto& detailed = detailed_descriptors[index];

            EXPECT_FALSE(raw.description.empty())
                << "Raw C++ ParamDescriptor description is empty for system "
                << system_id << " parameter '" << raw.name << "'";
            EXPECT_FALSE(detailed.description.empty())
                << "Detailed ParamDescriptor description is empty for system "
                << system_id << " parameter '" << detailed.name << "'";
            EXPECT_GT(detailed.description.size(), 32U)
                << "Detailed ParamDescriptor description is too short for system "
                << system_id << " parameter '" << detailed.name << "'";
            EXPECT_NE(detailed.description.find(system->name()), std::string::npos)
                << "Detailed description does not mention the owning model for system "
                << system_id << " parameter '" << detailed.name << "'";
            EXPECT_NE(detailed.description.find("Default"), std::string::npos)
                << "Detailed description is missing default-value information for system "
                << system_id << " parameter '" << detailed.name << "'";
            EXPECT_NE(detailed.description.find("Range"), std::string::npos)
                << "Detailed description is missing range information for system "
                << system_id << " parameter '" << detailed.name << "'";
        }
    }
}

TEST(SimulationInitialResponse, AllSystemsStayNonzeroThroughHalfSecondWithNonzeroInitialCondition) {
    Simulation::SimConfig config;
    config.dt = kTimeStep;
    config.totalTime = kHalfSecondWindow;
    config.method = Simulation::IntegrationMethod::RungeKutta4;

    for (const auto& [system_id, system_name] : Simulation::listSystems()) {
        auto system = make_shared_system(system_id);
        ASSERT_NE(system, nullptr) << "Failed to create system " << system_id;

        Simulation::SimulationEngine engine;
        engine.setSystem(system);
        engine.setConfig(config);
        engine.setInitialState(system->defaultInitialStateForUi());

        const auto record = engine.run();
        ASSERT_FALSE(record.times.empty())
            << "Simulation produced no samples for system " << system_id;
        EXPECT_TRUE(has_nonzero_amplitude_through_half_second(record))
            << "System " << system_id << " (" << system_name << ") lost all observable amplitude before 0.5 s";
    }
}

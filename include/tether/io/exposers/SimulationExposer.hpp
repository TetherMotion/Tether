/**
 * @file SimulationExposer.hpp
 * @brief Exposes SimulationEngine state and configuration.
 * @copyright Copyright (C) 2025-2026 Tether Authors
 */
#pragma once

#include "tether/io/ParameterExposer.hpp"
#include "tether/simulation/SimulationEngine.hpp"

namespace tether { namespace io { namespace exposers {

/**
 * @class SimulationExposer
 * @brief Exposes the SimulationEngine's configuration and runtime signals.
 *
 * ## Signals (read-only)
 *  - `current_time` (F64): Current simulation time.
 *  - `is_finished` (Bool): Whether the simulation has completed.
 *  - `state[0..N]` (F64): Individual state vector components.
 *
 * Uses idBase + 0x01..0x0F for scalar signals/params, idBase + 0x100+ for state components.
 */
class SimulationExposer : public IParameterExposer {
public:
    /**
     * @param engine    Reference to the SimulationEngine.
     * @param stateDim  Number of state vector dimensions to expose.
     */
    explicit SimulationExposer(tether::simulation::SimulationEngine& engine,
                               size_t stateDim = 0)
        : engine_(engine), stateDim_(stateDim) {}

    const char* moduleName() const override { return "simulation"; }

    void expose(Registry& registry, uint64_t idBase) override {
        const std::string group = "simulation";

        // -- Signals --
        registry.addSignal({
            makeId(idBase, 0x01), "current_time",
            "Current simulation time", group, ValueType::F64,
            [this](void* d) { double v = engine_.currentTime(); std::memcpy(d, &v, 8); }
        });

        registry.addSignal({
            makeId(idBase, 0x02), "is_finished",
            "Whether the simulation run has completed", group, ValueType::Bool,
            [this](void* d) { uint8_t v = engine_.isFinished() ? 1 : 0; std::memcpy(d, &v, 1); }
        });

        // Expose individual state vector components
        for (size_t i = 0; i < stateDim_; ++i) {
            std::string name = "state[" + std::to_string(i) + "]";
            registry.addSignal({
                makeId(idBase, static_cast<uint32_t>(0x100 + i)),
                name,
                "State vector component " + std::to_string(i),
                group + ".state",
                ValueType::F64,
                [this, i](void* d) {
                    const auto& st = engine_.currentState();
                    double v = (i < st.size()) ? st[i] : 0.0;
                    std::memcpy(d, &v, 8);
                }
            });
        }
    }

private:
    tether::simulation::SimulationEngine& engine_;
    size_t stateDim_;
};

}}} // namespace tether::io::exposers

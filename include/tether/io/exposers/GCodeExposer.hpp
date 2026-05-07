/**
 * @file GCodeExposer.hpp
 * @brief Exposes GCode Interpreter state, position, and statistics.
 * @copyright Copyright (C) 2025-2026 Tether Authors
 */
#pragma once

#include "tether/io/ParameterExposer.hpp"
#include "tether/gcode/GCodeInterpreter.hpp"

namespace tether { namespace io { namespace exposers {

/**
 * @class GCodeExposer
 * @brief Exposes the G-code interpreter's parameters and signals.
 *
 * ## Parameters (read/write)
 *  - `dry_run` (Bool): Toggle dry-run mode.
 *  - `block_delete` (Bool): Toggle block-delete switch.
 *  - `optional_stop` (Bool): Toggle M01 optional stop.
 *  - `mode` (U8): InterpreterMode enum.
 *
 * ## Signals (read-only)
 *  - `state` (U8): InterpreterState enum.
 *  - `current_line` (U32): Currently executing line number.
 *  - `total_lines` (U32): Total lines in loaded program.
 *  - `program_loaded` (Bool): Whether a program is loaded.
 *  - `finished` (Bool): Whether execution is finished.
 *  - `position.x`, `position.y`, `position.z` (F64): Current work position.
 *  - `machine_position.x/y/z` (F64): Current machine position.
 *  - `stats.lines_processed` (U32): Lines processed count.
 *  - `stats.motion_segments` (U32): Motion segments generated.
 *  - `stats.total_path_length` (F64): Total path length.
 *  - `stats.errors` (U32): Error count.
 */
class GCodeExposer : public IParameterExposer {
public:
    explicit GCodeExposer(tether::gcode::Interpreter& interp)
        : interp_(interp) {}

    const char* moduleName() const override { return "gcode"; }

    void expose(Registry& registry, uint64_t idBase) override {
        const std::string group = "gcode";

        // -- Parameters --
        registry.addParam({
            makeId(idBase, 0x01), "dry_run",
            "Enable dry-run mode (no real motion)", group, ValueType::Bool,
            [this](void* d) { uint8_t v = interp_.isDryRun() ? 1 : 0; std::memcpy(d, &v, 1); },
            [this](const void* s) { uint8_t v; std::memcpy(&v, s, 1); interp_.setDryRun(v != 0); }
        });

        registry.addParam({
            makeId(idBase, 0x02), "block_delete",
            "Enable block-delete switch (/)", group, ValueType::Bool,
            [this](void* d) { uint8_t v = interp_.isBlockDeleteEnabled() ? 1 : 0; std::memcpy(d, &v, 1); },
            [this](const void* s) { uint8_t v; std::memcpy(&v, s, 1); interp_.setBlockDelete(v != 0); }
        });

        registry.addParam({
            makeId(idBase, 0x03), "optional_stop",
            "Enable M01 optional stop", group, ValueType::Bool,
            [this](void* d) { uint8_t v = interp_.isOptionalStopEnabled() ? 1 : 0; std::memcpy(d, &v, 1); },
            [this](const void* s) { uint8_t v; std::memcpy(&v, s, 1); interp_.setOptionalStop(v != 0); }
        });

        registry.addParam({
            makeId(idBase, 0x04), "mode",
            "Interpreter mode (0=AUTO, 1=MDI, 2=STEP, 3=VERIFY)", group, ValueType::U8,
            [this](void* d) { uint8_t v = static_cast<uint8_t>(interp_.getMode()); std::memcpy(d, &v, 1); },
            [this](const void* s) {
                uint8_t v; std::memcpy(&v, s, 1);
                interp_.setMode(static_cast<tether::gcode::InterpreterMode>(v));
            }
        });

        // -- Signals --
        registry.addSignal({
            makeId(idBase, 0x81), "state",
            "Interpreter state enum", group, ValueType::U8,
            [this](void* d) { uint8_t v = static_cast<uint8_t>(interp_.getState()); std::memcpy(d, &v, 1); }
        });

        registry.addSignal({
            makeId(idBase, 0x82), "current_line",
            "Currently executing G-code line number", group, ValueType::U32,
            [this](void* d) { uint32_t v = interp_.getCurrentLine(); std::memcpy(d, &v, 4); }
        });

        registry.addSignal({
            makeId(idBase, 0x83), "total_lines",
            "Total lines in loaded program", group, ValueType::U32,
            [this](void* d) { uint32_t v = interp_.getTotalLines(); std::memcpy(d, &v, 4); }
        });

        registry.addSignal({
            makeId(idBase, 0x84), "program_loaded",
            "Whether a program is loaded", group, ValueType::Bool,
            [this](void* d) { uint8_t v = interp_.isProgramLoaded() ? 1 : 0; std::memcpy(d, &v, 1); }
        });

        registry.addSignal({
            makeId(idBase, 0x85), "finished",
            "Whether execution is finished", group, ValueType::Bool,
            [this](void* d) { uint8_t v = interp_.isFinished() ? 1 : 0; std::memcpy(d, &v, 1); }
        });

        // Position components
        registry.addSignal({
            makeId(idBase, 0x90), "position.x",
            "Current X work coordinate", group + ".position", ValueType::F64,
            [this](void* d) {
                auto pos = interp_.getCurrentPosition();
                double v = pos.x;
                std::memcpy(d, &v, 8);
            }
        });
        registry.addSignal({
            makeId(idBase, 0x91), "position.y",
            "Current Y work coordinate", group + ".position", ValueType::F64,
            [this](void* d) {
                auto pos = interp_.getCurrentPosition();
                double v = pos.y;
                std::memcpy(d, &v, 8);
            }
        });
        registry.addSignal({
            makeId(idBase, 0x92), "position.z",
            "Current Z work coordinate", group + ".position", ValueType::F64,
            [this](void* d) {
                auto pos = interp_.getCurrentPosition();
                double v = pos.z;
                std::memcpy(d, &v, 8);
            }
        });

        registry.addSignal({
            makeId(idBase, 0x93), "machine_position.x",
            "Current X machine coordinate", group + ".machine_position", ValueType::F64,
            [this](void* d) {
                auto pos = interp_.getMachinePosition();
                double v = pos.x;
                std::memcpy(d, &v, 8);
            }
        });
        registry.addSignal({
            makeId(idBase, 0x94), "machine_position.y",
            "Current Y machine coordinate", group + ".machine_position", ValueType::F64,
            [this](void* d) {
                auto pos = interp_.getMachinePosition();
                double v = pos.y;
                std::memcpy(d, &v, 8);
            }
        });
        registry.addSignal({
            makeId(idBase, 0x95), "machine_position.z",
            "Current Z machine coordinate", group + ".machine_position", ValueType::F64,
            [this](void* d) {
                auto pos = interp_.getMachinePosition();
                double v = pos.z;
                std::memcpy(d, &v, 8);
            }
        });

        // Statistics
        registry.addSignal({
            makeId(idBase, 0xA0), "stats.lines_processed",
            "Number of G-code lines processed", group + ".stats", ValueType::U32,
            [this](void* d) { uint32_t v = interp_.getStatistics().linesProcessed; std::memcpy(d, &v, 4); }
        });
        registry.addSignal({
            makeId(idBase, 0xA1), "stats.motion_segments",
            "Number of motion segments generated", group + ".stats", ValueType::U32,
            [this](void* d) { uint32_t v = interp_.getStatistics().motionSegments; std::memcpy(d, &v, 4); }
        });
        registry.addSignal({
            makeId(idBase, 0xA2), "stats.total_path_length",
            "Total path length in program units", group + ".stats", ValueType::F64,
            [this](void* d) { double v = interp_.getStatistics().totalPathLength; std::memcpy(d, &v, 8); }
        });
        registry.addSignal({
            makeId(idBase, 0xA3), "stats.errors",
            "Number of errors encountered", group + ".stats", ValueType::U32,
            [this](void* d) { uint32_t v = interp_.getStatistics().errors; std::memcpy(d, &v, 4); }
        });
    }

private:
    tether::gcode::Interpreter& interp_;
};

}}} // namespace tether::io::exposers

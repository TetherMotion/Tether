/**
 * @file PIDExposer.hpp
 * @brief Exposes PIDController gains (params) and diagnostics (signals).
 * @copyright Copyright (C) 2025-2026 Tether Authors
 */
#pragma once

#include "tether/io/ParameterExposer.hpp"
#include "tether/control/PIDControllers.hpp"

namespace tether { namespace io { namespace exposers {

/**
 * @class PIDExposer
 * @brief Exposes a PIDController's tuning parameters and diagnostic signals.
 *
 * ## Parameters (read/write)
 *  - `kp`, `ki`, `kd` (F64): PID gains.
 *  - `derivative_filter` (F64): Derivative low-pass filter time constant.
 *  - `integral` (F64): Integral accumulator (read/write for bumpless transfer).
 *  - `enabled` (Bool): Controller enable flag.
 *  - `output_min`, `output_max` (F64): Output saturation limits.
 *
 * ## Signals (read-only)
 *  - `error` (F64): Last computed error.
 *  - `proportional` (F64): Proportional term.
 *  - `integral_term` (F64): Integral term.
 *  - `derivative_term` (F64): Derivative term.
 *  - `output` (F64): Total control output.
 *  - `saturated` (Bool): Whether output is saturated.
 *  - `cycle_count` (U64): Number of compute cycles.
 */
class PIDExposer : public IParameterExposer {
public:
    /**
     * @param controller Reference to the PIDController.
     * @param axisName   Optional axis identifier (e.g. "x", "y", "z").
     */
    explicit PIDExposer(tether::control::PIDController& controller,
                        const std::string& axisName = "")
        : ctrl_(controller), axisName_(axisName) {}

    const char* moduleName() const override { return "pid"; }

    void expose(Registry& registry, uint64_t idBase) override {
        std::string prefix = axisName_.empty() ? "" : axisName_ + ".";
        std::string group  = axisName_.empty() ? "pid" : "pid." + axisName_;

        // -- Parameters --
        registry.addParam({
            makeId(idBase, 0x01), prefix + "kp",
            "Proportional gain", group, ValueType::F64,
            [this](void* d) { double v = ctrl_.getKp(); std::memcpy(d, &v, 8); },
            [this](const void* s) { double v; std::memcpy(&v, s, 8); ctrl_.setGains(v, ctrl_.getKi(), ctrl_.getKd()); }
        });

        registry.addParam({
            makeId(idBase, 0x02), prefix + "ki",
            "Integral gain", group, ValueType::F64,
            [this](void* d) { double v = ctrl_.getKi(); std::memcpy(d, &v, 8); },
            [this](const void* s) { double v; std::memcpy(&v, s, 8); ctrl_.setGains(ctrl_.getKp(), v, ctrl_.getKd()); }
        });

        registry.addParam({
            makeId(idBase, 0x03), prefix + "kd",
            "Derivative gain", group, ValueType::F64,
            [this](void* d) { double v = ctrl_.getKd(); std::memcpy(d, &v, 8); },
            [this](const void* s) { double v; std::memcpy(&v, s, 8); ctrl_.setGains(ctrl_.getKp(), ctrl_.getKi(), v); }
        });

        registry.addParam({
            makeId(idBase, 0x04), prefix + "derivative_filter",
            "Derivative filter time constant (Tf)", group, ValueType::F64,
            [this](void* d) { double v = ctrl_.getDerivativeFilter(); std::memcpy(d, &v, 8); },
            [this](const void* s) { double v; std::memcpy(&v, s, 8); ctrl_.setDerivativeFilter(v); }
        });

        registry.addParam({
            makeId(idBase, 0x05), prefix + "integral",
            "Integral accumulator value", group, ValueType::F64,
            [this](void* d) { double v = ctrl_.getIntegral(); std::memcpy(d, &v, 8); },
            [this](const void* s) { double v; std::memcpy(&v, s, 8); ctrl_.setIntegral(v); }
        });

        registry.addParam({
            makeId(idBase, 0x06), prefix + "enabled",
            "Controller enable flag", group, ValueType::Bool,
            [this](void* d) { uint8_t v = ctrl_.isEnabled() ? 1 : 0; std::memcpy(d, &v, 1); },
            [this](const void* s) { uint8_t v; std::memcpy(&v, s, 1); ctrl_.setEnabled(v != 0); }
        });

        registry.addParam({
            makeId(idBase, 0x07), prefix + "output_min",
            "Output minimum saturation limit", group, ValueType::F64,
            [this](void* d) {
                double v = ctrl_.getSaturationLimits().outputMin;
                std::memcpy(d, &v, 8);
            },
            [this](const void* s) {
                double v; std::memcpy(&v, s, 8);
                auto lim = ctrl_.getSaturationLimits();
                lim.outputMin = v;
                ctrl_.setSaturationLimits(lim);
            }
        });

        registry.addParam({
            makeId(idBase, 0x08), prefix + "output_max",
            "Output maximum saturation limit", group, ValueType::F64,
            [this](void* d) {
                double v = ctrl_.getSaturationLimits().outputMax;
                std::memcpy(d, &v, 8);
            },
            [this](const void* s) {
                double v; std::memcpy(&v, s, 8);
                auto lim = ctrl_.getSaturationLimits();
                lim.outputMax = v;
                ctrl_.setSaturationLimits(lim);
            }
        });

        // -- Signals --
        registry.addSignal({
            makeId(idBase, 0x81), prefix + "error",
            "Last computed error (reference - measured)", group, ValueType::F64,
            [this](void* d) {
                double v = ctrl_.getLastOutput().error;
                std::memcpy(d, &v, 8);
            }
        });

        registry.addSignal({
            makeId(idBase, 0x82), prefix + "proportional",
            "Proportional term of last computation", group, ValueType::F64,
            [this](void* d) {
                double v = ctrl_.getLastOutput().proportional;
                std::memcpy(d, &v, 8);
            }
        });

        registry.addSignal({
            makeId(idBase, 0x83), prefix + "integral_term",
            "Integral term of last computation", group, ValueType::F64,
            [this](void* d) {
                double v = ctrl_.getLastOutput().integral;
                std::memcpy(d, &v, 8);
            }
        });

        registry.addSignal({
            makeId(idBase, 0x84), prefix + "derivative_term",
            "Derivative term of last computation", group, ValueType::F64,
            [this](void* d) {
                double v = ctrl_.getLastOutput().derivative;
                std::memcpy(d, &v, 8);
            }
        });

        registry.addSignal({
            makeId(idBase, 0x85), prefix + "output",
            "Total control output", group, ValueType::F64,
            [this](void* d) {
                double v = ctrl_.getLastOutput().control;
                std::memcpy(d, &v, 8);
            }
        });

        registry.addSignal({
            makeId(idBase, 0x86), prefix + "saturated",
            "Whether output is at saturation limit", group, ValueType::Bool,
            [this](void* d) {
                uint8_t v = ctrl_.getLastOutput().saturated ? 1 : 0;
                std::memcpy(d, &v, 1);
            }
        });

        registry.addSignal({
            makeId(idBase, 0x87), prefix + "cycle_count",
            "Number of compute cycles", group, ValueType::U64,
            [this](void* d) {
                uint64_t v = ctrl_.getDiagnostics().cycleCount;
                std::memcpy(d, &v, 8);
            }
        });

        registry.addSignal({
            makeId(idBase, 0x88), prefix + "last_derivative",
            "Last raw derivative value", group, ValueType::F64,
            [this](void* d) {
                double v = ctrl_.getLastDerivative();
                std::memcpy(d, &v, 8);
            }
        });
    }

private:
    tether::control::PIDController& ctrl_;
    std::string axisName_;
};

}}} // namespace tether::io::exposers

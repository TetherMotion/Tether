/**
 * @file MotionPlannerExposer.hpp
 * @brief Exposes MotionPlan state and MotionPlanBuilder limits.
 * @copyright Copyright (C) 2025-2026 Tether Authors
 */
#pragma once

#include "tether/io/ParameterExposer.hpp"
#include "tether/motion_planner/MotionPlan.hpp"
#include "tether/motion_planner/MotionPlanBuilder.hpp"

namespace tether { namespace io { namespace exposers {

/**
 * @class MotionPlanExposer3D
 * @brief Exposes a 3D MotionPlan's runtime state and MotionPlanBuilder limits.
 *
 * ## Parameters (via builder)
 *  - `feed_override` (F64): Feed rate override factor.
 *
 * ## Signals (via plan)
 *  - `total_duration` (F64): Total plan duration.
 *  - `total_length` (F64): Total path length.
 *  - `num_segments` (U32): Number of motion segments.
 *  - `is_paused` (Bool): Whether the plan is paused.
 *  - `is_reverse` (Bool): Whether reverse motion is active.
 *
 * The plan pointer can be updated at runtime via setPlan() when a new plan
 * is built; readers will get the latest values.
 */
class MotionPlanExposer3D : public IParameterExposer {
public:
    using Plan    = tether::motion::MotionPlan3D;
    using Builder = tether::motion::MotionPlanBuilder3D;

    /**
     * @param builder  Reference to the builder (for limits/config parameters).
     */
    explicit MotionPlanExposer3D(Builder& builder)
        : builder_(builder) {}

    /// Set (or update) the active plan pointer.
    void setPlan(Plan* plan) { plan_ = plan; }

    const char* moduleName() const override { return "motion_planner"; }

    void expose(Registry& registry, uint64_t idBase) override {
        const std::string group = "motion_planner";

        // -- Parameters --
        registry.addParam({
            makeId(idBase, 0x01), "feed_override",
            "Feed rate override factor (1.0 = 100%)", group, ValueType::F64,
            [this](void* d) {
                double v = plan_ ? plan_->feedOverride() : 1.0;
                std::memcpy(d, &v, 8);
            },
            [this](const void* s) {
                if (!plan_) return;
                double v; std::memcpy(&v, s, 8);
                plan_->setFeedOverride(v);
            }
        });

        // -- Signals --
        registry.addSignal({
            makeId(idBase, 0x81), "total_duration",
            "Total motion plan duration in seconds", group, ValueType::F64,
            [this](void* d) {
                double v = plan_ ? plan_->totalDuration() : 0.0;
                std::memcpy(d, &v, 8);
            }
        });

        registry.addSignal({
            makeId(idBase, 0x82), "total_length",
            "Total path length in user units", group, ValueType::F64,
            [this](void* d) {
                double v = plan_ ? plan_->totalLength() : 0.0;
                std::memcpy(d, &v, 8);
            }
        });

        registry.addSignal({
            makeId(idBase, 0x83), "num_segments",
            "Number of motion segments in the plan", group, ValueType::U32,
            [this](void* d) {
                uint32_t v = plan_ ? static_cast<uint32_t>(plan_->numSegments()) : 0u;
                std::memcpy(d, &v, 4);
            }
        });

        registry.addSignal({
            makeId(idBase, 0x84), "is_paused",
            "Whether the motion plan is paused", group, ValueType::Bool,
            [this](void* d) {
                uint8_t v = (plan_ && plan_->isPaused()) ? 1 : 0;
                std::memcpy(d, &v, 1);
            }
        });

        registry.addSignal({
            makeId(idBase, 0x85), "is_reverse",
            "Whether reverse motion is active", group, ValueType::Bool,
            [this](void* d) {
                uint8_t v = (plan_ && plan_->isReverse()) ? 1 : 0;
                std::memcpy(d, &v, 1);
            }
        });
    }

private:
    Builder& builder_;
    Plan* plan_ = nullptr;
};

}}} // namespace tether::io::exposers

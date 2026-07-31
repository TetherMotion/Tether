/**
 * @file MotionBlockSink.hpp
 * @brief Receives MotionBlocks for analysis, logging, and visualization.
 *
 * @details
 * The device emits MotionBlocks (decoded queue_step sequences) to a
 * MotionBlockSink. The sink can record them for later analysis, print them,
 * or feed them to a visualization pipeline. Examples use a PrintingSink that
 * logs blocks to stdout; tests use a RecordingSink that stores blocks for
 * assertions.
 */

#pragma once

#include "tether/klipper/motion/MotionBlock.hpp"

#include <vector>
#include <functional>
#include <mutex>
#include <cstdio>

namespace tether::klipper::motion {

/// @brief Abstract sink for MotionBlocks.
class MotionBlockSink {
public:
    virtual ~MotionBlockSink() = default;
    virtual void emit(const MotionBlock& block) = 0;
};

/// @brief A sink that records all blocks in memory.
class RecordingSink : public MotionBlockSink {
public:
    void emit(const MotionBlock& block) override {
        std::lock_guard<std::mutex> lk(mtx_);
        blocks_.push_back(block);
    }
    const std::vector<MotionBlock>& blocks() const { return blocks_; }
    void clear() { blocks_.clear(); }
private:
    std::mutex mtx_;
    std::vector<MotionBlock> blocks_;
};

/// @brief A sink that prints blocks to stdout (for examples).
class PrintingSink : public MotionBlockSink {
public:
    void emit(const MotionBlock& block) override {
        std::lock_guard<std::mutex> lk(mtx_);
        std::printf("MotionBlock oid=%u steps=%u duration=%u rate=%.3f src=%s\n",
                   block.oid, block.totalSteps(), block.totalDuration(),
                   block.averageStepRate(),
                   block.sourceLabel.c_str());
        for (size_t i = 0; i < block.steps.size(); ++i) {
            const auto& s = block.steps[i];
            std::printf("  [%zu] interval=%u count=%u add=%d\n",
                       i, s.interval, s.count, s.add);
        }
    }
private:
    std::mutex mtx_;
};

/// @brief A sink that forwards to a callback.
class CallbackSink : public MotionBlockSink {
public:
    using Callback = std::function<void(const MotionBlock&)>;
    explicit CallbackSink(Callback cb) : cb_(std::move(cb)) {}
    void emit(const MotionBlock& block) override { cb_(block); }
private:
    Callback cb_;
};

} // namespace tether::klipper::motion

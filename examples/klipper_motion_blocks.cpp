/**
 * @file klipper_motion_blocks.cpp
 * @brief Example: stepper motion blocks and reconstruction.
 *
 * @details
 * Demonstrates the motion subsystem:
 *   1. Create a stepper and enqueue step sequences.
 *   2. Tick the stepper and observe position changes.
 *   3. Reconstruct the trajectory from the step commands.
 *   4. Emit motion blocks to a printing sink for analysis.
 */

#include "tether/klipper/objects/Stepper.hpp"
#include "tether/klipper/motion/MotionReconstructor.hpp"
#include "tether/klipper/motion/MotionBlockSink.hpp"

#include <cstdio>

using namespace tether::klipper::objects;
using namespace tether::klipper::motion;

int main() {
    // 1. Create a stepper
    Stepper stepper(0);

    // 2. Enqueue step sequences (simulating a move with acceleration)
    // First segment: 1000 ticks/step, 5 steps, no acceleration
    stepper.enqueueStep({1000, 5, 0}, 0);
    // Second segment: 800 ticks/step, 3 steps (faster)
    stepper.enqueueStep({800, 3, 0}, 5000);

    std::printf("Stepper OID=%u, pending commands=%zu\n",
                stepper.oid(), stepper.pendingCommands());

    // 3. Tick the stepper through the motion
    uint32_t totalSteps = 0;
    for (uint32_t clock = 0; clock <= 10000; clock += 1000) {
        uint32_t steps = stepper.tick(clock);
        if (steps > 0) {
            std::printf("clock=%u: %u steps, position=%d\n",
                        clock, steps, stepper.position());
            totalSteps += steps;
        }
    }
    std::printf("Total steps: %u, final position: %d\n",
                totalSteps, stepper.position());

    // 4. Reconstruct the trajectory
    std::vector<StepCommand> steps = {{1000, 5, 0}, {800, 3, 0}};
    auto traj = MotionReconstructor::reconstruct(steps, 0, 80.0);
    std::printf("\nReconstructed trajectory: %zu points\n", traj.size());
    for (size_t i = 0; i < traj.size(); ++i) {
        std::printf("  [%zu] clock=%u pos=%.4fmm vel=%.6f steps/tick\n",
                    i, traj[i].clock, traj[i].position, traj[i].velocity);
    }

    // 5. Emit motion blocks to a printing sink
    std::printf("\nMotion blocks:\n");
    PrintingSink sink;
    auto block = MotionReconstructor::toMotionBlock(steps, 0, 0, 80.0);
    sink.emit(block);

    return 0;
}

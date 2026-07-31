/**
 * @file Stepper.cpp
 * @brief Stepper device-side handler implementation.
 */

#include "tether/klipper/objects/Stepper.hpp"

namespace tether::klipper::objects {

uint32_t Stepper::tick(uint32_t clock) {
    uint32_t stepsTaken = 0;
    while (true) {
        if (remainingSteps_ == 0) {
            // Need a new command; if none available, we're done.
            if (queue_.empty()) break;
            auto& front = queue_.front();
            if (clock < front.startClock) break; // not time yet
            currentInterval_ = front.cmd.interval;
            remainingSteps_ = front.cmd.count;
            currentAdd_ = front.cmd.add;
            nextStepClock_ = front.startClock;
            queue_.pop_front();
        }
        // Execute steps whose time has arrived.
        while (remainingSteps_ > 0 && clock >= nextStepClock_) {
            position_ += dir_;
            ++stepsTaken;
            --remainingSteps_;
            nextStepClock_ += currentInterval_;
            currentInterval_ = static_cast<uint32_t>(
                static_cast<int32_t>(currentInterval_) + currentAdd_);
        }
        if (remainingSteps_ > 0) break; // waiting for more clock ticks
    }
    return stepsTaken;
}

} // namespace tether::klipper::objects

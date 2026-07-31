/**
 * @file klipper_gpio_pwm.cpp
 * @brief Example: GPIO digital output and PWM output peripherals.
 *
 * @details
 * Demonstrates the peripheral objects:
 *   1. Create a DigitalOut and schedule value changes.
 *   2. Create a PwmOut and set duty cycles.
 *   3. Tick both at simulated clock times.
 */

#include "tether/klipper/objects/Peripherals.hpp"

#include <cstdio>

using namespace tether::klipper::objects;

int main() {
    // 1. Digital output with scheduled changes
    DigitalOut dout(0);
    std::printf("DigitalOut OID=%u initial value=%u\n", dout.oid(), dout.value());

    // Schedule: set high at clock=1000, low at clock=5000
    dout.scheduleValue(1, 1000);
    dout.scheduleValue(0, 5000);
    std::printf("Pending scheduled changes: %zu\n", dout.pending());

    // Tick through the schedule
    for (uint32_t clock = 0; clock <= 6000; clock += 1000) {
        dout.tick(clock);
        std::printf("clock=%u: value=%u\n", clock, dout.value());
    }

    // 2. PWM output with duty cycle changes
    PwmOut pwm(1);
    pwm.setDuty(256, 1024); // 25% duty
    std::printf("\nPwmOut OID=%u duty=%u/%u (%.1f%%)\n",
                pwm.oid(), pwm.duty(), pwm.maxCycle(),
                100.0 * pwm.duty() / pwm.maxCycle());

    // Schedule a duty change
    pwm.scheduleDuty(768, 2000); // 75% duty at clock=2000
    pwm.tick(1000);
    std::printf("clock=1000: duty=%u (%.1f%%)\n",
                pwm.duty(), 100.0 * pwm.duty() / pwm.maxCycle());
    pwm.tick(2000);
    std::printf("clock=2000: duty=%u (%.1f%%)\n",
                pwm.duty(), 100.0 * pwm.duty() / pwm.maxCycle());

    // 3. Endstop and Trsync
    Endstop endstop(2);
    endstop.setState(1);
    std::printf("\nEndstop OID=%u state=%u (triggered)\n",
                endstop.oid(), endstop.state());

    Trsync trsync(3);
    trsync.arm(1000, 5000);
    std::printf("Trsync OID=%u armed, state=%d\n",
                trsync.oid(), (int)trsync.state());
    trsync.trigger(2500);
    std::printf("After trigger: state=%d triggerClock=%u\n",
                (int)trsync.state(), trsync.triggerClock());

    return 0;
}

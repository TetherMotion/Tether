#include <gtest/gtest.h>

#include "tether/control/Controllers.hpp"

using namespace Control;

// Comprehensive unit tests that exercise ControllerBase polymorphism and
// core behaviors (type/name, compute contributions, saturation, reset,
// manual/disabled modes and getLastOutput).

TEST(Controllers, TypeAndName)
{
    PController p;
    PDController pd;
    PIDController pid;

    EXPECT_EQ(p.getType(), ControllerType::P);
    EXPECT_STREQ(p.getName(), "P Controller");

    EXPECT_EQ(pd.getType(), ControllerType::PD);
    EXPECT_STREQ(pd.getName(), "PD Controller");

    EXPECT_EQ(pid.getType(), ControllerType::PID);
    EXPECT_STREQ(pid.getName(), "PID Controller");
}

TEST(Controllers, PIDComputeSaturationAndReset)
{
    PIDController pid;
    // make math simple: use dt = 1.0s so discrete formulas are easy to reason about
    pid.setGains(2.0, 1.0, 0.5); // Kp, Ki, Kd
    pid.setDerivativeFilter(0.0); // no filtering for deterministic derivative

    SaturationLimits limits;
    limits.outputMin = -5.0;
    limits.outputMax = 5.0;
    pid.setSaturationLimits(limits);

    ControllerInput input;
    input.reference = 3.0;   // setpoint
    input.measured = 0.0;    // measured value
    input.dt = 1.0;          // 1 second timestep to simplify expectation

    // First compute: raw output (P + I + D) would exceed limits so controller will saturate.
    auto out1 = pid.compute(input);

    // Proportional contribution is Kp * error = 2 * 3 = 6
    EXPECT_NEAR(out1.proportional, 6.0, 1e-9);
    // With anti-windup default (Clamping) the integrator is NOT allowed to grow while
    // the controller output is saturating, so integral stays at 0 on the first calls.
    EXPECT_NEAR(out1.integral, 0.0, 1e-12);
    EXPECT_NEAR(out1.derivative, 0.0, 1e-8);

    // Controller should report saturation and clipped control
    EXPECT_TRUE(out1.saturated);
    EXPECT_NEAR(out1.control, 5.0, 1e-9);

    // If we reduce the error so the controller is not saturated, the integrator should then grow
    input.reference = 1.0; // error = 1.0, P = 2.0
    auto outNotSat = pid.compute(input);
    EXPECT_FALSE(outNotSat.saturated);
    EXPECT_GT(outNotSat.integral, 0.0);

    // Reset controller and verify integrator/state cleared
    pid.reset();
    input.reference = 1.0;
    auto outAfterReset = pid.compute(input);
    // After reset the integral should be consistent with a single integration step
    EXPECT_NEAR(outAfterReset.integral, pid.getLastOutput().integral, 1e-12);

    // getLastOutput() should reflect the most recent compute() result
    const auto& last = pid.getLastOutput();
    EXPECT_NEAR(last.control, outAfterReset.control, 1e-12);
}

TEST(Controllers, ManualAndDisabledModes)
{
    PController p;
    ControllerInput input;
    input.reference = 1.0;
    input.measured = 0.0;
    input.dt = 0.1;

    // Manual mode: compute should return manual output regardless of gains
    p.setGain(10.0);
    p.setMode(ControllerMode::Manual);
    p.setManualOutput(2.5);
    auto mOut = p.compute(input);
    EXPECT_NEAR(mOut.control, 2.5, 1e-9);

    // Disabled mode: controller should produce zero output
    p.setEnabled(false);
    auto dOut = p.compute(input);
    EXPECT_NEAR(dOut.control, 0.0, 1e-9);
}

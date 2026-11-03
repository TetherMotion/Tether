#include <gtest/gtest.h>

#include <cmath>

#include "tether/control/Controllers.hpp"

using namespace tether::control;

// Coverage tests for src/control/PIDControllers.cpp — drives every
// controller class, tuning method, derivative filter, and anti-windup arm.

static ControllerInput makeInput(double ref, double meas, double dt = 0.01) {
    ControllerInput in;
    in.reference = ref;
    in.measured = meas;
    in.dt = dt;
    return in;
}

// ============================================================================
// P / PD / PI
// ============================================================================

TEST(PIDControllersCoverage, PControllerCompute) {
    PController p;
    p.setGain(2.5);
    EXPECT_DOUBLE_EQ(p.getGain(), 2.5);
    auto out = p.compute(makeInput(10.0, 4.0));
    EXPECT_DOUBLE_EQ(out.control, 15.0);
    EXPECT_DOUBLE_EQ(out.error, 6.0);
}

TEST(PIDControllersCoverage, PDSkipsDerivativeOnFirstRun) {
    PDController pd;
    pd.setGains(1.0, 5.0);
    EXPECT_DOUBLE_EQ(pd.getKp(), 1.0);
    EXPECT_DOUBLE_EQ(pd.getKd(), 5.0);
    auto out = pd.compute(makeInput(1.0, 0.0));
    EXPECT_DOUBLE_EQ(out.derivative, 0.0);  // first run: no kick
    out = pd.compute(makeInput(1.0, 0.5));
    EXPECT_NE(out.derivative, 0.0);
}

TEST(PIDControllersCoverage, PDDerivativeOnError) {
    PDController pd;
    pd.setGains(0.0, 1.0);
    pd.setDerivativeOnMeasurement(false);  // derivative on error
    pd.compute(makeInput(0.0, 0.0));
    auto out = pd.compute(makeInput(1.0, 0.0));
    EXPECT_DOUBLE_EQ(out.derivative, 1.0 / 0.01);
}

TEST(PIDControllersCoverage, PDDerivativeFilter) {
    PDController pd;
    pd.setGains(0.0, 1.0);
    pd.setDerivativeFilter(0.05);
    EXPECT_DOUBLE_EQ(pd.getDerivativeFilter(), 0.05);
    pd.compute(makeInput(0.0, 0.0));
    auto out = pd.compute(makeInput(0.0, 1.0));
    // filtered derivative is pulled below the raw -1/0.01 = -100
    EXPECT_GT(out.derivative, -100.0);
    EXPECT_LT(out.derivative, 0.0);
}

TEST(PIDControllersCoverage, PDReset) {
    PDController pd;
    pd.setGains(0.0, 1.0);
    pd.compute(makeInput(0.0, 0.0));
    pd.compute(makeInput(0.0, 1.0));
    pd.reset();
    // after reset the next compute is again a first run: no derivative
    auto out = pd.compute(makeInput(0.0, 5.0));
    EXPECT_DOUBLE_EQ(out.derivative, 0.0);
}

TEST(PIDControllersCoverage, PIBasicAndIntegralAccumulation) {
    PIController pi;
    pi.setGains(1.0, 2.0);
    EXPECT_DOUBLE_EQ(pi.getKp(), 1.0);
    EXPECT_DOUBLE_EQ(pi.getKi(), 2.0);
    pi.compute(makeInput(1.0, 0.0));  // e=1, dt=0.01 -> I += 0.02
    EXPECT_NEAR(pi.getIntegral(), 0.02, 1e-9);
    pi.compute(makeInput(1.0, 0.0));
    EXPECT_NEAR(pi.getIntegral(), 0.04, 1e-9);
}

TEST(PIDControllersCoverage, PISetGainsFromTi) {
    PIController pi;
    pi.setGainsFromTi(4.0, 0.5);
    EXPECT_DOUBLE_EQ(pi.getKp(), 4.0);
    EXPECT_DOUBLE_EQ(pi.getKi(), 8.0);
}

TEST(PIDControllersCoverage, PIIntegralLimitsClamp) {
    PIController pi;
    pi.setGains(0.0, 1.0);
    pi.setIntegralLimits(-0.05, 0.05);
    for (int i = 0; i < 100; ++i) pi.compute(makeInput(1.0, 0.0));
    EXPECT_NEAR(pi.getIntegral(), 0.05, 1e-9);
    for (int i = 0; i < 200; ++i) pi.compute(makeInput(-1.0, 0.0));
    EXPECT_NEAR(pi.getIntegral(), -0.05, 1e-9);
}

TEST(PIDControllersCoverage, PIAntiWindupMethods) {
    // Each method must run without crashing through a saturating compute.
    const AntiWindupMethod methods[] = {
        AntiWindupMethod::None, AntiWindupMethod::Clamping,
        AntiWindupMethod::BackCalculation, AntiWindupMethod::Conditional,
        AntiWindupMethod::Tracking, AntiWindupMethod::ObserverBased};
    for (auto m : methods) {
        PIController pi;
        pi.setGains(1.0, 10.0);
        pi.setAntiWindup(m, 0.1);
        SaturationLimits lim;
        lim.outputMin = -1.0;
        lim.outputMax = 1.0;
        pi.setSaturationLimits(lim);
        for (int i = 0; i < 20; ++i) pi.compute(makeInput(10.0, 0.0));
        EXPECT_TRUE(std::isfinite(pi.getIntegral()));
    }
}

TEST(PIDControllersCoverage, PISetIntegralAndReset) {
    PIController pi;
    pi.setGains(0.0, 1.0);
    pi.setIntegral(3.0);
    EXPECT_DOUBLE_EQ(pi.getIntegral(), 3.0);
    pi.reset();
    EXPECT_DOUBLE_EQ(pi.getIntegral(), 0.0);
}

// ============================================================================
// PIDController: gains, autotune, derivative filters, anti-windup
// ============================================================================

TEST(PIDControllersCoverage, PIDGainsAndTimeConstants) {
    PIDController pid;
    pid.setGains(1.0, 2.0, 0.5);
    EXPECT_DOUBLE_EQ(pid.getKp(), 1.0);
    EXPECT_DOUBLE_EQ(pid.getKi(), 2.0);
    EXPECT_DOUBLE_EQ(pid.getKd(), 0.5);
    pid.setGainsFromTimeConstants(3.0, 0.5, 0.25);
    EXPECT_DOUBLE_EQ(pid.getKi(), 6.0);
    EXPECT_DOUBLE_EQ(pid.getKd(), 0.75);
}

TEST(PIDControllersCoverage, AutoTuneAllMethods) {
    const PIDController::TuningMethod methods[] = {
        PIDController::TuningMethod::ZieglerNichols,
        PIDController::TuningMethod::ZieglerNicholsPI,
        PIDController::TuningMethod::ZieglerNicholsPID,
        PIDController::TuningMethod::CohenCoon,
        PIDController::TuningMethod::Lambda,
        PIDController::TuningMethod::SIMC,
        PIDController::TuningMethod::AMIGO,
        PIDController::TuningMethod::TyreusLuyben};
    for (auto m : methods) {
        PIDController pid;
        pid.autoTune(1.5, 2.0, 0.3, m);
        EXPECT_TRUE(std::isfinite(pid.getKp()));
        EXPECT_TRUE(std::isfinite(pid.getKi()));
        EXPECT_TRUE(std::isfinite(pid.getKd()));
        EXPECT_GT(pid.getKp(), 0.0);
    }
    // Lambda with explicit lambda parameter
    PIDController pid;
    pid.autoTune(1.5, 2.0, 0.3, PIDController::TuningMethod::Lambda, 4.0);
    EXPECT_GT(pid.getKp(), 0.0);
    EXPECT_DOUBLE_EQ(pid.getKd(), 0.0);
}

TEST(PIDControllersCoverage, AutoTuneDegenerateInputs) {
    PIDController pid;
    pid.setGains(1.0, 1.0, 1.0);
    pid.autoTune(0.0, 2.0, 0.3);   // K == 0: early return
    EXPECT_DOUBLE_EQ(pid.getKp(), 1.0);
    pid.autoTune(1.5, 2.0, 0.0);   // theta == 0: early return
    EXPECT_DOUBLE_EQ(pid.getKp(), 1.0);
}

TEST(PIDControllersCoverage, AutoTuneFromUltimate) {
    const PIDController::TuningMethod methods[] = {
        PIDController::TuningMethod::ZieglerNichols,
        PIDController::TuningMethod::ZieglerNicholsPID,
        PIDController::TuningMethod::ZieglerNicholsPI,
        PIDController::TuningMethod::TyreusLuyben,
        PIDController::TuningMethod::CohenCoon,  // hits default: Z-N fallback
    };
    for (auto m : methods) {
        PIDController pid;
        pid.autoTuneFromUltimate(4.0, 2.0, m);
        EXPECT_GT(pid.getKp(), 0.0);
        EXPECT_TRUE(std::isfinite(pid.getKi()));
    }
}

TEST(PIDControllersCoverage, DerivativeFilterTypes) {
    const DerivativeFilterType types[] = {
        DerivativeFilterType::None, DerivativeFilterType::FirstOrder,
        DerivativeFilterType::SecondOrder, DerivativeFilterType::MovingAverage,
        DerivativeFilterType::Median};
    for (auto t : types) {
        PIDController pid;
        pid.setGains(0.0, 0.0, 1.0);
        pid.setDerivativeFilter(0.05);
        pid.setDerivativeFilterType(t);
        pid.compute(makeInput(0.0, 0.0));
        auto out = pid.compute(makeInput(0.0, 1.0));
        EXPECT_TRUE(std::isfinite(out.derivative));
    }
}

TEST(PIDControllersCoverage, DerivativeOnErrorAndUnfiltered) {
    PIDController pid;
    pid.setGains(0.0, 0.0, 1.0);
    pid.setDerivativeOnMeasurement(false);
    pid.compute(makeInput(0.0, 0.0));
    auto out = pid.compute(makeInput(1.0, 0.0));
    EXPECT_NEAR(out.derivative, 100.0, 1e-6);  // unfiltered, m_tf == 0
}

TEST(PIDControllersCoverage, PIDAntiWindupAllMethods) {
    const AntiWindupMethod methods[] = {
        AntiWindupMethod::None, AntiWindupMethod::Clamping,
        AntiWindupMethod::BackCalculation, AntiWindupMethod::Conditional,
        AntiWindupMethod::Tracking, AntiWindupMethod::ObserverBased};
    for (auto m : methods) {
        PIDController pid;
        pid.setGains(1.0, 5.0, 0.1);
        pid.setAntiWindup(m, 0.1);
        SaturationLimits lim;
        lim.outputMin = -1.0;
        lim.outputMax = 1.0;
        pid.setSaturationLimits(lim);
        for (int i = 0; i < 30; ++i) pid.compute(makeInput(50.0, 0.0));
        EXPECT_TRUE(std::isfinite(pid.getIntegral()));
        // drive error negative to unwind / exercise sign checks
        for (int i = 0; i < 30; ++i) pid.compute(makeInput(-50.0, 0.0));
        EXPECT_TRUE(std::isfinite(pid.getIntegral()));
    }
}

TEST(PIDControllersCoverage, AntiWindupBackCalcDefaults) {
    // param == 0 -> default Tt = sqrt(kd/ki), and the 1/kp fallback when
    // that is non-positive (kd==0).
    PIDController pid;
    pid.setGains(2.0, 4.0, 0.25);
    pid.setAntiWindup(AntiWindupMethod::BackCalculation, 0.0);
    SaturationLimits lim;
    lim.outputMin = -0.5;
    lim.outputMax = 0.5;
    pid.setSaturationLimits(lim);
    pid.compute(makeInput(10.0, 0.0));
    EXPECT_TRUE(std::isfinite(pid.getIntegral()));

    PIDController pid2;
    pid2.setGains(2.0, 4.0, 0.0);  // kd == 0 -> Tt fallback 1/kp
    pid2.setAntiWindup(AntiWindupMethod::BackCalculation, 0.0);
    pid2.setSaturationLimits(lim);
    pid2.compute(makeInput(10.0, 0.0));
    EXPECT_TRUE(std::isfinite(pid2.getIntegral()));
}

TEST(PIDControllersCoverage, AntiWindupConditionalLargeErrorGate) {
    // |error| > param skips integration entirely.
    PIDController pid;
    pid.setGains(0.0, 1.0, 0.0);
    pid.setAntiWindup(AntiWindupMethod::Conditional, 5.0);
    pid.compute(makeInput(10.0, 0.0));  // |e|=10 > 5: no integration
    EXPECT_DOUBLE_EQ(pid.getIntegral(), 0.0);
    pid.compute(makeInput(2.0, 0.0));   // |e|=2 < 5: integrates
    EXPECT_GT(pid.getIntegral(), 0.0);
}

TEST(PIDControllersCoverage, PIDSaturationFlag) {
    PIDController pid;
    pid.setGains(10.0, 0.0, 0.0);
    SaturationLimits lim;
    lim.outputMin = -5.0;
    lim.outputMax = 5.0;
    pid.setSaturationLimits(lim);
    auto out = pid.compute(makeInput(100.0, 0.0));
    EXPECT_TRUE(out.saturated);
    EXPECT_DOUBLE_EQ(out.control, 5.0);
}

TEST(PIDControllersCoverage, PIDReset) {
    PIDController pid;
    pid.setGains(1.0, 1.0, 1.0);
    pid.compute(makeInput(1.0, 0.0));
    pid.compute(makeInput(1.0, 0.5));
    pid.reset();
    EXPECT_DOUBLE_EQ(pid.getIntegral(), 0.0);
    auto out = pid.compute(makeInput(0.0, 9.0));
    EXPECT_DOUBLE_EQ(out.derivative, 0.0);  // first run after reset
}

// ============================================================================
// PID2DOF
// ============================================================================

TEST(PIDControllersCoverage, PID2DOFWeights) {
    PID2DOFController pid;
    pid.setGains(1.0, 0.0, 1.0);
    pid.setSetpointWeights(0.5, 0.0);
    pid.compute(makeInput(0.0, 0.0));
    auto out = pid.compute(makeInput(2.0, 1.0));
    // b=0.5 -> proportional acts on (b*ref - meas) = 0; c=0 -> deriv on meas
    EXPECT_TRUE(std::isfinite(out.control));
}

TEST(PIDControllersCoverage, PID2DOFWeightClamping) {
    PID2DOFController pid;
    pid.setSetpointWeights(2.0, -1.0);  // clamped to [0,1]
    pid.setGains(1.0, 0.0, 1.0);
    pid.compute(makeInput(1.0, 0.0));
    auto out = pid.compute(makeInput(1.0, 0.5));
    EXPECT_TRUE(std::isfinite(out.control));
}

// ============================================================================
// BangBang
// ============================================================================

TEST(PIDControllersCoverage, BangBangLevelsAndHysteresis) {
    BangBangController bb;
    bb.setOutputLevels(-1.0, 1.0);
    bb.setHysteresis(2.0);  // band +/-1
    auto out = bb.compute(makeInput(10.0, 0.0));  // e=10 > +1 -> max
    EXPECT_DOUBLE_EQ(out.control, 1.0);
    out = bb.compute(makeInput(10.0, 9.5));       // e=0.5 inside band: keep last
    EXPECT_DOUBLE_EQ(out.control, 1.0);
    out = bb.compute(makeInput(10.0, 12.0));      // e=-2 < -1 -> min
    EXPECT_DOUBLE_EQ(out.control, -1.0);
    out = bb.compute(makeInput(10.0, 10.5));      // inside band: keep -1
    EXPECT_DOUBLE_EQ(out.control, -1.0);
}

TEST(PIDControllersCoverage, BangBangNeutralOutput) {
    BangBangController bb;
    bb.setOutputLevels(-1.0, 1.0);
    bb.setHysteresis(2.0);
    bb.setNeutralOutput(0.25, true);
    auto out = bb.compute(makeInput(10.0, 10.5));  // inside band, no prior -> neutral
    EXPECT_DOUBLE_EQ(out.control, 0.25);
}

TEST(PIDControllersCoverage, BangBangReset) {
    BangBangController bb;
    bb.setOutputLevels(-1.0, 1.0);
    bb.setNeutralOutput(0.0, false);  // neutral disabled
    bb.compute(makeInput(10.0, 0.0));
    bb.reset();
    auto out = bb.compute(makeInput(10.0, 10.5));  // in band, state=false -> uMin
    EXPECT_DOUBLE_EQ(out.control, -1.0);
}

// ============================================================================
// PDPlus
// ============================================================================

TEST(PIDControllersCoverage, PDPlusCompensationCallback) {
    PDPlusController pd;
    pd.setGains(1.0, 0.0);
    pd.setCompensationOnDesired(false);  // evaluate comp on measured
    pd.setCompensationCallback([](double pos) { return 2.0 * pos; });
    auto out = pd.compute(makeInput(0.0, 3.0));
    EXPECT_DOUBLE_EQ(out.feedforward, 6.0);
    EXPECT_DOUBLE_EQ(out.control, -3.0 + 6.0);
}

TEST(PIDControllersCoverage, PDPlusFullCompensation) {
    PDPlusController pd;
    pd.setGains(0.0, 0.0);
    pd.setFullCompensationCallback(
        [](double pos, double vel) { return pos + vel; });
    pd.setCompensationOnDesired(false);
    pd.compute(makeInput(0.0, 1.0));
    auto out = pd.compute(makeInput(0.0, 2.0));
    EXPECT_DOUBLE_EQ(out.feedforward, 2.0 - 100.0);  // pos + deriv-of-meas
}

TEST(PIDControllersCoverage, PDPlusCompOnDesired) {
    PDPlusController pd;
    pd.setGains(0.0, 0.0);
    pd.setFullCompensationCallback(
        [](double pos, double vel) { return pos + vel; });
    pd.setCompensationOnDesired(true);
    ControllerInput in = makeInput(5.0, 1.0);
    in.referenceDerivative = 0.5;
    auto out = pd.compute(in);
    EXPECT_DOUBLE_EQ(out.feedforward, 5.5);
}

TEST(PIDControllersCoverage, PDPlusReset) {
    PDPlusController pd;
    pd.setGains(0.0, 1.0);
    pd.compute(makeInput(0.0, 0.0));
    pd.compute(makeInput(0.0, 1.0));
    pd.reset();
    auto out = pd.compute(makeInput(0.0, 5.0));
    EXPECT_DOUBLE_EQ(out.derivative, 0.0);
}

// ============================================================================
// DualLoopPID
// ============================================================================

TEST(PIDControllersCoverage, DualLoopCascade) {
    DualLoopPIDController c;
    c.setOuterGains(10.0, 0.0, 0.0);
    c.setInnerGains(1.0, 0.5, 0.0);
    c.setVelocityLimits(-100.0, 100.0);
    c.setVelocityFeedback(2.0);
    auto out = c.compute(makeInput(1.0, 0.0));
    // outer: kp*(1-0)=10 -> clamped to 100 -> vel cmd 10
    // inner: kp*(10-2)=8 + ki*8*dt
    EXPECT_TRUE(std::isfinite(out.control));
    EXPECT_DOUBLE_EQ(out.error, 1.0);
    EXPECT_DOUBLE_EQ(out.proportional, 10.0);
}

TEST(PIDControllersCoverage, DualLoopVelocityClampAndTorqueLimits) {
    DualLoopPIDController c;
    c.setOuterGains(1000.0, 0.0, 0.0);
    c.setInnerGains(10.0, 0.0, 0.0);
    c.setVelocityLimits(-5.0, 5.0);
    c.setTorqueLimits(-2.0, 2.0);
    auto out = c.compute(makeInput(10.0, 0.0));
    // vel cmd clamped to 5, inner torque clamped to 2
    EXPECT_DOUBLE_EQ(out.control, 2.0);
    EXPECT_TRUE(c.getInnerController().getLastOutput().saturated);
}

TEST(PIDControllersCoverage, DualLoopAntiWindupAndAccessors) {
    DualLoopPIDController c;
    c.setOuterAntiWindup(AntiWindupMethod::BackCalculation, 0.2);
    c.setInnerAntiWindup(AntiWindupMethod::Conditional, 0.2);
    EXPECT_EQ(c.getOuterController().getAntiWindup(),
              AntiWindupMethod::BackCalculation);
    EXPECT_EQ(c.getInnerController().getAntiWindup(),
              AntiWindupMethod::Conditional);
    c.setOuterGains(1.0, 1.0, 0.0);
    c.setInnerGains(1.0, 1.0, 0.0);
    c.compute(makeInput(1.0, 0.0));
    c.reset();
    auto out = c.compute(makeInput(0.0, 0.0));
    EXPECT_DOUBLE_EQ(out.control, 0.0);
}

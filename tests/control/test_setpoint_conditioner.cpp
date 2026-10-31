// SetpointConditioner: refresh gating, law semantics (ZOH / first-order
// lag / linear ramp / slew rate), custom laws, boundary validation.
#include <gtest/gtest.h>

#include "tether/control/SetpointConditioner.hpp"

using namespace tether::control;

using Vec = std::vector<double>;
using CondD = SetpointConditioner<double>;
using CondV = SetpointConditioner<Vec>;

// Custom Value type exercising the ConditionerOps extension point.
struct Vec3 {
  double v[3] = {0, 0, 0};
  bool operator==(const Vec3&) const = default;
};

namespace tether::control {
template <>
struct ConditionerOps<Vec3> {
  static Vec3 lerp(const Vec3& from, const Vec3& to, double t) {
    Vec3 out;
    for (int i = 0; i < 3; ++i)
      out.v[i] = ConditionerOps<double>::lerp(from.v[i], to.v[i], t);
    return out;
  }
  static Vec3 approach(const Vec3& from, const Vec3& to, double d) {
    Vec3 out;
    for (int i = 0; i < 3; ++i)
      out.v[i] = ConditionerOps<double>::approach(from.v[i], to.v[i], d);
    return out;
  }
};
}  // namespace tether::control

namespace {

using Cond3 = SetpointConditioner<Vec3>;

TEST(SetpointConditioner, ZeroOrderHoldSnapsImmediately) {
  CondD c(std::make_unique<ZeroOrderHold<double>>(), 4, 1.0);
  EXPECT_DOUBLE_EQ(c.step(), 1.0);  // initial before any refresh
  EXPECT_TRUE(c.refreshDue(0));
  EXPECT_FALSE(c.refreshDue(1));
  c.refresh(5.0);
  EXPECT_DOUBLE_EQ(c.step(), 5.0);
  EXPECT_DOUBLE_EQ(c.step(), 5.0);  // held between refreshes
  c.refresh(-2.0);
  EXPECT_DOUBLE_EQ(c.step(), -2.0);
}

TEST(SetpointConditioner, UpdateFusesGateAndProduce) {
  CondD c(std::make_unique<ZeroOrderHold<double>>(), 3, 0.0);
  int produced = 0;
  auto produce = [&] {
    ++produced;
    return 10.0 * produced;
  };
  EXPECT_DOUBLE_EQ(c.update(0, produce), 10.0);  // due: produce + emit
  EXPECT_DOUBLE_EQ(c.update(1, produce), 10.0);  // held
  EXPECT_DOUBLE_EQ(c.update(2, produce), 10.0);  // held
  EXPECT_DOUBLE_EQ(c.update(3, produce), 20.0);  // next due tick
  EXPECT_EQ(produced, 2);
}

TEST(SetpointConditioner, FirstOrderLagApproachesExponentially) {
  CondD c(std::make_unique<FirstOrderLag<double>>(0.5), 1, 0.0);
  c.refresh(10.0);
  EXPECT_DOUBLE_EQ(c.step(), 5.0);
  EXPECT_DOUBLE_EQ(c.step(), 7.5);
  EXPECT_DOUBLE_EQ(c.step(), 8.75);
  // α=1 degenerates to ZOH.
  CondD snap(std::make_unique<FirstOrderLag<double>>(1.0), 1, 0.0);
  snap.refresh(10.0);
  EXPECT_DOUBLE_EQ(snap.step(), 10.0);
}

TEST(SetpointConditioner, LinearRampReachesTargetOverRampTicks) {
  CondD c(std::make_unique<LinearRampHold<double>>(4), 1, 0.0);
  c.refresh(8.0);
  EXPECT_DOUBLE_EQ(c.step(), 2.0);
  EXPECT_DOUBLE_EQ(c.step(), 4.0);
  EXPECT_DOUBLE_EQ(c.step(), 6.0);
  EXPECT_DOUBLE_EQ(c.step(), 8.0);
  EXPECT_DOUBLE_EQ(c.step(), 8.0);  // clamps at target
}

TEST(SetpointConditioner, LinearRampMidRampRefreshStaysContinuous) {
  auto law = std::make_unique<LinearRampHold<double>>(4);
  CondD c(std::move(law), 1, 0.0);
  c.refresh(8.0);
  EXPECT_DOUBLE_EQ(c.step(), 2.0);
  EXPECT_DOUBLE_EQ(c.step(), 4.0);
  // New target lands mid-ramp: continues from current output (4.0), no jump.
  c.refresh(12.0);
  const double next = c.step();
  EXPECT_GT(next, 4.0);
  EXPECT_LT(next, 12.0);
  EXPECT_DOUBLE_EQ(next, 4.0 + (12.0 - 4.0) / 4.0);
}

TEST(SetpointConditioner, SlewRateBoundsPerStepDelta) {
  CondD c(std::make_unique<SlewRateHold<double>>(1.5), 1, 0.0);
  c.refresh(10.0);
  EXPECT_DOUBLE_EQ(c.step(), 1.5);
  EXPECT_DOUBLE_EQ(c.step(), 3.0);
  // Negative direction and eventual arrival.
  c.refresh(-10.0);
  EXPECT_DOUBLE_EQ(c.step(), 1.5);
  for (int i = 0; i < 10; ++i) c.step();
  EXPECT_DOUBLE_EQ(c.step(), -10.0);
}

TEST(SetpointConditioner, VectorLawsAreElementwise) {
  CondV c(std::make_unique<FirstOrderLag<Vec>>(0.5), 1, Vec{0, 0, 0});
  c.refresh(Vec{10, -10, 4});
  const Vec& v = c.step();
  EXPECT_DOUBLE_EQ(v[0], 5.0);
  EXPECT_DOUBLE_EQ(v[1], -5.0);
  EXPECT_DOUBLE_EQ(v[2], 2.0);
}

TEST(SetpointConditioner, MultiTargetProducerUsesExplicitRefresh) {
  // One gate, several conditioners with different laws — the multi-member
  // pattern: positions smoothed, gains snapped.
  CondV pos(std::make_unique<FirstOrderLag<Vec>>(0.5), 2, Vec{0, 0});
  CondV gains(std::make_unique<ZeroOrderHold<Vec>>(), 2, Vec{1, 1});
  double p = 0.0, g = 0.0;
  for (uint64_t tick = 0; tick < 6; ++tick) {
    if (pos.refreshDue(tick)) {  // one check — same divisor for all members
      pos.refresh(Vec{100, 100});
      gains.refresh(Vec{9, 9});
    }
    p = pos.step()[0];
    g = gains.step()[0];
  }
  // 6 lag steps at α=0.5 from 0 toward 100: 100·(1−2⁻⁶).
  EXPECT_DOUBLE_EQ(p, 100.0 * (1.0 - 1.0 / 64.0));
  EXPECT_DOUBLE_EQ(g, 9.0);  // snapped on the first refresh
}

// User-defined law via virtual interface: a "deadband" hold that ignores
// target changes smaller than 0.5.
class DeadbandHold final : public IConditionLaw<double> {
 public:
  const double& step(const double& target) override {
    if (std::abs(target - out_) >= 0.5) out_ = target;
    return out_;
  }
  void reset(const double& initial) override { out_ = initial; }

 private:
  double out_{};
};

TEST(SetpointConditioner, UserDefinedLawPlugsIn) {
  CondD c(std::make_unique<DeadbandHold>(), 1, 0.0);
  c.refresh(0.3);
  EXPECT_DOUBLE_EQ(c.step(), 0.0);   // below deadband: ignored
  c.refresh(0.6);
  EXPECT_DOUBLE_EQ(c.step(), 0.6);   // above: snaps
}

TEST(SetpointConditioner, ResetReanchorsLawAndTarget) {
  CondD c(std::make_unique<FirstOrderLag<double>>(0.5), 1, 0.0);
  c.refresh(10.0);
  c.step();
  c.reset(3.0);
  EXPECT_DOUBLE_EQ(c.step(), 3.0);  // re-anchored, not converging to 10
  EXPECT_DOUBLE_EQ(c.target(), 3.0);
}

TEST(SetpointConditioner, RampTicksOneEqualsZeroOrderHold) {
  CondD c(std::make_unique<LinearRampHold<double>>(1), 1, 0.0);
  c.refresh(8.0);
  EXPECT_DOUBLE_EQ(c.step(), 8.0);  // single-step ramp = snap
  c.refresh(-3.0);
  EXPECT_DOUBLE_EQ(c.step(), -3.0);
}

TEST(SetpointConditioner, SlewRateArrivesExactlyWhenDeltaDivides) {
  CondD c(std::make_unique<SlewRateHold<double>>(2.0), 1, 0.0);
  c.refresh(8.0);
  EXPECT_DOUBLE_EQ(c.step(), 2.0);
  EXPECT_DOUBLE_EQ(c.step(), 4.0);
  EXPECT_DOUBLE_EQ(c.step(), 6.0);
  EXPECT_DOUBLE_EQ(c.step(), 8.0);   // lands exactly
  EXPECT_DOUBLE_EQ(c.step(), 8.0);   // and stays
}

TEST(SetpointConditioner, DivisorOneRefreshesEveryTick) {
  CondD c(std::make_unique<ZeroOrderHold<double>>(), 1, 0.0);
  for (uint64_t tick = 0; tick < 5; ++tick)
    EXPECT_TRUE(c.refreshDue(tick));
}

TEST(SetpointConditioner, RefreshBeforeFirstStepIsHandled) {
  // Two refreshes without a step in between: the second refresh anchors
  // off the last output (still the initial value), not the first target.
  auto law = std::make_unique<LinearRampHold<double>>(4);
  CondD c(std::move(law), 1, 2.0);
  c.refresh(10.0);
  c.refresh(10.0);
  const double v = c.step();
  EXPECT_GT(v, 2.0);
  EXPECT_LT(v, 10.0);  // ramping from 2.0, not from 10.0
}

TEST(SetpointConditioner, InstancesAreIndependent) {
  CondD a(std::make_unique<FirstOrderLag<double>>(0.5), 1, 0.0);
  CondD b(std::make_unique<FirstOrderLag<double>>(0.5), 1, 0.0);
  a.refresh(10.0);
  b.refresh(-10.0);
  EXPECT_DOUBLE_EQ(a.step(), 5.0);
  EXPECT_DOUBLE_EQ(b.step(), -5.0);
}

TEST(SetpointConditioner, MoveConstructionPreservesState) {
  CondD a(std::make_unique<FirstOrderLag<double>>(0.5), 2, 1.0);
  a.refresh(5.0);
  a.step();
  CondD b(std::move(a));
  EXPECT_DOUBLE_EQ(b.target(), 5.0);
  EXPECT_TRUE(b.refreshDue(0));
  EXPECT_FALSE(b.refreshDue(1));
}

TEST(SetpointConditioner, CustomValueTypeViaOpsSpecialization) {
  Cond3 c(std::make_unique<FirstOrderLag<Vec3>>(0.5), 1, Vec3{});
  Vec3 t{{4.0, -4.0, 2.0}};
  c.refresh(t);
  const Vec3& out = c.step();
  EXPECT_DOUBLE_EQ(out.v[0], 2.0);
  EXPECT_DOUBLE_EQ(out.v[1], -2.0);
  EXPECT_DOUBLE_EQ(out.v[2], 1.0);
}

TEST(SetpointConditioner, LargeTickValuesDoNotOverflow) {
  CondD c(std::make_unique<ZeroOrderHold<double>>(), 7, 0.0);
  const uint64_t big = ~uint64_t{0} - 3;
  EXPECT_EQ(c.refreshDue(big), big % 7 == 0);  // modulo on huge ticks
  EXPECT_EQ(c.refreshDue(big - (big % 7)), true);
}

TEST(SetpointConditioner, EmittedRefValidUntilNextStep) {
  CondD c(std::make_unique<FirstOrderLag<double>>(0.5), 1, 0.0);
  c.refresh(10.0);
  const double& r1 = c.step();
  const double v1 = r1;
  const double& r2 = c.step();
  EXPECT_DOUBLE_EQ(v1, 5.0);
  EXPECT_DOUBLE_EQ(r2, 7.5);  // second step advanced; r1 was read before it
}

TEST(SetpointConditioner, BoundaryValidation) {
  EXPECT_THROW(CondD(nullptr, 1, 0.0), std::invalid_argument);
  EXPECT_THROW(CondD(std::make_unique<ZeroOrderHold<double>>(), 0, 0.0),
               std::invalid_argument);
  EXPECT_THROW(std::make_unique<FirstOrderLag<double>>(0.0),
               std::invalid_argument);
  EXPECT_THROW(std::make_unique<FirstOrderLag<double>>(1.5),
               std::invalid_argument);
  EXPECT_THROW(std::make_unique<LinearRampHold<double>>(0),
               std::invalid_argument);
  EXPECT_THROW(std::make_unique<SlewRateHold<double>>(0.0),
               std::invalid_argument);
  // Vector size drift across refresh is a contract violation.
  CondV c(std::make_unique<FirstOrderLag<Vec>>(0.5), 1, Vec{0, 0});
  c.refresh(Vec{1, 2, 3});
  EXPECT_THROW((void)c.step(), std::invalid_argument);
}

}  // namespace

// SetpointConditioner — bridge a slow setpoint producer to a fast consumer.
//
// A target refreshed at a low cadence (every Nth tick, or on any producer
// event) is emitted every tick through a pluggable condition law:
//
//   ZeroOrderHold    snap to the newest target (the classic sample&hold)
//   FirstOrderLag    held += α·(target − held); α=1 degenerates to ZOH
//   LinearRampHold   linear move from the last output to the target over
//                    a fixed tick count (a simple trajectory generator)
//   SlewRateHold     move ≤ maxDelta per step toward the target
//   user-defined     subclass IConditionLaw<Value>
//
// Per-channel behavior is per-Value: use one conditioner per member (e.g.
// positions smoothed, gains snapped) or a custom Value type with a
// ConditionerOps specialization.
//
// Assumptions checked at boundaries:
//   * refresh divisor ≥ 1, α ∈ (0,1], ramp ticks ≥ 1, slew delta > 0.
//   * Vector targets must keep a constant size across refreshes
//     (std::invalid_argument on drift — silent resizing would mask a
//     dimension bug upstream).
//   * The law pointer must be non-null (std::invalid_argument).
//
// Threading: single-threaded per instance. The emitted reference is valid
// until the next refresh()/step() call — copy it when crossing threads.
#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <memory>
#include <stdexcept>
#include <vector>

namespace tether::control {

// ---- Value arithmetic ------------------------------------------------------
// The laws need only two vector-space ops. Specialize for custom Value
// types (both must be defined together).
template <typename V>
struct ConditionerOps;  // undefined: forces a specialization

// out = from + t·(to − from); out = move ≤maxDelta per element toward to.
template <>
struct ConditionerOps<double> {
  static double lerp(double from, double to, double t) {
    return from + t * (to - from);
  }
  static double approach(double from, double to, double maxDelta) {
    const double d = to - from;
    return from + std::clamp(d, -maxDelta, maxDelta);
  }
};

// Element-wise; size drift is a contract violation, reported loudly.
template <>
struct ConditionerOps<std::vector<double>> {
  static std::vector<double> lerp(const std::vector<double>& from,
                                  const std::vector<double>& to, double t) {
    checkSize(from, to);
    std::vector<double> out(from.size());
    for (size_t i = 0; i < out.size(); ++i)
      out[i] = ConditionerOps<double>::lerp(from[i], to[i], t);
    return out;
  }
  static std::vector<double> approach(const std::vector<double>& from,
                                      const std::vector<double>& to,
                                      double maxDelta) {
    checkSize(from, to);
    std::vector<double> out(from.size());
    for (size_t i = 0; i < out.size(); ++i)
      out[i] = ConditionerOps<double>::approach(from[i], to[i], maxDelta);
    return out;
  }
  static void checkSize(const std::vector<double>& a,
                        const std::vector<double>& b) {
    if (a.size() != b.size())
      throw std::invalid_argument("SetpointConditioner: value size drift");
  }
};

// ---- Law interface ---------------------------------------------------------

// Governs the emitted value between refreshes. One law instance per
// conditioner; methods run on the conditioner's thread, exactly once each
// per refresh()/step().
template <typename Value>
class IConditionLaw {
 public:
  virtual ~IConditionLaw() = default;
  // A fresh target landed (called before the step() that emits it).
  // `previous` is the target being replaced. Default: ignore.
  virtual void refreshed(const Value& previous) { (void)previous; }
  // Emit this tick's conditioned value. The returned reference stays
  // valid until the next step()/refreshed() on this law.
  virtual const Value& step(const Value& target) = 0;
  // Re-anchor internal state at `initial` (e.g. re-prime from a measured
  // state so the output resumes smoothly).
  virtual void reset(const Value& initial) = 0;
};

// Snap to the newest target. Stateless.
template <typename Value>
class ZeroOrderHold final : public IConditionLaw<Value> {
 public:
  const Value& step(const Value& target) override { return target; }
  void reset(const Value&) override {}
};

// Exponential approach: out += α·(target − out). α=1 equals ZOH.
template <typename Value>
class FirstOrderLag final : public IConditionLaw<Value> {
 public:
  explicit FirstOrderLag(double alpha) : alpha_(alpha) {
    if (!(alpha > 0.0 && alpha <= 1.0) || !std::isfinite(alpha))
      throw std::invalid_argument("FirstOrderLag: alpha must be in (0,1]");
  }
  const Value& step(const Value& target) override {
    out_ = ConditionerOps<Value>::lerp(out_, target, alpha_);
    return out_;
  }
  void reset(const Value& initial) override { out_ = initial; }
  double alpha() const { return alpha_; }

 private:
  double alpha_;
  Value out_{};
};

// Linear move from the output at refresh time to the target, over
// `rampTicks` steps. Starting from the last output keeps the signal
// continuous even when a refresh lands mid-ramp.
template <typename Value>
class LinearRampHold final : public IConditionLaw<Value> {
 public:
  explicit LinearRampHold(uint64_t rampTicks) : rampTicks_(rampTicks) {
    if (rampTicks < 1)
      throw std::invalid_argument("LinearRampHold: rampTicks must be >= 1");
  }
  void refreshed(const Value&) override {
    src_ = out_;
    t_ = 0;
  }
  const Value& step(const Value& target) override {
    ++t_;
    const double f =
        std::min(1.0, static_cast<double>(t_) / static_cast<double>(rampTicks_));
    out_ = ConditionerOps<Value>::lerp(src_, target, f);
    return out_;
  }
  void reset(const Value& initial) override {
    src_ = initial;
    out_ = initial;
    t_ = rampTicks_;  // settled: next step emits the target
  }

 private:
  uint64_t rampTicks_;
  uint64_t t_ = 0;
  Value src_{}, out_{};
};

// Move ≤ maxDelta per step toward the target (per element). A torque/slew
// rate limiter: bounds how fast the emitted value may change regardless of
// target jumps.
template <typename Value>
class SlewRateHold final : public IConditionLaw<Value> {
 public:
  explicit SlewRateHold(double maxDeltaPerStep) : maxDelta_(maxDeltaPerStep) {
    if (!(maxDeltaPerStep > 0.0) || !std::isfinite(maxDeltaPerStep))
      throw std::invalid_argument("SlewRateHold: maxDeltaPerStep must be > 0");
  }
  const Value& step(const Value& target) override {
    out_ = ConditionerOps<Value>::approach(out_, target, maxDelta_);
    return out_;
  }
  void reset(const Value& initial) override { out_ = initial; }

 private:
  double maxDelta_;
  Value out_{};
};

// ---- Conditioner -----------------------------------------------------------

// Owns the refresh gate and the law. Producer-side decimation is
// refreshDue(): tick % divisor == 0 — check it once per tick and push the
// produced bundle via refresh() per member, or use update() for the common
// single-target case.
template <typename Value>
class SetpointConditioner {
 public:
  // `law` must be non-null; `refreshDivisor` >= 1 (1 = refresh every tick).
  // `initial` seeds both the target and the law state so step() emits a
  // sane value before the first refresh.
  SetpointConditioner(std::unique_ptr<IConditionLaw<Value>> law,
                      uint64_t refreshDivisor = 1, Value initial = {})
      : law_(std::move(law)),
        divisor_(refreshDivisor),
        target_(std::move(initial)) {
    if (!law_)
      throw std::invalid_argument("SetpointConditioner: null law");
    if (divisor_ < 1)
      throw std::invalid_argument("SetpointConditioner: refreshDivisor < 1");
    law_->reset(target_);
  }

  SetpointConditioner(const SetpointConditioner&) = delete;
  SetpointConditioner& operator=(const SetpointConditioner&) = delete;

  // Producer gate: is a refresh due on this tick?
  bool refreshDue(uint64_t tick) const { return tick % divisor_ == 0; }

  // Push a fresh target. Call on due ticks — or on any producer event;
  // the conditioner does not care why targets refresh.
  void refresh(Value target) {
    law_->refreshed(target_);
    target_ = std::move(target);
  }

  // Emit this tick's conditioned value (once per consumer tick).
  const Value& step() { return law_->step(target_); }

  // Fused single-target form: produce a fresh target on due ticks, always
  // step. Equivalent to: if (refreshDue(tick)) refresh(produce()); step().
  template <typename ProduceFn>
  const Value& update(uint64_t tick, ProduceFn&& produce) {
    if (refreshDue(tick)) refresh(produce());
    return step();
  }

  // Re-anchor target + law state (e.g. to a measured state on restart).
  void reset(const Value& initial) {
    target_ = initial;
    law_->reset(initial);
  }

  const Value& target() const { return target_; }
  const IConditionLaw<Value>& law() const { return *law_; }
  uint64_t refreshDivisor() const { return divisor_; }

 private:
  std::unique_ptr<IConditionLaw<Value>> law_;
  uint64_t divisor_;
  Value target_;
};

}  // namespace tether::control

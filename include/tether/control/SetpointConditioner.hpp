/**
 * @file SetpointConditioner.hpp
 * @brief Bridge a slow setpoint producer to a fast consumer through a
 *        pluggable condition law
 *
 * A target refreshed at a low cadence (every Nth tick, or on any producer
 * event) is emitted every tick through a selectable IConditionLaw:
 *
 *   - @ref ZeroOrderHold   — snap to the newest target (the classic
 *                            sample&hold)
 *   - @ref FirstOrderLag   — held += α·(target − held); α=1 degenerates
 *                            to ZOH
 *   - @ref LinearRampHold  — linear move from the last output to the
 *                            target over a fixed tick count (a simple
 *                            trajectory generator)
 *   - @ref SlewRateHold    — move ≤ maxDelta per step toward the target
 *   - user-defined         — subclass @ref IConditionLaw
 *
 * Per-channel behavior is per-Value: use one conditioner per member (e.g.
 * positions smoothed, gains snapped) or a custom Value type with a
 * @ref ConditionerOps specialization.
 *
 * ## Usage
 *
 * @code
 *   using Cond = SetpointConditioner<std::vector<double>>;
 *   Cond pos(std::make_unique<FirstOrderLag<std::vector<double>>>(0.5),
 *            /*divisor=* /20, initial);
 *   Cond gains(std::make_unique<ZeroOrderHold<std::vector<double>>(),
 *              20, initial);
 *
 *   // fast loop:
 *   if (pos.refreshDue(tick)) {         // one gate check for all members
 *       auto bundle = produce();
 *       pos.refresh(bundle.pos);
 *       gains.refresh(bundle.gains);
 *   }
 *   const auto& q_des = pos.step();     // smoothed emission this tick
 *   const auto& kp    = gains.step();   // snapped emission this tick
 * @endcode
 *
 * ## Assumptions checked at boundaries
 *
 *   - refresh divisor ≥ 1, α ∈ (0,1], ramp ticks ≥ 1, slew delta > 0.
 *   - Vector targets must keep a constant size across refreshes
 *     (std::invalid_argument on drift — silent resizing would mask a
 *     dimension bug upstream).
 *   - The law pointer must be non-null (std::invalid_argument).
 *
 * ## Threading
 *
 * Single-threaded per instance. The emitted reference is valid until the
 * next refresh()/step() call — copy it when crossing threads.
 */
#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <memory>
#include <stdexcept>
#include <vector>

namespace tether::control {

// ---- Value arithmetic ------------------------------------------------------

/**
 * @brief Vector-space ops the laws need.
 *
 * Specialize both members together for a custom Value type:
 * lerp(from,to,t) = from + t·(to − from); approach(from,to,d) = move ≤ d
 * per element toward `to`.
 *
 * @tparam V  value type (double and std::vector<double> are provided)
 */
template <typename V>
struct ConditionerOps;  // undefined: forces a specialization

/// @copydoc ConditionerOps
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

/// @copydoc ConditionerOps
/// Element-wise; size drift is a contract violation, reported loudly.
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

/**
 * @brief Governs the emitted value between refreshes.
 *
 * One law instance per conditioner; methods run on the conditioner's
 * thread, exactly once each per refresh()/step().
 *
 * @tparam Value  target/output type
 */
template <typename Value>
class IConditionLaw {
 public:
  virtual ~IConditionLaw() = default;
  /**
   * @brief A fresh target landed (called before the step() emitting it).
   * @param previous  the target being replaced
   *
   * Default: ignore. Laws that ramp/track transitions (e.g.
   * LinearRampHold) anchor their internal state here.
   */
  virtual void refreshed(const Value& previous) { (void)previous; }
  /**
   * @brief Emit this tick's conditioned value.
   * @param target  newest refreshed target
   * @return reference valid until the next step()/refreshed() on this law
   */
  virtual const Value& step(const Value& target) = 0;
  /**
   * @brief Re-anchor internal state at `initial`.
   * @param initial  value the law should emit/converge from
   *
   * E.g. re-prime from a measured state so the output resumes smoothly.
   */
  virtual void reset(const Value& initial) = 0;
};

/**
 * @brief Snap to the newest target. Stateless.
 * @tparam Value  any type; no ConditionerOps needed
 */
template <typename Value>
class ZeroOrderHold final : public IConditionLaw<Value> {
 public:
  const Value& step(const Value& target) override { return target; }
  void reset(const Value&) override {}
};

/**
 * @brief Exponential approach: out += α·(target − out). α=1 equals ZOH.
 * @tparam Value  needs ConditionerOps<Value>::lerp
 */
template <typename Value>
class FirstOrderLag final : public IConditionLaw<Value> {
 public:
  /// @param alpha  blend factor in (0,1]; 1 = snap, smaller = smoother
  /// @throw std::invalid_argument  α outside (0,1] or non-finite
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

/**
 * @brief Linear move from the output at refresh time to the target, over
 *        `rampTicks` steps.
 *
 * Starting from the last output (not the previous target) keeps the
 * signal continuous even when a refresh lands mid-ramp. rampTicks=1
 * degenerates to ZOH.
 *
 * @tparam Value  needs ConditionerOps<Value>::lerp
 */
template <typename Value>
class LinearRampHold final : public IConditionLaw<Value> {
 public:
  /// @param rampTicks  steps to reach the target; ≥ 1
  /// @throw std::invalid_argument  rampTicks < 1
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

/**
 * @brief Move ≤ maxDelta per step toward the target (per element).
 *
 * A torque/slew rate limiter: bounds how fast the emitted value may
 * change regardless of target jumps.
 *
 * @tparam Value  needs ConditionerOps<Value>::approach
 */
template <typename Value>
class SlewRateHold final : public IConditionLaw<Value> {
 public:
  /// @param maxDeltaPerStep  max |Δ| per step, > 0
  /// @throw std::invalid_argument  non-positive or non-finite delta
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

/**
 * @brief Owns the refresh gate and the law.
 *
 * Producer-side decimation is refreshDue() (`tick % divisor == 0`):
 * check it once per tick and push the produced bundle via refresh() per
 * member, or use update() for the common single-target case.
 *
 * @tparam Value  target/output type (e.g. double, std::vector<double>,
 *                or a custom type with a ConditionerOps specialization)
 */
template <typename Value>
class SetpointConditioner {
 public:
  /**
   * @param law            hold law (must be non-null)
   * @param refreshDivisor  refresh gate divisor; ≥ 1 (1 = every tick)
   * @param initial        seeds both the target and the law state so
   *                       step() emits a sane value before the first
   *                       refresh
   * @throw std::invalid_argument  null law or divisor < 1
   */
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
  SetpointConditioner(SetpointConditioner&&) = default;
  SetpointConditioner& operator=(SetpointConditioner&&) = default;

  /// @brief Producer gate: is a refresh due on this tick?
  /// @param tick  consumer's tick counter
  /// @return `tick % refreshDivisor == 0`
  bool refreshDue(uint64_t tick) const { return tick % divisor_ == 0; }

  /**
   * @brief Push a fresh target.
   * @param target  newest producer output (moved in)
   *
   * Call on due ticks — or on any producer event; the conditioner does
   * not care why targets refresh.
   */
  void refresh(Value target) {
    law_->refreshed(target_);
    target_ = std::move(target);
  }

  /// @brief Emit this tick's conditioned value (once per consumer tick).
  /// @return reference valid until the next refresh()/step()
  const Value& step() { return law_->step(target_); }

  /**
   * @brief Fused single-target form: produce a fresh target on due
   *        ticks, always step.
   * @param tick     consumer's tick counter
   * @param produce  callable returning a fresh Value; invoked only on
   *                 due ticks
   * @return this tick's conditioned value
   * @tparam ProduceFn  `Value() -> Value` callable
   *
   * Equivalent to: `if (refreshDue(tick)) refresh(produce()); step();`
   */
  template <typename ProduceFn>
  const Value& update(uint64_t tick, ProduceFn&& produce) {
    if (refreshDue(tick)) refresh(produce());
    return step();
  }

  /// @brief Re-anchor target + law state (e.g. to a measured state on
  ///        restart).
  /// @param initial  new target and law-state anchor
  void reset(const Value& initial) {
    target_ = initial;
    law_->reset(initial);
  }

  /// Newest refreshed target.
  const Value& target() const { return target_; }
  /// The installed condition law (read-only).
  const IConditionLaw<Value>& law() const { return *law_; }
  /// Refresh gate divisor (1 = every tick due).
  uint64_t refreshDivisor() const { return divisor_; }

 private:
  std::unique_ptr<IConditionLaw<Value>> law_;
  uint64_t divisor_;
  Value target_;
};

}  // namespace tether::control

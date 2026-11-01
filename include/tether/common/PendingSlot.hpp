/**
 * @file PendingSlot.hpp
 * @brief Latest-wins deferred hand-off with plesiosynchronicity statistics
 *
 * A PendingSlot<T> is a single-value mailbox between a producer and a
 * consumer that run on different schedules: post() parks the newest value,
 * take() retrieves and clears it at the consumer's next safe boundary
 * (e.g. the top of a control tick). Posting twice before a take keeps only
 * the newest value — "latest wins". The consumer pulls; nothing mutates
 * its state mid-tick.
 *
 * ## Plesiosynchronicity accounting
 *
 * The slot's base assumption is *plesiosynchronicity*: puts and gets
 * loosely alternate 1:1 — each post() eventually matched by exactly one
 * take(), and each take() finding something. Both directions of violation
 * are counted, because a broken assumption is a diagnosis, not a bug:
 *
 *   - **double-put-single-get** (producer lapped the consumer): a post()
 *     landing while a value is still pending drops the pending value and
 *     counts one @ref Stats::overruns. The stale value never reaches the
 *     consumer — usually fine for "latest wins" semantics, still worth
 *     seeing.
 *   - **double-get-single-put** (consumer starved): a take() finding the
 *     slot empty counts one @ref Stats::misses. The consumer sampled
 *     faster than the producer supplied.
 *   - Run-length maxima: @ref Stats::maxPutStreak /
 *     @ref Stats::maxGetStreak record the worst observed run of
 *     consecutive same-side operations, i.e. how far the phases can drift.
 *
 * Only take() counts as a get; hasPending() is a non-consuming probe and
 * does not perturb the model.
 *
 * ## Contracts
 *
 *   - T must be move-constructible (stored inside std::optional<T>).
 *   - All methods take a short mutex: safe to call from any thread pair,
 *     none block beyond the microseconds a single post()/take() holds it.
 *   - stats() returns a consistent snapshot taken under the same mutex —
 *     counters never disagree with each other within one call.
 *   - reset() drops the pending value AND zeroes all counters/streaks.
 */
#pragma once

#include <cstdint>
#include <mutex>
#include <optional>
#include <utility>

namespace tether::common {

/**
 * @brief Single-value mailbox with latest-wins semantics and put/get
 *        balance statistics.
 *
 * @tparam T  payload type; must be move-constructible. A default-
 *            constructible T is not required.
 */
template <typename T>
class PendingSlot {
 public:
  /**
   * @brief Operational counters describing the put/get balance.
   *
   * All fields are exact counts since construction or the last reset();
   * together they satisfy: puts = hits + overruns + pending (when the
   * pending slot is non-empty), gets = hits + misses.
   */
  struct Stats {
    uint64_t puts = 0; /**< total post() calls */
    uint64_t gets = 0; /**< total take() calls */
    uint64_t hits = 0; /**< takes that returned a value */
    uint64_t misses = 0; /**< takes that found the slot empty (starved get) */
    uint64_t overruns = 0; /**< posts that dropped an untaken value */
    uint64_t maxPutStreak = 0; /**< longest run of posts without a take */
    uint64_t maxGetStreak = 0; /**< longest run of takes without a post */
  };

  PendingSlot() = default;
  PendingSlot(const PendingSlot&) = delete;
  PendingSlot& operator=(const PendingSlot&) = delete;

  /**
   * @brief Park the newest value, dropping any untaken one.
   * @param value  payload to publish (moved in)
   *
   * If a value is already pending it is discarded and an overrun is
   * counted — the consumer only ever sees the newest.
   */
  void post(T value) {
    std::lock_guard lk(mtx_);
    if (pending_) ++overruns_;
    pending_ = std::move(value);
    ++puts_;
    ++putRun_;
    getRun_ = 0;
    if (putRun_ > maxPutStreak_) maxPutStreak_ = putRun_;
  }

  /**
   * @brief Retrieve and clear the pending value, if any.
   * @return the posted value, or std::nullopt when the slot was empty
   *
   * Every call counts as a get; an empty slot counts a miss. A returned
   * value is moved out — the slot is empty afterwards.
   */
  std::optional<T> take() {
    std::lock_guard lk(mtx_);
    ++gets_;
    ++getRun_;
    putRun_ = 0;
    if (getRun_ > maxGetStreak_) maxGetStreak_ = getRun_;
    if (pending_) {
      ++hits_;
      auto out = std::move(*pending_);
      pending_.reset();  // moved-from optional stays engaged; clear it
      return out;
    }
    ++misses_;
    return std::nullopt;
  }

  /**@brief Whether a value is currently pending (non-consuming probe). */
  bool hasPending() const {
    std::lock_guard lk(mtx_);
    return pending_.has_value();
  }

  /**@brief Consistent snapshot of all counters and streak maxima. */
  Stats stats() const {
    std::lock_guard lk(mtx_);
    return Stats{puts_,     gets_,         hits_,
                 misses_,   overruns_,     maxPutStreak_,
                 maxGetStreak_};
  }

  /**@brief Drop the pending value and zero all counters/streaks. */
  void reset() {
    std::lock_guard lk(mtx_);
    pending_.reset();
    puts_ = gets_ = hits_ = misses_ = overruns_ = 0;
    putRun_ = getRun_ = maxPutStreak_ = maxGetStreak_ = 0;
  }

 private:
  mutable std::mutex mtx_;
  std::optional<T> pending_;
  uint64_t puts_ = 0, gets_ = 0, hits_ = 0, misses_ = 0, overruns_ = 0;
  uint64_t putRun_ = 0, getRun_ = 0;  // streak cursors
  uint64_t maxPutStreak_ = 0, maxGetStreak_ = 0;
};

}  // namespace tether::common

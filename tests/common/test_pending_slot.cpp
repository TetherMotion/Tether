// PendingSlot: latest-wins hand-off, take-empty semantics, and the
// plesiosynchronicity counters (overruns, misses, put/get streak maxima).
#include <gtest/gtest.h>

#include <atomic>
#include <memory>
#include <thread>
#include <vector>

#include "tether/common/PendingSlot.hpp"

using tether::common::PendingSlot;

namespace {

TEST(PendingSlot, PostTakeRoundtrip) {
  PendingSlot<int> slot;
  slot.post(42);
  const auto v = slot.take();
  ASSERT_TRUE(v.has_value());
  EXPECT_EQ(*v, 42);
  EXPECT_FALSE(slot.take().has_value());  // slot is empty after the take
}

TEST(PendingSlot, LatestWinsOverwrite) {
  PendingSlot<int> slot;
  slot.post(1);
  slot.post(2);
  slot.post(3);
  const auto v = slot.take();
  ASSERT_TRUE(v.has_value());
  EXPECT_EQ(*v, 3);  // 1 and 2 never reached the consumer
  const auto s = slot.stats();
  EXPECT_EQ(s.puts, 3u);
  EXPECT_EQ(s.overruns, 2u);  // two double-put-single-get violations
  EXPECT_EQ(s.hits, 1u);
}

TEST(PendingSlot, TakeOnEmptyIsMissNotError) {
  PendingSlot<int> slot;
  EXPECT_FALSE(slot.take().has_value());
  EXPECT_FALSE(slot.take().has_value());
  const auto s = slot.stats();
  EXPECT_EQ(s.gets, 2u);
  EXPECT_EQ(s.misses, 2u);  // double-get-single-put violations
  EXPECT_EQ(s.hits, 0u);
}

TEST(PendingSlot, HasPendingIsNonConsuming) {
  PendingSlot<int> slot;
  EXPECT_FALSE(slot.hasPending());
  slot.post(7);
  EXPECT_TRUE(slot.hasPending());
  EXPECT_TRUE(slot.hasPending());  // probe does not consume
  EXPECT_TRUE(slot.take().has_value());
  EXPECT_FALSE(slot.hasPending());
  // Probes also do not perturb the put/get model.
  const auto s = slot.stats();
  EXPECT_EQ(s.gets, 1u);
  EXPECT_EQ(s.puts, 1u);
}

TEST(PendingSlot, StreakMaximaTrackPhaseDrift) {
  PendingSlot<int> slot;
  // Run of 3 puts, then run of 4 takes (1 hit + 3 misses).
  slot.post(1);
  slot.post(2);
  slot.post(3);
  for (int i = 0; i < 4; ++i) (void)slot.take();
  const auto s = slot.stats();
  EXPECT_EQ(s.maxPutStreak, 3u);
  EXPECT_EQ(s.maxGetStreak, 4u);
  EXPECT_EQ(s.overruns, 2u);
  EXPECT_EQ(s.hits, 1u);
  EXPECT_EQ(s.misses, 3u);
}

TEST(PendingSlot, StatsInvariantPutsEqualsHitsPlusOverrunsPlusPending) {
  PendingSlot<int> slot;
  // puts=5: 2 taken (hits), 2 dropped (overruns), 1 still pending.
  slot.post(1);  // pending=1
  slot.post(2);  // overrun, pending=1
  (void)slot.take();  // hit (gets 2)
  slot.post(3);
  (void)slot.take();  // hit (gets 3)
  slot.post(4);
  slot.post(5);  // overrun
  const auto s = slot.stats();
  const uint64_t pending = slot.hasPending() ? 1 : 0;
  EXPECT_EQ(s.puts, s.hits + s.overruns + pending);
  EXPECT_EQ(s.gets, s.hits + s.misses);
}

TEST(PendingSlot, ResetClearsValueAndCounters) {
  PendingSlot<int> slot;
  slot.post(9);
  slot.post(10);
  (void)slot.take();
  (void)slot.take();
  slot.reset();
  EXPECT_FALSE(slot.hasPending());
  const auto s = slot.stats();
  EXPECT_EQ(s.puts + s.gets + s.hits + s.misses + s.overruns, 0u);
  EXPECT_EQ(s.maxPutStreak + s.maxGetStreak, 0u);
}

TEST(PendingSlot, MoveOnlyPayload) {
  PendingSlot<std::unique_ptr<int>> slot;
  slot.post(std::make_unique<int>(11));
  auto v = slot.take();
  ASSERT_TRUE(v.has_value());
  ASSERT_TRUE(*v);
  EXPECT_EQ(**v, 11);
  EXPECT_FALSE(slot.take().has_value());
}

TEST(PendingSlot, SharedPtrPayloadVesomaShape) {
  // The deferred-swap pattern: a shared_ptr parked by one thread, claimed
  // at the consumer's tick boundary.
  PendingSlot<std::shared_ptr<int>> slot;
  auto p = std::make_shared<int>(5);
  slot.post(p);
  EXPECT_EQ(p.use_count(), 2);  // slot holds one ref
  const auto v = slot.take();
  ASSERT_TRUE(v.has_value());
  EXPECT_EQ(**v, 5);
  EXPECT_EQ(v->use_count(), 2);  // caller + original
  EXPECT_FALSE(slot.hasPending());
}

// A payload that detects copies: take() must move, not copy, out.
struct Counted {
  Counted() = default;
  Counted(const Counted&) { ++copies; }
  Counted& operator=(const Counted&) {
    ++copies;
    return *this;
  }
  Counted(Counted&&) noexcept = default;
  Counted& operator=(Counted&&) noexcept = default;
  static inline int copies = 0;
};

TEST(PendingSlot, TakeReturnsMovedValueNotCopy) {
  PendingSlot<Counted> slot;
  slot.post(Counted{});
  Counted::copies = 0;
  const auto v = slot.take();
  ASSERT_TRUE(v.has_value());
  EXPECT_EQ(Counted::copies, 0);
}

TEST(PendingSlot, ConcurrentProducerConsumer) {
  // Producer bursts posts; consumer takes at its own pace. The model must
  // stay exact under real interleaving: every put is a hit, an overrun, or
  // still pending — nothing is lost silently.
  PendingSlot<int> slot;
  constexpr int kPosts = 5000;
  std::atomic<bool> done{false};
  int lastTaken = -1;
  uint64_t takenCount = 0;

  std::thread producer([&] {
    for (int i = 0; i < kPosts; ++i) slot.post(i);
    done.store(true, std::memory_order_release);
  });
  std::thread consumer([&] {
    while (!done.load(std::memory_order_acquire) || slot.hasPending()) {
      if (const auto v = slot.take()) {
        lastTaken = *v;
        ++takenCount;
      }
    }
  });
  producer.join();
  consumer.join();

  const auto s = slot.stats();
  EXPECT_EQ(s.puts, static_cast<uint64_t>(kPosts));
  EXPECT_EQ(takenCount, s.hits);
  // Invariant holds under concurrency.
  const uint64_t pending = slot.hasPending() ? 1 : 0;
  EXPECT_EQ(s.puts, s.hits + s.overruns + pending);
  EXPECT_EQ(s.gets, s.hits + s.misses);
  // The last taken value is the final posted one (latest wins to the end).
  EXPECT_EQ(lastTaken, kPosts - 1);
}

}  // namespace

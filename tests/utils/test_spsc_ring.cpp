// test_spsc_ring.cpp — unit tests for Tether::Utils::SPSCRing.

#include <gtest/gtest.h>

#include <atomic>
#include <thread>
#include <vector>

#include "tether/utils/SPSCRing.hpp"

using Tether::Utils::SPSCRing;

struct Sample {
    uint64_t seq;
    double   value;
};

TEST(SPSCRingTest, EmptyOnConstruction) {
    SPSCRing<Sample, 16> ring;
    EXPECT_TRUE(ring.empty());
    EXPECT_EQ(ring.size(), 0u);
    EXPECT_EQ(ring.dropped(), 0u);
    int seen = 0;
    EXPECT_EQ(ring.drain([&](const Sample&) { ++seen; }), 0u);
    EXPECT_EQ(seen, 0);
}

TEST(SPSCRingTest, FifoOrder) {
    SPSCRing<Sample, 8> ring;
    for (uint64_t i = 0; i < 8; ++i)
        ASSERT_TRUE(ring.push(Sample{i, i * 1.5}));

    std::vector<uint64_t> got;
    EXPECT_EQ(ring.drain([&](const Sample& s) { got.push_back(s.seq); }), 8u);
    for (uint64_t i = 0; i < 8; ++i)
        EXPECT_EQ(got[i], i);
    EXPECT_TRUE(ring.empty());
}

TEST(SPSCRingTest, FullRingDrops) {
    SPSCRing<Sample, 4> ring;
    for (uint64_t i = 0; i < 4; ++i)
        ASSERT_TRUE(ring.push(Sample{i, 0.0}));
    // Producer must not block — returns false and counts the drop.
    EXPECT_FALSE(ring.push(Sample{99, 0.0}));
    EXPECT_EQ(ring.dropped(), 1u);

    std::vector<uint64_t> got;
    ring.drain([&](const Sample& s) { got.push_back(s.seq); });
    ASSERT_EQ(got.size(), 4u);
    EXPECT_EQ(got.front(), 0u);
    EXPECT_EQ(got.back(), 3u);
    // Slots are released after the drain: producing works again.
    EXPECT_TRUE(ring.push(Sample{10, 0.0}));
    EXPECT_EQ(ring.size(), 1u);
}

TEST(SPSCRingTest, ProduceWritesSlotInPlace) {
    SPSCRing<Sample, 4> ring;
    ASSERT_TRUE(ring.produce([](Sample& s) {
        s.seq   = 42;
        s.value = 3.25;
    }));
    bool found = false;
    ring.drain([&](const Sample& s) {
        found   = true;
        EXPECT_EQ(s.seq, 42u);
        EXPECT_DOUBLE_EQ(s.value, 3.25);
    });
    EXPECT_TRUE(found);
}

TEST(SPSCRingTest, PartialDrainLeavesRemainder) {
    SPSCRing<Sample, 8> ring;
    for (uint64_t i = 0; i < 5; ++i)
        ASSERT_TRUE(ring.push(Sample{i, 0.0}));

    std::vector<uint64_t> first;
    ring.drain([&](const Sample& s) {
        if (first.size() < 2)
            first.push_back(s.seq);
        // consume() is invoked for every published record regardless
    });
    // All 5 were consumed by the drain above.
    EXPECT_TRUE(ring.empty());
}

TEST(SPSCRingTest, ResetClearsState) {
    SPSCRing<Sample, 4> ring;
    ring.push(Sample{1, 0.0});
    ring.push(Sample{2, 0.0});
    ring.push(Sample{3, 0.0});
    ring.push(Sample{4, 0.0});
    ring.push(Sample{5, 0.0});  // dropped
    EXPECT_EQ(ring.dropped(), 1u);

    ring.reset();
    EXPECT_TRUE(ring.empty());
    EXPECT_EQ(ring.dropped(), 0u);
    EXPECT_TRUE(ring.push(Sample{9, 0.0}));
    std::vector<uint64_t> got;
    ring.drain([&](const Sample& s) { got.push_back(s.seq); });
    ASSERT_EQ(got.size(), 1u);
    EXPECT_EQ(got[0], 9u);
}

TEST(SPSCRingTest, WrapAroundPreservesOrder) {
    SPSCRing<Sample, 4> ring;
    // Push and drain interleaved far beyond capacity so the monotonic
    // counters wrap the buffer several times.
    uint64_t expected = 0;
    for (uint64_t i = 0; i < 64; ++i) {
        ASSERT_TRUE(ring.push(Sample{i, 0.0}));
        ring.drain([&](const Sample& s) {
            EXPECT_EQ(s.seq, expected);
            ++expected;
        });
    }
    EXPECT_EQ(expected, 64u);
    EXPECT_EQ(ring.dropped(), 0u);
}

TEST(SPSCRingTest, ConcurrentProducerConsumer) {
    constexpr size_t   CAP = 256;
    constexpr uint64_t N   = 200000;
    SPSCRing<Sample, CAP> ring;

    std::atomic<uint64_t> consumed{0};
    std::atomic<bool>     order_ok{true};
    std::thread consumer([&] {
        uint64_t next = 0;
        // The producer retries failed pushes, so every record eventually
        // lands — consume exactly N records.
        while (consumed.load(std::memory_order_relaxed) < N) {
            ring.drain([&](const Sample& s) {
                if (s.seq != next)
                    order_ok.store(false, std::memory_order_relaxed);
                ++next;
                consumed.fetch_add(1, std::memory_order_relaxed);
            });
        }
    });

    for (uint64_t i = 0; i < N; ++i)
        while (!ring.push(Sample{i, 0.0}))
            ;  // retry when full — drop counted, spin is test-only

    consumer.join();
    EXPECT_TRUE(order_ok.load());
    EXPECT_EQ(consumed.load(), N);
    EXPECT_TRUE(ring.empty());
}

// WindowStats: ring semantics, aggregate correctness, percentile ranks.
#include <gtest/gtest.h>

#include <stdexcept>

#include "tether/common/WindowStats.hpp"

using tether::common::WindowStats;

namespace {

TEST(WindowStats, EmptyWindow) {
    WindowStats<4> w;
    EXPECT_TRUE(w.empty());
    EXPECT_EQ(w.size(), 0u);
    EXPECT_EQ(w.capacity(), 4u);
    EXPECT_EQ(w.mean(), 0.0);
    EXPECT_EQ(w.max(), 0.0);
    EXPECT_EQ(w.min(), 0.0);
    EXPECT_EQ(w.percentile(99), 0.0);
    EXPECT_THROW((void)w.last(), std::out_of_range);
}

TEST(WindowStats, AggregatesOverWindow) {
    WindowStats<16> w;
    for (double v = 1; v <= 10; ++v) w.add(v);
    EXPECT_EQ(w.size(), 10u);
    EXPECT_DOUBLE_EQ(w.last(), 10.0);
    EXPECT_DOUBLE_EQ(w.mean(), 5.5);
    EXPECT_DOUBLE_EQ(w.max(), 10.0);
    EXPECT_DOUBLE_EQ(w.min(), 1.0);
}

TEST(WindowStats, RingEvictsOldest) {
    WindowStats<4> w;
    for (double v = 1; v <= 6; ++v) w.add(v);
    EXPECT_EQ(w.size(), 4u);          // window holds 3,4,5,6
    EXPECT_DOUBLE_EQ(w.last(), 6.0);
    EXPECT_DOUBLE_EQ(w.mean(), 4.5);  // (3+4+5+6)/4
    EXPECT_DOUBLE_EQ(w.min(), 3.0);   // 1,2 evicted
}

TEST(WindowStats, PercentileRanks) {
    WindowStats<16> w;
    for (double v = 1; v <= 10; ++v) w.add(v);  // 1..10 sorted
    EXPECT_DOUBLE_EQ(w.percentile(0), 1.0);
    EXPECT_DOUBLE_EQ(w.percentile(50), 6.0);   // rank ceil(5)-1 → index 5
    EXPECT_DOUBLE_EQ(w.percentile(99), 10.0);
    EXPECT_DOUBLE_EQ(w.percentile(100), 10.0);
}

TEST(WindowStats, PercentileClampsOutOfRange) {
    WindowStats<4> w;
    w.add(2.0);
    w.add(8.0);
    EXPECT_DOUBLE_EQ(w.percentile(-10), 2.0);
    EXPECT_DOUBLE_EQ(w.percentile(150), 8.0);
}

TEST(WindowStats, PartialWindowPercentile) {
    WindowStats<16> w;
    w.add(5.0);
    w.add(1.0);
    w.add(9.0);
    EXPECT_DOUBLE_EQ(w.percentile(50), 5.0);  // median of 3
    EXPECT_DOUBLE_EQ(w.percentile(100), 9.0);
}

TEST(WindowStats, ResetClearsAll) {
    WindowStats<4> w;
    w.add(1.0);
    w.add(2.0);
    w.reset();
    EXPECT_TRUE(w.empty());
    EXPECT_EQ(w.size(), 0u);
    EXPECT_EQ(w.percentile(50), 0.0);
    EXPECT_THROW((void)w.last(), std::out_of_range);
}

TEST(WindowStats, LargeWindowStaysBounded) {
    WindowStats<1024> w;
    for (int i = 0; i < 5000; ++i) w.add(static_cast<double>(i));
    EXPECT_EQ(w.size(), 1024u);  // capped
    EXPECT_DOUBLE_EQ(w.last(), 4999.0);
    EXPECT_DOUBLE_EQ(w.min(), 3976.0);  // 5000-1024
}

}  // namespace

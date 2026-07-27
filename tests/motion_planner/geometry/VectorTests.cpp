/**
 * @file VectorTests.cpp
 * @brief Unit tests for tether::motion::RVec (fixed-capacity runtime-dim vector)
 */

#include <gtest/gtest.h>
#include <tether/motion_planner/geometry/Vector.hpp>

#include <cmath>
#include <stdexcept>

using tether::motion::RVec;

TEST(RVec, ZeroFactory) {
    RVec v = RVec::zero(3);
    EXPECT_EQ(v.dim(), 3u);
    for (std::size_t i = 0; i < 3; ++i) EXPECT_DOUBLE_EQ(v[i], 0.0);
    EXPECT_THROW(RVec::zero(0), std::invalid_argument);
    EXPECT_THROW(RVec::zero(6), std::invalid_argument);
}

TEST(RVec, InitializerListSetsDimension) {
    RVec a{1.0, 2.0};
    RVec b{1.0, 2.0, 3.0, 4.0, 5.0};
    EXPECT_EQ(a.dim(), 2u);
    EXPECT_EQ(b.dim(), 5u);
    EXPECT_THROW(RVec({}), std::invalid_argument);
    EXPECT_THROW((RVec{1, 2, 3, 4, 5, 6}), std::invalid_argument);
}

TEST(RVec, Arithmetic) {
    RVec a{1.0, 2.0, 3.0};
    RVec b{4.0, 5.0, 6.0};

    RVec sum = a + b;
    EXPECT_DOUBLE_EQ(sum[0], 5.0);
    EXPECT_DOUBLE_EQ(sum[1], 7.0);
    EXPECT_DOUBLE_EQ(sum[2], 9.0);

    RVec diff = b - a;
    EXPECT_DOUBLE_EQ(diff[0], 3.0);
    EXPECT_DOUBLE_EQ(diff[2], 3.0);

    RVec scaled = a * 2.0;
    EXPECT_DOUBLE_EQ(scaled[1], 4.0);
    RVec scaled2 = 2.0 * a;
    EXPECT_EQ(scaled, scaled2);

    RVec neg = -a;
    EXPECT_DOUBLE_EQ(neg[0], -1.0);

    a += b;
    EXPECT_DOUBLE_EQ(a[0], 5.0);
    a -= b;
    EXPECT_DOUBLE_EQ(a[0], 1.0);
}

TEST(RVec, DotNormNormalized) {
    RVec a{1.0, 2.0, 2.0};
    RVec b{3.0, 4.0, 0.0};
    EXPECT_DOUBLE_EQ(a.dot(b), 11.0);
    EXPECT_DOUBLE_EQ(a.normSq(), 9.0);
    EXPECT_DOUBLE_EQ(a.norm(), 3.0);

    RVec n = b.normalized();
    EXPECT_DOUBLE_EQ(n.norm(), 1.0);
    EXPECT_DOUBLE_EQ(n[0], 0.6);
    EXPECT_DOUBLE_EQ(n[1], 0.8);

    EXPECT_THROW(RVec::zero(3).normalized(), std::domain_error);
}

TEST(RVec, ExactEquality) {
    RVec a{1.0, 2.0, 3.0};
    RVec b{1.0, 2.0, 3.0};
    // 1e-16 is below the ULP of 3.0 (~4.4e-16) and rounds back to 3.0; use
    // 1e-15 so the third component actually differs bitwise.
    RVec c{1.0, 2.0, 3.0 + 1e-15};
    RVec d{1.0, 2.0};
    EXPECT_EQ(a, b);
    EXPECT_NE(a, c); // exact comparison by design
    EXPECT_NE(a, d); // different dimensions
    EXPECT_TRUE(a.nearEqual(c, 1e-12));
}

TEST(RVec, DimensionMismatchThrows) {
    RVec a{1.0, 2.0};
    RVec b{1.0, 2.0, 3.0};
    EXPECT_THROW(a + b, std::invalid_argument);
    EXPECT_THROW(a - b, std::invalid_argument);
    EXPECT_THROW(a.dot(b), std::invalid_argument);
    EXPECT_THROW(a.distanceTo(b), std::invalid_argument);
    EXPECT_THROW(a.nearEqual(b, 1.0), std::invalid_argument);
}

TEST(RVec, BoundsChecking) {
    RVec a{1.0, 2.0};
    EXPECT_THROW(a[2], std::out_of_range);
    EXPECT_NO_THROW(a[1]);
}

TEST(RVec, FiveDimensions) {
    RVec a{1.0, 2.0, 3.0, 4.0, 5.0};
    EXPECT_DOUBLE_EQ(a.norm(), std::sqrt(55.0));
    RVec b = a.normalized();
    EXPECT_NEAR(b.norm(), 1.0, 1e-15);
}

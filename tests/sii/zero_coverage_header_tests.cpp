#include <gtest/gtest.h>

// Header-only APIs / factories that should compile and be usable without linking
#include "tether/ethercat/ConditionalPacketRouter.hpp"
#include "tether/ethercat/EtherCATReset.hpp"

using namespace EtherCAT;

TEST(PacketFilter_Header, Factories) {
    auto f1 = PacketFilter::byIndex(7);
    EXPECT_TRUE(f1.match_idx);
    EXPECT_EQ(f1.idx, 7u);

    auto a = PacketFilter::aprd(1, 0x1000, 3);
    EXPECT_TRUE(a.match_command);
    EXPECT_TRUE(a.match_ado);
    EXPECT_TRUE(a.match_idx);
    EXPECT_EQ(a.ado, 0x1000u);

    auto b = PacketFilter::brd(0x2000, 5);
    EXPECT_TRUE(b.match_command);
    EXPECT_TRUE(b.match_ado);
    EXPECT_EQ(b.ado, 0x2000u);

    auto l = PacketFilter::lrw(0xFF00u, 2, 9);
    EXPECT_TRUE(l.match_logical);
    EXPECT_EQ(l.logical_length, 2u);
}

TEST(WaitResult_Header, Helpers) {
    auto t = WaitResult::Timeout();
    EXPECT_TRUE(t.timeout);

    auto s = WaitResult::Success(3, 10, Raw::EtherCATCommand::APRD, 0x1000, 0x2000, 4);
    EXPECT_TRUE(s.success);
    EXPECT_EQ(s.wkc, 3u);
    EXPECT_EQ(s.data_length, 10u);
    EXPECT_EQ(s.idx, 4u);
}

TEST(Types_Header, CompileTimeConstants) {
    // Basic compile-time checks for constants
    EXPECT_EQ(ALControl::StateMask & ALControl::AckError, 0x0000u);
    EXPECT_GE(ALStatusCode::VendorSpecificStart, 0x8000u);

    // Ensure PacketFilter size is reasonable (sanity compile check)
    EXPECT_LT(sizeof(PacketFilter), 200u);
}

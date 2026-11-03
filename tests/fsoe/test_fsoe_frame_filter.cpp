/**
 * @file test_fsoe_frame_filter.cpp
 * @brief Coverage for FSoEFrameFilter — the NoFSoEFilter passthrough and
 *        the LW1→LW2 slave→master frame expander.
 */

#include <gtest/gtest.h>

#include <cstring>
#include <vector>

#include "tether/drives/Synapticon/SynapticonPDO.hpp"
#include "tether/fsoe/FSoECRC.hpp"
#include "tether/fsoe/FSoEFrameFilter.hpp"

using namespace FSoE;
namespace SynPDO = EtherCAT::Drives::SynapticonPDO;

namespace {

constexpr uint16_t kConnId = 0x1234;

/// Build a valid FSoE frame carrying `n` data bytes of a ramp pattern.
size_t makeFrame(uint8_t* out, size_t out_cap, uint8_t cmd, size_t n,
                 uint16_t conn = kConnId) {
    std::vector<uint8_t> data(n);
    for (size_t i = 0; i < n; ++i) data[i] = static_cast<uint8_t>(0x40 + i);
    return CRC::buildFSoEFrame(out, cmd, data.data(), n, conn, 0, 1);
}

}  // namespace

// ============================================================================
// NoFSoEFilter
// ============================================================================

TEST(NoFSoEFilterTest, PassthroughBothDirections) {
    NoFSoEFilter f;
    const uint8_t in[5] = {1, 2, 3, 4, 5};
    uint8_t out[5] = {};
    EXPECT_EQ(f.filterSlaveToMaster(in, sizeof(in), out, sizeof(out)), 5u);
    EXPECT_EQ(std::memcmp(in, out, sizeof(in)), 0);
    std::memset(out, 0, sizeof(out));
    EXPECT_EQ(f.filterMasterToSlave(in, sizeof(in), out, sizeof(out)), 5u);
    EXPECT_EQ(std::memcmp(in, out, sizeof(in)), 0);
}

TEST(NoFSoEFilterTest, ClampsToOutputCapacity) {
    NoFSoEFilter f;
    const uint8_t in[10] = {9, 8, 7, 6, 5, 4, 3, 2, 1, 0};
    uint8_t out[4] = {};
    EXPECT_EQ(f.filterSlaveToMaster(in, sizeof(in), out, sizeof(out)), 4u);
    EXPECT_EQ(out[0], 9);
    EXPECT_EQ(out[3], 6);
}

TEST(NoFSoEFilterTest, Metadata) {
    NoFSoEFilter f;
    EXPECT_STREQ(f.name(), "NoFSoEFilter");
    EXPECT_TRUE(f.isTransparent());
}

// ============================================================================
// LW1ToLW2FSoEFilter
// ============================================================================

TEST(LW1ToLW2FSoEFilterTest, Metadata) {
    LW1ToLW2FSoEFilter f;
    EXPECT_STREQ(f.name(), "LW1ToLW2FSoEFilter");
    EXPECT_FALSE(f.isTransparent());
}

TEST(LW1ToLW2FSoEFilterTest, ExpandsLw1FrameToLw2) {
    LW1ToLW2FSoEFilter f;
    uint8_t frame[64];
    const size_t n = makeFrame(frame, sizeof(frame), Command::ProcessData,
                               SynPDO::fsoeSafeDataSize(
                                   SynPDO::FSoEFrameVariant::LW1));
    ASSERT_GT(n, 0u);

    uint8_t out[64];
    const size_t m = f.filterSlaveToMaster(frame, n, out, sizeof(out));
    ASSERT_GT(m, 0u);

    uint8_t cmd = 0;
    uint8_t data[64];
    size_t data_len = 0;
    uint16_t conn_id = 0;
    ASSERT_TRUE(CRC::extractFSoEFrame(out, m, cmd, data, data_len, conn_id));
    EXPECT_EQ(cmd, Command::ProcessData);
    EXPECT_EQ(conn_id, kConnId);
    EXPECT_EQ(data_len,
              SynPDO::fsoeSafeDataSize(SynPDO::FSoEFrameVariant::LW2));

    // Original LW1 payload preserved, appended safe-torque word zeroed.
    for (size_t i = 0; i < 14; ++i)
        EXPECT_EQ(data[i], static_cast<uint8_t>(0x40 + i)) << i;
    EXPECT_EQ(data[14], 0);
    EXPECT_EQ(data[15], 0);
}

TEST(LW1ToLW2FSoEFilterTest, RechainsAlreadyLw2Frame) {
    LW1ToLW2FSoEFilter f;
    uint8_t frame[64];
    const size_t n = makeFrame(frame, sizeof(frame), Command::ProcessData,
                               SynPDO::fsoeSafeDataSize(
                                   SynPDO::FSoEFrameVariant::LW2));
    ASSERT_GT(n, 0u);

    uint8_t out[64];
    const size_t m = f.filterSlaveToMaster(frame, n, out, sizeof(out));
    ASSERT_GT(m, 0u);

    uint8_t cmd = 0;
    uint8_t data[64];
    size_t data_len = 0;
    uint16_t conn_id = 0;
    ASSERT_TRUE(CRC::extractFSoEFrame(out, m, cmd, data, data_len, conn_id));
    EXPECT_EQ(data_len,
              SynPDO::fsoeSafeDataSize(SynPDO::FSoEFrameVariant::LW2));
    EXPECT_EQ(conn_id, kConnId);
}

TEST(LW1ToLW2FSoEFilterTest, RejectsGarbageInput) {
    LW1ToLW2FSoEFilter f;
    const uint8_t garbage[8] = {0, 1, 2, 3, 4, 5, 6, 7};
    uint8_t out[64];
    EXPECT_EQ(f.filterSlaveToMaster(garbage, sizeof(garbage), out,
                                    sizeof(out)),
              0u);
    EXPECT_EQ(f.filterSlaveToMaster(garbage, 0, out, sizeof(out)), 0u);
}

TEST(LW1ToLW2FSoEFilterTest, RejectsUnknownCommand) {
    LW1ToLW2FSoEFilter f;
    uint8_t frame[64];
    const size_t n = makeFrame(frame, sizeof(frame), 0x11, 14);  // not FSoE
    ASSERT_GT(n, 0u);
    uint8_t out[64];
    EXPECT_EQ(f.filterSlaveToMaster(frame, n, out, sizeof(out)), 0u);
}

TEST(LW1ToLW2FSoEFilterTest, MasterToSlaveIsPassthrough) {
    LW1ToLW2FSoEFilter f;
    uint8_t frame[64];
    const size_t n = makeFrame(frame, sizeof(frame), Command::ProcessData, 14);
    ASSERT_GT(n, 0u);
    uint8_t out[64];
    ASSERT_EQ(f.filterMasterToSlave(frame, n, out, sizeof(out)), n);
    EXPECT_EQ(std::memcmp(frame, out, n), 0);
}

TEST(LW1ToLW2FSoEFilterTest, RespectsOutputCapacity) {
    LW1ToLW2FSoEFilter f;
    uint8_t frame[64];
    const size_t n = makeFrame(frame, sizeof(frame), Command::ProcessData, 14);
    ASSERT_GT(n, 0u);
    uint8_t small[8];
    EXPECT_EQ(f.filterSlaveToMaster(frame, n, small, sizeof(small)), 0u);
}

TEST(LW1ToLW2FSoEFilterTest, NoteFramesThenFilterStillWorks) {
    LW1ToLW2FSoEFilter f;
    uint8_t master_frame[64];
    uint8_t slave_frame[64];
    const size_t mn = makeFrame(master_frame, sizeof(master_frame),
                                Command::ProcessData, 14);
    const size_t sn = makeFrame(slave_frame, sizeof(slave_frame),
                                Command::ProcessData, 14, 0x4321);
    ASSERT_GT(mn, 0u);
    ASSERT_GT(sn, 0u);

    f.noteMasterFrame(master_frame, mn);
    f.noteSlaveFrame(slave_frame, sn);

    uint8_t out[64];
    ASSERT_GT(f.filterSlaveToMaster(slave_frame, sn, out, sizeof(out)), 0u);

    f.reset();
    ASSERT_GT(f.filterSlaveToMaster(slave_frame, sn, out, sizeof(out)), 0u);
}

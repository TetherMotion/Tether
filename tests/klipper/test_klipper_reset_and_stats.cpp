/**
 * @file test_klipper_reset_and_stats.cpp
 * @brief Tests for KlipperDevice::reset(), KlippyHost::reset(), and the
 *        blockParseStats/skippedBlockCount accessors that expose the
 *        internal BlockReader state.
 *
 * These mirror the pcapng reader's reset() and skippedBlockCount() tests,
 * adapted to the Klipper device/host layers.
 */

#include <gtest/gtest.h>
#include "tether/klipper/device/KlipperDevice.hpp"
#include "tether/klipper/device/KlipperDeviceConfig.hpp"
#include "tether/klipper/klippy/KlippyHost.hpp"
#include "tether/klipper/protocol/DataDictionary.hpp"
#include "tether/klipper/protocol/MessageBlock.hpp"
#include "tether/klipper/protocol/BlockReader.hpp"
#include "tether/klipper/transport/LoopbackTransport.hpp"
#include "tether/klipper/objects/Stepper.hpp"

#include <memory>
#include <vector>

using namespace tether::klipper;
using namespace tether::klipper::device;
using namespace tether::klipper::klippy;
using namespace tether::klipper::transport;
using namespace tether::klipper::protocol;
using namespace tether::klipper::objects;

// ============================================================================
// Helpers
// ============================================================================

/// Build a minimal dictionary with the core commands needed for testing.
static DataDictionary makeTestDict() {
    DataDictionary dict;
    dict.addCommand("allocate_oids oid=%c");
    dict.addCommand("get_config");
    dict.addCommand("get_status");
    dict.addCommand("shutdown");
    dict.addCommand("finalize_config crc=%u");
    dict.addCommand("get_clock");
    dict.addCommand("queue_step oid=%c interval=%u count=%hu add=%hi");
    dict.addCommand("set_next_step_dir oid=%c dir=%c");
    dict.addCommand("reset_step_clock oid=%c clock=%u");
    dict.addResponse("config_result oid_count=%c config_crc=%u");
    dict.addResponse("status clock=%u status=%c");
    dict.addResponse("clock clock=%u");
    return dict;
}

// ============================================================================
// KlipperDevice::reset() tests
// ============================================================================

TEST(KlipperDeviceReset, ClearsShutdownState) {
    auto pair = std::make_shared<LoopbackTransportPair>();
    auto dict = makeTestDict();
    KlipperDeviceConfig config;
    auto device = std::make_shared<KlipperDevice>(
        std::shared_ptr<IByteStreamTransport>(
            std::shared_ptr<LoopbackTransport>{}, &pair->deviceEnd()),
        dict, config);
    device->start();

    // Send a shutdown command to set shutdown_=true.
    // Build a shutdown command block manually.
    auto shutdownMsgid = dict.lookupCommand("shutdown");
    ASSERT_TRUE(shutdownMsgid);
    std::vector<uint8_t> content;
    ASSERT_TRUE(encodeMessage(dict, *shutdownMsgid, {}, content));
    auto wire = buildBlockVec(0, content);
    pair->hostEnd().write(wire);
    device->pump();
    EXPECT_TRUE(device->isShutdown());

    // Reset should clear shutdown state.
    device->reset();
    EXPECT_FALSE(device->isShutdown());
}

TEST(KlipperDeviceReset, ClearsConfigFinalized) {
    auto pair = std::make_shared<LoopbackTransportPair>();
    auto dict = makeTestDict();
    KlipperDeviceConfig config;
    auto device = std::make_shared<KlipperDevice>(
        std::shared_ptr<IByteStreamTransport>(
            std::shared_ptr<LoopbackTransport>{}, &pair->deviceEnd()),
        dict, config);
    device->start();

    // Send finalize_config to set configFinalized_=true.
    auto finMsgid = dict.lookupCommand("finalize_config crc=%u");
    ASSERT_TRUE(finMsgid);
    std::vector<uint8_t> content;
    ASSERT_TRUE(encodeMessage(dict, *finMsgid,
                              std::vector<ParamValue>{ParamValue{static_cast<int32_t>(0x1234)}}, content));
    auto wire = buildBlockVec(0, content);
    pair->hostEnd().write(wire);
    device->pump();
    EXPECT_TRUE(device->isConfigFinalized());

    // Reset should clear finalized state.
    device->reset();
    EXPECT_FALSE(device->isConfigFinalized());
}

TEST(KlipperDeviceReset, ClearsOidAllocator) {
    auto pair = std::make_shared<LoopbackTransportPair>();
    auto dict = makeTestDict();
    KlipperDeviceConfig config;
    auto device = std::make_shared<KlipperDevice>(
        std::shared_ptr<IByteStreamTransport>(
            std::shared_ptr<LoopbackTransport>{}, &pair->deviceEnd()),
        dict, config);
    device->start();

    // Allocate some OIDs.
    auto allocMsgid = dict.lookupCommand("allocate_oids oid=%c");
    ASSERT_TRUE(allocMsgid);
    std::vector<uint8_t> content;
    ASSERT_TRUE(encodeMessage(dict, *allocMsgid,
                              std::vector<ParamValue>{ParamValue{static_cast<int32_t>(5)}}, content));
    auto wire = buildBlockVec(0, content);
    pair->hostEnd().write(wire);
    device->pump();
    EXPECT_EQ(device->allocatedOidCount(), 5u);

    // Reset should clear the OID allocator.
    device->reset();
    EXPECT_EQ(device->allocatedOidCount(), 0u);
}

TEST(KlipperDeviceReset, ClearsLastReceivedSeq) {
    auto pair = std::make_shared<LoopbackTransportPair>();
    auto dict = makeTestDict();
    KlipperDeviceConfig config;
    auto device = std::make_shared<KlipperDevice>(
        std::shared_ptr<IByteStreamTransport>(
            std::shared_ptr<LoopbackTransport>{}, &pair->deviceEnd()),
        dict, config);
    device->start();

    // Send a block with sequence 1.
    auto wire = buildBlockVec(1, std::vector<uint8_t>{0x01});
    pair->hostEnd().write(wire);
    device->pump();
    EXPECT_EQ(device->lastReceivedSeq(), 1u);

    // Reset should clear the last received sequence.
    device->reset();
    EXPECT_EQ(device->lastReceivedSeq(), 0u);
}

TEST(KlipperDeviceReset, BlockParseStatsAfterPump) {
    auto pair = std::make_shared<LoopbackTransportPair>();
    auto dict = makeTestDict();
    KlipperDeviceConfig config;
    auto device = std::make_shared<KlipperDevice>(
        std::shared_ptr<IByteStreamTransport>(
            std::shared_ptr<LoopbackTransport>{}, &pair->deviceEnd()),
        dict, config);
    device->start();

    // Send a valid block.
    auto wire = buildBlockVec(0, std::vector<uint8_t>{0x01});
    pair->hostEnd().write(wire);
    device->pump();

    // The device should have parsed at least one block.
    EXPECT_GE(device->blockParseStats().blocksParsed, 1u);
}

TEST(KlipperDeviceReset, RecoveryModeSkipsBadBlocks) {
    auto pair = std::make_shared<LoopbackTransportPair>();
    auto dict = makeTestDict();
    KlipperDeviceConfig config;
    auto device = std::make_shared<KlipperDevice>(
        std::shared_ptr<IByteStreamTransport>(
            std::shared_ptr<LoopbackTransport>{}, &pair->deviceEnd()),
        dict, config);
    device->start();

    // Enable recovery mode.
    device->setBlockRecoveryMode(true);

    // Send a bad block followed by a good block.
    auto good = buildBlockVec(0, std::vector<uint8_t>{0x01});
    auto bad = buildBlockVec(0, std::vector<uint8_t>{0x02});
    bad[bad.size() - 3] ^= 0xFF; // corrupt CRC

    pair->hostEnd().write(bad);
    pair->hostEnd().write(good);
    device->pump();

    // The bad block should have been skipped.
    EXPECT_GE(device->skippedBlockCount(), 1u);
    EXPECT_GE(device->blockParseStats().badCrcCount, 1u);
    // The good block should have been parsed.
    EXPECT_GE(device->blockParseStats().blocksParsed, 1u);
}

// ============================================================================
// KlippyHost::reset() tests
// ============================================================================

TEST(KlippyHostReset, ClearsReadyState) {
    auto pair = std::make_shared<LoopbackTransportPair>();
    // KlippyHost needs a shared_ptr to the transport.
    auto hostTransport = std::shared_ptr<IByteStreamTransport>(
        std::shared_ptr<LoopbackTransport>{}, &pair->hostEnd());
    KlippyHost host(hostTransport);

    // Before connect, not ready.
    EXPECT_FALSE(host.isReady());

    host.connect();
    // After connect but before dict download, not ready.
    EXPECT_FALSE(host.isReady());

    // Reset should clear connected state.
    host.reset();
    EXPECT_FALSE(host.isReady());
}

TEST(KlippyHostReset, ClearsOidAllocator) {
    auto pair = std::make_shared<LoopbackTransportPair>();
    auto hostTransport = std::shared_ptr<IByteStreamTransport>(
        std::shared_ptr<LoopbackTransport>{}, &pair->hostEnd());
    KlippyHost host(hostTransport);
    host.connect();

    // Allocate an OID.
    host.allocateOid("stepper");
    // Reset should clear the OID allocator (no direct accessor, but
    // after reset + reconnect, allocation should start from 0 again).
    host.reset();
    host.connect();
    uint8_t oid = host.allocateOid("stepper");
    EXPECT_EQ(oid, 0u);
}

TEST(KlippyHostReset, BlockParseStatsAccessible) {
    auto pair = std::make_shared<LoopbackTransportPair>();
    auto hostTransport = std::shared_ptr<IByteStreamTransport>(
        std::shared_ptr<LoopbackTransport>{}, &pair->hostEnd());
    KlippyHost host(hostTransport);
    host.connect();

    // Initially no blocks parsed.
    EXPECT_EQ(host.blockParseStats().blocksParsed, 0u);

    // Send an ack block from the "device" side.
    auto ack = buildAckBlock(0);
    pair->deviceEnd().write(ack);
    host.pump();

    // Should have parsed at least one block (the ack).
    EXPECT_GE(host.blockParseStats().blocksParsed, 1u);
}

TEST(KlippyHostReset, RecoveryModeSkipsBadBlocks) {
    auto pair = std::make_shared<LoopbackTransportPair>();
    auto hostTransport = std::shared_ptr<IByteStreamTransport>(
        std::shared_ptr<LoopbackTransport>{}, &pair->hostEnd());
    KlippyHost host(hostTransport);
    host.connect();
    host.setBlockRecoveryMode(true);

    // Send a bad block followed by a good ack.
    auto goodAck = buildAckBlock(0);
    auto bad = buildBlockVec(0, std::vector<uint8_t>{0x02});
    bad[bad.size() - 3] ^= 0xFF; // corrupt CRC

    pair->deviceEnd().write(bad);
    pair->deviceEnd().write(goodAck);
    host.pump();

    // The bad block should have been skipped.
    EXPECT_GE(host.skippedBlockCount(), 1u);
    EXPECT_GE(host.blockParseStats().badCrcCount, 1u);
}

TEST(KlippyHostReset, ResetClearsStats) {
    auto pair = std::make_shared<LoopbackTransportPair>();
    auto hostTransport = std::shared_ptr<IByteStreamTransport>(
        std::shared_ptr<LoopbackTransport>{}, &pair->hostEnd());
    KlippyHost host(hostTransport);
    host.connect();

    // Send and parse an ack block.
    auto ack = buildAckBlock(0);
    pair->deviceEnd().write(ack);
    host.pump();
    EXPECT_GE(host.blockParseStats().blocksParsed, 1u);

    // Reset should clear stats.
    host.reset();
    EXPECT_EQ(host.blockParseStats().blocksParsed, 0u);
}

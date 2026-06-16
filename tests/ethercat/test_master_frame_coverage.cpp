/**
 * @file test_master_frame_coverage.cpp
 * @brief Coverage tests for Master frame construction, parsing, and
 *        transport methods: sendSingleDatagram, handleRxFrame (parseEtherCATFrame),
 *        sendRawFrame, ecAprd with queued responses, requestSlaveState,
 *        getSlaveState, forceMailboxDefaults, autoConfigureMailbox,
 *        configurePDOSyncManagersFromSii, isRunning, getSrcMac, networkInterface,
 *        getDiscoveredSlaveCount, siiReadString, and various edge-case paths.
 */
#include <gtest/gtest.h>
#include "tether/ethercat/EtherCATMaster.hpp"
#include "tether/ethercat/EtherCATDC.hpp"
#include <cstring>
#include <vector>
#include <functional>

using namespace EtherCAT;

// ============================================================================
// Helper: build a minimal valid EtherCAT response frame for handleRxFrame
// ============================================================================
static std::vector<uint8_t> buildEtherCATFrame(
    Command cmd, uint8_t idx,
    uint16_t adp, uint16_t ado,
    const uint8_t* payload, uint16_t payload_len,
    uint16_t wkc)
{
    // EthernetHeader (14) + FrameHeader (2) + DatagramHeader (10) + payload + WKC (2)
    const size_t dgram_total = 10 + payload_len + 2; // DatagramHeader + data + WKC
    const size_t frame_size = 14 + 2 + dgram_total;
    std::vector<uint8_t> frame(std::max(frame_size, (size_t)60), 0);

    // Ethernet header
    // dst MAC: EtherCAT broadcast
    frame[0] = 0x01; frame[1] = 0x01; frame[2] = 0x05;
    frame[3] = 0x00; frame[4] = 0x00; frame[5] = 0x00;
    // src MAC: arbitrary
    frame[6] = 0xAA; frame[7] = 0xBB; frame[8] = 0xCC;
    frame[9] = 0xDD; frame[10] = 0xEE; frame[11] = 0xFF;
    // EtherType = 0x88A4 big-endian
    frame[12] = 0x88;
    frame[13] = 0xA4;

    // FrameHeader (2 bytes, little-endian): length[10:0] | reserved[11] | type[15:12]
    // type = 1, length = dgram_total
    uint16_t ec_raw = static_cast<uint16_t>((dgram_total & 0x07FFu) | (1u << 12));
    frame[14] = ec_raw & 0xFF;
    frame[15] = (ec_raw >> 8) & 0xFF;

    // DatagramHeader (10 bytes)
    frame[16] = static_cast<uint8_t>(cmd);     // cmd
    frame[17] = idx;                            // idx
    frame[18] = adp & 0xFF;                     // adp_le low
    frame[19] = (adp >> 8) & 0xFF;              // adp_le high
    frame[20] = ado & 0xFF;                     // ado_le low
    frame[21] = (ado >> 8) & 0xFF;              // ado_le high
    // lenFlags: datalen[10:0] | reserved[13:11] | C[14] | M[15]
    uint16_t len_flags = payload_len & 0x07FFu;
    frame[22] = len_flags & 0xFF;
    frame[23] = (len_flags >> 8) & 0xFF;
    // irq = 0
    frame[24] = 0;
    frame[25] = 0;

    // Payload
    if (payload && payload_len > 0) {
        std::memcpy(&frame[26], payload, payload_len);
    }

    // WKC (2 bytes, little-endian)
    size_t wkc_offset = 26 + payload_len;
    frame[wkc_offset] = wkc & 0xFF;
    frame[wkc_offset + 1] = (wkc >> 8) & 0xFF;

    return frame;
}

// ============================================================================
// sendRawFrame — no interface
// ============================================================================

TEST(MasterFrameCoverage, SendRawFrameNoInterface) {
    Master master;
    uint8_t buf[64] = {};
    EXPECT_FALSE(master.sendRawFrame(buf, sizeof(buf)));
}

// NOTE: sendRawFrame with a real interface requires master.start() which spawns
// a blocking master thread (discoverSlaves with retries). Instead, we test the
// send path indirectly via sendSingleDatagram (which also calls iface_.send).
// The test callback paths for ecApwr/ecAprd bypass sendSingleDatagram entirely.

// ============================================================================
// sendSingleDatagram — various paths (no interface = expected failure)
// ============================================================================

TEST(MasterFrameCoverage, SendSingleDatagramNoInterface) {
    Master master;
    // No interface registered → should fail (logs "No NetworkInterface available")
    uint8_t data[2] = {0x01, 0x02};
    EXPECT_FALSE(master.sendSingleDatagram(
        Command::APWR, 0x01, 0x0000, 0x0130, data, 2, false));
}

TEST(MasterFrameCoverage, SendSingleDatagramOversized) {
    Master master;
    // datalen that would exceed kMaxEthFrameNoFcs (1514)
    // required_len = sizeof(header=26) + datalen + 2(wkc) > 1514
    // datalen > 1514 - 28 = 1486
    uint8_t data[1490] = {};
    EXPECT_FALSE(master.sendSingleDatagram(
        Command::APWR, 0x01, 0x0000, 0x0130, data, 1490, false));
}

TEST(MasterFrameCoverage, SendSingleDatagramNullData) {
    Master master;
    // data=nullptr, no interface → hits the no-interface code path
    EXPECT_FALSE(master.sendSingleDatagram(
        Command::APRD, 0x01, 0x0000, 0x0130, nullptr, 4, true));
}

// ============================================================================
// handleRxFrame / parseEtherCATFrame — frame parsing tests
// ============================================================================

TEST(MasterFrameCoverage, HandleRxFrameTooShort) {
    Master master;
    // Frame shorter than EthernetHeader + FrameHeader (14 + 2 = 16)
    uint8_t buf[10] = {};
    master.handleRxFrame(buf, sizeof(buf)); // should not crash
}

TEST(MasterFrameCoverage, HandleRxFrameWrongEtherType) {
    Master master;
    // Valid length but wrong EtherType
    uint8_t buf[64] = {};
    // Set EtherType to 0x0800 (IPv4) instead of 0x88A4
    buf[12] = 0x08;
    buf[13] = 0x00;
    master.handleRxFrame(buf, sizeof(buf)); // should be silently ignored
}

TEST(MasterFrameCoverage, HandleRxFramePayloadTooShort) {
    Master master;
    // Correct EtherType but ec_len says more data than available
    uint8_t buf[20] = {};
    buf[12] = 0x88; buf[13] = 0xA4;
    // FrameHeader with length = 0x7FF (max) but only 4 bytes available
    buf[14] = 0xFF; buf[15] = 0x17; // length=0x7FF, type=1
    master.handleRxFrame(buf, sizeof(buf)); // should bail
}

TEST(MasterFrameCoverage, HandleRxFrameWkcMissing) {
    Master master;
    // Valid EtherType, valid ec_len, but frame too short for datagram + data + WKC
    auto frame = buildEtherCATFrame(Command::APRD, 0x01, 0, 0x0130, nullptr, 2, 1);
    // Truncate before WKC
    size_t truncated_len = 26 + 2 - 1; // just 1 byte short of WKC end
    master.handleRxFrame(frame.data(), truncated_len); // should bail gracefully
}

TEST(MasterFrameCoverage, HandleRxFrameValidAPRD) {
    Master master;
    uint8_t payload[2] = {0x02, 0x00}; // PRE_OP state
    auto frame = buildEtherCATFrame(Command::APRD, 0x55, 0x0000, 0x0130, payload, 2, 1);

    // This will be an unrouted packet since no waiter is registered
    master.handleRxFrame(frame.data(), frame.size()); // should enqueue to rxQueue
}

TEST(MasterFrameCoverage, HandleRxFrameFireAndForgetAPRD) {
    Master master;
    uint8_t payload[2] = {0x08, 0x00}; // OP state
    // Use fire-and-forget index (0xFE)
    auto frame = buildEtherCATFrame(Command::APRD, 0xFE, 0x0000, 0x0130, payload, 2, 1);
    master.handleRxFrame(frame.data(), frame.size()); // should route to txpdo_rx_queue_
}

TEST(MasterFrameCoverage, HandleRxFrameFireAndForgetNonAPRD) {
    Master master;
    uint8_t payload[2] = {0x08, 0x00};
    // Fire-and-forget with non-APRD command should NOT enqueue
    auto frame = buildEtherCATFrame(Command::APWR, 0xFE, 0x0000, 0x0130, payload, 2, 1);
    master.handleRxFrame(frame.data(), frame.size());
}

// ============================================================================
// readRegister — queued responses
// ============================================================================

TEST(MasterFrameCoverage, EcAprdWithQueuedResponse) {
    Master master;
    // Push a response for ADP=0x0000, ADO=0x0130
    uint8_t resp_data[2] = {0x02, 0x00};
    master.pushAprdResponse(true, 0x0000, 0x0130, resp_data, 2);

    uint16_t al_le = 0;
    bool ok = master.readRegister(0x0000, 0x0130, al_le, 200);
    EXPECT_TRUE(ok);
    EXPECT_EQ(al_le, 0x0002); // PRE_OP in LE
}

TEST(MasterFrameCoverage, EcAprdWithQueuedResponseFail) {
    Master master;
    // Push a failing response
    master.pushAprdResponse(false, 0x0000, 0x0130, nullptr, 0);

    uint16_t val = 0;
    EXPECT_FALSE(master.readRegister(0x0000, 0x0130, val, 200));
}

TEST(MasterFrameCoverage, EcAprdWithQueuedResponseMismatch) {
    Master master;
    // Push response for a different ADO than what we read
    uint8_t resp_data[2] = {0xFF, 0xFF};
    master.pushAprdResponse(true, 0x0000, 0x9999, resp_data, 2);

    // Read from different ADO — should still "succeed" (fallback returns zeroed data)
    uint16_t val = 0xDEAD;
    bool ok = master.readRegister(0x0000, 0x0130, val, 200);
    // When aprd_responses_ is non-empty but no match for this ado, it zeroes out
    EXPECT_TRUE(ok);
    EXPECT_EQ(val, 0);

    // Now clear queued responses
    master.clearAprdResponses();
}

TEST(MasterFrameCoverage, EcAprdWithTestCallback) {
    Master master;
    master.setAprdTestCallback([](uint16_t adp, uint16_t ado,
                                  void* out, uint16_t len,
                                  unsigned int timeout_ms) -> bool {
        if (ado == 0x0130 && out && len >= 2) {
            uint16_t val = 0x0004; // SAFE_OP
            std::memcpy(out, &val, 2);
            return true;
        }
        return false;
    });

    uint16_t al_le = 0;
    EXPECT_TRUE(master.readRegister(0x0000, 0x0130, al_le, 200));
    EXPECT_EQ(al_le, 0x0004);
}

// ============================================================================
// writeRegister — EEPROM shortcircuit
// ============================================================================

TEST(MasterFrameCoverage, EcApwrEepromShortcut) {
    Master master;
    // When there are queued aprd_responses, writeRegister to EEPCTL should shortcircuit
    uint8_t dummy[2] = {0x01, 0x00};
    master.pushAprdResponse(true, 0, 0, dummy, 2);

    uint16_t eepctl_data = 0x1234;
    // EC_REG_EEPCTL = 0x0502
    EXPECT_TRUE(master.writeRegister(0x0000, 0x0502, &eepctl_data, sizeof(eepctl_data), 200));

    master.clearAprdResponses();
}

// ============================================================================
// requestSlaveApplicationLayerState / readSlaveApplicationLayerState
// ============================================================================

TEST(MasterFrameCoverage, RequestSlaveState) {
    Master master;
    bool write_called = false;
    uint16_t written_value = 0;
    master.setApwrTestCallback([&](uint16_t adp, uint16_t ado,
                                   const void* data, uint16_t len,
                                   unsigned int timeout_ms) -> bool {
        if (ado == 0x0120 && len == 2) { // AL_CONTROL
            write_called = true;
            std::memcpy(&written_value, data, 2);
            return true;
        }
        return false;
    });

    EXPECT_TRUE(master.requestSlaveApplicationLayerState(0, 0x02)); // request PRE_OP
    EXPECT_TRUE(write_called);
    EXPECT_EQ(written_value, 0x0002); // LE
}

TEST(MasterFrameCoverage, GetSlaveState) {
    Master master;
    master.setAprdTestCallback([](uint16_t adp, uint16_t ado,
                                  void* out, uint16_t len,
                                  unsigned int timeout_ms) -> bool {
        if (ado == 0x0130 && out && len >= 2) { // AL_STATUS
            uint16_t val = 0x0008; // OP
            std::memcpy(out, &val, 2);
            return true;
        }
        return false;
    });

    uint8_t state = 0;
    EXPECT_TRUE(master.readSlaveApplicationLayerState(0, state));
    EXPECT_EQ(state, 0x08); // OP
}

TEST(MasterFrameCoverage, GetSlaveStateFails) {
    Master master;
    master.setAprdTestCallback([](uint16_t, uint16_t, void*, uint16_t,
                                  unsigned int) -> bool { return false; });
    uint8_t state = 0xFF;
    EXPECT_FALSE(master.readSlaveApplicationLayerState(0, state));
}

// ============================================================================
// isRunning / getSrcMac / networkInterface / getDiscoveredSlaveCount
// ============================================================================

TEST(MasterFrameCoverage, IsRunningBeforeStart) {
    Master master;
    EXPECT_FALSE(master.isRunning());
}

TEST(MasterFrameCoverage, GetSrcMac) {
    Master master;
    const uint8_t* mac = master.getSrcMac();
    EXPECT_NE(mac, nullptr);
    // Should be all zeros before start
    for (int i = 0; i < 6; i++) {
        EXPECT_EQ(mac[i], 0);
    }
}

TEST(MasterFrameCoverage, NetworkInterface) {
    Master master;
    const NetworkInterface* ni = master.networkInterface();
    EXPECT_NE(ni, nullptr);
}

TEST(MasterFrameCoverage, GetDiscoveredSlaveCountInitiallyZero) {
    Master master;
    EXPECT_EQ(master.getDiscoveredSlaveCount(), 0);
}

// ============================================================================
// siiReadString — stub always returns false
// ============================================================================

TEST(MasterFrameCoverage, SiiReadString) {
    Master master;
    char buf[64] = {};
    // Stub always returns false
    EXPECT_FALSE(master.siiReadString(0x0000, 1, buf, sizeof(buf)));
}

// ============================================================================
// forceMailboxDefaults
// ============================================================================

TEST(MasterFrameCoverage, ForceMailboxDefaultsOutOfRange) {
    Master master;
    // kMaxPDOSlaves is typically 16 or 32; use a value way above
    EXPECT_FALSE(master.forceMailboxDefaults(9999));
}

TEST(MasterFrameCoverage, ForceMailboxDefaultsValid) {
    Master master;
    master.setApwrTestCallback([](uint16_t, uint16_t, const void*, uint16_t,
                                  unsigned int) -> bool { return true; });

    // forceMailboxDefaults writes SM0/SM1 with conservative defaults
    bool result = master.forceMailboxDefaults(0);
    // May succeed or fail depending on SM validation; just exercise the path
    (void)result;
}

// ============================================================================
// autoConfigureMailbox
// ============================================================================

TEST(MasterFrameCoverage, AutoConfigureMailboxInfoLevel) {
    Master master;
    // Uses APRD/APWR callbacks for SII, SDO config
    master.setAprdTestCallback([](uint16_t, uint16_t ado, void* out, uint16_t len,
                                  unsigned int) -> bool {
        if (out && len > 0) std::memset(out, 0, len);
        return true;
    });
    master.setApwrTestCallback([](uint16_t, uint16_t, const void*, uint16_t,
                                  unsigned int) -> bool { return true; });

    // This calls configureMailboxFromSii + setMailboxOverride + SDO config
    bool ok = master.autoConfigureMailbox(0, Tether::Platform::LogLevel::Info);
    EXPECT_TRUE(ok);
}

TEST(MasterFrameCoverage, AutoConfigureMailboxDebugLevel) {
    Master master;
    master.setAprdTestCallback([](uint16_t, uint16_t ado, void* out, uint16_t len,
                                  unsigned int) -> bool {
        if (out && len > 0) std::memset(out, 0, len);
        return true;
    });
    master.setApwrTestCallback([](uint16_t, uint16_t, const void*, uint16_t,
                                  unsigned int) -> bool { return true; });

    // Debug-level logging exercises more branches
    bool ok = master.autoConfigureMailbox(0, Tether::Platform::LogLevel::Debug);
    EXPECT_TRUE(ok);
}

// ============================================================================
// configurePDOSyncManagersFromSii
// ============================================================================

TEST(MasterFrameCoverage, ConfigurePDOSMsOutOfRange) {
    Master master;
    EXPECT_FALSE(master.configureProcessDataSyncManagersFromSii(9999));
}

TEST(MasterFrameCoverage, ConfigurePDOSMsWithCallbacks) {
    Master master;
    // Provide APRD/APWR callbacks that return zeros (SII read will fail)
    master.setAprdTestCallback([](uint16_t, uint16_t, void* out, uint16_t len,
                                  unsigned int) -> bool {
        if (out && len > 0) std::memset(out, 0, len);
        return true;
    });
    master.setApwrTestCallback([](uint16_t, uint16_t, const void*, uint16_t,
                                  unsigned int) -> bool { return true; });

    // SII read will fail (all zeros) → fallback to defaults for SM2/SM3
    bool ok = master.configureProcessDataSyncManagersFromSii(0);
    EXPECT_TRUE(ok);
}

// ============================================================================
// configureMailboxFromSii — directly (via master wrapper)
// ============================================================================

TEST(MasterFrameCoverage, ConfigureMailboxFromSiiDefaults) {
    Master master;
    // APRD returns empty/zero data → SII read fails → defaults applied
    master.setAprdTestCallback([](uint16_t, uint16_t, void* out, uint16_t len,
                                  unsigned int) -> bool {
        if (out && len > 0) std::memset(out, 0, len);
        return true;
    });
    master.setApwrTestCallback([](uint16_t, uint16_t, const void*, uint16_t,
                                  unsigned int) -> bool { return true; });

    uint16_t wr_addr = 0, wr_len = 0, rd_addr = 0, rd_len = 0, proto = 0;
    bool ok = master.configureMailboxFromSii(0x0000, &wr_addr, &wr_len,
                                            &rd_addr, &rd_len, &proto);
    EXPECT_TRUE(ok);
    // Should have set defaults
    // Defaults: wr(Receive/M→S)=0x1000, rd(Send/S→M)=0x1400
    EXPECT_EQ(wr_addr, 0x1000);
    EXPECT_EQ(wr_len, 256);
    EXPECT_EQ(rd_addr, 0x1400);
    EXPECT_EQ(rd_len, 256);
    EXPECT_NE(proto, 0); // default includes CoE|EoE|AoE
}

TEST(MasterFrameCoverage, ConfigureMailboxFromSiiNullOutputs) {
    Master master;
    master.setAprdTestCallback([](uint16_t, uint16_t, void* out, uint16_t len,
                                  unsigned int) -> bool {
        if (out && len > 0) std::memset(out, 0, len);
        return true;
    });
    master.setApwrTestCallback([](uint16_t, uint16_t, const void*, uint16_t,
                                  unsigned int) -> bool { return true; });

    // Pass nullptr for all outputs — should not crash
    bool ok = master.configureMailboxFromSii(0x0000, nullptr, nullptr,
                                            nullptr, nullptr, nullptr);
    EXPECT_TRUE(ok);
}

// ============================================================================
// setEnableMailboxFallback / isMailboxFallbackEnabled
// ============================================================================

TEST(MasterFrameCoverage, MailboxFallbackConfig) {
    Master master;
    EXPECT_FALSE(master.isMailboxFallbackEnabled());
    master.setEnableMailboxFallback(true);
    EXPECT_TRUE(master.isMailboxFallbackEnabled());
    master.setEnableMailboxFallback(false);
    EXPECT_FALSE(master.isMailboxFallbackEnabled());
}

TEST(MasterFrameCoverage, MailboxFallbackCallback) {
    Master master;
    uint16_t cb_slave = 0xFFFF;
    master.setMailboxFallbackCallback([&](uint16_t slave) { cb_slave = slave; });
    master.setApwrTestCallback([](uint16_t, uint16_t, const void*, uint16_t,
                                  unsigned int) -> bool { return true; });

    // forceMailboxDefaults triggers the callback
    master.forceMailboxDefaults(0);
    EXPECT_EQ(cb_slave, 0);
}

// ============================================================================
// getStats (only available with TETHER_ENABLE_ETHERCAT_STATS)
// ============================================================================

#if TETHER_ENABLE_ETHERCAT_STATS
TEST(MasterFrameCoverage, GetStatsAfterFrames) {
    Master master;
    // Process a few frames to increment counters
    uint8_t payload[2] = {0x01, 0x00};
    auto frame = buildEtherCATFrame(Command::APRD, 0x01, 0, 0x0130, payload, 2, 1);
    master.handleRxFrame(frame.data(), frame.size());
    master.handleRxFrame(frame.data(), frame.size());

    auto stats = master.getStats();
    EXPECT_GE(stats.rx_frame_count, 2u);
}
#endif

// ============================================================================
// Multiple queued responses test (uses pushAprdResponse + ecAprd)
// ============================================================================

TEST(MasterFrameCoverage, MultipleQueuedResponsesDrained) {
    Master master;

    // Queue 3 responses
    uint8_t d1[2] = {0x01, 0x00};
    uint8_t d2[2] = {0x02, 0x00};
    uint8_t d3[2] = {0x04, 0x00};
    master.pushAprdResponse(true, 0x0000, 0x0130, d1, 2);
    master.pushAprdResponse(true, 0x0000, 0x0130, d2, 2);
    master.pushAprdResponse(true, 0x0000, 0x0130, d3, 2);

    uint16_t val = 0;
    EXPECT_TRUE(master.readRegister(0x0000, 0x0130, val, 200));
    EXPECT_EQ(val, 0x0001);

    EXPECT_TRUE(master.readRegister(0x0000, 0x0130, val, 200));
    EXPECT_EQ(val, 0x0002);

    EXPECT_TRUE(master.readRegister(0x0000, 0x0130, val, 200));
    EXPECT_EQ(val, 0x0004);
}

// ============================================================================
// EEPROM status shortcircuit in ecAprd
// ============================================================================

TEST(MasterFrameCoverage, EcAprdEepromStatusShortcircuit) {
    Master master;
    // When apwr_cb_ is set and ado == 0x0502 (EEPSTAT), ecAprd shortcircuits
    master.setApwrTestCallback([](uint16_t, uint16_t, const void*, uint16_t,
                                  unsigned int) -> bool { return true; });

    uint16_t val = 0xDEAD;
    // 0x0502 is EC_REG_EEPSTAT (not what the shortcircuit checks, let me verify)
    // Actually ecAprd checks: if (ado == 0x0502 && apwr_cb_) — yes, 0x0502 = EC_REG_EEPCTL/EEPSTAT
    bool ok = master.readRegister(0x0000, 0x0502, val, 200);
    EXPECT_TRUE(ok);
    EXPECT_EQ(val, 0); // zeroed out
}

// ============================================================================
// logDiscoveredSlavesSummary — exercises SII reading paths
// ============================================================================

TEST(MasterFrameCoverage, LogDiscoveredSlavesSummaryZero) {
    Master master;
    // No slaves discovered — should just log "0 slave(s)"
    master.logDiscoveredSlavesSummary("test");
}

// ============================================================================
// Config constructor
// ============================================================================

TEST(MasterFrameCoverage, ConfigConstructor) {
    Master::Config cfg;
    cfg.rx_queue_depth = 128;
    cfg.txpdo_queue_depth = 16;
    cfg.enable_mailbox_fallback = true;

    Master master(cfg);
    EXPECT_TRUE(master.isMailboxFallbackEnabled());
    EXPECT_NE(master.rxQueue(), nullptr);
    EXPECT_NE(master.txpdoRxQueue(), nullptr);
}

// ============================================================================
// coeSdoUpload / coeSdoDownload — mailbox size validation
// ============================================================================

TEST(MasterFrameCoverage, CoeSdoUploadSmallMailbox) {
    Master master;
    master.setAprdTestCallback([](uint16_t, uint16_t, void* out, uint16_t len,
                                  unsigned int) -> bool {
        if (out && len > 0) std::memset(out, 0, len);
        return true;
    });
    master.setApwrTestCallback([](uint16_t, uint16_t, const void*, uint16_t,
                                  unsigned int) -> bool { return true; });

    uint8_t buf[256] = {};
    size_t out_len = 0;
    uint8_t mbx_cnt = 1;
    // mbx_wr_len = 1 (too small) → should fail early
    bool ok = master.coeSdoUpload(0x0000, &mbx_cnt,
                                   0x1400, 1,   // wr_addr, wr_len=1 (too small)
                                   0x1000, 256,  // rd_addr, rd_len
                                   0x6040, 0x00, // index, sub
                                   buf, sizeof(buf), &out_len, false);
    EXPECT_FALSE(ok);
}

TEST(MasterFrameCoverage, CoeSdoDownloadSmallMailbox) {
    Master master;
    master.setAprdTestCallback([](uint16_t, uint16_t, void* out, uint16_t len,
                                  unsigned int) -> bool {
        if (out && len > 0) std::memset(out, 0, len);
        return true;
    });
    master.setApwrTestCallback([](uint16_t, uint16_t, const void*, uint16_t,
                                  unsigned int) -> bool { return true; });

    uint8_t data[4] = {0x06, 0x00, 0x00, 0x00};
    uint8_t mbx_cnt = 1;
    // mbx_wr_len = 1 (too small) → should fail early
    bool ok = master.coeSdoDownload(0x0000, &mbx_cnt,
                                     0x1400, 1,   // wr_addr, wr_len=1 (too small)
                                     0x1000, 256,  // rd_addr, rd_len
                                     0x6040, 0x00, // index, sub
                                     data, sizeof(data), false);
    EXPECT_FALSE(ok);
}

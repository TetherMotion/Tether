#include <gtest/gtest.h>
#include "tether/ethercat/WriteVerify.hpp"
#include "tether/ethercat/WriteVerify.hpp" // ensure types are available

using namespace EtherCAT::Verify;

// Minimal stub transport for legacy tests
struct LegacyWVTransport : public IWriteVerifyTransport {
    uint8_t allocIdx() override { return 0; }
    bool sendDatagram(uint8_t, uint8_t, uint16_t, uint16_t, const void*, uint16_t, bool) override { return false; }
    bool waitForResponse(uint8_t, unsigned int, DatagramResponse&) override { return false; }
    bool readRegister(uint16_t, uint16_t, void*, uint16_t, unsigned int) override { return false; }
    void delayMs(unsigned int) override {}
};

TEST(WriteVerify_ConfigStats, BasicOps) {
    LegacyWVTransport transport;
    WriteVerifier wv(transport);
    reset_stats(wv);
    auto s = get_stats(wv);
    EXPECT_EQ(s.total_writes, 0);

    WriteVerifyConfig cfg = WriteVerifyConfig::defaults();
    cfg.retry_count = 1;
    set_config(wv, cfg);
    EXPECT_EQ(get_config(wv).retry_count, 1);

    set_enabled(wv, false);
    EXPECT_FALSE(is_enabled(wv));
    set_enabled(wv, true);
    EXPECT_TRUE(is_enabled(wv));

    // log_stats just exercises logging paths
    log_stats(wv);
}

TEST(WriteVerify_FastPath_APWR, SuccessAndFailure) {
    LegacyWVTransport transport;
    WriteVerifier wv(transport);
    set_enabled(wv, false);
    reset_stats(wv);
    auto s = get_stats(wv);
    EXPECT_EQ(s.total_writes, 0);

    // Use result factory helpers to exercise verify result constructors
    auto ok = WriteVerifyResult::Success(1, 1);
    EXPECT_TRUE(ok.success);
    EXPECT_EQ(ok.attempts, 1u);

    auto wf = WriteVerifyResult::WriteFailed(0, 1);
    EXPECT_FALSE(wf.success);

    auto vf = WriteVerifyResult::VerifyFailed(1, 1, 2, 0, 0x12, 0x34);
    EXPECT_FALSE(vf.success);
    EXPECT_FALSE(vf.verify_ok);
    EXPECT_EQ(vf.mismatch_offset, 0u);
}

TEST(WriteVerify_ConfigAdvanced, ConfigRoundtrip) {
    LegacyWVTransport transport;
    WriteVerifier wv(transport);
    WriteVerifyConfig cfg = WriteVerifyConfig::defaults();
    cfg.retry_count = 5;
    cfg.retry_delay_ms = 20;
    cfg.read_delay_ms = 2;
    cfg.log_failures = false;
    set_config(wv, cfg);

    auto got = get_config(wv);
    EXPECT_EQ(got.retry_count, 5u);
    EXPECT_EQ(got.retry_delay_ms, 20u);
    EXPECT_EQ(got.read_delay_ms, 2u);
    EXPECT_FALSE(got.log_failures);
}

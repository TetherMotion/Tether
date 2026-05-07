/**
 * @file test_voe_eoe.cpp
 * @brief Unit tests for VoE and EoE stub implementations.
 */
#include <gtest/gtest.h>
#include "tether/ethercat/EtherCATVoE.hpp"
#include "tether/ethercat/EtherCATEoE.hpp"

using namespace EtherCAT;

// ============================================================================
// VoE Stub Tests
// ============================================================================

TEST(VoEStub, InitAndDeinit) {
    EXPECT_TRUE(VoE::voe_init());
    VoE::voe_deinit(); // no-op, just verify no crash
}

TEST(VoEStub, IsInitialized) {
    EXPECT_TRUE(VoE::voe_is_initialized());
}

TEST(VoEStub, TransactReturnsNotInitialized) {
    VoE::VoERequest req;
    uint8_t resp[16];
    size_t resp_len = 0;
    auto result = VoE::voe_transact(&req, resp, sizeof(resp), &resp_len);
    EXPECT_FALSE(result.success);
    EXPECT_EQ(result.error_code, VoE::VoEError::NOT_INITIALIZED);
}

TEST(VoEStub, SendReturnsFalse) {
    VoE::VoERequest req;
    EXPECT_FALSE(VoE::voe_send(&req));
}

TEST(VoEStub, QueueRequestReturnsFalse) {
    VoE::VoERequest req;
    EXPECT_FALSE(VoE::voe_queue_request(req));
}

TEST(VoEStub, PendingCountReturnsZero) {
    EXPECT_EQ(VoE::voe_pending_count(), 0u);
}

TEST(VoEStub, RegisterHandlerReturnsFalse) {
    EXPECT_FALSE(VoE::voe_register_handler(0x1234, nullptr));
}

TEST(VoEStub, UnregisterHandlerNoOp) {
    VoE::voe_unregister_handler(0x1234); // no crash
}

TEST(VoEStub, GetStatsReturnsZeroed) {
    auto stats = VoE::voe_get_stats();
    EXPECT_EQ(stats.requests_sent, 0u);
    EXPECT_EQ(stats.responses_received, 0u);
}

TEST(VoEStub, ResetStatsNoOp) {
    VoE::voe_reset_stats(); // no crash
}

// ============================================================================
// EoE Stub Tests
// ============================================================================

TEST(EoEStub, InitAndDeinit) {
    EXPECT_TRUE(EoE::eoe_init());
    EoE::eoe_deinit(); // no-op
}

TEST(EoEStub, GetStatsReturnsZeroed) {
    auto stats = EoE::eoe_get_stats();
    EXPECT_EQ(stats.frames_sent, 0u);
    EXPECT_EQ(stats.frames_received, 0u);
}

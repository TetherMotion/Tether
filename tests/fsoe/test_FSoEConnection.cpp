#include <gtest/gtest.h>
#include "tether/fsoe/FSoEConnection.hpp"

using namespace FSoE;

TEST(FSoEConnection, Smoke) {
    ConnectionConfig cfg;
    cfg.connection_id = 0x1234;
    cfg.slave_addr = 0x100;
    cfg.input_size = 2;
    cfg.output_size = 2;
    cfg.fail_safe_values = {0xAA, 0x55, 0,0,0,0,0,0};

    FSoEConnection conn(cfg);
    EXPECT_FALSE(conn.isInitialized());
    EXPECT_TRUE(conn.initialize());
    EXPECT_TRUE(conn.isInitialized());

    // initial stats should be zeroed
    auto s = conn.getStats();
    EXPECT_EQ(s.frames_sent, 0u);
    EXPECT_EQ(s.frames_received, 0u);

    // safe outputs accept correct length, reject incorrect
    uint8_t outs[2] = {0x01, 0x02};
    EXPECT_TRUE(conn.setSafeOutputs(outs, 2));
    EXPECT_FALSE(conn.setSafeOutputs(outs, 1));

    // trigger fail-safe and verify behavior
    conn.triggerFailSafe(ErrorCode::ApplicationError);
    EXPECT_TRUE(conn.isFailSafe());
    EXPECT_FALSE(conn.setSafeOutputs(outs, 2));  // writes rejected in fail-safe

    // clear error should recover to Reset
    EXPECT_TRUE(conn.clearError());
    EXPECT_FALSE(conn.isFailSafe());

    // request session reset -> session id assigned and state updated
    EXPECT_TRUE(conn.requestSessionReset());
    EXPECT_NE(conn.getStatus().session_id, 0u);
    EXPECT_EQ(conn.getState(), ConnectionState::Session);

    // diagnostics contain connection id and state
    auto diag = conn.getDiagnostics();
    EXPECT_NE(diag.find("Connection ID"), std::string::npos);
    EXPECT_NE(diag.find(std::to_string(cfg.connection_id)), std::string::npos);
}

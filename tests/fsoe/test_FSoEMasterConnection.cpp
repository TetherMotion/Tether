#include <gtest/gtest.h>
#include "tether/fsoe/FSoEMasterConnection.hpp"

using namespace FSoE;

TEST(FSoEMasterConnection, Smoke) {
    MasterConnectionConfig cfg;
    cfg.connection_id = 0x1234;
    cfg.slave_addr = 0x100;
    cfg.slave_safety_addr = 0x100;
    cfg.input_size = 2;
    cfg.output_size = 2;
    cfg.fail_safe_values = {0xAA, 0x55, 0,0,0,0,0,0};

    FSoEMasterConnection conn(cfg);
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

    // trigger fail-safe from Reset state — goes back to Reset (NOT_OK transition)
    // Error code is preserved after resetConnection
    conn.triggerFailSafe(ErrorCode::ApplicationError);
    EXPECT_FALSE(conn.isFailSafe());
    EXPECT_EQ(conn.getState(), ConnectionState::Reset);
    EXPECT_NE(conn.getErrorCode(), ErrorCode::NoError);

    // setSafeOutputs is NOT blocked (isFailSafe is false after NOT_OK→Reset)
    EXPECT_TRUE(conn.setSafeOutputs(outs, 2));

    // clearError only works from Error state or Data+fail_safe
    // After NOT_OK→Reset, we're in Reset, so clearError returns false
    EXPECT_FALSE(conn.clearError());
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

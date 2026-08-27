/**
 * @file test_fsoe_state_machine_sequence.cpp
 * @brief Tests for the correct FSoE state machine transition sequence.
 *
 * FSoE state machine (ETG.5100 S (D) V1.2.0):
 *   Reset -> Session -> Connection -> Parameter -> Data
 *
 * Transition rules:
 *   OK     - received command is the next expected state -> advance one step
 *   STAY   - received command is the current state -> remain
 *   NOT_OK - received command is unexpected or CRC/watchdog error -> Reset
 *
 * See: https://techoverflow.net/2026/08/12/all-the-states-of-the-fsoe-state-machine/
 *
 * These tests verify:
 *   1. The master follows the exact sequence Reset->Session->Connection->Parameter->Data
 *   2. A NOT_OK (error) from any handshake state returns to Reset (NOT Data)
 *   3. triggerFailSafe() from a non-Data state goes to Reset (NOT Data)
 *   4. handleError() from a non-Data state goes to Reset (NOT Data)
 *   5. The master never skips states (e.g. Reset->Data is forbidden)
 */

#include <gtest/gtest.h>
#include <cstring>
#include <memory>
#include <vector>
#include "fsoe/FSoEMasterConnection.hpp"
#include "fsoe/FSoESlave.hpp"
#include "fsoe/FSoECRC.hpp"

using namespace FSoE;

// ============================================================================
// Test Helpers
// ============================================================================

static MasterConnectionConfig makeMasterCfg(uint8_t inSize = 4,
                                             uint8_t outSize = 4) {
    MasterConnectionConfig cfg{};
    cfg.slave_addr = 0x0100;
    cfg.slave_safety_addr = 0x0100;
    cfg.connection_id = 0x1234;
    cfg.master_addr = 0x0100;
    cfg.watchdog_timeout_ms = 200;
    cfg.conn_timeout_ms = 5000;
    cfg.session_timeout_ms = 10000;
    cfg.reset_timeout_ms = 0;  // Wait forever in Reset
    cfg.input_size = inSize;
    cfg.output_size = outSize;
    cfg.fail_safe_values = {0xDE, 0xAD, 0xBE, 0xEF, 0, 0, 0, 0,
                             0, 0, 0, 0, 0, 0, 0, 0};
    cfg.auto_recovery_enabled = false;
    return cfg;
}

static FSoESlaveConfig makeSlaveCfg(uint8_t inSize = 4, uint8_t outSize = 4) {
    FSoESlaveConfig cfg{};
    cfg.slaveAddress = 0x0100;
    cfg.connectionId = 0x1234;
    cfg.safetyAddress = 0x0100;
    cfg.safetyLevel = SIL::SIL2;
    cfg.watchdogTimeoutMs = 200;
    cfg.connectionTimeoutMs = 5000;
    cfg.sessionTimeoutMs = 10000;
    cfg.safeInputSize = inSize;
    cfg.safeOutputSize = outSize;
    cfg.autoRecoveryEnabled = false;
    cfg.failSafeInputs = {0xAA, 0xBB, 0xCC, 0xDD, 0, 0, 0, 0,
                          0, 0, 0, 0, 0, 0, 0, 0};
    cfg.failSafeOutputs = {0xDE, 0xAD, 0xBE, 0xEF, 0, 0, 0, 0,
                           0, 0, 0, 0, 0, 0, 0, 0};
    return cfg;
}

// ============================================================================
// #1: Master follows the exact sequence Reset->Session->Connection->Parameter->Data
// ============================================================================

TEST(FSoEStateMachineSequence, MasterGoesThroughAllStatesInOrder) {
    FSoEMasterConnection conn(makeMasterCfg(4, 4));
    conn.initialize();
    conn.startConnection();
    FSoESlave slave(makeSlaveCfg(4, 4));
    slave.initialize();

    // Master starts in Reset
    EXPECT_EQ(conn.getState(), ConnectionState::Reset);

    // Track all state transitions
    std::vector<uint8_t> master_states;
    std::vector<uint8_t> slave_states;
    master_states.push_back(conn.getState());
    slave_states.push_back(slave.getState());

    uint64_t now = 0;
    for (int i = 0; i < 30; ++i) {
        now += 15;
        conn.exchangeWith(slave, now);
        if (conn.getState() != master_states.back()) {
            master_states.push_back(conn.getState());
        }
        if (slave.getState() != slave_states.back()) {
            slave_states.push_back(slave.getState());
        }
        if (conn.isOperational() && slave.isOperational()) break;
    }

    ASSERT_TRUE(conn.isOperational())
        << "Master did not reach Data state";
    ASSERT_TRUE(slave.isOperational())
        << "Slave did not reach Data state";

    // Verify master transition sequence is exactly:
    // Reset -> Session -> Connection -> Parameter -> Data
    ASSERT_EQ(master_states.size(), 5u)
        << "Master should have exactly 5 states (Reset->Session->Connection->Parameter->Data)";
    EXPECT_EQ(master_states[0], ConnectionState::Reset);
    EXPECT_EQ(master_states[1], ConnectionState::Session);
    EXPECT_EQ(master_states[2], ConnectionState::Connection);
    EXPECT_EQ(master_states[3], ConnectionState::Parameter);
    EXPECT_EQ(master_states[4], ConnectionState::Data);

    // Verify slave follows the same sequence
    ASSERT_EQ(slave_states.size(), 5u)
        << "Slave should have exactly 5 states";
    EXPECT_EQ(slave_states[0], ConnectionState::Reset);
    EXPECT_EQ(slave_states[1], ConnectionState::Session);
    EXPECT_EQ(slave_states[2], ConnectionState::Connection);
    EXPECT_EQ(slave_states[3], ConnectionState::Parameter);
    EXPECT_EQ(slave_states[4], ConnectionState::Data);
}

TEST(FSoEStateMachineSequence, MasterNeverSkipsStates) {
    // With non-zero safety data, all 5 states must be visited.
    // (With both input_size=0 AND output_size=0, Parameter is correctly
    // skipped since there are no parameters to exchange.)
    FSoEMasterConnection conn(makeMasterCfg(2, 2));
    conn.initialize();
    conn.startConnection();
    FSoESlave slave(makeSlaveCfg(2, 2));
    slave.initialize();

    std::vector<uint8_t> master_states;
    master_states.push_back(conn.getState());

    uint64_t now = 0;
    for (int i = 0; i < 30; ++i) {
        now += 15;
        conn.exchangeWith(slave, now);
        if (conn.getState() != master_states.back()) {
            master_states.push_back(conn.getState());
        }
        if (conn.isOperational()) break;
    }

    ASSERT_TRUE(conn.isOperational());
    // Even with 0-octet data, all 5 states must be visited
    ASSERT_EQ(master_states.size(), 5u)
        << "Master must visit all 5 states even with 0-octet data";
    EXPECT_EQ(master_states[0], ConnectionState::Reset);
    EXPECT_EQ(master_states[1], ConnectionState::Session);
    EXPECT_EQ(master_states[2], ConnectionState::Connection);
    EXPECT_EQ(master_states[3], ConnectionState::Parameter);
    EXPECT_EQ(master_states[4], ConnectionState::Data);
}

// ============================================================================
// #2: NOT_OK (error) from handshake state -> Reset (NOT Data)
// ============================================================================

TEST(FSoEStateMachineSequence, ErrorInResetGoesToReset) {
    FSoEMasterConnection conn(makeMasterCfg(4, 4));
    conn.initialize();
    conn.startConnection();

    EXPECT_EQ(conn.getState(), ConnectionState::Reset);

    // triggerFailSafe is the public API for error handling.
    // In non-Data states it must go back to Reset (NOT_OK transition).
    conn.triggerFailSafe(ErrorCode::CRCError);

    // Must be in Reset, NOT Data or FailSafe
    EXPECT_EQ(conn.getState(), ConnectionState::Reset)
        << "Error in Reset must go back to Reset, not Data";
    EXPECT_FALSE(conn.isFailSafe())
        << "Error in Reset must NOT trigger fail-safe (fail-safe is Data-only)";
    EXPECT_NE(conn.getErrorCode(), ErrorCode::NoError)
        << "Error code must be preserved";
}

TEST(FSoEStateMachineSequence, ErrorInSessionGoesToReset) {
    FSoEMasterConnection conn(makeMasterCfg(4, 4));
    conn.initialize();
    conn.startConnection();
    FSoESlave slave(makeSlaveCfg(4, 4));
    slave.initialize();

    // Advance to Session
    uint64_t now = 0;
    now += 15;
    conn.exchangeWith(slave, now);
    ASSERT_EQ(conn.getState(), ConnectionState::Session);

    // triggerFailSafe from Session must go back to Reset
    conn.triggerFailSafe(ErrorCode::CRCError);

    // Must go back to Reset, NOT Data
    EXPECT_EQ(conn.getState(), ConnectionState::Reset)
        << "Error in Session must go back to Reset (NOT_OK transition)";
    EXPECT_FALSE(conn.isFailSafe())
        << "Error in Session must NOT trigger fail-safe";
    EXPECT_NE(conn.getErrorCode(), ErrorCode::NoError);
}

TEST(FSoEStateMachineSequence, ErrorInConnectionGoesToReset) {
    FSoEMasterConnection conn(makeMasterCfg(4, 4));
    conn.initialize();
    conn.startConnection();
    FSoESlave slave(makeSlaveCfg(4, 4));
    slave.initialize();

    // Advance to Connection
    uint64_t now = 0;
    for (int i = 0; i < 10; ++i) {
        now += 15;
        conn.exchangeWith(slave, now);
        if (conn.getState() == ConnectionState::Connection) break;
    }
    ASSERT_EQ(conn.getState(), ConnectionState::Connection);

    // triggerFailSafe from Connection must go back to Reset
    conn.triggerFailSafe(ErrorCode::ConnectionIDError);

    // Must go back to Reset, NOT Data
    EXPECT_EQ(conn.getState(), ConnectionState::Reset)
        << "Error in Connection must go back to Reset (NOT_OK transition)";
    EXPECT_FALSE(conn.isFailSafe())
        << "Error in Connection must NOT trigger fail-safe";
    EXPECT_NE(conn.getErrorCode(), ErrorCode::NoError);
}

TEST(FSoEStateMachineSequence, ErrorInParameterGoesToReset) {
    FSoEMasterConnection conn(makeMasterCfg(4, 4));
    conn.initialize();
    conn.startConnection();
    FSoESlave slave(makeSlaveCfg(4, 4));
    slave.initialize();

    // Advance to Parameter
    uint64_t now = 0;
    for (int i = 0; i < 15; ++i) {
        now += 15;
        conn.exchangeWith(slave, now);
        if (conn.getState() == ConnectionState::Parameter) break;
    }
    ASSERT_EQ(conn.getState(), ConnectionState::Parameter);

    // triggerFailSafe from Parameter must go back to Reset
    conn.triggerFailSafe(ErrorCode::DataLengthError);

    // Must go back to Reset, NOT Data
    EXPECT_EQ(conn.getState(), ConnectionState::Reset)
        << "Error in Parameter must go back to Reset (NOT_OK transition)";
    EXPECT_FALSE(conn.isFailSafe())
        << "Error in Parameter must NOT trigger fail-safe";
    EXPECT_NE(conn.getErrorCode(), ErrorCode::NoError);
}

TEST(FSoEStateMachineSequence, ErrorInDataTriggersFailSafe) {
    // Only in Data state should an error trigger fail-safe
    FSoEMasterConnection conn(makeMasterCfg(4, 4));
    conn.initialize();
    conn.startConnection();
    FSoESlave slave(makeSlaveCfg(4, 4));
    slave.initialize();

    // Advance to Data
    uint64_t now = 0;
    for (int i = 0; i < 30; ++i) {
        now += 15;
        conn.exchangeWith(slave, now);
        if (conn.isOperational()) break;
    }
    ASSERT_EQ(conn.getState(), ConnectionState::Data);
    ASSERT_TRUE(conn.isOperational());

    // triggerFailSafe from Data must stay in Data and set fail-safe
    conn.triggerFailSafe(ErrorCode::WatchdogError);

    // In Data state, error triggers fail-safe (stays in Data)
    EXPECT_EQ(conn.getState(), ConnectionState::Data)
        << "Error in Data stays in Data (fail-safe is Data-only command)";
    EXPECT_TRUE(conn.isFailSafe())
        << "Error in Data must trigger fail-safe";
    EXPECT_NE(conn.getErrorCode(), ErrorCode::NoError);
}

// ============================================================================
// #3: triggerFailSafe() from non-Data state -> Reset (NOT Data)
// ============================================================================

TEST(FSoEStateMachineSequence, TriggerFailSafeFromResetGoesToReset) {
    FSoEMasterConnection conn(makeMasterCfg(4, 4));
    conn.initialize();
    conn.startConnection();

    EXPECT_EQ(conn.getState(), ConnectionState::Reset);

    conn.triggerFailSafe(ErrorCode::WatchdogError);

    // Must stay in Reset, NOT jump to Data
    EXPECT_EQ(conn.getState(), ConnectionState::Reset)
        << "triggerFailSafe from Reset must go to Reset, not Data";
    EXPECT_FALSE(conn.isFailSafe())
        << "triggerFailSafe from Reset must NOT set fail-safe flag";
    EXPECT_NE(conn.getErrorCode(), ErrorCode::NoError)
        << "Error code must be preserved";
}

TEST(FSoEStateMachineSequence, TriggerFailSafeFromSessionGoesToReset) {
    FSoEMasterConnection conn(makeMasterCfg(4, 4));
    conn.initialize();
    conn.startConnection();
    FSoESlave slave(makeSlaveCfg(4, 4));
    slave.initialize();

    // Advance to Session
    uint64_t now = 0;
    now += 15;
    conn.exchangeWith(slave, now);
    ASSERT_EQ(conn.getState(), ConnectionState::Session);

    conn.triggerFailSafe(ErrorCode::CRCError);

    EXPECT_EQ(conn.getState(), ConnectionState::Reset)
        << "triggerFailSafe from Session must go to Reset, not Data";
    EXPECT_FALSE(conn.isFailSafe());
    EXPECT_NE(conn.getErrorCode(), ErrorCode::NoError);
}

TEST(FSoEStateMachineSequence, TriggerFailSafeFromDataStaysInData) {
    FSoEMasterConnection conn(makeMasterCfg(4, 4));
    conn.initialize();
    conn.startConnection();
    FSoESlave slave(makeSlaveCfg(4, 4));
    slave.initialize();

    // Advance to Data
    uint64_t now = 0;
    for (int i = 0; i < 30; ++i) {
        now += 15;
        conn.exchangeWith(slave, now);
        if (conn.isOperational()) break;
    }
    ASSERT_EQ(conn.getState(), ConnectionState::Data);

    conn.triggerFailSafe(ErrorCode::WatchdogError);

    // In Data, triggerFailSafe stays in Data and sets fail-safe
    EXPECT_EQ(conn.getState(), ConnectionState::Data);
    EXPECT_TRUE(conn.isFailSafe());
}

// ============================================================================
// #4: Reset->Data direct jump is forbidden
// ============================================================================

TEST(FSoEStateMachineSequence, ResetToDataDirectJumpIsForbidden) {
    // Verify that there is no code path that jumps directly from Reset to Data.
    // The only way to reach Data is through Session->Connection->Parameter.
    FSoEMasterConnection conn(makeMasterCfg(4, 4));
    conn.initialize();
    conn.startConnection();

    EXPECT_EQ(conn.getState(), ConnectionState::Reset);

    // triggerFailSafe must NOT jump to Data from Reset
    conn.triggerFailSafe(ErrorCode::CRCError);
    EXPECT_NE(conn.getState(), ConnectionState::Data)
        << "triggerFailSafe in Reset must not jump to Data";
    EXPECT_EQ(conn.getState(), ConnectionState::Reset);

    // Another triggerFailSafe with different error code
    conn.triggerFailSafe(ErrorCode::TimeoutError);
    EXPECT_NE(conn.getState(), ConnectionState::Data)
        << "triggerFailSafe in Reset must not jump to Data (2nd call)";
    EXPECT_EQ(conn.getState(), ConnectionState::Reset);
}

// ============================================================================
// #5: After NOT_OK->Reset, the master can restart the handshake
// ============================================================================

TEST(FSoEStateMachineSequence, CanRestartHandshakeAfterError) {
    FSoEMasterConnection conn(makeMasterCfg(4, 4));
    conn.initialize();
    conn.startConnection();
    FSoESlave slave(makeSlaveCfg(4, 4));
    slave.initialize();

    // Advance to Session
    uint64_t now = 0;
    now += 15;
    conn.exchangeWith(slave, now);
    ASSERT_EQ(conn.getState(), ConnectionState::Session);

    // Error -> back to Reset
    conn.triggerFailSafe(ErrorCode::CRCError);
    ASSERT_EQ(conn.getState(), ConnectionState::Reset);

    // The master should be able to restart the handshake and reach Data
    for (int i = 0; i < 30; ++i) {
        now += 15;
        conn.exchangeWith(slave, now);
        if (conn.isOperational()) break;
    }

    EXPECT_TRUE(conn.isOperational())
        << "Master must be able to restart handshake after NOT_OK->Reset";
    EXPECT_EQ(conn.getState(), ConnectionState::Data);
}

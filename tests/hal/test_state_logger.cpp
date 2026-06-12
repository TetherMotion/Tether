/**
 * @file test_state_logger.cpp
 * @brief Unit tests for State Machine Logger
 */

#include <gtest/gtest.h>
#include <magic_enum/magic_enum.hpp>
#include "tether/hal/StateMachineLogger.hpp"
#include "mocks/MockHAL.hpp"

using namespace EtherCAT::HAL;
using namespace EtherCAT::HAL::mock;

// ============================================================================
// ALState Tests
// ============================================================================

TEST(ALStateTest, StateToString) {
    EXPECT_EQ(alStateToString(ALState::Init), "Init");
    EXPECT_EQ(alStateToString(ALState::PreOp), "PreOp");
    EXPECT_EQ(alStateToString(ALState::SafeOp), "SafeOp");
    EXPECT_EQ(alStateToString(ALState::Op), "Op");
    EXPECT_EQ(alStateToString(ALState::Bootstrap), "Bootstrap");
    EXPECT_EQ(alStateToString(ALState::Unknown), "Unknown");
}

TEST(ALStateTest, ErrorStates) {
    EXPECT_EQ(alStateToString(ALState::InitError), "Init+Error");
    EXPECT_EQ(alStateToString(ALState::PreOpError), "PreOp+Error");
    EXPECT_EQ(alStateToString(ALState::SafeOpError), "SafeOp+Error");
    EXPECT_EQ(alStateToString(ALState::OpError), "Op+Error");
}

TEST(ALStateTest, HasError) {
    EXPECT_FALSE(alStateHasError(ALState::Init));
    EXPECT_FALSE(alStateHasError(ALState::PreOp));
    EXPECT_FALSE(alStateHasError(ALState::SafeOp));
    EXPECT_FALSE(alStateHasError(ALState::Op));
    
    EXPECT_TRUE(alStateHasError(ALState::InitError));
    EXPECT_TRUE(alStateHasError(ALState::PreOpError));
    EXPECT_TRUE(alStateHasError(ALState::SafeOpError));
    EXPECT_TRUE(alStateHasError(ALState::OpError));
}

TEST(ALStateTest, BaseState) {
    EXPECT_EQ(alStateBase(ALState::Init), ALState::Init);
    EXPECT_EQ(alStateBase(ALState::InitError), ALState::Init);
    EXPECT_EQ(alStateBase(ALState::PreOpError), ALState::PreOp);
    EXPECT_EQ(alStateBase(ALState::SafeOpError), ALState::SafeOp);
}

// ============================================================================
// StateTransition Tests
// ============================================================================

TEST(StateTransitionTest, IsError) {
    StateTransition t;
    t.newState = ALState::Init;
    t.alStatusCode = 0;
    EXPECT_FALSE(t.isError());
    
    t.newState = ALState::InitError;
    EXPECT_TRUE(t.isError());
    
    t.newState = ALState::Init;
    t.alStatusCode = 0x0001;  // Error code
    EXPECT_TRUE(t.isError());
}

TEST(StateTransitionTest, IsRecovery) {
    StateTransition t;
    t.previousState = ALState::InitError;
    t.newState = ALState::Init;
    EXPECT_TRUE(t.isRecovery());
    
    t.previousState = ALState::Init;
    t.newState = ALState::PreOp;
    EXPECT_FALSE(t.isRecovery());
}

TEST(StateTransitionTest, IsDowngrade) {
    StateTransition t;
    t.previousState = ALState::Op;
    t.newState = ALState::SafeOp;
    EXPECT_TRUE(t.isDowngrade());
    
    t.previousState = ALState::SafeOp;
    t.newState = ALState::Op;
    EXPECT_FALSE(t.isDowngrade());
}

// ============================================================================
// MockStateMachineLogger Tests
// ============================================================================

TEST(MockStateMachineLoggerTest, ExpectCalls) {
    MockStateMachineLogger mockLogger;
    
    StateMachineLoggerConfig config;
    EXPECT_CALL(mockLogger, init(testing::_)).WillOnce(testing::Return(Error::OK));
    EXPECT_CALL(mockLogger, recordTransition(testing::_)).Times(1);
    EXPECT_CALL(mockLogger, getCurrentState(0)).WillOnce(testing::Return(ALState::PreOp));
    
    mockLogger.init(config);
    
    StateTransition t;
    t.slaveIndex = 0;
    t.previousState = ALState::Init;
    t.newState = ALState::PreOp;
    mockLogger.recordTransition(t);
    
    EXPECT_EQ(mockLogger.getCurrentState(0), ALState::PreOp);
}

// ============================================================================
// SlaveStateTracker Tests
// ============================================================================

TEST(SlaveStateTrackerTest, Construction) {
    SlaveStateTracker tracker(0, 0x1001);
    
    EXPECT_EQ(tracker.getSlaveIndex(), 0);
    EXPECT_EQ(tracker.getConfiguredAddress(), 0x1001);
    EXPECT_EQ(tracker.getCurrentState(), ALState::Unknown);
}

TEST(SlaveStateTrackerTest, UpdateState) {
    SlaveStateTracker tracker(0);
    
    EXPECT_TRUE(tracker.updateState(ALState::Init));
    EXPECT_EQ(tracker.getCurrentState(), ALState::Init);
    
    // Same state - no change
    EXPECT_FALSE(tracker.updateState(ALState::Init));
    
    EXPECT_TRUE(tracker.updateState(ALState::PreOp));
    EXPECT_EQ(tracker.getCurrentState(), ALState::PreOp);
}

TEST(SlaveStateTrackerTest, RequestState) {
    SlaveStateTracker tracker(0);
    tracker.updateState(ALState::Init);
    
    tracker.requestState(ALState::PreOp);
    EXPECT_EQ(tracker.getRequestedState(), ALState::PreOp);
    
    // When actual state matches requested
    EXPECT_TRUE(tracker.updateState(ALState::PreOp));
    EXPECT_TRUE(tracker.getLastTransition().requested);
}

TEST(SlaveStateTrackerTest, ErrorState) {
    SlaveStateTracker tracker(0);
    tracker.updateState(ALState::Init);
    
    tracker.updateState(ALState::InitError, 0x0001);
    EXPECT_TRUE(tracker.hasError());
    EXPECT_EQ(tracker.getLastStatusCode(), 0x0001);
}

TEST(SlaveStateTrackerTest, Callback) {
    SlaveStateTracker tracker(0);
    
    StateTransition receivedTransition;
    bool callbackCalled = false;
    
    tracker.setCallback([&](const StateTransition& t) {
        callbackCalled = true;
        receivedTransition = t;
    });
    
    tracker.updateState(ALState::Init);
    EXPECT_TRUE(callbackCalled);
    EXPECT_EQ(receivedTransition.newState, ALState::Init);
    EXPECT_EQ(receivedTransition.previousState, ALState::Unknown);
}

TEST(SlaveStateTrackerTest, LastTransition) {
    SlaveStateTracker tracker(0, 0x1001);
    
    tracker.updateState(ALState::Init);
    tracker.updateState(ALState::PreOp);
    
    const auto& t = tracker.getLastTransition();
    EXPECT_EQ(t.slaveIndex, 0);
    EXPECT_EQ(t.configuredAddress, 0x1001);
    EXPECT_EQ(t.previousState, ALState::Init);
    EXPECT_EQ(t.newState, ALState::PreOp);
    EXPECT_GT(t.timestamp, 0u);
}

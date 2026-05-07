/**
 * @file test_HomingHandler.cpp
 * @brief Tests for CiA402::HomingHandler
 */
#include <gtest/gtest.h>
#include "tether/profiles/cia402/HomingHandler.hpp"
#include "tether/profiles/cia402/CiA402Config.hpp"

using namespace CiA402;

// ============================================================================
// HomingSwitchStatus struct
// ============================================================================

TEST(HomingSwitchStatusStruct, Default) {
    HomingSwitchStatus s{};
    EXPECT_FALSE(s.positiveLimitActive);
    EXPECT_FALSE(s.negativeLimitActive);
    EXPECT_FALSE(s.homeSwitchActive);
    EXPECT_FALSE(s.indexPulseDetected);
}

// ============================================================================
// HomingState enum (from HomingHandler)
// ============================================================================

TEST(HomingHandlerStateEnum, AllDistinct) {
    EXPECT_NE(static_cast<int>(HomingState::Idle),
              static_cast<int>(HomingState::SearchingSwitch));
    EXPECT_NE(static_cast<int>(HomingState::Reversing),
              static_cast<int>(HomingState::SearchingIndex));
    EXPECT_NE(static_cast<int>(HomingState::Attained),
              static_cast<int>(HomingState::Error));
}

// ============================================================================
// HomingError enum (from HomingHandler)
// ============================================================================

TEST(HomingHandlerErrorEnum, AllDistinct) {
    EXPECT_NE(static_cast<int>(HomingError::None),
              static_cast<int>(HomingError::Timeout));
    EXPECT_NE(static_cast<int>(HomingError::LimitReached),
              static_cast<int>(HomingError::DriveError));
    EXPECT_NE(static_cast<int>(HomingError::InvalidMethod),
              static_cast<int>(HomingError::Aborted));
}

// ============================================================================
// HomingHandler construction
// ============================================================================

TEST(HomingHandlerTest, DefaultConstruction) {
    HomingHandler handler;
    EXPECT_FALSE(handler.isComplete());
    EXPECT_FALSE(handler.hasError());
    EXPECT_EQ(handler.getState(), HomingState::Idle);
    EXPECT_EQ(handler.getError(), HomingError::None);
}

// ============================================================================
// Configuration
// ============================================================================

TEST(HomingHandlerTest, SetMethod) {
    HomingHandler handler;
    handler.setMethod(HomingMethod::CurrentPosition);
    EXPECT_EQ(handler.getMethod(), HomingMethod::CurrentPosition);
}

TEST(HomingHandlerTest, SetTimeout) {
    HomingHandler handler;
    handler.setTimeout(5000);
}

TEST(HomingHandlerTest, SetSwitchCallback) {
    HomingHandler handler;
    handler.setSwitchCallback([]() -> HomingSwitchStatus {
        return HomingSwitchStatus{};
    });
}

TEST(HomingHandlerTest, SetCompleteCallback) {
    HomingHandler handler;
    bool completed = false;
    handler.setCompleteCallback([&](bool success, HomingError err) {
        completed = true;
        (void)success;
        (void)err;
    });
    (void)completed;
}

// ============================================================================
// Start / Abort / Update
// ============================================================================

TEST(HomingHandlerTest, StartWithoutCallback) {
    HomingHandler handler;
    handler.setMethod(HomingMethod::CurrentPosition);
    bool ok = handler.start();
    (void)ok;
}

TEST(HomingHandlerTest, StartWithCallback) {
    HomingHandler handler;
    handler.setMethod(HomingMethod::CurrentPosition);
    handler.setSwitchCallback([]() -> HomingSwitchStatus {
        return HomingSwitchStatus{};
    });
    bool ok = handler.start();
    (void)ok;
}

TEST(HomingHandlerTest, Abort) {
    HomingHandler handler;
    handler.setMethod(HomingMethod::CurrentPosition);
    handler.setSwitchCallback([]() -> HomingSwitchStatus {
        return HomingSwitchStatus{};
    });
    handler.start();
    handler.abort();
}

TEST(HomingHandlerTest, UpdateCurrentPosition) {
    HomingHandler handler;
    handler.setMethod(HomingMethod::CurrentPosition);
    handler.setSwitchCallback([]() -> HomingSwitchStatus {
        return HomingSwitchStatus{};
    });
    handler.start();
    for (int i = 0; i < 10; ++i) {
        handler.update(i * 100);
    }
    (void)handler.isComplete();
    (void)handler.getHomePosition();
}

TEST(HomingHandlerTest, UpdateNegLimit) {
    HomingHandler handler;
    handler.setMethod(HomingMethod::NegLimitOnly);
    int callCount = 0;
    handler.setSwitchCallback([&]() -> HomingSwitchStatus {
        HomingSwitchStatus s{};
        if (callCount > 5) s.negativeLimitActive = true;
        callCount++;
        return s;
    });
    handler.start();
    for (int i = 0; i < 20; ++i) {
        handler.update(i * 100);
    }
    (void)handler.isComplete();
}

// ============================================================================
// Static methods
// ============================================================================

TEST(HomingHandlerTest, MethodRequiresLimit) {
    EXPECT_TRUE(HomingHandler::methodRequiresLimit(HomingMethod::NegLimitOnly));
    EXPECT_TRUE(HomingHandler::methodRequiresLimit(HomingMethod::PosLimitOnly));
    EXPECT_FALSE(HomingHandler::methodRequiresLimit(HomingMethod::CurrentPosition));
}

TEST(HomingHandlerTest, MethodRequiresHome) {
    (void)HomingHandler::methodRequiresHome(HomingMethod::HomeSwitchPosIndex);
    EXPECT_FALSE(HomingHandler::methodRequiresHome(HomingMethod::CurrentPosition));
}

TEST(HomingHandlerTest, MethodRequiresIndex) {
    EXPECT_TRUE(HomingHandler::methodRequiresIndex(HomingMethod::NegLimitIndex));
    EXPECT_FALSE(HomingHandler::methodRequiresIndex(HomingMethod::CurrentPosition));
}

TEST(HomingHandlerTest, GetInitialDirection) {
    int dir1 = HomingHandler::getInitialDirection(HomingMethod::NegLimitIndex);
    (void)dir1;
    int dir2 = HomingHandler::getInitialDirection(HomingMethod::PosLimitIndex);
    (void)dir2;
}

TEST(HomingHandlerTest, GetMethodDescription) {
    const char* desc = HomingHandler::getMethodDescription(HomingMethod::CurrentPosition);
    EXPECT_NE(desc, nullptr);
    const char* desc2 = HomingHandler::getMethodDescription(HomingMethod::NegLimitIndex);
    EXPECT_NE(desc2, nullptr);
}

// ============================================================================
// Multiple homing operations
// ============================================================================

TEST(HomingHandlerTest, MultipleOperations) {
    HomingHandler handler;
    handler.setSwitchCallback([]() -> HomingSwitchStatus {
        return HomingSwitchStatus{};
    });

    // First operation
    handler.setMethod(HomingMethod::CurrentPosition);
    handler.start();
    for (int i = 0; i < 10; ++i) handler.update(i * 100);

    // Abort and start new
    handler.abort();
    handler.setMethod(HomingMethod::CurrentPosition);
    handler.start();
    for (int i = 0; i < 10; ++i) handler.update(1000 + i * 100);
}

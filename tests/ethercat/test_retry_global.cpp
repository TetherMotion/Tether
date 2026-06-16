/**
 * @file test_retry_global.cpp
 * @brief Tests for RetryExecutor ownership and lifecycle (instance-based).
 *
 * Verifies that locally-owned RetryExecutor instances can be created,
 * used, and destroyed cleanly.
 */

#include <gtest/gtest.h>

#include "tether/ethercat/Retry.hpp"
#include "tether/ethercat/ConditionalPacketRouter.hpp"

using EtherCAT::ConditionalPacketRouter;
using EtherCAT::Raw::RetryExecutor;
using EtherCAT::Raw::StoredDatagram;

namespace {

class RetryOwnershipTest : public ::testing::Test {
protected:
    ConditionalPacketRouter router_;
    EtherCAT::Raw::SendFunction send_ = [](const StoredDatagram&) -> bool { return true; };
};

TEST_F(RetryOwnershipTest, CreateAndDestroy) {
    auto exec = std::make_unique<RetryExecutor>(router_, send_);
    EXPECT_NE(exec, nullptr);
    exec.reset();
}

TEST_F(RetryOwnershipTest, MultipleInstances) {
    RetryExecutor exec1(router_, send_);
    RetryExecutor exec2(router_, send_);
    // Both coexist without issue
    (void)exec1;
    (void)exec2;
}

TEST_F(RetryOwnershipTest, ReCreateAfterDestroy) {
    {
        RetryExecutor exec(router_, send_);
        (void)exec;
    }
    // Create another after the first has been destroyed
    RetryExecutor exec2(router_, send_);
    (void)exec2;
}

} // anonymous namespace

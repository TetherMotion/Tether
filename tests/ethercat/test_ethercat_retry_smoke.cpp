// Keep this file extremely small: it forces compilation of EtherCATRetry helpers
// under the tether_tests target even if future globbing changes.

#include <gtest/gtest.h>

#include "tether/ethercat/Retry.hpp"

TEST(EtherCATRetry, RetryPolicyTimeoutComputation) {
    auto pol = EtherCAT::Raw::RetryPolicy::standard();
    pol.initial_timeout_ms = 10;
    pol.max_timeout_ms = 25;
    pol.backoff_multiplier = 2.0f;
    pol.use_exponential_backoff = true;
    EXPECT_EQ(pol.getTimeoutForAttempt(0), 10u);
    EXPECT_EQ(pol.getTimeoutForAttempt(1), 20u);
    EXPECT_EQ(pol.getTimeoutForAttempt(2), 25u); // capped
}

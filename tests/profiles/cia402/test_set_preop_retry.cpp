#include <gtest/gtest.h>
#include "tether/ethercat/EtherCATMaster.hpp"
#include "tether/ethercat/EtherCATFaultDetection.hpp"
#include "ethercat/raw/internal.hpp"

using namespace EtherCAT::Raw;
using namespace EtherCAT;

// NOTE: SucceedsAfterRetries and FailsWhenNeverPreop live in
//       ethercat/test_set_preop_retry.cpp — avoid ODR duplicates.

TEST(EtherCATMasterPreopRetry, LogsDetailedALStatusOnError) {
    // Simulate AL_STATUS with error bit set and AL_STATUS_CODE = InvalidMailboxConfig
    int read_count = 0;

    EtherCAT::EtherCATMaster::Config cfg;
    cfg.rx_queue_depth = 4;
    cfg.txpdo_queue_depth = 4;
    EtherCAT::EtherCATMaster master(cfg);

    master.setApwrTestCallback([&](uint16_t adp, uint16_t ado, const void* data, uint16_t len, unsigned ms){
        return true;
    });
    master.setAprdTestCallback([&](uint16_t adp, uint16_t ado, void* out, uint16_t len, unsigned ms){
        if (ado == Raw::EC_REG_AL_STATUS) {
            // Return INIT with error bit set so PRE_OP transition will be retried and fail
            uint16_t val = static_cast<uint16_t>(0x0001u | 0x0010u); // INIT + ERROR
            if (out && len >= 2) std::memcpy(out, &val, 2);
            read_count++;
            return true;
        }
        if (ado == Raw::EC_REG_AL_STATUS_CODE) {
            uint16_t code = static_cast<uint16_t>(EtherCAT::ALStatusCode::InvalidMailboxConfig);
            if (out && len >= 2) std::memcpy(out, &code, 2);
            return true;
        }
        return false;
    });

    // Should fail and have read AL status and code
    EXPECT_FALSE(master.transitionSlaveToPreOperational(0));
    EXPECT_GT(read_count, 0);

    master.setAprdTestCallback(nullptr);
    master.setApwrTestCallback(nullptr);
}


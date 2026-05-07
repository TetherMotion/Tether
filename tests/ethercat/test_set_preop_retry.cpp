#include <gtest/gtest.h>
#include "tether/ethercat/EtherCATMaster.hpp"
#include "ethercat/raw/internal.hpp"

using namespace EtherCAT::Raw;
using namespace EtherCAT;

TEST(EtherCATMasterPreopRetry, SucceedsAfterRetries) {
    // Simulated behavior: AL_STATUS reads return non-PRE_OP for first 2 attempts
    int call_count = 0;

    EtherCAT::EtherCATMaster::Config cfg;
    cfg.rx_queue_depth = 4;
    cfg.txpdo_queue_depth = 4;
    EtherCAT::EtherCATMaster master(cfg);

    master.setApwrTestCallback([&](uint16_t adp, uint16_t ado, const void* data, uint16_t len, unsigned ms){
        // Accept writes
        return true;
    });

    master.setAprdTestCallback([&](uint16_t adp, uint16_t ado, void* out, uint16_t len, unsigned ms){
        // AL_STATUS register
        if (ado == Raw::EC_REG_AL_STATUS) {
            uint16_t val = 0;
            // First two attempts return INIT, later return PRE_OP
            if (call_count < 2) val = 0x0001; else val = 0x0002;
            if (out && len >= 2) std::memcpy(out, &val, 2);
            // increment on each read so after a few loops it will succeed
            ++call_count;
            return true;
        }
        // Default
        return false;
    });

    // Expect setPreop to succeed eventually
    EXPECT_TRUE(master.transitionSlaveToPreOperational(0));

    master.setAprdTestCallback(nullptr);
    master.setApwrTestCallback(nullptr);
}

TEST(EtherCATMasterPreopRetry, FailsWhenNeverPreop) {
    EtherCAT::EtherCATMaster::Config cfg;
    cfg.rx_queue_depth = 4;
    cfg.txpdo_queue_depth = 4;
    EtherCAT::EtherCATMaster master(cfg);

    // Always return INIT status
    master.setApwrTestCallback([&](uint16_t adp, uint16_t ado, const void* data, uint16_t len, unsigned ms){
        return true;
    });
    master.setAprdTestCallback([&](uint16_t adp, uint16_t ado, void* out, uint16_t len, unsigned ms){
        if (ado == Raw::EC_REG_AL_STATUS) {
            uint16_t val = 0x0001; // INIT always
            if (out && len >= 2) std::memcpy(out, &val, 2);
            return true;
        }
        return false;
    });

    EXPECT_FALSE(master.transitionSlaveToPreOperational(0));

    master.setAprdTestCallback(nullptr);
    master.setApwrTestCallback(nullptr);
}

#include <gtest/gtest.h>
#include "tether/ethercat/Master.hpp"
#include "tether/ethercat/FaultDetection.hpp"
#include "ethercat/raw/internal.hpp"

using namespace EtherCAT::Raw;
using namespace EtherCAT;

TEST(EtherCATMasterPreopRetry, FaultDiagnosedOnceOnError) {
    // First AL_STATUS read indicates an error condition, later returns PRE_OP
    int call_count = 0;

    EtherCAT::Master::Config cfg;
    cfg.rx_queue_depth = 4;
    cfg.txpdo_queue_depth = 4;
    EtherCAT::Master master(cfg);

    master.setApwrTestCallback([&](uint16_t adp, uint16_t ado, const void* data, uint16_t len, unsigned ms){
        return true;
    });

    master.setAprdTestCallback([&](uint16_t adp, uint16_t ado, void* out, uint16_t len, unsigned ms){
        if (ado == Raw::EC_REG_AL_STATUS) {
            uint16_t val = 0;
            if (call_count == 0) {
                // Return INIT with ERROR bit set
                val = 0x0011;
            } else {
                // Then return PRE_OP
                val = 0x0002;
            }
            if (out && len >= 2) std::memcpy(out, &val, 2);
            ++call_count;
            return true;
        }
        return false;
    });

    EXPECT_TRUE(master.transitionSlaveToPreOperational(0));
    EXPECT_TRUE(master.wasFaultDiagnosed(0));

    master.setAprdTestCallback(nullptr);
    master.setApwrTestCallback(nullptr);
}

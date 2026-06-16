#include <gtest/gtest.h>
#include "tether/ethercat/Master.hpp"
#include "tether/ethercat/FaultDetection.hpp"
#include "ethercat/raw/internal.hpp"

using namespace EtherCAT::Raw;
using namespace EtherCAT;

TEST(EtherCATMasterPreopRetry, FaultDiagnosedAlwaysError) {
    // First AL_STATUS read indicates an error condition, later returns PRE_OP
    int call_count = 0;

    EtherCAT::Master::Config cfg;
    cfg.rx_queue_depth = 4;
    cfg.txpdo_queue_depth = 4;
    EtherCAT::Master master(cfg);

    master.setApwrTestCallback([&](uint16_t adp, uint16_t ado, const void* data, uint16_t len, unsigned ms){
        (void)adp; (void)ado; (void)data; (void)len; (void)ms; return true;
    });

    master.setAprdTestCallback([&](uint16_t adp, uint16_t ado, void* out, uint16_t len, unsigned ms){
        (void)adp;
        if (ado == Raw::EC_REG_AL_STATUS) {
            // Always report INIT + ERROR so the PRE_OP transition will fail
            uint16_t val = static_cast<uint16_t>(0x0001u | 0x0010u);
            if (out && len >= 2) std::memcpy(out, &val, 2);
            ++call_count;
            return true;
        }
        if (ado == Raw::EC_REG_AL_STATUS_CODE) {
            uint16_t code = static_cast<uint16_t>(EtherCAT::ALStatusCode::InvalidMailboxConfig);
            if (out && len >= 2) std::memcpy(out, &code, 2);
            return true;
        }
        return false;
    });

    // Sanity-check the apwr short-circuit
    EXPECT_TRUE(master.writeRegister(EtherCAT::Master::adpForSlaveIndex(0), Raw::EC_REG_AL_CONTROL, static_cast<uint16_t>(0x0002)));

    EXPECT_FALSE(master.transitionSlaveToPreOperational(0));
    EXPECT_TRUE(master.wasFaultDiagnosed(0));

    master.setAprdTestCallback(nullptr);
    master.setApwrTestCallback(nullptr);
}

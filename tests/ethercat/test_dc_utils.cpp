#include "gtest/gtest.h"

#include "tether/ethercat/DC/Utils.hpp"

using namespace EtherCAT;

TEST(DCUtilsTest, SyncActivationMaskToString) {
    EXPECT_EQ(DC::Utils::syncActivationMaskToString(0x0000), "SYNC0_DIS SYNC1_DIS");
    EXPECT_EQ(DC::Utils::syncActivationMaskToString(0x0001), "SYNC0_EN SYNC1_DIS");
    EXPECT_EQ(DC::Utils::syncActivationMaskToString(0x0002), "SYNC0_DIS SYNC1_EN");
    EXPECT_EQ(DC::Utils::syncActivationMaskToString(0x0003), "SYNC0_EN SYNC1_EN");
}

TEST(DCUtilsTest, PrintDCDiagnosticsNoDc) {
    // master constructed without DC configured
    EtherCAT::EtherCATMaster master;
    // should simply log warnings and not crash
    DC::Utils::printDCDiagnostics(master, 0, "TESTDC");
    SUCCEED();
}

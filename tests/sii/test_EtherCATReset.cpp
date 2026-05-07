#include <gtest/gtest.h>
#include "tether/ethercat/EtherCATReset.hpp"

using namespace EtherCAT;

TEST(EtherCATReset, NamesAndCodes) {
    EXPECT_NE(getResetLevelName(ResetLevel::SoftReset), nullptr);
    EXPECT_NE(getResetLevelDescription(ResetLevel::HardwareReset), nullptr);
    EXPECT_NE(getALStatusCodeName(ALStatusCode::NoError), nullptr);
}

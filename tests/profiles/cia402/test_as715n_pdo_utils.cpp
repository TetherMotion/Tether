#include <gtest/gtest.h>

#include "tether/drives/AS715N/AS715NPDO.hpp"
#include "tether/ethercat/Master.hpp"
#include "tether/profiles/cia402/CiA402Drive.hpp"

using namespace EtherCAT;
using namespace EtherCAT::Drives;

TEST(AS715NPDOUtils, DumpUsingDescriptorsWithAssignment)
{
    Master master;
    CiA402Drive drive(master, 0);

    // assign known PDO indices/sizes so the helper can select descriptors
    drive.assignFixedPDOs(AS715N_pdo::RxPDO_1705.index,
                           AS715N_pdo::TxPDO_1B04.index,
                           AS715N_pdo::RxPDO_1705.size,
                           AS715N_pdo::TxPDO_1B04.size);

    std::string txdump = AS715N_pdo::dumpUsingDescriptors(drive, true);
    EXPECT_NE(txdump.find("0x1B04"), std::string::npos);

    std::string rxdump = AS715N_pdo::dumpUsingDescriptors(drive, false);
    EXPECT_NE(rxdump.find("0x1705"), std::string::npos);
}

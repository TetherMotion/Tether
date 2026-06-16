#include <gtest/gtest.h>
#include "profiles/cia402/CiA402Drive.hpp"
#include "tether/ethercat/Master.hpp"
#include "tether/drives/AS715N/AS715NPDO.hpp"

using namespace EtherCAT;

TEST(CiA402AssignFixedPDOs, AssignStoresSizes) {
    EtherCAT::Master::Config cfg;
    cfg.rx_queue_depth = 4;
    cfg.txpdo_queue_depth = 4;
    EtherCAT::Master master(cfg);

    CiA402Drive drive(master, 0);

    // assignFixedPDOs always returns true (sizes stored regardless of SDO outcome)
    using namespace EtherCAT::Drives::AS715N_pdo;
    EXPECT_TRUE(drive.assignFixedPDOs(RxPDO_1705.index, TxPDO_1B04.index,
                                       RxPDO_1705.size, TxPDO_1B04.size));

    EXPECT_EQ(drive.getRxPDOSize(), RxPDO_1705.size);
    EXPECT_EQ(drive.getTxPDOSize(), TxPDO_1B04.size);
    EXPECT_EQ(drive.getRxPDOIndex(), RxPDO_1705.index);
    EXPECT_EQ(drive.getTxPDOIndex(), TxPDO_1B04.index);
}

TEST(CiA402AssignFixedPDOs, TypedAccessWorks) {
    EtherCAT::Master::Config cfg;
    cfg.rx_queue_depth = 4;
    cfg.txpdo_queue_depth = 4;
    EtherCAT::Master master(cfg);

    CiA402Drive drive(master, 0);

    using namespace EtherCAT::Drives::AS715N_pdo;
    drive.assignFixedPDOs(RxPDO_1705.index, TxPDO_1B04.index,
                          RxPDO_1705.size, TxPDO_1B04.size);

    // Write via typed pointer
    auto* rx = drive.rxPDO<AS715N_RxPDO_1705>();
    rx->controlword = 0x000F;
    rx->target_position = 50000;
    rx->modes_of_operation = 8;
    rx->positive_torque_limit = 1000;
    rx->negative_torque_limit = 1000;

    // Verify bytes in underlying buffer
    auto* raw = static_cast<uint8_t*>(drive.getRxPDOBuffer());
    uint16_t cw;
    std::memcpy(&cw, raw, 2);
    EXPECT_EQ(cw, 0x000Fu);

    int32_t pos;
    std::memcpy(&pos, raw + 2, 4);
    EXPECT_EQ(pos, 50000);
}

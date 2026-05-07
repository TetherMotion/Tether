#include <gtest/gtest.h>

#include "tether/ethercat/EtherCATMaster.hpp"
#include "tether/ethercat/EtherCATPDO.hpp"
#include "tether/ethercat/EtherCATSDO.hpp"
#include "tether/ethercat/EtherCATDC.hpp"
#include "tether/ethercat/EtherCATFoE.hpp"
#include "tether/ethercat/EtherCATVoE.hpp"
#include "tether/ethercat/EtherCATEoE.hpp"
#include "tether/ethercat/EtherCATFaultDetection.hpp"

TEST(EtherCATMasterSubManagers, CreatedOnConstruction)
{
    EtherCAT::EtherCATMaster master;

    auto& pdo = master.pdo();
    auto& sdo = master.sdoManager();
    auto& dc  = master.dc();
    auto& foe = master.foe();
    auto& voe = master.voe();
    auto& eoe = master.eoe();
    auto& faults = master.faults();

    // Each manager should reference back to the same EtherCATMaster instance
    EXPECT_EQ(&pdo.master(), &master);
    (void)sdo; // SDOManager doesn't store master ref directly
    EXPECT_EQ(&dc.master(), &master);
    EXPECT_EQ(&foe.master(), &master);
    EXPECT_EQ(&voe.master(), &master);
    EXPECT_EQ(&eoe.master(), &master);
    EXPECT_EQ(&faults.master(), &master);

    // Lightweight smoke checks (should not throw)
    (void)pdo.getMapping();
    (void)dc.getState();
}

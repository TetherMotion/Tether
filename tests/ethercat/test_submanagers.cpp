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
    EtherCAT::Master master;

    auto& pdo = master.pdo();
    auto& sdo = master.sdoManager();
    auto& dc  = master.dc();
    auto& foe = master.foe();
    auto& voe = master.voe();
    auto& eoe = master.eoe();
    auto& faults = master.faults();

    // Lightweight smoke checks (should not throw)
    (void)pdo.mapping();
    (void)sdo; // SDOManager
    (void)dc;
    (void)foe;
    (void)voe;
    (void)eoe;
    (void)faults;
}

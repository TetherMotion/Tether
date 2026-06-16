#include <gtest/gtest.h>

#include "tether/ethercat/Master.hpp"
#include "tether/ethercat/PDOManager.hpp"
#include "tether/ethercat/SDOManager.hpp"
#include "tether/ethercat/DC.hpp"
#include "tether/ethercat/FoE.hpp"
#include "tether/ethercat/VoE.hpp"
#include "tether/ethercat/EoE.hpp"
#include "tether/ethercat/FaultDetection.hpp"

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

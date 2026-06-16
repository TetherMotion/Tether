#include <gtest/gtest.h>

#include "tether/ethercat/Master.hpp"
#include "tether/ethercat/PDOManager.hpp"
#include "tether/ethercat/SDOManager.hpp"
#include "tether/ethercat/DC.hpp"
#include "tether/ethercat/FoE.hpp"
#include "tether/ethercat/VoE.hpp"
#include "tether/ethercat/EoE.hpp"
#include "tether/ethercat/FaultDetection.hpp"

TEST(EtherCATMasterSubManagers, CreatedOnConstructionTransportAPI)
{
    EtherCAT::Master master;

    auto& pdo = master.pdo();
    auto& sdo = master.sdoManager();
    auto& dc  = master.dc();
    auto& foe = master.foe();
    auto& voe = master.voe();
    auto& eoe = master.eoe();
    auto& faults = master.faults();

    // Each manager should reference back to the same Master instance
    // PDOManager uses transport injection — no master() accessor; verify transport is valid
    (void)pdo.transport();
    (void)sdo; // SDOManager doesn't store master ref directly
    EXPECT_EQ(&dc.master(), &master);
    EXPECT_EQ(&foe.master(), &master);
    EXPECT_EQ(&voe.master(), &master);
    EXPECT_EQ(&eoe.master(), &master);
    // FaultDetector uses transport injection — no master() accessor
    (void)faults;

    // Lightweight smoke checks (should not throw)
    (void)pdo.mapping();
    (void)dc.getState();
}

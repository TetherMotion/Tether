#include "gtest/gtest.h"

#include "tether/ethercat/SyncManager/Utils.hpp"
#include "tether/profiles/cia402/1Cxx-SyncManagerParameters.hpp"

using namespace EtherCAT;
using namespace CiA301::Parameters1Cxx;
// Do not pull in Utils namespace to avoid ambiguity with string helpers; qualify explicitly

TEST(SyncManagerUtilsTest, SyncModeToString) {
    EXPECT_STREQ(EtherCAT::SyncManager::Utils::syncModeToString(0), "FreeRun");
    EXPECT_STREQ(EtherCAT::SyncManager::Utils::syncModeToString(1), "SM-Synchronous");
    EXPECT_STREQ(EtherCAT::SyncManager::Utils::syncModeToString(2), "DC-SYNC0");
    EXPECT_STREQ(EtherCAT::SyncManager::Utils::syncModeToString(3), "DC-SYNC1");
    EXPECT_STREQ(EtherCAT::SyncManager::Utils::syncModeToString(0xFF), "(unknown)");
}

TEST(SyncManagerUtilsTest, SupportedSyncTypesToString) {
    // each single flag
    EXPECT_EQ(EtherCAT::SyncManager::Utils::supportedSyncTypesToString(static_cast<uint16_t>(SupportedSyncTypes::FreeRun)), "FreeRun");
    EXPECT_EQ(EtherCAT::SyncManager::Utils::supportedSyncTypesToString(static_cast<uint16_t>(SupportedSyncTypes::SMSync)), "SM-Synchronous");
    EXPECT_EQ(EtherCAT::SyncManager::Utils::supportedSyncTypesToString(static_cast<uint16_t>(SupportedSyncTypes::DcSync0)), "DC-SYNC0");
    EXPECT_EQ(EtherCAT::SyncManager::Utils::supportedSyncTypesToString(static_cast<uint16_t>(SupportedSyncTypes::DcSync1)), "DC-SYNC1");
    EXPECT_EQ(EtherCAT::SyncManager::Utils::supportedSyncTypesToString(static_cast<uint16_t>(SupportedSyncTypes::SubAppCycle)), "SubAppCycle");
    // combination
    uint16_t combo = static_cast<uint16_t>(SupportedSyncTypes::FreeRun) |
                     static_cast<uint16_t>(SupportedSyncTypes::DcSync0);
    std::string combo_str = EtherCAT::SyncManager::Utils::supportedSyncTypesToString(combo);
    // order is deterministic: FreeRun first then DC-SYNC0
    EXPECT_EQ(combo_str, "FreeRun, DC-SYNC0");

    // unknown bits should be shown as hex suffix
    uint16_t mask = 0x8000;
    std::string masked = EtherCAT::SyncManager::Utils::supportedSyncTypesToString(mask);
    EXPECT_NE(masked.find("0x8000"), std::string::npos);
}

// We can't easily exercise the SDO read helpers without a real or fake SDOManager,
// but we can at least ensure the convenient printSyncDiagnostics function compiles
// and doesn't crash when used with a default-constructed master (which has no
// slaves). It should simply log warnings and return.
TEST(SyncManagerUtilsTest, PrintSyncDiagnosticsNoSlave) {
    EtherCAT::Master master;
    // no slaves discovered; this call should not crash
    EtherCAT::SyncManager::Utils::printSyncDiagnostics(master, 0, "TEST");
    SUCCEED();
}

// entry-based helpers should compile and behave reasonably with an empty master
TEST(SyncManagerUtilsTest, EntryBasedReadsNoSlave) {
    EtherCAT::Master master;
    auto& sdo = master.sdoManager();

    // pick a known sync manager object dictionary entry
    const auto& entry = CiA301::Parameters1Cxx::SM2SyncMode;

    // the underlying SDO read should fail since no slave is present
    uint16_t mode = 0;
    EXPECT_FALSE(EtherCAT::SyncManager::Utils::readSync(sdo, 0, entry, mode));

    // the non-template variant should also return false
    uint16_t buf[1];
    EXPECT_FALSE(EtherCAT::SyncManager::Utils::readSyncEntry(sdo, 0, entry, buf, sizeof(buf)));

    // printSyncEntry should not crash; it will log a warning
    EtherCAT::SyncManager::Utils::printSyncEntry(sdo, 0, entry, "TEST");
}

#include <gtest/gtest.h>

#include "tether/ethercat/Mailbox/Utils.hpp"
#include "tether/ethercat/EtherCATMaster.hpp"
#include "tether/profiles/cia402/CiA402Drive.hpp"

using namespace EtherCAT;
using namespace EtherCAT::Mailbox;

TEST(MailboxUtils, DumpHeaderAndStatusNoMailboxConfig)
{
    Master master;
    // master has no mailbox configured, should simply log a warning and not
    // crash.
    Mailbox::Utils::dumpHeaderAndStatus(master, 0, "TESTMBX");
    SUCCEED();
}

TEST(MailboxUtils, DumpSlaveSyncAndMailboxInfoHappyPath)
{
    Master master;
    CiA402Drive drive(master, 0);
    // even though master has no slaves configured, the function should
    // gracefully log and return.
    Mailbox::Utils::dumpSlaveSyncAndMailboxInfo(drive, "TESTMBX");
    SUCCEED();
}

TEST(MailboxUtils, StatuswordDiagnostics)
{
    bool warn_active = false;
    uint64_t first_cycle = 0;

    // bit7 rising edge should set warn_active and record cycle
    Mailbox::Utils::logStatuswordDiagnostics(0x0080, warn_active, first_cycle, 10, "TEST");
    EXPECT_TRUE(warn_active);
    EXPECT_EQ(first_cycle, 10);

    // still active should not clear until bit goes low
    Mailbox::Utils::logStatuswordDiagnostics(0x0080, warn_active, first_cycle, 11, "TEST");
    EXPECT_TRUE(warn_active);

    // clearing warning
    Mailbox::Utils::logStatuswordDiagnostics(0x0000, warn_active, first_cycle, 20, "TEST");
    EXPECT_FALSE(warn_active);

    // error logging paths exercise without checking output
    Mailbox::Utils::logStatuswordDiagnostics(0x2000, warn_active, first_cycle, 500, "TEST");
    Mailbox::Utils::logStatuswordDiagnostics(0x0800, warn_active, first_cycle, 1000, "TEST");

    SUCCEED();
}

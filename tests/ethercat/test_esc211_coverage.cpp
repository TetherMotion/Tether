/**
 * @file test_esc211_coverage.cpp
 * @brief NexcobotESC211 error-table parsing (pure) plus the
 *        SystemErrorManager SDO poll loop over a real
 *        Master ↔ SlaveEmulator wire (EmulatedBus).
 *
 * check() reads 0xF101 (system state) + 0xF102 (error code) via
 * pollSdoObjects, and 0xF103 (VisibleString message) on error edges —
 * all answerable by the emulator once the OD objects are injected.
 */

#include <gtest/gtest.h>

#include "mocks/EmulatedBus.hpp"
#include "tether/drives/NexcobotESC211/SystemErrorManager.hpp"
#include "tether/drives/NexcobotESC211Errors.hpp"
#include "tether/ethercat/CoEManager.hpp"
#include "tether/ethercat/Slave.hpp"

using namespace EtherCAT;
using EtherCAT::Emulator::SlaveEmulator;
namespace ECerr = EtherCAT::Drives::ErrorCodes::NexcobotESC211;
using EtherCAT::Drives::ESC211::SystemErrorManager;

// ============================================================================
// NexcobotESC211Errors — pure error-table parsing
// ============================================================================

TEST(NexcobotESC211ErrorsTest, ParseZeroIsNoError) {
    auto e = ECerr::NexcobotESC211Error::parse(0);
    EXPECT_EQ(e.raw_code, 0);
    EXPECT_STREQ(e.name, "NoError");
    EXPECT_STREQ(e.error_handling, "No action required.");
}

TEST(NexcobotESC211ErrorsTest, RsapCodeParsesWithCategory) {
    auto e = ECerr::NexcobotESC211Error::parse(-1);
    EXPECT_EQ(e.category, ECerr::ErrorCategory::RSAP);
    EXPECT_NE(e.name, nullptr);
    EXPECT_TRUE(e.is_recoverable);
}

TEST(NexcobotESC211ErrorsTest, SafetyDiRangeMapsToDiEntry) {
    auto e = ECerr::NexcobotESC211Error::parse(-305);
    EXPECT_EQ(e.category, ECerr::ErrorCategory::SAFETY);
    EXPECT_NE(ECerr::NexcobotESC211Error::findEntry(-300), nullptr);
    EXPECT_NE(ECerr::NexcobotESC211Error::findEntry(-310), nullptr);
}

TEST(NexcobotESC211ErrorsTest, SafetyDoRangeMapsToDoEntry) {
    auto e = ECerr::NexcobotESC211Error::parse(-325);
    EXPECT_EQ(e.category, ECerr::ErrorCategory::SAFETY);
    EXPECT_NE(ECerr::NexcobotESC211Error::findEntry(-320), nullptr);
    EXPECT_NE(ECerr::NexcobotESC211Error::findEntry(-330), nullptr);
}

TEST(NexcobotESC211ErrorsTest, UnknownCodeFallsBack) {
    auto e = ECerr::NexcobotESC211Error::parse(-9999);
    EXPECT_STREQ(e.name, "UnknownError");
    EXPECT_EQ(e.category, ECerr::ErrorCategory::None);
    EXPECT_FALSE(e.is_recoverable);
    EXPECT_EQ(ECerr::NexcobotESC211Error::findEntry(-9999), nullptr);
}

TEST(NexcobotESC211ErrorsTest, CategoryToStringCoversAll) {
    EXPECT_STREQ(ECerr::ErrorCategoryToString(ECerr::ErrorCategory::None), "None");
    EXPECT_STREQ(ECerr::ErrorCategoryToString(ECerr::ErrorCategory::RSAP), "RSAP");
    EXPECT_STREQ(ECerr::ErrorCategoryToString(ECerr::ErrorCategory::CONFIG), "CONFIG");
    EXPECT_STREQ(ECerr::ErrorCategoryToString(ECerr::ErrorCategory::SAFETY), "SAFETY");
    EXPECT_STREQ(ECerr::ErrorCategoryToString(ECerr::ErrorCategory::SYSMGR), "SYSMGR");
    EXPECT_STREQ(ECerr::ErrorCategoryToString(ECerr::ErrorCategory::FSoE), "FSoE");
    EXPECT_STREQ(ECerr::ErrorCategoryToString(static_cast<ECerr::ErrorCategory>(99)),
                 "Unknown");
}

// ============================================================================
// SystemErrorManager — SDO poll loop over the emulated wire
// ============================================================================

namespace {

class ESC211EmulatorTest : public ::testing::Test {
protected:
    void SetUp() override {
        auto slave = Emulator::createCiA402Drive(0x00000002, 0x0442, "esc211");
        slave_ = slave.get();
        bus_.net.addSlave(std::move(slave));
        bus_.attach(master_);
        master_.start(bus_.iface, kEmulatedMac);
        auto found = master_.discovery().discover(DiscoveryOptions{});
        ASSERT_EQ(found.size(), 1u);
        master_.sdoManager(0).configureMailbox(0x1000, 128, 0x1080, 128);
    }
    void TearDown() override {
        bus_.shutdown();
        master_.packetRouter().shutdown();
    }

    /// u32 LE helper for OD data vectors.
    static std::vector<uint8_t> u32le(uint32_t v) {
        return {static_cast<uint8_t>(v),
                static_cast<uint8_t>(v >> 8),
                static_cast<uint8_t>(v >> 16),
                static_cast<uint8_t>(v >> 24)};
    }

    void addU32(uint16_t index, uint32_t value, uint8_t access = 0x01) {
        slave_->addODEntry({index, 0, 0x0007, 32, access, u32le(value), "x"});
    }

    EmulatedBus bus_;
    SlaveEmulator* slave_ = nullptr;
    Master master_;
};

} // namespace

TEST_F(ESC211EmulatorTest, CheckNoErrorWhenCodeIsZero) {
    addU32(0xF101, 4);   // system state
    addU32(0xF102, 0);   // no error

    SystemErrorManager mgr(master_.slave(0), std::chrono::milliseconds(50));
    EXPECT_EQ(mgr.check(), SystemErrorManager::Result::NoError);
    EXPECT_FALSE(mgr.hasError());
    EXPECT_EQ(mgr.lastErrorCode(), 0);
    EXPECT_EQ(mgr.lastSystemState(), 4u);
    EXPECT_TRUE(mgr.lastErrorMessage().empty());
    EXPECT_FALSE(mgr.isRunning());
    EXPECT_FALSE(mgr.consumeRestartRequest());
}

TEST_F(ESC211EmulatorTest, CheckAppErrorReadsMessageString) {
    addU32(0xF101, 4);
    addU32(0xF102, static_cast<uint32_t>(-305));  // SAFETY DI code
    const char msg[] = "axis undervoltage";
    slave_->addODEntry({0xF103, 0, 0x0009,
                        static_cast<uint16_t>(sizeof(msg) * 8),
                        0x01,
                        std::vector<uint8_t>(msg, msg + sizeof(msg) - 1),
                        "msg"});

    SystemErrorManager mgr(master_.slave(0), std::chrono::milliseconds(50));
    EXPECT_EQ(mgr.check(), SystemErrorManager::Result::AppError);
    EXPECT_TRUE(mgr.hasError());
    EXPECT_EQ(mgr.lastErrorCode(), -305);
    // 0xF103 VisibleString arrives via segmented SDO upload (7 B/segment).
    EXPECT_EQ(mgr.lastErrorMessage(), "axis undervoltage");
}

TEST_F(ESC211EmulatorTest, MissingErrorObjectKeepsPreviousState) {
    addU32(0xF101, 4);
    // No 0xF102 — the poll fails, check() reports the stale (zero) code.
    SystemErrorManager mgr(master_.slave(0), std::chrono::milliseconds(50));
    EXPECT_EQ(mgr.check(), SystemErrorManager::Result::NoError);
    EXPECT_FALSE(mgr.hasError());
}

TEST_F(ESC211EmulatorTest, RestartTriggerFiresOnEdgeThenStaysLatched) {
    addU32(0xF101, 3);  // matches the trigger below
    addU32(0xF102, 0);

    SystemErrorManager mgr(master_.slave(0), std::chrono::milliseconds(50));
    mgr.setRestartTriggerState(3);

    EXPECT_EQ(mgr.check(), SystemErrorManager::Result::NoError);
    EXPECT_TRUE(mgr.consumeRestartRequest());   // edge fired
    EXPECT_FALSE(mgr.consumeRestartRequest());  // latched, consumed
    // Same state polled again — no immediate refire (5 s re-arm window).
    EXPECT_EQ(mgr.check(), SystemErrorManager::Result::NoError);
    EXPECT_FALSE(mgr.consumeRestartRequest());
}

TEST_F(ESC211EmulatorTest, BackgroundPollThreadPicksUpNewError) {
    addU32(0xF101, 4);
    addU32(0xF102, 0);

    SystemErrorManager mgr(master_.slave(0), std::chrono::milliseconds(10));
    mgr.start();
    EXPECT_TRUE(mgr.isRunning());
    mgr.start();  // idempotent

    // Inject an error mid-run — the thread should observe it.
    addU32(0xF102, static_cast<uint32_t>(-1));
    for (int i = 0; i < 200 && !mgr.hasError(); ++i) {
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
    mgr.stop();
    mgr.stop();  // idempotent

    EXPECT_TRUE(mgr.hasError());
    EXPECT_EQ(mgr.lastErrorCode(), -1);
}

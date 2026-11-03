/**
 * @file test_esc211_managers.cpp
 * @brief NexcobotESC211 manager classes over a real
 *        Master ↔ SlaveEmulator wire (EmulatedBus).
 *
 * The managers are all `EtherCAT::Slave&`-driven SDO clients: command
 * protocol (0xF100), system-state reads (0xF101..0xF110), config data
 * sections (0xF2xx segmented download), and the RSAP poll tables
 * (0x40xx/0x41xx).  Injecting OD entries into the emulator exercises
 * the full poll/wait/sequence logic without hardware.
 */

#include <gtest/gtest.h>

#include <atomic>
#include <chrono>
#include <thread>

#include "mocks/EmulatedBus.hpp"
#include "tether/drives/NexcobotESC211/ConfigManager.hpp"
#include "tether/drives/NexcobotESC211/ControlCommandChannel.hpp"
#include "tether/drives/NexcobotESC211/CrcVerifier.hpp"
#include "tether/drives/NexcobotESC211/ErrorStateReader.hpp"
#include "tether/drives/NexcobotESC211/RSAPInformationProcessor.hpp"
#include "tether/drives/NexcobotESC211/RSAPMonitoringProcessor.hpp"
#include "tether/drives/NexcobotESC211/SafetyStateMachineManager.hpp"
#include "tether/drives/NexcobotESC211/StatusReader.hpp"
#include "tether/ethercat/CoEManager.hpp"
#include "tether/ethercat/Slave.hpp"

using namespace EtherCAT;
using EtherCAT::Emulator::SlaveEmulator;
namespace ESC211 = EtherCAT::Drives::ESC211;
using EtherCAT::Drives::ESC211::CrcExpectation;
using EtherCAT::Drives::Registers::NexcobotESC211::UserSystem::ControlCommandCode;

namespace {

class ESC211ManagersTest : public ::testing::Test {
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

    static std::vector<uint8_t> le(uint32_t v, size_t n) {
        std::vector<uint8_t> d(n);
        for (size_t i = 0; i < n; ++i) d[i] = static_cast<uint8_t>(v >> (8 * i));
        return d;
    }

    /// u8/u16/u32 object; access 0x03 (rw) or 0x01 (ro).
    void addU(uint16_t idx, uint8_t sub, uint32_t value,
            uint8_t bytes, uint8_t access = 0x03) {
        const uint16_t dt = (bytes == 1) ? 0x0005 : (bytes == 2) ? 0x0006 : 0x0007;
        slave_->addODEntry(
            {idx, sub, dt, static_cast<uint16_t>(bytes * 8), access,
             le(value, bytes), "x"});
    }
    void addStr(uint16_t idx, uint8_t sub, const std::string& s,
                uint8_t access = 0x01) {
        slave_->addODEntry(
            {idx, sub, 0x0009, static_cast<uint16_t>(s.size() * 8), access,
             std::vector<uint8_t>(s.begin(), s.end()), "s"});
    }
    void addSection(uint16_t idx, uint8_t sub) {
        slave_->addODEntry({idx, sub, 0x0000, 0, 0x03, {}, "sec"});
    }
    /// Standard 0xF100 command registers + 0xF101 state.
    void addCommandRegs() {
        addU(0xF100, 0x01, 0, 4);  // command
        addU(0xF100, 0x02, 0, 4);  // response
        addU(0xF100, 0x03, 0, 4);  // error code
        addU(0xF101, 0x00, 4, 4);  // system state
    }
    void setResponse(uint32_t v) { addU(0xF100, 0x02, v, 4); }
    void setState(uint32_t v)    { addU(0xF101, 0x00, v, 4); }

    EmulatedBus bus_;
    SlaveEmulator* slave_ = nullptr;
    Master master_;
};

} // namespace

// ============================================================================
// StatusReader — one-shot snapshot reads
// ============================================================================

TEST_F(ESC211ManagersTest, StatusReaderPopulatesSnapshot) {
    addU(0xF100, 0x02, 2, 4);            // last command response = Done
    addU(0xF101, 0x00, 6, 4);            // MONITORING
    addU(0xF102, 0x00, static_cast<uint32_t>(-305), 4);
    addStr(0xF103, 0x00, "di wiring fault");

    ESC211::StatusReader::SystemStateSnapshot snap;
    ESC211::StatusReader::readSystemState(master_.slave(0), snap);

    EXPECT_TRUE(snap.last_command_response_ok);
    EXPECT_EQ(snap.last_command_response, 2u);
    EXPECT_TRUE(snap.system_state_ok);
    EXPECT_EQ(snap.system_state, 6u);
    EXPECT_TRUE(snap.error_code_ok);
    EXPECT_EQ(snap.error_code, -305);
    EXPECT_TRUE(snap.error_message_ok);
    EXPECT_EQ(snap.error_message, "di wiring fault");
}

TEST_F(ESC211ManagersTest, StatusReaderReportsReadFailures) {
    // Nothing injected — every read aborts.
    ESC211::StatusReader::SystemStateSnapshot snap;
    ESC211::StatusReader::readSystemState(master_.slave(0), snap);
    EXPECT_FALSE(snap.last_command_response_ok);
    EXPECT_FALSE(snap.system_state_ok);
    EXPECT_FALSE(snap.error_code_ok);
    EXPECT_FALSE(snap.error_message_ok);
}

TEST_F(ESC211ManagersTest, StatusReaderReadsDebugMessageTable) {
    addU(0xF110, 0x00, 2, 1);  // entry count
    addStr(0xF110, 0x01, "boot ok");
    addStr(0xF110, 0x02, "watchdog armed");

    std::vector<std::string> lines;
    ESC211::StatusReader::readDebugMessages(master_.slave(0), lines);
    ASSERT_GE(lines.size(), 3u);  // count line + 2 entries
    EXPECT_NE(lines[1].find("boot ok"), std::string::npos);
    EXPECT_NE(lines[2].find("watchdog armed"), std::string::npos);
}

TEST_F(ESC211ManagersTest, StatusReaderDebugTableFailureIsReported) {
    std::vector<std::string> lines;
    ESC211::StatusReader::readDebugMessages(master_.slave(0), lines);
    ASSERT_FALSE(lines.empty());
    EXPECT_NE(lines[0].find("read failed"), std::string::npos);
}

// ============================================================================
// CrcVerifier — poll-until-match over injected CRC registers
// ============================================================================

TEST_F(ESC211ManagersTest, CrcVerifierReadsAndMatchesImmediately) {
    addU(0xF202, 0x00, 0xABCD1234, 4);
    addU(0xF212, 0x00, 0x00000055, 4);

    ESC211::CrcVerifier crc(master_.slave(0), std::chrono::milliseconds(5));
    EXPECT_EQ(crc.readCrc(0xF202), 0xABCD1234u);

    const CrcExpectation exps[] = {
        {0xF202, 0xABCD1234, "FNI"},
        {0xF212, 0x00000055, "RSP"},
    };
    EXPECT_TRUE(crc.waitUntilMatch(exps, std::chrono::milliseconds(500)));
}

TEST_F(ESC211ManagersTest, CrcVerifierMatchesAfterDeviceUpdate) {
    addU(0xF202, 0x00, 0x00000000, 4);  // not yet ready

    std::thread updater([this] {
        std::this_thread::sleep_for(std::chrono::milliseconds(60));
        addU(0xF202, 0x00, 0xCAFEBABE, 4);
    });

    ESC211::CrcVerifier crc(master_.slave(0), std::chrono::milliseconds(10));
    const CrcExpectation exps[] = {{0xF202, 0xCAFEBABE, "FNI"}};
    EXPECT_TRUE(crc.waitUntilMatch(exps, std::chrono::milliseconds(2000)));
    updater.join();
}

TEST_F(ESC211ManagersTest, CrcVerifierTimesOutOnMismatch) {
    addU(0xF202, 0x00, 0x11111111, 4);
    ESC211::CrcVerifier crc(master_.slave(0), std::chrono::milliseconds(5));
    const CrcExpectation exps[] = {{0xF202, 0x22222222, "FNI"}};
    EXPECT_FALSE(crc.waitUntilMatch(exps, std::chrono::milliseconds(150)));
}

TEST_F(ESC211ManagersTest, CrcVerifierAcceptsAlternateValue) {
    addU(0xF202, 0x00, 0xAAAA0000, 4);
    ESC211::CrcVerifier crc(master_.slave(0), std::chrono::milliseconds(5));
    const CrcExpectation exps[] = {{0xF202, 0x11111111, "FNI", 0xAAAA0000}};
    EXPECT_TRUE(crc.waitUntilMatch(exps, std::chrono::milliseconds(300)));
}

TEST_F(ESC211ManagersTest, CrcVerifierReadErrorReturnsSentinel) {
    ESC211::CrcVerifier crc(master_.slave(0), std::chrono::milliseconds(5));
    EXPECT_EQ(crc.readCrc(0xF299), 0xFFFFFFFFu);
}

// ============================================================================
// ControlCommandChannel — command/response protocol at 0xF100
// ============================================================================

TEST_F(ESC211ManagersTest, SendAndWaitSucceedsWhenResponseDone) {
    addCommandRegs();
    setResponse(2);  // Done already latched

    ESC211::ControlCommandChannel::Config cfg;
    cfg.command_timeout = std::chrono::milliseconds(500);
    cfg.poll_interval = std::chrono::milliseconds(5);
    cfg.pre_command_reset_delay = std::chrono::milliseconds(1);

    ESC211::ControlCommandChannel ch(master_.slave(0), cfg);
    EXPECT_EQ(ch.sendAndWait(ControlCommandCode::ClearLastErrorCode, {}, {}),
              ESC211::ControlCommandChannel::Result::Success);
}

TEST_F(ESC211ManagersTest, SendAndWaitFailsWhenDeviceReportsFailed) {
    addCommandRegs();
    setResponse(3);  // Failed

    ESC211::ControlCommandChannel::Config cfg;
    cfg.command_timeout = std::chrono::milliseconds(500);
    cfg.poll_interval = std::chrono::milliseconds(5);
    cfg.pre_command_reset_delay = std::chrono::milliseconds(1);

    ESC211::ControlCommandChannel ch(master_.slave(0), cfg);
    EXPECT_EQ(ch.sendAndWait(ControlCommandCode::StopSafetyAppAndFSoE, {}, {}),
              ESC211::ControlCommandChannel::Result::Failed);
}

TEST_F(ESC211ManagersTest, WaitForResponseTimesOutWithoutMatch) {
    addCommandRegs();
    setResponse(1);  // Ongoing — never becomes Done

    ESC211::ControlCommandChannel::Config cfg;
    cfg.poll_interval = std::chrono::milliseconds(5);

    ESC211::ControlCommandChannel ch(master_.slave(0), cfg);
    EXPECT_EQ(ch.waitForResponse(ControlCommandCode::StartFSoEAndSafetyApp,
                                 {}, std::chrono::milliseconds(120)),
              ESC211::ControlCommandChannel::Result::Timeout);
}

TEST_F(ESC211ManagersTest, WaitForResponseSeesLiveProgression) {
    addCommandRegs();
    setResponse(1);  // Ongoing
    std::thread done([this] {
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
        setResponse(2);  // Done
    });

    ESC211::ControlCommandChannel::Config cfg;
    cfg.poll_interval = std::chrono::milliseconds(5);
    ESC211::ControlCommandChannel ch(master_.slave(0), cfg);
    EXPECT_EQ(ch.waitForResponse(ControlCommandCode::StartFSoEAndSafetyApp,
                                 {}, std::chrono::milliseconds(1500)),
              ESC211::ControlCommandChannel::Result::Success);
    done.join();
}

TEST_F(ESC211ManagersTest, PrimitivesSendAndPoll) {
    addCommandRegs();
    setResponse(2);

    ESC211::ControlCommandChannel ch(master_.slave(0), {});
    EXPECT_TRUE(ch.resetResponseToStandby());
    EXPECT_TRUE(ch.sendCommand(ControlCommandCode::FlashMemoryReset));
    // The emulator records the command word on the wire.
    const auto* cmd = slave_->odEntry(0xF100, 0x01);
    ASSERT_NE(cmd, nullptr);
    uint32_t v = 0;
    for (size_t i = 0; i < cmd->data.size() && i < 4; ++i)
        v |= static_cast<uint32_t>(cmd->data[i]) << (8 * i);
    EXPECT_EQ(v, 9001u);  // FlashMemoryReset

    uint32_t resp = 0;
    EXPECT_TRUE(ch.pollResponse(resp));
    EXPECT_EQ(resp, 2u);
}

TEST_F(ESC211ManagersTest, ResultToStringCoversAllValues) {
    using R = ESC211::ControlCommandChannel::Result;
    EXPECT_NE(ESC211::ControlCommandChannel::resultToString(R::Success), nullptr);
    EXPECT_NE(ESC211::ControlCommandChannel::resultToString(R::Failed), nullptr);
    EXPECT_NE(ESC211::ControlCommandChannel::resultToString(R::Timeout), nullptr);
    EXPECT_NE(ESC211::ControlCommandChannel::resultToString(R::SDOError), nullptr);
}

TEST_F(ESC211ManagersTest, CommandNameForKnownAndUnknown) {
    EXPECT_NE(ESC211::ControlCommandChannel::commandName(
                  ControlCommandCode::ClearLastErrorCode),
              nullptr);
    EXPECT_NE(ESC211::ControlCommandChannel::commandName(
                  static_cast<ControlCommandCode>(999)),
              nullptr);
}

// ============================================================================
// SafetyStateMachineManager — start/stop/reset sequences
// ============================================================================

TEST_F(ESC211ManagersTest, StartMonitoringConnectionOnlyReachesMonitoring) {
    addCommandRegs();
    setResponse(2);          // command Done
    setState(6);             // already MONITORING

    ESC211::SafetyStateMachineManager::Config cfg;
    cfg.command_timeout = std::chrono::milliseconds(500);
    cfg.poll_interval = std::chrono::milliseconds(5);
    cfg.monitoring_state_timeout = std::chrono::milliseconds(500);

    ESC211::SafetyStateMachineManager mgr(master_.slave(0), cfg);
    EXPECT_EQ(mgr.startMonitoring(ESC211::SafetyStateMachineManager::
                                  StartMode::FSoEConnectionOnly),
              ESC211::SafetyStateMachineManager::Result::Success);
    EXPECT_TRUE(mgr.isMonitoring());
    EXPECT_EQ(mgr.lastSystemState(), 6u);
}

TEST_F(ESC211ManagersTest, StartMonitoringTimesOutWithoutMonitoringState) {
    addCommandRegs();
    setResponse(2);
    setState(4);  // stuck below MONITORING

    ESC211::SafetyStateMachineManager::Config cfg;
    cfg.command_timeout = std::chrono::milliseconds(300);
    cfg.poll_interval = std::chrono::milliseconds(5);
    cfg.monitoring_state_timeout = std::chrono::milliseconds(120);

    ESC211::SafetyStateMachineManager mgr(master_.slave(0), cfg);
    EXPECT_NE(mgr.startMonitoring(ESC211::SafetyStateMachineManager::
                                  StartMode::FSoEConnectionOnly),
              ESC211::SafetyStateMachineManager::Result::Success);
    EXPECT_FALSE(mgr.isMonitoring());
}

TEST_F(ESC211ManagersTest, StopClearAndFlashResetSendCommands) {
    addCommandRegs();
    setResponse(2);

    ESC211::SafetyStateMachineManager::Config cfg;
    cfg.command_timeout = std::chrono::milliseconds(500);
    cfg.poll_interval = std::chrono::milliseconds(5);

    ESC211::SafetyStateMachineManager mgr(master_.slave(0), cfg);
    using R = ESC211::SafetyStateMachineManager::Result;
    EXPECT_EQ(mgr.clearApplicationErrorState(), R::Success);
    EXPECT_EQ(mgr.clearErrorState(), R::Success);
    EXPECT_EQ(mgr.stopMonitoring(), R::Success);
    EXPECT_EQ(mgr.flashMemoryReset(), R::Success);
}

TEST_F(ESC211ManagersTest, SafetyResultToStringCoversAllValues) {
    using R = ESC211::SafetyStateMachineManager::Result;
    EXPECT_NE(ESC211::SafetyStateMachineManager::resultToString(R::Success), nullptr);
    EXPECT_NE(ESC211::SafetyStateMachineManager::resultToString(R::Failed), nullptr);
    EXPECT_NE(ESC211::SafetyStateMachineManager::resultToString(R::Timeout), nullptr);
    EXPECT_NE(ESC211::SafetyStateMachineManager::resultToString(R::SDOError), nullptr);
}

// ============================================================================
// ErrorStateReader — vendor-mirror error info read (Master&)
// ============================================================================

TEST_F(ESC211ManagersTest, ErrorStateReaderReadsCleanState) {
    addU(0xF101, 0x00, 4, 4);
    addU(0xF102, 0x00, 0, 4);
    addStr(0xF103, 0x00, "");
    addStr(0xF110, 0x01, "info: watchdog armed");

    ESC211::ErrorStateReader r(master_, 0);
    auto snap = r.readAndLog();
    EXPECT_TRUE(snap.valid);
    EXPECT_EQ(snap.system_state, 4u);
    EXPECT_EQ(snap.error_code, 0);
    EXPECT_TRUE(ESC211::ErrorStateReader::isOk(snap));
}

TEST_F(ESC211ManagersTest, ErrorStateReaderReadsErrorFields) {
    addU(0xF101, 0x00, 3, 4);
    addU(0xF102, 0x00, static_cast<uint32_t>(-1), 4);
    addStr(0xF103, 0x00, "RSAP state invalid");
    addStr(0xF110, 0x01, "crc mismatch");

    ESC211::ErrorStateReader r(master_, 0);
    auto snap = r.readAndLog();
    EXPECT_TRUE(snap.valid);
    EXPECT_EQ(snap.error_code, -1);
    EXPECT_FALSE(ESC211::ErrorStateReader::isOk(snap));
}

TEST_F(ESC211ManagersTest, ErrorStateReaderMarksInvalidOnFailure) {
    // Nothing injected — reads fail.
    ESC211::ErrorStateReader r(master_, 0);
    auto snap = r.readAndLog();
    EXPECT_FALSE(snap.valid);
    EXPECT_FALSE(ESC211::ErrorStateReader::isOk(snap));
}

// ============================================================================
// RSAPInformationProcessor — 0x40xx poll table + decode
// ============================================================================

TEST_F(ESC211ManagersTest, RsapInfoPollFillsSnapshotAndDecodes) {
    for (const auto& o : ESC211::RSAPInformationProcessor::kObjects) {
        addU(o.index, o.subindex, 0, o.bytes);
    }
    // RSAP State = 1 (MONITORING), Op mode = 0 (T1)
    addU(0x4000, 0x00, 1, 1);
    addU(0x4003, 0x00, 0, 1);

    ESC211::RSAPInformationProcessor proc(master_.slave(0));
    EXPECT_EQ(proc.poll(), ESC211::RSAPInformationProcessor::kObjectCount);
    auto snap = proc.snapshot();
    EXPECT_EQ(snap.read_ok, ESC211::RSAPInformationProcessor::kObjectCount);
    EXPECT_EQ(snap.generation, 1u);
    EXPECT_FALSE(ESC211::RSAPInformationProcessor::anyTriggered(snap));

    auto all = ESC211::RSAPInformationProcessor::decodeAll(snap);
    EXPECT_EQ(all.size(), ESC211::RSAPInformationProcessor::kObjectCount);
    bool saw_state = false;
    for (const auto& l : all) {
        if (l.find("RSAP State") != std::string::npos) saw_state = true;
    }
    EXPECT_TRUE(saw_state);
    EXPECT_TRUE(ESC211::RSAPInformationProcessor::decodeTriggered(snap).empty());
}

TEST_F(ESC211ManagersTest, RsapInfoTriggeredBitfieldsDecode) {
    for (const auto& o : ESC211::RSAPInformationProcessor::kObjects) {
        addU(o.index, o.subindex, 0, o.bytes);
    }
    addU(0x400B, 0x00, 0x05, 1);          // Emergency stop bits 0+2
    addU(0x4002, 0x00, 12, 2);            // Error code non-zero

    ESC211::RSAPInformationProcessor proc(master_.slave(0));
    proc.poll();
    auto snap = proc.snapshot();
    EXPECT_TRUE(ESC211::RSAPInformationProcessor::anyTriggered(snap));

    auto trig = ESC211::RSAPInformationProcessor::decodeTriggered(snap);
    EXPECT_GE(trig.size(), 2u);  // estop field + error code line
    bool saw_bit = false;
    for (const auto& l : trig) {
        if (l.find("Emergency") != std::string::npos &&
            (l.find("bit") != std::string::npos || l.find("(") != std::string::npos))
            saw_bit = true;
    }
    EXPECT_TRUE(saw_bit);
}

TEST_F(ESC211ManagersTest, RsapInfoFailedReadsLeaveObjectsOut) {
    // Only a few objects exist — the rest fail and are excluded.
    addU(0x4000, 0x00, 1, 1);

    ESC211::RSAPInformationProcessor proc(master_.slave(0));
    EXPECT_LT(proc.poll(), ESC211::RSAPInformationProcessor::kObjectCount);
    auto snap = proc.snapshot();
    auto all = ESC211::RSAPInformationProcessor::decodeAll(snap);
    EXPECT_EQ(all.size(), 1u);
    EXPECT_FALSE(ESC211::RSAPInformationProcessor::anyTriggered(snap));
}

TEST_F(ESC211ManagersTest, RsapInfoStaticNameTables) {
    // Named entries: 0, the RSAP range -1..-17, deeper CONFIG/SAFETY codes,
    // and the two safety-diagnosis ranges — plus the unknown fallback.
    EXPECT_STREQ(ESC211::RSAPInformationProcessor::errorCodeName(0), "success");
    EXPECT_NE(ESC211::RSAPInformationProcessor::errorCodeName(-1), nullptr);
    EXPECT_NE(ESC211::RSAPInformationProcessor::errorCodeName(-17), nullptr);
    EXPECT_NE(ESC211::RSAPInformationProcessor::errorCodeName(-101), nullptr);
    EXPECT_NE(ESC211::RSAPInformationProcessor::errorCodeName(-305), nullptr);
    EXPECT_NE(ESC211::RSAPInformationProcessor::errorCodeName(-325), nullptr);
    // Unmapped codes return nullptr — callers print the number verbatim.
    EXPECT_EQ(ESC211::RSAPInformationProcessor::errorCodeName(99999), nullptr);
}

// ============================================================================
// RSAPMonitoringProcessor — 0x41xx calculated values poll
// ============================================================================

TEST_F(ESC211ManagersTest, RsapMonitoringPollsAndGetsValues) {
    for (const auto& o : ESC211::RSAPMonitoringProcessor::kObjects) {
        addU(o.index, o.subindex, 0, 4);
    }
    addU(0x4100, 0x01, 0x00012345, 4);          // TCP 0 Pos X
    addU(0x4100, 0x04, static_cast<uint32_t>(-1000), 4);  // signed director
    addU(0x4101, 0x00, 5000, 4);                // speed limit

    ESC211::RSAPMonitoringProcessor mon(master_.slave(0));
    EXPECT_EQ(mon.poll(), ESC211::RSAPMonitoringProcessor::kObjectCount);

    auto snap = mon.snapshot();
    ASSERT_TRUE(snap.get(0x4100, 0x01).has_value());
    EXPECT_EQ(*snap.get(0x4100, 0x01), 0x12345u);
    ASSERT_TRUE(snap.getSigned(0x4100, 0x04).has_value());
    EXPECT_EQ(*snap.getSigned(0x4100, 0x04), -1000);
    ASSERT_TRUE(snap.get(0x4101).has_value());
    EXPECT_EQ(*snap.get(0x4101), 5000u);
    // Out-of-table lookups return nullopt.
    EXPECT_FALSE(snap.get(0x4999).has_value());
}

TEST_F(ESC211ManagersTest, RsapMonitoringMissingObjectsUnread) {
    addU(0x4101, 0x00, 42, 4);  // only the scalar limit

    ESC211::RSAPMonitoringProcessor mon(master_.slave(0));
    EXPECT_LT(mon.poll(), ESC211::RSAPMonitoringProcessor::kObjectCount);
    auto snap = mon.snapshot();
    ASSERT_TRUE(snap.get(0x4101).has_value());
    EXPECT_EQ(*snap.get(0x4101), 42u);
    EXPECT_FALSE(snap.get(0x4100, 0x01).has_value());
}

// ============================================================================
// ConfigManager — section writes via segmented download
// ============================================================================

TEST_F(ESC211ManagersTest, ConfigWriteSectionsUsesSegmentedDownload) {
    for (uint8_t s = 1; s <= 2; ++s) addSection(0xF200, s);

    ESC211::ConfigManager::Config cfg;
    cfg.post_command_reset = false;
    ESC211::ConfigManager mgr(master_.slave(0), cfg);

    std::vector<uint8_t> sec1(256, 0x11), sec2(256, 0x22);
    const std::vector<std::vector<uint8_t>> sections = {sec1, sec2};
    auto res = mgr.writeSections(ESC211::ConfigDataType::FNI, sections);
    EXPECT_EQ(res, ESC211::ConfigManager::Result::Success);

    const auto* e1 = slave_->odEntry(0xF200, 1);
    ASSERT_NE(e1, nullptr);
    EXPECT_EQ(e1->data.size(), 256u);
    EXPECT_EQ(e1->data[0], 0x11);
}

TEST_F(ESC211ManagersTest, ConfigWriteRawDataSplitsAndPads) {
    addSection(0xF200, 1);

    ESC211::ConfigManager mgr(master_.slave(0), {});
    std::vector<uint8_t> raw(100, 0xAB);
    auto res = mgr.writeRawData(ESC211::ConfigDataType::FNI, raw);
    EXPECT_EQ(res, ESC211::ConfigManager::Result::Success);

    const auto* e = slave_->odEntry(0xF200, 1);
    ASSERT_NE(e, nullptr);
    ASSERT_EQ(e->data.size(), 256u);  // zero-padded to a full section
    EXPECT_EQ(e->data[0], 0xAB);
    EXPECT_EQ(e->data.back(), 0x00);
}

TEST_F(ESC211ManagersTest, ConfigWriteRawDataRejectsOversized) {
    ESC211::ConfigManager mgr(master_.slave(0), {});
    std::vector<uint8_t> raw(9 * 256, 0x01);  // FNI max is 8 sections
    auto res = mgr.writeRawData(ESC211::ConfigDataType::FNI, raw);
    EXPECT_EQ(res, ESC211::ConfigManager::Result::NoData);
}

TEST_F(ESC211ManagersTest, ConfigActivateSendsCommandAndWaitsDone) {
    addCommandRegs();
    setResponse(2);  // Done

    ESC211::ConfigManager::Config cfg;
    cfg.command_timeout = std::chrono::milliseconds(500);
    cfg.post_command_reset = false;
    ESC211::ConfigManager mgr(master_.slave(0), cfg);

    const ESC211::ConfigDataType types[] = {ESC211::ConfigDataType::FNI};
    auto res = mgr.activate(types, {}, false);
    EXPECT_EQ(res, ESC211::ConfigManager::Result::Success);

    const auto* cmd = slave_->odEntry(0xF100, 0x01);
    ASSERT_NE(cmd, nullptr);
    uint32_t v = 0;
    for (size_t i = 0; i < cmd->data.size() && i < 4; ++i)
        v |= static_cast<uint32_t>(cmd->data[i]) << (8 * i);
    EXPECT_EQ(v, 3u);  // UpdateFNIFromObject
}

TEST_F(ESC211ManagersTest, ConfigManagerResultToString) {
    using R = ESC211::ConfigManager::Result;
    EXPECT_NE(ESC211::ConfigManager::resultToString(R::Success), nullptr);
    EXPECT_NE(ESC211::ConfigManager::resultToString(R::Failed), nullptr);
    EXPECT_NE(ESC211::ConfigManager::resultToString(R::Timeout), nullptr);
    EXPECT_NE(ESC211::ConfigManager::resultToString(R::SDOError), nullptr);
    EXPECT_NE(ESC211::ConfigManager::resultToString(R::NoData), nullptr);
}

/**
 * @file test_cia405_wire.cpp
 * @brief Wire-level coverage for CiA405::PLCDevice (CiA405PLC.cpp): initialize/
 *        detectCapabilities, update() SDO reads, status/debug TxPDO +
 *        control RxPDO paths, program/task commands, variable access,
 *        %I/%Q/%M memory areas, exceptions, debugging and file transfer —
 *        driven over EmulatedBus with a real SlaveEmulator.
 */

#include <gtest/gtest.h>

#include <bit>
#include <cstdint>
#include <cstring>
#include <memory>
#include <string>
#include <vector>

#include "mocks/EmulatedBus.hpp"
#include "tether/ethercat/CoEManager.hpp"
#include "tether/ethercat/SlaveEmulator.hpp"
#include "tether/platform/Platform.hpp"
#include "tether/profiles/cia405/CiA405PLC.hpp"
#include "tether/profiles/cia405/CiA405Defs.hpp"

using namespace EtherCAT;
using namespace EtherCAT::Emulator;
using namespace CiA405;

namespace {

SlaveEmulator* addPlcSlave(EmulatedBus& bus) {
    auto slave = std::make_unique<SlaveEmulator>();
    SIIConfig cfg;
    cfg.vendor_id = 0x00000002;
    cfg.product_code = 0x0405;
    cfg.mailbox_protocol = 0x04;
    auto addSm = [&cfg](uint16_t addr, uint16_t len, uint8_t ctrl) {
        SIIConfig::SMConfig sm{};
        sm.start_addr = addr;
        sm.length     = len;
        sm.control    = std::bit_cast<SyncManager::SMControlReg>(ctrl);
        sm.enable     = std::bit_cast<SyncManager::SMActivateReg>(
            static_cast<uint8_t>(0x01));
        cfg.sync_managers.push_back(sm);
    };
    addSm(0x1000, 128, 0x26);
    addSm(0x1080, 128, 0x22);
    slave->setSIIConfig(cfg);

    auto u8 = [](uint8_t v) { return std::vector<uint8_t>{v}; };
    auto u16 = [](uint16_t v) {
        return std::vector<uint8_t>{static_cast<uint8_t>(v),
                                    static_cast<uint8_t>(v >> 8)};
    };
    auto u32 = [](uint32_t v) {
        return std::vector<uint8_t>{static_cast<uint8_t>(v),
                                    static_cast<uint8_t>(v >> 8),
                                    static_cast<uint8_t>(v >> 16),
                                    static_cast<uint8_t>(v >> 24)};
    };
    auto str = [](const char* s) {
        return std::vector<uint8_t>(s, s + strlen(s) + 1);
    };
    std::vector<ODEntry> entries;
    auto add = [&](uint16_t i, uint8_t s, const std::vector<uint8_t>& d) {
        entries.push_back({i, s, 0x0002,
                           static_cast<uint16_t>(d.size() * 8),
                           0x03, d, "cia405"});
    };

    add(0x1000, 0, u32(0x00000195));            // device type, profile 405
    add(ProgramControl, ProgControlSub::Command, u8(0));
    add(ProgramStatus, 0, u8(1));
    add(ProgramError, 0, u16(0));
    add(ProgramName, 0, str("MAIN"));
    add(ProgramVersion, 0, u16(0x0102));
    add(ProgramChecksum, 0, u32(0x1234));
    for (uint8_t i = 0; i < 4; ++i) {
        add(TaskControl1 + i, TaskControlSub::Command, u8(0));
        add(TaskPriority1 + i, 0, u8(0));
        add(TaskInterval1 + i, 0, u32(0));
    }
    add(TaskStatus1, 0, u8(1));                 // 2 tasks
    add(TaskStatus2, 0, u8(2));
    add(ExceptionStatus, 0, u8(0));
    add(ExceptionCode, 0, u16(0));
    add(ExceptionInfoIndex, 0, u32(0));
    add(SystemTime, 0, u32(424242));
    add(CycleTime, 0, u16(1000));
    add(MaxCycleTime, 0, u16(2000));
    add(MinCycleTime, 0, u16(500));
    add(WatchdogTime, 0, u16(0));
    add(WatchdogTime, 1, u32(0));
    add(WatchdogTime, 2, u32(0));
    // Variables
    add(VariableCount, 0, u16(2));
    add(VariableName, 0, str("speed"));
    add(VariableName, 1, str("count"));
    add(VariableType, 0, u8(6));
    add(VariableType, 1, u8(3));
    add(VariableAddress, 0, u32(0x100));
    add(VariableAddress, 1, u32(0x104));
    add(VariableSize, 0, u16(4));
    add(VariableSize, 1, u16(2));
    for (uint8_t i = 0; i < 4; ++i) {
        add(VariableReadByIndex, i, u32(0));
        add(VariableWriteByIndex, i, u32(0));
    }
    // Memory areas: %I 16B, %Q 16B, %M 32B
    add(InputVariables, 0, u32(16));
    add(OutputVariables, 0, u32(16));
    add(MemoryVariables, 0, u32(32));
    for (uint8_t i = 0; i < 4; ++i) {
        add(InputByte, i, u8(0x11 + i));
        add(InputWord, i, u16(0x2222));
        add(InputDWord, i, u32(0x33333333));
        add(OutputByte, i, u8(0));
        add(OutputWord, i, u16(0));
        add(OutputDWord, i, u32(0));
        add(MemoryByte, i, u8(0));
        add(MemoryWord, i, u16(0));
        add(MemoryDWord, i, u32(0));
    }
    // Debug interface
    for (uint8_t i = 0; i < 8; ++i) add(DebugControl, i, u8(0));
    add(DebugStatus, 0, u8(1));
    for (uint8_t i = 0; i < 8; ++i) add(BreakpointAddress, i, u32(0));
    add(SingleStepControl, 0, u8(0));
    // CPU/memory + file transfer
    add(CPULoad, 0, u16(4200));
    add(MemoryUsed, 0, u32(65536));
    add(FileTransferControl, 0, u8(0));
    add(FileTransferStatusIndex, 0, u8(1));
    add(FileName, 0, str(""));
    add(FileSize, 0, u32(0));
    for (uint8_t i = 0; i < 4; ++i) add(FileData, i, std::vector<uint8_t>(256, 0));
    for (auto& e : entries) slave->addODEntry(e);

    auto* raw = slave.get();
    bus.net.addSlave(std::move(slave));
    return raw;
}

class CiA405WireTest : public ::testing::Test {
protected:
    void SetUp() override {
        slave_ = addPlcSlave(bus_);
        bus_.attach(master_);
        master_.start(bus_.iface, kEmulatedMac);
        auto found = master_.discovery().discover(DiscoveryOptions{});
        ASSERT_EQ(found.size(), 1u);
        transport_ = makeMasterSDOTransport(master_);
        coe_ = std::make_unique<CoE::CoEManager>(0, *transport_);
        coe_->init();
        coe_->configureMailbox(0x1000, 128, 0x1080, 128);
        plc_ = std::make_unique<PLCDevice>(*coe_);
    }
    void TearDown() override {
        plc_.reset();
        coe_->deinit();
        coe_.reset();
        transport_.reset();
        bus_.shutdown();
        master_.packetRouter().shutdown();
    }

    void writeOD(uint16_t index, uint8_t sub, const void* data, size_t len) {
        auto r = coe_->writeSync(index, sub, data, len, {});
        ASSERT_TRUE(r.has_value());
    }

    EmulatedBus bus_;
    Master master_;
    SlaveEmulator* slave_ = nullptr;
    std::unique_ptr<SDO::ISDOTransport> transport_;
    std::unique_ptr<CoE::CoEManager> coe_;
    std::unique_ptr<PLCDevice> plc_;
};

} // namespace

TEST_F(CiA405WireTest, InitializeDetectsCapabilities) {
    ASSERT_TRUE(plc_->initialize());
    EXPECT_TRUE(plc_->isInitialized());
    const auto& caps = plc_->getCapabilities();
    EXPECT_EQ(caps.max_tasks, 2u);
    EXPECT_EQ(caps.max_variables, 2u);
    EXPECT_EQ(caps.input_size, 16u);
    EXPECT_EQ(caps.output_size, 16u);
    EXPECT_EQ(caps.memory_size, 32u);
    EXPECT_TRUE(caps.supports_debug);
    EXPECT_TRUE(caps.supports_file_transfer);
    EXPECT_EQ(plc_->getTaskCount(), 2u);
}

TEST_F(CiA405WireTest, UpdateReadsProgramAndTasks) {
    ASSERT_TRUE(plc_->initialize());
    plc_->update();
    EXPECT_EQ(plc_->getProgramState(), 1);
    EXPECT_EQ(plc_->getTaskInfo(1).state, 1);
    EXPECT_EQ(plc_->getTaskInfo(2).state, 2);
    EXPECT_EQ(plc_->getSystemTime(), 424242u);
}

TEST_F(CiA405WireTest, ProgramCommandsOverWire) {
    ASSERT_TRUE(plc_->initialize());
    EXPECT_TRUE(plc_->startProgram());
    EXPECT_TRUE(plc_->startProgram(true));          // cold start
    EXPECT_TRUE(plc_->stopProgram());
    EXPECT_TRUE(plc_->haltProgram());
    EXPECT_TRUE(plc_->continueProgram());
    EXPECT_TRUE(plc_->resetProgram());
}

TEST_F(CiA405WireTest, TaskCommandsAndConfig) {
    ASSERT_TRUE(plc_->initialize());
    EXPECT_TRUE(plc_->startTask(1));
    EXPECT_TRUE(plc_->stopTask(1));
    EXPECT_TRUE(plc_->suspendTask(1));
    EXPECT_TRUE(plc_->resumeTask(1));
    EXPECT_TRUE(plc_->singleCycleTask(2));
    EXPECT_FALSE(plc_->startTask(9));
    EXPECT_TRUE(plc_->configureTask(1, 5, 10000, 100));
    EXPECT_EQ(plc_->getTaskInfo(1).priority, 5);
    EXPECT_FALSE(plc_->configureTask(9, 1, 1, 0));
}

TEST_F(CiA405WireTest, StatusAndDebugPDO) {
    ASSERT_TRUE(plc_->initialize());
    int stateHits = 0;
    plc_->setProgramStateCallback(
        [&stateHits](uint8_t, uint8_t) { ++stateHits; });
    int taskHits = 0;
    plc_->setTaskStateCallback(
        [&taskHits](uint8_t, uint8_t, uint8_t) { ++taskHits; });
    int exHits = 0;
    plc_->setExceptionCallback(
        [&exHits](const ExceptionInfo&) { ++exHits; });

    TxPDO_Status pdo{};
    pdo.program_state = 2;
    pdo.task1_state = 3;
    pdo.task2_state = 4;
    pdo.exception_code = ExceptionCodes::Watchdog;
    plc_->processTxPDO(reinterpret_cast<const uint8_t*>(&pdo), sizeof(pdo));
    EXPECT_EQ(plc_->getProgramState(), 2);
    EXPECT_GE(stateHits, 1);
    EXPECT_GE(taskHits, 1);
    EXPECT_GE(exHits, 1);
    EXPECT_FALSE(plc_->getExceptionHistory().empty());

    // Debug mapping: status + debug trailer
    plc_->applyPDOMapping(PDOMappingPreset::Debug);
    int bpHits = 0;
    plc_->setBreakpointCallback([&bpHits](const Breakpoint&) { ++bpHits; });
    ASSERT_TRUE(plc_->enableDebug());
    ASSERT_TRUE(plc_->setBreakpoint(0x1234, ""));
    std::vector<uint8_t> dbg(sizeof(TxPDO_Status) + sizeof(TxPDO_Debug), 0);
    auto* tail = reinterpret_cast<TxPDO_Debug*>(dbg.data() + sizeof(TxPDO_Status));
    tail->current_address = 0x1234;
    tail->current_line = 7;
    tail->breakpoint_hit = 1;
    plc_->processTxPDO(dbg.data(), dbg.size());
    EXPECT_EQ(bpHits, 1);
}

TEST_F(CiA405WireTest, RxPDOEmitsPendingCommands) {
    ASSERT_TRUE(plc_->initialize());
    EXPECT_TRUE(plc_->startProgram());
    EXPECT_TRUE(plc_->startTask(1));
    EXPECT_TRUE(plc_->singleCycleTask(2));
    uint8_t buf[sizeof(RxPDO_Control)] = {};
    size_t n = plc_->prepareRxPDO(buf, sizeof(buf));
    ASSERT_EQ(n, sizeof(RxPDO_Control));
    auto* pdo = reinterpret_cast<RxPDO_Control*>(buf);
    EXPECT_EQ(pdo->program_command, ProgramCommands::Start);
    EXPECT_EQ(pdo->task1_command, TaskCommands::Start);
    EXPECT_EQ(pdo->task2_command, TaskCommands::SingleCycle);
    // Pending commands are cleared after emission.
    memset(buf, 0, sizeof(buf));
    plc_->prepareRxPDO(buf, sizeof(buf));
    EXPECT_EQ(buf[0], ProgramCommands::NoOp);
}

TEST_F(CiA405WireTest, VariablesByIndexAndName) {
    ASSERT_TRUE(plc_->initialize());
    ASSERT_TRUE(plc_->loadSymbolTable());
    EXPECT_EQ(plc_->getVariables().size(), 2u);
    ASSERT_NE(plc_->getVariableInfo(0), nullptr);
    EXPECT_EQ(plc_->getVariableInfo(0)->name, "speed");
    ASSERT_NE(plc_->getVariableInfo(std::string("count")), nullptr);
    EXPECT_EQ(plc_->getVariableInfo(std::string("count"))->index, 1);
    EXPECT_EQ(plc_->getVariableInfo(99), nullptr);
    EXPECT_EQ(plc_->getVariableInfo(std::string("nope")), nullptr);

    uint32_t v = 0;
    EXPECT_TRUE(plc_->readVariable(1, &v, sizeof(v)));
    int32_t w = -7;
    EXPECT_TRUE(plc_->writeVariable(1, &w, sizeof(w)));

    int32_t nv = 0;
    EXPECT_TRUE(plc_->readVariableByName("count", &nv, sizeof(nv)));
    EXPECT_TRUE(plc_->writeVariableByName("speed", &nv, sizeof(nv)));
    EXPECT_FALSE(plc_->readVariableByName("missing", &nv, sizeof(nv)));
}

TEST_F(CiA405WireTest, TypedVariableAccess) {
    ASSERT_TRUE(plc_->initialize());
    bool b = false;
    int16_t i16 = 0;
    int32_t i32 = 0;
    float f = 0.0f;
    EXPECT_TRUE(plc_->readBool(1, b));
    EXPECT_TRUE(plc_->writeBool(1, true));
    EXPECT_TRUE(plc_->readInt16(1, i16));
    EXPECT_TRUE(plc_->writeInt16(1, -12));
    EXPECT_TRUE(plc_->readInt32(1, i32));
    EXPECT_TRUE(plc_->writeInt32(1, 424242));
    EXPECT_TRUE(plc_->readReal(1, f));
    EXPECT_TRUE(plc_->writeReal(1, 3.5f));
}

TEST_F(CiA405WireTest, MemoryAreaAccess) {
    ASSERT_TRUE(plc_->initialize());
    uint8_t b = 0;
    uint16_t w = 0;
    uint32_t d = 0;
    EXPECT_TRUE(plc_->readInputByte(0, b));
    EXPECT_EQ(b, 0x11);
    EXPECT_TRUE(plc_->readInputWord(1, w));
    EXPECT_EQ(w, 0x2222);
    EXPECT_TRUE(plc_->readInputDWord(2, d));
    EXPECT_EQ(d, 0x33333333u);
    EXPECT_TRUE(plc_->readOutputByte(0, b));
    EXPECT_TRUE(plc_->readOutputWord(1, w));
    EXPECT_TRUE(plc_->readOutputDWord(2, d));
    EXPECT_TRUE(plc_->writeOutputByte(0, 0xAA));
    EXPECT_TRUE(plc_->writeOutputWord(1, 0xBBBB));
    EXPECT_TRUE(plc_->writeOutputDWord(2, 0xCCCCCCCC));
    EXPECT_TRUE(plc_->readMemoryByte(0, b));
    EXPECT_TRUE(plc_->readMemoryWord(1, w));
    EXPECT_TRUE(plc_->readMemoryDWord(2, d));
    EXPECT_TRUE(plc_->writeMemoryByte(0, 1));
    EXPECT_TRUE(plc_->writeMemoryWord(1, 2));
    EXPECT_TRUE(plc_->writeMemoryDWord(2, 3));
}

TEST_F(CiA405WireTest, ExceptionsViaWire) {
    ASSERT_TRUE(plc_->initialize());
    int hits = 0;
    plc_->setExceptionCallback([&hits](const ExceptionInfo&) { ++hits; });
    uint16_t code = ExceptionCodes::DivisionByZero;
    writeOD(ExceptionCode, 0, &code, sizeof(code));
    plc_->update();
    EXPECT_EQ(hits, 1);
    EXPECT_FALSE(plc_->getExceptionHistory().empty());
    EXPECT_TRUE(plc_->clearException());
}

TEST_F(CiA405WireTest, DebugOperations) {
    ASSERT_TRUE(plc_->initialize());
    EXPECT_TRUE(plc_->enableDebug());
    EXPECT_TRUE(plc_->setBreakpoint(0x2000, "x > 1"));
    EXPECT_TRUE(plc_->clearBreakpoint(1));
    EXPECT_TRUE(plc_->setBreakpoint(0x3000, ""));
    EXPECT_TRUE(plc_->clearAllBreakpoints());
    EXPECT_TRUE(plc_->stepInto());
    EXPECT_TRUE(plc_->stepOver());
    EXPECT_TRUE(plc_->stepOut());
    EXPECT_TRUE(plc_->runToCursor(0x4000));
    EXPECT_TRUE(plc_->addWatch(1));
    EXPECT_TRUE(plc_->addWatch(2));
    EXPECT_TRUE(plc_->removeWatch(1));
    EXPECT_TRUE(plc_->disableDebug());
}

TEST_F(CiA405WireTest, FileTransfer) {
    ASSERT_TRUE(plc_->initialize());
    std::vector<uint8_t> prog(600, 0xAB);
    EXPECT_TRUE(plc_->downloadProgram(prog.data(), prog.size(), "fw.bin"));
    EXPECT_EQ(plc_->getFileTransferStatus().state,
              FileTransferState::Complete);
    EXPECT_EQ(plc_->getFileTransferStatus().transferred, 600u);

    // FileSize was written by the download, so upload reads it back.
    std::vector<uint8_t> back;
    EXPECT_TRUE(plc_->uploadProgram(back));
    EXPECT_EQ(back.size(), 600u);
    EXPECT_EQ(plc_->getFileTransferStatus().state,
              FileTransferState::Complete);
    EXPECT_TRUE(plc_->deleteFile("fw.bin"));
    EXPECT_TRUE(plc_->abortFileTransfer());
}

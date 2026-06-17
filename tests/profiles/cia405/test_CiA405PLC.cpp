/**
 * @file test_CiA405PLC.cpp
 * @brief Comprehensive tests for CiA 405 PLC Device
 */
#include <gtest/gtest.h>
#include "tether/profiles/cia405/CiA405PLC.hpp"
#include "tether/ethercat/SDOManager.hpp"
#include "tether/ethercat/CoEManager.hpp"
#include <cstring>

using namespace CiA405;

namespace {
class NullSDOTransport : public EtherCAT::SDO::ISDOTransport {
public:
    bool sdoUpload(uint16_t, uint8_t*, uint16_t, uint16_t,
                   uint16_t, uint16_t, uint16_t, uint8_t,
                   uint8_t*, size_t, size_t*, bool, unsigned int,
                   unsigned int) override { return false; }
    bool sdoDownload(uint16_t, uint8_t*, uint16_t, uint16_t,
                     uint16_t, uint16_t, uint16_t, uint8_t,
                     const uint8_t*, size_t, bool, unsigned int,
                     unsigned int) override { return false; }
    uint64_t getMicroseconds() override { return 0; }
};
} // namespace

// ============================================================================
// Struct helper tests
// ============================================================================

TEST(CiA405Structs, ProgramInfo) {
    ProgramInfo p{};
    p.name = "test_prog";
    p.state = ProgramState::Running;
    EXPECT_TRUE(p.isRunning());
    EXPECT_FALSE(p.isStopped());
    EXPECT_FALSE(p.hasException());
    p.state = ProgramState::Stopped;
    EXPECT_TRUE(p.isStopped());
    EXPECT_FALSE(p.isRunning());
    p.state = ProgramState::Exception;
    EXPECT_TRUE(p.hasException());
}

TEST(CiA405Structs, TaskInfo) {
    TaskInfo t{};
    t.state = TaskState::Running;
    EXPECT_TRUE(t.isRunning());
    t.state = TaskState::Stopped;
    EXPECT_FALSE(t.isRunning());
}

TEST(CiA405Structs, ExceptionInfo) {
    ExceptionInfo e{};
    e.code = ExceptionCodes::None;
    EXPECT_FALSE(e.hasException());
    e.code = ExceptionCodes::DivisionByZero;
    EXPECT_TRUE(e.hasException());
    e.code = ExceptionCodes::StackOverflow;
    EXPECT_TRUE(e.hasException());
}

TEST(CiA405Structs, ResourceUsage) {
    ResourceUsage r{};
    r.cpu_load = 1234;       // 12.34%
    r.cpu_temperature = 250; // 25.0C
    EXPECT_NEAR(r.getCPULoadPercent(), 12.34f, 0.01f);
    EXPECT_NEAR(r.getCPUTempCelsius(), 25.0f, 0.1f);
}

TEST(CiA405Structs, FileTransferStatus) {
    FileTransferStatus f{};
    f.state = FileTransferState::Idle;
    EXPECT_FALSE(f.isActive());
    f.state = FileTransferState::Downloading;
    EXPECT_TRUE(f.isActive());
    f.state = FileTransferState::Uploading;
    EXPECT_TRUE(f.isActive());
    f.state = FileTransferState::Complete;
    EXPECT_FALSE(f.isActive());
}

TEST(CiA405Structs, PLCCapabilities) {
    PLCCapabilities c{};
    EXPECT_EQ(c.max_tasks, 0u);
    EXPECT_EQ(c.max_variables, 0u);
}

// ============================================================================
// Enum tests
// ============================================================================

TEST(CiA405Enums, ProgramState) {
    EXPECT_NE(static_cast<uint8_t>(ProgramState::Running),
              static_cast<uint8_t>(ProgramState::Stopped));
    EXPECT_NE(static_cast<uint8_t>(ProgramState::Halted),
              static_cast<uint8_t>(ProgramState::Exception));
}

TEST(CiA405Enums, ExceptionCodes) {
    EXPECT_EQ(static_cast<uint8_t>(ExceptionCodes::None), 0u);
    EXPECT_NE(static_cast<uint8_t>(ExceptionCodes::DivisionByZero),
              static_cast<uint8_t>(ExceptionCodes::Overflow));
}

TEST(CiA405Enums, IEC61131Types) {
    EXPECT_NE(static_cast<uint8_t>(IEC61131Types::BOOL),
              static_cast<uint8_t>(IEC61131Types::INT));
}

// ============================================================================
// PLCDevice fixture
// ============================================================================

class CiA405Test : public ::testing::Test {
protected:
    void SetUp() override {
        transport_ = std::make_unique<NullSDOTransport>();
        coe_ = std::make_unique<EtherCAT::CoE::CoEManager>(1, *transport_);
        coe_->init();
        plc_ = std::make_unique<PLCDevice>(*coe_);
    }
    void TearDown() override {
        plc_.reset();
        coe_->deinit();
    }
    std::unique_ptr<NullSDOTransport> transport_;
    std::unique_ptr<EtherCAT::CoE::CoEManager> coe_;
    std::unique_ptr<PLCDevice> plc_;
};

TEST_F(CiA405Test, Construction) {
    PLCDevice p2(*coe_);
    EXPECT_FALSE(p2.isInitialized());
}

TEST_F(CiA405Test, Initialize) {
    plc_->initialize();
}

TEST_F(CiA405Test, PDOMapping) {
    plc_->initialize();
    plc_->applyPDOMapping(PDOMappingPreset::Minimal);
    plc_->applyPDOMapping(PDOMappingPreset::Standard);
    plc_->applyPDOMapping(PDOMappingPreset::WithTasks);
    plc_->applyPDOMapping(PDOMappingPreset::FullControl);
    plc_->applyPDOMapping(PDOMappingPreset::Debug);
    plc_->applyPDOMapping(PDOMappingPreset::Custom);
}

TEST_F(CiA405Test, ProgramControl) {
    plc_->initialize();
    plc_->startProgram(true);  // cold start
    plc_->startProgram(false); // warm start
    (void)plc_->isProgramRunning();
    (void)plc_->getProgramState();
    (void)plc_->getProgramInfo();
    plc_->haltProgram();
    plc_->continueProgram();
    plc_->stopProgram();
    plc_->resetProgram();
}

TEST_F(CiA405Test, TaskControl) {
    plc_->initialize();
    (void)plc_->getTaskCount();
    plc_->startTask(0);
    plc_->stopTask(0);
    plc_->suspendTask(0);
    plc_->resumeTask(0);
    plc_->singleCycleTask(0);
    (void)plc_->getTaskInfo(0);
    plc_->configureTask(0, 10, 1000, 5000);
}

TEST_F(CiA405Test, VariableAccess) {
    plc_->initialize();
    uint8_t buf[64] = {};
    plc_->readVariable(0, buf, sizeof(buf));
    plc_->writeVariable(0, buf, sizeof(buf));
    plc_->readVariableByName("test_var", buf, sizeof(buf));
    plc_->writeVariableByName("test_var", buf, sizeof(buf));
    (void)plc_->getVariableInfo(0u);
    (void)plc_->getVariableInfo(std::string("test_var"));
    plc_->loadSymbolTable();
    auto vars = plc_->getVariables();
    (void)vars;
}

TEST_F(CiA405Test, TypedVariableAccess) {
    plc_->initialize();
    bool bval = false;
    (void)plc_->readBool(0, bval);
    plc_->writeBool(0, true);
    int16_t i16val = 0;
    (void)plc_->readInt16(0, i16val);
    plc_->writeInt16(0, 42);
    int32_t i32val = 0;
    (void)plc_->readInt32(0, i32val);
    plc_->writeInt32(0, 12345);
    float fval = 0.0f;
    (void)plc_->readReal(0, fval);
    plc_->writeReal(0, 3.14f);
}

TEST_F(CiA405Test, MemoryAccess) {
    plc_->initialize();
    uint8_t u8val = 0;
    uint16_t u16val = 0;
    uint32_t u32val = 0;
    (void)plc_->readInputByte(0, u8val);
    (void)plc_->readInputWord(0, u16val);
    (void)plc_->readInputDWord(0, u32val);
    (void)plc_->readOutputByte(0, u8val);
    (void)plc_->readOutputWord(0, u16val);
    (void)plc_->readOutputDWord(0, u32val);
    (void)plc_->readMemoryByte(0, u8val);
    (void)plc_->readMemoryWord(0, u16val);
    (void)plc_->readMemoryDWord(0, u32val);
    plc_->writeOutputByte(0, 0xAA);
    plc_->writeOutputWord(0, 0x5678);
    plc_->writeOutputDWord(0, 0xCAFEBABE);
    plc_->writeMemoryByte(0, 0x01);
    plc_->writeMemoryWord(0, 0x0203);
    plc_->writeMemoryDWord(0, 0x04050607);
}

TEST_F(CiA405Test, Exceptions) {
    plc_->initialize();
    auto ex = plc_->getCurrentException();
    EXPECT_FALSE(ex.hasException());
    EXPECT_FALSE(plc_->hasException());
    plc_->clearException();
    auto hist = plc_->getExceptionHistory();
    (void)hist;
}

TEST_F(CiA405Test, DebugOperations) {
    plc_->initialize();
    plc_->enableDebug();
    // SDO may fail, so debug state may not change
    (void)plc_->isDebugEnabled();
    plc_->disableDebug();
    (void)plc_->isDebugEnabled();

    plc_->enableDebug();
    plc_->setBreakpoint(0x1000, "POU == 10");
    plc_->clearBreakpoint(0);
    plc_->clearAllBreakpoints();
    plc_->stepInto();
    plc_->stepOver();
    plc_->stepOut();
    plc_->runToCursor(0x2000);
    (void)plc_->getCurrentAddress();
    (void)plc_->getCurrentLine();
    plc_->addWatch(0);
    plc_->removeWatch(0);
}

TEST_F(CiA405Test, FileTransfer) {
    plc_->initialize();
    std::vector<uint8_t> data = {1, 2, 3, 4};
    plc_->downloadProgram(data.data(), data.size(), "program.bin");
    std::vector<uint8_t> uploaded;
    plc_->uploadProgram(uploaded);
    plc_->deleteFile("program.bin");
    auto status = plc_->getFileTransferStatus();
    EXPECT_FALSE(status.isActive());
    plc_->abortFileTransfer();
}

TEST_F(CiA405Test, Timing) {
    plc_->initialize();
    auto usage = plc_->getResourceUsage();
    (void)usage;
    (void)plc_->getSystemTime();
    (void)plc_->getCycleTime();
    plc_->setWatchdogTime(5000);
}

TEST_F(CiA405Test, Callbacks) {
    plc_->setProgramStateCallback([](uint8_t, uint8_t) {});
    plc_->setTaskStateCallback([](uint8_t, uint8_t, uint8_t) {});
    plc_->setExceptionCallback([](const ExceptionInfo&) {});
    plc_->setBreakpointCallback([](const Breakpoint&) {});
    plc_->setVariableCallback([](uint16_t, const void*, size_t) {});
}

TEST_F(CiA405Test, Diagnostics) {
    plc_->initialize();
    auto diag = plc_->getDiagnostics();
    EXPECT_FALSE(diag.empty());
}

TEST_F(CiA405Test, PDOProcess) {
    plc_->initialize();
    uint8_t txbuf[128] = {};
    plc_->processTxPDO(txbuf, sizeof(txbuf));
    uint8_t rxbuf[128] = {};
    plc_->prepareRxPDO(rxbuf, sizeof(rxbuf));
}

TEST_F(CiA405Test, Update) {
    plc_->initialize();
    plc_->update();
}

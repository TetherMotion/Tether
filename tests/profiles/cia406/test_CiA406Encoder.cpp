/**
 * @file test_CiA406Encoder.cpp
 * @brief Comprehensive tests for CiA 406 Encoder
 */
#include <gtest/gtest.h>
#include "tether/profiles/cia406/CiA406Encoder.hpp"
#include "tether/ethercat/SDOManager.hpp"

using namespace CiA406;

namespace {
class NullSDOTransport : public EtherCAT::SDO::ISDOTransport {
public:
    bool sdoUpload(uint16_t, uint8_t*, uint16_t, uint16_t,
                   uint16_t, uint16_t, uint16_t, uint8_t,
                   uint8_t*, size_t, size_t*) override { return false; }
    bool sdoDownload(uint16_t, uint8_t*, uint16_t, uint16_t,
                     uint16_t, uint16_t, uint16_t, uint8_t,
                     const uint8_t*, size_t) override { return false; }
    uint64_t getMicroseconds() override { return 0; }
};
} // namespace

// ============================================================================
// Enum name lookups
// ============================================================================

TEST(CiA406Enums, EncoderClassName) {
    EXPECT_STREQ(getEncoderClassName(EncoderClassEx::C1_AbsoluteSingleTurn), "C1 (Absolute Single-Turn)");
    EXPECT_NE(getEncoderClassName(EncoderClassEx::C2_AbsoluteMultiTurn), nullptr);
    EXPECT_NE(getEncoderClassName(EncoderClassEx::C3_Incremental), nullptr);
}

TEST(CiA406Enums, InterfaceTypeName) {
    EXPECT_STREQ(getInterfaceTypeName(InterfaceType::BiSS_C), "BiSS-C");
    EXPECT_NE(getInterfaceTypeName(InterfaceType::SSI), nullptr);
    EXPECT_NE(getInterfaceTypeName(InterfaceType::EnDat22), nullptr);
}

// ============================================================================
// OperatingStatus
// ============================================================================

TEST(CiA406OperatingStatus, FromRaw) {
    auto os = OperatingStatus::fromRaw(0x0003);
    EXPECT_TRUE(os.position_valid);
    EXPECT_TRUE(os.scaling_active);
    os = OperatingStatus::fromRaw(0x0000);
    EXPECT_FALSE(os.position_valid);
}

// ============================================================================
// AlarmFlags
// ============================================================================

TEST(CiA406AlarmFlags, FromRaw) {
    auto af = AlarmFlags::fromRaw(0x0001);
    EXPECT_TRUE(af.hasAnyAlarm());
    auto desc = af.getDescription();
    EXPECT_GT(desc.size(), 0u);
}

TEST(CiA406AlarmFlags, NoAlarm) {
    auto af = AlarmFlags::fromRaw(0x0000);
    EXPECT_FALSE(af.hasAnyAlarm());
    // getDescription may still return content (e.g. "OK")
    auto desc = af.getDescription();
    (void)desc;
}

TEST(CiA406AlarmFlags, MultipleBits) {
    auto af = AlarmFlags::fromRaw(0x0005);
    EXPECT_TRUE(af.hasAnyAlarm());
    auto desc = af.getDescription();
    EXPECT_GE(desc.size(), 1u);
}

// ============================================================================
// PDOMappingPreset enum values
// ============================================================================

TEST(CiA406PDOPreset, EnumDistinct) {
    EXPECT_NE(static_cast<int>(PDOMappingPreset::Basic),
              static_cast<int>(PDOMappingPreset::WithVelocity));
    EXPECT_NE(static_cast<int>(PDOMappingPreset::Full),
              static_cast<int>(PDOMappingPreset::Custom));
    EXPECT_NE(static_cast<int>(PDOMappingPreset::MultiTurn),
              static_cast<int>(PDOMappingPreset::HighSpeed));
    EXPECT_NE(static_cast<int>(PDOMappingPreset::Diagnostic),
              static_cast<int>(PDOMappingPreset::Basic));
}

// ============================================================================
// WorkingAreaState enum
// ============================================================================

TEST(CiA406WorkingArea, EnumValues) {
    EXPECT_NE(static_cast<int>(WorkingAreaState::WithinArea1),
              static_cast<int>(WorkingAreaState::BelowLowLimit1));
    EXPECT_NE(static_cast<int>(WorkingAreaState::AboveHighLimit1),
              static_cast<int>(WorkingAreaState::NotConfigured));
}

// ============================================================================
// EncoderEvent enum
// ============================================================================

TEST(CiA406Events, EnumValues) {
    EXPECT_NE(static_cast<int>(EncoderEvent::PositionUpdated),
              static_cast<int>(EncoderEvent::AlarmTriggered));
    EXPECT_NE(static_cast<int>(EncoderEvent::ReferenceDone),
              static_cast<int>(EncoderEvent::CommunicationError));
}

// ============================================================================
// Encoder fixture
// ============================================================================

class CiA406Test : public ::testing::Test {
protected:
    void SetUp() override {
        transport_ = std::make_unique<NullSDOTransport>();
        sdo_ = std::make_unique<EtherCAT::SDO::SDOManager>(*transport_);
        sdo_->init();
        enc_ = std::make_unique<Encoder>(*sdo_, 1);
    }
    void TearDown() override {
        enc_.reset();
        sdo_->deinit();
    }
    std::unique_ptr<NullSDOTransport> transport_;
    std::unique_ptr<EtherCAT::SDO::SDOManager> sdo_;
    std::unique_ptr<Encoder> enc_;
};

TEST_F(CiA406Test, Construction) {
    Encoder e2(*sdo_, 0x100, true);
    EXPECT_FALSE(e2.isInitialized());
}

TEST_F(CiA406Test, Initialize) {
    enc_->initialize();
}

TEST_F(CiA406Test, Capabilities) {
    enc_->initialize();
    auto& caps = enc_->getCapabilities();
    (void)caps;
}

TEST_F(CiA406Test, PDOMappingAll) {
    enc_->initialize();
    enc_->applyPDOMapping(PDOMappingPreset::Basic);
    enc_->applyPDOMapping(PDOMappingPreset::WithVelocity);
    enc_->applyPDOMapping(PDOMappingPreset::Full);
    enc_->applyPDOMapping(PDOMappingPreset::MultiTurn);
    enc_->applyPDOMapping(PDOMappingPreset::HighSpeed);
    enc_->applyPDOMapping(PDOMappingPreset::Diagnostic);
    enc_->applyPDOMapping(PDOMappingPreset::Custom);
}

TEST_F(CiA406Test, PositionReadings) {
    enc_->initialize();
    (void)enc_->getPosition();
    (void)enc_->getRawPosition();
    (void)enc_->getMultiTurnValue();
    (void)enc_->getSingleTurnValue();
}

TEST_F(CiA406Test, VelocityAndAccel) {
    enc_->initialize();
    (void)enc_->getVelocity();
    (void)enc_->getAcceleration();
}

TEST_F(CiA406Test, Status) {
    enc_->initialize();
    auto& os = enc_->getStatus();
    (void)os;
    auto& al = enc_->getAlarms();
    (void)al;
    EXPECT_FALSE(enc_->hasAlarm());
    (void)enc_->getAlarmsRaw();
    (void)enc_->getWarnings();
    EXPECT_FALSE(enc_->hasWarning());
    (void)enc_->isPositionValid();
}

TEST_F(CiA406Test, ClearAlarms) {
    enc_->initialize();
    enc_->clearAlarms();
}

TEST_F(CiA406Test, Scaling) {
    enc_->initialize();
    enc_->setScaling(4096, 360000);
    enc_->setScaling(4096, 360000, 100);
    enc_->setScalingFromRange(4096, 360000);
    auto& sc = enc_->getScaling();
    (void)sc;
    enc_->disableScaling();
    enc_->setOffset(100);
}

TEST_F(CiA406Test, PresetAndReference) {
    enc_->initialize();
    enc_->presetPosition(0);
    enc_->presetPosition(1000);
    enc_->setZero();
    EXPECT_FALSE(enc_->isReferenced());
    enc_->abortReference();
}

TEST_F(CiA406Test, WorkingArea) {
    enc_->initialize();
    enc_->setWorkingArea(-10000, 10000);
    enc_->setWorkingArea(-5000, 5000, 2);
    auto ws = enc_->getWorkingAreaState();
    (void)ws;
    (void)enc_->isWithinWorkingArea();
}

TEST_F(CiA406Test, Diagnostics) {
    enc_->initialize();
    enc_->readDiagnostics();
    (void)enc_->getTemperature();
    (void)enc_->getSupplyVoltage();
    (void)enc_->getSignalQuality();
    (void)enc_->getOperatingTime();
    (void)enc_->getSerialNumber();
    (void)enc_->getFirmwareVersion();
}

TEST_F(CiA406Test, InterfaceConfig) {
    enc_->initialize();
    enc_->configureSSI(1000, 25, true, true);
    enc_->configureSSI(500, 13, false, false);
    enc_->configureBiSS(10000000, 25);
    enc_->configureEnDat(8000000);
}

TEST_F(CiA406Test, SDOAccess) {
    enc_->initialize();
    uint32_t data = 0;
    size_t out_size = 0;
    enc_->readObject(0x6000, 0, &data, sizeof(data), &out_size);
    enc_->writeObject(0x6000, 0, &data, sizeof(data));
}

TEST_F(CiA406Test, Update) {
    enc_->initialize();
    enc_->update();
}

TEST_F(CiA406Test, EventCallback) {
    enc_->setEventCallback([](EncoderEvent, uint16_t, uint32_t) {});
    enc_->clearEventCallback();
}

TEST_F(CiA406Test, SlaveInfo) {
    EXPECT_EQ(enc_->getSlaveAddress(), 1u);
    EXPECT_FALSE(enc_->isUsingConfiguredAddress());
}

TEST_F(CiA406Test, PDOMappingBasic) {
    enc_->initialize();
    enc_->applyPDOMapping(PDOMappingPreset::Basic);
    EXPECT_EQ(enc_->getCurrentMapping(), PDOMappingPreset::Basic);
    enc_->update();
}

TEST_F(CiA406Test, PDOMappingWithVelocity) {
    enc_->initialize();
    enc_->applyPDOMapping(PDOMappingPreset::WithVelocity);
    enc_->update();
}

TEST_F(CiA406Test, PDOMappingFull) {
    enc_->initialize();
    enc_->applyPDOMapping(PDOMappingPreset::Full);
    enc_->update();
}

TEST_F(CiA406Test, PDOMappingMultiTurn) {
    enc_->initialize();
    enc_->applyPDOMapping(PDOMappingPreset::MultiTurn);
    enc_->update();
}

TEST_F(CiA406Test, PDOMappingHighSpeed) {
    enc_->initialize();
    enc_->applyPDOMapping(PDOMappingPreset::HighSpeed);
    enc_->update();
}

TEST_F(CiA406Test, PDOMappingDiagnostic) {
    enc_->initialize();
    enc_->applyPDOMapping(PDOMappingPreset::Diagnostic);
    enc_->update();
}

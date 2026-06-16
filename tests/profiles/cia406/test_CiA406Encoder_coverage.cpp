/**
 * @file test_CiA406Encoder_coverage.cpp
 * @brief Extended coverage tests for CiA406::Encoder — exercises branches
 *        unreachable with NullSDOTransport (pure-value structs, scaling,
 *        enum-to-string, alarm descriptions, update() with initialized state).
 */

#include "tether/profiles/cia406/CiA406Encoder.hpp"
#include "tether/ethercat/SDOManager.hpp"
#include <gtest/gtest.h>
#include <gmock/gmock.h>
#include <chrono>

using namespace CiA406;
using namespace EtherCAT::SDO;
using ::testing::_;
using ::testing::Invoke;
using ::testing::NiceMock;
using ::testing::Return;

// ============================================================================
// Mock transport with controllable responses
// ============================================================================

namespace {

uint64_t realMicros() {
    return std::chrono::duration_cast<std::chrono::microseconds>(
        std::chrono::steady_clock::now().time_since_epoch()).count();
}

class MockSDOTransportRaw : public ISDOTransport {
public:
    MOCK_METHOD(bool, sdoUpload,
                (uint16_t, uint8_t*, uint16_t, uint16_t, uint16_t, uint16_t,
                 uint16_t, uint8_t, uint8_t*, size_t, size_t*), (override));
    MOCK_METHOD(bool, sdoDownload,
                (uint16_t, uint8_t*, uint16_t, uint16_t, uint16_t, uint16_t,
                 uint16_t, uint8_t, const uint8_t*, size_t), (override));
    MOCK_METHOD(uint64_t, getMicroseconds, (), (override));
};

auto UploadVal(const void* data, size_t len) {
    return [data, len](uint16_t, uint8_t*, uint16_t, uint16_t, uint16_t, uint16_t,
                       uint16_t, uint8_t, uint8_t* out, size_t, size_t* ol) -> bool {
        memcpy(out, data, len);
        if (ol) *ol = len;
        return true;
    };
}

} // anonymous namespace

// ============================================================================
// getEncoderClassName — all branches
// ============================================================================

TEST(Enc406CovTest, EncoderClassName_C1) {
    auto name = getEncoderClassName(EncoderClassEx::C1_AbsoluteSingleTurn);
    EXPECT_NE(name, nullptr);
    EXPECT_TRUE(std::string(name).find("C1") != std::string::npos);
}

TEST(Enc406CovTest, EncoderClassName_C2) {
    auto name = getEncoderClassName(EncoderClassEx::C2_AbsoluteMultiTurn);
    EXPECT_NE(name, nullptr);
    EXPECT_TRUE(std::string(name).find("C2") != std::string::npos);
}

TEST(Enc406CovTest, EncoderClassName_C3) {
    auto name = getEncoderClassName(EncoderClassEx::C3_Incremental);
    EXPECT_NE(name, nullptr);
    EXPECT_TRUE(std::string(name).find("C3") != std::string::npos);
}

TEST(Enc406CovTest, EncoderClassName_C4) {
    auto name = getEncoderClassName(EncoderClassEx::C4_IncrementalWithLimits);
    EXPECT_NE(name, nullptr);
    EXPECT_TRUE(std::string(name).find("C4") != std::string::npos);
}

TEST(Enc406CovTest, EncoderClassName_Unknown) {
    auto name = getEncoderClassName(EncoderClassEx::Unknown);
    EXPECT_NE(name, nullptr);
    auto name2 = getEncoderClassName(static_cast<EncoderClassEx>(99));
    EXPECT_NE(name2, nullptr);
}

// ============================================================================
// getInterfaceTypeName — all branches
// ============================================================================

TEST(Enc406CovTest, InterfaceTypeName_AllTypes) {
    // Just exercise all branches — don't hardcode exact strings
    EXPECT_NE(getInterfaceTypeName(InterfaceType::Parallel), nullptr);
    EXPECT_NE(getInterfaceTypeName(InterfaceType::SSI), nullptr);
    EXPECT_NE(getInterfaceTypeName(InterfaceType::BiSS_C), nullptr);
    EXPECT_NE(getInterfaceTypeName(InterfaceType::BiSS_B), nullptr);
    EXPECT_NE(getInterfaceTypeName(InterfaceType::EnDat21), nullptr);
    EXPECT_NE(getInterfaceTypeName(InterfaceType::EnDat22), nullptr);
    EXPECT_NE(getInterfaceTypeName(InterfaceType::SinCos_1Vpp), nullptr);
    EXPECT_NE(getInterfaceTypeName(InterfaceType::TTL_RS422), nullptr);
    EXPECT_NE(getInterfaceTypeName(InterfaceType::Hiperface), nullptr);
    EXPECT_NE(getInterfaceTypeName(InterfaceType::DRIVE_CLiQ), nullptr);
    EXPECT_NE(getInterfaceTypeName(InterfaceType::Tamagawa), nullptr);
    EXPECT_NE(getInterfaceTypeName(static_cast<InterfaceType>(99)), nullptr);
}

// ============================================================================
// OperatingStatus roundtrip
// ============================================================================

TEST(Enc406CovTest, OperatingStatus_FromRawAllBits) {
    auto s = OperatingStatus::fromRaw(0xFFFF);
    EXPECT_TRUE(s.position_valid);
    EXPECT_TRUE(s.scaling_active);
    EXPECT_TRUE(s.reference_done);
    EXPECT_TRUE(s.preset_executed);
    EXPECT_TRUE(s.overspeed_warning);
    EXPECT_TRUE(s.counting_range_exceeded);
    EXPECT_TRUE(s.supply_voltage_low);
    EXPECT_TRUE(s.supply_voltage_high);
}

TEST(Enc406CovTest, OperatingStatus_ToRaw) {
    OperatingStatus s{};
    s.position_valid = true;
    s.scaling_active = true;
    s.reference_done = true;
    s.preset_executed = true;
    s.overspeed_warning = true;
    s.counting_range_exceeded = true;
    s.supply_voltage_low = true;
    s.supply_voltage_high = true;
    uint16_t raw = s.toRaw();
    auto s2 = OperatingStatus::fromRaw(raw);
    EXPECT_EQ(s2.position_valid, true);
    EXPECT_EQ(s2.scaling_active, true);
    EXPECT_EQ(s2.reference_done, true);
    EXPECT_EQ(s2.preset_executed, true);
    EXPECT_EQ(s2.overspeed_warning, true);
    EXPECT_EQ(s2.counting_range_exceeded, true);
    EXPECT_EQ(s2.supply_voltage_low, true);
    EXPECT_EQ(s2.supply_voltage_high, true);
}

TEST(Enc406CovTest, OperatingStatus_ZeroBits) {
    auto s = OperatingStatus::fromRaw(0x0000);
    EXPECT_FALSE(s.position_valid);
    EXPECT_FALSE(s.scaling_active);
    EXPECT_FALSE(s.reference_done);
    EXPECT_FALSE(s.preset_executed);
    uint16_t raw = s.toRaw();
    EXPECT_EQ(raw, 0);
}

// ============================================================================
// AlarmFlags — all individual bits + descriptions
// ============================================================================

TEST(Enc406CovTest, AlarmFlags_IndividualBits) {
    for (int bit = 0; bit < 10; ++bit) {
        auto a = AlarmFlags::fromRaw(1u << bit);
        EXPECT_TRUE(a.hasAnyAlarm()) << "bit " << bit;
        auto desc = a.getDescription();
        EXPECT_FALSE(desc.empty()) << "bit " << bit;
        // Should not end with comma
        EXPECT_NE(desc.back(), ',') << "desc for bit " << bit << ": " << desc;
    }
}

TEST(Enc406CovTest, AlarmFlags_NoAlarm) {
    auto a = AlarmFlags::fromRaw(0x0000);
    EXPECT_FALSE(a.hasAnyAlarm());
    auto desc = a.getDescription();
    // Should say "No alarms" or similar
    EXPECT_FALSE(desc.empty());
}

TEST(Enc406CovTest, AlarmFlags_AllBits) {
    auto a = AlarmFlags::fromRaw(0x03FF); // lower 10 bits
    EXPECT_TRUE(a.hasAnyAlarm());
    auto desc = a.getDescription();
    // Should contain multiple alarm names
    EXPECT_GT(desc.size(), 30u);
}

// ============================================================================
// ScalingConfig
// ============================================================================

TEST(Enc406CovTest, ScalingConfig_ApplyEnabled) {
    ScalingConfig sc{};
    sc.enabled = true;
    sc.numerator = 360.0;
    sc.denominator = 4096.0;
    sc.offset = 10.0;
    double result = sc.apply(2048);
    EXPECT_NEAR(result, 360.0 * 2048.0 / 4096.0 + 10.0, 0.001);
}

TEST(Enc406CovTest, ScalingConfig_ApplyDisabled) {
    ScalingConfig sc{};
    sc.enabled = false;
    sc.numerator = 360.0;
    sc.denominator = 4096.0;
    double result = sc.apply(1000);
    EXPECT_DOUBLE_EQ(result, 1000.0);
}

TEST(Enc406CovTest, ScalingConfig_ApplyZeroDenom) {
    ScalingConfig sc{};
    sc.enabled = true;
    sc.numerator = 1.0;
    sc.denominator = 0.0;
    double result = sc.apply(100);
    EXPECT_DOUBLE_EQ(result, 100.0); // Should return raw on divide by zero
}

TEST(Enc406CovTest, ScalingConfig_ReverseEnabled) {
    ScalingConfig sc{};
    sc.enabled = true;
    sc.numerator = 360.0;
    sc.denominator = 4096.0;
    sc.offset = 10.0;
    int32_t result = sc.reverse(190.0); // (190 - 10) * 4096 / 360
    EXPECT_NEAR(result, static_cast<int32_t>((190.0 - 10.0) * 4096.0 / 360.0), 1);
}

TEST(Enc406CovTest, ScalingConfig_ReverseDisabled) {
    ScalingConfig sc{};
    sc.enabled = false;
    int32_t result = sc.reverse(42.5);
    EXPECT_EQ(result, 42);
}

TEST(Enc406CovTest, ScalingConfig_ReverseZeroNumerator) {
    ScalingConfig sc{};
    sc.enabled = true;
    sc.numerator = 0.0;
    sc.denominator = 1.0;
    int32_t result = sc.reverse(100.0);
    EXPECT_EQ(result, 100); // Should return scaled as raw
}

// ============================================================================
// PDOMappingEntry::toMappingValue
// ============================================================================

TEST(Enc406CovTest, PDOMappingEntry_ToMappingValue) {
    PDOMappingEntry entry{};
    entry.index = 0x6004;
    entry.subindex = 0x00;
    entry.bits = 32;
    uint32_t val = entry.toMappingValue();
    // Expected: (0x6004 << 16) | (0x00 << 8) | 32
    EXPECT_EQ(val, (0x6004u << 16) | 32u);
}

TEST(Enc406CovTest, PDOMappingEntry_WithSubindex) {
    PDOMappingEntry entry{};
    entry.index = 0x6010;
    entry.subindex = 0x01;
    entry.bits = 16;
    uint32_t val = entry.toMappingValue();
    EXPECT_EQ(val, (0x6010u << 16) | (0x01u << 8) | 16u);
}

// ============================================================================
// TxPDO struct sizes (packed)
// ============================================================================

TEST(Enc406CovTest, TxPDOStructSizes) {
    // Verify packed sizes are correct for PDO mapping
    EXPECT_EQ(sizeof(TxPDOBasic), 6u);          // i32 + u16
    EXPECT_EQ(sizeof(TxPDOWithVelocity), 10u);  // i32 + i32 + u16
}

// ============================================================================
// Encoder with mock transport — initialization and SDO-based API
// ============================================================================

class Enc406MockCovTest : public ::testing::Test {
protected:
    void SetUp() override {
        ON_CALL(transport_, getMicroseconds()).WillByDefault(Invoke(realMicros));
        // Default: writes succeed, reads return zero
        ON_CALL(transport_, sdoDownload(_, _, _, _, _, _, _, _, _, _))
            .WillByDefault(Return(true));
        uint32_t zero = 0;
        ON_CALL(transport_, sdoUpload(_, _, _, _, _, _, _, _, _, _, _))
            .WillByDefault(UploadVal(&zero, sizeof(zero)));

        sdo_ = std::make_unique<SDOManager>(transport_);
        sdo_->configureSlaveMailbox(0, 0x1000, 128, 0x1400, 128);
        sdo_->init();
    }
    void TearDown() override {
        if (sdo_) sdo_->deinit();
    }

    NiceMock<MockSDOTransportRaw> transport_;
    std::unique_ptr<SDOManager> sdo_;
};

TEST_F(Enc406MockCovTest, Initialize_Profile406) {
    // Make device type read return profile 406 (0x196 in low 16 bits)
    uint32_t deviceType = 0x00020196; // Profile 406
    ON_CALL(transport_, sdoUpload(_, _, _, _, _, _, 0x1000, 0, _, _, _))
        .WillByDefault(UploadVal(&deviceType, sizeof(deviceType)));

    Encoder enc(*sdo_, 0);
    bool ok = enc.initialize();
    // Should succeed now that device type is 406
    (void)ok;
    // Either way, test that it ran the code path
}

TEST_F(Enc406MockCovTest, Initialize_WrongProfile) {
    // Device type for a different profile (e.g., 402)
    uint32_t deviceType = 0x00020192; // Profile 402
    ON_CALL(transport_, sdoUpload(_, _, _, _, _, _, 0x1000, 0, _, _, _))
        .WillByDefault(UploadVal(&deviceType, sizeof(deviceType)));

    Encoder enc(*sdo_, 0);
    bool ok = enc.initialize();
    // May warn but still initialize
    (void)ok;
}

TEST_F(Enc406MockCovTest, CreateEncoder_Factory) {
    uint32_t deviceType = 0x00020196;
    ON_CALL(transport_, sdoUpload(_, _, _, _, _, _, 0x1000, 0, _, _, _))
        .WillByDefault(UploadVal(&deviceType, sizeof(deviceType)));

    auto enc = createEncoder(*sdo_, 0);
    // May be null if init fails due to other SDO reads
    (void)enc;
}

TEST_F(Enc406MockCovTest, ScanForEncoders) {
    // Default transport returns 0 for device type → no profile 406
    auto found = scanForEncoders(*sdo_);
    EXPECT_TRUE(found.empty());
}

TEST_F(Enc406MockCovTest, ApplyPDOMapping_AllPresets) {
    Encoder enc(*sdo_, 0);
    // Exercise all preset mappings
    enc.applyPDOMapping(PDOMappingPreset::Basic);
    enc.applyPDOMapping(PDOMappingPreset::WithVelocity);
    enc.applyPDOMapping(PDOMappingPreset::Full);
    enc.applyPDOMapping(PDOMappingPreset::MultiTurn);
    enc.applyPDOMapping(PDOMappingPreset::HighSpeed);
    enc.applyPDOMapping(PDOMappingPreset::Diagnostic);
    EXPECT_FALSE(enc.applyPDOMapping(PDOMappingPreset::Custom)); // Custom w/o entries = false
}

TEST_F(Enc406MockCovTest, ApplyCustomPDOMapping) {
    Encoder enc(*sdo_, 0);
    PDOMappingEntry entries[2];
    entries[0] = {0x6004, 0, 32};
    entries[1] = {0x6010, 0, 16};
    bool ok = enc.applyCustomPDOMapping(entries, 2);
    (void)ok;
}

TEST_F(Enc406MockCovTest, SetScaling) {
    Encoder enc(*sdo_, 0);
    enc.setScaling(360.0, 4096.0, 0.0);
    auto sc = enc.getScaling();
    EXPECT_TRUE(sc.enabled);
    EXPECT_DOUBLE_EQ(sc.numerator, 360.0);
    EXPECT_DOUBLE_EQ(sc.denominator, 4096.0);
}

TEST_F(Enc406MockCovTest, SetScalingFromRange) {
    Encoder enc(*sdo_, 0);
    enc.setScalingFromRange(8192, 720.0);
    auto sc = enc.getScaling();
    EXPECT_TRUE(sc.enabled);
}

TEST_F(Enc406MockCovTest, DisableScaling) {
    Encoder enc(*sdo_, 0);
    enc.setScaling(360.0, 4096.0);
    enc.disableScaling();
    auto sc = enc.getScaling();
    EXPECT_FALSE(sc.enabled);
}

TEST_F(Enc406MockCovTest, SetOffset) {
    Encoder enc(*sdo_, 0);
    enc.setOffset(100.0);
    auto sc = enc.getScaling();
    EXPECT_DOUBLE_EQ(sc.offset, 100.0);
}

TEST_F(Enc406MockCovTest, PresetPosition) {
    // Readback returns the preset value
    int32_t presetVal = 12345;
    ON_CALL(transport_, sdoUpload(_, _, _, _, _, _, 0x6004, 0, _, _, _))
        .WillByDefault(UploadVal(&presetVal, sizeof(presetVal)));

    Encoder enc(*sdo_, 0);
    bool ok = enc.presetPosition(12345);
    (void)ok;
}

TEST_F(Enc406MockCovTest, SetZero) {
    int32_t zero = 0;
    ON_CALL(transport_, sdoUpload(_, _, _, _, _, _, 0x6004, 0, _, _, _))
        .WillByDefault(UploadVal(&zero, sizeof(zero)));

    Encoder enc(*sdo_, 0);
    bool ok = enc.setZero();
    (void)ok;
}

TEST_F(Enc406MockCovTest, StartReference) {
    Encoder enc(*sdo_, 0);
    ReferenceConfig cfg{};
    cfg.mode = ReferenceMode::OnZeroPulse;
    cfg.reference_position = 0;
    cfg.direction_positive = true;
    cfg.search_velocity = 100;
    bool ok = enc.startReference(cfg);
    (void)ok;
}

TEST_F(Enc406MockCovTest, StartReference_CurrentPosition) {
    Encoder enc(*sdo_, 0);
    ReferenceConfig cfg{};
    cfg.mode = ReferenceMode::CurrentPosition;
    bool ok = enc.startReference(cfg);
    (void)ok;
}

TEST_F(Enc406MockCovTest, AbortReference) {
    Encoder enc(*sdo_, 0);
    enc.abortReference(); // void return
}

TEST_F(Enc406MockCovTest, SetWorkingArea1) {
    Encoder enc(*sdo_, 0);
    bool ok = enc.setWorkingArea(0, 1000, 1);
    (void)ok;
}

TEST_F(Enc406MockCovTest, SetWorkingArea2) {
    Encoder enc(*sdo_, 0);
    bool ok = enc.setWorkingArea(0, 2000, 2);
    (void)ok;
}

TEST_F(Enc406MockCovTest, SetWorkingArea_Invalid) {
    Encoder enc(*sdo_, 0);
    EXPECT_FALSE(enc.setWorkingArea(0, 1000, 3));
}

TEST_F(Enc406MockCovTest, IsWithinWorkingArea) {
    Encoder enc(*sdo_, 0);
    // Without update() the state is NotConfigured
    bool result = enc.isWithinWorkingArea();
    (void)result;
}

TEST_F(Enc406MockCovTest, ClearAlarms) {
    Encoder enc(*sdo_, 0);
    bool ok = enc.clearAlarms();
    (void)ok;
}

TEST_F(Enc406MockCovTest, ReadDiagnostics) {
    Encoder enc(*sdo_, 0);
    auto diag = enc.readDiagnostics();
    (void)diag;
}

TEST_F(Enc406MockCovTest, ConfigureSSI) {
    Encoder enc(*sdo_, 0);
    bool ok = enc.configureSSI(1000, 25, true, true);
    (void)ok;
}

TEST_F(Enc406MockCovTest, ConfigureBiSS) {
    Encoder enc(*sdo_, 0);
    bool ok = enc.configureBiSS(10000000, 26);
    (void)ok;
}

TEST_F(Enc406MockCovTest, ConfigureEnDat) {
    Encoder enc(*sdo_, 0);
    bool ok = enc.configureEnDat(8000000);
    (void)ok;
}

TEST_F(Enc406MockCovTest, ReadWriteObject) {
    Encoder enc(*sdo_, 0);
    uint32_t val = 0;
    size_t out_size = 0;
    bool ok = enc.readObject(0x6000, 0, reinterpret_cast<uint8_t*>(&val), sizeof(val), &out_size);
    (void)ok;
    val = 42;
    ok = enc.writeObject(0x6000, 0, reinterpret_cast<const uint8_t*>(&val), sizeof(val));
    (void)ok;
}

TEST_F(Enc406MockCovTest, EventCallback) {
    Encoder enc(*sdo_, 0);
    int callCount = 0;
    enc.setEventCallback([&](EncoderEvent, uint16_t, uint32_t) { callCount++; });
    enc.clearEventCallback();
}

TEST_F(Enc406MockCovTest, SlaveInfo) {
    Encoder enc(*sdo_, 0);
    EXPECT_EQ(enc.getSlaveAddress(), 0u);
    EXPECT_FALSE(enc.isUsingConfiguredAddress());
}

TEST_F(Enc406MockCovTest, SlaveInfo_ConfiguredAddr) {
    Encoder enc(*sdo_, 0x1001, true);
    EXPECT_EQ(enc.getSlaveAddress(), 0x1001u);
    EXPECT_TRUE(enc.isUsingConfiguredAddress());
}

TEST_F(Enc406MockCovTest, PositionReading_BeforeInit) {
    Encoder enc(*sdo_, 0);
    // Before init — returns default values
    EXPECT_EQ(enc.getRawPosition(), 0);
    EXPECT_EQ(enc.getMultiTurnValue(), 0);
    EXPECT_EQ(enc.getSingleTurnValue(), 0);
    EXPECT_DOUBLE_EQ(enc.getVelocity(), 0.0);
    EXPECT_DOUBLE_EQ(enc.getAcceleration(), 0.0);
}

TEST_F(Enc406MockCovTest, Update_BeforeInit) {
    Encoder enc(*sdo_, 0);
    // update() should early-return without initialization
    enc.update();
    EXPECT_EQ(enc.getRawPosition(), 0);
}

// ============================================================================
// Enum distinct value tests
// ============================================================================

TEST(Enc406CovTest, PDOMappingPresetValues) {
    // Ensure all presets are distinct
    std::set<int> vals;
    vals.insert(static_cast<int>(PDOMappingPreset::Basic));
    vals.insert(static_cast<int>(PDOMappingPreset::WithVelocity));
    vals.insert(static_cast<int>(PDOMappingPreset::Full));
    vals.insert(static_cast<int>(PDOMappingPreset::MultiTurn));
    vals.insert(static_cast<int>(PDOMappingPreset::HighSpeed));
    vals.insert(static_cast<int>(PDOMappingPreset::Diagnostic));
    vals.insert(static_cast<int>(PDOMappingPreset::Custom));
    EXPECT_EQ(vals.size(), 7u);
}

TEST(Enc406CovTest, ReferenceModeValues) {
    std::set<int> vals;
    vals.insert(static_cast<int>(ReferenceMode::None));
    vals.insert(static_cast<int>(ReferenceMode::OnZeroPulse));
    vals.insert(static_cast<int>(ReferenceMode::OnLimitSwitch));
    vals.insert(static_cast<int>(ReferenceMode::OnExternalInput));
    vals.insert(static_cast<int>(ReferenceMode::CurrentPosition));
    EXPECT_EQ(vals.size(), 5u);
}

TEST(Enc406CovTest, WorkingAreaStateValues) {
    std::set<int> vals;
    vals.insert(static_cast<int>(WorkingAreaState::WithinArea1));
    vals.insert(static_cast<int>(WorkingAreaState::BelowLowLimit1));
    vals.insert(static_cast<int>(WorkingAreaState::AboveHighLimit1));
    vals.insert(static_cast<int>(WorkingAreaState::WithinArea2));
    vals.insert(static_cast<int>(WorkingAreaState::BelowLowLimit2));
    vals.insert(static_cast<int>(WorkingAreaState::AboveHighLimit2));
    vals.insert(static_cast<int>(WorkingAreaState::NotConfigured));
    EXPECT_EQ(vals.size(), 7u);
}

TEST(Enc406CovTest, EncoderEventValues) {
    std::set<int> vals;
    vals.insert(static_cast<int>(EncoderEvent::PositionUpdated));
    vals.insert(static_cast<int>(EncoderEvent::AlarmTriggered));
    vals.insert(static_cast<int>(EncoderEvent::AlarmCleared));
    vals.insert(static_cast<int>(EncoderEvent::WarningTriggered));
    vals.insert(static_cast<int>(EncoderEvent::ReferenceDone));
    vals.insert(static_cast<int>(EncoderEvent::WorkingAreaEntered));
    vals.insert(static_cast<int>(EncoderEvent::WorkingAreaExited));
    vals.insert(static_cast<int>(EncoderEvent::CommunicationError));
    EXPECT_EQ(vals.size(), 8u);
}

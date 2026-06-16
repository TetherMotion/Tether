/**
 * @file test_CiA430Meter.cpp
 * @brief Comprehensive tests for CiA 430 Energy Meter
 */
#include <gtest/gtest.h>
#include "tether/profiles/cia430/CiA430Meter.hpp"
#include "tether/ethercat/SDOManager.hpp"

using namespace CiA430;

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
// VoltageReading conversions
// ============================================================================

TEST(CiA430Voltage, Conversions) {
    VoltageReading v{};
    v.l1n = voltsToRaw(230.5f);
    EXPECT_NEAR(v.getL1N_V(), 230.5f, 0.02f);
    v.l2n = voltsToRaw(231.0f);
    EXPECT_NEAR(v.getL2N_V(), 231.0f, 0.02f);
    v.l3n = voltsToRaw(229.5f);
    EXPECT_NEAR(v.getL3N_V(), 229.5f, 0.02f);
    v.l1l2 = voltsToRaw(400.0f);
    EXPECT_NEAR(v.getL1L2_V(), 400.0f, 0.1f);
    v.l2l3 = voltsToRaw(399.0f);
    v.l3l1 = voltsToRaw(401.0f);
    v.avg_ln = voltsToRaw(230.3f);
    v.avg_ll = voltsToRaw(400.0f);
    // avg_ln and avg_ll don't have struct getters;
    // average voltages are accessed via EnergyMeter methods
    EXPECT_NEAR(rawToVolts(v.avg_ln), 230.3f, 0.02f);
    EXPECT_NEAR(rawToVolts(v.avg_ll), 400.0f, 0.1f);
}

// ============================================================================
// CurrentReading conversions
// ============================================================================

TEST(CiA430Current, Conversions) {
    CurrentReading c{};
    c.l1 = ampsToRaw(12.345f);
    EXPECT_NEAR(c.getL1_A(), 12.345f, 0.002f);
    c.l2 = ampsToRaw(12.0f);
    c.l3 = ampsToRaw(11.5f);
    c.neutral = ampsToRaw(0.5f);
    c.avg = ampsToRaw(11.95f);
    c.total = ampsToRaw(35.845f);
    EXPECT_NEAR(c.getL2_A(), 12.0f, 0.002f);
    EXPECT_NEAR(c.getL3_A(), 11.5f, 0.002f);
    EXPECT_NEAR(c.getNeutral_A(), 0.5f, 0.002f);
}

// ============================================================================
// PowerReading conversions
// ============================================================================

TEST(CiA430Power, Conversions) {
    PowerReading p{};
    p.active_total = wattsToRaw(1234.5f);
    EXPECT_NEAR(p.getActiveTotal_W(), 1234.5f, 0.2f);
    EXPECT_NEAR(p.getActiveTotal_kW(), 1.2345f, 0.001f);
}

// ============================================================================
// EnergyReading conversions
// ============================================================================

TEST(CiA430Energy, Conversions) {
    EnergyReading e{};
    e.active_import = kWhToRaw(12.345f);
    EXPECT_NEAR(e.getActiveImport_kWh(), 12.345f, 0.002f);
    e.active_export = kWhToRaw(1.0f);
    EXPECT_NEAR(e.getActiveExport_kWh(), 1.0f, 0.002f);
}

// ============================================================================
// PowerQuality conversions
// ============================================================================

TEST(CiA430Quality, Conversions) {
    PowerQuality q{};
    q.pf_total = pfToRaw(0.95f);
    EXPECT_NEAR(q.getPowerFactorTotal(), 0.95f, 0.002f);
    q.frequency = hzToRaw(50.0f);
    EXPECT_NEAR(q.getFrequency_Hz(), 50.0f, 0.01f);
}

// ============================================================================
// MeterState helpers
// ============================================================================

TEST(CiA430MeterState, Default) {
    MeterState ms{};
    EXPECT_FALSE(ms.isReady());
    EXPECT_FALSE(ms.isDataValid());
    EXPECT_FALSE(ms.hasAlarm());
    EXPECT_FALSE(ms.hasFault());
}

TEST(CiA430MeterState, StatusBits) {
    MeterState ms{};
    ms.statusword = StatuswordBits::Ready | StatuswordBits::DataValid;
    EXPECT_TRUE(ms.isReady());
    EXPECT_TRUE(ms.isDataValid());
    EXPECT_FALSE(ms.hasFault());
}

TEST(CiA430MeterState, Exporting) {
    MeterState ms{};
    ms.statusword = StatuswordBits::EnergyDirection;
    EXPECT_TRUE(ms.isExporting());
}

TEST(CiA430MeterState, PhaseBits) {
    MeterState ms{};
    ms.statusword = StatuswordBits::PhaseL1OK | StatuswordBits::PhaseL2OK | StatuswordBits::PhaseL3OK;
    EXPECT_TRUE(ms.isPhaseL1OK());
    EXPECT_TRUE(ms.isPhaseL2OK());
    EXPECT_TRUE(ms.isPhaseL3OK());
}

// ============================================================================
// Raw conversion round-trips
// ============================================================================

TEST(CiA430Conversions, VoltsRoundTrip) {
    EXPECT_NEAR(rawToVolts(voltsToRaw(123.45f)), 123.45f, 0.01f);
}

TEST(CiA430Conversions, AmpsRoundTrip) {
    EXPECT_NEAR(rawToAmps(ampsToRaw(5.678f)), 5.678f, 0.001f);
}

TEST(CiA430Conversions, WattsRoundTrip) {
    EXPECT_NEAR(rawToWatts(wattsToRaw(2500.0f)), 2500.0f, 0.1f);
}

TEST(CiA430Conversions, KWhRoundTrip) {
    EXPECT_NEAR(rawToKWh(kWhToRaw(100.5f)), 100.5f, 0.001f);
}

TEST(CiA430Conversions, PFRoundTrip) {
    EXPECT_NEAR(rawToPF(pfToRaw(0.85f)), 0.85f, 0.001f);
}

TEST(CiA430Conversions, HzRoundTrip) {
    EXPECT_NEAR(rawToHz(hzToRaw(60.0f)), 60.0f, 0.01f);
}

// ============================================================================
// EnergyMeter fixture
// ============================================================================

class CiA430Test : public ::testing::Test {
protected:
    void SetUp() override {
        transport_ = std::make_unique<NullSDOTransport>();
        sdo_ = std::make_unique<EtherCAT::SDO::SDOManager>(*transport_);
        sdo_->init();
        meter_ = std::make_unique<EnergyMeter>(*sdo_, 1);
    }
    void TearDown() override {
        meter_.reset();
        sdo_->deinit();
    }
    std::unique_ptr<NullSDOTransport> transport_;
    std::unique_ptr<EtherCAT::SDO::SDOManager> sdo_;
    std::unique_ptr<EnergyMeter> meter_;
};

TEST_F(CiA430Test, Construction) {
    EnergyMeter m2(*sdo_, 0x100, true);
    EXPECT_FALSE(m2.isInitialized());
}

TEST_F(CiA430Test, Initialize) {
    // initialize() may fail with NullSDOTransport
    (void)meter_->initialize();
}

TEST_F(CiA430Test, GetSpec) {
    meter_->initialize();
    auto spec = meter_->getSpec();
    (void)spec.meter_type;
    (void)spec.num_phases;
}

TEST_F(CiA430Test, PDOMappingAll) {
    meter_->initialize();
    meter_->applyPDOMapping(PDOMappingPreset::Basic);
    meter_->applyPDOMapping(PDOMappingPreset::Extended);
    meter_->applyPDOMapping(PDOMappingPreset::Full);
    meter_->applyPDOMapping(PDOMappingPreset::Custom);
}

TEST_F(CiA430Test, VoltageReadings) {
    meter_->initialize();
    EXPECT_NEAR(meter_->getVoltageL1N(), 0.0f, 1e-3f);
    EXPECT_NEAR(meter_->getVoltageL2N(), 0.0f, 1e-3f);
    EXPECT_NEAR(meter_->getVoltageL3N(), 0.0f, 1e-3f);
    EXPECT_NEAR(meter_->getVoltageL1L2(), 0.0f, 1e-3f);
    EXPECT_NEAR(meter_->getVoltageL2L3(), 0.0f, 1e-3f);
    EXPECT_NEAR(meter_->getVoltageL3L1(), 0.0f, 1e-3f);
    EXPECT_NEAR(meter_->getVoltageAvgLN(), 0.0f, 1e-3f);
    EXPECT_NEAR(meter_->getVoltageAvgLL(), 0.0f, 1e-3f);
    auto v = meter_->getVoltage();
    (void)v;
}

TEST_F(CiA430Test, CurrentReadings) {
    meter_->initialize();
    EXPECT_NEAR(meter_->getCurrentL1(), 0.0f, 1e-3f);
    EXPECT_NEAR(meter_->getCurrentL2(), 0.0f, 1e-3f);
    EXPECT_NEAR(meter_->getCurrentL3(), 0.0f, 1e-3f);
    EXPECT_NEAR(meter_->getCurrentNeutral(), 0.0f, 1e-3f);
    EXPECT_NEAR(meter_->getCurrentAvg(), 0.0f, 1e-3f);
    auto c = meter_->getCurrent();
    (void)c;
}

TEST_F(CiA430Test, PowerReadings) {
    meter_->initialize();
    EXPECT_NEAR(meter_->getActivePowerTotal(), 0.0f, 1e-3f);
    EXPECT_NEAR(meter_->getActivePowerL1(), 0.0f, 1e-3f);
    EXPECT_NEAR(meter_->getActivePowerL2(), 0.0f, 1e-3f);
    EXPECT_NEAR(meter_->getActivePowerL3(), 0.0f, 1e-3f);
    EXPECT_NEAR(meter_->getReactivePowerTotal(), 0.0f, 1e-3f);
    EXPECT_NEAR(meter_->getApparentPowerTotal(), 0.0f, 1e-3f);
    auto p = meter_->getPower();
    (void)p;
}

TEST_F(CiA430Test, EnergyReadings) {
    meter_->initialize();
    EXPECT_NEAR(meter_->getActiveEnergyImport(), 0.0f, 1e-3f);
    EXPECT_NEAR(meter_->getActiveEnergyExport(), 0.0f, 1e-3f);
    EXPECT_NEAR(meter_->getNetActiveEnergy(), 0.0f, 1e-3f);
    EXPECT_NEAR(meter_->getReactiveEnergyImport(), 0.0f, 1e-3f);
    EXPECT_NEAR(meter_->getReactiveEnergyExport(), 0.0f, 1e-3f);
    auto e = meter_->getEnergy();
    (void)e;
    meter_->resetEnergyCounters();
    meter_->freezeEnergyCounters();
}

TEST_F(CiA430Test, PowerQuality) {
    meter_->initialize();
    EXPECT_NEAR(meter_->getPowerFactorTotal(), 0.0f, 1e-3f);
    EXPECT_NEAR(meter_->getPowerFactorL1(), 0.0f, 1e-3f);
    EXPECT_NEAR(meter_->getPowerFactorL2(), 0.0f, 1e-3f);
    EXPECT_NEAR(meter_->getPowerFactorL3(), 0.0f, 1e-3f);
    EXPECT_NEAR(meter_->getFrequency(), 0.0f, 1e-3f);
    EXPECT_NEAR(meter_->getTHDV_L1(), 0.0f, 1e-3f);
    EXPECT_NEAR(meter_->getTHDV_L2(), 0.0f, 1e-3f);
    EXPECT_NEAR(meter_->getTHDV_L3(), 0.0f, 1e-3f);
    EXPECT_NEAR(meter_->getTHDI_L1(), 0.0f, 1e-3f);
    EXPECT_NEAR(meter_->getTHDI_L2(), 0.0f, 1e-3f);
    EXPECT_NEAR(meter_->getTHDI_L3(), 0.0f, 1e-3f);
    EXPECT_NEAR(meter_->getVoltageUnbalance(), 0.0f, 1e-3f);
    EXPECT_NEAR(meter_->getCurrentUnbalance(), 0.0f, 1e-3f);
    (void)meter_->getPhaseSequence();
    auto q = meter_->getPowerQuality();
    (void)q;
}

TEST_F(CiA430Test, Harmonics) {
    meter_->initialize();
    float buf[16] = {};
    meter_->getVoltageHarmonics(0, buf, 16);
    meter_->getCurrentHarmonics(0, buf, 16);
    meter_->getVoltageHarmonics(1, buf, 16);
    meter_->getVoltageHarmonics(2, buf, 16);
}

TEST_F(CiA430Test, Demand) {
    meter_->initialize();
    auto d = meter_->getDemandData();
    (void)d;
    meter_->setDemandPeriod(15, 4);
    meter_->resetDemandValues();
    meter_->resetMaxDemand();
}

TEST_F(CiA430Test, Tariff) {
    meter_->initialize();
    EXPECT_EQ(meter_->getActiveTariff(), 0u);
    meter_->setActiveTariff(1);
    EXPECT_NEAR(meter_->getTariffEnergy(0), 0.0f, 1e-3f);
}

TEST_F(CiA430Test, Alarms) {
    meter_->initialize();
    AlarmThresholds at{};
    at.voltage_high = 25000;
    at.voltage_low = 20000;
    meter_->setAlarmThresholds(at);
    // SDO may fail, so thresholds may keep defaults
    auto got = meter_->getAlarmThresholds();
    (void)got;
    meter_->enableAlarms(0xFFFFFFFF);
    EXPECT_EQ(meter_->getAlarmStatus(), 0u);
    EXPECT_FALSE(meter_->isAlarmActive(AlarmBits::VoltageHighL1));
    meter_->resetAlarms();
}

TEST_F(CiA430Test, Diagnostics) {
    meter_->initialize();
    auto st = meter_->getState();
    EXPECT_FALSE(st.hasFault());
    EXPECT_EQ(meter_->getFaultCode(), 0u);
    (void)meter_->getOperatingHours();
    (void)meter_->getTemperature();
    auto diag = meter_->getDiagnostics();
    EXPECT_FALSE(diag.empty());
}

TEST_F(CiA430Test, Callbacks) {
    meter_->setAlarmCallback([](uint32_t) {});
    meter_->setEnergyCallback([](const EnergyReading&) {});
    meter_->setDataCallback([](const MeterState&) {});
}

TEST_F(CiA430Test, PDOProcess) {
    meter_->initialize();
    uint8_t txbuf[256] = {};
    meter_->processTxPDO(txbuf, sizeof(txbuf));
    uint8_t rxbuf[256] = {};
    meter_->prepareRxPDO(rxbuf, sizeof(rxbuf));
}

TEST_F(CiA430Test, Update) {
    meter_->initialize();
    meter_->update();
}

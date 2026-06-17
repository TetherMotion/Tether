/**
 * @file CiA430Meter.cpp
 * @brief CiA 430 Energy Meter Controller Implementation
 */

#include "profiles/cia430/CiA430Meter.hpp"
#include "tether/ethercat/CoEManager.hpp"

#include <cstring>

namespace CiA430 {

// ============================================================================
// Constructor / Destructor
// ============================================================================

EnergyMeter::EnergyMeter(EtherCAT::CoE::CoEManager& coe)
    : m_coe(coe)
    , initialized_(false)
    , spec_{}
    , state_{}
    , prev_statusword_(0)
    , prev_alarm_status_(0)
    , controlword_(0)
    , current_mapping_(PDOMappingPreset::Basic)
    , alarm_callback_(nullptr)
    , energy_callback_(nullptr)
    , data_callback_(nullptr)
{
}

EnergyMeter::~EnergyMeter() = default;

// ============================================================================
// Initialization
// ============================================================================

bool EnergyMeter::initialize()
{
    if (!readSpec()) {
        return false;
    }
    
    initialized_ = true;
    return true;
}

bool EnergyMeter::readSpec()
{
    // Read meter type
    if (!readSDO(MeterIdentification::MeterType, 0, &spec_.meter_type, 1)) {
        return false;
    }
    
    // Determine number of phases from meter type
    switch (spec_.meter_type) {
        case MeterType::SinglePhase:
            spec_.num_phases = 1;
            break;
        case MeterType::ThreePhaseWye:
        case MeterType::ThreePhaseDelta:
            spec_.num_phases = 3;
            break;
        default:
            spec_.num_phases = 3;
            break;
    }
    
    // Read rated values
    readSDO(MeterIdentification::RatedVoltage, 0, &spec_.rated_voltage, 2);
    readSDO(MeterIdentification::RatedCurrent, 0, &spec_.rated_current, 2);
    readSDO(MeterIdentification::RatedFrequency, 0, &spec_.rated_frequency, 2);
    readSDO(MeterIdentification::AccuracyClass, 0, &spec_.accuracy_class, 1);
    readSDO(MeterIdentification::CTRatio, 0, &spec_.ct_ratio, 2);
    readSDO(MeterIdentification::VTRatio, 0, &spec_.vt_ratio, 2);
    
    return true;
}

// ============================================================================
// PDO Configuration
// ============================================================================

bool EnergyMeter::applyPDOMapping(PDOMappingPreset preset)
{
    current_mapping_ = preset;
    return true;  // PDO mapping would be applied via SDO writes in real implementation
}

// ============================================================================
// Cyclic Update
// ============================================================================

void EnergyMeter::processTxPDO(const uint8_t* data, size_t len)
{
    if (!initialized_ || !data) return;
    
    switch (current_mapping_) {
        case PDOMappingPreset::Basic:
            processBasicPDO(data, len);
            break;
        case PDOMappingPreset::Extended:
            processExtendedPDO(data, len);
            break;
        case PDOMappingPreset::Full:
            processFullPDO(data, len);
            break;
        default:
            processBasicPDO(data, len);
            break;
    }
    
    checkStateChanges();
    
    if (data_callback_) {
        data_callback_(state_);
    }
}

void EnergyMeter::processBasicPDO(const uint8_t* data, size_t len)
{
    if (len < sizeof(BasicPDOMapping)) return;
    
    const auto* pdo = reinterpret_cast<const BasicPDOMapping*>(data);
    
    state_.statusword = pdo->statusword;
    state_.power.active_total = pdo->active_power_total;
    state_.power.reactive_total = pdo->reactive_power_total;
    state_.energy.active_import = pdo->energy_import;
}

void EnergyMeter::processExtendedPDO(const uint8_t* data, size_t len)
{
    if (len < sizeof(ExtendedPDOMapping)) return;
    
    const auto* pdo = reinterpret_cast<const ExtendedPDOMapping*>(data);
    
    // Include basic fields
    state_.statusword = pdo->statusword;
    state_.voltage.l1n = pdo->voltage_l1n;
    state_.voltage.l2n = pdo->voltage_l2n;
    state_.voltage.l3n = pdo->voltage_l3n;
    state_.current.l1 = pdo->current_l1;
    state_.current.l2 = pdo->current_l2;
    state_.current.l3 = pdo->current_l3;
    
    // Power
    state_.power.active_total = pdo->active_power_total;
    state_.power.reactive_total = pdo->reactive_power_total;
    
    // Power quality
    state_.quality.pf_total = pdo->power_factor;
    state_.quality.frequency = pdo->frequency;
}

void EnergyMeter::processFullPDO(const uint8_t* data, size_t len)
{
    if (len < sizeof(FullPDOMapping)) return;
    
    const auto* pdo = reinterpret_cast<const FullPDOMapping*>(data);
    
    state_.statusword = pdo->statusword;
    
    // All voltage readings
    state_.voltage.l1n = pdo->voltage_l1n;
    state_.voltage.l2n = pdo->voltage_l2n;
    state_.voltage.l3n = pdo->voltage_l3n;

    // All current readings
    state_.current.l1 = pdo->current_l1;
    state_.current.l2 = pdo->current_l2;
    state_.current.l3 = pdo->current_l3;
    
    // All power readings
    state_.power.active_l1 = pdo->active_power_l1;
    state_.power.active_l2 = pdo->active_power_l2;
    state_.power.active_l3 = pdo->active_power_l3;
    state_.power.active_total = pdo->active_power_total;
    state_.power.reactive_total = pdo->reactive_power_total;
    state_.power.apparent_total = pdo->apparent_power_total;
    
    // Power quality
    state_.quality.pf_total = pdo->power_factor_total;
    state_.quality.frequency = pdo->frequency;
    
    // Energy - all counters
    state_.energy.active_import = pdo->energy_import;
    state_.energy.active_export = pdo->energy_export;
    
    state_.alarm_status = pdo->alarm_status;
}

size_t EnergyMeter::prepareRxPDO(uint8_t* data, size_t max_len)
{
    if (max_len < 2) return 0;
    
    // RxPDO contains controlword
    memcpy(data, &controlword_, 2);
    return 2;
}

void EnergyMeter::update()
{
    // Periodic state refresh if needed
    checkStateChanges();
}

void EnergyMeter::checkStateChanges()
{
    // Check for alarm changes
    if (state_.alarm_status != prev_alarm_status_) {
        if (alarm_callback_) {
            alarm_callback_(state_.alarm_status);
        }
        prev_alarm_status_ = state_.alarm_status;
    }
    
    prev_statusword_ = state_.statusword;
}

// ============================================================================
// Voltage Reading
// ============================================================================

float EnergyMeter::getVoltageL1N() const
{
    return rawToVolts(state_.voltage.l1n);
}

float EnergyMeter::getVoltageL2N() const
{
    return rawToVolts(state_.voltage.l2n);
}

float EnergyMeter::getVoltageL3N() const
{
    return rawToVolts(state_.voltage.l3n);
}

float EnergyMeter::getVoltageL1L2() const
{
    return rawToVolts(state_.voltage.l1l2);
}

float EnergyMeter::getVoltageL2L3() const
{
    return rawToVolts(state_.voltage.l2l3);
}

float EnergyMeter::getVoltageL3L1() const
{
    return rawToVolts(state_.voltage.l3l1);
}

float EnergyMeter::getVoltageAvgLN() const
{
    return rawToVolts(state_.voltage.avg_ln);
}

float EnergyMeter::getVoltageAvgLL() const
{
    return rawToVolts(state_.voltage.avg_ll);
}

// ============================================================================
// Current Reading
// ============================================================================

float EnergyMeter::getCurrentL1() const
{
    return rawToAmps(state_.current.l1);
}

float EnergyMeter::getCurrentL2() const
{
    return rawToAmps(state_.current.l2);
}

float EnergyMeter::getCurrentL3() const
{
    return rawToAmps(state_.current.l3);
}

float EnergyMeter::getCurrentNeutral() const
{
    return rawToAmps(state_.current.neutral);
}

float EnergyMeter::getCurrentAvg() const
{
    return rawToAmps(state_.current.avg);
}

// ============================================================================
// Power Reading
// ============================================================================

float EnergyMeter::getActivePowerTotal() const
{
    return rawToWatts(state_.power.active_total);
}

float EnergyMeter::getReactivePowerTotal() const
{
    return rawToWatts(state_.power.reactive_total);
}

float EnergyMeter::getApparentPowerTotal() const
{
    return rawToWatts(state_.power.apparent_total);
}

float EnergyMeter::getActivePowerL1() const
{
    return rawToWatts(state_.power.active_l1);
}

float EnergyMeter::getActivePowerL2() const
{
    return rawToWatts(state_.power.active_l2);
}

float EnergyMeter::getActivePowerL3() const
{
    return rawToWatts(state_.power.active_l3);
}

// ============================================================================
// Energy Reading
// ============================================================================

float EnergyMeter::getActiveEnergyImport() const
{
    return rawToKWh(state_.energy.active_import);
}

float EnergyMeter::getActiveEnergyExport() const
{
    return rawToKWh(state_.energy.active_export);
}

float EnergyMeter::getNetActiveEnergy() const
{
    return rawToKWh(state_.energy.active_import - state_.energy.active_export);
}

float EnergyMeter::getReactiveEnergyImport() const
{
    return rawToKWh(state_.energy.reactive_import);
}

float EnergyMeter::getReactiveEnergyExport() const
{
    return rawToKWh(state_.energy.reactive_export);
}

bool EnergyMeter::resetEnergyCounters()
{
    uint8_t cmd = ControlwordBits::ResetEnergy;
    return writeSDO(0x6000, 0, &cmd, 1);
}

bool EnergyMeter::freezeEnergyCounters()
{
    uint8_t cmd = ControlwordBits::FreezeEnergy;
    return writeSDO(0x6000, 0, &cmd, 1);
}

// ============================================================================
// Power Quality
// ============================================================================

float EnergyMeter::getPowerFactorTotal() const
{
    return rawToPF(state_.quality.pf_total);
}

float EnergyMeter::getPowerFactorL1() const
{
    return rawToPF(state_.quality.pf_l1);
}

float EnergyMeter::getPowerFactorL2() const
{
    return rawToPF(state_.quality.pf_l2);
}

float EnergyMeter::getPowerFactorL3() const
{
    return rawToPF(state_.quality.pf_l3);
}

float EnergyMeter::getFrequency() const
{
    return rawToHz(state_.quality.frequency);
}

float EnergyMeter::getTHDV_L1() const
{
    return rawToTHD(state_.quality.thd_v_l1);
}

float EnergyMeter::getTHDV_L2() const
{
    return rawToTHD(state_.quality.thd_v_l2);
}

float EnergyMeter::getTHDV_L3() const
{
    return rawToTHD(state_.quality.thd_v_l3);
}

float EnergyMeter::getTHDI_L1() const
{
    return rawToTHD(state_.quality.thd_i_l1);
}

float EnergyMeter::getTHDI_L2() const
{
    return rawToTHD(state_.quality.thd_i_l2);
}

float EnergyMeter::getTHDI_L3() const
{
    return rawToTHD(state_.quality.thd_i_l3);
}

float EnergyMeter::getVoltageUnbalance() const
{
    return state_.quality.voltage_unbalance / 10.0f;
}

float EnergyMeter::getCurrentUnbalance() const
{
    return state_.quality.current_unbalance / 10.0f;
}

uint8_t EnergyMeter::getPhaseSequence() const
{
    return state_.quality.phase_sequence;
}

// ============================================================================
// Harmonics
// ============================================================================

bool EnergyMeter::getVoltageHarmonics(uint8_t phase, float* harmonics, size_t count)
{
    if (!harmonics || count == 0 || phase > 3) return false;
    
    uint16_t base_index = Harmonics::THDVoltage;
    if (phase > 0) {
        base_index = Harmonics::HarmonicVoltage;
    }
    
    for (size_t i = 0; i < count && i < 50; ++i) {
        uint16_t raw = 0;
        if (readSDO(base_index, static_cast<uint8_t>(i + 1), &raw, 2)) {
            harmonics[i] = raw / 10.0f;  // 0.1%
        } else {
            harmonics[i] = 0.0f;
        }
    }
    
    return true;
}

bool EnergyMeter::getCurrentHarmonics(uint8_t phase, float* harmonics, size_t count)
{
    if (!harmonics || count == 0 || phase > 3) return false;
    
    uint16_t base_index = Harmonics::THDCurrent;
    if (phase > 0) {
        base_index = Harmonics::HarmonicCurrent;
    }
    
    for (size_t i = 0; i < count && i < 50; ++i) {
        uint16_t raw = 0;
        if (readSDO(base_index, static_cast<uint8_t>(i + 1), &raw, 2)) {
            harmonics[i] = raw / 10.0f;  // 0.1%
        } else {
            harmonics[i] = 0.0f;
        }
    }
    
    return true;
}

// ============================================================================
// Demand
// ============================================================================

DemandData EnergyMeter::getDemandData()
{
    DemandData demand;
    
    readSDO(Demand::ActivePower, 0, &demand.active_power, 4);
    readSDO(Demand::ReactivePower, 0, &demand.reactive_power, 4);
    readSDO(Demand::ApparentPower, 0, &demand.apparent_power, 4);
    readSDO(Demand::Current, 0, &demand.current, 4);
    readSDO(Demand::MaxActivePower, 0, &demand.max_active_power, 4);
    readSDO(Demand::MaxReactivePower, 0, &demand.max_reactive_power, 4);
    readSDO(Demand::MaxCurrent, 0, &demand.max_current, 4);
    readSDO(Demand::Period, 0, &demand.period, 2);
    readSDO(Demand::Subperiods, 0, &demand.subperiods, 1);
    
    return demand;
}

bool EnergyMeter::setDemandPeriod(uint16_t period_minutes, uint8_t subperiods)
{
    if (!writeSDO(Demand::Period, 0, &period_minutes, 2)) {
        return false;
    }
    return writeSDO(Demand::Subperiods, 0, &subperiods, 1);
}

bool EnergyMeter::resetDemandValues()
{
    uint8_t reset = 1;
    return writeSDO(Demand::Reset, 0, &reset, 1);
}

bool EnergyMeter::resetMaxDemand()
{
    uint8_t reset = 2;  // Reset max only
    return writeSDO(Demand::Reset, 0, &reset, 1);
}

// ============================================================================
// Tariff
// ============================================================================

bool EnergyMeter::setActiveTariff(uint8_t tariff)
{
    if (tariff > 4) return false;
    return writeSDO(Tariff::ActiveTariff, 0, &tariff, 1);
}

float EnergyMeter::getTariffEnergy(uint8_t tariff) const
{
    if (tariff > 4) return 0.0f;
    return rawToKWh(state_.energy.tariff_energy[tariff]);
}

// ============================================================================
// Alarms
// ============================================================================

bool EnergyMeter::setAlarmThresholds(const AlarmThresholds& thresholds)
{
    bool ok = true;
    ok &= writeSDO(Alarms::VoltageHighThreshold, 0, &thresholds.voltage_high, 4);
    ok &= writeSDO(Alarms::VoltageLowThreshold, 0, &thresholds.voltage_low, 4);
    ok &= writeSDO(Alarms::CurrentHighThreshold, 0, &thresholds.current_high, 4);
    ok &= writeSDO(Alarms::PowerFactorLowThreshold, 0, &thresholds.power_factor_low, 2);
    ok &= writeSDO(Alarms::FrequencyHighThreshold, 0, &thresholds.frequency_high, 2);
    ok &= writeSDO(Alarms::FrequencyLowThreshold, 0, &thresholds.frequency_low, 2);
    ok &= writeSDO(Alarms::THDThreshold, 0, &thresholds.thd_threshold, 2);
    return ok;
}

AlarmThresholds EnergyMeter::getAlarmThresholds()
{
    AlarmThresholds thresholds;
    
    readSDO(Alarms::VoltageHighThreshold, 0, &thresholds.voltage_high, 4);
    readSDO(Alarms::VoltageLowThreshold, 0, &thresholds.voltage_low, 4);
    readSDO(Alarms::CurrentHighThreshold, 0, &thresholds.current_high, 4);
    readSDO(Alarms::PowerFactorLowThreshold, 0, &thresholds.power_factor_low, 2);
    readSDO(Alarms::FrequencyHighThreshold, 0, &thresholds.frequency_high, 2);
    readSDO(Alarms::FrequencyLowThreshold, 0, &thresholds.frequency_low, 2);
    readSDO(Alarms::THDThreshold, 0, &thresholds.thd_threshold, 2);
    
    return thresholds;
}

bool EnergyMeter::enableAlarms(uint32_t alarm_mask)
{
    return writeSDO(Alarms::Enable, 0, &alarm_mask, 4);
}

bool EnergyMeter::isAlarmActive(uint32_t alarm_bit) const
{
    return (state_.alarm_status & alarm_bit) != 0;
}

bool EnergyMeter::resetAlarms()
{
    uint8_t reset = 1;
    return writeSDO(Alarms::Reset, 0, &reset, 1);
}

// ============================================================================
// Diagnostics
// ============================================================================

uint32_t EnergyMeter::getOperatingHours()
{
    uint32_t hours = 0;
    readSDO(Object::OperatingHours, 0, &hours, 4);
    return hours;
}

float EnergyMeter::getTemperature()
{
    int16_t temp = 0;
    readSDO(Object::Temperature, 0, &temp, 2);
    return temp / 10.0f;  // 0.1°C
}

std::string EnergyMeter::getDiagnostics() const
{
    std::string diag;
    
    diag += "Energy Meter Diagnostics:\n";
    diag += "  Type: " + std::to_string(spec_.meter_type) + " (" +
            std::to_string(spec_.num_phases) + "-phase)\n";
    diag += "  Status: 0x" + std::to_string(state_.statusword) + "\n";
    diag += "  Ready: " + std::string(state_.isReady() ? "Yes" : "No") + "\n";
    diag += "  Data Valid: " + std::string(state_.isDataValid() ? "Yes" : "No") + "\n";
    diag += "  Voltage L1-N: " + std::to_string(getVoltageL1N()) + " V\n";
    diag += "  Voltage L2-N: " + std::to_string(getVoltageL2N()) + " V\n";
    diag += "  Voltage L3-N: " + std::to_string(getVoltageL3N()) + " V\n";
    diag += "  Current L1: " + std::to_string(getCurrentL1()) + " A\n";
    diag += "  Current L2: " + std::to_string(getCurrentL2()) + " A\n";
    diag += "  Current L3: " + std::to_string(getCurrentL3()) + " A\n";
    diag += "  Active Power: " + std::to_string(getActivePowerTotal() / 1000.0f) + " kW\n";
    diag += "  Power Factor: " + std::to_string(getPowerFactorTotal()) + "\n";
    diag += "  Frequency: " + std::to_string(getFrequency()) + " Hz\n";
    diag += "  Energy Import: " + std::to_string(getActiveEnergyImport()) + " kWh\n";
    diag += "  Energy Export: " + std::to_string(getActiveEnergyExport()) + " kWh\n";
    diag += "  Active Tariff: " + std::to_string(state_.active_tariff) + "\n";
    
    if (state_.hasAlarm()) {
        diag += "  ALARM: 0x" + std::to_string(state_.alarm_status) + "\n";
    }
    if (state_.hasFault()) {
        diag += "  FAULT: " + std::to_string(state_.fault_code) + "\n";
    }
    
    return diag;
}

// ============================================================================
// Callbacks
// ============================================================================

void EnergyMeter::setAlarmCallback(AlarmCallback callback)
{
    alarm_callback_ = std::move(callback);
}

void EnergyMeter::setEnergyCallback(EnergyCallback callback)
{
    energy_callback_ = std::move(callback);
}

void EnergyMeter::setDataCallback(DataCallback callback)
{
    data_callback_ = std::move(callback);
}

// ============================================================================
// SDO Access (Private)
// ============================================================================

bool EnergyMeter::readSDO(uint16_t index, uint8_t subindex, void* data, size_t len)
{
    size_t actual_size;
    return m_coe.readSync(index, subindex, data, len, EtherCAT::SDO::kDefaultSDOTimeoutMs, &actual_size);
}

bool EnergyMeter::writeSDO(uint16_t index, uint8_t subindex, const void* data, size_t len)
{
    auto result = m_coe.writeSync(index, subindex, data, len, {.timeout_ms = EtherCAT::SDO::kDefaultSDOTimeoutMs});
    return result.has_value();
}

} // namespace CiA430

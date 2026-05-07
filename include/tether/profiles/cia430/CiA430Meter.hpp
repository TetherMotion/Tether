/**
 * @file CiA430Meter.hpp
 * @brief CiA 430 Energy Meter Controller
 *
 * Provides comprehensive energy metering and power quality monitoring.
 *
 * Features:
 * - Three-phase power measurement
 * - Energy accumulation (import/export)
 * - Power factor and frequency monitoring
 * - THD and harmonics analysis
 * - Demand metering
 * - Multi-tariff support
 * - Power quality events
 */

#pragma once

#include "profiles/cia430/CiA430Defs.hpp"
#include <cstdint>
#include <string>
#include <functional>
#include <array>

namespace EtherCAT { namespace SDO { class SDOManager; } }

namespace CiA430 {

// ============================================================================
// Data Structures
// ============================================================================

/**
 * @brief Meter specifications
 */
struct MeterSpec {
    uint8_t  meter_type = 0;
    uint8_t  num_phases = 3;
    uint16_t rated_voltage = 0;   // V
    uint16_t rated_current = 0;   // A
    uint16_t rated_frequency = 0; // Hz * 100
    uint8_t  accuracy_class = 0;
    uint16_t ct_ratio = 1;
    uint16_t vt_ratio = 1;
};

/**
 * @brief Per-phase voltage readings
 */
struct VoltageReading {
    int32_t l1n = 0;    // 0.01V
    int32_t l2n = 0;
    int32_t l3n = 0;
    int32_t l1l2 = 0;
    int32_t l2l3 = 0;
    int32_t l3l1 = 0;
    int32_t avg_ln = 0;
    int32_t avg_ll = 0;
    
    float getL1N_V() const { return rawToVolts(l1n); }
    float getL2N_V() const { return rawToVolts(l2n); }
    float getL3N_V() const { return rawToVolts(l3n); }
    float getL1L2_V() const { return rawToVolts(l1l2); }
    float getL2L3_V() const { return rawToVolts(l2l3); }
    float getL3L1_V() const { return rawToVolts(l3l1); }
};

/**
 * @brief Per-phase current readings
 */
struct CurrentReading {
    int32_t l1 = 0;     // 0.001A
    int32_t l2 = 0;
    int32_t l3 = 0;
    int32_t neutral = 0;
    int32_t avg = 0;
    int32_t total = 0;
    
    float getL1_A() const { return rawToAmps(l1); }
    float getL2_A() const { return rawToAmps(l2); }
    float getL3_A() const { return rawToAmps(l3); }
    float getNeutral_A() const { return rawToAmps(neutral); }
};

/**
 * @brief Power readings
 */
struct PowerReading {
    int32_t active_l1 = 0;      // 0.1W
    int32_t active_l2 = 0;
    int32_t active_l3 = 0;
    int32_t active_total = 0;
    int32_t reactive_l1 = 0;    // 0.1VAr
    int32_t reactive_l2 = 0;
    int32_t reactive_l3 = 0;
    int32_t reactive_total = 0;
    int32_t apparent_l1 = 0;    // 0.1VA
    int32_t apparent_l2 = 0;
    int32_t apparent_l3 = 0;
    int32_t apparent_total = 0;
    
    float getActiveTotal_W() const { return rawToWatts(active_total); }
    float getReactiveTotal_VAr() const { return rawToWatts(reactive_total); }
    float getApparentTotal_VA() const { return rawToWatts(apparent_total); }
    float getActiveTotal_kW() const { return rawToWatts(active_total) / 1000.0f; }
};

/**
 * @brief Energy readings
 */
struct EnergyReading {
    int64_t active_import = 0;     // Wh
    int64_t active_export = 0;
    int64_t reactive_import = 0;   // VArh
    int64_t reactive_export = 0;
    int64_t apparent_import = 0;   // VAh
    int64_t apparent_export = 0;
    std::array<int64_t, 4> tariff_energy = {0};
    
    float getActiveImport_kWh() const { return rawToKWh(active_import); }
    float getActiveExport_kWh() const { return rawToKWh(active_export); }
    float getNetActive_kWh() const { return rawToKWh(active_import - active_export); }
};

/**
 * @brief Power quality data
 */
struct PowerQuality {
    int16_t  pf_l1 = 0;          // 0.001
    int16_t  pf_l2 = 0;
    int16_t  pf_l3 = 0;
    int16_t  pf_total = 0;
    uint16_t frequency = 0;       // 0.01Hz
    uint16_t thd_v_l1 = 0;        // 0.1%
    uint16_t thd_v_l2 = 0;
    uint16_t thd_v_l3 = 0;
    uint16_t thd_i_l1 = 0;
    uint16_t thd_i_l2 = 0;
    uint16_t thd_i_l3 = 0;
    uint16_t voltage_unbalance = 0;
    uint16_t current_unbalance = 0;
    uint8_t  phase_sequence = 0;
    
    float getPowerFactorTotal() const { return rawToPF(pf_total); }
    float getFrequency_Hz() const { return rawToHz(frequency); }
    float getTHDV_L1() const { return rawToTHD(thd_v_l1); }
};

/**
 * @brief Demand data
 */
struct DemandData {
    int32_t  active_power = 0;
    int32_t  reactive_power = 0;
    int32_t  apparent_power = 0;
    int32_t  current = 0;
    int32_t  max_active_power = 0;
    int32_t  max_reactive_power = 0;
    int32_t  max_current = 0;
    uint16_t period = 0;          // minutes
    uint8_t  subperiods = 0;
};

/**
 * @brief Current meter state
 */
struct MeterState {
    uint16_t statusword = 0;
    VoltageReading voltage;
    CurrentReading current;
    PowerReading power;
    EnergyReading energy;
    PowerQuality quality;
    uint32_t alarm_status = 0;
    uint16_t fault_code = 0;
    uint8_t  active_tariff = 0;
    
    bool isReady() const { return statusword & StatuswordBits::Ready; }
    bool isDataValid() const { return statusword & StatuswordBits::DataValid; }
    bool isExporting() const { return statusword & StatuswordBits::EnergyDirection; }
    bool hasAlarm() const { return statusword & StatuswordBits::AlarmActive; }
    bool hasFault() const { return statusword & StatuswordBits::Fault; }
    bool isPhaseL1OK() const { return statusword & StatuswordBits::PhaseL1OK; }
    bool isPhaseL2OK() const { return statusword & StatuswordBits::PhaseL2OK; }
    bool isPhaseL3OK() const { return statusword & StatuswordBits::PhaseL3OK; }
};

/**
 * @brief Alarm thresholds
 */
struct AlarmThresholds {
    int32_t voltage_high = 26400;  // 264.00V
    int32_t voltage_low = 19800;   // 198.00V
    int32_t current_high = 100000; // 100.000A
    int16_t power_factor_low = 850;// 0.850
    uint16_t frequency_high = 5100;// 51.00Hz
    uint16_t frequency_low = 4900; // 49.00Hz
    uint16_t thd_threshold = 80;   // 8.0%
};

// ============================================================================
// Callback Types
// ============================================================================

using AlarmCallback = std::function<void(uint32_t alarm_bits)>;
using EnergyCallback = std::function<void(const EnergyReading& energy)>;
using DataCallback = std::function<void(const MeterState& state)>;

// ============================================================================
// PDO Mapping Presets
// ============================================================================

enum class PDOMappingPreset {
    Basic,
    Extended,
    Full,
    Custom
};

// ============================================================================
// Energy Meter Controller Class
// ============================================================================

class EnergyMeter {
public:
    explicit EnergyMeter(EtherCAT::SDO::SDOManager& sdo, uint16_t slave_addr, bool use_configured_addr = false);
    ~EnergyMeter();
    
    // ========================================================================
    // Initialization
    // ========================================================================
    
    bool initialize();
    bool isInitialized() const { return initialized_; }
    const MeterSpec& getSpec() const { return spec_; }
    
    // ========================================================================
    // PDO Configuration
    // ========================================================================
    
    bool applyPDOMapping(PDOMappingPreset preset);
    
    // ========================================================================
    // Cyclic Update
    // ========================================================================
    
    void processTxPDO(const uint8_t* data, size_t len);
    size_t prepareRxPDO(uint8_t* data, size_t max_len);
    void update();
    
    // ========================================================================
    // Voltage Reading
    // ========================================================================
    
    float getVoltageL1N() const;
    float getVoltageL2N() const;
    float getVoltageL3N() const;
    float getVoltageL1L2() const;
    float getVoltageL2L3() const;
    float getVoltageL3L1() const;
    float getVoltageAvgLN() const;
    float getVoltageAvgLL() const;
    const VoltageReading& getVoltage() const { return state_.voltage; }
    
    // ========================================================================
    // Current Reading
    // ========================================================================
    
    float getCurrentL1() const;
    float getCurrentL2() const;
    float getCurrentL3() const;
    float getCurrentNeutral() const;
    float getCurrentAvg() const;
    const CurrentReading& getCurrent() const { return state_.current; }
    
    // ========================================================================
    // Power Reading
    // ========================================================================
    
    float getActivePowerTotal() const;
    float getReactivePowerTotal() const;
    float getApparentPowerTotal() const;
    float getActivePowerL1() const;
    float getActivePowerL2() const;
    float getActivePowerL3() const;
    const PowerReading& getPower() const { return state_.power; }
    
    // ========================================================================
    // Energy Reading
    // ========================================================================
    
    float getActiveEnergyImport() const;   // kWh
    float getActiveEnergyExport() const;
    float getNetActiveEnergy() const;
    float getReactiveEnergyImport() const; // kVArh
    float getReactiveEnergyExport() const;
    const EnergyReading& getEnergy() const { return state_.energy; }
    
    bool resetEnergyCounters();
    bool freezeEnergyCounters();
    
    // ========================================================================
    // Power Quality
    // ========================================================================
    
    float getPowerFactorTotal() const;
    float getPowerFactorL1() const;
    float getPowerFactorL2() const;
    float getPowerFactorL3() const;
    float getFrequency() const;
    
    float getTHDV_L1() const;
    float getTHDV_L2() const;
    float getTHDV_L3() const;
    float getTHDI_L1() const;
    float getTHDI_L2() const;
    float getTHDI_L3() const;
    
    float getVoltageUnbalance() const;
    float getCurrentUnbalance() const;
    uint8_t getPhaseSequence() const;
    
    const PowerQuality& getPowerQuality() const { return state_.quality; }
    
    // ========================================================================
    // Harmonics
    // ========================================================================
    
    bool getVoltageHarmonics(uint8_t phase, float* harmonics, size_t count);
    bool getCurrentHarmonics(uint8_t phase, float* harmonics, size_t count);
    
    // ========================================================================
    // Demand
    // ========================================================================
    
    DemandData getDemandData();
    bool setDemandPeriod(uint16_t period_minutes, uint8_t subperiods);
    bool resetDemandValues();
    bool resetMaxDemand();
    
    // ========================================================================
    // Tariff
    // ========================================================================
    
    uint8_t getActiveTariff() const { return state_.active_tariff; }
    bool setActiveTariff(uint8_t tariff);
    float getTariffEnergy(uint8_t tariff) const;  // kWh
    
    // ========================================================================
    // Alarms
    // ========================================================================
    
    bool setAlarmThresholds(const AlarmThresholds& thresholds);
    AlarmThresholds getAlarmThresholds();
    bool enableAlarms(uint32_t alarm_mask);
    uint32_t getAlarmStatus() const { return state_.alarm_status; }
    bool isAlarmActive(uint32_t alarm_bit) const;
    bool resetAlarms();
    
    // ========================================================================
    // Diagnostics
    // ========================================================================
    
    const MeterState& getState() const { return state_; }
    uint16_t getFaultCode() const { return state_.fault_code; }
    uint32_t getOperatingHours();
    float getTemperature();
    std::string getDiagnostics() const;
    
    // ========================================================================
    // Callbacks
    // ========================================================================
    
    void setAlarmCallback(AlarmCallback callback);
    void setEnergyCallback(EnergyCallback callback);
    void setDataCallback(DataCallback callback);

private:
    bool readSpec();
    void processBasicPDO(const uint8_t* data, size_t len);
    void processExtendedPDO(const uint8_t* data, size_t len);
    void processFullPDO(const uint8_t* data, size_t len);
    void checkStateChanges();
    
    bool readSDO(uint16_t index, uint8_t subindex, void* data, size_t len);
    bool writeSDO(uint16_t index, uint8_t subindex, const void* data, size_t len);
    
    EtherCAT::SDO::SDOManager& m_sdo;
    uint16_t slave_addr_;
    bool use_configured_addr_;
    bool initialized_;
    
    MeterSpec spec_;
    MeterState state_;
    uint16_t prev_statusword_;
    uint32_t prev_alarm_status_;
    
    uint16_t controlword_;
    PDOMappingPreset current_mapping_;
    
    AlarmCallback alarm_callback_;
    EnergyCallback energy_callback_;
    DataCallback data_callback_;
};

} // namespace CiA430

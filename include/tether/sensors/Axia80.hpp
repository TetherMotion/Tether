/**
 * @file Axia80.hpp
 * @brief High-level Tether helper for the ATI Axia80 Force/Torque Sensor
 *
 * Wraps Master/Slave to provide:
 *   - Calibration reading via SDO
 *   - Control command builders (bias, filter, calibration slot, sample rate)
 *   - Status checking and unit conversion
 *   - PDO buffer helpers
 */

#pragma once

#include <atomic>
#include <cmath>
#include <cstring>

#include "tether/ethercat/Master.hpp"
#include "tether/ethercat/Slave.hpp"
#include "tether/ethercat/PDOManager.hpp"
#include "tether/ethercat/DebugFlags.hpp"
#include "tether/sensors/Axia80/Axia80PDO.hpp"
#include "tether/sensors/Axia80/Axia80Registers.hpp"
#include "tether/platform/Platform.hpp"

namespace EtherCAT {
namespace Sensors {

/**
 * @brief High-level wrapper for ATI Axia80 F/T sensor operations.
 *
 * This class does **not** own the Master; it operates on a
 * reference passed at construction.  Typical usage:
 *
 * @code
 *   EtherCAT::Master master;
 *   // ... start master, discover slaves ...
 *   EtherCAT::Sensors::Axia80Sensor sensor(master, slave_index);
 *   sensor.init();                     // mailbox + PDO mapping + OP
 *   sensor.readCalibrationData(cal);   // SDO read
 *   sensor.setBias();                  // write control via RxPDO
 *   // In loop:
 *   auto* tx = sensor.txPDO();        // typed pointer to mapped buffer
 *   // ... process fx, fy, fz, tx, ty, tz ...
 * @endcode
 */
class Axia80Sensor {
public:
    /**
     * @param master      Running Master instance
     * @param slave_index Bus position of the Axia80 sensor
     */
    Axia80Sensor(Master& master, uint16_t slave_index)
        : master_(master), slave_index_(slave_index) {}

    // -- Identification ----------------------------------------------------

    uint16_t slaveIndex() const { return slave_index_; }

    Slave& slave() { return master_.slave(slave_index_); }
    const Master& master() const { return master_; }

    // -- Vendor verification ---------------------------------------------

    /**
     * @brief Check whether the discovered slave matches Axia80 vendor/product.
     *
     * Reads SII identity object (0x1018) via SDO and compares vendor ID
     * and product code against known ATI Axia80 values.
     */
    bool isAxia80Device();

    // -- Initialisation ----------------------------------------------------

    /**
     * @brief Full sensor initialisation sequence.
     *
     * 1. autoConfigureMailbox
     * 2. transitionToPreOp
     * 3. Map PDOs via PDOManager
     * 4. configurePDOSyncManagers
     * 5. transitionToSafeOp → transitionToOp
     *
     * @param log_level  Verbosity for diagnostic output
     * @return true on success
     */
    bool init(Tether::Platform::LogLevel log_level = Tether::Platform::LogLevel::Info,
              bool transition_to_op = true);

    /**
     * @brief Print detailed PDO layout information for debugging.
     *
     * Logs the structure of TxPDO and RxPDO with field names, offsets,
     * and sizes. Called automatically when PDO debug flags are enabled.
     */
    static void printPDOLayout();

    /**
     * @brief Print TxPDO data contents for debugging.
     *
     * Logs the current values of all TxPDO fields.
     */
    void printTxPDOData() const;

    /**
     * @brief Print RxPDO data contents for debugging.
     *
     * Logs the current values of all RxPDO fields.
     */
    void printRxPDOData() const;

    // -- PDO access --------------------------------------------------------

    /** @brief Typed pointer to the mapped TxPDO buffer (slave → master). */
    Axia80_pdo::Axia80_TxPDO* txPDO() {
        return reinterpret_cast<Axia80_pdo::Axia80_TxPDO*>(txpdo_buffer_);
    }

    /** @brief Typed pointer to the mapped RxPDO buffer (master → slave). */
    Axia80_pdo::Axia80_RxPDO* rxPDO() {
        return reinterpret_cast<Axia80_pdo::Axia80_RxPDO*>(rxpdo_buffer_);
    }

    const Axia80_pdo::Axia80_TxPDO* txPDO() const {
        return reinterpret_cast<const Axia80_pdo::Axia80_TxPDO*>(txpdo_buffer_);
    }

    const Axia80_pdo::Axia80_RxPDO* rxPDO() const {
        return reinterpret_cast<const Axia80_pdo::Axia80_RxPDO*>(rxpdo_buffer_);
    }

    uint8_t* txPDOBuffer() { return txpdo_buffer_; }
    uint8_t* rxPDOBuffer() { return rxpdo_buffer_; }
    uint16_t txPDOSize() const { return Axia80_pdo::TxPDO_1A00.size; }
    uint16_t rxPDOSize() const { return Axia80_pdo::RxPDO_1601.size; }

    // -- Control helpers ---------------------------------------------------

    /** @brief Set control word 1 (0x7010.1) — convenience for raw access. */
    void setControl1(uint32_t value) {
        if (auto* pdo = rxPDO()) { pdo->control1 = value; }
    }

    /** @brief Set control word 2 (0x7010.2) — convenience for raw access. */
    void setControl2(uint32_t value) {
        if (auto* pdo = rxPDO()) { pdo->control2 = value; }
    }

    /**
     * @brief Build a control1 value from discrete settings.
     *
     * @param filter     Low-pass filter type
     * @param slot       Calibration slot
     * @param rate       Sample rate
     * @param bias       Set bias bit
     * @param clearBias  Clear bias bit
     * @return Composed control1 value
     */
    static uint32_t buildControl1(
        Axia80::FilterType filter = Axia80::FilterType::NO_FILTER,
        Axia80::CalibrationSlot slot = Axia80::CalibrationSlot::SLOT_0,
        Axia80::SampleRate rate = Axia80::SampleRate::RATE_1953_HZ,
        bool bias = false,
        bool clearBias = false)
    {
        uint32_t ctrl = 0;
        if (bias)      ctrl |= Axia80::CTRL_BIAS_BIT;
        if (clearBias) ctrl |= Axia80::CTRL_CLEAR_BIAS_BIT;
        ctrl |= (static_cast<uint32_t>(filter) << Axia80::CTRL_FILTER_SHIFT) & Axia80::CTRL_FILTER_MASK;
        ctrl |= (static_cast<uint32_t>(slot)   << Axia80::CTRL_CALIBRATION_SHIFT) & Axia80::CTRL_CALIBRATION_MASK;
        ctrl |= (static_cast<uint32_t>(rate)  << Axia80::CTRL_SAMPLE_RATE_SHIFT) & Axia80::CTRL_SAMPLE_RATE_MASK;
        return ctrl;
    }

    /** @brief Pulse the bias bit (set for one cycle, then clear). */
    void setBias();

    /** @brief Pulse the clear-bias bit. */
    void clearBias();

    /** @brief Write filter + calibration + sample rate settings. */
    void setConfiguration(Axia80::FilterType filter,
                          Axia80::CalibrationSlot slot,
                          Axia80::SampleRate rate);

    // -- Status helpers ----------------------------------------------------

    /** @brief Check if the status code indicates any error condition. */
    static bool hasError(uint32_t status) {
        return (status & Axia80::STATUS_ERROR) != 0;
    }

    static bool hasTemperatureError(uint32_t status) {
        return (status & Axia80::STATUS_TEMP_OUT_OF_RANGE) != 0;
    }

    static bool hasVoltageError(uint32_t status) {
        return (status & Axia80::STATUS_VOLTAGE_OUT_OF_RANGE) != 0;
    }

    static bool hasBrokenGage(uint32_t status) {
        return (status & Axia80::STATUS_BROKEN_GAGE) != 0;
    }

    static bool isBusy(uint32_t status) {
        return (status & Axia80::STATUS_BUSY) != 0;
    }

    static bool hasSensingRangeExceeded(uint32_t status) {
        return (status & Axia80::STATUS_SENSING_RANGE_EXCEEDED) != 0;
    }

    // -- Conversion --------------------------------------------------------

    /**
     * @brief Convert raw sensor counts to engineering units.
     *
     * Uses counts-per-force and counts-per-torque from calibration data.
     *
     * @param raw_counts  Raw int32 sensor value
     * @param counts_per_unit  Calibration scaling factor
     * @return Value in engineering units (N or Nm)
     */
    static double convertToUnits(int32_t raw_counts, uint32_t counts_per_unit) {
        if (counts_per_unit == 0) return static_cast<double>(raw_counts);
        return static_cast<double>(raw_counts) / static_cast<double>(counts_per_unit);
    }

    /**
     * @brief Convert an entire 6-DOF wrench from counts to engineering units.
     *
     * @param[in]  raw      6-element int32 array [fx, fy, fz, tx, ty, tz]
     * @param[out] out      6-element double array
     * @param      counts_per_force   From calibration
     * @param      counts_per_torque  From calibration
     */
    static void convertWrench(const int32_t raw[6], double out[6],
                               uint32_t counts_per_force,
                               uint32_t counts_per_torque)
    {
        for (int i = 0; i < 3; ++i) {
            out[i] = convertToUnits(raw[i], counts_per_force);
        }
        for (int i = 3; i < 6; ++i) {
            out[i] = convertToUnits(raw[i], counts_per_torque);
        }
    }

    // -- SDO reading -------------------------------------------------------

    /**
     * @brief Read the full calibration object (0x2021) via SDO.
     *
     * @param[out] cal  Populated calibration data
     * @return true on success
     */
    bool readCalibrationData(Axia80::CalibrationData& cal);

    /**
     * @brief Read status info (0x2080) via SDO.
     */
    bool readStatusInfo(Axia80::StatusInfo& info);

    /**
     * @brief Read version info (0x2090) via SDO.
     */
    bool readVersionInfo(Axia80::VersionInfo& info);

    /**
     * @brief Read product description (0x2019) via SDO.
     */
    bool readProductDescription(Axia80::ProductDescription& desc);

    /**
     * @brief Read device name string (0x1008) via SDO.
     */
    bool readDeviceName(char* out, size_t capacity);

private:
    Master& master_;
    uint16_t slave_index_;

    // PDO buffers (owned by PDOManager mapping, not by this class)
    alignas(8) uint8_t txpdo_buffer_[Axia80_pdo::TxPDO_1A00.size] = {};
    alignas(8) uint8_t rxpdo_buffer_[Axia80_pdo::RxPDO_1601.size] = {};
};

// ============================================================================
// Inline implementations
// ============================================================================

inline bool Axia80Sensor::isAxia80Device()
{
    EtherCAT::SII::SIIIdentity id;
    if (!EtherCAT::SII::readSIIIdentity(master_, slave_index_, id)) {
        TETHER_LOGW("Axia80", "Slave %u: SII identity read failed — cannot verify device type", slave_index_);
        return false;
    }
    bool match = (id.vendor_id == Axia80::kVendorId) && (id.product_code == Axia80::kProductCode);
    if (!match) {
        TETHER_LOGW("Axia80", "Slave %u: VID/PID mismatch — expected 0x%08X/0x%08X, got 0x%08X/0x%08X",
                    slave_index_, Axia80::kVendorId, Axia80::kProductCode,
                    id.vendor_id, id.product_code);
    }
    return match;
}

inline bool Axia80Sensor::init(Tether::Platform::LogLevel log_level,
                                bool transition_to_op)
{
    auto& sl = slave();

    // Print PDO layout if debug flags are enabled
    if (debug::rxPDO() || debug::txPDO()) {
        printPDOLayout();
    }

    // 1. Mailbox
    if (sl.configureMailbox(log_level) != SlaveError::Ok) {
        TETHER_LOGE("Axia80", "Slave %u: mailbox config failed", slave_index_);
        return false;
    }

    // 2. PRE_OP
    if (sl.transitionToPreOp() != SlaveError::Ok) {
        TETHER_LOGE("Axia80", "Slave %u: PRE_OP failed", slave_index_);
        return false;
    }

    // 3. PDO mapping via PDOManager
    auto& pdo_mgr = master_.pdo();
    if (!pdo_mgr.isInitialized()) {
        pdo_mgr.init();
    }

    pdo_mgr.mapping().add_rxpdo(slave_index_, rxPDOBuffer(), rxPDOSize(),
                                 Axia80_pdo::RxPDO_1601.index,
                                 PDO::PDOAddressMode::Position);
    pdo_mgr.mapping().add_txpdo(slave_index_, txPDOBuffer(), txPDOSize(),
                                 Axia80_pdo::TxPDO_1A00.index,
                                 PDO::PDOAddressMode::Position);

    if (!pdo_mgr.finalizeMapping(slave_index_)) {
        TETHER_LOGE("Axia80", "Slave %u: PDO finalize failed", slave_index_);
        return false;
    }

    // 4. Configure SM2/SM3
    if (sl.configurePDOSyncManagers() != SlaveError::Ok) {
        TETHER_LOGE("Axia80", "Slave %u: PDO SM config failed", slave_index_);
        return false;
    }

    // 5. SAFE_OP → OP
    if (sl.transitionToSafeOp() != SlaveError::Ok) {
        TETHER_LOGE("Axia80", "Slave %u: SAFE_OP failed", slave_index_);
        return false;
    }
    if (transition_to_op) {
        if (sl.transitionToOp() != SlaveError::Ok) {
            TETHER_LOGE("Axia80", "Slave %u: OP failed", slave_index_);
            return false;
        }
    }

    TETHER_LOGI("Axia80", "Slave %u initialised successfully%s", slave_index_,
                transition_to_op ? "" : " (SAFE-OP only)");
    return true;
}

inline void Axia80Sensor::setBias()
{
    if (auto* pdo = rxPDO()) {
        pdo->control1 |= Axia80::CTRL_BIAS_BIT;
    }
}

inline void Axia80Sensor::clearBias()
{
    if (auto* pdo = rxPDO()) {
        pdo->control1 |= Axia80::CTRL_CLEAR_BIAS_BIT;
    }
}

inline void Axia80Sensor::setConfiguration(Axia80::FilterType filter,
                                              Axia80::CalibrationSlot slot,
                                              Axia80::SampleRate rate)
{
    if (auto* pdo = rxPDO()) {
        pdo->control1 = buildControl1(filter, slot, rate, false, false);
    }
}

// -- SDO helpers ----------------------------------------------------------

inline bool Axia80Sensor::readCalibrationData(Axia80::CalibrationData& cal)
{
    auto& sl = slave();
    bool ok = true;
    size_t sz = 0;

    sz = sizeof(cal.ft_serial);
    ok &= (sl.sdoRead(Axia80::OD_CALIBRATION_MATRIX, Axia80::CAL_SUBIDX_FT_SERIAL,
                      cal.ft_serial, sz) == SlaveError::Ok);

    sz = sizeof(cal.part_number);
    ok &= (sl.sdoRead(Axia80::OD_CALIBRATION_MATRIX, Axia80::CAL_SUBIDX_PART_NUMBER,
                      cal.part_number, sz) == SlaveError::Ok);

    sz = sizeof(cal.family);
    ok &= (sl.sdoRead(Axia80::OD_CALIBRATION_MATRIX, Axia80::CAL_SUBIDX_FAMILY,
                      cal.family, sz) == SlaveError::Ok);

    sz = sizeof(cal.cal_time);
    ok &= (sl.sdoRead(Axia80::OD_CALIBRATION_MATRIX, Axia80::CAL_SUBIDX_TIME,
                      cal.cal_time, sz) == SlaveError::Ok);

    // Read matrix entries (sub 5..46) as 16-byte strings, parse to double
    for (uint8_t i = 0; i < 42; ++i) {
        if (master_.isCancelRequested()) {
            TETHER_LOGW("axia80", "Calibration read interrupted by cancel");
            return false;
        }
        char buf[17] = {};
        sz = sizeof(buf) - 1;
        if (sl.sdoRead(Axia80::OD_CALIBRATION_MATRIX, Axia80::CAL_SUBIDX_MATRIX_FX_G0 + i,
                       buf, sz) == SlaveError::Ok) {
            cal.matrix[i / 7][i % 7] = std::strtod(buf, nullptr);
        }
    }

    uint8_t fu = 0, tu = 0;
    ok &= (sl.sdoReadU8(Axia80::OD_CALIBRATION_MATRIX, Axia80::CAL_SUBIDX_FORCE_UNITS, fu) == SlaveError::Ok);
    ok &= (sl.sdoReadU8(Axia80::OD_CALIBRATION_MATRIX, Axia80::CAL_SUBIDX_TORQUE_UNITS, tu) == SlaveError::Ok);
    cal.force_units = static_cast<Axia80::ForceUnits>(fu);
    cal.torque_units = static_cast<Axia80::TorqueUnits>(tu);

    for (int i = 0; i < 6; ++i) {
        ok &= (sl.sdoReadU32(Axia80::OD_CALIBRATION_MATRIX, Axia80::CAL_SUBIDX_MAX_FX_COUNTS + i,
                             *reinterpret_cast<uint32_t*>(&cal.max_counts[i])) == SlaveError::Ok);
    }

    ok &= (sl.sdoReadU32(Axia80::OD_CALIBRATION_MATRIX, Axia80::CAL_SUBIDX_COUNTS_PER_FORCE,
                          cal.counts_per_force) == SlaveError::Ok);
    ok &= (sl.sdoReadU32(Axia80::OD_CALIBRATION_MATRIX, Axia80::CAL_SUBIDX_COUNTS_PER_TORQUE,
                          cal.counts_per_torque) == SlaveError::Ok);

    for (int i = 0; i < 8; ++i) {
        ok &= (sl.sdoReadU16(Axia80::OD_CALIBRATION_MATRIX, Axia80::CAL_SUBIDX_GAIN_G0 + i,
                              cal.gains[i]) == SlaveError::Ok);
    }

    return ok;
}

inline bool Axia80Sensor::readStatusInfo(Axia80::StatusInfo& info)
{
    auto& sl = slave();
    bool ok = true;
    size_t sz = sizeof(info.supply_voltage_x10);
    ok &= (sl.sdoRead(Axia80::OD_STATUS_INFO, 1, &info.supply_voltage_x10, sz) == SlaveError::Ok);
    sz = sizeof(info.temperature_x10);
    ok &= (sl.sdoRead(Axia80::OD_STATUS_INFO, 2, &info.temperature_x10, sz) == SlaveError::Ok);
    sz = sizeof(info.status_message) - 1;
    ok &= (sl.sdoRead(Axia80::OD_STATUS_INFO, 3, info.status_message, sz) == SlaveError::Ok);
    return ok;
}

inline bool Axia80Sensor::readVersionInfo(Axia80::VersionInfo& info)
{
    auto& sl = slave();
    bool ok = true;
    size_t sz = sizeof(info.major);
    ok &= (sl.sdoRead(Axia80::OD_VERSION_INFO, 1, &info.major, sz) == SlaveError::Ok);
    sz = sizeof(info.minor);
    ok &= (sl.sdoRead(Axia80::OD_VERSION_INFO, 2, &info.minor, sz) == SlaveError::Ok);
    sz = sizeof(info.revision);
    ok &= (sl.sdoRead(Axia80::OD_VERSION_INFO, 3, &info.revision, sz) == SlaveError::Ok);
    sz = sizeof(info.bootloader_version);
    ok &= (sl.sdoRead(Axia80::OD_VERSION_INFO, 4, &info.bootloader_version, sz) == SlaveError::Ok);
    sz = sizeof(info.sensor_hw_version);
    ok &= (sl.sdoRead(Axia80::OD_VERSION_INFO, 5, &info.sensor_hw_version, sz) == SlaveError::Ok);
    sz = sizeof(info.sensor_instrument);
    ok &= (sl.sdoRead(Axia80::OD_VERSION_INFO, 6, &info.sensor_instrument, sz) == SlaveError::Ok);
    return ok;
}

inline bool Axia80Sensor::readProductDescription(Axia80::ProductDescription& desc)
{
    auto& sl = slave();
    bool ok = true;
    size_t sz = sizeof(desc.vendor_id);
    ok &= (sl.sdoRead(Axia80::OD_PRODUCT_DESCRIPTION, 1, &desc.vendor_id, sz) == SlaveError::Ok);
    sz = sizeof(desc.product_code);
    ok &= (sl.sdoRead(Axia80::OD_PRODUCT_DESCRIPTION, 2, &desc.product_code, sz) == SlaveError::Ok);
    sz = sizeof(desc.product_name) - 1;
    ok &= (sl.sdoRead(Axia80::OD_PRODUCT_DESCRIPTION, 3, desc.product_name, sz) == SlaveError::Ok);
    sz = sizeof(desc.product_revision);
    ok &= (sl.sdoRead(Axia80::OD_PRODUCT_DESCRIPTION, 4, &desc.product_revision, sz) == SlaveError::Ok);
    sz = sizeof(desc.product_serial_number);
    ok &= (sl.sdoRead(Axia80::OD_PRODUCT_DESCRIPTION, 5, &desc.product_serial_number, sz) == SlaveError::Ok);
    sz = sizeof(desc.manufacturer) - 1;
    ok &= (sl.sdoRead(Axia80::OD_PRODUCT_DESCRIPTION, 6, desc.manufacturer, sz) == SlaveError::Ok);
    return ok;
}

inline bool Axia80Sensor::readDeviceName(char* out, size_t capacity)
{
    size_t sz = capacity;
    return slave().sdoRead(Axia80::OD_DEVICE_NAME, 0, out, sz) == SlaveError::Ok;
}

inline void Axia80Sensor::printPDOLayout()
{
    TETHER_LOGI("Axia80", "=== Axia80 PDO Layout ===");
    TETHER_LOGI("Axia80", "");
    TETHER_LOGI("Axia80", "TxPDO (0x1A00) — Slave → Master, %u bytes:", Axia80_pdo::TxPDO_1A00.size);
    TETHER_LOGI("Axia80", "  Offset  Size  Field          Description");
    TETHER_LOGI("Axia80", "  ------  ----  -------------  ----------------------------------------");
    TETHER_LOGI("Axia80", "  0x00    4     fx             Force X (counts, int32_t)");
    TETHER_LOGI("Axia80", "  0x04    4     fy             Force Y (counts, int32_t)");
    TETHER_LOGI("Axia80", "  0x08    4     fz             Force Z (counts, int32_t)");
    TETHER_LOGI("Axia80", "  0x0C    4     tx             Torque X (counts, int32_t)");
    TETHER_LOGI("Axia80", "  0x10    4     ty             Torque Y (counts, int32_t)");
    TETHER_LOGI("Axia80", "  0x14    4     tz             Torque Z (counts, int32_t)");
    TETHER_LOGI("Axia80", "  0x18    4     status         Status code (0x6010, uint32_t)");
    TETHER_LOGI("Axia80", "  0x1C    4     counter        Sample counter (0x6020, uint32_t)");
    TETHER_LOGI("Axia80", "");
    TETHER_LOGI("Axia80", "RxPDO (0x1601) — Master → Slave, %u bytes:", Axia80_pdo::RxPDO_1601.size);
    TETHER_LOGI("Axia80", "  Offset  Size  Field          Description");
    TETHER_LOGI("Axia80", "  ------  ----  -------------  ----------------------------------------");
    TETHER_LOGI("Axia80", "  0x00    4     control1       Control register 1 (0x7010.1, uint32_t)");
    TETHER_LOGI("Axia80", "  0x04    4     control2       Control register 2 (0x7010.2, uint32_t)");
    TETHER_LOGI("Axia80", "");
    TETHER_LOGI("Axia80", "control1 bit fields:");
    TETHER_LOGI("Axia80", "  Bit 0:   BIAS (set to tare)");
    TETHER_LOGI("Axia80", "  Bit 1:   CLEAR_BIAS");
    TETHER_LOGI("Axia80", "  Bits 2-4: FILTER (0=none, 1..7=filter levels)");
    TETHER_LOGI("Axia80", "  Bits 5-6: CALIBRATION_SLOT (0..3)");
    TETHER_LOGI("Axia80", "  Bits 7-10: SAMPLE_RATE (0=780Hz, 1=1563Hz, 2=3125Hz, 3=6250Hz, 4=12500Hz)");
    TETHER_LOGI("Axia80", "========================");
}

inline void Axia80Sensor::printTxPDOData() const
{
    const auto* tx = txPDO();
    if (!tx) return;
    TETHER_LOGI("Axia80", "[TxPDO-DEBUG] Axia80 TxPDO (0x1A00) contents:");
    TETHER_LOGI("Axia80", "  fx=%ld  fy=%ld  fz=%ld  tx=%ld  ty=%ld  tz=%ld",
                static_cast<long>(tx->fx), static_cast<long>(tx->fy),
                static_cast<long>(tx->fz), static_cast<long>(tx->tx),
                static_cast<long>(tx->ty), static_cast<long>(tx->tz));
    TETHER_LOGI("Axia80", "  status=0x%08X  counter=%u", tx->status, tx->counter);
}

inline void Axia80Sensor::printRxPDOData() const
{
    const auto* rx = rxPDO();
    if (!rx) return;
    TETHER_LOGI("Axia80", "[RxPDO-DEBUG] Axia80 RxPDO (0x1601) contents:");
    TETHER_LOGI("Axia80", "  control1=0x%08X  control2=0x%08X", rx->control1, rx->control2);
    TETHER_LOGI("Axia80", "  control1 breakdown:");
    TETHER_LOGI("Axia80", "    BIAS=%s  CLEAR_BIAS=%s  FILTER=%u  SLOT=%u  RATE=%u",
                (rx->control1 & Axia80::CTRL_BIAS_BIT) ? "1" : "0",
                (rx->control1 & Axia80::CTRL_CLEAR_BIAS_BIT) ? "1" : "0",
                (rx->control1 & Axia80::CTRL_FILTER_MASK) >> Axia80::CTRL_FILTER_SHIFT,
                (rx->control1 & Axia80::CTRL_CALIBRATION_MASK) >> Axia80::CTRL_CALIBRATION_SHIFT,
                (rx->control1 & Axia80::CTRL_SAMPLE_RATE_MASK) >> Axia80::CTRL_SAMPLE_RATE_SHIFT);
}

} // namespace Sensors
} // namespace EtherCAT

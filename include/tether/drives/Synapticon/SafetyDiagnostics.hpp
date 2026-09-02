/**
 * @file SafetyDiagnostics.hpp
 * @brief Synapticon SOMANET — comprehensive safety module diagnostics
 *
 * Provides extensive SDO-based querying of the Synapticon safety subsystem
 * (SMM — Safe Motion Module) to diagnose why the FSoE slave is not
 * responding or why the safety system is in a faulted state.
 *
 * Key objects queried:
 *   0x2611 — Safety Module input diagnostics (safe state inputs)
 *   0x2620 — General safety (FSoE active, safe address)
 *   0x2621 — Safety digital IO
 *   0x6621 — Safety statusword (STO, SBC, SS1, SS2, SOS, SLS bits)
 *   0x6632 — Error acknowledge
 *   0x6630 — Restart acknowledge
 *   0x203F — Error report (8-char fault code string, e.g. "SmmFIO25")
 *   0x603F — Error code (CiA 402 drive error code)
 *   0x6041 — Statusword (CiA 402 drive state)
 *   0x6770 — FSoE Master Frame Elements (command, ConnID, CRCs)
 *   0x6760 — FSoE Slave Frame Elements (command, ConnID, CRCs)
 *   0xF030 — Configured module ident list
 *   0xF050 — Detected module ident list
 *   0xF980 — Device Safety Address
 *   0x1001 — Error register
 *
 * The main entry point is runFullSafetyDiagnostics(), which reads all
 * relevant objects, decodes them, and logs a comprehensive report.
 */

#pragma once

#include <cstdint>
#include <cstring>
#include "tether/ethercat/Slave.hpp"
#include "tether/fsoe/FSoEHelpers.hpp"
#include "logging/Logger.hpp"

namespace EtherCAT {
namespace Drives {
namespace Synapticon {

// ---------------------------------------------------------------------------
// Object dictionary constants
// ---------------------------------------------------------------------------

// 0x2611 — Safety Module input diagnostics
static constexpr uint16_t kSafetyModuleDiagnosticsIndex = 0x2611;
static constexpr uint8_t  kSafetyModuleInput1Subindex   = 0x01;
static constexpr uint8_t  kSafetyModuleInput2Subindex   = 0x02;

// 0x2620 — General safety
static constexpr uint16_t kGeneralSafetyIndex    = 0x2620;
static constexpr uint8_t  kSafeFieldbusSubindex  = 0x02;
static constexpr uint8_t  kSafeAddressSubindex   = 0x03;

// 0x2621 — Safety digital IO
static constexpr uint16_t kSafetyDigitalIOIndex  = 0x2621;

// 0x6621 — Safety statusword
static constexpr uint16_t kSafetyStatuswordIndex = 0x6621;
static constexpr uint8_t  kSafetyStatusByte1Sub  = 0x01;  // STO / SS1 / SOS / etc.
static constexpr uint8_t  kSafetyStatusByte2Sub  = 0x02;  // SBC / SLS / etc.

// 0x6632 — Error acknowledge
static constexpr uint16_t kErrorAcknowledgeIndex = 0x6632;

// 0x6630 — Restart acknowledge
static constexpr uint16_t kRestartAcknowledgeIndex = 0x6630;

// 0x203F — Error report (STRING(8), e.g. "SmmFIO25", "NoFault ")
static constexpr uint16_t kErrorReportIndex = 0x203F;
static constexpr uint8_t  kErrorReportDescSub = 0x01;

// 0x603F — CiA 402 Error code
static constexpr uint16_t kCiA402ErrorCodeIndex = 0x603F;

// 0x6041 — CiA 402 Statusword
static constexpr uint16_t kCiA402StatuswordIndex = 0x6041;

// 0x6770 — FSoE Master Frame Elements (master→slave, RxPDO)
static constexpr uint16_t kFSoEMasterFrameIndex = 0x6770;
static constexpr uint8_t  kFSoEMasterCmdSub     = 0x01;
static constexpr uint8_t  kFSoEMasterConnIDSub  = 0x02;
static constexpr uint8_t  kFSoEMasterCRC0Sub    = 0x03;
static constexpr uint8_t  kFSoEMasterCRC1Sub    = 0x04;

// 0x6760 — FSoE Slave Frame Elements (slave→master, TxPDO)
static constexpr uint16_t kFSoESlaveFrameIndex  = 0x6760;
static constexpr uint8_t  kFSoESlaveCmdSub      = 0x01;
static constexpr uint8_t  kFSoESlaveConnIDSub   = 0x02;

// 0xF030 — Configured module ident list
static constexpr uint16_t kConfiguredModuleIdentIndex = 0xF030;

// 0xF050 — Detected module ident list
static constexpr uint16_t kDetectedModuleIdentIndex = 0xF050;

// 0xF980 — Device Safety Address
static constexpr uint16_t kFSoESafetyAddressIndex   = 0xF980;
static constexpr uint8_t  kFSoESafetyAddressSubindex = 0x01;

// 0x1001 — Error register
static constexpr uint16_t kErrorRegisterIndex = 0x1001;

// Module ident values for FSoE parameter configuration
static constexpr uint32_t kModuleIdentNoParam  = 0x22D20001;  // No parameter changes via master
static constexpr uint32_t kModuleIdentWithParam = 0x22D20002;  // With parameter changes via master

// FSoE command codes
static constexpr uint8_t kFSoECmdReset      = 0x2A;
static constexpr uint8_t kFSoECmdSession    = 0x4E;
static constexpr uint8_t kFSoECmdConnection = 0x36;
static constexpr uint8_t kFSoECmdParameter  = 0x08;
static constexpr uint8_t kFSoECmdData       = 0x14;
static constexpr uint8_t kFSoECmdFailSafe   = 0x00;

// ---------------------------------------------------------------------------
// Data structures
// ---------------------------------------------------------------------------

/// Result of reading the safety module input diagnostics (0x2611) and the
/// FSoE active indicator (0x2620:2 "Safe fieldbus").
struct SafetyModuleState {
    bool ok = false;
    uint8_t input1 = 0;
    uint8_t input2 = 0;
    bool fsoe_read_ok = false;
    uint8_t safe_fieldbus = 0;

    [[nodiscard]] bool isInSafeState() const noexcept {
        return (input1 == 0) || (input2 == 0);
    }
    [[nodiscard]] bool motionAllowed() const noexcept {
        return (input1 != 0) && (input2 != 0);
    }
    [[nodiscard]] bool fsoeActive() const noexcept {
        return fsoe_read_ok && (safe_fieldbus != 0);
    }
    [[nodiscard]] const char* fsoeStateSummary() const noexcept {
        if (!fsoe_read_ok) return "unknown (SDO read failed)";
        return fsoeActive() ? "active" : "inactive";
    }
    [[nodiscard]] const char* stateSummary() const noexcept {
        if (!ok) return "unknown (SDO read failed)";
        if (isInSafeState()) return "SAFE STATE (motion inhibited)";
        return "operational (motion allowed)";
    }
};

/// Comprehensive safety diagnostic report.
struct SafetyDiagnosticReport {
    // Safety module inputs (0x2611)
    bool inputs_ok = false;
    uint8_t input1 = 0;
    uint8_t input2 = 0;

    // General safety (0x2620)
    bool general_safety_ok = false;
    uint8_t safe_fieldbus = 0;       // 0x2620:2 — non-zero = FSoE active
    uint16_t safe_address = 0;       // 0x2620:3 — FSoE connection ID

    // Safety statusword (0x6621)
    bool statusword_ok = false;
    uint8_t status_byte1 = 0;        // STO, SS1, SS2, SOS, SLS1-4
    uint8_t status_byte2 = 0;        // SBC, safe outputs, etc.

    // Error report (0x203F)
    bool error_report_ok = false;
    char error_report[9] = {};       // 8-char string + null

    // CiA 402 error code (0x603F)
    bool cia402_error_ok = false;
    uint16_t cia402_error_code = 0;

    // CiA 402 statusword (0x6041)
    bool cia402_status_ok = false;
    uint16_t cia402_statusword = 0;

    // FSoE master frame (0x6770) — what the slave received from master
    bool master_frame_ok = false;
    uint8_t master_cmd = 0;
    uint16_t master_conn_id = 0;
    uint16_t master_crc0 = 0;
    uint16_t master_crc1 = 0;

    // FSoE slave frame (0x6760) — what the slave is sending
    bool slave_frame_ok = false;
    uint8_t slave_cmd = 0;
    uint16_t slave_conn_id = 0;

    // Module ident (0xF030 / 0xF050)
    bool module_ident_ok = false;
    uint32_t configured_ident_pos2 = 0;
    uint32_t detected_ident_pos2 = 0;

    // Device safety address (0xF980)
    bool safety_addr_ok = false;
    uint16_t device_safety_address = 0;

    // Error register (0x1001)
    bool error_reg_ok = false;
    uint8_t error_register = 0;

    // --- Convenience accessors ---

    [[nodiscard]] bool hasFault() const noexcept {
        return error_report_ok &&
               std::strncmp(error_report, "NoFault", 7) != 0 &&
               error_report[0] != '\0';
    }

    [[nodiscard]] bool isInSafeState() const noexcept {
        return inputs_ok && ((input1 == 0) || (input2 == 0));
    }

    [[nodiscard]] bool fsoeActive() const noexcept {
        return general_safety_ok && (safe_fieldbus != 0);
    }

    [[nodiscard]] bool moduleIdentMismatch() const noexcept {
        return module_ident_ok &&
               (configured_ident_pos2 != detected_ident_pos2);
    }

    [[nodiscard]] bool connIdMismatch() const noexcept {
        return safety_addr_ok && general_safety_ok &&
               (device_safety_address != safe_address);
    }
};

// ---------------------------------------------------------------------------
// Decoding helpers
// ---------------------------------------------------------------------------

/// Re-use the generic FSoE command name decoder from the FSoE driver.
/// (The kFSoECmd* constants above are kept for SDO diagnostic reference,
///  but the name decoder uses the canonical ETG.5100 values from
///  FSoE::Command in FSoEDefs.hpp, which are verified by the protocol
///  implementation in FSoESlave.cpp / FSoEMasterConnection.cpp.)
using FSoE::fsoeCommandName;

/// Decode safety status byte 1 (0x6621:1) bit flags.
/// Bit layout per Synapticon SMM documentation:
///   bit 0: STO (Safe Torque Off) — 1 = active (torque inhibited)
///   bit 1: SS1 (Safe Stop 1)
///   bit 2: SS2 (Safe Stop 2)
///   bit 3: SOS (Safe Operating Stop)
///   bit 4: SLS1 (Safely Limited Speed 1)
///   bit 5: SLS2 (Safely Limited Speed 2)
///   bit 6: SLS3 (Safely Limited Speed 3)
///   bit 7: SLS4 (Safely Limited Speed 4)
inline void decodeSafetyStatusByte1(uint8_t val, char* buf, size_t buf_size) {
    if (!buf || buf_size < 80) { if (buf) buf[0] = '\0'; return; }
    snprintf(buf, buf_size, "STO=%d SS1=%d SS2=%d SOS=%d SLS1=%d SLS2=%d SLS3=%d SLS4=%d",
             (val >> 0) & 1, (val >> 1) & 1, (val >> 2) & 1, (val >> 3) & 1,
             (val >> 4) & 1, (val >> 5) & 1, (val >> 6) & 1, (val >> 7) & 1);
}

/// Decode safety status byte 2 (0x6621:2) bit flags.
///   bit 0: SBC (Safe Brake Control)
///   bit 1-3: Safe output bits
///   bit 4-7: Safety parameter validation bits
inline void decodeSafetyStatusByte2(uint8_t val, char* buf, size_t buf_size) {
    if (!buf || buf_size < 80) { if (buf) buf[0] = '\0'; return; }
    snprintf(buf, buf_size, "SBC=%d SafeOut=[%d%d%d%d] ParamValid=[%d%d%d]",
             (val >> 0) & 1,
             (val >> 1) & 1, (val >> 2) & 1, (val >> 3) & 1, (val >> 4) & 1,
             (val >> 5) & 1, (val >> 6) & 1, (val >> 7) & 1);
}

/// Decode the CiA 402 statusword (0x6041) to a state name.
inline const char* cia402StateName(uint16_t sw) {
    uint16_t state = sw & 0x006F;  // Mask relevant bits
    switch (state) {
        case 0x0000: return "Not ready to switch on";
        case 0x0040: return "Switch on disabled";
        case 0x0021: return "Ready to switch on";
        case 0x0023: return "Switched on";
        case 0x0027: return "Operation enabled";
        case 0x0007: return "Quick stop active";
        case 0x000F: return "Fault reaction active";
        case 0x0008: return "Fault";
        default:     return "Unknown";
    }
}

/// Decode the module ident value to a parameter configuration description.
inline const char* moduleIdentDesc(uint32_t ident) {
    if (ident == kModuleIdentNoParam)  return "No parameter changes via master (params from OBLAC)";
    if (ident == kModuleIdentWithParam) return "With parameter changes via master";
    return "Unknown module ident";
}

/// Known Synapticon error report codes (from 0x203F:1).
/// These are 8-character strings; only the first 7 are typically meaningful.
inline const char* knownErrorReport(const char* code) {
    if (!code) return "unknown";
    if (std::strncmp(code, "NoFault", 7) == 0)  return "No fault";
    if (std::strncmp(code, "SmmFIO25", 8) == 0) return "Black channel fault — safety parameters not verified in OBLAC Drives";
    if (std::strncmp(code, "SmmFIO26", 8) == 0) return "FSoE connection timeout — master not sending FSoE frames";
    if (std::strncmp(code, "SmmFIO27", 8) == 0) return "FSoE CRC error — CRC mismatch in received frame";
    if (std::strncmp(code, "SmmFIO28", 8) == 0) return "FSoE ConnectionID mismatch";
    if (std::strncmp(code, "SmmFIO29", 8) == 0) return "FSoE command error — unexpected command in current state";
    if (std::strncmp(code, "SmmFIO30", 8) == 0) return "FSoE watchdog timeout — master stopped sending frames";
    if (std::strncmp(code, "SmmFIO31", 8) == 0) return "FSoE device address mismatch";
    if (std::strncmp(code, "CyclicHb", 8) == 0) return "Cyclic heartbeat — PDO communication lost";
    if (std::strncmp(code, "SynDifHi", 8) == 0) return "Sync difference too high — DC sync issue";
    return nullptr;  // Unknown — caller should print raw code
}

// ---------------------------------------------------------------------------
// SDO read helpers (internal)
// ---------------------------------------------------------------------------

namespace detail {

inline bool readU8(::EtherCAT::Slave& slave, uint16_t idx, uint8_t sub,
                   uint8_t& out, const char* tag = "SafetyDiag") {
    auto err = slave.sdoReadU8(idx, sub, out);
    if (err != ::EtherCAT::SlaveError::Ok) {
        TETHER_LOGW(tag, "  SDO read 0x%04X:%u FAILED (%s)",
                    idx, sub, ::EtherCAT::slaveErrorToString(err));
        return false;
    }
    return true;
}

inline bool readU16(::EtherCAT::Slave& slave, uint16_t idx, uint8_t sub,
                    uint16_t& out, const char* tag = "SafetyDiag") {
    auto err = slave.sdoReadU16(idx, sub, out);
    if (err != ::EtherCAT::SlaveError::Ok) {
        TETHER_LOGW(tag, "  SDO read 0x%04X:%u FAILED (%s)",
                    idx, sub, ::EtherCAT::slaveErrorToString(err));
        return false;
    }
    return true;
}

inline bool readU32(::EtherCAT::Slave& slave, uint16_t idx, uint8_t sub,
                    uint32_t& out, const char* tag = "SafetyDiag") {
    auto err = slave.sdoReadU32(idx, sub, out);
    if (err != ::EtherCAT::SlaveError::Ok) {
        TETHER_LOGW(tag, "  SDO read 0x%04X:%u FAILED (%s)",
                    idx, sub, ::EtherCAT::slaveErrorToString(err));
        return false;
    }
    return true;
}

/// Read the 8-byte error report string from 0x203F:1.
/// This object is a STRING(8) type.  The SDO read returns the raw bytes;
/// we copy them and also log the hex representation for debugging.
inline bool readErrorReport(::EtherCAT::Slave& slave, char* out_buf,
                            size_t out_buf_size, const char* tag = "SafetyDiag") {
    if (!out_buf || out_buf_size < 9) return false;
    std::memset(out_buf, 0, out_buf_size);

    // 0x203F:1 is a STRING(8) — read raw bytes via sdoRead
    uint8_t raw[16] = {};
    size_t raw_len = sizeof(raw);
    auto err = slave.sdoRead(kErrorReportIndex, kErrorReportDescSub,
                             raw, raw_len);
    if (err != ::EtherCAT::SlaveError::Ok) {
        TETHER_LOGW(tag, "  SDO read 0x203F:1 (Error report) FAILED (%s)",
                    ::EtherCAT::slaveErrorToString(err));
        return false;
    }

    // Log raw hex for debugging
    TETHER_LOGI(tag, "  0x203F:1 raw bytes (len=%zu): %02X %02X %02X %02X %02X %02X %02X %02X",
                raw_len,
                raw_len > 0 ? raw[0] : 0, raw_len > 1 ? raw[1] : 0,
                raw_len > 2 ? raw[2] : 0, raw_len > 3 ? raw[3] : 0,
                raw_len > 4 ? raw[4] : 0, raw_len > 5 ? raw[5] : 0,
                raw_len > 6 ? raw[6] : 0, raw_len > 7 ? raw[7] : 0);

    // Copy printable characters; replace non-printable with '.'
    size_t copy_len = raw_len < 8 ? raw_len : 8;
    for (size_t i = 0; i < copy_len; i++) {
        out_buf[i] = (raw[i] >= 0x20 && raw[i] < 0x7F)
                     ? static_cast<char>(raw[i]) : '.';
    }
    out_buf[copy_len] = '\0';
    return true;
}

} // namespace detail

// ---------------------------------------------------------------------------
// Public API — individual read functions
// ---------------------------------------------------------------------------

/// Read the safety module input diagnostics (0x2611) and FSoE active
/// indicator (0x2620:2) from a SOMANET drive.
inline SafetyModuleState readSafetyModuleState(::EtherCAT::Slave& slave) {
    SafetyModuleState state;

    const auto err1 = slave.sdoReadU8(
        kSafetyModuleDiagnosticsIndex, kSafetyModuleInput1Subindex,
        state.input1);
    const auto err2 = slave.sdoReadU8(
        kSafetyModuleDiagnosticsIndex, kSafetyModuleInput2Subindex,
        state.input2);

    state.ok = (err1 == ::EtherCAT::SlaveError::Ok) &&
               (err2 == ::EtherCAT::SlaveError::Ok);

    const auto err_fsoe = slave.sdoReadU8(
        kGeneralSafetyIndex, kSafeFieldbusSubindex,
        state.safe_fieldbus);
    state.fsoe_read_ok = (err_fsoe == ::EtherCAT::SlaveError::Ok);

    return state;
}

/// Read the FSoE safety address (0xF980:1) from a SOMANET drive.
inline ::EtherCAT::SlaveError readFSoESafetyAddress(
        ::EtherCAT::Slave& slave, uint16_t& safety_address) {
    return slave.sdoReadU16(kFSoESafetyAddressIndex,
                            kFSoESafetyAddressSubindex,
                            safety_address);
}

// ---------------------------------------------------------------------------
// Public API — comprehensive diagnostic
// ---------------------------------------------------------------------------

/**
 * @brief Run a comprehensive safety diagnostics query on a SOMANET drive.
 *
 * Reads all safety-related objects via SDO and logs a detailed report.
 * This function is designed to be called after the drive has reached
 * SAFE_OP or OP state, before starting the cyclic PDO exchange loop.
 *
 * The report includes:
 *   - Safety module input diagnostics (0x2611) — safe state inputs
 *   - General safety (0x2620) — FSoE active, safe address
 *   - Safety statusword (0x6621) — STO, SBC, SS1, SS2, SOS, SLS bits
 *   - Error report (0x203F) — 8-char fault code with human-readable decode
 *   - CiA 402 error code (0x603F) and statusword (0x6041)
 *   - FSoE master frame elements (0x6770) — what the slave received
 *   - FSoE slave frame elements (0x6760) — what the slave is sending
 *   - Module ident (0xF030/0xF050) — parameter configuration mode
 *   - Device safety address (0xF980)
 *   - Error register (0x1001)
 *
 * @param slave  Reference to the EtherCAT slave (SOMANET drive).
 * @return SafetyDiagnosticReport with all read values.  Individual `*_ok`
 *         fields indicate which reads succeeded.
 */
inline SafetyDiagnosticReport runFullSafetyDiagnostics(
        ::EtherCAT::Slave& slave) {

    static const char* TAG = "SafetyDiag";
    SafetyDiagnosticReport rpt;

    TETHER_LOGI(TAG, "========================================");
    TETHER_LOGI(TAG, "=== Safety System Diagnostics ===");
    TETHER_LOGI(TAG, "========================================");

    // --- 1. Safety module input diagnostics (0x2611) ---
    TETHER_LOGI(TAG, "--- Safety Module Input Diagnostics (0x2611) ---");
    {
        uint8_t v1 = 0, v2 = 0;
        bool ok1 = detail::readU8(slave, kSafetyModuleDiagnosticsIndex,
                                  kSafetyModuleInput1Subindex, v1, TAG);
        bool ok2 = detail::readU8(slave, kSafetyModuleDiagnosticsIndex,
                                  kSafetyModuleInput2Subindex, v2, TAG);
        rpt.inputs_ok = ok1 && ok2;
        rpt.input1 = v1;
        rpt.input2 = v2;
        if (rpt.inputs_ok) {
            TETHER_LOGI(TAG, "  Input 1 = %u (%s)", v1,
                        v1 ? "NOT safe state" : "SAFE STATE");
            TETHER_LOGI(TAG, "  Input 2 = %u (%s)", v2,
                        v2 ? "NOT safe state" : "SAFE STATE");
            if (rpt.isInSafeState()) {
                TETHER_LOGW(TAG, "  >> Drive is in SAFE STATE — motion inhibited!");
            } else {
                TETHER_LOGI(TAG, "  >> Drive is NOT in safe state — motion allowed");
            }
        } else {
            TETHER_LOGE(TAG, "  >> FAILED to read safety inputs — assuming safe state (fail-safe)");
        }
    }

    // --- 2. General safety (0x2620) ---
    TETHER_LOGI(TAG, "--- General Safety (0x2620) ---");
    {
        uint8_t fieldbus = 0;
        uint16_t addr = 0;
        bool ok1 = detail::readU8(slave, kGeneralSafetyIndex,
                                  kSafeFieldbusSubindex, fieldbus, TAG);
        bool ok2 = detail::readU16(slave, kGeneralSafetyIndex,
                                   kSafeAddressSubindex, addr, TAG);
        rpt.general_safety_ok = ok1 && ok2;
        rpt.safe_fieldbus = fieldbus;
        rpt.safe_address = addr;
        if (ok1) {
            TETHER_LOGI(TAG, "  0x2620:2 Safe fieldbus = 0x%02X (%s)",
                        fieldbus, fieldbus ? "FSoE ACTIVE" : "FSoE INACTIVE");
        }
        if (ok2) {
            TETHER_LOGI(TAG, "  0x2620:3 Safe address = 0x%04X", addr);
        }
        if (rpt.general_safety_ok && !rpt.fsoeActive()) {
            TETHER_LOGW(TAG, "  >> FSoE is NOT active on the drive!");
            TETHER_LOGW(TAG, "     The safety module will not process FSoE PDO data.");
            TETHER_LOGW(TAG, "     Check OBLAC Drives configuration: FSoE must be enabled.");
        }
    }

    // --- 3. Safety statusword (0x6621) ---
    TETHER_LOGI(TAG, "--- Safety Statusword (0x6621) ---");
    {
        uint8_t b1 = 0, b2 = 0;
        bool ok1 = detail::readU8(slave, kSafetyStatuswordIndex,
                                  kSafetyStatusByte1Sub, b1, TAG);
        bool ok2 = detail::readU8(slave, kSafetyStatuswordIndex,
                                  kSafetyStatusByte2Sub, b2, TAG);
        rpt.statusword_ok = ok1 && ok2;
        rpt.status_byte1 = b1;
        rpt.status_byte2 = b2;
        if (rpt.statusword_ok) {
            char dec1[80], dec2[80];
            decodeSafetyStatusByte1(b1, dec1, sizeof(dec1));
            decodeSafetyStatusByte2(b2, dec2, sizeof(dec2));
            TETHER_LOGI(TAG, "  0x6621:1 Status byte 1 = 0x%02X  [%s]", b1, dec1);
            TETHER_LOGI(TAG, "  0x6621:2 Status byte 2 = 0x%02X  [%s]", b2, dec2);
            // Highlight active safety functions
            if (b1 & 0x01) TETHER_LOGW(TAG, "  >> STO (Safe Torque Off) is ACTIVE");
            if (b1 & 0x02) TETHER_LOGI(TAG, "  >> SS1 (Safe Stop 1) is active");
            if (b1 & 0x04) TETHER_LOGI(TAG, "  >> SS2 (Safe Stop 2) is active");
            if (b1 & 0x08) TETHER_LOGI(TAG, "  >> SOS (Safe Operating Stop) is active");
            if (b2 & 0x01) TETHER_LOGW(TAG, "  >> SBC (Safe Brake Control) is ACTIVE");
        }
    }

    // --- 4. Error report (0x203F) — the most important diagnostic ---
    TETHER_LOGI(TAG, "--- Error Report (0x203F) ---");
    {
        rpt.error_report_ok = detail::readErrorReport(
            slave, rpt.error_report, sizeof(rpt.error_report), TAG);
        if (rpt.error_report_ok) {
            TETHER_LOGI(TAG, "  0x203F:1 Error report = '%s'",
                        rpt.error_report);
            const char* desc = knownErrorReport(rpt.error_report);
            if (desc) {
                if (rpt.hasFault()) {
                    TETHER_LOGE(TAG, "  >> FAULT: %s", desc);
                } else {
                    TETHER_LOGI(TAG, "  >> %s", desc);
                }
            } else if (rpt.hasFault()) {
                TETHER_LOGE(TAG, "  >> UNKNOWN FAULT CODE: '%s'", rpt.error_report);
                TETHER_LOGE(TAG, "     Check Synapticon documentation for this error code.");
            }
        } else {
            TETHER_LOGW(TAG, "  >> Could not read error report — SDO read failed");
        }
    }

    // --- 5. CiA 402 error code (0x603F) and statusword (0x6041) ---
    TETHER_LOGI(TAG, "--- CiA 402 Drive State (0x603F / 0x6041) ---");
    {
        uint16_t err_code = 0, statusword = 0;
        bool ok1 = detail::readU16(slave, kCiA402ErrorCodeIndex, 0,
                                    err_code, TAG);
        bool ok2 = detail::readU16(slave, kCiA402StatuswordIndex, 0,
                                    statusword, TAG);
        rpt.cia402_error_ok = ok1;
        rpt.cia402_status_ok = ok2;
        rpt.cia402_error_code = err_code;
        rpt.cia402_statusword = statusword;
        if (ok1) {
            if (err_code == 0) {
                TETHER_LOGI(TAG, "  0x603F Error code = 0x0000 (no error)");
            } else {
                TETHER_LOGE(TAG, "  0x603F Error code = 0x%04X (drive fault!)", err_code);
            }
        }
        if (ok2) {
            TETHER_LOGI(TAG, "  0x6041 Statusword = 0x%04X (%s)",
                        statusword, cia402StateName(statusword));
        }
    }

    // --- 6. FSoE master frame elements (0x6770) — what the slave received ---
    TETHER_LOGI(TAG, "--- FSoE Master Frame Elements (0x6770, slave received) ---");
    {
        uint8_t cmd = 0;
        uint16_t conn_id = 0, crc0 = 0, crc1 = 0;
        bool ok1 = detail::readU8(slave, kFSoEMasterFrameIndex,
                                  kFSoEMasterCmdSub, cmd, TAG);
        bool ok2 = detail::readU16(slave, kFSoEMasterFrameIndex,
                                   kFSoEMasterConnIDSub, conn_id, TAG);
        bool ok3 = detail::readU16(slave, kFSoEMasterFrameIndex,
                                   kFSoEMasterCRC0Sub, crc0, TAG);
        bool ok4 = detail::readU16(slave, kFSoEMasterFrameIndex,
                                   kFSoEMasterCRC1Sub, crc1, TAG);
        rpt.master_frame_ok = ok1 && ok2 && ok3 && ok4;
        rpt.master_cmd = cmd;
        rpt.master_conn_id = conn_id;
        rpt.master_crc0 = crc0;
        rpt.master_crc1 = crc1;
        if (rpt.master_frame_ok) {
            TETHER_LOGI(TAG, "  0x6770:1 Command    = 0x%02X (%s)",
                        cmd, fsoeCommandName(cmd));
            TETHER_LOGI(TAG, "  0x6770:2 ConnID     = 0x%04X", conn_id);
            TETHER_LOGI(TAG, "  0x6770:3 CRC_0      = 0x%04X", crc0);
            TETHER_LOGI(TAG, "  0x6770:4 CRC_1      = 0x%04X", crc1);
            if (cmd == 0) {
                TETHER_LOGW(TAG, "  >> Master command is 0x00 — slave is NOT receiving FSoE frames!");
                TETHER_LOGW(TAG, "     This indicates the FSoE PDO data is not reaching the safety module.");
                TETHER_LOGW(TAG, "     Possible causes: PDO mapping mismatch, ESC chip bug, or FSoE not active.");
            }
        }
    }

    // --- 7. FSoE slave frame elements (0x6760) — what the slave is sending ---
    TETHER_LOGI(TAG, "--- FSoE Slave Frame Elements (0x6760, slave sending) ---");
    {
        uint8_t cmd = 0;
        uint16_t conn_id = 0;
        bool ok1 = detail::readU8(slave, kFSoESlaveFrameIndex,
                                  kFSoESlaveCmdSub, cmd, TAG);
        bool ok2 = detail::readU16(slave, kFSoESlaveFrameIndex,
                                   kFSoESlaveConnIDSub, conn_id, TAG);
        rpt.slave_frame_ok = ok1 && ok2;
        rpt.slave_cmd = cmd;
        rpt.slave_conn_id = conn_id;
        if (rpt.slave_frame_ok) {
            TETHER_LOGI(TAG, "  0x6760:1 Command    = 0x%02X (%s)",
                        cmd, fsoeCommandName(cmd));
            TETHER_LOGI(TAG, "  0x6760:2 ConnID     = 0x%04X", conn_id);
            if (cmd == 0 && conn_id == 0) {
                TETHER_LOGW(TAG, "  >> Slave FSoE frame is all zeros — safety module is not producing FSoE output!");
            }
        }
    }

    // --- 8. Module identification (0xF030 / 0xF050) ---
    TETHER_LOGI(TAG, "--- Module Identification (0xF030 / 0xF050) ---");
    {
        uint32_t cfg_pos2 = 0, det_pos2 = 0;
        bool ok1 = detail::readU32(slave, kConfiguredModuleIdentIndex, 2,
                                    cfg_pos2, TAG);
        bool ok2 = detail::readU32(slave, kDetectedModuleIdentIndex, 2,
                                    det_pos2, TAG);
        rpt.module_ident_ok = ok1 && ok2;
        rpt.configured_ident_pos2 = cfg_pos2;
        rpt.detected_ident_pos2 = det_pos2;
        if (rpt.module_ident_ok) {
            TETHER_LOGI(TAG, "  0xF030:2 Configured = 0x%08X (%s)",
                        cfg_pos2, moduleIdentDesc(cfg_pos2));
            TETHER_LOGI(TAG, "  0xF050:2 Detected   = 0x%08X (%s)",
                        det_pos2, moduleIdentDesc(det_pos2));
            if (rpt.moduleIdentMismatch()) {
                TETHER_LOGE(TAG, "  >> Module ident MISMATCH! Configured != Detected");
                TETHER_LOGE(TAG, "     This may indicate a hardware or configuration problem.");
            }
        }
    }

    // --- 9. Device safety address (0xF980) ---
    TETHER_LOGI(TAG, "--- Device Safety Address (0xF980) ---");
    {
        uint16_t addr = 0;
        bool ok = detail::readU16(slave, kFSoESafetyAddressIndex,
                                  kFSoESafetyAddressSubindex, addr, TAG);
        rpt.safety_addr_ok = ok;
        rpt.device_safety_address = addr;
        if (ok) {
            TETHER_LOGI(TAG, "  0xF980:1 Safety address = 0x%04X", addr);
            if (rpt.general_safety_ok && rpt.connIdMismatch()) {
                TETHER_LOGE(TAG, "  >> Connection ID MISMATCH!");
                TETHER_LOGE(TAG, "     Device safety address (0xF980) = 0x%04X",
                            rpt.device_safety_address);
                TETHER_LOGE(TAG, "     General safety address (0x2620:3) = 0x%04X",
                            rpt.safe_address);
                TETHER_LOGE(TAG, "     The master must use the device safety address as the FSoE connection ID.");
            }
        }
    }

    // --- 10. Error register (0x1001) ---
    TETHER_LOGI(TAG, "--- Error Register (0x1001) ---");
    {
        uint8_t reg = 0;
        bool ok = detail::readU8(slave, kErrorRegisterIndex, 0,
                                 reg, TAG);
        rpt.error_reg_ok = ok;
        rpt.error_register = reg;
        if (ok) {
            TETHER_LOGI(TAG, "  0x1001:0 Error register = 0x%02X", reg);
            if (reg & 0x01) TETHER_LOGW(TAG, "  >> Generic error bit set");
            if (reg & 0x02) TETHER_LOGW(TAG, "  >> Device profile error bit set");
            if (reg & 0x04) TETHER_LOGW(TAG, "  >> Communication error bit set");
            if (reg & 0x08) TETHER_LOGW(TAG, "  >> Device-specific error bit set");
        }
    }

    // --- Summary ---
    TETHER_LOGI(TAG, "========================================");
    TETHER_LOGI(TAG, "=== Safety Diagnostics Summary ===");
    TETHER_LOGI(TAG, "========================================");

    if (rpt.hasFault()) {
        TETHER_LOGE(TAG, "  FAULT: '%s'", rpt.error_report);
        const char* desc = knownErrorReport(rpt.error_report);
        if (desc) TETHER_LOGE(TAG, "         %s", desc);
    } else if (rpt.error_report_ok) {
        TETHER_LOGI(TAG, "  No fault reported (NoFault)");
    } else {
        TETHER_LOGW(TAG, "  Could not read error report");
    }

    if (rpt.isInSafeState()) {
        TETHER_LOGW(TAG, "  Drive is in SAFE STATE (motion inhibited)");
    } else if (rpt.inputs_ok) {
        TETHER_LOGI(TAG, "  Drive is NOT in safe state (motion allowed)");
    }

    if (rpt.fsoeActive()) {
        TETHER_LOGI(TAG, "  FSoE is active on the drive");
    } else if (rpt.general_safety_ok) {
        TETHER_LOGW(TAG, "  FSoE is NOT active on the drive");
    }

    if (rpt.master_frame_ok && rpt.master_cmd == 0) {
        TETHER_LOGW(TAG, "  Slave is NOT receiving FSoE frames (master cmd = 0x00)");
    }

    if (rpt.slave_frame_ok && rpt.slave_cmd == 0 && rpt.slave_conn_id == 0) {
        TETHER_LOGW(TAG, "  Slave is NOT producing FSoE output (all zeros)");
    }

    if (rpt.moduleIdentMismatch()) {
        TETHER_LOGE(TAG, "  Module ident mismatch detected");
    }

    if (rpt.connIdMismatch()) {
        TETHER_LOGE(TAG, "  Connection ID mismatch (0xF980 != 0x2620:3)");
    }

    TETHER_LOGI(TAG, "========================================");

    return rpt;
}

} // namespace Synapticon
} // namespace Drives
} // namespace EtherCAT

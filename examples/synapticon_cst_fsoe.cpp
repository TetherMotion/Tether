/**
 * @file synapticon_cst_fsoe.cpp
 * @brief Synapticon SOMANET drive  -  FSoE safe-motion example (no motion)
 *
 * Interfaces to a Synapticon SOMANET drive (Vendor 0x22D2, CiA 402 firmware
 * v5.1.x) and runs the FSoE safe-motion protocol over the drive's safety
 * PDOs (RxPDO 0x1700 / TxPDO 0x1B00).  By default no CiA 402 motion is
 * performed  -  the drive is never enabled and no torque/velocity/position
 * commands are sent.  Pass --torque-nm to enable CST sine-torque motion
 * (same algorithm as synapticon_cst): the torque command is gated on the
 * FSoE status reporting STO off / motion allowed, and the drive is only
 * enabled once the brake has been released.
 *
 * On startup the slave is automatically reset to INIT if it is currently
 * in a higher ESM state (e.g. left over from a previous run that didn't
 * shut down cleanly).  This ensures a clean starting point for mailbox
 * configuration and PDO mapping.
 *
 * FSoE frames are exchanged each cycle with the REAL drive via the FSoE
 * safety PDOs, which are mapped alongside the CiA 402 motion PDOs using
 * the multi-PDO-per-sync-manager API (the combined assignment is required
 * by the drive firmware).  The FSoE master state machine (MainInstance)
 * builds the command frame into the RxPDO 0x1700 buffer and processes the
 * drive's safety status frame from the TxPDO 0x1B00 buffer each cycle.
 * The drive's safety firmware handles the slave side of the FSoE protocol.
 *
 * Rich diagnostics are printed each cycle:
 *   - Drive statusword, mode display, actual torque/position
 *   - FSoE connection state, error codes, frame/watchdog statistics
 *   - Safe-motion status (STO, SS1, SS2, SOS, motion-allowed)
 *
 * Usage (Linux, requires root or CAP_NET_RAW):
 *   ./synapticon_cst_fsoe                       # auto-select iface, slave 0, 10 s
 *   ./synapticon_cst_fsoe -i enx34298f762c4e    # specify interface
 *   ./synapticon_cst_fsoe -s 1 -d 30            # slave 1, 30 s
 *   ./synapticon_cst_fsoe --dc-sync             # enable DC synchronization
 *   ./synapticon_cst_fsoe --connection-id 0x0006 --safety-address 0x0006 --watchdog-ms 15
 *   ./synapticon_cst_fsoe --debug fsoe          # high-level FSoE protocol trace
 *   ./synapticon_cst_fsoe --debug fsoe-frame    # decoded FSoE PDO struct fields (on change)
 *   ./synapticon_cst_fsoe --debug fsoe-raw      # FSoE protocol trace + raw frame hex dumps (on change)
 *   ./synapticon_cst_fsoe --debug fsoe-wire     # every-cycle PDO wire dumps (firehose)
 *   ./synapticon_cst_fsoe --debug fsoe-sequence # per-cycle frame accept/reject + state change summary
 *   ./synapticon_cst_fsoe --debug fsoe-crc      # CRC parameters used for TX build and RX check
 *   ./synapticon_cst_fsoe --debug rx-pdo,tx-pdo # EtherCAT PDO data logging
 *   ./synapticon_cst_fsoe --debug coe-reads,coe-writes  # SDO access logging
 *   ./synapticon_cst_fsoe --debug al-state,pdo-sm       # state machine + SM config
 *   ./synapticon_cst_fsoe --debug help         # list all available debug flags
 *   ./synapticon_cst_fsoe --sto 0 --sbc 0               # raw STO bit=0, SBC bit=0
 *   ./synapticon_cst_fsoe --sto 1 --sbc 1               # raw STO bit=1, SBC bit=1
 *   ./synapticon_cst_fsoe --sto 1 --sos 1 --sbc 1       # raw STO/SOS/SBC bit=1
 *   ./synapticon_cst_fsoe --diagnostics-after 5         # FSoE 5s, then CoE diagnostics + exit
 *   ./synapticon_cst_fsoe --torque-nm 1.0 --freq-hz 0.5 # FSoE + 1 Nm P-P CST sine torque
 *   ./synapticon_cst_fsoe --torque-nm 1.0 --rated-torque-mnm 4200  # override rated torque
 */

#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <future>
#include <iostream>
#include <memory>
#include <string>
#include <vector>

#include "DS402ExampleSupport.hpp"
#include "common/ExampleHelpers.hpp"
#include "tether/drives/Synapticon.hpp"
#include "tether/drives/Synapticon/SynapticonPDO.hpp"
#include "tether/drives/Synapticon/SafetyDiagnostics.hpp"
#include "tether/ethercat/ALResetController.hpp"
#include "tether/ethercat/FaultDetection.hpp"
#include "tether/ethercat/Slave.hpp"
#include "tether/fsoe/FSoEDefs.hpp"
#include "tether/profiles/cia402/CiA402BitLabels.hpp"
#include "tether/fsoe/FSoEHelpers.hpp"
#include "tether/fsoe/Synapticon/SafeMotionFSoE.hpp"
#include "tether/fsoe/Synapticon/FSoEPDODecoder.hpp"
#include "tether/platform/EspCompat.hpp"
#include "tether/profiles/cia301/CiA402Defs.hpp"
#include "tether/profiles/cia402/DS402Master.hpp"
#include "tether/ethercat/CoEManager.hpp"

#include <argparse/argparse.hpp>

namespace {

constexpr const char* TAG = "synapticon_cst_fsoe";

// Format a failed CoE transaction for a one-line log message:
//   abort   -> "Object does not exist (0x06020000)"
//   other   -> "Timeout" / "Transport error" / ...
// coeErrorStr() already decodes the slave's SDO abort code, so the caller
// only needs this to append the raw code for aborts.
static std::string coeErrorDetail(const EtherCAT::CoE::CoEError& e) {
    std::string s = EtherCAT::CoE::coeErrorStr(e);
    if (e.code == EtherCAT::CoE::CoEErrorCode::Aborted && e.abort_code != 0) {
        char buf[20];
        std::snprintf(buf, sizeof(buf), " (0x%08X)", e.abort_code);
        s += buf;
    }
    return s;
}

// ============================================================================
// Mailbox settings  -  from SOMANET_CiA_402_v5.1.9.xml (ESI)
// ============================================================================
//
// All three SOMANET devices in the ESI (SOMANET Node ProductCode 0x0201,
// SOMANET Circulo 0x0301, and the third device) share identical mailbox
// sync-manager definitions:
//
//   <Sm DefaultSize="1024" StartAddress="#x1000" ControlByte="#x26" ...>MBoxOut</Sm>
//   <Sm DefaultSize="1024" StartAddress="#x1400" ControlByte="#x22" ...>MBoxIn</Sm>
//
// The ControlByte values are the authoritative direction indicators:
//   0x26 = MAILBOX | DIR_WRITE | WATCHDOG  → master writes → SM0 (M→S)
//   0x22 = MAILBOX | DIR_READ  | WATCHDOG  → master reads  → SM1 (S→M)
//
// So despite the ESI "MBoxOut"/"MBoxIn" labels (master-perspective naming),
// the hardware mapping is:
//   SM0 (write, master→slave): addr=0x1000, size=1024
//   SM1 (read,  slave→master): addr=0x1400, size=1024
//
// Mailbox protocols (from <Mailbox DataLinkLayer="1">):
//   <CoE SdoInfo="1" PdoAssign="1" PdoConfig="1" .../>
//   <FoE/>
//   → CoE (0x0004) | FoE (0x0008) = 0x000C
//
// Mailbox timeouts (from <Info><Mailbox><Timeout>):
//   RequestTimeout  = 100 ms
//   ResponseTimeout = 6000 ms
//
// These constants are now provided by the Synapticon driver header
// (tether/drives/Synapticon.hpp) and are re-exported here for readability.

// SM0  -  master→slave write mailbox (ESI "MBoxOut", ControlByte 0x26)
constexpr uint16_t kMailboxWriteAddr = EtherCAT::Drives::Synapticon::kMailboxWriteAddr;
// NOTE: The SOMANET ESI advertises 1024-byte mailbox buffers, but the
// firmware only accepts 512 bytes.  The driver header now defines
// kMailboxWriteSize/kMailboxReadSize as 512 to match the firmware.
constexpr uint16_t kMailboxWriteSize = EtherCAT::Drives::Synapticon::kMailboxWriteSize;

// SM1  -  slave→master read mailbox (ESI "MBoxIn", ControlByte 0x22)
constexpr uint16_t kMailboxReadAddr = EtherCAT::Drives::Synapticon::kMailboxReadAddr;
constexpr uint16_t kMailboxReadSize = EtherCAT::Drives::Synapticon::kMailboxReadSize;

// CoE | FoE
constexpr uint16_t kMailboxProtocols = EtherCAT::Drives::Synapticon::kMailboxProtocols;

// SDO response timeout from ESI ResponseTimeout (6000 ms)
constexpr uint32_t kSdoTimeoutMs = EtherCAT::Drives::Synapticon::kSdoTimeoutMs;

// SOMANET PDO types  -  from SynapticonPDO.hpp (extracted from ESI)
//
// CiA 402 motion PDOs (CST mode):
//   RxPDO 0x1600: controlword, modes_of_operation, target_torque,
//                 target_position, target_velocity, torque_offset,
//                 tuning_command  (19 bytes)
//   TxPDO 0x1A00: statusword, modes_of_operation_display, position_actual,
//                 velocity_actual, torque_actual  (13 bytes)
//
// FSoE safety PDOs:
//   RxPDO 0x1700: FSoE command frame (master→slave, 11 bytes)
//   TxPDO 0x1B00: FSoE status frame (slave→master, 35 bytes)
//
// Both sets are mapped simultaneously using the multi-PDO-per-sync-manager
// API.  ALL PDOs (including FSoE) are written explicitly to 0x1C12/0x1C13.
// FSoE PDOs come FIRST in the assignment order, then motion PDOs.
//   SM2 (Rx, master→slave): [0x1700 (11B)][0x1600 (19B)][0x1601][0x1602] = 46 bytes
//   SM3 (Tx, slave→master): [0x1B00 (35B)][0x1A00 (13B)][0x1A01][0x1A02][0x1A03] = 82 bytes
//
// FSoE PDOs come FIRST in the assignment order.  This is critical because
// the Synapticon Circulo EtherCAT chip has a bug where the last word in the
// SM buffer is zeroed.  If the FSoE PDO were last, the ConnectionID (the
// final word of the FSoE frame) would be zeroed and the slave would reject
// every frame.  By placing motion PDOs last, the zeroed word falls on
// motion data, not the FSoE ConnectionID.
// See: https://doc.synapticon.com/circulo_safe_motion/smm/ecat_fsoe_issues.htm
using RxPDO = EtherCAT::Drives::SynapticonPDO::SOMANET_RxPDO_1600;
using TxPDO = EtherCAT::Drives::SynapticonPDO::SOMANET_TxPDO_1A00;
using FSoERxPDO = EtherCAT::Drives::SynapticonPDO::SOMANET_RxPDO_1700;
using FSoETxPDO = EtherCAT::Drives::SynapticonPDO::SOMANET_TxPDO_1B00;
using FSoETxPDOLW1 = EtherCAT::Drives::SynapticonPDO::SOMANET_TxPDO_1B00_LW1;
using FSoEFrameVariant = EtherCAT::Drives::SynapticonPDO::FSoEFrameVariant;

// PDO offsets within the combined PDO buffer.
// FSoE PDOs come FIRST (offset 0), motion PDOs follow after the FSoE PDO.
//   SM2: 0x1700 (11B) first, then 0x1600 (19B) at offset 11
//   SM3: 0x1B00 (35B LW2 / 31B LW1) first, then 0x1A00 (13B)
//
// The slave→master FSoE frame size depends on the configured frame
// variant (--fsoe-frame lw1|lw2): LW2 carries a 2-byte "safe torque data"
// word + CRC appended before the ConnectionID (35 bytes), LW1 does not
// (31 bytes).  The FSoE TxPDO size, the motion TxPDO offset and the SM3
// total length are therefore runtime values derived from the variant.
constexpr size_t kFSoERxPDOOffset   = 0;                    // FSoE first
constexpr size_t kMotionRxPDOOffset = sizeof(FSoERxPDO);    // 11 bytes
constexpr size_t kFSoETxPDOOffset   = 0;                    // FSoE first

// Total SM2 length (FSoE + all motion PDOs — same for both variants).
constexpr size_t kSM2TotalLen = EtherCAT::Drives::SynapticonPDO::kSM2CombinedSize;   // 46

using FSoEMain = EtherCAT::Drives::Synapticon::SafeMotion::MainInstance;

// ============================================================================
// Sine-wave torque motion controller for CST mode (--torque-nm)
// ============================================================================
//
// Same algorithm as synapticon_cst's SineTorqueController: generates a
// sinusoidal torque command at the specified frequency and peak-to-peak
// amplitude, converted from Nm to CiA 402 per-mille units (0.1% of rated
// torque) using the motor's rated torque (object 0x6076, in mNm):
//
//   target_torque_permille = (torque_Nm * 1'000'000) / rated_torque_mNm
//
// Two differences from the non-FSoE variant:
//   - The motion RxPDO (0x1600) is not at buffer offset 0  -  the FSoE PDO
//     (0x1700) comes first in the combined SM2 buffer, so all writes go
//     through rx_pdo_offset.
//   - Torque output is gated on the FSoE link: while the connection is not
//     operational or the drive reports motion not allowed (STO/SS1/SOS
//     active), the commanded torque is forced to zero.  The controlword is
//     still written so the CiA 402 enable sequence can proceed once the
//     safety layer clears.

class SineTorqueController final : public EtherCAT::DS402Master::IDriveMotionController {
public:
    /// @param amplitude_nm   Peak torque amplitude in Nm (half of peak-to-peak)
    /// @param frequency_hz   Sine frequency in Hz
    /// @param rated_torque_mnm  Motor rated torque in mNm (from object 0x6076)
    /// @param rx_pdo_offset  Byte offset of the motion RxPDO (0x1600) in the
    ///                       combined PDO buffer (after the FSoE PDO)
    /// @param fsoe_main      FSoE main instance used to gate torque output
    SineTorqueController(double amplitude_nm, double frequency_hz,
                         uint32_t rated_torque_mnm, size_t rx_pdo_offset,
                         const FSoEMain& fsoe_main)
        : amplitude_nm_(amplitude_nm)
        , frequency_hz_(frequency_hz)
        , rated_torque_mnm_(rated_torque_mnm)
        , rx_pdo_offset_(rx_pdo_offset)
        , fsoe_main_(fsoe_main)
    {
        // Pre-compute the per-mille scaling factor:
        //   permille = Nm * 1e6 / mNm
        if (rated_torque_mnm > 0) {
            nm_to_permille_ = 1'000'000.0 / static_cast<double>(rated_torque_mnm);
        }
    }

    bool start(EtherCAT::CiA402Drive& drive) override {
        return drive.setOperatingMode(static_cast<int8_t>(CiA402::OperatingMode::CyclicSyncTorque));
    }

    void stop(EtherCAT::CiA402Drive&) override {}

    bool update(EtherCAT::CiA402Drive& drive, double dt_seconds) override {
        auto* base = static_cast<uint8_t*>(drive.getRxPDOBuffer());
        if (base == nullptr) return false;
        auto* rx = reinterpret_cast<RxPDO*>(base + rx_pdo_offset_);

        // Advance the phase accumulator
        elapsed_s_ += dt_seconds;

        // Compute the sine torque command in Nm, then convert to per-mille
        double torque_nm =
            amplitude_nm_ * std::sin(2.0 * M_PI * frequency_hz_ * elapsed_s_);

        // Gate on the FSoE status: no torque unless the connection is
        // operational and the drive reports motion allowed (STO off).
        const auto conn_status = fsoe_main_.rawConnection().getStatus();
        const bool motion_ok =
            conn_status.isOperational() &&
            fsoe_main_.hasStatus() &&
            fsoe_main_.status().motionAllowed();
        if (!motion_ok) {
            torque_nm = 0.0;
        }

        const double torque_permille = torque_nm * nm_to_permille_;

        // Clamp to INT16 range (-32768..32767 per-mille = ±3276.8% rated)
        constexpr double kMaxPermille = 32767.0;
        constexpr double kMinPermille = -32768.0;
        const double clamped = (torque_permille > kMaxPermille) ? kMaxPermille :
                               (torque_permille < kMinPermille) ? kMinPermille :
                               torque_permille;

        rx->controlword = static_cast<uint16_t>(CiA402::ControlWord::EnableOperation);
        rx->modes_of_operation = static_cast<int8_t>(CiA402::OperatingMode::CyclicSyncTorque);
        rx->target_torque = static_cast<int16_t>(clamped);
        rx->target_velocity = 0;
        rx->target_position = 0;

        last_torque_nm_ = torque_nm;
        return true;
    }

    /// Returns the most recently commanded torque in Nm (for diagnostics)
    double lastTorqueNm() const { return last_torque_nm_; }

private:
    double   amplitude_nm_      = 0.0;
    double   frequency_hz_      = 0.0;
    uint32_t rated_torque_mnm_  = 0;
    size_t   rx_pdo_offset_     = 0;
    const FSoEMain& fsoe_main_;
    double   nm_to_permille_    = 0.0;  // per-mille per Nm
    double   elapsed_s_         = 0.0;
    double   last_torque_nm_    = 0.0;
};

// Reusable FSoE PDO decoding/logging helpers (from the Synapticon FSoE driver).
namespace fsoe_dbg = EtherCAT::Drives::Synapticon::FSoEDebug;

// ============================================================================
// General hex dump helper (used by the FSoE trace callback)
// ============================================================================

void hexDump(const char* tag, const char* label, const uint8_t* data, size_t len) {
    constexpr size_t kBytesPerLine = 16;
    char hex[kBytesPerLine * 3 + 1];
    for (size_t i = 0; i < len; i += kBytesPerLine) {
        size_t pos = 0;
        for (size_t j = i; j < i + kBytesPerLine && j < len; j++) {
            pos += snprintf(hex + pos, sizeof(hex) - pos, "%02X ", data[j]);
        }
        TETHER_LOGI(tag, "  {} [{:3}/{:3}]: {}", label, i, len, hex);
    }
}

// ============================================================================
// FSoE + drive diagnostics cyclic task
// ============================================================================

class FSoEDiagnosticsTask final : public EtherCAT::DS402Master::ICyclicTask {
public:
    FSoEDiagnosticsTask(uint16_t slave_index,
                        FSoEMain& fsoe_main,
                        uint32_t interval_ms,
                        size_t motion_tx_pdo_offset)
        : slave_index_(slave_index)
        , fsoe_main_(fsoe_main)
        , interval_ms_(interval_ms)
        , motion_tx_pdo_offset_(motion_tx_pdo_offset)
    {
    }

    bool update(EtherCAT::DS402Master& master, double dt_seconds) override {
        elapsed_ms_ += static_cast<uint64_t>(dt_seconds * 1000.0);
        if (elapsed_ms_ - last_print_ms_ < interval_ms_) return true;
        last_print_ms_ = elapsed_ms_;

        auto* drive = master.driveBySlaveIndex(slave_index_);
        if (drive == nullptr) return true;

        // Motion PDO is at motion_tx_pdo_offset_ (FSoE comes first, motion second)
        auto* tx = reinterpret_cast<const TxPDO*>(
            static_cast<const uint8_t*>(drive->getTxPDOBuffer()) + motion_tx_pdo_offset_);
        // Also read the commanded target_torque from the RxPDO for comparison
        auto* rx = reinterpret_cast<const RxPDO*>(
            static_cast<const uint8_t*>(drive->getRxPDOBuffer()) + kMotionRxPDOOffset);
        if (tx) {
            TETHER_LOGI(TAG,
                "--- Drive @ {} ms ---",
                static_cast<unsigned long long>(elapsed_ms_));
            TETHER_LOGI(TAG,
                "  statusword=0x{:04X} [{}] mode_display={} "
                "target_torque={} torque_actual={} "
                "position_actual={}",
                tx->statusword,
                CiA402::kStatuswordFormatter.formatActive(tx->statusword),
                static_cast<int>(tx->modes_of_operation_display),
                rx ? static_cast<int>(rx->target_torque) : 0,
                static_cast<int>(tx->torque_actual),
                static_cast<long long>(tx->position_actual));
        }

        auto& conn = fsoe_main_.rawConnection();
        const auto status = conn.getStatus();
        const auto stats = conn.getStats();

        TETHER_LOGI(TAG,
            "--- FSoE @ {} ms ---",
            static_cast<unsigned long long>(elapsed_ms_));
        TETHER_LOGI(TAG,
            "  state={}({}) operational={} fail_safe={} data_valid={}",
            FSoE::fsoeStateName(status.state), status.state,
            status.isOperational() ? 1 : 0,
            status.isFailSafe() ? 1 : 0,
            status.data_valid ? 1 : 0);
        TETHER_LOGI(TAG,
            "  session_id=0x{:04X} rx_seq={} watchdog={} ms",
            status.session_id, status.sequence_number,
            status.watchdog_counter);
        if (status.hasError()) {
            TETHER_LOGW(TAG,
                "  ERROR: 0x{:04X} ({})",
                status.error_code, FSoE::fsoeErrorName(status.error_code));
        }
        TETHER_LOGI(TAG,
            "  frames: tx={} rx={} | crc_err={} seq_err={} watchdog_evt={} "
            "reset_evt={} timeout_evt={} dup={} invalid={}",
            stats.frames_sent, stats.frames_received,
            stats.crc_errors, stats.sequence_errors, stats.watchdog_events,
            stats.reset_events, stats.timeout_events,
            stats.duplicate_frames, stats.invalid_frames);
        TETHER_LOGI(TAG,
            "  recovery: attempts={} successful={}",
            stats.recovery_attempts, stats.successful_recoveries);

        if (fsoe_main_.hasStatus()) {
            const auto& sm = fsoe_main_.status();
            TETHER_LOGI(TAG,
                "  safe-motion: motion_allowed={} sto={} ss1={} ss2={} "
                "sos={} error={}",
                sm.motionAllowed() ? 1 : 0,
                sm.sto_active ? 1 : 0,
                sm.ss1_active ? 1 : 0,
                sm.ss2_active ? 1 : 0,
                sm.sos_active ? 1 : 0,
                sm.error_active ? 1 : 0);
        }

        return true;
    }

private:
    uint16_t slave_index_;
    FSoEMain& fsoe_main_;
    uint32_t interval_ms_;
    size_t motion_tx_pdo_offset_;
    uint64_t elapsed_ms_ = 0;
    uint64_t last_print_ms_ = 0;
};

// ============================================================================
// Safety state confirmation monitor
// ============================================================================
//
// Monitors the drive's FSoE status feedback to confirm that commanded STO/SBC
// state changes have been acknowledged by the drive.  When the operator uses
// --sto / --sbc overrides, this task logs:
//   - When the command is first applied (commanded state)
//   - When the drive confirms the state (status matches command)
//   - A warning if the drive does NOT confirm within a timeout
//
// The monitor tracks the expected state from the Command struct and compares
// it against the Status struct each cycle once FSoE reaches the Data state.

class SafetyStateMonitor final : public EtherCAT::DS402Master::ICyclicTask {
public:
    SafetyStateMonitor(FSoEMain& fsoe_main,
                       bool track_sto, bool track_sbc,
                       bool expected_sto, bool expected_sbc,
                       uint32_t confirm_timeout_ms = 3000)
        : fsoe_main_(fsoe_main)
        , track_sto_(track_sto)
        , track_sbc_(track_sbc)
        , expected_sto_(expected_sto)
        , expected_sbc_(expected_sbc)
        , confirm_timeout_ms_(confirm_timeout_ms)
    {
    }

    bool update(EtherCAT::DS402Master& /*master*/, double dt_seconds) override {
        if (!track_sto_ && !track_sbc_) return true;

        elapsed_ms_ += static_cast<uint64_t>(dt_seconds * 1000.0);

        // Only check once FSoE has reached Data state and we have status.
        if (!fsoe_main_.hasStatus()) return true;
        const auto& conn_status = fsoe_main_.rawConnection().getStatus();
        if (!conn_status.isOperational()) return true;

        // Record the timestamp when we first see Data state.
        if (data_state_entered_ms_ == 0) {
            data_state_entered_ms_ = elapsed_ms_;
        }

        const auto& sm = fsoe_main_.status();

        // --- STO confirmation ---
        if (track_sto_ && !sto_confirmed_) {
            // STO is "active" in the status when torque is inhibited.
            // We expect sto_active == expected_sto_.
            if (sm.sto_active == expected_sto_) {
                sto_confirmed_ = true;
                TETHER_LOGI(TAG,
                    "[safety-confirm] STO {} confirmed by drive after {} ms "
                    "(sto_active={})",
                    expected_sto_ ? "ON (torque inhibited)" : "OFF (torque allowed)",
                    static_cast<unsigned long long>(
                        elapsed_ms_ - data_state_entered_ms_),
                    sm.sto_active ? 1 : 0);
            } else if (!sto_warned_ &&
                       (elapsed_ms_ - data_state_entered_ms_) > confirm_timeout_ms_) {
                sto_warned_ = true;
                TETHER_LOGW(TAG,
                    "[safety-confirm] STO NOT confirmed after {} ms! "
                    "Commanded STO={} but drive reports sto_active={} "
                    "(expected {}). The drive may be ignoring the command "
                    "bit or using a different polarity/position.",
                    confirm_timeout_ms_,
                    expected_sto_ ? "ON" : "OFF",
                    sm.sto_active ? 1 : 0,
                    expected_sto_ ? 1 : 0);
            }
        }

        // --- SBC confirmation ---
        if (track_sbc_ && !sbc_confirmed_) {
            // brake_engaged in status reflects the SBC state.
            if (sm.brake_engaged == expected_sbc_) {
                sbc_confirmed_ = true;
                TETHER_LOGI(TAG,
                    "[safety-confirm] SBC {} confirmed by drive after {} ms "
                    "(brake_engaged={})",
                    expected_sbc_ ? "ENGAGED" : "RELEASED",
                    static_cast<unsigned long long>(
                        elapsed_ms_ - data_state_entered_ms_),
                    sm.brake_engaged ? 1 : 0);
            } else if (!sbc_warned_ &&
                       (elapsed_ms_ - data_state_entered_ms_) > confirm_timeout_ms_) {
                sbc_warned_ = true;
                TETHER_LOGW(TAG,
                    "[safety-confirm] SBC NOT confirmed after {} ms! "
                    "Commanded SBC={} but drive reports brake_engaged={} "
                    "(expected {}). The brake may not be responding or the "
                    "drive uses a different bit polarity/position.",
                    confirm_timeout_ms_,
                    expected_sbc_ ? "ENGAGED" : "RELEASED",
                    sm.brake_engaged ? 1 : 0,
                    expected_sbc_ ? 1 : 0);
            }
        }

        return true;
    }

private:
    FSoEMain& fsoe_main_;
    bool track_sto_;
    bool track_sbc_;
    bool expected_sto_;
    bool expected_sbc_;
    uint32_t confirm_timeout_ms_;
    uint64_t elapsed_ms_ = 0;
    uint64_t data_state_entered_ms_ = 0;
    bool sto_confirmed_ = false;
    bool sbc_confirmed_ = false;
    bool sto_warned_ = false;
    bool sbc_warned_ = false;
};

// ============================================================================
// FSoE PDO exchange cyclic task (real drive via PDOs)
// ============================================================================
//
// Exchanges FSoE frames with the real Synapticon drive via the FSoE safety
// PDOs (RxPDO 0x1700 / TxPDO 0x1B00) that are mapped alongside the CiA 402
// motion PDOs in the combined PDO buffer.  The FSoE PDOs are at a fixed
// offset within the buffer (after the motion PDO data).

class FSoEPDOExchangeTask final : public EtherCAT::DS402Master::ICyclicTask {
public:
    FSoEPDOExchangeTask(uint16_t slave_index,
                        FSoEMain& main_instance,
                        size_t rx_pdo_offset,
                        size_t tx_pdo_offset,
                        FSoEFrameVariant fsoe_variant,
                        size_t sm3_total_len,
                        bool debug_raw = false,
                        bool debug_frame = false,
                        bool debug_wire = false)
        : slave_index_(slave_index)
        , main_instance_(main_instance)
        , rx_pdo_offset_(rx_pdo_offset)
        , tx_pdo_offset_(tx_pdo_offset)
        , fsoe_variant_(fsoe_variant)
        , fsoe_tx_size_(EtherCAT::Drives::SynapticonPDO::fsoeTxPDOSize(fsoe_variant))
        , sm3_total_len_(sm3_total_len)
        , debug_raw_(debug_raw)
        , debug_frame_(debug_frame)
        , debug_wire_(debug_wire)
    {}

    /// Suppress all FSoE debug output from this task (for diagnostics mode).
    void suppressDebug() {
        debug_raw_ = false;
        debug_frame_ = false;
        debug_wire_ = false;
    }

    bool update(EtherCAT::DS402Master& master, double dt_seconds) override {
        if (!main_instance_.featureEnabled()) {
            return true;
        }

        elapsed_time_ms_ += static_cast<uint64_t>(dt_seconds * 1000.0);

        auto* drive = master.driveBySlaveIndex(slave_index_);
        if (drive == nullptr) {
            return true;  // don't stop the process
        }

        // Access the FSoE PDO region within the combined PDO buffer.
        // The FSoE PDO (0x1700/0x1B00) comes FIRST at offset 0; the motion
        // PDO (0x1600/0x1A00) follows after the FSoE PDO.
        uint8_t* rx_buffer = static_cast<uint8_t*>(drive->getRxPDOBuffer()) + rx_pdo_offset_;
        const uint8_t* tx_buffer = static_cast<const uint8_t*>(drive->getTxPDOBuffer()) + tx_pdo_offset_;

        // --debug fsoe-wire: dump every cycle, unconditionally.
        // This is the "firehose" mode for seeing the raw PDO wire bytes
        // on every single cycle, even when nothing changes.
        if (debug_wire_) {
            dumpWire(tx_buffer, rx_buffer);
        }

        // --debug fsoe-raw / fsoe-frame: dump only when the FSoE frame
        // content changes.  Compares the current TxPDO (slave-to-master) and
        // RxPDO (master-to-slave) FSoE regions against the last seen copies.
        const bool tx_changed = (debug_raw_ || debug_frame_) &&
            std::memcmp(tx_buffer, last_tx_.data(), fsoe_tx_size_) != 0;
        if (tx_changed) {
            std::memcpy(last_tx_.data(), tx_buffer, fsoe_tx_size_);
        }

        // --debug fsoe-raw: verbose multi-line struct dump (CRCs, raw hex, etc.)
        if (debug_raw_ && tx_changed) {
            if (fsoe_variant_ == FSoEFrameVariant::LW2) {
                fsoe_dbg::dumpTxPDO(TAG,
                    *reinterpret_cast<const FSoETxPDO*>(tx_buffer));
            } else {
                fsoe_dbg::dumpTxPDO(TAG,
                    *reinterpret_cast<const FSoETxPDOLW1*>(tx_buffer));
            }
        }

        // --debug fsoe-frame: compact one-line interpretation on change
        if (debug_frame_ && tx_changed) {
            if (fsoe_variant_ == FSoEFrameVariant::LW2) {
                fsoe_dbg::dumpTxPDOFrame(TAG,
                    *reinterpret_cast<const FSoETxPDO*>(tx_buffer));
            } else {
                fsoe_dbg::dumpTxPDOFrame(TAG,
                    *reinterpret_cast<const FSoETxPDOLW1*>(tx_buffer));
            }
        }

        const bool ok = main_instance_.exchangeViaPDO(
            rx_buffer, sizeof(FSoERxPDO),
            tx_buffer, fsoe_tx_size_,
            elapsed_time_ms_);

        // RxPDO (master-to-slave) is checked AFTER exchangeViaPDO -- it
        // contains what the master just built for this cycle.
        const bool rx_changed = (debug_raw_ || debug_frame_) &&
            std::memcmp(rx_buffer, last_rx_.data(), sizeof(FSoERxPDO)) != 0;
        if (rx_changed) {
            std::memcpy(last_rx_.data(), rx_buffer, sizeof(FSoERxPDO));
        }

        if (debug_frame_ && rx_changed) {
            const auto* rx_pdo = reinterpret_cast<const FSoERxPDO*>(rx_buffer);
            fsoe_dbg::dumpRxPDOFrame(TAG, *rx_pdo);
        }

        if (debug_raw_ && rx_changed) {
            const auto* rx_pdo = reinterpret_cast<const FSoERxPDO*>(rx_buffer);
            fsoe_dbg::dumpRxPDO(TAG, *rx_pdo);
        }

        // Always return true  -  exchangeViaPDO() returns false for duplicate
        // frames in Data state, which is normal (the slave re-sends the same
        // response while the master's TX is cached).  Returning false would
        // halt the CyclicTaskScheduler::executeAll() chain and prevent
        // subsequent cyclic tasks (e.g. FSoEDiagnosticsTask) from running.
        // FSoE errors are handled internally via the state machine's
        // error/fail-safe callbacks, not via the return value here.
        (void)ok;
        return true;
    }

private:
    uint16_t slave_index_;
    FSoEMain& main_instance_;
    size_t rx_pdo_offset_;
    size_t tx_pdo_offset_;
    FSoEFrameVariant fsoe_variant_;
    size_t fsoe_tx_size_;
    size_t sm3_total_len_;
    bool debug_raw_ = false;
    bool debug_frame_ = false;
    bool debug_wire_ = false;
    uint64_t elapsed_time_ms_ = 0;
    uint32_t cycle_count_ = 0;
    // Last-seen FSoE PDO content for change detection (max frame size —
    // only the first fsoe_tx_size_ bytes are compared/copied)
    std::array<uint8_t, sizeof(FSoETxPDO)> last_tx_{};
    std::array<uint8_t, sizeof(FSoERxPDO)> last_rx_{};

    void dumpWire(const uint8_t* tx_buffer, const uint8_t* rx_buffer) {
        fsoe_dbg::dumpWire("fsoe-wire", tx_buffer, rx_buffer,
                           kSM2TotalLen, sm3_total_len_, cycle_count_);
        cycle_count_++;
    }
};

// ============================================================================
// Default PDO / SM / FMMU diagnostic dump
// ============================================================================
//
// Reads the drive's default PDO assignment (0x1C12/0x1C13), SyncManager
// registers (0x0810–0x081F), and FMMU registers (0x0600–0x063F) via SDO
// and register access.  This shows the drive's power-on defaults before
// the master overwrites them  -  useful for understanding what the drive
// expects for FSoE + motion PDO configuration.
//
// Call this right after PRE_OP transition, before any PDO configuration.

static void dumpDefaultPdoConfig(EtherCAT::Master& ec_master,
                                  uint16_t slave_idx,
                                  EtherCAT::CoE::CoEManager& sdo) {
    constexpr uint32_t kSdoTimeoutMs = 1000;

    if (!ec_master.debugFlags().isEnabled("pdo-configuration", slave_idx)) {
        return;
    }

    const auto slave_addr = EtherCAT::Master::slaveAddressFromADP(
        EtherCAT::Master::adpForSlaveIndex(slave_idx));

    TETHER_LOGI(TAG, "===== Default PDO/SM/FMMU dump (slave {}) =====", slave_idx);

    // --- 0x1C12 (SM2 RxPDO assignment) ---
    {
        auto cnt = sdo.readU8(0x1C12, 0, {.timeout_ms = kSdoTimeoutMs});
        if (cnt.has_value()) {
            TETHER_LOGI(TAG, "  0x1C12 (SM2 RxPDO assign): {} entries", cnt.value());
            for (uint8_t i = 1; i <= cnt.value() && i <= 16; i++) {
                auto entry = sdo.readU16(0x1C12, i, {.timeout_ms = kSdoTimeoutMs});
                if (entry.has_value())
                    TETHER_LOGI(TAG, "    0x1C12:{} = 0x{:04X}", i, entry.value());
                else
                    TETHER_LOGW(TAG, "    0x1C12:{} FAILED ({})", i, coeErrorDetail(entry.error()));
            }
        } else {
            TETHER_LOGW(TAG, "  0x1C12:0 FAILED ({}) — no default RxPDO assignment", coeErrorDetail(cnt.error()));
        }
    }

    // --- 0x1C13 (SM3 TxPDO assignment) ---
    {
        auto cnt = sdo.readU8(0x1C13, 0, {.timeout_ms = kSdoTimeoutMs});
        if (cnt.has_value()) {
            TETHER_LOGI(TAG, "  0x1C13 (SM3 TxPDO assign): {} entries", cnt.value());
            for (uint8_t i = 1; i <= cnt.value() && i <= 16; i++) {
                auto entry = sdo.readU16(0x1C13, i, {.timeout_ms = kSdoTimeoutMs});
                if (entry.has_value())
                    TETHER_LOGI(TAG, "    0x1C13:{} = 0x{:04X}", i, entry.value());
                else
                    TETHER_LOGW(TAG, "    0x1C13:{} FAILED ({})", i, coeErrorDetail(entry.error()));
            }
        } else {
            TETHER_LOGW(TAG, "  0x1C13:0 FAILED ({}) — no default TxPDO assignment", coeErrorDetail(cnt.error()));
        }
    }

    // --- SyncManager registers ---
    // SM0: 0x0800-0x0807, SM1: 0x0808-0x080F, SM2: 0x0810-0x0817, SM3: 0x0818-0x081F
    const char* sm_names[] = {"SM0 (MBoxOut)", "SM1 (MBoxIn)", "SM2 (Outputs)", "SM3 (Inputs)"};
    for (int sm = 0; sm < 4; sm++) {
        uint16_t base = 0x0800 + static_cast<uint16_t>(sm) * 8;
        uint16_t start_addr = 0;
        uint16_t length = 0;
        uint8_t control = 0;
        uint8_t status = 0;
        uint8_t activate = 0;
        uint8_t pdi_control = 0;

        ec_master.readRegister(slave_addr, base,     &start_addr, 2, 200);
        ec_master.readRegister(slave_addr, base + 2, &length, 2, 200);
        ec_master.readRegister(slave_addr, base + 4, &control, 1, 200);
        ec_master.readRegister(slave_addr, base + 5, &status, 1, 200);
        ec_master.readRegister(slave_addr, base + 6, &activate, 1, 200);
        ec_master.readRegister(slave_addr, base + 7, &pdi_control, 1, 200);

        TETHER_LOGI(TAG, "  {}: start=0x{:04X} len={} ctrl=0x{:02X} status=0x{:02X} act=0x{:02X} pdi=0x{:02X}",
                    sm_names[sm], start_addr, length, control, status, activate, pdi_control);
    }

    // --- FMMU registers ---
    // FMMU0: 0x0600-0x060F, FMMU1: 0x0610-0x061F, FMMU2: 0x0620-0x062F, FMMU3: 0x0630-0x063F
    for (int fmmu = 0; fmmu < 4; fmmu++) {
        uint16_t base = 0x0600 + static_cast<uint16_t>(fmmu) * 16;
        uint32_t log_addr = 0;
        uint16_t length = 0;
        uint16_t phys_addr = 0;
        uint8_t fmmu_type = 0;  // bit0=read, bit1=write, bit3=enable

        ec_master.readRegister(slave_addr, base,     &log_addr, 4, 200);
        ec_master.readRegister(slave_addr, base + 4, &length, 2, 200);
        ec_master.readRegister(slave_addr, base + 6, &phys_addr, 2, 200);
        ec_master.readRegister(slave_addr, base + 8, &fmmu_type, 1, 200);

        bool enabled = fmmu_type & 0x08;
        bool read_access = fmmu_type & 0x01;
        bool write_access = fmmu_type & 0x02;

        if (enabled || log_addr != 0 || length != 0) {
            const char* dir = (read_access && write_access) ? "RW" :
                              read_access ? "R" : write_access ? "W" : "--";
            TETHER_LOGI(TAG, "  FMMU{}: log=0x{:08X} phys=0x{:04X} len={} type=0x{:02X} [{} {}]",
                        fmmu, log_addr, phys_addr, length, fmmu_type,
                        enabled ? "EN" : "DIS", dir);
        } else {
            TETHER_LOGI(TAG, "  FMMU{}: (unused)", fmmu);
        }
    }

    // --- Read individual PDO mapping objects for the assigned PDOs ---
    // For each PDO index in 0x1C12/0x1C13, read 0x1A00:0 etc. to get the
    // mapping entry count and total size.
    auto dump_pdo_mapping = [&](uint16_t pdo_idx, const char* label) {
        auto cnt = sdo.readU8(pdo_idx, 0, {.timeout_ms = kSdoTimeoutMs});
        if (!cnt.has_value()) {
            TETHER_LOGW(TAG, "  {} (0x{:04X}): FAILED to read count ({})", label, pdo_idx, coeErrorDetail(cnt.error()));
            return;
        }
        uint16_t total_bits = 0;
        TETHER_LOGI(TAG, "  {} (0x{:04X}): {} entries", label, pdo_idx, cnt.value());
        for (uint8_t i = 1; i <= cnt.value() && i <= 32; i++) {
            auto entry = sdo.readU32(pdo_idx, i, {.timeout_ms = kSdoTimeoutMs});
            if (entry.has_value()) {
                uint32_t v = entry.value();
                uint16_t idx = (v >> 16) & 0xFFFF;
                uint8_t sub = (v >> 8) & 0xFF;
                uint8_t bits = v & 0xFF;
                total_bits += bits;
                TETHER_LOGI(TAG, "    {}:{} = 0x{:08X} (idx=0x{:04X} sub={} bits={})",
                            label, i, v, idx, sub, bits);
            } else {
                TETHER_LOGW(TAG, "    {}:{} FAILED ({})", label, i, coeErrorDetail(entry.error()));
            }
        }
        TETHER_LOGI(TAG, "    → total: {} bits = {} bytes", total_bits, total_bits / 8);
    };

    // Read mappings for all relevant PDOs
    dump_pdo_mapping(0x1600, "RxPDO_1600");
    dump_pdo_mapping(0x1601, "RxPDO_1601");
    dump_pdo_mapping(0x1602, "RxPDO_1602");
    dump_pdo_mapping(0x1700, "FSoE_RxPDO_1700");
    dump_pdo_mapping(0x1A00, "TxPDO_1A00");
    dump_pdo_mapping(0x1A01, "TxPDO_1A01");
    dump_pdo_mapping(0x1A02, "TxPDO_1A02");
    dump_pdo_mapping(0x1A03, "TxPDO_1A03");
    dump_pdo_mapping(0x1B00, "FSoE_TxPDO_1B00");

    TETHER_LOGI(TAG, "===== End default PDO/SM/FMMU dump =====");
}

// ============================================================================
// Main
// ============================================================================

struct Args {
    std::string interface;
    int slave_index = 0;
    double duration = 10.0;
    bool enable_dc_sync = false;
    uint16_t connection_id = 0x0006;  ///< Must match drive's Device Safety Address (0xF980:1)
    uint16_t safety_address = 0x0006; ///< FSoE slave safety address (0x2620:3)
    uint16_t watchdog_ms = EtherCAT::Drives::Synapticon::SafeMotion::Timing::kMinimumWatchdogTimeMs;
    uint32_t diag_interval_ms = 1000;
    std::string debug;
    Tether::Examples::VlanConfig vlan;
    ///< STO override: -1 = not set (use motionEnabled default), 0 = force STO off, 1 = force STO on
    int sto_override = -1;
    ///< SOS override: -1 = not set (defaults to STO value), 0 = force SOS off, 1 = force SOS on
    int sos_override = -1;
    ///< SBC override: -1 = not set (use motionEnabled default), 0 = force brake released, 1 = force brake engaged
    int sbc_override = -1;
    ///< After N seconds, run full safety diagnostics via CoE then exit. 0 = disabled.
    double diagnostics_after = 0.0;
    ///< FSoE status-frame variant: "lw1" (31 B frame) or "lw2" (35 B frame).
    std::string fsoe_frame = "lw2";
    double torque_pp_nm = 0.0;       ///< Peak-to-peak CST sine torque in Nm (0 = no motion)
    double freq_hz = 0.5;            ///< Sine torque frequency in Hz
    uint32_t rated_torque_mnm = 0;   ///< Motor rated torque in mNm (0 = auto-detect from 0x6076)
};

bool parseArgs(int argc, char** argv, Args& out) {
    argparse::ArgumentParser program("synapticon_cst_fsoe", "1.0", argparse::default_arguments::help);
    Tether::Examples::addInterfaceArg(program);
    program.add_argument("-s", "--slave")
        .scan<'i', int>()
        .default_value(0)
        .help("Slave index on the bus (0-based)");
    program.add_argument("-d", "--duration")
        .scan<'g', double>()
        .default_value(10.0)
        .help("Duration in seconds");
    program.add_argument("--dc-sync")
        .default_value(false)
        .implicit_value(true)
        .help("Enable EtherCAT distributed-clock synchronization (off by default)");
    program.add_argument("--connection-id")
        .scan<'x', unsigned int>()
        .default_value(static_cast<unsigned int>(0x0006))
        .help("FSoE connection ID (hex, default 0x0006  —  must match drive's "
              "Device Safety Address 0xF980:1)");
    program.add_argument("--safety-address")
        .scan<'x', unsigned int>()
        .default_value(static_cast<unsigned int>(0x0006))
        .help("FSoE slave safety address (hex, default 0x0006  —  from drive's "
              "0x2620:3)");
    program.add_argument("--watchdog-ms")
        .scan<'i', int>()
        .default_value(static_cast<int>(
            EtherCAT::Drives::Synapticon::SafeMotion::Timing::kMinimumWatchdogTimeMs))
        .help("FSoE watchdog timeout in ms");
    program.add_argument("--diag-interval-ms")
        .scan<'i', int>()
        .default_value(1000)
        .help("Diagnostics print interval in ms");
    program.add_argument("--debug")
        .default_value(std::string(""))
        .help("Comma-separated debug flags. Use '--debug help' for a list.");
    program.add_argument("--sto")
        .scan<'i', int>()
        .default_value(-1)
        .help("Set raw STO bit value in the FSoE PDO: 0 = bit cleared, 1 = bit set. "
              "Default (-1) uses motionEnabled() default. "
              "NOTE: the codec uses 0-active encoding, so bit=0 means STO active "
              "(torque off) and bit=1 means STO inactive (torque allowed).");
    program.add_argument("--sos")
        .scan<'i', int>()
        .default_value(-1)
        .help("Set raw SOS bit value in the FSoE PDO: 0 = bit cleared, 1 = bit set. "
              "Default (-1) follows the STO override value (or motionEnabled() default). "
              "NOTE: the codec uses 0-active encoding, so bit=0 means SOS active "
              "(safe operating stop) and bit=1 means SOS inactive.");
    program.add_argument("--sbc")
        .scan<'i', int>()
        .default_value(-1)
        .help("Set raw SBC bit value in the FSoE PDO: 0 = bit cleared, 1 = bit set. "
              "Default (-1) uses motionEnabled() default. "
              "NOTE: the codec uses 0-active encoding, so bit=0 means SBC active "
              "(brake engaged) and bit=1 means SBC inactive (brake released).");
    program.add_argument("--diagnostics-after")
        .scan<'g', double>()
        .default_value(0.0)
        .help("After N seconds of FSoE operation, run full safety diagnostics "
              "via CoE (SDO reads of safety objects), then exit. "
              "FSoE output is suppressed during diagnostics to keep the "
              "output clean. 0 = disabled (default).");
    program.add_argument("--fsoe-frame")
        .default_value(std::string("lw2"))
        .help("FSoE status-frame variant the slave is configured with: "
              "'lw1' = 31-byte frame, 'lw2' = 35-byte frame (safe torque "
              "data + CRC appended, default). Must match the drive's "
              "safety parameter set.");
    program.add_argument("--torque-nm")
        .scan<'g', double>()
        .default_value(0.0)
        .help("Peak-to-peak CST sine torque amplitude in Nm. "
              "Default 0 = FSoE-only, no motion. When > 0 the drive is "
              "enabled in CST mode once FSoE reports motion allowed and "
              "the brake is released.");
    program.add_argument("--freq-hz")
        .scan<'g', double>()
        .default_value(0.5)
        .help("Sine torque frequency in Hz (default 0.5)");
    program.add_argument("--rated-torque-mnm")
        .scan<'i', int>()
        .default_value(static_cast<int>(0))
        .help("Motor rated torque in mNm (0 = auto-detect from object 0x6076)");

    try {
        program.parse_args(argc, argv);
    } catch (const std::runtime_error& err) {
        std::cerr << err.what() << "\n" << program;
        return false;
    }

    out.interface = Tether::Examples::resolveInterface(program.get<std::string>("--interface"), TAG);
    out.slave_index = program.get<int>("--slave");
    out.duration = program.get<double>("--duration");
    out.enable_dc_sync = program.get<bool>("--dc-sync");
    out.connection_id = static_cast<uint16_t>(program.get<unsigned int>("--connection-id"));
    out.safety_address = static_cast<uint16_t>(program.get<unsigned int>("--safety-address"));
    out.watchdog_ms = static_cast<uint16_t>(program.get<int>("--watchdog-ms"));
    out.diag_interval_ms = static_cast<uint32_t>(program.get<int>("--diag-interval-ms"));
    out.debug = program.get<std::string>("--debug");
    out.sto_override = program.get<int>("--sto");
    out.sos_override = program.get<int>("--sos");
    out.sbc_override = program.get<int>("--sbc");
    out.diagnostics_after = program.get<double>("--diagnostics-after");
    out.fsoe_frame = program.get<std::string>("--fsoe-frame");
    if (out.fsoe_frame != "lw1" && out.fsoe_frame != "lw2") {
        std::cerr << "Invalid --fsoe-frame '" << out.fsoe_frame
                  << "' (expected 'lw1' or 'lw2')\n";
        return false;
    }
    out.torque_pp_nm = program.get<double>("--torque-nm");
    out.freq_hz = program.get<double>("--freq-hz");
    out.rated_torque_mnm = static_cast<uint32_t>(program.get<int>("--rated-torque-mnm"));
    if (out.torque_pp_nm < 0.0) {
        std::cerr << "--torque-nm must be >= 0\n";
        return false;
    }
    if (!Tether::Examples::parseVlanArgs(
            program.get<std::string>("--rx-vlan"),
            program.get<std::string>("--tx-vlan"),
            out.vlan, TAG)) {
        return false;
    }
    Tether::Examples::logVlanConfig(out.vlan, TAG);
    return true;
}

} // namespace

int main(int argc, char** argv) {
    Args args;
    if (!parseArgs(argc, argv, args)) return 1;

    if (args.debug == "help") {
        std::cout << "Available --debug flags (comma-separated):\n";
        for (const auto& info : EtherCAT::debug::allDebugFlags()) {
            std::cout << "  " << info.name << "\n      " << info.description << "\n";
        }
        std::cout << "\nFilter syntax:\n"
                  << "  --debug flagname:(slaves:0,2,5),otherflag:(slaves:1-3)\n"
                  << "  (default: pass-all for every flag)\n";
        return 0;
    }

    if (args.slave_index < 0 || args.slave_index > 65535) {
        std::cerr << "Invalid slave index\n";
        return 1;
    }
    const uint16_t slave_idx = static_cast<uint16_t>(args.slave_index);

    // FSoE frame variant (--fsoe-frame): selects the slave→master frame
    // layout — LW2 (35 B, safe torque data appended) or LW1 (31 B).
    // Everything derived from the TxPDO 0x1B00 size (motion PDO offset,
    // SM3 length, connection input size) follows the variant.
    const FSoEFrameVariant fsoe_variant =
        args.fsoe_frame == "lw2" ? FSoEFrameVariant::LW2 : FSoEFrameVariant::LW1;
    const size_t fsoe_tx_size =
        EtherCAT::Drives::SynapticonPDO::fsoeTxPDOSize(fsoe_variant);
    const size_t motion_tx_pdo_offset =
        EtherCAT::Drives::SynapticonPDO::motionTxPDOOffset(fsoe_variant);
    const size_t sm3_total_len =
        EtherCAT::Drives::SynapticonPDO::sm3CombinedSize(fsoe_variant);

    Tether::Platform::ensureRealtimeKernelOrExit();

    TETHER_LOGI(TAG,
        "synapticon_cst_fsoe  —  interface={} slave={} duration={:.1f} dc_sync={} debug='{}' "
        "conn_id=0x{:04X} safety_addr=0x{:04X} "
        "sto_override={} sos_override={} sbc_override={} diagnostics_after={:.1f} "
        "fsoe_frame={} ({} B)",
        args.interface.c_str(), slave_idx, args.duration,
        args.enable_dc_sync ? "on" : "off",
        args.debug.c_str(),
        args.connection_id, args.safety_address,
        args.sto_override < 0 ? "default" : std::to_string(args.sto_override).c_str(),
        args.sos_override < 0 ? "default" : std::to_string(args.sos_override).c_str(),
        args.sbc_override < 0 ? "default" : std::to_string(args.sbc_override).c_str(),
        args.diagnostics_after,
        args.fsoe_frame.c_str(), fsoe_tx_size);

    // --- Start EtherCAT master ---
    EtherCAT::DS402Master master;
    Tether::Examples::HostMasterSession session;
    if (!Tether::Examples::startHostMasterSession(args.interface, master, session, TAG, args.vlan)) {
        return 2;
    }

    // --- Configure mailbox with SOMANET ESI values ---
    // The SOMANET_CiA_402_v5.1.9.xml ESI defines the mailbox sync managers
    // with 1024-byte buffers at 0x1000 (SM0, M→S write) and 0x1400 (SM1,
    // S→M read), supporting CoE + FoE.  In practice the drive firmware only
    // accepts 512-byte buffers, so kMailboxWriteSize/kMailboxReadSize are
    // overridden to 512 above.  These values are used instead of relying on
    // SII EEPROM auto-configuration so the correct mailbox geometry is
    // always used for SOMANET drives.
    {
        if (master.ethercatMaster().discovery().discover(EtherCAT::DiscoveryOptions()).empty()) {
            TETHER_LOGW(TAG, "No slaves discovered during pre-config scan");
        }
        if (!master.waitForDriveCount(
                static_cast<uint16_t>(slave_idx + 1), 2000)) {
            TETHER_LOGE(TAG, "Timed out waiting for slave {}", slave_idx);
            Tether::Examples::stopHostMasterSession(master, session);
            return 2;
        }

        // --- Apply EtherCAT framework debug flags ---
        // The --debug string may contain both FSoE-specific flags (fsoe,
        // fsoe-frame, etc.) and EtherCAT framework flags (rx-pdo, coe-reads,
        // al-state, etc.).  The FSoE flags are parsed separately below via
        // string search; the framework flags are applied here via the shared
        // ExampleHelpers parser, which supports per-slave filter syntax.
        {
            const auto debug_flags =
                Tether::Examples::parseDebugFlags(args.debug);
            Tether::Examples::applyDebugFlags(
                debug_flags, master.ethercatMaster(), TAG);
        }

        // --- Slave identity verification is intentionally skipped ---
        // The Synapticon drive does not support APWR to the EEPCTL register,
        // so SII/EEPROM reads are impossible via the standard register
        // interface.  This example targets SOMANET drives only and relies on
        // the hardcoded ESI values from tether/drives/Synapticon.hpp for all
        // mailbox/PDO configuration  -  no SII read or vendor/product
        // verification is performed.

        // --- Reset slave to INIT if currently in a higher state ---
        // If the slave is already in PRE_OP, SAFE_OP, or OP (e.g. from a
        // previous run that didn't shut down cleanly), bring it back to INIT
        // before reconfiguring the mailbox and PDO mapping.  This ensures a
        // clean starting point regardless of the slave's current state.
        {
            uint8_t current_state = 0;
            if (master.ethercatMaster().readSlaveApplicationLayerState(
                    slave_idx, current_state)) {
                TETHER_LOGI(TAG,
                    "Slave {} current AL state: 0x{:02X} ({})",
                    slave_idx, current_state,
                    EtherCAT::getECStateName(static_cast<EtherCAT::ECState>(current_state)));

                if (current_state != static_cast<uint8_t>(EtherCAT::ECState::Init)) {
                    TETHER_LOGI(TAG,
                        "Slave {} is not in INIT (0x{:02X})  —  resetting to INIT "
                        "before configuration",
                        slave_idx, current_state);

                    EtherCAT::ALResetController reset_ctrl(master.ethercatMaster());
                    reset_ctrl.setProgressCallback(
                        [](uint16_t si, int iter, int max_iter,
                           uint16_t al, uint16_t code, bool reached) {
                            if (!reached) {
                                const char* state_name =
                                    EtherCAT::al_status_get_state_name(al);
                                const bool has_err = EtherCAT::al_status_has_error(al);
                                TETHER_LOGI(TAG,
                                    "  Slave {} reset iter {}/{}: "
                                    "AL_STATUS=0x{:04X} (state={}, error={})",
                                    si, iter, max_iter, al,
                                    state_name, has_err ? "true" : "false");
                                if (code != 0) {
                                    TETHER_LOGI(TAG,
                                        "    AL_STATUS_CODE=0x{:04X} ({})",
                                        code, EtherCAT::getALStatusCodeName(code));
                                }
                            }
                        });

                    const auto result = reset_ctrl.resetSlave(
                        slave_idx, static_cast<uint8_t>(EtherCAT::ECState::Init));

                    if (result.success) {
                        TETHER_LOGI(TAG,
                            "Slave {} reset to INIT OK ({}, {} iterations)",
                            slave_idx, result.message.c_str(),
                            result.iterations_used);
                    } else {
                        TETHER_LOGE(TAG,
                            "Slave {} reset to INIT FAILED ({}, {} iterations, "
                            "final AL_STATUS=0x{:04X}, AL_STATUS_CODE=0x{:04X})",
                            slave_idx, result.message.c_str(),
                            result.iterations_used,
                            result.final_al_status,
                            result.final_al_status_code);
                        Tether::Examples::stopHostMasterSession(master, session);
                        return 2;
                    }

                    // Give the slave a moment to settle after the reset.
                    Tether::Platform::Clock::instance().delayMilliseconds(100);
                }
            } else {
                TETHER_LOGW(TAG,
                    "Could not read AL state for slave {}  —  continuing anyway",
                    slave_idx);
            }
        }

        auto& slave = master.ethercatMaster().slave(slave_idx);
        // configureMailbox(mbox_out, mbox_in, protocols):
        //   mbox_in  → SM0 (mailbox_write, master→slave)
        //   mbox_out → SM1 (mailbox_read,  slave→master)
        const auto mb_err = slave.configureMailbox(
            {.address = kMailboxReadAddr,  .length = kMailboxReadSize},  // mbox_out → SM1
            {.address = kMailboxWriteAddr, .length = kMailboxWriteSize}, // mbox_in  → SM0
            kMailboxProtocols);
        if (mb_err != EtherCAT::SlaveError::Ok) {
            TETHER_LOGE(TAG, "Failed to configure mailbox: {}",
                        EtherCAT::slaveErrorToString(mb_err));
            Tether::Examples::stopHostMasterSession(master, session);
            return 2;
        }
        TETHER_LOGI(TAG,
            "Mailbox configured from SOMANET ESI: "
            "SM0(M->S)=0x{:04X}/{} SM1(S->M)=0x{:04X}/{} proto=0x{:04X}",
            kMailboxWriteAddr, kMailboxWriteSize,
            kMailboxReadAddr, kMailboxReadSize, kMailboxProtocols);

        // Transition to PRE_OP before any SDO exchange.  Mailbox communication
        // (CoE/SDO) is only valid in PRE_OP or higher  -  the slave's PDI does
        // not service the mailbox in INIT, leaving SM0 full and SM1 empty.
        const auto pre_err = slave.transitionToPreOp();
        if (pre_err != EtherCAT::SlaveError::Ok) {
            TETHER_LOGE(TAG, "Failed to transition to PRE_OP: {}",
                        EtherCAT::slaveErrorToString(pre_err));
            Tether::Examples::stopHostMasterSession(master, session);
            return 2;
        }
        TETHER_LOGI(TAG, "Slave {} transitioned to PRE_OP", slave_idx);
    }

    // --- Dump default PDO/SM/FMMU configuration ---
    // Read the drive's power-on defaults before we overwrite anything.
    // This shows what the drive expects for FSoE + motion PDO layout.
    {
        auto& sdo = master.ethercatMaster().sdoManager(slave_idx);
        dumpDefaultPdoConfig(master.ethercatMaster(), slave_idx, sdo);
    }

    // --- Read ETG.5000/5001 modular device profile objects (0xFxxx) ---
    // These objects describe the modular framework state and FSoE configuration.
    // Must be read in PRE_OP (mailbox SDOs don't work in OP).
    {
        auto& sdo = master.ethercatMaster().sdoManager(slave_idx);
        TETHER_LOGI(TAG, "===== ETG.5000/5001 objects (0xFxxx) =====");

        auto read_u8 = [&](uint16_t idx, uint8_t sub, const char* name) -> void {
            auto res = sdo.readU8(idx, sub, {.timeout_ms = kSdoTimeoutMs});
            if (res.has_value())
                TETHER_LOGI(TAG, "  0x{:04X}:{} ({}) = 0x{:02X}", idx, sub, name, res.value());
            else
                TETHER_LOGW(TAG, "  0x{:04X}:{} ({}) error: {}", idx, sub, name, coeErrorDetail(res.error()));
        };
        auto read_u16 = [&](uint16_t idx, uint8_t sub, const char* name) -> void {
            auto res = sdo.readU16(idx, sub, {.timeout_ms = kSdoTimeoutMs});
            if (res.has_value())
                TETHER_LOGI(TAG, "  0x{:04X}:{} ({}) = 0x{:04X}", idx, sub, name, res.value());
            else
                TETHER_LOGW(TAG, "  0x{:04X}:{} ({}) error: {}", idx, sub, name, coeErrorDetail(res.error()));
        };
        auto read_u32 = [&](uint16_t idx, uint8_t sub, const char* name) -> void {
            auto res = sdo.readU32(idx, sub, {.timeout_ms = kSdoTimeoutMs});
            if (res.has_value())
                TETHER_LOGI(TAG, "  0x{:04X}:{} ({}) = 0x{:08X}", idx, sub, name, res.value());
            else
                TETHER_LOGW(TAG, "  0x{:04X}:{} ({}) error: {}", idx, sub, name, coeErrorDetail(res.error()));
        };

        // ETG.5000 Modular Device Profile objects
        TETHER_LOGI(TAG, "  -- Module Ident Lists --");
        read_u8 (0xF002, 0, "Module Ident List count");
        read_u32(0xF002, 1, "Module Ident[1]");
        read_u32(0xF002, 2, "Module Ident[2]");

        TETHER_LOGI(TAG, "  -- Configured Module Ident (0xF030) --");
        read_u8 (0xF030, 0, "0xF030 count");
        read_u32(0xF030, 1, "0xF030:1 (Module ident pos 1)");
        read_u32(0xF030, 2, "0xF030:2 (0x22D20001=no-param, 0x22D20002=with-param)");

        TETHER_LOGI(TAG, "  -- Detected Module Ident (0xF050) --");
        read_u8 (0xF050, 0, "0xF050 count");
        read_u32(0xF050, 1, "0xF050:1 (Detected Module ident pos 1)");
        read_u32(0xF050, 2, "0xF050:2 (Detected Module ident pos 2)");

        // ETG.5001 FSoE objects
        TETHER_LOGI(TAG, "  -- Device Safety Address (0xF980) --");
        read_u8 (0xF980, 0, "0xF980 count");
        read_u16(0xF980, 1, "Device Safety Address");

        // Additional 0xFxxx objects that may exist
        TETHER_LOGI(TAG, "  -- Other 0xFxxx objects --");
        read_u8 (0xF010, 0, "0xF010 count (Modular Device Profile)");
        read_u16(0xF010, 1, "0xF010:1 (Profile type)");
        read_u16(0xF010, 2, "0xF010:2 (Profile instance)");

        TETHER_LOGI(TAG, "===== End ETG.5000/5001 objects =====");
    }

    // --- Sync configured module ident list (0xF030) to detected (0xF050) ---
    // The slave validates the module list during the PRE-OP -> SAFE-OP
    // transition and rejects SAFE-OP with AL status 0x0070 when they differ
    // (e.g. firmware 5.6.6 reports ident 0x22D20003 "FSoE Module with safe
    // Torque" while the stored configuration expects 0x22D20001).  The
    // SOMANET ESI marks the ident list as downloadable
    // (DownloadModuleIdentList="1"), so writing the detected list into
    // 0xF030 is the intended fix.
    {
        auto& slave = master.ethercatMaster().slave(slave_idx);
        constexpr uint16_t kCfgIdentIdx = 0xF030;
        constexpr uint16_t kDetIdentIdx = 0xF050;

        auto readIdentList = [&](uint16_t index, std::vector<uint32_t>& out) -> bool {
            out.clear();
            uint8_t count = 0;
            if (slave.sdoReadU8(index, 0x00, count) != EtherCAT::SlaveError::Ok)
                return false;
            for (uint8_t i = 1; i <= count; ++i) {
                uint32_t ident = 0;
                if (slave.sdoReadU32(index, i, ident) != EtherCAT::SlaveError::Ok)
                    return false;
                out.push_back(ident);
            }
            return true;
        };

        std::vector<uint32_t> configured, detected;
        if (readIdentList(kCfgIdentIdx, configured) &&
            readIdentList(kDetIdentIdx, detected)) {
            if (configured != detected) {
                TETHER_LOGW(TAG,
                    "Module ident MISMATCH (slave {}): "
                    "configured 0xF030 = [{}] vs detected 0xF050 = [{}].  "
                    "Updating configured list to match detected modules "
                    "(otherwise SAFE-OP is rejected with AL 0x0070)...",
                    slave_idx,
                    [&]{ std::string r; for (auto v : configured) r += std::format("0x{:08X} ", v); return r; }(),
                    [&]{ std::string r; for (auto v : detected) r += std::format("0x{:08X} ", v); return r; }());

                // Standard CoE list write: clear sub0, write entries,
                // then set sub0 to the entry count.
                bool write_ok =
                    slave.sdoWriteU8(kCfgIdentIdx, 0x00, 0) == EtherCAT::SlaveError::Ok;
                for (size_t i = 0; write_ok && i < detected.size(); ++i) {
                    write_ok = slave.sdoWriteU32(kCfgIdentIdx,
                                                 static_cast<uint8_t>(i + 1),
                                                 detected[i]) == EtherCAT::SlaveError::Ok;
                }
                if (write_ok) {
                    write_ok = slave.sdoWriteU8(kCfgIdentIdx, 0x00,
                        static_cast<uint8_t>(detected.size())) == EtherCAT::SlaveError::Ok;
                }
                if (write_ok) {
                    TETHER_LOGI(TAG,
                        "Configured module ident list updated ({} entries)",
                        detected.size());
                } else {
                    TETHER_LOGW(TAG,
                        "Failed to update 0xF030 — "
                        "SAFE-OP will likely be rejected (AL 0x0070)");
                }
            }
        } else {
            TETHER_LOGW(TAG,
                "Could not read module ident lists (0xF030/0xF050) — "
                "skipping ident sync");
        }
    }

    // --- Read FSoE safety address (0xF980:1) for verification ---
    // Object 0xF980:1 contains the safety address configured on the drive.
    // For Synapticon SOMANET drives, the FSoE connection ID must match this
    // value (default 0x0006).  Reading it before starting the FSoE handshake
    // lets us warn the operator if --connection-id doesn't match the drive's
    // configured safety address, which would cause ConnectionIDError failures.
    {
        auto& slave = master.ethercatMaster().slave(slave_idx);
        uint16_t drive_safety_address = 0;
        const auto addr_err =
            EtherCAT::Drives::Synapticon::readFSoESafetyAddress(
                slave, drive_safety_address);
        if (addr_err == EtherCAT::SlaveError::Ok) {
            TETHER_LOGI(TAG,
                "FSoE safety address (0xF980:1): 0x{:04X}  —  "
                "--connection-id=0x{:04X} --safety-address=0x{:04X}",
                drive_safety_address,
                args.connection_id,
                args.safety_address);
            if (drive_safety_address != args.connection_id) {
                TETHER_LOGW(TAG,
                    "WARNING: --connection-id (0x{:04X}) does NOT match the "
                    "drive's Device Safety Address 0xF980:1 (0x{:04X}).  "
                    "This will likely cause FSoE ConnectionIDError.  "
                    "Use --connection-id 0x{:04X} to match the drive.",
                    args.connection_id, drive_safety_address,
                    drive_safety_address);
            }
        } else {
            TETHER_LOGW(TAG,
                "Failed to read FSoE safety address (0xF980:1) via SDO "
                "(err={})  —  falling back to --connection-id=0x{:04X}",
                static_cast<unsigned>(addr_err),
                args.connection_id);
        }
    }

    // --- Configure drive: combined FSoE + motion PDO mapping ---
    //
    // Both the CiA 402 motion PDOs (0x1600/0x1A00) and the FSoE safety PDOs
    // (0x1700/0x1B00) must be mapped simultaneously.  This requires the
    // multi-PDO-per-sync-manager API
    // (CiA402Drive::transitionToOp(const Slave::MultiPDOAssignment&)).
    //
    // PDO buffer layout (combined):
    //   SM2 (Rx): [0x1600 (12B)][0x1700 (11B)] = 23 bytes
    //   SM3 (Tx): [0x1A00 (12B)][0x1B00 (35B)] = 47 bytes
    int rc = 0;

    // Discover slaves and initialize distributed clocks
    if (master.ethercatMaster().discovery().discover(EtherCAT::DiscoveryOptions()).empty()) {
        TETHER_LOGW(TAG, "No slaves discovered");
    }
    const uint16_t minimum_drive_count = static_cast<uint16_t>(slave_idx + 1);
    if (!master.waitForDriveCount(minimum_drive_count, 2000)) {
        TETHER_LOGE(TAG, "Timed out waiting for {} drive(s)", minimum_drive_count);
        Tether::Examples::stopHostMasterSession(master, session);
        return 3;
    }

    {
        EtherCAT::DC::DCConfig dc_config = EtherCAT::DC::DCConfig::defaults();
        if (!master.initializeDistributedClocks(dc_config)) {
            TETHER_LOGE(TAG, "Failed to initialize distributed clocks");
            Tether::Examples::stopHostMasterSession(master, session);
            return 3;
        }
        if (!master.startDistributedClocks()) {
            TETHER_LOGE(TAG, "Failed to start distributed clocks");
            Tether::Examples::stopHostMasterSession(master, session);
            return 3;
        }
    }

    // Get-or-create the drive and configure it for OP transition.
    //
    // ensureDrive() is used instead of driveBySlaveIndex() because the
    // multi-PDO FSoE path below bypasses configureDrive() (which would
    // otherwise create the drive).  Without this, driveBySlaveIndex()
    // would return nullptr  -  no CiA402Drive object exists yet and the
    // slave role defaults to NonDS402.
    auto& drive = master.ensureDrive(slave_idx);
    drive.setSDOTimeout(kSdoTimeoutMs);

    {
        // Combined FSoE + motion PDO mapping via multi-PDO assignment.
        //
        // Uses the ESI layout with ALL PDOs, FSoE first:
        //   SM2: 0x1700 + 0x1600 + 0x1601 + 0x1602 = 46 bytes
        //   SM3: 0x1B00 + 0x1A00 + 0x1A01 + 0x1A02 + 0x1A03 = 78 bytes
        //
        // ALL PDOs (including FSoE) are written explicitly to 0x1C12/0x1C13.
        // FSoE PDOs come FIRST (critical for the Synapticon ESC bug  -  see
        // comment above).
        const auto assignment =
            EtherCAT::Drives::SynapticonPDO::makeCombinedPDOAssignment(fsoe_variant);

        TETHER_LOGI(TAG,
            "Transitioning to OP with combined FSoE+motion PDO mapping: "
            "SM2={} bytes (FSoE {}B + motion {}B), "
            "SM3={} bytes (FSoE {}B [{}] + motion {}B)",
            static_cast<uint16_t>(kSM2TotalLen),
            static_cast<uint16_t>(sizeof(FSoERxPDO)),
            EtherCAT::Drives::SynapticonPDO::kSM2TotalSize,
            static_cast<uint16_t>(sm3_total_len),
            static_cast<uint16_t>(fsoe_tx_size),
            args.fsoe_frame == "lw2" ? "LW2" : "LW1",
            EtherCAT::Drives::SynapticonPDO::kSM3TotalSize);

        // Note: Safety parameters (0x2620, 0x2641, etc.) are configured on the
        // drive via OBLAC Drives and cannot be written via SDO (error 0x08000021
        // "Local control error").  The FSoE module reads them from the drive's
        // safety parameter store.  The drive is already configured with:
        //   0x2620:2 (Safe fieldbus) = 0xFF (FSoE active)
        //   0x2620:3 (Safe address)  = 0x0006 (connection ID)
        //   0xF980:1 (Device Safety Address) = 0x0006

        if (!drive.transitionToOp(assignment)) {
            TETHER_LOGE(TAG, "Failed to transition to OP with combined PDO assignment");
            master.stopDistributedClocks();
            Tether::Examples::stopHostMasterSession(master, session);
            return 3;
        }
        TETHER_LOGI(TAG, "Slave {} transitioned to OP with combined FSoE+motion PDOs", slave_idx);

        // --- DC reconfigure (SYNC0 for the safety module's SM sampling) ---
        // Without SYNC0 the drive firmware samples the SM input region
        // asynchronously to the LRW arrival, so it can read a frame that is
        // half old / half new — the FSoE slave then never produces valid
        // output (all zeros / garbage in the TxPDO FSoE region).
        if (!master.ethercatMaster().dc().reconfigureSync(slave_idx)) {
            TETHER_LOGW(TAG,
                "DC reconfiguration failed on slave {} — continuing anyway",
                slave_idx);
        }
        Tether::Platform::Clock::instance().delayMilliseconds(50);

        // --- Comprehensive safety diagnostics ---
        // Query all safety-related objects via SDO before starting the PDO
        // exchange loop.  This catches configuration issues (FSoE not active,
        // parameter validation missing, connection ID mismatch) and reports
        // any active safety faults with human-readable descriptions.
        {
            auto& slave_ref = master.ethercatMaster().slave(slave_idx);
            auto diag_report = EtherCAT::Drives::Synapticon::runFullSafetyDiagnostics(
                slave_ref);

            // Log actionable warnings based on the diagnostic report
            if (diag_report.hasFault()) {
                TETHER_LOGE(TAG,
                    "SAFETY FAULT detected before starting PDO loop: '{}'  —  "
                    "FSoE communication may fail.  Check OBLAC Drives parameter "
                    "validation and safety configuration.",
                    diag_report.error_report);
            }
            if (diag_report.general_safety_ok && !diag_report.fsoeActive()) {
                TETHER_LOGE(TAG,
                    "FSoE is NOT active on the drive (0x2620:2 = 0).  "
                    "The safety module will not process FSoE PDO data.  "
                    "Enable FSoE in OBLAC Drives configuration.");
            }
            if (diag_report.module_ident_ok &&
                diag_report.configured_ident_pos2 == EtherCAT::Drives::Synapticon::kModuleIdentNoParam) {
                TETHER_LOGW(TAG,
                    "Module ident = 0x22D20001 (no parameter changes via master).  "
                    "Safety parameters must be validated in OBLAC Drives.  "
                    "If not validated, the safety module will report 'SmmFIO25' "
                    "(black channel fault) and refuse to communicate via FSoE.");
            }
            if (diag_report.connIdMismatch()) {
                TETHER_LOGE(TAG,
                    "Connection ID mismatch: 0xF980 (0x{:04X}) != 0x2620:3 (0x{:04X}).  "
                    "The master --connection-id must match the device safety address.",
                    diag_report.device_safety_address,
                    diag_report.safe_address);
            }
        }

        // --- PDO write / SDO readback diagnostic ---
        // Write a known pattern to the FSoE RxPDO region (0x1700), let the
        // cyclic task send it for a few cycles, then read back the individual
        // mapped objects via SDO to verify the data lands at the correct
        // offsets in the slave's object dictionary.
        TETHER_LOGI(TAG, "=== PDO write / SDO readback diagnostic ===");

        // Enable PDO exchange so the data actually gets sent
        master.ethercatMaster().pdo().resetStats();
        master.ethercatMaster().dc().setPDOEnabled(true);

        // Read the slave's configured station address (register 0x0010)
        // and set it in the PDOManager so exchangePhysical() uses the
        // correct FPWR/FPRD address.  Without this, exchangePhysical()
        // falls back to adp=0x0000 (physical position) which fails with
        // WKC=0 because the slave's configured address is different.
        {
            uint16_t cfg_addr = 0;
            if (master.ethercatMaster().readRegister(
                    EtherCAT::Master::slaveAddressFromADP(
                        EtherCAT::Master::adpForSlaveIndex(slave_idx)),
                    0x0010, cfg_addr, 200)) {
                TETHER_LOGI(TAG, "Slave {} configured station address (reg 0x0010) = 0x{:04X}",
                    slave_idx, cfg_addr);
                // Set the configured address in the PDO mapping so
                // exchangePhysical() uses FPWR/FPRD with the correct adp.
                master.ethercatMaster().pdo().mapping().set_slave_configured_address(
                    slave_idx, cfg_addr);
                // Also set it in the SlaveConfig so exchangePhysical()'s
                // cfg->configured_address check uses the right value.
                auto* sc = master.ethercatMaster().pdo().slaveConfigs();
                if (sc && slave_idx < EtherCAT::PDO::kMaxPDOSlaves) {
                    sc[slave_idx].configured_address = cfg_addr;
                }
            } else {
                TETHER_LOGW(TAG, "Failed to read configured station address (reg 0x0010)");
            }
        }

        // First, verify that SDO readback of PDO-mapped objects works at all.
        // Write a known value to the Controlword (0x6040) via PDO, then read
        // it back via SDO.  If this doesn't work, SDO readback of PDO-mapped
        // objects isn't supported on this slave.
        {
            uint8_t* rx_buf = static_cast<uint8_t*>(drive.getRxPDOBuffer());
            // Controlword is at offset kMotionRxPDOOffset in the combined PDO
            // (FSoE PDO comes first, motion PDO second)
            rx_buf[kMotionRxPDOOffset + 0] = 0x0F;  // Controlword low byte
            rx_buf[kMotionRxPDOOffset + 1] = 0x00;  // Controlword high byte
            TETHER_LOGI(TAG, "Wrote Controlword=0x000F to RxPDO offset {}",
                kMotionRxPDOOffset);
        }

        // Wait for the cyclic task to send the PDO data a few times
        Tether::Platform::Clock::instance().delayMilliseconds(100);

        // Read back Controlword via SDO
        auto& sdo = master.ethercatMaster().sdoManager(slave_idx);
        {
            auto cw = sdo.readU16(0x6040, 0, {.timeout_ms = kSdoTimeoutMs});
            if (cw.has_value())
                TETHER_LOGI(TAG, "  SDO read 0x6040:0 (Controlword) = 0x{:04X} (expect 0x000F if PDO readback works)",
                    cw.value());
            else
                TETHER_LOGW(TAG, "  SDO read 0x6040:0 (Controlword) FAILED ({})", coeErrorDetail(cw.error()));
        }

        // Now write a known FSoE Reset frame to the FSoE PDO region
        // using the FSoE::buildResetPdu helper (zero-fills data/CRC bytes
        // and writes the Connection ID at the end of the buffer).
        {
            uint8_t* rx_buf = static_cast<uint8_t*>(drive.getRxPDOBuffer());
            FSoE::buildResetPdu(rx_buf + kFSoERxPDOOffset,
                                sizeof(FSoERxPDO), args.connection_id);
            TETHER_LOGI(TAG, "Wrote FSoE Reset pattern to RxPDO offset {}",
                kFSoERxPDOOffset);
        }

        // Wait for the cyclic task to send the PDO data a few times
        Tether::Platform::Clock::instance().delayMilliseconds(200);

        // Read back the FSoE objects via SDO
        auto read_u8 = [&](uint16_t idx, uint8_t sub, const char* name) -> void {
            auto res = sdo.readU8(idx, sub, {.timeout_ms = kSdoTimeoutMs});
            if (res.has_value())
                TETHER_LOGI(TAG, "  SDO read 0x{:04X}:{} ({}) = 0x{:02X}", idx, sub, name, res.value());
            else
                TETHER_LOGW(TAG, "  SDO read 0x{:04X}:{} ({}) error: {}", idx, sub, name, coeErrorDetail(res.error()));
        };
        auto read_u16 = [&](uint16_t idx, uint8_t sub, const char* name) -> void {
            auto res = sdo.readU16(idx, sub, {.timeout_ms = kSdoTimeoutMs});
            if (res.has_value())
                TETHER_LOGI(TAG, "  SDO read 0x{:04X}:{} ({}) = 0x{:04X}", idx, sub, name, res.value());
            else
                TETHER_LOGW(TAG, "  SDO read 0x{:04X}:{} ({}) error: {}", idx, sub, name, coeErrorDetail(res.error()));
        };
        auto read_u32 = [&](uint16_t idx, uint8_t sub, const char* name) -> void {
            auto res = sdo.readU32(idx, sub, {.timeout_ms = kSdoTimeoutMs});
            if (res.has_value())
                TETHER_LOGI(TAG, "  SDO read 0x{:04X}:{} ({}) = 0x{:08X}", idx, sub, name, res.value());
            else
                TETHER_LOGW(TAG, "  SDO read 0x{:04X}:{} ({}) error: {}", idx, sub, name, coeErrorDetail(res.error()));
        };

        TETHER_LOGI(TAG, "Reading back FSoE objects via SDO:");
        read_u8 (0x6770, 1, "FSoE Command (expect 0x2A)");
        read_u16(0x6770, 2, "FSoE ConnectionID (expect 0x0006)");
        read_u16(0x6770, 3, "FSoE CRC_0 (expect 0x0000)");
        read_u16(0x6770, 4, "FSoE CRC_1 (expect 0x0000)");
        read_u16(0x6640, 0, "STO (expect 0x0000)");
        read_u16(0x26F0, 1, "Safe output 1 (expect 0x0000)");

        // Also read back the TxPDO FSoE objects to see what the slave sends
        TETHER_LOGI(TAG, "Reading slave TxPDO FSoE objects via SDO:");
        read_u8 (0x6760, 1, "FSoE Command (slave->master)");
        read_u16(0x6760, 2, "FSoE ConnectionID (slave->master)");

        // Check PDO mapping objects  -  these tell us if the slave has
        // actually configured the FSoE PDO entries
        TETHER_LOGI(TAG, "Checking PDO mapping objects:");
        read_u8 (0x1700, 0, "RxPDO 0x1700 mapping count (expect 18)");
        read_u8 (0x1B00, 0, "TxPDO 0x1B00 mapping count (expect 18)");

        // Full PDO mapping readout and module identification are not needed
        // for normal operation  -  commented out to reduce SDO traffic and
        // log noise.  Uncomment for low-level PDO layout debugging.
#if 0
        // Read the full PDO mapping to understand the actual byte layout
        TETHER_LOGI(TAG, "Reading full RxPDO 0x1700 mapping:");
        {
            auto cnt = sdo.readU8(0x1700, 0, {.timeout_ms = kSdoTimeoutMs});
            if (cnt.has_value()) {
                for (uint8_t i = 1; i <= cnt.value() && i <= 30; i++) {
                    auto entry = sdo.readU32(0x1700, i, {.timeout_ms = kSdoTimeoutMs});
                    if (entry.has_value()) {
                        uint32_t v = entry.value();
                        uint16_t idx = (v >> 16) & 0xFFFF;
                        uint8_t sub = (v >> 8) & 0xFF;
                        uint8_t bits = v & 0xFF;
                        TETHER_LOGI(TAG, "  0x1700:{} = 0x{:08X} (idx=0x{:04X} sub={} bits={})",
                            i, v, idx, sub, bits);
                    } else {
                        TETHER_LOGW(TAG, "  0x1700:{} FAILED ({})", i, coeErrorDetail(entry.error()));
                    }
                }
            }
        }
        TETHER_LOGI(TAG, "Reading full TxPDO 0x1B00 mapping:");
        {
            auto cnt = sdo.readU8(0x1B00, 0, {.timeout_ms = kSdoTimeoutMs});
            if (cnt.has_value()) {
                for (uint8_t i = 1; i <= cnt.value() && i <= 50; i++) {
                    auto entry = sdo.readU32(0x1B00, i, {.timeout_ms = kSdoTimeoutMs});
                    if (entry.has_value()) {
                        uint32_t v = entry.value();
                        uint16_t idx = (v >> 16) & 0xFFFF;
                        uint8_t sub = (v >> 8) & 0xFF;
                        uint8_t bits = v & 0xFF;
                        TETHER_LOGI(TAG, "  0x1B00:{} = 0x{:08X} (idx=0x{:04X} sub={} bits={})",
                            i, v, idx, sub, bits);
                    } else {
                        TETHER_LOGW(TAG, "  0x1B00:{} FAILED ({})", i, coeErrorDetail(entry.error()));
                    }
                }
            }
        }

        // Check module identification (ETG.5000 modular device profile)
        TETHER_LOGI(TAG, "Checking module identification:");
        read_u16(0xF002, 0, "ModuleIdentList count");
        if (true) {
            auto cnt = sdo.readU16(0xF002, 0, {.timeout_ms = kSdoTimeoutMs});
            if (cnt.has_value()) {
                for (uint16_t i = 1; i <= cnt.value() && i <= 4; i++) {
                    char label[64];
                    snprintf(label, sizeof(label), "ModuleIdent[%u]", i);
                    read_u16(0xF002, i, label);
                }
            }
        }
#endif

        // Check 0x2620 (General safety object) subindices
        TETHER_LOGI(TAG, "Checking safety general object (0x2620):");
        read_u8 (0x2620, 0, "0x2620 count");
        read_u8 (0x2620, 1, "0x2620:1 (unknown)");
        read_u8 (0x2620, 2, "Safe fieldbus (expect 255=active)");
        read_u16(0x2620, 3, "Safe address");
        read_u8 (0x2620, 4, "0x2620:4 (unknown)");

        // Check 0xF980 (Device Safety Address)
        TETHER_LOGI(TAG, "Checking Device Safety Address (0xF980):");
        read_u8 (0xF980, 0, "0xF980 count");
        read_u16(0xF980, 1, "Device Safety Address");

        // Check 0xF030 (Configured module ident list) and 0xF050 (Detected Module Ident List)
        // 0xF030:2 = 0x22D20001 → No parameter changes via master
        // 0xF030:2 = 0x22D20002 → With parameter changes via master
        TETHER_LOGI(TAG, "Checking Module ID configuration (0xF030/0xF050):");
        read_u8 (0xF030, 0, "0xF030 count");
        read_u32(0xF030, 1, "0xF030:1 (Module ident pos 1)");
        read_u32(0xF030, 2, "0xF030:2 (Module ident pos 2  —  0x22D20001=no-param, 0x22D20002=with-param)");
        read_u8 (0xF050, 0, "0xF050 count");
        read_u32(0xF050, 1, "0xF050:1 (Detected Module ident pos 1)");
        read_u32(0xF050, 2, "0xF050:2 (Detected Module ident pos 2)");

        // Check 0x2610 (Manufacturing parameters)
        TETHER_LOGI(TAG, "Checking manufacturing parameters (0x2610):");
        read_u8 (0x2610, 0, "0x2610 count");

        // Check 0x2621 (Safety digital IO)
        TETHER_LOGI(TAG, "Checking safety digital IO (0x2621):");
        read_u8 (0x2621, 0, "0x2621 count");

        // Read Safety statusword (0x6621)  -  this tells us the FSoE module's
        // internal state and might explain why it's not responding
        TETHER_LOGI(TAG, "Reading safety statusword (0x6621):");
        read_u8(0x6621, 0, "Safety statusword count");
        read_u8(0x6621, 1, "Safety status byte 1 (STO)");
        read_u8(0x6621, 2, "Safety status byte 2 (SBC)");

        // Read error report (0x203F)  -  may contain safety-related errors
        TETHER_LOGI(TAG, "Reading error report (0x203F):");
        read_u8(0x203F, 0, "Error report count");
        {
            uint8_t err_buf[16] = {};
            size_t err_len = 0;
            if (sdo.readSync(0x203F, 1, err_buf, sizeof(err_buf), kSdoTimeoutMs, &err_len)) {
                char err_str[17];
                size_t copy_len = err_len < 8 ? err_len : 8;
                for (size_t i = 0; i < copy_len; i++) {
                    err_str[i] = (err_buf[i] >= 0x20 && err_buf[i] < 0x7F) ? static_cast<char>(err_buf[i]) : '.';
                }
                err_str[copy_len] = '\0';
                TETHER_LOGI(TAG, "  SDO read 0x203F:1 (Error report) = '{}' (len={}, hex: {:02X} {:02X} {:02X} {:02X} {:02X} {:02X} {:02X} {:02X})",
                    err_str, err_len,
                    err_buf[0], err_buf[1], err_buf[2], err_buf[3],
                    err_buf[4], err_buf[5], err_buf[6], err_buf[7]);
            } else {
                const uint32_t abort_code = sdo.lastSdoAbortCode();
                TETHER_LOGW(TAG, "  SDO read 0x203F:1 (Error report) FAILED ({} 0x{:08X})",
                            EtherCAT::sdoAbortCodeStr(abort_code), abort_code);
            }
        }

        TETHER_LOGI(TAG, "=== End PDO/SDO diagnostic ===");

        // Note: PDO exchange stays enabled  -  the FSoE cyclic task needs it
    }

    // --- Configure PDO-based field offsets ---
    // The motion PDO is at kMotionRxPDOOffset / motion_tx_pdo_offset in the
    // combined buffer (after the FSoE PDO — the Tx offset depends on the
    // FSoE frame variant).  These offsets keep the drive object's
    // statusword/controlword decoding consistent with the buffer layout
    // even though no motion commands are sent.
    {
        // Controlword (0x6040) is at the start of the motion RxPDO
        drive.setControlwordPDOOffset(static_cast<int>(kMotionRxPDOOffset));

        // Statusword (0x6041) is at the start of the motion TxPDO
        drive.setStatuswordPDOOffset(static_cast<int>(motion_tx_pdo_offset));

        const size_t opmode_offset = kMotionRxPDOOffset +
            offsetof(EtherCAT::Drives::SynapticonPDO::SOMANET_RxPDO_1600,
                    modes_of_operation);
        drive.setOpmodePDOOffset(static_cast<int>(opmode_offset));

        // Velocity field offsets for controlledShutdown()'s CSV stop
        // phase — the motion PDOs sit behind the FSoE PDO, so the struct
        // member offsets are shifted by the PDO base offsets.
        drive.setTargetVelocityPDOOffset(static_cast<int>(
            kMotionRxPDOOffset +
            offsetof(EtherCAT::Drives::SynapticonPDO::SOMANET_RxPDO_1600,
                     target_velocity)));
        drive.setActualVelocityPDOOffset(static_cast<int>(
            motion_tx_pdo_offset +
            offsetof(EtherCAT::Drives::SynapticonPDO::SOMANET_TxPDO_1A00,
                     velocity_actual)));

        TETHER_LOGI(TAG,
            "PDO offsets: controlword={} statusword={} opmode={} "
            "target_velocity={} velocity_actual={}",
            kMotionRxPDOOffset, motion_tx_pdo_offset, opmode_offset,
            drive.targetVelocityPDOOffset(),
            drive.actualVelocityPDOOffset());
    }

    // --- Brake release ---
    // The Synapticon brake is spring-activated (engages when powered off).
    // It is released via CoE SDO write to 0x2004:7 (Brake status) once the
    // drive confirms STO=off and SBC=off in its FSoE status feedback  -  see
    // the run loop below.
    // Object 0x2004:7 controls the brake in automatic mode.
    // See: https://doc.synapticon.com/node/sw5.1/objects_html/2xxx/2004.html
    uint64_t fsoe_data_time_ms = 0;  // timestamp when FSoE Data state was reached
    bool brake_released = false;
    bool drive_enabled = false;      // set once enableDrive() succeeds (motion mode)

    // --- Set up FSoE safe-motion (real drive via PDOs) ---
    std::unique_ptr<FSoEMain> fsoe_main;
    FSoEPDOExchangeTask* fsoe_task_ptr = nullptr;  // raw ptr for suppressDebug()

    // --- Thread synchronization for FSoE Data-state detection ---
    //
    // The FSoE state machine runs in the realtime motion loop thread, while
    // the main thread monitors progress below.  We use a std::promise<bool>
    // as a one-shot signaling mechanism:
    //   true  → FSoE reached Data state
    //   false → FSoE entered Error state
    //
    // The promise is fulfilled from the state change callback (realtime
    // thread).  The main thread blocks on the shared_future with a timeout.
    // An atomic<bool> guard ensures set_value() is called exactly once,
    // even if the state machine transitions through multiple terminal states.
    auto fsoe_ready_promise = std::make_unique<std::promise<bool>>();
    std::shared_future<bool> fsoe_ready_future = fsoe_ready_promise->get_future();
    std::atomic<bool> fsoe_signaled{false};

    auto signal_fsoe = [&fsoe_ready_promise, &fsoe_signaled](bool success) {
        if (!fsoe_signaled.exchange(true)) {
            fsoe_ready_promise->set_value(success);
        }
    };

    {
        // FSoE debug flags are part of the Tether debug framework and
        // support per-slave filtering.  They are applied via applyDebugFlags()
        // above (same call as the EtherCAT framework flags).  Query them here
        // for the target slave.
        const auto& dbg = master.ethercatMaster().debugFlags();
        const bool debug_fsoe         = dbg.isEnabled("fsoe",         slave_idx);
        const bool debug_fsoe_frame   = dbg.isEnabled("fsoe-frame",   slave_idx);
        const bool debug_fsoe_raw     = dbg.isEnabled("fsoe-raw",     slave_idx);
        const bool debug_fsoe_wire    = dbg.isEnabled("fsoe-wire",    slave_idx);
        const bool debug_fsoe_sequence= dbg.isEnabled("fsoe-sequence",slave_idx);
        const bool debug_fsoe_crc     = dbg.isEnabled("fsoe-crc",     slave_idx);

        EtherCAT::Drives::Synapticon::SafeMotion::MainConfig main_config;
        main_config.feature_enabled = true;
        main_config.slave_address = slave_idx;
        main_config.safety_address = args.safety_address;  // FSoE slave safety address
        main_config.connection_id = args.connection_id;    // FSoE connection ID
        main_config.master_address = 0x0001;
        main_config.watchdog_time_ms = args.watchdog_ms;
        main_config.frame_variant = fsoe_variant;

        fsoe_main = std::make_unique<FSoEMain>(main_config);

        if (!fsoe_main->initialize()) {
            TETHER_LOGE(TAG, "FSoE initialization failed");
            rc = 5;
            master.clearCyclicTasks();
            Tether::Examples::shutdownSingleDrive(master, slave_idx);
            Tether::Examples::stopHostMasterSession(master, session);
            return rc;
        }

        fsoe_main->requestMotionEnabled();

        // Apply STO/SOS/SBC overrides from command-line flags.
        // The flags control RAW bit values in the PDO (0=bit cleared, 1=bit set).
        // The codec uses setZeroActive for STO, SOS, and SBC, which inverts:
        //   setZeroActive(true)  → bit=0
        //   setZeroActive(false) → bit=1
        // So to get raw bit=0, we set the Command field to true (active),
        // and to get raw bit=1, we set it to false (inactive).
        //
        // SOS defaults to the STO override value when --sos is not given,
        // so SOS always tracks STO unless explicitly overridden.
        if (args.sto_override >= 0 || args.sos_override >= 0 ||
            args.sbc_override >= 0) {
            auto cmd = fsoe_main->command();

            // Determine the raw bit value to apply to ALL zero-active safety
            // bits.  STO and SBC must agree; if only one is given, the other
            // defaults to the same value.  SOS defaults to STO's value.
            // All other zero-active fields (SS1, SS2, SLS1-4) follow suit.
            int raw_bit;
            if (args.sto_override >= 0 && args.sbc_override >= 0) {
                raw_bit = args.sto_override;  // both must match anyway
            } else if (args.sto_override >= 0) {
                raw_bit = args.sto_override;
            } else {
                raw_bit = args.sbc_override;
            }

            // SOS can be overridden independently; defaults to raw_bit (STO).
            const int sos_raw_bit =
                (args.sos_override >= 0) ? args.sos_override : raw_bit;

            // setZeroActive: bit=0 when active=true, bit=1 when active=false
            const bool active = (raw_bit == 0);
            const bool sos_active = (sos_raw_bit == 0);

            cmd.sto           = active;
            cmd.ss1           = active;
            cmd.ss2           = active;
            cmd.sos           = sos_active;
            cmd.sls           = {{active, active, active, active}};
            cmd.brake_engage  = active;

            TETHER_LOGI(TAG,
                "[safety-command] STO raw_bit={} SOS raw_bit={} SBC raw_bit={} "
                "(active={}/{}/{} → {})",
                raw_bit, sos_raw_bit, raw_bit,
                active, sos_active, active,
                active ? "ALL safety functions ACTIVE (safe state)"
                       : "ALL safety functions INACTIVE (motion enabled)");

            fsoe_main->setCommand(cmd);
        }

        // Install FSoE callbacks for real-time state tracking.
        // The state change and error callbacks also signal the main thread
        // via the promise/future pair declared above the FSoE block.
        fsoe_main->rawConnection().setStateChangeCallback(
            [&signal_fsoe](uint8_t old_s, uint8_t new_s) {
                TETHER_LOGI(TAG,
                    "[FSoE] state: {} -> {}",
                    FSoE::fsoeStateName(old_s), FSoE::fsoeStateName(new_s));
                if (new_s == FSoE::ConnectionState::Data) {
                    signal_fsoe(true);
                } else if (new_s == FSoE::ConnectionState::Error) {
                    signal_fsoe(false);
                }
            });
        fsoe_main->rawConnection().setErrorCallback(
            [&signal_fsoe, &fsoe_main](uint16_t code, const FSoE::FSoEErrorDetail& detail) {
                if (detail.message[0] != '\0') {
                    TETHER_LOGE(TAG,
                        "[FSoE] error: 0x{:04X} ({}): {}",
                        code, FSoE::fsoeErrorName(code), detail.message);
                } else {
                    TETHER_LOGE(TAG,
                        "[FSoE] error: 0x{:04X} ({})",
                        code, FSoE::fsoeErrorName(code));
                }
                // Only signal failure if the master is NOT auto-recovering.
                // With auto_fail_safe_on_error=true (default), handshake errors
                // trigger resetConnection()  -  the master goes back to Reset
                // and retries.  The error callback fires AFTER the state
                // transition, so the state is already Reset (recovering),
                // Error (gave up), or Data+fail_safe.  Signalling failure on
                // every error would abort before the retry happens.
                const auto state = fsoe_main->rawConnection().getState();
                if (state == FSoE::ConnectionState::Error) {
                    signal_fsoe(false);
                } else if (state == FSoE::ConnectionState::Data &&
                           fsoe_main->rawConnection().getStatus().isFailSafe()) {
                    signal_fsoe(false);
                }
            });
        fsoe_main->rawConnection().setFailSafeCallback(
            []() {
                TETHER_LOGW(TAG, "[FSoE] fail-safe activated");
            });

        // High-level protocol trace (--debug fsoe or --debug fsoe-raw).
        // The trace callback emits human-readable descriptions of every
        // protocol decision: what the master is sending and why, what it
        // received from the slave and how it interpreted it.
        if (debug_fsoe || debug_fsoe_raw) {
            fsoe_main->rawConnection().setTraceCallback(
                [](const char* message) {
                    TETHER_LOGI(TAG, "[fsoe] {}", message);
                });
        }

        // Per-cycle sequence trace (--debug fsoe-sequence).
        // Emits one line per exchangeViaPDO() call summarizing:
        //   cycle N: state_before -> state_after  cmd=0xXX  accepted/rejected  reason
        if (debug_fsoe_sequence) {
            fsoe_main->rawConnection().setSequenceTraceCallback(
                [](const FSoE::SequenceTraceInfo& info) {
                    const char* arrow = info.state_changed ? " -> " : " == ";
                    TETHER_LOGI(TAG,
                        "[fsoe-seq] cycle {}: {}{}{}  cmd=0x{:02X}  {}{}  reason={}",
                        info.cycle,
                        FSoE::fsoeStateName(info.state_before),
                        arrow,
                        FSoE::fsoeStateName(info.state_after),
                        info.rx_cmd,
                        info.frame_accepted ? "ACCEPTED" : "REJECTED",
                        info.tx_rebuilt ? " (tx rebuilt)" : "",
                        info.reason);
                });
        }

        // CRC parameter trace (--debug fsoe-crc).
        // Emits the exact CRC inputs and outputs for every frame built (TX)
        // and checked (RX): start_crc, seq_expected, seq_used, CRC0, and
        // whether the seq±1 fallback was used.  This is the low-level
        // diagnostic view for debugging CRC/seq synchronization issues.
        if (debug_fsoe_crc) {
            fsoe_main->rawConnection().setCrcTraceCallback(
                [](const FSoE::CrcTraceInfo& info) {
                    // Format SafeData bytes as hex string.
                    char data_hex[64] = {};
                    size_t pos = 0;
                    for (size_t i = 0; i < info.data_len && pos < sizeof(data_hex) - 4; i++) {
                        pos += static_cast<size_t>(snprintf(
                            data_hex + pos, sizeof(data_hex) - pos, "%02X ", info.data[i]));
                    }
                    if (pos == 0) {
                        snprintf(data_hex, sizeof(data_hex), "(none)");
                    }

                    if (info.direction ==
                        FSoE::CrcTraceInfo::Direction::TX) {
                        // TX: master building a frame.
                        // CRC inputs (in byte processing order):
                        //   start_crc(Lo,Hi), conn_id(Lo,Hi),
                        //   seq(Lo,Hi), command, data[0..n-1]
                        TETHER_LOGI(TAG,
                            "[fsoe-crc] TX {} cmd=0x{:02X}: "
                            "start_crc=0x{:04X} seq={} -> "
                            "CRC0=0x{:04X} | "
                            "CRC inputs: oldCRC=0x{:04X} conn_id=0x{:04X} "
                            "seq={} cmd=0x{:02X} data[{}]={{}}",
                            FSoE::fsoeStateName(info.state),
                            info.command,
                            info.start_crc, info.seq_used,
                            info.crc0,
                            info.start_crc, info.conn_id,
                            info.seq_used, info.command,
                            info.data_len, data_hex);
                    } else {
                        // RX: master checking a frame.
                        // Compute the expected CRC0 from the master's
                        // parameters to show the mismatch directly.
                        uint16_t expected_crc0 = 0;
                        if (info.data_len > 0) {
                            uint8_t tmpbuf[32];
                            FSoE::CRC::buildFSoEFrame(
                                tmpbuf, info.command,
                                info.data, info.data_len,
                                info.conn_id,
                                info.start_crc,
                                info.crc_ok ? info.seq_used : info.seq_expected,
                                &expected_crc0);
                        }

                        if (info.crc_ok) {
                            const char* fb = "";
                            char fb_buf[64] = {};
                            if (info.fallback_used) {
                                snprintf(fb_buf, sizeof(fb_buf),
                                    " [FALLBACK seq%+d: expected=%u]",
                                    info.fallback_delta, info.seq_expected);
                                fb = fb_buf;
                            }
                            TETHER_LOGI(TAG,
                                "[fsoe-crc] RX {} cmd=0x{:02X}: "
                                "start_crc=0x{:04X} seq={} -> "
                                "CRC0=0x{:04X} OK (expected=0x{:04X}) | "
                                "CRC inputs: oldCRC=0x{:04X} conn_id=0x{:04X} "
                                "seq={} cmd=0x{:02X} data[{}]={{}}{}",
                                FSoE::fsoeStateName(info.state),
                                info.command,
                                info.start_crc, info.seq_used,
                                info.crc0, expected_crc0,
                                info.start_crc, info.conn_id,
                                info.seq_used, info.command,
                                info.data_len, data_hex, fb);
                        } else {
                            TETHER_LOGE(TAG,
                                "[fsoe-crc] RX {} cmd=0x{:02X}: "
                                "start_crc=0x{:04X} seq={} -> "
                                "CRC FAIL: received=0x{:04X} expected=0x{:04X} | "
                                "CRC inputs: oldCRC=0x{:04X} conn_id=0x{:04X} "
                                "seq={} cmd=0x{:02X} data[{}]={{}}",
                                FSoE::fsoeStateName(info.state),
                                info.command,
                                info.start_crc, info.seq_expected,
                                info.crc0, expected_crc0,
                                info.start_crc, info.conn_id,
                                info.seq_expected, info.command,
                                info.data_len, data_hex);
                        }
                    }
                });
        }

        // Raw frame hex dumps (--debug fsoe-raw).
        // Frame event listeners are invoked from inside the FSoE state
        // machine for every master-to-slave (tx) and slave-to-master (rx)
        // frame.  Each listener receives an immutable shared_ptr<const
        // std::vector<uint8_t>> copy of the frame bytes.
        // Only prints when the frame content changes from the previous cycle.
        if (debug_fsoe_raw) {
            auto last_tx = std::make_shared<std::vector<uint8_t>>();
            auto last_rx = std::make_shared<std::vector<uint8_t>>();
            fsoe_main->rawConnection().txFrameEvents().addListener(
                [last_tx](std::shared_ptr<const std::vector<uint8_t>> data) {
                    if (*data == *last_tx) return;  // skip unchanged
                    *last_tx = *data;
                    const uint8_t cmd = (!data->empty()) ? (*data)[0] : 0;
                    TETHER_LOGI(TAG, "[fsoe-raw] TX (master->slave) len={} cmd={}",
                                data->size(), FSoE::fsoeCommandName(cmd));
                    hexDump(TAG, "TX (master->slave)", data->data(), data->size());
                });
            fsoe_main->rawConnection().rxFrameEvents().addListener(
                [last_rx](std::shared_ptr<const std::vector<uint8_t>> data) {
                    if (*data == *last_rx) return;  // skip unchanged
                    *last_rx = *data;
                    const uint8_t cmd = (!data->empty()) ? (*data)[0] : 0;
                    TETHER_LOGI(TAG, "[fsoe-raw] RX (slave->master) len={} cmd={}",
                                data->size(), FSoE::fsoeCommandName(cmd));
                    hexDump(TAG, "RX (slave->master)", data->data(), data->size());
                });
        }

        // Add FSoE PDO exchange as a cyclic task.
        // This exchanges FSoE frames with the REAL drive via the FSoE safety
        // PDOs (0x1700/0x1B00) mapped in the combined PDO buffer.  The FSoE
        // PDOs are at a fixed offset after the motion PDO data.
        auto fsoe_task = std::make_unique<FSoEPDOExchangeTask>(
            slave_idx, *fsoe_main,
            kFSoERxPDOOffset, kFSoETxPDOOffset,
            fsoe_variant, sm3_total_len,
            debug_fsoe_raw, debug_fsoe_frame, debug_fsoe_wire);
        fsoe_task_ptr = fsoe_task.get();
        if (!master.addCyclicTask(std::move(fsoe_task))) {
            TETHER_LOGE(TAG, "Failed to add FSoE PDO exchange task");
            rc = 6;
            master.clearCyclicTasks();
            Tether::Examples::shutdownSingleDrive(master, slave_idx);
            Tether::Examples::stopHostMasterSession(master, session);
            return rc;
        }

        // Add diagnostics task
        if (!master.addCyclicTask(
                std::make_unique<FSoEDiagnosticsTask>(
                    slave_idx, *fsoe_main, args.diag_interval_ms,
                    motion_tx_pdo_offset))) {
            TETHER_LOGW(TAG, "Failed to add FSoE diagnostics task (non-fatal)");
        }

        // Add safety state confirmation monitor (only if overrides are set)
        if (args.sto_override >= 0 || args.sbc_override >= 0) {
            const auto& cmd = fsoe_main->command();
            // The monitor expects the drive's status to match the Command's
            // semantic fields (sto_active should match cmd.sto, brake_engaged
            // should match cmd.brake_engage).
            if (!master.addCyclicTask(
                    std::make_unique<SafetyStateMonitor>(
                        *fsoe_main,
                        args.sto_override >= 0, args.sbc_override >= 0,
                        args.sto_override >= 0 ? cmd.sto : false,
                        args.sbc_override >= 0 ? cmd.brake_engage : false))) {
                TETHER_LOGW(TAG, "Failed to add safety state monitor (non-fatal)");
            }
        }

        TETHER_LOGI(TAG,
            "FSoE enabled: conn_id=0x{:04X} safety_addr=0x{:04X} watchdog={} ms debug={}{}{}{}{}{}",
            args.connection_id, args.safety_address, args.watchdog_ms,
            debug_fsoe ? "fsoe" : "off",
            debug_fsoe_frame ? "+frame" : "",
            debug_fsoe_raw ? "+raw" : "",
            debug_fsoe_wire ? "+wire" : "",
            debug_fsoe_sequence ? "+seq" : "",
            debug_fsoe_crc ? "+crc" : "");
    }

    // --- Optional CST sine-torque motion (--torque-nm) ---
    // The motion controller must be installed BEFORE the realtime loop
    // starts: DS402Master::motion_controllers_ is mutated here (main
    // thread) and iterated by the loop thread, so adding a controller
    // while the loop runs is not safe.
    //
    // The controller writes the controlword/modes_of_operation every
    // cycle so the CiA 402 enable sequence can proceed, but forces
    // target_torque to zero until the FSoE link is operational and the
    // drive reports motion allowed.  The drive is only enabled (and the
    // brake released) from the run loop below once the FSoE status
    // confirms STO=off and SBC=off.
    bool motion_active = false;
    if (args.torque_pp_nm > 0.0) {
        // Read motor rated torque (0x6076) for Nm→per-mille conversion.
        // CiA 402 target_torque (0x6071) is in per-mille (0.1%) of rated
        // torque; 0x6076 (Motor Rated Torque) is in mNm.
        uint32_t rated_torque_mnm = args.rated_torque_mnm;
        if (rated_torque_mnm == 0) {
            auto& sdo_mgr = master.ethercatMaster().sdoManager(slave_idx);
            auto rated = sdo_mgr.readU32(0x6076, 0, {.timeout_ms = kSdoTimeoutMs});
            if (rated.has_value() && rated.value() > 0) {
                rated_torque_mnm = rated.value();
                TETHER_LOGI(TAG,
                    "Motor rated torque (0x6076) = {} mNm ({:.3f} Nm)",
                    rated_torque_mnm, rated_torque_mnm / 1000.0);
            } else {
                TETHER_LOGE(TAG,
                    "Failed to read motor rated torque (0x6076) via SDO  —  "
                    "cannot convert Nm to per-mille.  Use --rated-torque-mnm "
                    "to specify it manually.");
                master.stopDistributedClocks();
                Tether::Examples::shutdownSingleDrive(master, slave_idx);
                Tether::Examples::stopHostMasterSession(master, session);
                return 3;
            }
        } else {
            TETHER_LOGI(TAG,
                "Using --rated-torque-mnm = {} mNm ({:.3f} Nm)",
                rated_torque_mnm, rated_torque_mnm / 1000.0);
        }

        // Amplitude is half the peak-to-peak value (±torque_pp_nm/2).
        const double amplitude_nm = args.torque_pp_nm / 2.0;
        TETHER_LOGI(TAG,
            "Sine torque: {:.3f} Nm P-P ({:.3f} Nm amplitude) at {:.3f} Hz, "
            "rated torque {} mNm -> {:.1f} per-mille peak "
            "(gated on FSoE motion-allowed)",
            args.torque_pp_nm, amplitude_nm, args.freq_hz,
            rated_torque_mnm,
            amplitude_nm * 1'000'000.0 / static_cast<double>(rated_torque_mnm));

        if (!master.addMotionController(
                slave_idx,
                std::make_unique<SineTorqueController>(
                    amplitude_nm, args.freq_hz, rated_torque_mnm,
                    kMotionRxPDOOffset, *fsoe_main))) {
            TETHER_LOGE(TAG, "Failed to add sine torque controller");
            master.stopDistributedClocks();
            Tether::Examples::shutdownSingleDrive(master, slave_idx);
            Tether::Examples::stopHostMasterSession(master, session);
            return 4;
        }
        motion_active = true;
    }

    // --- Start realtime motion loop ---
    // The cyclic-task scheduler (which runs FSoEPDOExchangeTask) is driven
    // by the realtime motion loop; the loop is what actually exchanges the
    // FSoE PDOs (and the motion PDOs when --torque-nm is given).
    EtherCAT::Master::RealtimeMotionLoopConfig loop_config;
    loop_config.cycle_period_us = 1000;
    loop_config.sync_interval_cycles = 10;
    loop_config.enable_dc_synchronization = args.enable_dc_sync;
    if (!master.startRealtimeMotionControlLoop(loop_config)) {
        TETHER_LOGE(TAG, "Failed to start realtime motion loop");
        rc = 7;
        master.clearCyclicTasks();
        Tether::Examples::shutdownSingleDrive(master, slave_idx);
        Tether::Examples::stopHostMasterSession(master, session);
        return rc;
    }

    // --- Wait for FSoE to reach Data state ---
    // The drive starts in SAFE STATE when FSoE safety is active.  The FSoE
    // master must reach the Data state to exchange real safety data.
    //
    // The FSoE state machine runs in the realtime motion loop thread.  We
    // block here on the std::shared_future that is fulfilled by the state
    // change callback (see above) when Data state is reached or an Error
    // occurs.  This is the idiomatic C++ one-shot cross-thread signaling
    // pattern: no polling, no sleep loops, no shared mutable state beyond
    // the promise/future pair.
    bool fsoe_data_reached = false;
    {
        TETHER_LOGI(TAG, "Waiting for FSoE Data state (timeout 5 s)...");
        using namespace std::chrono_literals;
        const auto fsoe_status = fsoe_ready_future.wait_for(5s);
        if (fsoe_status == std::future_status::timeout) {
            TETHER_LOGE(TAG,
                "FSoE did not reach Data state within 5 s  —  "
                "current state: {}.  Aborting.",
                FSoE::fsoeStateName(fsoe_main->rawConnection().getState()));
            rc = 9;
        } else if (!fsoe_ready_future.get()) {
            TETHER_LOGE(TAG,
                "FSoE entered Error state before reaching Data  —  "
                "aborting.");
            rc = 9;
        } else {
            TETHER_LOGI(TAG, "FSoE Data state reached");
            fsoe_data_reached = true;
            fsoe_data_time_ms = Tether::Platform::Clock::instance().getMilliseconds();
        }
    }

    // --- Run loop ---
    // Monitor the FSoE connection for the requested duration.  If the slave
    // enters FailSafe or Error (e.g. sends a Reset due to a CRC error),
    // shut down cleanly immediately rather than continuing to run in an
    // unsafe state.
    //
    // If --diagnostics-after is set, after that many seconds of FSoE
    // operation, suppress all FSoE debug output, run full safety
    // diagnostics via CoE (SDO reads), then exit immediately.
    if (rc == 0) {
        if (motion_active) {
            TETHER_LOGI(TAG,
                "FSoE + CST motion: {:.3f} Nm P-P at {:.3f} Hz for {:.1f} s "
                "(waiting for STO/SBC release before enabling drive)",
                args.torque_pp_nm, args.freq_hz, args.duration);
        } else {
            TETHER_LOGI(TAG,
                "FSoE-only mode: monitoring safety state for {:.1f} s (no motion)",
                args.duration);
        }

        const auto run_start_ms = Tether::Platform::Clock::instance().getMilliseconds();
        const uint32_t run_duration_ms =
            static_cast<uint32_t>(args.duration * 1000.0);
        const uint32_t diag_after_ms =
            static_cast<uint32_t>(args.diagnostics_after * 1000.0);
        bool diag_done = false;

        while (true) {
            const auto elapsed_ms =
                Tether::Platform::Clock::instance().getMilliseconds() - run_start_ms;

            // --- Diagnostics trigger ---
            if (!diag_done && diag_after_ms > 0 && elapsed_ms >= diag_after_ms) {
                diag_done = true;
                TETHER_LOGI(TAG,
                    "=== --diagnostics-after={:.1f}s reached  —  "
                    "suppressing FSoE output and running CoE diagnostics ===",
                    args.diagnostics_after);

                // Suppress all FSoE-related debug output so the diagnostics
                // output is clean (no interleaved FSoE frame dumps).
                // The debug flags on the master are checked by the FSoE
                // trace callback; the cyclic task has its own local copies
                // that must be suppressed separately.
                auto& dbg = master.ethercatMaster().debugFlags();
                dbg.setFlag("fsoe", false);
                dbg.setFlag("fsoe-frame", false);
                dbg.setFlag("fsoe-raw", false);
                dbg.setFlag("fsoe-wire", false);
                dbg.setFlag("fsoe-sequence", false);
                dbg.setFlag("fsoe-crc", false);
                if (fsoe_task_ptr) {
                    fsoe_task_ptr->suppressDebug();
                }

                // Run full safety diagnostics via CoE SDO reads.
                // FSoE is still running in the realtime loop  -  we're just
                // reading safety objects via SDO in parallel.
                auto& slave_ref = master.ethercatMaster().slave(slave_idx);
                [[maybe_unused]] auto diag_report =
                    EtherCAT::Drives::Synapticon::runFullSafetyDiagnostics(
                        slave_ref);

                TETHER_LOGI(TAG,
                    "=== CoE diagnostics complete  —  exiting ===");
                break;
            }

            if (elapsed_ms >= run_duration_ms) break;

            // --- Release the brake once STO=off and SBC=off are confirmed ---
            // The brake is spring-activated and must be disengaged via CoE
            // (0x2004:7).  We wait for the slave to confirm STO=off (torque
            // allowed) and SBC=off (brake released at the safety level) in
            // its FSoE status feedback, then immediately disengage the
            // physical brake via CoE.
            if (!brake_released && fsoe_data_time_ms > 0 &&
                fsoe_main->hasStatus()) {
                const auto& sm = fsoe_main->status();
                if (!sm.sto_active && !sm.brake_engaged) {
                    brake_released = true;
                    const auto since_data_ms =
                        Tether::Platform::Clock::instance().getMilliseconds() -
                        fsoe_data_time_ms;
                    TETHER_LOGI(TAG,
                        "STO=off and SBC=off confirmed by drive after {} ms  —  "
                        "releasing brake via CoE",
                        since_data_ms);
                    auto& brake_sdo = master.ethercatMaster().sdoManager(slave_idx);
                    if (!EtherCAT::Drives::Synapticon::BrakeControl::disengageBrake(
                            brake_sdo, kSdoTimeoutMs)) {
                        TETHER_LOGW(TAG,
                            "Brake disengage failed or unverified");
                    }

                    // --- Enable the drive for CST motion ---
                    // Only now — with the FSoE link in Data state, the drive
                    // reporting motion allowed and the brake released — is
                    // it safe to run the CiA 402 enable sequence.
                    if (motion_active && !drive_enabled) {
                        if (master.enableDrive(slave_idx, 5000)) {
                            drive_enabled = true;
                            TETHER_LOGI(TAG,
                                "Slave {} drive enabled — CST sine torque active",
                                slave_idx);
                        } else {
                            TETHER_LOGE(TAG,
                                "Failed to enable slave {} for CST motion",
                                slave_idx);
                            rc = 8;
                            break;
                        }
                    }
                }
            }

            // If the slave enters FailSafe or Error, stop immediately.
            {
                const auto status = fsoe_main->rawConnection().getStatus();
                if (status.isFailSafe() || status.hasError()) {
                    TETHER_LOGE(TAG,
                        "FSoE entered {} state during run (code=0x{:04X})  —  "
                        "shutting down cleanly",
                        status.isFailSafe() ? "FailSafe" : "Error",
                        status.error_code);
                    rc = 10;
                    break;
                }
            }

            Tether::Platform::Clock::instance().delayMilliseconds(50);
        }
    }

    // --- Stop and clean up ---
    // In motion mode, mirror synapticon_cst's shutdown ordering:
    //   1. Remove the motion controller so it stops overwriting the RxPDO
    //      buffer every cycle.
    //   2. controlledShutdown() runs the CSV stop -> disable -> brake
    //      phases through the still-live PDO exchange.
    //   3. Only then stop the realtime loop.
    if (motion_active) {
        (void)master.removeMotionController(slave_idx);
        EtherCAT::CiA402Drive::ControlledShutdownConfig scfg;
        scfg.stop_mode     = EtherCAT::CiA402Drive::ShutdownStopMode::FixedTimeVelocity;
        scfg.stop_time_ms  = 500;
        scfg.standstill_ms = 25;
        // The SOMANET auto-engages its holding brake when the drive leaves
        // Operation Enabled; phase 3 ensures the engaged state.
        scfg.brake_action = [&master, slave_idx] {
            auto& sdo = master.ethercatMaster().sdoManager(slave_idx);
            return EtherCAT::Drives::Synapticon::BrakeControl::engageBrake(sdo);
        };
        if (!drive.controlledShutdown(scfg)) {
            TETHER_LOGW(TAG, "Controlled shutdown reported errors");
        }
    }
    master.stopMotionControlLoop();
    master.clearCyclicTasks();

    // Final FSoE diagnostics dump
    if (fsoe_main) {
        TETHER_LOGI(TAG, "=== Final FSoE Diagnostics ===");
        TETHER_LOGI(TAG, "{}", fsoe_main->rawConnection().getDiagnostics().c_str());
    }

    // PDO transfer statistics  -  reveals WKC errors (slave not ack'ing frames)
    {
        const auto& pdo = master.ethercatMaster().pdo();
        const auto& st = pdo.getStats();
        TETHER_LOGI(TAG, "=== PDO Transfer Statistics ===");
        TETHER_LOGI(TAG, "  [FMMU] total_cycles={} rx_sent={} tx_recv={}",
            (unsigned long long)st.total_cycles,
            (unsigned long long)st.rxpdo_frames_sent,
            (unsigned long long)st.txpdo_frames_recv);
        TETHER_LOGI(TAG, "  [FMMU] rx_errors={} tx_errors={} wkc_errors={}",
            st.rxpdo_errors, st.txpdo_errors, st.wkc_errors);
        const auto& ps = pdo.getPhysicalStats();
        TETHER_LOGI(TAG, "  [PHYS] fpwr_ok={} fpwr_wkc_err={} fprd_ok={} fprd_wkc_err={}",
            ps.fpwr_success, ps.fpwr_wkc_errors, ps.fprd_success, ps.fprd_wkc_errors);
        TETHER_LOGI(TAG, "  [PHYS] send_err={} timeout_err={}",
            ps.send_errors, ps.timeout_errors);
    }

    Tether::Examples::shutdownSingleDrive(master, slave_idx);
    Tether::Examples::stopHostMasterSession(master, session);
    return rc;
}

#pragma once

#include <array>
#include <cstdint>
#include <optional>
#include <span>

#include "tether/drives/Synapticon/SynapticonPDO.hpp"
#include "tether/fsoe/FSoESlaveEmulator.hpp"
#include "tether/fsoe/TypedProcessData.hpp"
#include "tether/profiles/cia402/DS402Master.hpp"

namespace EtherCAT::Drives::Synapticon::SafeMotion {

namespace PDO = EtherCAT::Drives::SynapticonPDO;

/// diagnostic_flags field mask → absolute bit index in the status safe
/// data: the diagnostic word occupies status safe-data bits 16-31.
constexpr uint8_t diagStatusBit(uint16_t diagnostic_flags_mask)
{
    return ::FSoE::bitIndexOf(
        static_cast<uint32_t>(diagnostic_flags_mask) << 16);
}

struct Timing {
    static constexpr uint16_t kMinimumWatchdogTimeMs = 200;
    static constexpr uint16_t kTypicalMasterToSlaveDelayMs = 8;
    static constexpr uint16_t kTypicalSlaveToMasterDelayMs = 7;
    static constexpr uint16_t kTypicalRoundTripTimeMs = 15;
    static constexpr uint16_t kInternalDriveDelayMs = 2;
};

struct Command {
    bool sto = true;
    bool ss1 = true;
    bool ss2 = true;
    bool sos = true;
    std::array<bool, 4> sls{{false, false, false, false}};
    bool error_acknowledge = false;
    bool restart_acknowledge = false;
    bool brake_engage = true;
    bool reset_position = false;
    bool safe_output_1_high = false;

    static Command safeStop();
    static Command motionEnabled();
};

struct Status {
    bool sto_active = true;
    bool sos_active = true;
    bool error_active = false;
    bool ss1_active = true;
    bool ss2_active = true;
    std::array<bool, 4> sls_active{{false, false, false, false}};
    bool restart_acknowledge_required = false;
    bool brake_engaged = true;
    bool temperature_ok = true;
    bool safe_position_valid = false;
    bool safe_velocity_valid = false;
    bool safe_input_1_high = false;
    bool safe_input_2_high = false;
    bool safe_output_1_high = false;
    bool analog_input_diagnostic_active = false;
    bool analog_input_value_valid = false;
    int32_t safe_position = 0;
    int32_t safe_velocity = 0;
    int16_t safe_analog_input = 0;
    /// LW2 frame variant only — stays 0 when the slave uses LW1 framing.
    int16_t safe_torque = 0;

    bool motionAllowed() const;
};

struct MainConfig {
    uint16_t slave_address = 0x0001;
    uint16_t safety_address = 0x0001;
    uint16_t connection_id = 0x1234;
    uint16_t master_address = 0x0001;
    uint16_t watchdog_time_ms = Timing::kMinimumWatchdogTimeMs;
    bool feature_enabled = false;
    /// FSoE status-frame variant the slave is configured with:
    /// LW1 = 31-byte frame (14 safe-data bytes), LW2 = 35-byte frame
    /// (16 safe-data bytes — safe torque data appended).  Must match the
    /// TxPDO 0x1B00 size used in the PDO assignment.
    PDO::FSoEFrameVariant frame_variant = PDO::FSoEFrameVariant::LW1;
};

struct ServoEmulatorConfig {
    uint16_t slave_address = 0x0001;
    uint16_t connection_id = 0x1234;
    uint16_t safety_address = 0x0001;
    uint16_t watchdog_time_ms = Timing::kMinimumWatchdogTimeMs;
    bool position_monitoring_enabled = true;
    bool velocity_monitoring_enabled = true;
    bool require_restart_acknowledge_after_error = true;
    bool temperature_ok = true;
    bool safe_input_1_high = false;
    bool safe_input_2_high = false;
    bool analog_input_diagnostic_active = true;
    bool analog_input_value_valid = true;
    int16_t analog_input_value = 0;
    bool accept_any_connection_id = false;

    // CRC model for state-transition frames.  When true, a frame that
    // fails the inherited CRC chain is retried with a fresh chain
    // (start_crc=0, seq=initialSeqNo) — matching masters like the
    // ESC211 that reset the CRC chain at each state transition, and
    // letting the slave re-synchronize after a missed transition.
    bool reset_crc_on_state_transition = false;

    /// FSoE status-frame variant this slave emulates:
    /// LW1 = 31-byte frame (14 safe-data bytes), LW2 = 35-byte frame
    /// (16 safe-data bytes — safe torque data appended).
    PDO::FSoEFrameVariant frame_variant = PDO::FSoEFrameVariant::LW1;

    // Override the slave→master safe-data size (bytes).  0 (default)
    // derives the size from frame_variant (14 for LW1, 16 for LW2).
    // Set this when the FNI configures a different response frame size —
    // e.g. the ESC211's channels 3-4 carry 16 safe-data bytes (35-byte
    // wire frame) instead of 14 (31-byte).
    uint8_t safe_input_size_override = 0;

    // Native CRC-chain resynchronization (opt-in).  When a received
    // frame fails CRC verification, the slave solves the exact
    // (startCrc, seqNo) seed from the frame's own CRCs (a GF(2) linear
    // solve — FSoESlaveConfig::crcResyncEnabled) and re-anchors its
    // sequence tracking, so a slave that joined mid-stream or missed a
    // state-transition frame can synchronize and continue.  Intended
    // for emulator/test use — see the safety note in FSoESlaveConfig.
    bool crc_resync = false;

    // When true, the master→slave STO bit is interpreted as
    // one-active (bit=1 → STO active) instead of the standard
    // zero-active (bit=0 → STO active) semantics.  Only takes effect
    // when the slave is in the Data/ProcessData state.
    bool invert_sto = false;

    ::FSoE::FSoESlaveConfig toSlaveConfig() const;
};

struct Codec {
    static constexpr std::size_t kMainToSlaveSize = 4;
    /// Maximum slave→master safe-data size (LW2).  LW1 uses the first
    /// kSlaveToMainSizeLW1 bytes — encode/decode take spans so the same
    /// codec serves both frame variants.
    static constexpr std::size_t kSlaveToMainSize = 16;
    /// Slave→master safe-data size for the LW1 frame variant (no safe
    /// torque data).
    static constexpr std::size_t kSlaveToMainSizeLW1 = 14;

    static void encodeMainToSlave(const Command& command,
                                  std::span<uint8_t> bytes);
    static std::optional<Command> decodeMainToSlave(
        std::span<const uint8_t> bytes);

    /// Encode the slave→master safe data.  The first 14 bytes are the
    /// LW1 payload; when @p bytes has room for 16 (LW2) the safe-torque
    /// word is appended.
    static void encodeSlaveToMain(const Status& status,
                                  std::span<uint8_t> bytes);
    /// Decode slave→master safe data.  Requires at least 14 bytes (LW1);
    /// with 16 bytes (LW2) the safe-torque word is decoded as well.
    static std::optional<Status> decodeSlaveToMain(
        std::span<const uint8_t> bytes);

    /// Command→status bit-mirror table for the SOMANET SafeMotion wire
    /// format.  The FSoE master (e.g. ESC211) expects each master→slave
    /// command bit echoed VERBATIM into the corresponding slave→master
    /// status bit — at a DIFFERENT position, since the two directions
    /// use different field layouts:
    ///
    ///   master→slave cmd (safe data)      slave→master status
    ///   safety_flags bit 0  STO            safety_state bit 0
    ///   safety_flags bit 1  SS1            safety_state bit 8
    ///   safety_flags bit 2  SS2            safety_state bit 9
    ///   safety_flags bit 3  SOS            safety_state bit 3
    ///   safety_flags bit 8-11 SLS 1-4      safety_state bits 12-15
    ///   safety_flags bit 13 SBC            diagnostic_flags bit 1
    ///   safe-data bits 28-29 safe outputs  diagnostic_flags bits 12-13
    ///
    /// Positions are extracted from the PDO mask definitions in
    /// SynapticonPDO.hpp (via ::FSoE::bitIndexOf; diagStatusBit shifts
    /// the diagnostic-field masks into the status safe-data space) —
    /// no raw bit numbers are hardcoded here.
    ///
    /// Consumed automatically by FSoESlaveEmulator::refreshPublishedStatus().
    static constexpr ::FSoE::BitMirror kStatusBitMirrors[] = {
        {::FSoE::bitIndexOf(PDO::SOMANET_RxPDO_1700::kSTO),
         ::FSoE::bitIndexOf(PDO::SOMANET_TxPDO_1B00::kSTOState)},
        {::FSoE::bitIndexOf(PDO::SOMANET_RxPDO_1700::kSS1),
         ::FSoE::bitIndexOf(PDO::SOMANET_TxPDO_1B00::kSS1State)},
        {::FSoE::bitIndexOf(PDO::SOMANET_RxPDO_1700::kSS2),
         ::FSoE::bitIndexOf(PDO::SOMANET_TxPDO_1B00::kSS2State)},
        {::FSoE::bitIndexOf(PDO::SOMANET_RxPDO_1700::kSOS),
         ::FSoE::bitIndexOf(PDO::SOMANET_TxPDO_1B00::kSOSState)},
        {::FSoE::bitIndexOf(PDO::SOMANET_RxPDO_1700::kSLS_Instance1),
         ::FSoE::bitIndexOf(PDO::SOMANET_TxPDO_1B00::kSLSInstance1)},
        {::FSoE::bitIndexOf(PDO::SOMANET_RxPDO_1700::kSLS_Instance2),
         ::FSoE::bitIndexOf(PDO::SOMANET_TxPDO_1B00::kSLSInstance2)},
        {::FSoE::bitIndexOf(PDO::SOMANET_RxPDO_1700::kSLS_Instance3),
         ::FSoE::bitIndexOf(PDO::SOMANET_TxPDO_1B00::kSLSInstance3)},
        {::FSoE::bitIndexOf(PDO::SOMANET_RxPDO_1700::kSLS_Instance4),
         ::FSoE::bitIndexOf(PDO::SOMANET_TxPDO_1B00::kSLSInstance4)},
        {::FSoE::bitIndexOf(PDO::SOMANET_RxPDO_1700::kSBCCommand),
         diagStatusBit(PDO::SOMANET_TxPDO_1B00::kSBCState)},
        // Safe outputs: command safe-data bits 28-29 (FNI 0x26F0:1-2);
        // the PDO header only defines the byte-local field bits (0-1),
        // which do not match the wire layout, so the FNI positions are
        // used directly.  Monitors are diagnostic_flags bits 12-13.
        {28, diagStatusBit(PDO::SOMANET_TxPDO_1B00::kSafeOutputMonitor1)},
        {29, diagStatusBit(PDO::SOMANET_TxPDO_1B00::kSafeOutputMonitor2)},
    };
};

class SafeMotionServoEmulator;

class MainInstance {
public:
    explicit MainInstance(const MainConfig& config = {});

    bool initialize();
    void setFeatureEnabled(bool enabled);
    bool featureEnabled() const { return feature_enabled_; }

    void setCommand(const Command& command);
    const Command& command() const { return command_; }
    void requestMotionEnabled();
    void requestSafeStop();
    void pulseErrorAcknowledge();
    void pulseRestartAcknowledge();
    void pulseResetPosition();

    bool exchangeWith(SafeMotionServoEmulator& slave, uint64_t current_time_ms);

    /// Exchange FSoE frames via EtherCAT PDO buffers (real drive communication).
    ///
    /// Encodes the current command, runs the FSoE state machine, builds the
    /// master-to-slave frame into @p rx_pdo_out (the RxPDO 0x1700 buffer),
    /// and processes the slave-to-master frame from @p tx_pdo_in (the TxPDO
    /// 0x1B00 buffer).  On success, decodes the safety status and clears
    /// pulse bits.
    ///
    /// @param rx_pdo_out      Output buffer for the master→slave FSoE frame (RxPDO)
    /// @param rx_pdo_max      Capacity of rx_pdo_out (must be ≥ 11 for Data state)
    /// @param tx_pdo_in       Input buffer with the slave→master FSoE frame (TxPDO)
    /// @param tx_pdo_len      Number of valid bytes in tx_pdo_in
    /// @param current_time_ms Monotonic time in milliseconds
    /// @return true if the frame was processed successfully
    bool exchangeViaPDO(uint8_t* rx_pdo_out, size_t rx_pdo_max,
                        const uint8_t* tx_pdo_in, size_t tx_pdo_len,
                        uint64_t current_time_ms);

    const Status& status() const { return status_; }
    bool hasStatus() const { return has_status_; }
    bool motionAllowed() const;

    ::FSoE::FSoEMasterConnection& rawConnection() { return connection_; }
    const ::FSoE::FSoEMasterConnection& rawConnection() const { return connection_; }

private:
    void clearPulseBits();

    MainConfig config_;
    ::FSoE::MasterConnectionConfig connection_config_{};
    ::FSoE::FSoEMasterConnection connection_;
    ::FSoE::TypedMainProcessDataView<Command, Status, Codec> typed_view_;
    Command command_ = Command::safeStop();
    Status status_{};
    bool initialized_ = false;
    bool feature_enabled_ = false;
    bool has_status_ = false;
};

class SafeMotionServoEmulator
    : public ::FSoE::FSoESlaveEmulator<Codec, Command, Status, ServoEmulatorConfig> {
    using Base = ::FSoE::FSoESlaveEmulator<Codec, Command, Status, ServoEmulatorConfig>;

public:
    explicit SafeMotionServoEmulator(const ServoEmulatorConfig& config = {});

    void step(double requested_velocity_counts_per_second, double dt_seconds);
    void injectError(bool require_restart_acknowledge = true);
    void clearError();

    /// Reset the emulator to the safe state (STO active, SBC active / brakes
    /// engaged).  Resets last_command_ to safeStop(), clears error state,
    /// and republishes the status.  Call this after rawSlave().reset() to
    /// ensure the slave reports STO+SBC active immediately, before the master
    /// sends any new command.
    void resetToSafeState();

    bool motionAllowed() const { return published_status_.motionAllowed(); }

    /// Toggle one-active (bit=1) vs. zero-active (bit=0) interpretation
    /// of the master→slave STO bit.  Only affects the ProcessData state.
    void setInvertSto(bool invert);
    bool invertSto() const;

protected:
    void onInitialize() override;
    void onCommandConsumed(const Command& cmd) override;
    void buildStatus(Status& out) override;
    void onReset() override;

private:
    bool error_active_ = false;
    bool restart_required_ = false;
    bool previous_error_acknowledge_ = false;
    bool previous_restart_acknowledge_ = false;
    bool previous_reset_position_ = false;
    double position_counts_ = 0.0;
    double velocity_counts_per_second_ = 0.0;
};

template<typename RxPDO>
class MainLoopFeature final : public EtherCAT::DS402Master::ICyclicTask {
public:
    MainLoopFeature(uint16_t slave_index,
                    MainInstance& main_instance,
                    SafeMotionServoEmulator& servo)
        : slave_index_(slave_index)
        , main_instance_(main_instance)
        , servo_(servo)
    {
    }

    bool update(EtherCAT::DS402Master& master, double dt_seconds) override
    {
        if (!main_instance_.featureEnabled()) {
            return true;
        }

        elapsed_time_ms_ += static_cast<uint64_t>(dt_seconds * 1000.0);

        auto* drive = master.driveBySlaveIndex(slave_index_);
        if (drive == nullptr) {
            return false;
        }

        auto* rx = drive->rxPDO<RxPDO>();
        if (rx == nullptr) {
            return false;
        }

        double requested_velocity = 0.0;
        if constexpr (requires(RxPDO& pdo) { pdo.target_velocity; }) {
            requested_velocity = main_instance_.motionAllowed()
                ? static_cast<double>(rx->target_velocity)
                : 0.0;
        }

        servo_.step(requested_velocity, dt_seconds);
        if (!main_instance_.exchangeWith(servo_, elapsed_time_ms_)) {
            return false;
        }

        if (!main_instance_.motionAllowed()) {
            if constexpr (requires(RxPDO& pdo) { pdo.target_velocity; }) {
                rx->target_velocity = 0;
            }
            if constexpr (requires(RxPDO& pdo) { pdo.target_torque; }) {
                rx->target_torque = 0;
            }
        }

        return true;
    }

private:
    uint16_t slave_index_;
    MainInstance& main_instance_;
    SafeMotionServoEmulator& servo_;
    uint64_t elapsed_time_ms_ = 0;
};

} // namespace EtherCAT::Drives::Synapticon::SafeMotion
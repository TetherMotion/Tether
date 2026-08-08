#pragma once

#include <array>
#include <cstdint>
#include <optional>

#include "tether/fsoe/TypedProcessData.hpp"
#include "tether/profiles/cia402/DS402Master.hpp"

namespace EtherCAT::Drives::Synapticon::SafeMotion {

struct Timing {
    static constexpr uint16_t kMinimumWatchdogTimeMs = 15;
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

    bool motionAllowed() const;
};

struct MainConfig {
    uint16_t slave_address = 0x0001;
    uint16_t safety_address = 0x0001;
    uint16_t connection_id = 0x1234;
    uint16_t master_address = 0x0001;
    uint16_t watchdog_time_ms = Timing::kMinimumWatchdogTimeMs;
    bool feature_enabled = false;
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
};

struct Codec {
    static constexpr std::size_t kMainToSlaveSize = 4;
    static constexpr std::size_t kSlaveToMainSize = 14;

    static void encodeMainToSlave(const Command& command,
                                  std::array<uint8_t, kMainToSlaveSize>& bytes);
    static std::optional<Command> decodeMainToSlave(
        const std::array<uint8_t, kMainToSlaveSize>& bytes);

    static void encodeSlaveToMain(const Status& status,
                                  std::array<uint8_t, kSlaveToMainSize>& bytes);
    static std::optional<Status> decodeSlaveToMain(
        const std::array<uint8_t, kSlaveToMainSize>& bytes);
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

class SafeMotionServoEmulator {
public:
    explicit SafeMotionServoEmulator(const ServoEmulatorConfig& config = {});

    bool initialize();
    void step(double requested_velocity_counts_per_second, double dt_seconds);
    void injectError(bool require_restart_acknowledge = true);
    void clearError();
    void synchronizeCommandAndStatus();

    const Status& status() const { return published_status_; }
    const Command& lastCommand() const { return last_command_; }
    bool motionAllowed() const { return published_status_.motionAllowed(); }

    ::FSoE::FSoESlave& rawSlave() { return slave_; }
    const ::FSoE::FSoESlave& rawSlave() const { return slave_; }

private:
    void consumeLatestCommand();
    void refreshPublishedStatus();

    ServoEmulatorConfig config_;
    ::FSoE::FSoESlaveConfig slave_config_{};
    ::FSoE::FSoESlave slave_;
    ::FSoE::TypedSlaveProcessDataView<Command, Status, Codec> typed_view_;
    Command last_command_ = Command::safeStop();
    Status published_status_{};
    bool initialized_ = false;
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
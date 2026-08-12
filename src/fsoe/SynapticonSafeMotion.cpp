#include "tether/fsoe/Synapticon/SafeMotionFSoE.hpp"

#include <cmath>

namespace EtherCAT::Drives::Synapticon::SafeMotion {

namespace {

constexpr uint8_t kStoBit = 0;
constexpr uint8_t kSs1Bit = 1;
constexpr uint8_t kSs2Bit = 2;
constexpr uint8_t kSosBit = 3;
constexpr uint8_t kErrorAckBit = 7;
constexpr uint8_t kSls1Bit = 8;
constexpr uint8_t kSls2Bit = 9;
constexpr uint8_t kSls3Bit = 10;
constexpr uint8_t kSls4Bit = 11;
constexpr uint8_t kRestartAckBit = 12;
constexpr uint8_t kSbcBit = 13;
constexpr uint8_t kResetPositionBit = 14;
constexpr uint8_t kSafeOutput1Bit = 12;

constexpr uint8_t kStatusStoBit = 0;
constexpr uint8_t kStatusSosBit = 3;
constexpr uint8_t kStatusErrorBit = 7;
constexpr uint8_t kStatusSs1Bit = 8;
constexpr uint8_t kStatusSs2Bit = 9;
constexpr uint8_t kStatusSls1Bit = 12;
constexpr uint8_t kStatusSls2Bit = 13;
constexpr uint8_t kStatusSls3Bit = 14;
constexpr uint8_t kStatusSls4Bit = 15;
constexpr uint8_t kStatusRestartRequiredBit = 0;
constexpr uint8_t kStatusSbcBit = 1;
constexpr uint8_t kStatusTemperatureOkBit = 2;
constexpr uint8_t kStatusPositionValidBit = 3;
constexpr uint8_t kStatusVelocityValidBit = 4;
constexpr uint8_t kStatusInput1Bit = 8;
constexpr uint8_t kStatusInput2Bit = 9;
constexpr uint8_t kStatusOutput1Bit = 12;
constexpr uint8_t kStatusAnalogDiagnosticBit = 14;
constexpr uint8_t kStatusAnalogValidBit = 15;

constexpr uint16_t bitMask(uint8_t bit)
{
    return static_cast<uint16_t>(1u << bit);
}

uint16_t setOneActive(uint16_t word, uint8_t bit, bool active)
{
    return active ? static_cast<uint16_t>(word | bitMask(bit))
                  : static_cast<uint16_t>(word & ~bitMask(bit));
}

uint16_t setZeroActive(uint16_t word, uint8_t bit, bool active)
{
    return setOneActive(word, bit, !active);
}

bool getOneActive(uint16_t word, uint8_t bit)
{
    return (word & bitMask(bit)) != 0;
}

bool getZeroActive(uint16_t word, uint8_t bit)
{
    return !getOneActive(word, bit);
}

uint16_t readWord(const uint8_t* data)
{
    return static_cast<uint16_t>(data[0]) |
           (static_cast<uint16_t>(data[1]) << 8);
}

void writeWord(uint8_t* data, uint16_t value)
{
    data[0] = static_cast<uint8_t>(value & 0xFFu);
    data[1] = static_cast<uint8_t>((value >> 8) & 0xFFu);
}

} // namespace

Command Command::safeStop()
{
    return {};
}

Command Command::motionEnabled()
{
    Command command;
    command.sto = false;
    command.ss1 = false;
    command.ss2 = false;
    command.sos = false;
    command.brake_engage = false;
    return command;
}

bool Status::motionAllowed() const
{
    return !sto_active && !ss1_active && !ss2_active && !sos_active &&
           !error_active && !restart_acknowledge_required;
}

void Codec::encodeMainToSlave(const Command& command,
                              std::array<uint8_t, kMainToSlaveSize>& bytes)
{
    uint16_t control_word0 = 0;
    uint16_t control_word1 = 0;

    control_word0 = setZeroActive(control_word0, kStoBit, command.sto);
    control_word0 = setZeroActive(control_word0, kSs1Bit, command.ss1);
    control_word0 = setZeroActive(control_word0, kSs2Bit, command.ss2);
    control_word0 = setZeroActive(control_word0, kSosBit, command.sos);
    control_word0 = setOneActive(control_word0, kErrorAckBit, command.error_acknowledge);
    control_word0 = setZeroActive(control_word0, kSls1Bit, command.sls[0]);
    control_word0 = setZeroActive(control_word0, kSls2Bit, command.sls[1]);
    control_word0 = setZeroActive(control_word0, kSls3Bit, command.sls[2]);
    control_word0 = setZeroActive(control_word0, kSls4Bit, command.sls[3]);
    control_word0 = setOneActive(control_word0, kRestartAckBit, command.restart_acknowledge);
    control_word0 = setZeroActive(control_word0, kSbcBit, command.brake_engage);
    control_word0 = setOneActive(control_word0, kResetPositionBit, command.reset_position);
    control_word1 = setOneActive(control_word1, kSafeOutput1Bit, command.safe_output_1_high);

    writeWord(bytes.data(), control_word0);
    writeWord(bytes.data() + 2, control_word1);
}

std::optional<Command> Codec::decodeMainToSlave(
    const std::array<uint8_t, kMainToSlaveSize>& bytes)
{
    Command command;
    const uint16_t control_word0 = readWord(bytes.data());
    const uint16_t control_word1 = readWord(bytes.data() + 2);

    command.sto = getZeroActive(control_word0, kStoBit);
    command.ss1 = getZeroActive(control_word0, kSs1Bit);
    command.ss2 = getZeroActive(control_word0, kSs2Bit);
    command.sos = getZeroActive(control_word0, kSosBit);
    command.error_acknowledge = getOneActive(control_word0, kErrorAckBit);
    command.sls[0] = getZeroActive(control_word0, kSls1Bit);
    command.sls[1] = getZeroActive(control_word0, kSls2Bit);
    command.sls[2] = getZeroActive(control_word0, kSls3Bit);
    command.sls[3] = getZeroActive(control_word0, kSls4Bit);
    command.restart_acknowledge = getOneActive(control_word0, kRestartAckBit);
    command.brake_engage = getZeroActive(control_word0, kSbcBit);
    command.reset_position = getOneActive(control_word0, kResetPositionBit);
    command.safe_output_1_high = getOneActive(control_word1, kSafeOutput1Bit);

    return command;
}

void Codec::encodeSlaveToMain(const Status& status,
                              std::array<uint8_t, kSlaveToMainSize>& bytes)
{
    uint16_t status_word0 = 0;
    uint16_t status_word1 = 0;

    status_word0 = setOneActive(status_word0, kStatusStoBit, status.sto_active);
    status_word0 = setOneActive(status_word0, kStatusSosBit, status.sos_active);
    status_word0 = setOneActive(status_word0, kStatusErrorBit, status.error_active);
    status_word0 = setOneActive(status_word0, kStatusSs1Bit, status.ss1_active);
    status_word0 = setOneActive(status_word0, kStatusSs2Bit, status.ss2_active);
    status_word0 = setOneActive(status_word0, kStatusSls1Bit, status.sls_active[0]);
    status_word0 = setOneActive(status_word0, kStatusSls2Bit, status.sls_active[1]);
    status_word0 = setOneActive(status_word0, kStatusSls3Bit, status.sls_active[2]);
    status_word0 = setOneActive(status_word0, kStatusSls4Bit, status.sls_active[3]);

    status_word1 = setOneActive(status_word1, kStatusRestartRequiredBit,
                                status.restart_acknowledge_required);
    status_word1 = setOneActive(status_word1, kStatusSbcBit, status.brake_engaged);
    status_word1 = setOneActive(status_word1, kStatusTemperatureOkBit, status.temperature_ok);
    status_word1 = setOneActive(status_word1, kStatusPositionValidBit, status.safe_position_valid);
    status_word1 = setOneActive(status_word1, kStatusVelocityValidBit, status.safe_velocity_valid);
    status_word1 = setOneActive(status_word1, kStatusInput1Bit, status.safe_input_1_high);
    status_word1 = setOneActive(status_word1, kStatusInput2Bit, status.safe_input_2_high);
    status_word1 = setOneActive(status_word1, kStatusOutput1Bit, status.safe_output_1_high);
    status_word1 = setOneActive(status_word1, kStatusAnalogDiagnosticBit,
                                status.analog_input_diagnostic_active);
    status_word1 = setOneActive(status_word1, kStatusAnalogValidBit,
                                status.analog_input_value_valid);

    writeWord(bytes.data(), status_word0);
    writeWord(bytes.data() + 2, status_word1);
    writeWord(bytes.data() + 4, static_cast<uint16_t>(status.safe_position & 0xFFFF));
    writeWord(bytes.data() + 6, static_cast<uint16_t>((status.safe_position >> 16) & 0xFFFF));
    writeWord(bytes.data() + 8, static_cast<uint16_t>(status.safe_velocity & 0xFFFF));
    writeWord(bytes.data() + 10, static_cast<uint16_t>((status.safe_velocity >> 16) & 0xFFFF));
    writeWord(bytes.data() + 12, static_cast<uint16_t>(status.safe_analog_input));
}

std::optional<Status> Codec::decodeSlaveToMain(
    const std::array<uint8_t, kSlaveToMainSize>& bytes)
{
    Status status;
    const uint16_t status_word0 = readWord(bytes.data());
    const uint16_t status_word1 = readWord(bytes.data() + 2);
    const uint32_t position = static_cast<uint32_t>(readWord(bytes.data() + 4)) |
                              (static_cast<uint32_t>(readWord(bytes.data() + 6)) << 16);
    const uint32_t velocity = static_cast<uint32_t>(readWord(bytes.data() + 8)) |
                              (static_cast<uint32_t>(readWord(bytes.data() + 10)) << 16);

    status.sto_active = getOneActive(status_word0, kStatusStoBit);
    status.sos_active = getOneActive(status_word0, kStatusSosBit);
    status.error_active = getOneActive(status_word0, kStatusErrorBit);
    status.ss1_active = getOneActive(status_word0, kStatusSs1Bit);
    status.ss2_active = getOneActive(status_word0, kStatusSs2Bit);
    status.sls_active[0] = getOneActive(status_word0, kStatusSls1Bit);
    status.sls_active[1] = getOneActive(status_word0, kStatusSls2Bit);
    status.sls_active[2] = getOneActive(status_word0, kStatusSls3Bit);
    status.sls_active[3] = getOneActive(status_word0, kStatusSls4Bit);
    status.restart_acknowledge_required = getOneActive(status_word1, kStatusRestartRequiredBit);
    status.brake_engaged = getOneActive(status_word1, kStatusSbcBit);
    status.temperature_ok = getOneActive(status_word1, kStatusTemperatureOkBit);
    status.safe_position_valid = getOneActive(status_word1, kStatusPositionValidBit);
    status.safe_velocity_valid = getOneActive(status_word1, kStatusVelocityValidBit);
    status.safe_input_1_high = getOneActive(status_word1, kStatusInput1Bit);
    status.safe_input_2_high = getOneActive(status_word1, kStatusInput2Bit);
    status.safe_output_1_high = getOneActive(status_word1, kStatusOutput1Bit);
    status.analog_input_diagnostic_active = getOneActive(status_word1, kStatusAnalogDiagnosticBit);
    status.analog_input_value_valid = getOneActive(status_word1, kStatusAnalogValidBit);
    status.safe_position = static_cast<int32_t>(position);
    status.safe_velocity = static_cast<int32_t>(velocity);
    status.safe_analog_input = static_cast<int16_t>(readWord(bytes.data() + 12));
    return status;
}

MainInstance::MainInstance(const MainConfig& config)
    : config_(config)
    , connection_config_([&config]() {
        ::FSoE::MasterConnectionConfig cfg{};
        cfg.slave_addr = config.slave_address;
        cfg.slave_safety_addr = config.safety_address;
        cfg.connection_id = config.connection_id;
        cfg.master_addr = config.master_address;
        cfg.watchdog_timeout_ms = config.watchdog_time_ms;
        cfg.conn_timeout_ms = 1000;
        cfg.safety_level = ::FSoE::SIL::SIL2;
        cfg.input_size = static_cast<uint8_t>(Codec::kSlaveToMainSize);
        cfg.output_size = static_cast<uint8_t>(Codec::kMainToSlaveSize);
        // Synapticon SOMANET drives run FSoE at a lower internal rate
        // than the EtherCAT bus cycle, resulting in ~8 cycles of delay
        // between the master's TX command and the slave's TX response.
        // Use a generous budget (25) to tolerate jitter and multiple
        // FSoE task periods before the stale-exhaustion fail-safe fires.
        cfg.slave_response_delay_cycles = 25;
        return cfg;
    }())
    , connection_(connection_config_)
    , typed_view_(connection_)
    , feature_enabled_(config.feature_enabled)
{
}

bool MainInstance::initialize()
{
    initialized_ = connection_.initialize();
    if (initialized_) {
        connection_.startConnection();
        if (!feature_enabled_) {
            has_status_ = false;
        }
    }
    return initialized_;
}

void MainInstance::setFeatureEnabled(bool enabled)
{
    feature_enabled_ = enabled;
    if (enabled && !initialized_) {
        (void)initialize();
    }
}

void MainInstance::setCommand(const Command& command)
{
    command_ = command;
}

void MainInstance::requestMotionEnabled()
{
    command_ = Command::motionEnabled();
}

void MainInstance::requestSafeStop()
{
    command_ = Command::safeStop();
}

void MainInstance::pulseErrorAcknowledge()
{
    command_.error_acknowledge = true;
}

void MainInstance::pulseRestartAcknowledge()
{
    command_.restart_acknowledge = true;
}

void MainInstance::pulseResetPosition()
{
    command_.reset_position = true;
}

bool MainInstance::exchangeWith(SafeMotionServoEmulator& slave, uint64_t current_time_ms)
{
    if (!feature_enabled_) {
        return true;
    }
    if (!initialized_ && !initialize()) {
        return false;
    }

    // Delegate to the generic typed FSoE exchange.  The SafeMotion-specific
    // pieces are the mid-exchange emulator synchronization (so the servo
    // consumes the latest command and refreshes its published status before
    // the slave→master frame is built) and the post-success pulse-bit clear.
    const bool ok = typed_view_.exchangeWith(
        slave.rawSlave(), command_, current_time_ms,
        [&slave] { slave.synchronizeCommandAndStatus(); },
        [this] { clearPulseBits(); });

    if (ok) {
        if (const auto decoded = typed_view_.read()) {
            status_ = *decoded;
            has_status_ = true;
        }
    }
    return ok;
}

bool MainInstance::exchangeViaPDO(uint8_t* rx_pdo_out, size_t rx_pdo_max,
                                   const uint8_t* tx_pdo_in, size_t tx_pdo_len,
                                   uint64_t current_time_ms)
{
    if (!feature_enabled_) {
        return true;
    }
    if (!initialized_ && !initialize()) {
        return false;
    }

    // Delegate to the generic typed FSoE-over-PDO exchange.  The only
    // SafeMotion-specific piece is the post-success pulse-bit clear.
    const bool ok = typed_view_.exchangeViaPDO(
        rx_pdo_out, rx_pdo_max, tx_pdo_in, tx_pdo_len,
        command_, current_time_ms,
        [this] { clearPulseBits(); });

    if (ok) {
        if (const auto decoded = typed_view_.read()) {
            status_ = *decoded;
            has_status_ = true;
        }
    }
    return ok;
}

bool MainInstance::motionAllowed() const
{
    return !feature_enabled_ || (has_status_ && status_.motionAllowed());
}

void MainInstance::clearPulseBits()
{
    command_.error_acknowledge = false;
    command_.restart_acknowledge = false;
    command_.reset_position = false;
}

SafeMotionServoEmulator::SafeMotionServoEmulator(const ServoEmulatorConfig& config)
    : config_(config)
    , slave_config_([&config]() {
        ::FSoE::FSoESlaveConfig slave_config{};
        slave_config.slaveAddress = config.slave_address;
        slave_config.connectionId = config.connection_id;
        slave_config.safetyAddress = config.safety_address;
        slave_config.safetyLevel = ::FSoE::SIL::SIL2;
        slave_config.watchdogTimeoutMs = config.watchdog_time_ms;
        slave_config.connectionTimeoutMs = 1000;
        slave_config.sessionTimeoutMs = 5000;
        slave_config.safeInputSize = static_cast<uint8_t>(Codec::kSlaveToMainSize);
        slave_config.safeOutputSize = static_cast<uint8_t>(Codec::kMainToSlaveSize);
        slave_config.autoRecoveryEnabled = true;
        slave_config.recoveryDelayMs = 1000;
        slave_config.strictCrcCheck = true;
        slave_config.strictSequenceCheck = true;
        slave_config.treatCrcErrorAsCritical = true;
        slave_config.treatSequenceErrorAsCritical = true;
        slave_config.treatTimeoutAsCritical = true;
        slave_config.treatConnIdErrorAsCritical = true;
        slave_config.enableDiagnostics = true;
        slave_config.maxErrorLogEntries = 100;
        return slave_config;
    }())
    , slave_(slave_config_)
    , typed_view_(slave_)
{
}

bool SafeMotionServoEmulator::initialize()
{
    initialized_ = slave_.initialize();
    if (!initialized_) {
        return false;
    }

    published_status_ = {};
    published_status_.temperature_ok = config_.temperature_ok;
    published_status_.safe_input_1_high = config_.safe_input_1_high;
    published_status_.safe_input_2_high = config_.safe_input_2_high;
    published_status_.analog_input_diagnostic_active = config_.analog_input_diagnostic_active;
    published_status_.analog_input_value_valid = config_.analog_input_value_valid;
    published_status_.safe_analog_input = config_.analog_input_value;
    refreshPublishedStatus();
    return true;
}

void SafeMotionServoEmulator::step(double requested_velocity_counts_per_second, double dt_seconds)
{
    if (!initialized_) {
        (void)initialize();
    }

    consumeLatestCommand();

    const bool can_move = published_status_.motionAllowed();
    velocity_counts_per_second_ = can_move ? requested_velocity_counts_per_second : 0.0;
    position_counts_ += velocity_counts_per_second_ * dt_seconds;

    published_status_.safe_velocity = static_cast<int32_t>(std::lround(velocity_counts_per_second_));
    published_status_.safe_position = static_cast<int32_t>(std::lround(position_counts_));
    refreshPublishedStatus();
}

void SafeMotionServoEmulator::injectError(bool require_restart_acknowledge)
{
    error_active_ = true;
    restart_required_ = require_restart_acknowledge;
    refreshPublishedStatus();
}

void SafeMotionServoEmulator::clearError()
{
    error_active_ = false;
    restart_required_ = false;
    refreshPublishedStatus();
}

void SafeMotionServoEmulator::synchronizeCommandAndStatus()
{
    consumeLatestCommand();
    refreshPublishedStatus();
}

void SafeMotionServoEmulator::consumeLatestCommand()
{
    const auto decoded = typed_view_.consume();
    if (!decoded) {
        return;
    }

    last_command_ = *decoded;

    const bool error_ack_edge = last_command_.error_acknowledge && !previous_error_acknowledge_;
    const bool restart_ack_edge = last_command_.restart_acknowledge && !previous_restart_acknowledge_;
    const bool reset_position_edge = last_command_.reset_position && !previous_reset_position_;

    previous_error_acknowledge_ = last_command_.error_acknowledge;
    previous_restart_acknowledge_ = last_command_.restart_acknowledge;
    previous_reset_position_ = last_command_.reset_position;

    if (error_ack_edge) {
        error_active_ = false;
        restart_required_ = config_.require_restart_acknowledge_after_error && restart_required_;
    }
    if (restart_ack_edge) {
        restart_required_ = false;
    }
    if (reset_position_edge) {
        position_counts_ = 0.0;
    }
}

void SafeMotionServoEmulator::refreshPublishedStatus()
{
    published_status_.sto_active = error_active_ || last_command_.sto;
    published_status_.ss1_active = error_active_ || last_command_.ss1;
    published_status_.ss2_active = config_.position_monitoring_enabled ? last_command_.ss2 : false;
    published_status_.sos_active = config_.position_monitoring_enabled ? last_command_.sos : false;
    published_status_.sls_active = {
        config_.velocity_monitoring_enabled ? last_command_.sls[0] : false,
        config_.velocity_monitoring_enabled ? last_command_.sls[1] : false,
        config_.velocity_monitoring_enabled ? last_command_.sls[2] : false,
        config_.velocity_monitoring_enabled ? last_command_.sls[3] : false,
    };
    published_status_.error_active = error_active_;
    published_status_.restart_acknowledge_required = restart_required_;
    published_status_.brake_engaged = error_active_ || last_command_.brake_engage ||
                                      published_status_.sto_active || published_status_.ss1_active;
    published_status_.temperature_ok = config_.temperature_ok;
    published_status_.safe_position_valid = config_.position_monitoring_enabled;
    published_status_.safe_velocity_valid = config_.velocity_monitoring_enabled;
    published_status_.safe_input_1_high = config_.safe_input_1_high;
    published_status_.safe_input_2_high = config_.safe_input_2_high;
    published_status_.safe_output_1_high = last_command_.safe_output_1_high;
    published_status_.analog_input_diagnostic_active = config_.analog_input_diagnostic_active;
    published_status_.analog_input_value_valid = config_.analog_input_value_valid;
    published_status_.safe_analog_input = config_.analog_input_value;

    (void)typed_view_.publish(published_status_);
}

} // namespace EtherCAT::Drives::Synapticon::SafeMotion
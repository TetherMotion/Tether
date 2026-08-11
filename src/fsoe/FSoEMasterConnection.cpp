/**
 * @file FSoEMasterConnection.cpp
 * @brief FSoE Master Connection implementation — redesigned
 */

#include "fsoe/FSoEMasterConnection.hpp"
#include "fsoe/FSoESlave.hpp"
#include "fsoe/FSoECRC.hpp"
#include <cstring>
#include <algorithm>
#include <cstdio>
#include <cstdarg>

namespace FSoE {

// ============================================================================
// Protocol trace helper
// ============================================================================

namespace {
const char* stateName(uint8_t state) {
    switch (state) {
        case ConnectionState::Reset:      return "Reset";
        case ConnectionState::Session:    return "Session";
        case ConnectionState::Connection: return "Connection";
        case ConnectionState::Parameter:  return "Parameter";
        case ConnectionState::Data:       return "Data";
        case ConnectionState::FailSafe:   return "FailSafe";
        case ConnectionState::Error:      return "Error";
        default:                          return "Unknown";
    }
}

const char* commandName(uint8_t cmd) {
    switch (cmd) {
        case Command::ProcessData:    return "ProcessData(0x36)";
        case Command::Reset:          return "Reset(0x2A)";
        case Command::Session:        return "Session(0x4E)";
        case Command::Connection:     return "Connection(0x64)";
        case Command::Parameter:      return "Parameter(0x52)";
        case Command::FailSafeData:   return "FailSafeData(0x08)";
        default:                      return "Unknown";
    }
}
} // namespace

void FSoEMasterConnection::trace(const char* fmt, ...) const
{
    if (!trace_callback_) return;
    char buf[256];
    va_list args;
    va_start(args, fmt);
    vsnprintf(buf, sizeof(buf), fmt, args);
    va_end(args);
    trace_callback_(buf);
}

// ============================================================================
// Construction / Initialization
// ============================================================================

FSoEMasterConnection::FSoEMasterConnection(const MasterConnectionConfig& config)
    : config_(config)
{
}

FSoEMasterConnection::~FSoEMasterConnection() = default;

bool FSoEMasterConnection::initialize()
{
    std::lock_guard<std::recursive_mutex> lock(mutex_);

    if (config_.input_size > 16 || config_.output_size > 16) {
        return false;
    }

    std::copy(config_.fail_safe_values.begin(),
              config_.fail_safe_values.begin() + config_.output_size,
              safe_outputs_.begin());

    status_ = {};
    status_.state = ConnectionState::Reset;
    status_.state_entered_ms = 0;

    rx_sequence_ = 0x0F;  // Wrap so first expected RX sequence is 0
    tx_sequence_ = 0;
    last_tx_crc0_ = 0;
    last_rx_crc0_ = 0;
    tx_seq_no_ = 0;
    rx_seq_no_ = 0;
    current_param_index_ = 0;
    parameter_crc_ = 0;
    fail_safe_entered_ms_ = 0;
    pdo_tx_count_ = 0;
    last_rx_frame_.clear();

    resetStats();
    parameter_crc_ = computeParameterCRC();

    initialized_ = true;
    return true;
}

bool FSoEMasterConnection::isInitialized() const
{
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    return initialized_;
}

const MasterConnectionConfig& FSoEMasterConnection::getConfig() const
{
    return config_;
}

// ============================================================================
// Connection Control
// ============================================================================

bool FSoEMasterConnection::startConnection()
{
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    if (!initialized_) return false;

    status_.state = ConnectionState::Reset;
    status_.state_entered_ms = current_time_ms_;
    return true;
}

bool FSoEMasterConnection::resetConnection()
{
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    if (!initialized_) return false;

    transitionTo(ConnectionState::Reset);
    status_.error_code = ErrorCode::NoError;
    status_.data_valid = false;
    status_.fail_safe_active = false;
    rx_sequence_ = 0x0F;  // Wrap so first expected RX sequence is 0
    tx_sequence_ = 0;
    last_tx_crc0_ = 0;   // CRC inheritance starts at 0
    last_rx_crc0_ = 0;
    tx_seq_no_ = 0;
    rx_seq_no_ = 0;
    current_param_index_ = 0;
    pdo_tx_count_ = 0;
    last_rx_frame_.clear();
    stats_.reset_events++;

    return true;
}

bool FSoEMasterConnection::requestSessionReset()
{
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    if (!initialized_) return false;

    // Generate non-predictable session ID
    status_.session_id = static_cast<uint16_t>(rng_() & 0xFFFF);
    if (status_.session_id == 0) {
        status_.session_id = 1;
    }

    transitionTo(ConnectionState::Session);
    return true;
}

void FSoEMasterConnection::triggerFailSafe(uint16_t error_code)
{
    std::lock_guard<std::recursive_mutex> lock(mutex_);

    const bool was_fail_safe = status_.fail_safe_active;
    status_.fail_safe_active = true;
    status_.data_valid = false;

    if (error_code != ErrorCode::NoError) {
        status_.error_code = error_code;
    }

    std::copy(config_.fail_safe_values.begin(),
              config_.fail_safe_values.begin() + config_.output_size,
              safe_outputs_.begin());

    transitionTo(ConnectionState::FailSafe);
    fail_safe_entered_ms_ = current_time_ms_;

    if (!was_fail_safe && fail_safe_callback_) {
        fail_safe_callback_();
    }
}

bool FSoEMasterConnection::clearError()
{
    std::lock_guard<std::recursive_mutex> lock(mutex_);

    if (status_.state != ConnectionState::Error &&
        status_.state != ConnectionState::FailSafe) {
        return false;
    }

    status_.error_code = ErrorCode::NoError;
    status_.fail_safe_active = false;

    return resetConnection();
}

// ============================================================================
// State Machine — processRxFrame
// ============================================================================

bool FSoEMasterConnection::processRxFrame(const uint8_t* data, size_t len)
{
    std::lock_guard<std::recursive_mutex> lock(mutex_);

    if (!initialized_ || !data || len < CRC::MIN_FSOE_FRAME_SIZE) {
        return false;
    }

    stats_.frames_received++;

    // Duplicate frame detection: if the slave re-sends the exact same
    // frame bytes (e.g. it hasn't seen a new master frame yet and is
    // repeating its last response), skip re-processing.  This avoids
    // spurious state transitions or error handling from processing the
    // same handshake response twice (e.g. a Session response arriving
    // again after the master has already transitioned to Connection).
    //
    // Only applied during handshake states (Session, Connection,
    // Parameter).  In Data and FailSafe states, identical frames are
    // expected when the slave's inputs don't change — they must still
    // be processed to update the watchdog timestamp and refresh safe
    // input data.
    if ((status_.state == ConnectionState::Session ||
         status_.state == ConnectionState::Connection ||
         status_.state == ConnectionState::Parameter) &&
        !last_rx_frame_.empty() &&
        last_rx_frame_.size() == len &&
        std::memcmp(last_rx_frame_.data(), data, len) == 0) {
        stats_.duplicate_frames++;
        trace("RX duplicate %s frame (slave re-sent, skipping) (state=%s)",
              commandName(data[0]), stateName(status_.state));
        return false;
    }

    last_rx_frame_.assign(data, data + len);

    rx_frame_events_.emit([data, len] {
        return std::make_shared<const std::vector<uint8_t>>(data, data + len);
    });

    // Parse and validate frame (CRC verification happens inside parseFSoEFrame)
    // Buffer must accommodate MAX_PARSE_DATA_SIZE bytes because the slave's
    // buildFailSafeResponse sends safeInputSize + 2 bytes (inputs + error code),
    // which can be up to 18 bytes when safeInputSize = 16.
    uint8_t cmd = 0;
    uint8_t frame_data[CRC::MAX_PARSE_DATA_SIZE] = {0};
    size_t data_len = 0;
    uint16_t conn_id = 0;
    CRC::CrcErrorDetail crc_error_detail{};

    if (!CRC::parseFSoEFrame(data, len, cmd, frame_data, data_len, conn_id,
                             last_rx_crc0_, rx_seq_no_, &last_rx_crc0_,
                             &crc_error_detail)) {
        stats_.crc_errors++;
        FSoEErrorDetail detail;
        if (crc_error_detail.valid) {
            detail.crc_valid = true;
            detail.crc_segment_index = crc_error_detail.segment_index;
            detail.crc_expected = crc_error_detail.expected_crc;
            detail.crc_received = crc_error_detail.received_crc;
            detail.crc_frame_offset = crc_error_detail.frame_offset;
            snprintf(detail.message, sizeof(detail.message),
                     "Master received wrong CRC from slave: segment %d "
                     "expected 0x%04X got 0x%04X (frame offset %zu)",
                     detail.crc_segment_index,
                     detail.crc_expected, detail.crc_received,
                     detail.crc_frame_offset);
        } else {
            snprintf(detail.message, sizeof(detail.message),
                     "Master received malformed FSoE frame from slave "
                     "(frame too short or unparseable)");
        }
        handleError(ErrorCode::CRCError, detail);
        return false;
    }

    // Reject frames with unrecognized command bytes.
    //
    // On the first PDO cycle(s) before the slave has populated the TxPDO
    // buffer, the buffer is all zeros.  An all-zero frame passes CRC
    // trivially (CRC-16 of {0,0} with init 0x0000 is 0x0000) and would
    // otherwise be treated as a ConnectionIDError (conn_id=0 != configured).
    // Command 0x00 is not a valid FSoE command, so we silently skip these
    // frames and let the master retry on the next cycle.
    if (!isValidCommand(cmd)) {
        stats_.invalid_frames++;
        trace("RX cmd=0x%02X: not a valid FSoE command, skipping (state=%s)",
              cmd, stateName(status_.state));
        return false;
    }

    // Validate connection ID — but only after the connection is established.
    //
    // In Reset and Session states, the slave may not know the ConnID yet
    // (it has just been reset).  The real Synapticon drive echoes the Reset
    // command with conn_id=0x0000.  Skipping ConnID validation in these
    // early states matches the FSoE slave behavior (which also only validates
    // ConnID in Connection/Parameter/Data states).
    if (status_.state == ConnectionState::Connection ||
        status_.state == ConnectionState::Parameter ||
        status_.state == ConnectionState::Data ||
        status_.state == ConnectionState::FailSafe) {
        if (!validateConnectionID(conn_id)) {
            FSoEErrorDetail detail;
            detail.conn_id_valid = true;
            detail.expected_conn_id = config_.connection_id;
            detail.received_conn_id = conn_id;
            snprintf(detail.message, sizeof(detail.message),
                     "Master received wrong ConnectionID from slave: "
                     "expected 0x%04X got 0x%04X",
                     detail.expected_conn_id, detail.received_conn_id);
            handleError(ErrorCode::ConnectionIDError, detail);
            return false;
        }
    }

    // Update watchdog timestamp
    status_.last_valid_frame_ms = current_time_ms_;

    // Increment RX sequence number for CRC computation — but NOT for
    // Reset frames (3-byte frames with no CRC).  Reset frames don't
    // advance the CRC chain.
    if (len > CRC::MIN_FSOE_FRAME_SIZE) {
        rx_seq_no_++;
    }

    // Early detection of slave fail-safe response
    if (cmd == Command::FailSafeData &&
        status_.state != ConnectionState::FailSafe &&
        status_.state != ConnectionState::Error) {
        // Slave's fail-safe response contains input_size bytes of fail-safe
        // inputs followed by a 2-byte error code. Extract the error code
        // only if the full payload is present; otherwise use ApplicationError.
        if (data_len >= config_.input_size + 2) {
            uint16_t slave_error = static_cast<uint16_t>(
                frame_data[config_.input_size] |
                (frame_data[config_.input_size + 1] << 8));
            trace("RX FailSafeData(0x08): slave entered fail-safe, "
                  "reported error=0x%04X (state=%s)",
                  slave_error, stateName(status_.state));
            FSoEErrorDetail detail;
            snprintf(detail.message, sizeof(detail.message),
                     "Slave entered fail-safe and reported error 0x%04X",
                     slave_error);
            handleError(slave_error, detail);
        } else {
            FSoEErrorDetail detail;
            snprintf(detail.message, sizeof(detail.message),
                     "Slave sent fail-safe response but payload too short "
                     "(got %zu bytes, need %u+2)",
                     data_len, config_.input_size);
            handleError(ErrorCode::ApplicationError, detail);
        }
        return true;
    }

    // Process based on current state
    switch (status_.state) {
        case ConnectionState::Reset:
            handleResetState(cmd, frame_data, data_len);
            break;

        case ConnectionState::Session:
            handleSessionState(cmd, frame_data, data_len);
            break;

        case ConnectionState::Connection:
            handleConnectionState(cmd, frame_data, data_len);
            break;

        case ConnectionState::Parameter:
            handleParameterState(cmd, frame_data, data_len);
            break;

        case ConnectionState::Data:
            handleDataState(cmd, frame_data, data_len);
            break;

        case ConnectionState::FailSafe:
            handleFailSafeState(cmd, frame_data, data_len);
            break;

        case ConnectionState::Error:
            if (cmd == Command::Reset) {
                if (config_.auto_recovery_enabled) {
                    trace("RX Reset(0x2A): recovering from Error state");
                    resetConnection();
                    stats_.successful_recoveries++;
                }
            } else {
                // Ignore non-Reset commands in Error state
                trace("RX %s: ignored in Error state (expected Reset)",
                      commandName(cmd));
                return false;
            }
            break;
    }

    return true;
}

// ============================================================================
// State Machine — prepareTxFrame
// ============================================================================

size_t FSoEMasterConnection::prepareTxFrame(uint8_t* data, size_t max_len)
{
    std::lock_guard<std::recursive_mutex> lock(mutex_);

    if (!initialized_ || !data) return 0;

    // In Reset state, send a Reset command (0x2A) to force the slave back
    // to its initial state before starting the Session handshake.  The
    // master stays in Reset until it receives the slave's Session response
    // (handled in handleResetState), at which point it transitions to
    // Session and begins the handshake with a fresh session ID.
    size_t len = 0;

    switch (status_.state) {
        case ConnectionState::Reset:
            len = buildResetFrame(data, max_len);
            if (len > 0) {
                trace("TX Reset(0x2A): forcing slave to initial state "
                      "(state=Reset, %zu bytes)", len);
            }
            break;

        case ConnectionState::Session:
            len = buildSessionResetFrame(data, max_len);
            if (len > 0) {
                trace("TX Session(0x4E): starting handshake with "
                      "session_id=0x%04X (state=Session, %zu bytes)",
                      status_.session_id, len);
            }
            break;

        case ConnectionState::Connection:
            len = buildConnectionFrame(data, max_len);
            if (len > 0) {
                trace("TX Connection(0x64): requesting connection with "
                      "safety_addr=0x%04X param_crc=0x%04X "
                      "(state=Connection, %zu bytes)",
                      config_.slave_safety_addr, parameter_crc_, len);
            }
            break;

        case ConnectionState::Parameter:
            len = buildParameterFrame(data, max_len);
            if (len > 0) {
                trace("TX Parameter(0x52): watchdog=%u ms safety_level=%u "
                      "input_size=%u output_size=%u (state=Parameter, %zu bytes)",
                      config_.watchdog_timeout_ms, config_.safety_level,
                      config_.input_size, config_.output_size, len);
            }
            break;

        case ConnectionState::Data:
            len = buildDataFrame(data, max_len);
            if (len > 0) {
                trace("TX ProcessData(0x36): %u bytes of safe outputs "
                      "(state=Data, %zu bytes)",
                      config_.output_size, len);
            }
            break;

        case ConnectionState::FailSafe:
            len = buildFailSafeFrame(data, max_len);
            if (len > 0) {
                trace("TX FailSafeData(0x08): %u bytes of fail-safe values "
                      "(state=FailSafe, %zu bytes)",
                      config_.output_size, len);
            }
            break;

        case ConnectionState::Error:
            // No frames in Error state
            break;
    }

    if (len > 0) {
        stats_.frames_sent++;
        // Increment TX sequence number for CRC computation — but NOT for
        // Reset frames (3-byte frames with no CRC).  Reset frames don't
        // advance the CRC chain, so the sequence number must stay the same.
        // The sequence number is NOT transmitted — it is folded into the CRC
        // and shared between master and slave.
        if (len > CRC::MIN_FSOE_FRAME_SIZE) {
            tx_seq_no_++;
        }
        tx_frame_events_.emit([data, len] {
            return std::make_shared<const std::vector<uint8_t>>(data, data + len);
        });
    }

    return len;
}

// ============================================================================
// Update / Watchdog
// ============================================================================

void FSoEMasterConnection::update(uint64_t current_time_ms)
{
    std::lock_guard<std::recursive_mutex> lock(mutex_);

    current_time_ms_ = current_time_ms;
    stats_.uptime_ms = current_time_ms;

    checkPhaseTimeout(current_time_ms);
    checkWatchdog(current_time_ms);

    // Auto-recovery attempt
    if (status_.state == ConnectionState::FailSafe && config_.auto_recovery_enabled) {
        attemptAutoRecovery(current_time_ms);
    }
}

void FSoEMasterConnection::checkPhaseTimeout(uint64_t current_time_ms)
{
    // Reset state: fall back to Session if the slave doesn't respond within
    // the reset timeout.  The Reset command (0x2A) is a non-standard extension
    // that not all FSoE slaves support.  If the slave ignores it, the master
    // times out and starts the standard FSoE handshake with a Session command.
    if (status_.state == ConnectionState::Reset) {
        if (config_.reset_timeout_ms > 0) {
            uint64_t elapsed = current_time_ms - status_.state_entered_ms;
            if (elapsed > config_.reset_timeout_ms) {
                stats_.timeout_events++;
                // Slave did not respond to Reset(0x2A) — fall back to the
                // standard FSoE Session handshake.
                trace("Reset state timeout after %llu ms, falling back to Session handshake",
                      static_cast<unsigned long long>(elapsed));
                requestSessionReset();
            }
        }
        return;
    }

    if (status_.state == ConnectionState::Data ||
        status_.state == ConnectionState::FailSafe ||
        status_.state == ConnectionState::Error) {
        return;
    }

    uint64_t elapsed = current_time_ms - status_.state_entered_ms;
    uint16_t timeout = config_.conn_timeout_ms;

    if (status_.state == ConnectionState::Session) {
        timeout = config_.session_timeout_ms;
    }

    if (elapsed > timeout) {
        stats_.timeout_events++;
        FSoEErrorDetail detail;
        snprintf(detail.message, sizeof(detail.message),
                 "Phase timeout in state %u after %llu ms (limit %u ms)",
                 status_.state, static_cast<unsigned long long>(elapsed),
                 timeout);
        handleError(ErrorCode::TimeoutError, detail);
    }
}

void FSoEMasterConnection::checkWatchdog(uint64_t current_time_ms)
{
    if (status_.state != ConnectionState::Data) return;

    uint64_t elapsed = current_time_ms - status_.last_valid_frame_ms;
    status_.watchdog_counter = static_cast<uint32_t>(elapsed);

    if (elapsed > config_.watchdog_timeout_ms) {
        stats_.watchdog_events++;
        FSoEErrorDetail detail;
        snprintf(detail.message, sizeof(detail.message),
                 "Watchdog timeout in Data state after %llu ms (limit %u ms)",
                 static_cast<unsigned long long>(elapsed),
                 config_.watchdog_timeout_ms);
        handleError(ErrorCode::WatchdogError, detail);
    }
}

void FSoEMasterConnection::attemptAutoRecovery(uint64_t current_time_ms)
{
    uint64_t elapsed = current_time_ms - fail_safe_entered_ms_;
    if (elapsed >= config_.recovery_delay_ms) {
        trace("Auto-recovery: attempting reset after %llu ms in fail-safe",
              static_cast<unsigned long long>(elapsed));
        stats_.recovery_attempts++;
        resetConnection();
    }
}

// ============================================================================
// State Handlers
// ============================================================================

void FSoEMasterConnection::handleResetState(uint8_t cmd, const uint8_t* data, size_t data_len)
{
    (void)data;
    (void)data_len;

    // The master sent a Reset command (0x2A) to force the slave back to its
    // initial state.  The slave acknowledges by transitioning to Session and
    // responding with a Session frame (0x4E).  When we see that response,
    // generate a fresh session ID and transition to Session to begin the
    // handshake.  A Reset response (0x2A) is also accepted — some slaves
    // echo the Reset command before switching to Session.
    if (cmd == Command::Session || cmd == Command::Reset) {
        trace("RX %s: slave acknowledged reset, starting session handshake",
              commandName(cmd));
        requestSessionReset();
    } else {
        // Unexpected command in Reset state — ignore and keep retrying
        trace("RX %s: unexpected in Reset state (expected Session or Reset)",
              commandName(cmd));
        FSoEErrorDetail detail;
        snprintf(detail.message, sizeof(detail.message),
                 "Unexpected command 0x%02X in Reset state (expected Session or Reset)",
                 cmd);
        handleError(ErrorCode::CommandError, detail);
    }
}

void FSoEMasterConnection::handleSessionState(uint8_t cmd, const uint8_t* data, size_t data_len)
{
    (void)data;
    (void)data_len;

    if (cmd == Command::Session) {
        trace("RX Session(0x4E): slave accepted session, moving to Connection");
        transitionTo(ConnectionState::Connection);
    } else {
        trace("RX %s: unexpected in Session state (expected Session)",
              commandName(cmd));
        FSoEErrorDetail detail;
        snprintf(detail.message, sizeof(detail.message),
                 "Unexpected command 0x%02X in Session state (expected Session)",
                 cmd);
        handleError(ErrorCode::CommandError, detail);
    }
}

void FSoEMasterConnection::handleConnectionState(uint8_t cmd, const uint8_t* data, size_t data_len)
{
    if (cmd == Command::Connection) {
        // Validate slave's safety address and SIL from connection response.
        // Parsed data format: [safetyAddr_lo][safetyAddr_hi][sil][reserved]
        // Require the full 4-byte payload — the slave always sends 4 bytes.
        if (data_len < 4) {
            trace("RX Connection(0x64): response too short (%zu bytes, expected 4)",
                  data_len);
            FSoEErrorDetail detail;
            snprintf(detail.message, sizeof(detail.message),
                     "Connection response too short: got %zu bytes, expected 4",
                     data_len);
            handleError(ErrorCode::DataLengthError, detail);
            return;
        }
        uint16_t slave_safety_addr = data[0] | (data[1] << 8);
        if (slave_safety_addr != 0 && slave_safety_addr != config_.slave_safety_addr) {
            trace("RX Connection(0x64): safety address mismatch "
                  "(expected 0x%04X got 0x%04X)",
                  config_.slave_safety_addr, slave_safety_addr);
            FSoEErrorDetail detail;
            detail.conn_id_valid = true;
            detail.expected_conn_id = config_.slave_safety_addr;
            detail.received_conn_id = slave_safety_addr;
            snprintf(detail.message, sizeof(detail.message),
                     "Slave safety address mismatch: expected 0x%04X got 0x%04X",
                     detail.expected_conn_id, detail.received_conn_id);
            handleError(ErrorCode::ConnectionIDError, detail);
            return;
        }
        uint8_t slave_sil = data[2];
        if (slave_sil < config_.safety_level) {
            trace("RX Connection(0x64): slave SIL %u below required %u",
                  slave_sil, config_.safety_level);
            FSoEErrorDetail detail;
            snprintf(detail.message, sizeof(detail.message),
                     "Slave SIL %u below required SIL %u",
                     slave_sil, config_.safety_level);
            handleError(ErrorCode::ApplicationError, detail);
            return;
        }
        trace("RX Connection(0x64): slave confirmed safety_addr=0x%04X SIL=%u, "
              "moving to %s",
              slave_safety_addr, slave_sil,
              (config_.input_size > 0 || config_.output_size > 0)
                  ? "Parameter" : "Data");
        if (config_.input_size > 0 || config_.output_size > 0) {
            transitionTo(ConnectionState::Parameter);
        } else {
            transitionTo(ConnectionState::Data);
        }
    } else {
        trace("RX %s: unexpected in Connection state (expected Connection)",
              commandName(cmd));
        FSoEErrorDetail detail;
        snprintf(detail.message, sizeof(detail.message),
                 "Unexpected command 0x%02X in Connection state (expected Connection)",
                 cmd);
        handleError(ErrorCode::CommandError, detail);
    }
}

void FSoEMasterConnection::handleParameterState(uint8_t cmd, const uint8_t* data, size_t data_len)
{
    (void)data;
    (void)data_len;

    if (cmd == Command::Parameter) {
        trace("RX Parameter(0x52): slave accepted parameters, moving to Data");
        current_param_index_++;
        if (current_param_index_ >= 1) {
            transitionTo(ConnectionState::Data);
        }
    } else if (cmd == Command::ProcessData) {
        trace("RX ProcessData(0x36): slave skippped Parameter phase, moving to Data");
        transitionTo(ConnectionState::Data);
        handleDataState(cmd, data, data_len);
    } else {
        trace("RX %s: unexpected in Parameter state (expected Parameter or ProcessData)",
              commandName(cmd));
        FSoEErrorDetail detail;
        snprintf(detail.message, sizeof(detail.message),
                 "Unexpected command 0x%02X in Parameter state (expected Parameter or ProcessData)",
                 cmd);
        handleError(ErrorCode::CommandError, detail);
    }
}

void FSoEMasterConnection::handleDataState(uint8_t cmd, const uint8_t* data, size_t data_len)
{
    if (cmd != Command::ProcessData) {
        if (cmd == Command::Reset) {
            trace("RX Reset(0x2A): slave requested reset in Data state, resetting connection");
            resetConnection();
            return;
        }
        trace("RX %s: unexpected in Data state (expected ProcessData)",
              commandName(cmd));
        FSoEErrorDetail detail;
        snprintf(detail.message, sizeof(detail.message),
                 "Unexpected command 0x%02X in Data state (expected ProcessData)",
                 cmd);
        handleError(ErrorCode::CommandError, detail);
        return;
    }

    // ETG.5100 does not define a sequence number field
    // Frame integrity is ensured via CRC + watchdog

    if (data_len < config_.input_size) {
        stats_.invalid_frames++;
        trace("RX ProcessData(0x36): frame too short (%zu bytes, expected %u)",
              data_len, config_.input_size);
        FSoEErrorDetail detail;
        snprintf(detail.message, sizeof(detail.message),
                 "Data frame too short: got %zu bytes, expected %u",
                 data_len, config_.input_size);
        handleError(ErrorCode::DataLengthError, detail);
        return;
    }

    std::copy(data, data + config_.input_size, safe_inputs_.begin());
    status_.data_valid = true;
    trace("RX ProcessData(0x36): %u bytes of safe inputs (state=Data)",
          config_.input_size);

    if (data_callback_) {
        data_callback_(safe_inputs_.data(), config_.input_size);
    }
}

void FSoEMasterConnection::handleFailSafeState(uint8_t cmd, const uint8_t* data, size_t data_len)
{
    if (cmd == Command::Reset) {
        if (config_.auto_recovery_enabled) {
            trace("RX Reset(0x2A): slave ready to recover, resetting connection");
            stats_.successful_recoveries++;
            resetConnection();
        } else {
            trace("RX Reset(0x2A): slave ready to recover, but auto-recovery disabled");
        }
    } else if (cmd == Command::FailSafeData) {
        // Slave is also in fail-safe — acknowledge by staying in fail-safe.
        // Recovery will be attempted by attemptAutoRecovery() in update().
        // Extract slave error code for diagnostics (input_size bytes of
        // fail-safe inputs followed by 2-byte error code).
        if (data_len >= config_.input_size + 2) {
            uint16_t slave_error = static_cast<uint16_t>(
                data[config_.input_size] | (data[config_.input_size + 1] << 8));
            trace("RX FailSafeData(0x08): slave also in fail-safe (error=0x%04X)",
                  slave_error);
            // Update error code only if the slave reports a different error
            // than what we already have — avoids overwriting our own error.
            if (slave_error != ErrorCode::NoError &&
                slave_error != status_.error_code) {
                status_.error_code = slave_error;
            }
        } else {
            trace("RX FailSafeData(0x08): slave also in fail-safe (no error code)");
        }
    } else {
        // Unexpected command in FailSafe state
        trace("RX %s: unexpected in FailSafe state (expected Reset or FailSafeData)",
              commandName(cmd));
        FSoEErrorDetail detail;
        snprintf(detail.message, sizeof(detail.message),
                 "Unexpected command 0x%02X in FailSafe state (expected Reset or FailSafeData)",
                 cmd);
        handleError(ErrorCode::CommandError, detail);
    }
}

// ============================================================================
// Frame Building
// ============================================================================

size_t FSoEMasterConnection::buildResetFrame(uint8_t* data, size_t max_len)
{
    // Reset frame: CMD(0x2A) + ConnID (no safe data payload).
    // This is the minimum FSoE frame size (3 bytes).
    size_t needed = CRC::fsoeFrameSize(0);
    if (max_len < needed) return 0;
    return CRC::buildFSoEFrame(data, Command::Reset, nullptr, 0, config_.connection_id,
                               last_tx_crc0_, tx_seq_no_, &last_tx_crc0_);
}

size_t FSoEMasterConnection::buildSessionResetFrame(uint8_t* data, size_t max_len)
{
    // Session reset: CMD + session_id (2B payload) + ConnID
    uint8_t payload[2] = {0, 0};
    payload[0] = static_cast<uint8_t>(status_.session_id & 0xFF);
    payload[1] = static_cast<uint8_t>((status_.session_id >> 8) & 0xFF);
    size_t needed = CRC::fsoeFrameSize(2);
    if (max_len < needed) return 0;
    return CRC::buildFSoEFrame(data, Command::Session, payload, 2, config_.connection_id,
                               last_tx_crc0_, tx_seq_no_, &last_tx_crc0_);
}

size_t FSoEMasterConnection::buildConnectionFrame(uint8_t* data, size_t max_len)
{
    // Connection frame: CMD + payload(safety_addr 2B + param_crc 2B = 4B) + ConnID
    uint8_t payload[4] = {0, 0, 0, 0};
    payload[0] = config_.slave_safety_addr & 0xFF;
    payload[1] = (config_.slave_safety_addr >> 8) & 0xFF;
    payload[2] = parameter_crc_ & 0xFF;
    payload[3] = (parameter_crc_ >> 8) & 0xFF;
    size_t needed = CRC::fsoeFrameSize(4);
    if (max_len < needed) return 0;
    return CRC::buildFSoEFrame(data, Command::Connection, payload, 4, config_.connection_id,
                               last_tx_crc0_, tx_seq_no_, &last_tx_crc0_);
}

size_t FSoEMasterConnection::buildParameterFrame(uint8_t* data, size_t max_len)
{
    // Parameter frame: CMD + param data (6B) + ConnID
    // Layout: [watchdog_lo] [watchdog_hi] [safety_level] [input_size] [output_size] [reserved]
    uint8_t payload[6] = {0, 0, 0, 0, 0, 0};
    payload[0] = config_.watchdog_timeout_ms & 0xFF;
    payload[1] = (config_.watchdog_timeout_ms >> 8) & 0xFF;
    payload[2] = config_.safety_level;
    payload[3] = config_.input_size;
    payload[4] = config_.output_size;
    payload[5] = 0;  // reserved
    size_t needed = CRC::fsoeFrameSize(6);
    if (max_len < needed) return 0;
    return CRC::buildFSoEFrame(data, Command::Parameter, payload, 6, config_.connection_id,
                               last_tx_crc0_, tx_seq_no_, &last_tx_crc0_);
}

size_t FSoEMasterConnection::buildDataFrame(uint8_t* data, size_t max_len)
{
    // Data frame: CMD + safe_outputs + ConnID
    size_t needed = CRC::fsoeFrameSize(config_.output_size);
    if (max_len < needed) return 0;
    return CRC::buildFSoEFrame(data, Command::ProcessData,
                               safe_outputs_.data(), config_.output_size,
                               config_.connection_id,
                               last_tx_crc0_, tx_seq_no_, &last_tx_crc0_);
}

size_t FSoEMasterConnection::buildFailSafeFrame(uint8_t* data, size_t max_len)
{
    // Fail-safe frame sends FailSafeData command with fail-safe output values
    size_t needed = CRC::fsoeFrameSize(config_.output_size);
    if (max_len < needed) return 0;
    return CRC::buildFSoEFrame(data, Command::FailSafeData,
                               config_.fail_safe_values.data(), config_.output_size,
                               config_.connection_id,
                               last_tx_crc0_, tx_seq_no_, &last_tx_crc0_);
}

// ============================================================================
// Frame Validation
// ============================================================================

bool FSoEMasterConnection::validateCRC(const uint8_t* data, size_t len) const
{
    uint8_t cmd = 0;
    size_t data_len = 0;
    uint16_t conn_id = 0;
    return CRC::parseFSoEFrame(data, len, cmd, nullptr, data_len, conn_id,
                               last_rx_crc0_, rx_seq_no_);
}

bool FSoEMasterConnection::validateSequence(uint8_t seq)
{
    // The FSoE sequence number is NOT transmitted in the frame — it is
    // folded into the CRC computation and shared between master and slave.
    // If the sequence numbers diverge, CRC verification fails (which is the
    // intended safety behavior).  This method is kept for API compatibility
    // but the actual sequence validation happens via CRC verification.
    // See: https://techoverflow.net/2026/08/09/fsoe-how-does-crc-inheritance-work/
    (void)seq;
    return true;
}

bool FSoEMasterConnection::validateConnectionID(uint16_t conn_id) const
{
    return conn_id == config_.connection_id;
}

bool FSoEMasterConnection::isValidCommand(uint8_t cmd)
{
    switch (cmd) {
        case Command::ProcessData:
        case Command::Reset:
        case Command::Session:
        case Command::Connection:
        case Command::Parameter:
        case Command::FailSafeData:
            return true;
        default:
            return false;
    }
}

// ============================================================================
// State Transitions
// ============================================================================

void FSoEMasterConnection::transitionTo(uint8_t new_state)
{
    if (new_state == status_.state) return;

    uint8_t old_state = status_.state;
    status_.state = new_state;
    status_.state_entered_ms = current_time_ms_;

    if (old_state == ConnectionState::Data && new_state != ConnectionState::Data) {
        status_.data_valid = false;
    }

    if (state_change_callback_) {
        state_change_callback_(old_state, new_state);
    }
}

void FSoEMasterConnection::handleError(uint16_t error_code,
                                        const FSoEErrorDetail& detail)
{
    status_.error_code = error_code;

    if (config_.auto_fail_safe_on_error) {
        // Go directly to fail-safe (skip Error state)
        triggerFailSafe(error_code);
    } else {
        // Stay in Error state (persistent until explicitly cleared)
        transitionTo(ConnectionState::Error);
    }

    if (error_callback_) {
        error_callback_(error_code, detail);
    }
}

uint16_t FSoEMasterConnection::computeParameterCRC() const
{
    // Compute CRC over the parameter set.
    // Layout: 11 fixed bytes + fail_safe_values (up to 16 bytes) = up to 27 bytes.
    std::array<uint8_t, 27> param_data{};

    param_data[0] = config_.watchdog_timeout_ms & 0xFF;
    param_data[1] = (config_.watchdog_timeout_ms >> 8) & 0xFF;
    param_data[2] = config_.conn_timeout_ms & 0xFF;
    param_data[3] = (config_.conn_timeout_ms >> 8) & 0xFF;
    param_data[4] = config_.safety_level;
    param_data[5] = config_.input_size;
    param_data[6] = config_.output_size;
    param_data[7] = config_.slave_addr & 0xFF;
    param_data[8] = (config_.slave_addr >> 8) & 0xFF;
    param_data[9] = config_.master_addr & 0xFF;
    param_data[10] = (config_.master_addr >> 8) & 0xFF;

    for (size_t i = 0; i < config_.fail_safe_values.size(); ++i) {
        param_data[11 + i] = config_.fail_safe_values[i];
    }

    return CRC::calculate(param_data.data(), param_data.size());
}

// ============================================================================
// Safe Data Access
// ============================================================================

bool FSoEMasterConnection::setSafeOutputs(const uint8_t* data, size_t len)
{
    std::lock_guard<std::recursive_mutex> lock(mutex_);

    if (!data || len != config_.output_size) return false;
    if (status_.isFailSafe()) return false;

    std::copy(data, data + len, safe_outputs_.begin());
    return true;
}

bool FSoEMasterConnection::writeOutputProcessData(std::span<const uint8_t> data)
{
    return setSafeOutputs(data.data(), data.size());
}

size_t FSoEMasterConnection::getSafeInputs(uint8_t* data, size_t len) const
{
    std::lock_guard<std::recursive_mutex> lock(mutex_);

    if (!data || len < config_.input_size) return 0;
    if (!status_.data_valid) return 0;

    std::copy(safe_inputs_.begin(), safe_inputs_.begin() + config_.input_size, data);
    return config_.input_size;
}

size_t FSoEMasterConnection::readInputProcessData(std::span<uint8_t> data) const
{
    return getSafeInputs(data.data(), data.size());
}

std::vector<uint8_t> FSoEMasterConnection::inputProcessData() const
{
    std::lock_guard<std::recursive_mutex> lock(mutex_);

    std::vector<uint8_t> data(config_.input_size, 0);
    const size_t copied = getSafeInputs(data.data(), data.size());
    data.resize(copied);
    return data;
}

std::vector<uint8_t> FSoEMasterConnection::outputProcessData() const
{
    std::lock_guard<std::recursive_mutex> lock(mutex_);

    return std::vector<uint8_t>(safe_outputs_.begin(),
                                safe_outputs_.begin() + config_.output_size);
}

bool FSoEMasterConnection::exchangeWith(FSoESlave& slave, uint64_t current_time_ms)
{
    std::lock_guard<std::recursive_mutex> lock(mutex_);

    update(current_time_ms);
    slave.update(current_time_ms);

    std::array<uint8_t, 64> tx{};
    std::array<uint8_t, 64> rx{};

    const size_t tx_len = prepareTxFrame(tx.data(), tx.size());
    if (tx_len == 0) {
        return false;
    }

    if (!slave.processRxFrame(tx.data(), tx_len)) {
        return false;
    }

    const size_t rx_len = slave.prepareTxFrame(rx.data(), rx.size());
    if (rx_len == 0) {
        return false;
    }

    return processRxFrame(rx.data(), rx_len);
}

// ============================================================================
// PDO frame-size translation
// ============================================================================
//
// The FSoE frame has a variable length (ConnID at the END of the frame, not
// at a fixed PDO position).  When mapped into a fixed-size EtherCAT PDO,
// only the first N bytes of the PDO are the actual FSoE frame; the remaining
// bytes are unused/stale.  Both master and slave must determine the frame
// size from the command byte and only process the first N bytes.
//
// Frame sizes by command:
//   Reset(0x2A)      : 3 bytes  (0 data bytes, no CRC)
//   Session(0x4E)    : 7 bytes  (2 data bytes, 1 CRC)
//   Connection(0x64) : 11 bytes (4 data bytes, 2 CRCs)
//   Parameter(0x52)  : 15 bytes (6 data bytes, 3 CRCs)
//   ProcessData(0x36): fsoeFrameSize(input_size)
//   FailSafeData(0x08): fsoeFrameSize(input_size)

size_t FSoEMasterConnection::expectedRxFrameSize(uint8_t cmd, size_t pdo_size) const
{
    size_t frame_size = 0;

    switch (cmd) {
        case Command::Reset:
            frame_size = CRC::fsoeFrameSize(0);       // 3 bytes
            break;
        case Command::Session:
            frame_size = CRC::fsoeFrameSize(2);       // 7 bytes
            break;
        case Command::Connection:
            frame_size = CRC::fsoeFrameSize(4);       // 11 bytes
            break;
        case Command::Parameter:
            frame_size = CRC::fsoeFrameSize(6);       // 15 bytes
            break;
        case Command::ProcessData:
        case Command::FailSafeData:
            // Data/FailSafe frames use the negotiated input_size.
            // FailSafeData may include a 2-byte error code after the safe
            // inputs, but that only fits if the PDO is large enough.
            frame_size = CRC::fsoeFrameSize(config_.input_size);
            // Also check if FailSafeData with error code fits
            if (cmd == Command::FailSafeData) {
                size_t with_error = CRC::fsoeFrameSize(
                    static_cast<size_t>(config_.input_size) + 2);
                if (with_error <= pdo_size) {
                    frame_size = with_error;
                }
            }
            break;
        default:
            // Unrecognized command (0x00 = stale/empty PDO, or garbage)
            return 0;
    }

    // Cap at PDO size — the frame can't be larger than the PDO buffer
    if (frame_size > pdo_size) {
        frame_size = pdo_size;
    }

    return frame_size;
}

bool FSoEMasterConnection::exchangeViaPDO(uint8_t* rx_pdo_out, size_t rx_pdo_max,
                                          const uint8_t* tx_pdo_in, size_t tx_pdo_len,
                                          uint64_t current_time_ms)
{
    std::lock_guard<std::recursive_mutex> lock(mutex_);

    // Run the FSoE state machine (watchdog, phase timeouts, auto-recovery).
    update(current_time_ms);

    // ========================================================================
    // TX path: build FSoE frame and translate to device-specific RxPDO
    // ========================================================================
    //
    // The FSoE frame (ETG.5100) has ConnID at the END of the frame at a
    // variable position:
    //   Reset(0x2A):      [CMD, ConnID]                           = 3 bytes
    //   Session(0x4E):    [CMD, D0, D1, CRC0, ConnID]             = 7 bytes
    //   Connection(0x64): [CMD, D0, D1, CRC0, D2, D3, CRC1, ConnID] = 11 bytes
    //   ProcessData(0x36): [CMD, D0, D1, CRC0, D2, D3, CRC1, ConnID] = 11 bytes
    //
    // But the device-specific PDO (e.g. Synapticon SOMANET_RxPDO_1700)
    // has ConnID at a FIXED position — the last 2 bytes of the PDO:
    //   [CMD, Data..., CRCs..., ..., ConnID]  (ConnID always at end of PDO)
    //
    // For the DATA state, the FSoE frame size equals the PDO size, so
    // the layouts match and no translation is needed.
    //
    // For shorter frames (Reset, Session, Connection), the FSoE frame
    // is shorter than the PDO.  We need to:
    //   1. Write the FSoE frame at the beginning of the PDO
    //   2. Move ConnID from its variable-length position to the PDO end
    //   3. Zero-fill the gap between the frame and ConnID
    //
    // The slave determines the frame size from the command byte and only
    // processes the relevant data+CRC pairs.  The zero-filled gap and the
    // ConnID at the PDO end are ignored by the FSoE stack but are part of
    // the fixed PDO mapping expected by the EtherCAT slave.

    const size_t tx_len = prepareTxFrame(rx_pdo_out, rx_pdo_max);
    if (tx_len == 0) {
        return false;
    }

    // Translate the FSoE frame to the device-specific PDO layout.
    // Move ConnID from its variable-length position (end of frame) to the
    // fixed position (end of PDO), and zero-fill the gap.
    if (tx_len < rx_pdo_max && rx_pdo_max >= CRC::MIN_FSOE_FRAME_SIZE) {
        // Extract ConnID from the variable-length frame (last 2 bytes of frame)
        uint16_t conn_id = static_cast<uint16_t>(rx_pdo_out[tx_len - 2]) |
                           (static_cast<uint16_t>(rx_pdo_out[tx_len - 1]) << 8);

        // Zero-fill from the end of the frame to the end of the PDO
        // (overwrites the ConnID at its variable-length position)
        std::fill(rx_pdo_out + tx_len - 2, rx_pdo_out + rx_pdo_max, 0x00);

        // Write ConnID at the fixed position (last 2 bytes of PDO)
        rx_pdo_out[rx_pdo_max - 2] = conn_id & 0xFF;
        rx_pdo_out[rx_pdo_max - 1] = (conn_id >> 8) & 0xFF;
    }

    // ========================================================================
    // RX path: extract FSoE frame from device-specific TxPDO
    // ========================================================================
    //
    // The slave writes its FSoE response into the TxPDO with ConnID at the
    // fixed position (last 2 bytes of PDO).  The FSoE frame data (CMD +
    // data + CRCs) is at the beginning of the PDO.
    //
    // We need to reconstruct the variable-length FSoE frame by:
    //   1. Reading the command byte from PDO byte 0
    //   2. Determining the frame size from the command
    //   3. Taking bytes 0..frame_size-3 from the PDO (CMD + data + CRCs)
    //   4. Appending ConnID from the last 2 bytes of the PDO
    //
    // This reconstructs the FSoE frame with ConnID at the variable-length
    // position, which processRxFrame can parse correctly.

    // Startup grace period: skip RxFrame processing for the first few
    // PDO cycles.  The slave cannot have produced a valid response until
    // it has received at least one master frame and had time to process
    // it.  Without DC synchronization there is a one-cycle pipeline
    // delay (master writes cycle N → slave reads N+1 → slave writes N+1
    // → master reads N+2), so we need to skip 2 cycles.  With DC sync
    // the slave responds within the same cycle, but skipping 2 is
    // harmless — the master just re-sends its current-state frame.
    constexpr uint32_t kStartupSkipCycles = 2;
    if (pdo_tx_count_ < kStartupSkipCycles) {
        trace("PDO startup: skipping RX (cycle %u/%u, slave hasn't responded yet)",
              pdo_tx_count_ + 1, kStartupSkipCycles);
        pdo_tx_count_++;
        return false;
    }

    if (tx_pdo_len == 0 || tx_pdo_in == nullptr) {
        return false;
    }

    const uint8_t rx_cmd = tx_pdo_in[0];
    const size_t rx_frame_size = expectedRxFrameSize(rx_cmd, tx_pdo_len);

    if (rx_frame_size == 0) {
        // Unrecognized command (0x00 = stale/empty PDO, or garbage).
        // Silently skip — the master keeps retrying with its current-state
        // frame.  This handles the first few cycles before the slave has
        // populated the TxPDO.
        trace("PDO RX: skipping frame with unrecognized cmd=0x%02X (stale PDO?)",
              rx_cmd);
        return false;
    }

    // Reconstruct the FSoE frame from the TxPDO.
    //
    // For the DATA state, the frame size equals the PDO size, so the
    // frame is the entire PDO (ConnID is already at the right position).
    //
    // For shorter frames, ConnID is at the END of the PDO (fixed position),
    // not at the end of the frame.  We take the first (frame_size - 2)
    // bytes (CMD + data + CRCs) and append ConnID from the PDO end.
    if (rx_frame_size == tx_pdo_len) {
        // Frame fills the entire PDO — no reconstruction needed
        // (e.g. ProcessData with 14 input bytes in a 31-byte TxPDO)
        return processRxFrame(tx_pdo_in, rx_frame_size);
    }

    // Reconstruct: take first (frame_size - 2) bytes, append ConnID from PDO end
    uint8_t reconstructed[CRC::MAX_PARSE_DATA_SIZE + 10] = {0};
    const size_t data_and_crc_len = rx_frame_size - 2;  // everything except ConnID

    if (data_and_crc_len > 0) {
        std::copy(tx_pdo_in, tx_pdo_in + data_and_crc_len, reconstructed);
    }

    // Append ConnID from the last 2 bytes of the TxPDO (fixed position)
    reconstructed[data_and_crc_len]     = tx_pdo_in[tx_pdo_len - 2];
    reconstructed[data_and_crc_len + 1] = tx_pdo_in[tx_pdo_len - 1];

    return processRxFrame(reconstructed, rx_frame_size);
}

bool FSoEMasterConnection::areSafeInputsValid() const
{
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    return status_.data_valid && status_.isOperational();
}

// ============================================================================
// Bit-level Safe I/O
// ============================================================================

bool FSoEMasterConnection::getSafeInputBit(uint8_t bit_index) const
{
    std::lock_guard<std::recursive_mutex> lock(mutex_);

    if (!status_.data_valid) return false;

    uint8_t byte_idx = bit_index / 8;
    uint8_t bit_pos = bit_index % 8;

    if (byte_idx >= config_.input_size) return false;

    return (safe_inputs_[byte_idx] >> bit_pos) & 1;
}

bool FSoEMasterConnection::setSafeOutputBit(uint8_t bit_index, bool value)
{
    std::lock_guard<std::recursive_mutex> lock(mutex_);

    if (status_.isFailSafe()) return false;

    uint8_t byte_idx = bit_index / 8;
    uint8_t bit_pos = bit_index % 8;

    if (byte_idx >= config_.output_size) return false;

    if (value) {
        safe_outputs_[byte_idx] |= (1 << bit_pos);
    } else {
        safe_outputs_[byte_idx] &= ~(1 << bit_pos);
    }

    return true;
}

uint8_t FSoEMasterConnection::getSafeInputByte(uint8_t byte_index) const
{
    std::lock_guard<std::recursive_mutex> lock(mutex_);

    if (!status_.data_valid || byte_index >= config_.input_size) {
        return 0;
    }
    return safe_inputs_[byte_index];
}

bool FSoEMasterConnection::setSafeOutputByte(uint8_t byte_index, uint8_t value)
{
    std::lock_guard<std::recursive_mutex> lock(mutex_);

    if (status_.isFailSafe() || byte_index >= config_.output_size) {
        return false;
    }
    safe_outputs_[byte_index] = value;
    return true;
}

// ============================================================================
// Status & Diagnostics
// ============================================================================

MasterConnectionStatus FSoEMasterConnection::getStatus() const
{
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    return status_;
}

uint8_t FSoEMasterConnection::getState() const
{
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    return status_.state;
}

uint16_t FSoEMasterConnection::getErrorCode() const
{
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    return status_.error_code;
}

bool FSoEMasterConnection::isOperational() const
{
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    return status_.isOperational();
}

bool FSoEMasterConnection::isFailSafe() const
{
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    return status_.isFailSafe();
}

ConnectionStats FSoEMasterConnection::getStats() const
{
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    return stats_;
}

void FSoEMasterConnection::resetStats()
{
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    stats_ = {};
}

std::string FSoEMasterConnection::getDiagnostics() const
{
    std::lock_guard<std::recursive_mutex> lock(mutex_);

    std::string diag;

    diag += "FSoE Master Connection Diagnostics:\n";
    diag += "  Connection ID: " + std::to_string(config_.connection_id) + "\n";
    diag += "  Slave Address: " + std::to_string(config_.slave_addr) + "\n";
    diag += "  State: " + std::to_string(status_.state) + "\n";
    diag += "  Operational: " + std::string(status_.isOperational() ? "Yes" : "No") + "\n";
    diag += "  Fail-Safe: " + std::string(status_.isFailSafe() ? "Yes" : "No") + "\n";
    diag += "  Data Valid: " + std::string(status_.data_valid ? "Yes" : "No") + "\n";

    if (status_.hasError()) {
        char buf[32];
        snprintf(buf, sizeof(buf), "0x%04X", status_.error_code);
        diag += "  ERROR: " + std::string(buf) + "\n";
    }

    diag += "  Session ID: " + std::to_string(status_.session_id) + "\n";
    diag += "  RX Sequence: " + std::to_string(rx_sequence_) + "\n";
    diag += "  TX Sequence: " + std::to_string(tx_sequence_) + "\n";
    diag += "  TX SeqNo (CRC): " + std::to_string(tx_seq_no_) + "\n";
    diag += "  RX SeqNo (CRC): " + std::to_string(rx_seq_no_) + "\n";
    diag += "  Last TX CRC0: 0x" + std::to_string(last_tx_crc0_) + "\n";
    diag += "  Last RX CRC0: 0x" + std::to_string(last_rx_crc0_) + "\n";
    diag += "  Watchdog: " + std::to_string(status_.watchdog_counter) + " ms\n";
    diag += "  Parameter CRC: " + std::to_string(parameter_crc_) + "\n";

    diag += "\nStatistics:\n";
    diag += "  Frames Sent: " + std::to_string(stats_.frames_sent) + "\n";
    diag += "  Frames Received: " + std::to_string(stats_.frames_received) + "\n";
    diag += "  CRC Errors: " + std::to_string(stats_.crc_errors) + "\n";
    diag += "  Sequence Errors: " + std::to_string(stats_.sequence_errors) + "\n";
    diag += "  Watchdog Events: " + std::to_string(stats_.watchdog_events) + "\n";
    diag += "  Reset Events: " + std::to_string(stats_.reset_events) + "\n";
    diag += "  Invalid Frames: " + std::to_string(stats_.invalid_frames) + "\n";
    diag += "  Duplicate Frames: " + std::to_string(stats_.duplicate_frames) + "\n";
    diag += "  Recovery Attempts: " + std::to_string(stats_.recovery_attempts) + "\n";
    diag += "  Successful Recoveries: " + std::to_string(stats_.successful_recoveries) + "\n";

    return diag;
}

// ============================================================================
// Callbacks
// ============================================================================

void FSoEMasterConnection::setStateChangeCallback(StateChangeCallback callback)
{
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    state_change_callback_ = std::move(callback);
}

void FSoEMasterConnection::setErrorCallback(ErrorCallback callback)
{
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    error_callback_ = std::move(callback);
}

void FSoEMasterConnection::setFailSafeCallback(FailSafeCallback callback)
{
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    fail_safe_callback_ = std::move(callback);
}

void FSoEMasterConnection::setDataCallback(DataCallback callback)
{
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    data_callback_ = std::move(callback);
}

void FSoEMasterConnection::setTraceCallback(TraceCallback callback)
{
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    trace_callback_ = std::move(callback);
}

// ============================================================================
// Frame Event Sources
// ============================================================================

FrameEventSource& FSoEMasterConnection::txFrameEvents()
{
    return tx_frame_events_;
}

FrameEventSource& FSoEMasterConnection::rxFrameEvents()
{
    return rx_frame_events_;
}

} // namespace FSoE

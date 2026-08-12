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

    // ETG.5100 §8.2.2.4: Connection ID 0x0000 is not permitted.
    if (config_.connection_id == 0) {
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
    tx_seq_no_ = config_.initial_seq_no;
    rx_seq_no_ = config_.initial_seq_no;
    last_tx_seq_no_ = 0;
    last_rx_seq_no_ = 0;
    current_param_index_ = 0;
    parameter_crc_ = 0;
    fail_safe_entered_ms_ = 0;
    pdo_tx_count_ = 0;
    last_rx_frame_.clear();
    baseline_rx_.clear();
    expecting_rx_change_ = false;
    stale_rx_count_ = 0;
    cached_tx_pdo_.clear();
    cached_tx_pdo_state_ = 0xFF;
    tx_cache_dirty_ = true;
    consecutive_seq_fallback_ = 0;
    seq_fallback_disabled_ = false;

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
    status_.slave_session_id = 0;
    session_octet_idx_ = 0;
    connection_tx_idx_ = 0;
    connection_rx_idx_ = 0;
    memset(connection_tx_buf_, 0, sizeof(connection_tx_buf_));
    memset(connection_rx_buf_, 0, sizeof(connection_rx_buf_));
    rx_sequence_ = 0x0F;  // Wrap so first expected RX sequence is 0
    tx_sequence_ = 0;
    last_tx_crc0_ = 0;   // CRC inheritance starts at 0
    last_rx_crc0_ = 0;
    tx_seq_no_ = config_.initial_seq_no;
    rx_seq_no_ = config_.initial_seq_no;
    last_tx_seq_no_ = 0;
    last_rx_seq_no_ = 0;
    current_param_index_ = 0;
    parameter_crc_ = 0;
    param_tx_idx_ = 0;
    param_rx_idx_ = 0;
    param_tx_buf_.clear();
    param_rx_buf_.clear();
    pdo_tx_count_ = 0;
    last_rx_frame_.clear();
    baseline_rx_.clear();
    expecting_rx_change_ = false;
    stale_rx_count_ = 0;
    cached_tx_pdo_.clear();
    cached_tx_pdo_state_ = 0xFF;
    tx_cache_dirty_ = true;
    consecutive_seq_fallback_ = 0;
    seq_fallback_disabled_ = false;
    stats_.reset_events++;

    return true;
}

bool FSoEMasterConnection::requestSessionReset()
{
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    if (!initialized_) return false;

    // Generate non-predictable session ID
    // ETG.5100 §8.2.2.3: the master generates a random Session ID once
    // per connection attempt.  Must be non-zero (0 is not a valid ID).
    status_.session_id = static_cast<uint16_t>(rng_() & 0xFFFF);
    if (status_.session_id == 0) {
        status_.session_id = 1;
    }
    session_octet_idx_ = 0;

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

    // ETG.5100 S (D) V1.2.0, §8.2.2.6: FailSafeData is a command used
    // within the Data state, NOT a separate state.  The master stays in
    // Data state and uses the fail_safe_active flag to select the
    // FailSafeData command (all-zero SafeData) instead of ProcessData.
    // If triggerFailSafe is called from a non-Data state (e.g. an error
    // during handshake), transition to Data so the FailSafeData command
    // can be sent.  This matches the slave's behavior.
    // See: https://techoverflow.net/2026/08/12/fsoe-data-pdu-master-and-slave-structure/
    if (status_.state != ConnectionState::Data) {
        transitionTo(ConnectionState::Data);
    }
    fail_safe_entered_ms_ = current_time_ms_;

    if (!was_fail_safe && fail_safe_callback_) {
        fail_safe_callback_();
    }
}

bool FSoEMasterConnection::clearError()
{
    std::lock_guard<std::recursive_mutex> lock(mutex_);

    // Fail-safe is now a flag within the Data state, not a separate state.
    // Allow clearing from Error state or from Data state with fail_safe_active.
    if (status_.state != ConnectionState::Error &&
        !(status_.state == ConnectionState::Data && status_.fail_safe_active)) {
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

    // Store the raw frame for duplicate detection.
    // last_rx_frame_ is cleared in prepareTxFrame() so that a frame
    // received after the master sent a new frame is never considered a
    // duplicate — it's a fresh response, even if the bytes happen to be
    // identical (e.g. with 0-byte safe data where CRC doesn't change).
    //
    // Duplicate detection: if the slave re-sends the exact same frame
    // bytes WITHOUT the master having sent a new frame in between, skip
    // re-processing.  This happens in the PDO path when the slave hasn't
    // seen the master's new frame yet and is repeating its last response.
    //
    // Only applied during handshake states (Session, Connection,
    // Parameter).  In Data state, duplicate detection is handled in
    // exchangeViaPDO (not processRxFrame) to avoid interfering with
    // the direct exchange path (exchangeWith), where duplicates in
    // Data state are expected to be processed.
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

    // Parse and validate frame (CRC verification happens inside
    // parseFSoEFrameWithCollisionAvoidance, which also replicates the
    // ETG.5100 §8.1.3.4 collision avoidance algorithm on the checking side).
    // Buffer must accommodate MAX_PARSE_DATA_SIZE bytes because the slave's
    // buildFailSafeResponse sends safeInputSize + 2 bytes (inputs + error code),
    // which can be up to 18 bytes when safeInputSize = 16.
    //
    // CRC inheritance for RX parsing (self-inheriting):
    //   - Reset frames: start_crc=0, seq=initial_seq_no+1 (Reset resets chain)
    //   - Non-Reset frames: start_crc=last_tx_crc0_, seq=last_tx_seq_no_
    //     (the slave's TX inherits from the master's last TX CRC0 and seq,
    //      so the master verifies using its own last TX CRC0 and seq)
    uint8_t cmd = 0;
    uint8_t frame_data[CRC::MAX_PARSE_DATA_SIZE] = {0};
    size_t data_len = 0;
    uint16_t conn_id = 0;
    CRC::CrcErrorDetail crc_error_detail{};

    const bool is_reset_frame = (data[0] == Command::Reset);
    const uint16_t parse_start_crc = is_reset_frame ? 0 : last_tx_crc0_;
    // The slave's Reset response uses seq = initial_seq_no + 1.
    // Non-Reset frames use tx_seq_no_ (the master's NEXT TX seq, which was
    // incremented after the master's last TX).  The slave increments its
    // rx_seq_no_ after receiving the master's TX, and uses that incremented
    // value for its TX response.  So the master's RX must use tx_seq_no_
    // (which was also incremented after TX) to match.
    const uint16_t parse_seq_no = is_reset_frame
        ? CRC::incrementSeqNo(config_.initial_seq_no)
        : tx_seq_no_;
    uint16_t seq_used = 0;

    // CRC trace tracking (for --debug fsoe-crc)
    bool crc_trace_fb_used = false;
    int  crc_trace_fb_delta = 0;
    bool crc_trace_ok = false;
    uint16_t crc_trace_crc0 = 0;

    if (!CRC::parseFSoEFrameWithCollisionAvoidance(
            data, len, cmd, frame_data, data_len, conn_id,
            parse_start_crc, parse_seq_no,
            is_reset_frame ? nullptr : &last_rx_crc0_,
            &seq_used, &crc_error_detail)) {
        // --- Diagnostic seq±1 fallback ---
        // The CRC didn't verify with the expected seq.  As a diagnostic
        // aid, try seq-1 and seq+1 to detect off-by-one seq
        // synchronization issues (common during interoperability
        // debugging with real slaves).  This is rate-limited: if the
        // fallback rescues frames more than kMaxConsecutiveSeqFallback
        // times in a row, it is disabled (the seq is permanently off,
        // indicating a real bug, not a transient glitch).  The counter
        // resets whenever a frame verifies with the expected seq.
        if (!is_reset_frame && !seq_fallback_disabled_) {
            const uint16_t seq_minus = CRC::decrementSeqNo(parse_seq_no);
            const uint16_t seq_plus = CRC::incrementSeqNo(parse_seq_no);
            uint16_t fb_seq_used = 0;
            CRC::CrcErrorDetail fb_error{};
            const uint16_t fb_crc0_save = last_rx_crc0_;

            // Try seq-1
            if (CRC::parseFSoEFrameWithCollisionAvoidance(
                    data, len, cmd, frame_data, data_len, conn_id,
                    parse_start_crc, seq_minus,
                    &last_rx_crc0_, &fb_seq_used, &fb_error)) {
                consecutive_seq_fallback_++;
                seq_used = fb_seq_used;
                crc_trace_fb_used = true;
                crc_trace_fb_delta = -1;
                crc_trace_ok = true;
                crc_trace_crc0 = last_rx_crc0_;
                trace("RX CRC: seq±1 fallback SUCCEEDED with seq-1 "
                      "(expected=%u, used=%u, consecutive=%u/%u)",
                      parse_seq_no, seq_minus,
                      consecutive_seq_fallback_, kMaxConsecutiveSeqFallback);
                if (consecutive_seq_fallback_ >= kMaxConsecutiveSeqFallback) {
                    seq_fallback_disabled_ = true;
                    trace("RX CRC: seq±1 fallback DISABLED after %u "
                          "consecutive rescues — seq is permanently off",
                          consecutive_seq_fallback_);
                }
                // Fall through to normal processing with the fallback result.
                // Skip the "expected seq" reset below since this was a fallback.
                goto crc_fallback_succeeded;
            }
            last_rx_crc0_ = fb_crc0_save;  // restore on failure

            // Try seq+1
            if (CRC::parseFSoEFrameWithCollisionAvoidance(
                    data, len, cmd, frame_data, data_len, conn_id,
                    parse_start_crc, seq_plus,
                    &last_rx_crc0_, &fb_seq_used, &fb_error)) {
                consecutive_seq_fallback_++;
                seq_used = fb_seq_used;
                crc_trace_fb_used = true;
                crc_trace_fb_delta = +1;
                crc_trace_ok = true;
                crc_trace_crc0 = last_rx_crc0_;
                trace("RX CRC: seq±1 fallback SUCCEEDED with seq+1 "
                      "(expected=%u, used=%u, consecutive=%u/%u)",
                      parse_seq_no, seq_plus,
                      consecutive_seq_fallback_, kMaxConsecutiveSeqFallback);
                if (consecutive_seq_fallback_ >= kMaxConsecutiveSeqFallback) {
                    seq_fallback_disabled_ = true;
                    trace("RX CRC: seq±1 fallback DISABLED after %u "
                          "consecutive rescues — seq is permanently off",
                          consecutive_seq_fallback_);
                }
                goto crc_fallback_succeeded;
            }
            last_rx_crc0_ = fb_crc0_save;  // restore on failure
        }

        // No fallback worked (or fallback disabled) — report the error.
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

        // Emit CRC trace (RX failure): show what parameters were tried.
        if (crc_trace_callback_) {
            CrcTraceInfo info{};
            info.direction = CrcTraceInfo::Direction::RX;
            info.command = (len > 0) ? data[0] : 0;
            info.state = status_.state;
            info.start_crc = parse_start_crc;
            info.seq_expected = parse_seq_no;
            info.seq_used = 0;  // no seq matched
            info.crc0 = 0;
            info.crc_ok = false;
            info.fallback_used = false;
            info.fallback_delta = 0;
            info.conn_id = conn_id;
            info.data_len = data_len;
            crc_trace_callback_(info);
        }

        handleError(ErrorCode::CRCError, detail);
        return false;
    }

    // Frame verified with the expected seq — reset the fallback counter.
    consecutive_seq_fallback_ = 0;
    crc_trace_ok = true;
    crc_trace_crc0 = last_rx_crc0_;

crc_fallback_succeeded:

    // Emit CRC trace (RX success): show what parameters verified the frame.
    if (crc_trace_callback_) {
        CrcTraceInfo info{};
        info.direction = CrcTraceInfo::Direction::RX;
        info.command = cmd;
        info.state = status_.state;
        info.start_crc = parse_start_crc;
        info.seq_expected = parse_seq_no;
        info.seq_used = seq_used;
        info.crc0 = crc_trace_crc0;
        info.crc_ok = crc_trace_ok;
        info.fallback_used = crc_trace_fb_used;
        info.fallback_delta = crc_trace_fb_delta;
        info.conn_id = conn_id;
        info.data_len = data_len;
        crc_trace_callback_(info);
    }

    // Save the seq that matched (after collision avoidance).
    // The slave echoes the master's last TX seq, so rx_seq_no_ = seq_used
    // (no increment — the slave doesn't advance seq independently).
    last_rx_seq_no_ = seq_used;
    rx_seq_no_ = seq_used;  // for diagnostics

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

    // Validate connection ID.
    //
    // The FSoE frame is always full PDO size with ConnID at the last 2 bytes.
    // In Reset/Session states, the slave may send ConnID=0 (it hasn't learned
    // the ConnID yet).  We skip validation in Reset and Session states,
    // matching the FSoE slave behavior (which also only validates ConnID in
    // Connection/Parameter/Data states).
    //
    // Reset command frames (0x2A) ALWAYS have conn_id=0x0000 — the slave
    // resets its conn_id when it receives a Reset.  Skip validation for
    // Reset frames in any state, so the master can handle a slave-initiated
    // reset from Connection/Parameter/Data states without rejecting it as
    // a ConnectionIDError.
    if (cmd != Command::Reset &&
        (status_.state == ConnectionState::Connection ||
         status_.state == ConnectionState::Parameter ||
         status_.state == ConnectionState::Data ||
         status_.state == ConnectionState::FailSafe)) {
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

    // Log detailed frame evaluation for debugging.
    trace("RX eval: cmd=%s(0x%02X) data_len=%zu conn_id=0x%04X "
          "state=%s expected_cmd=%s",
          commandName(cmd), cmd, data_len, conn_id,
          stateName(status_.state),
          status_.state == ConnectionState::Reset ? "Session/Reset" :
          status_.state == ConnectionState::Session ? "Session" :
          status_.state == ConnectionState::Connection ? "Connection" :
          status_.state == ConnectionState::Parameter ? "Parameter" :
          status_.state == ConnectionState::Data ? "ProcessData" :
          status_.state == ConnectionState::FailSafe ? "FailSafeData" :
          "Reset");

    // Update watchdog timestamp
    status_.last_valid_frame_ms = current_time_ms_;

    // RX sequence number was already updated above (after collision-aware
    // parse).  No additional increment needed here.

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

    // Clear last_rx_frame_ so that the next received frame is not considered
    // a duplicate — the master is sending a new frame, so any response (even
    // if byte-identical) is a fresh response to this new frame.
    last_rx_frame_.clear();

    // In Reset state, send a Reset command (0x2A) to force the slave back
    // to its initial state before starting the Session handshake.  The
    // master stays in Reset until it receives the slave's Session response
    // (handled in handleResetState), at which point it transitions to
    // Session and begins the handshake with a fresh session ID.
    //
    // Save CRC input parameters before the build function updates them,
    // so we can emit a CRC trace with both input and output values.
    const uint16_t tx_start_crc_before = last_tx_crc0_;
    const uint16_t tx_seq_before = tx_seq_no_;
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
                trace("TX Parameter(0x52): watchdog=%u ms app_param_len=%zu "
                      "tx_idx=%u/%zu (state=Parameter, %zu bytes)",
                      config_.watchdog_timeout_ms,
                      config_.app_parameters.size(),
                      param_tx_idx_, param_tx_buf_.size(), len);
            }
            break;

        case ConnectionState::Data:
            // ETG.5100 S (D) V1.2.0, §8.2.2.6: In the Data state, the command
            // (ProcessData vs FailSafeData) is chosen independently based on
            // local circumstances.  If fail_safe_active is true, send
            // FailSafeData (all-zero SafeData); otherwise send ProcessData
            // with the current SafeOutputs.
            // See: https://techoverflow.net/2026/08/12/fsoe-data-pdu-master-and-slave-structure/
            if (status_.fail_safe_active) {
                len = buildFailSafeFrame(data, max_len);
                if (len > 0) {
                    trace("TX FailSafeData(0x08): all-zero safe data "
                          "(state=Data, fail-safe, %zu bytes)", len);
                }
            } else {
                len = buildDataFrame(data, max_len);
                if (len > 0) {
                    trace("TX ProcessData(0x36): %u bytes of safe outputs "
                          "(state=Data, %zu bytes)",
                          config_.output_size, len);
                }
            }
            break;

        case ConnectionState::FailSafe:
            // Dead code: the master no longer transitions to FailSafe state.
            // FailSafeData is sent from the Data state when fail_safe_active
            // is set.  This case is kept for backwards compatibility.
            len = buildFailSafeFrame(data, max_len);
            if (len > 0) {
                trace("TX FailSafeData(0x08): all-zero safe data "
                      "(state=FailSafe [legacy], %zu bytes)", len);
            }
            break;

        case ConnectionState::Error:
            // No frames in Error state
            break;
    }

    if (len > 0) {
        stats_.frames_sent++;
        // The build* function already set tx_seq_no_ = seq_used and
        // last_tx_crc0_ = new CRC0.  Save seq_used before incrementing.
        const uint16_t tx_seq_used = tx_seq_no_;
        const uint16_t tx_crc0_result = last_tx_crc0_;

        // Increment TX sequence number for the NEXT frame (self-inheriting
        // TX: the master's next TX inherits from this TX's CRC0 and uses
        // seq+1).  The actual seq used for this frame (after collision
        // avoidance) was already set by the build* function; here we
        // advance to the next expected value.
        // NOTE: In the PDO path, the TX cache prevents prepareTxFrame from
        // being called every cycle, so the increment only happens once per
        // state transition.  In the direct exchangeWith path, it increments
        // per frame as expected.
        tx_seq_no_ = CRC::incrementSeqNo(tx_seq_no_);

        // Emit CRC trace (TX): show the parameters used to build this frame.
        if (crc_trace_callback_) {
            CrcTraceInfo info{};
            info.direction = CrcTraceInfo::Direction::TX;
            info.command = (len > 0) ? data[0] : 0;
            info.state = status_.state;
            info.start_crc = tx_start_crc_before;
            info.seq_expected = tx_seq_before;
            info.seq_used = tx_seq_used;
            info.crc0 = tx_crc0_result;
            info.crc_ok = true;
            info.fallback_used = false;
            info.fallback_delta = 0;
            info.conn_id = config_.connection_id;
            info.data_len = config_.output_size;
            crc_trace_callback_(info);
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

    // Auto-recovery attempt — fail-safe is now a flag within the Data state,
    // not a separate state (ETG.5100 §8.2.2.6).
    if (status_.state == ConnectionState::Data &&
        status_.fail_safe_active && config_.auto_recovery_enabled) {
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
                // Dump the last TxPDO content for debugging.
                if (!last_txpdo_.empty()) {
                    char hex[256];
                    size_t pos = 0;
                    for (size_t b = 0; b < last_txpdo_.size() && pos + 3 < sizeof(hex); b++) {
                        pos += static_cast<size_t>(snprintf(hex + pos, sizeof(hex) - pos, "%02X ", last_txpdo_[b]));
                    }
                    trace("  last TxPDO (%zu bytes): %s", last_txpdo_.size(), hex);
                    const uint8_t last_cmd = last_txpdo_[0];
                    trace("  last TxPDO cmd=%s(0x%02X) — slave was %s",
                          commandName(last_cmd), last_cmd,
                          isValidCommand(last_cmd) ? "sending valid FSoE frames" : "NOT sending FSoE frames");
                } else {
                    trace("  no TxPDO received from slave (last_txpdo_ is empty)");
                }
                requestSessionReset();
            }
        }
        return;
    }

    if (status_.state == ConnectionState::Data ||
        status_.state == ConnectionState::Error) {
        // Data state runs indefinitely (no phase timeout).
        // Fail-safe is now a flag within Data state, not a separate state.
        return;
    }

    uint64_t elapsed = current_time_ms - status_.state_entered_ms;
    uint16_t timeout = config_.conn_timeout_ms;

    if (status_.state == ConnectionState::Session) {
        timeout = config_.session_timeout_ms;
    }

    if (elapsed > timeout) {
        stats_.timeout_events++;
        // Dump the last TxPDO content for debugging.
        if (!last_txpdo_.empty()) {
            char hex[256];
            size_t pos = 0;
            for (size_t b = 0; b < last_txpdo_.size() && pos + 3 < sizeof(hex); b++) {
                pos += static_cast<size_t>(snprintf(hex + pos, sizeof(hex) - pos, "%02X ", last_txpdo_[b]));
            }
            trace("%s state timeout after %llu ms — last TxPDO (%zu bytes): %s",
                  stateName(status_.state),
                  static_cast<unsigned long long>(elapsed),
                  last_txpdo_.size(), hex);
            const uint8_t last_cmd = last_txpdo_[0];
            trace("  last TxPDO cmd=%s(0x%02X) — slave was %s",
                  commandName(last_cmd), last_cmd,
                  isValidCommand(last_cmd) ? "sending valid FSoE frames" : "NOT sending FSoE frames");
        } else {
            trace("%s state timeout after %llu ms — no TxPDO received from slave",
                  stateName(status_.state),
                  static_cast<unsigned long long>(elapsed));
        }
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
        // CRC chain is already at 0 — Reset frames don't update it.
        requestSessionReset();
    } else if (cmd == Command::FailSafeData) {
        // Slave is aborting by sending FailSafeData.
        trace("RX FailSafeData(0x08): slave aborting in Reset state");
        FSoEErrorDetail detail;
        snprintf(detail.message, sizeof(detail.message),
                 "Slave sent FailSafeData in Reset state (handshake abort)");
        handleError(ErrorCode::ApplicationError, detail);
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
    // ETG.5100 S (D) V1.2.0, §8.2.2.3:
    // The slave responds with its OWN Slave Session ID in SafeData[0..1].
    // The master stores this for connection instance identification.
    // See: https://techoverflow.net/2026/08/12/fsoe-session-pdu-master-and-slave-structure/
    if (cmd == Command::Reset) {
        trace("RX Reset(0x2A): slave requested reset in Session state, resetting connection");
        resetConnection();
        return;
    }
    if (cmd == Command::Session) {
        // Extract the Slave Session ID from the response.
        // For safety data >= 2 octets, both bytes are in SafeData[0..1].
        // For 1-octet safety data, only one byte is present per cycle
        // (low byte on the first response, high byte on the second).
        // For 0-octet safety data, no Session ID can be transferred —
        // the frame is just [CMD][ConnID] with no data payload.  The
        // master transitions to Connection immediately (the Session ID
        // exchange is skipped, which is acceptable since the Session ID
        // has no safety relevance per ETG.5100 §8.2.2.3).
        if (config_.input_size == 0) {
            trace("RX Session(0x4E): 0-octet safety data, moving to Connection");
            session_octet_idx_ = 0;
            initConnectionTxBuf();
            transitionTo(ConnectionState::Connection);
        } else if (config_.input_size >= 2) {
            if (data && data_len >= 2) {
                status_.slave_session_id =
                    static_cast<uint16_t>(data[0]) |
                    (static_cast<uint16_t>(data[1]) << 8);
            }
            trace("RX Session(0x4E): slave session_id=0x%04X, moving to Connection",
                  status_.slave_session_id);
            session_octet_idx_ = 0;
            initConnectionTxBuf();
            transitionTo(ConnectionState::Connection);
        } else {
            // 1-octet safety data: two cycles needed for the full 16-bit ID.
            // ETG.5100 §8.2.2.3: the 16-bit Session ID is transferred in
            // two successive PDUs (low byte first, then high byte).
            // See: https://techoverflow.net/2026/08/12/fsoe-session-pdu-master-and-slave-structure/
            if (data && data_len >= 1) {
                if (session_octet_idx_ == 0) {
                    status_.slave_session_id =
                        static_cast<uint16_t>(data[0]);
                    session_octet_idx_ = 1;
                    trace("RX Session(0x4E): slave session_id low byte=0x%02X, "
                          "waiting for high byte", data[0]);
                    // Stay in Session state — need the high byte next.
                    // Invalidate TX cache so the master builds a new frame
                    // with advanced CRCs.  This ensures the slave sees a
                    // new (non-duplicate) frame and advances its own
                    // sessionOctetIdx_ to send the high byte.
                    tx_cache_dirty_ = true;
                } else {
                    status_.slave_session_id |=
                        (static_cast<uint16_t>(data[0]) << 8);
                    trace("RX Session(0x4E): slave session_id=0x%04X, "
                          "moving to Connection", status_.slave_session_id);
                    session_octet_idx_ = 0;
                    initConnectionTxBuf();
                    transitionTo(ConnectionState::Connection);
                }
            }
        }
    } else if (cmd == Command::FailSafeData) {
        // Slave is aborting the handshake by sending FailSafeData.
        // ETG.5100 §8.2.2.6: FailSafeData is normally a Data-state command,
        // but receiving it during the handshake means the slave detected an
        // error.  Enter fail-safe to acknowledge the abort.
        trace("RX FailSafeData(0x08): slave aborting in Session state");
        FSoEErrorDetail detail;
        snprintf(detail.message, sizeof(detail.message),
                 "Slave sent FailSafeData in Session state (handshake abort)");
        handleError(ErrorCode::ApplicationError, detail);
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
    // ETG.5100 S (D) V1.2.0, §8.2.2.4:
    // The slave echoes back the Connection ID and FSoE Slave Address.
    // When safety data < 4 octets, the echo arrives in multiple cycles.
    // See: https://techoverflow.net/2026/08/12/fsoe-connection-pdu-master-and-slave-structure/
    if (cmd == Command::Reset) {
        trace("RX Reset(0x2A): slave requested reset in Connection state, resetting connection");
        resetConnection();
        return;
    }
    if (cmd == Command::Connection) {
        // 0-octet safety data: no SafeData to transfer, rely on Conn_Id field.
        // Transition immediately.
        if (config_.input_size == 0) {
            trace("RX Connection(0x64): 0-octet data, moving to %s",
                  (config_.input_size > 0 || config_.output_size > 0)
                      ? "Parameter" : "Data");
            if (config_.input_size > 0 || config_.output_size > 0) {
                transitionTo(ConnectionState::Parameter);
            } else {
                transitionTo(ConnectionState::Data);
            }
            return;
        }

        // Advance TX index FIRST, so RX progress can be tied to it.
        // The slave has already received this cycle's TX bytes and echoed
        // them, so the echo reflects the master's TX progress AFTER this
        // cycle's advance.
        const uint8_t tx_chunk = std::min(static_cast<uint8_t>(4),
                                           config_.output_size);
        connection_tx_idx_ = std::min(static_cast<uint8_t>(connection_tx_idx_ + tx_chunk),
                                       static_cast<uint8_t>(4));

        // Accumulate echo bytes into connection_rx_buf_.
        const uint8_t rx_chunk = std::min(static_cast<uint8_t>(4),
                                           config_.input_size);
        // Reject frames with insufficient data length.
        if (data_len < rx_chunk) {
            FSoEErrorDetail detail;
            snprintf(detail.message, sizeof(detail.message),
                     "Connection response too short: got %zu bytes, expected %u",
                     data_len, rx_chunk);
            handleError(ErrorCode::DataLengthError, detail);
            return;
        }
        if (config_.input_size >= 4) {
            // Slave sends all 4 bytes each cycle from offset 0.
            // Only copy if we haven't received all 4 bytes yet (the slave
            // may send zero-padded frames after all bytes are echoed).
            // RX progress is tied to TX progress: the slave can only echo
            // bytes it has received, which equals the master's TX progress.
            if (connection_rx_idx_ < 4) {
                for (uint8_t i = 0; i < 4 && i < data_len; ++i) {
                    connection_rx_buf_[i] = data[i];
                }
                connection_rx_idx_ = std::min(connection_tx_idx_,
                                               static_cast<uint8_t>(4));
            }
        } else {
            // Slave sends input_size bytes per cycle from its TX offset.
            // Accumulate at connection_rx_idx_, limited by TX progress
            // (the slave can only echo bytes it has received from the master).
            if (connection_rx_idx_ < 4) {
                const uint8_t rx_off = std::min(connection_rx_idx_,
                                                 static_cast<uint8_t>(4));
                // Valid echo bytes: limited by TX progress and rx_chunk.
                const uint8_t echo_avail = (connection_tx_idx_ > connection_rx_idx_)
                    ? static_cast<uint8_t>(connection_tx_idx_ - connection_rx_idx_) : 0;
                const uint8_t valid = std::min(static_cast<uint8_t>(4 - rx_off),
                                                std::min(rx_chunk, echo_avail));
                for (uint8_t i = 0; i < valid && i < data_len; ++i) {
                    connection_rx_buf_[rx_off + i] = data[i];
                }
                connection_rx_idx_ = std::min(
                    static_cast<uint8_t>(connection_rx_idx_ + valid),
                    static_cast<uint8_t>(4));
            }
        }

        // Check if all 4 bytes have been transferred AND echoed.
        if (connection_tx_idx_ >= 4 && connection_rx_idx_ >= 4) {
            // Validate the accumulated echo.
            uint16_t echo_conn_id = static_cast<uint16_t>(connection_rx_buf_[0]) |
                (static_cast<uint16_t>(connection_rx_buf_[1]) << 8);
            uint16_t echo_addr = static_cast<uint16_t>(connection_rx_buf_[2]) |
                (static_cast<uint16_t>(connection_rx_buf_[3]) << 8);

            // ETG.5100 §8.2.2.4: Connection ID 0x0000 is not permitted.
            if (echo_conn_id != config_.connection_id) {
                trace("RX Connection(0x64): Connection ID mismatch "
                      "(expected 0x%04X got 0x%04X)",
                      config_.connection_id, echo_conn_id);
                FSoEErrorDetail detail;
                detail.conn_id_valid = true;
                detail.expected_conn_id = config_.connection_id;
                detail.received_conn_id = echo_conn_id;
                snprintf(detail.message, sizeof(detail.message),
                         "Slave Connection ID mismatch: expected 0x%04X got 0x%04X",
                         detail.expected_conn_id, detail.received_conn_id);
                handleError(ErrorCode::ConnectionIDError, detail);
                return;
            }
            if (echo_addr != config_.slave_safety_addr) {
                trace("RX Connection(0x64): slave address mismatch "
                      "(expected 0x%04X got 0x%04X)",
                      config_.slave_safety_addr, echo_addr);
                FSoEErrorDetail detail;
                snprintf(detail.message, sizeof(detail.message),
                         "Slave safety address mismatch: expected 0x%04X got 0x%04X",
                         config_.slave_safety_addr, echo_addr);
                handleError(ErrorCode::ConnectionIDError, detail);
                return;
            }

            trace("RX Connection(0x64): slave confirmed conn_id=0x%04X "
                  "addr=0x%04X, moving to %s",
                  echo_conn_id, echo_addr,
                  (config_.input_size > 0 || config_.output_size > 0)
                      ? "Parameter" : "Data");
            if (config_.input_size > 0 || config_.output_size > 0) {
                initParameterTxBuf();
                transitionTo(ConnectionState::Parameter);
            } else {
                transitionTo(ConnectionState::Data);
            }
        } else {
            // More cycles needed — invalidate TX cache to send next chunk.
            trace("RX Connection(0x64): multi-cycle transfer in progress "
                  "(tx_idx=%u/%u rx_idx=%u/%u)",
                  connection_tx_idx_, 4, connection_rx_idx_, 4);
            tx_cache_dirty_ = true;
        }
    } else if (cmd == Command::FailSafeData) {
        // Slave is aborting the handshake by sending FailSafeData.
        trace("RX FailSafeData(0x08): slave aborting in Connection state");
        FSoEErrorDetail detail;
        snprintf(detail.message, sizeof(detail.message),
                 "Slave sent FailSafeData in Connection state (handshake abort)");
        handleError(ErrorCode::ApplicationError, detail);
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
    // ETG.5100 S (D) V1.2.0, §8.2.2.5:
    // The slave echoes back the parameter SafeData each cycle.
    // The master validates the echo and advances to the next chunk.
    // When all bytes are transferred and echoed, transition to Data.
    // See: https://techoverflow.net/2026/08/12/fsoe-parameter-pdu-master-and-slave-structure/
    if (cmd == Command::Reset) {
        trace("RX Reset(0x2A): slave requested reset in Parameter state, resetting connection");
        resetConnection();
        return;
    }
    if (cmd == Command::Parameter) {
        // 0-octet safety data: no SafeData to transfer.  The parameter
        // payload is empty (or rather, can't be transferred via SafeData).
        // Transition immediately to Data.
        if (config_.input_size == 0) {
            trace("RX Parameter(0x52): 0-octet data, moving to Data");
            transitionTo(ConnectionState::Data);
            return;
        }

        const size_t total_len = param_tx_buf_.size();
        const uint8_t rx_chunk = std::min(static_cast<size_t>(config_.input_size),
                                           static_cast<size_t>(CRC::MAX_PARSE_DATA_SIZE));
        // Reject frames with insufficient data length.
        if (data_len < rx_chunk) {
            FSoEErrorDetail detail;
            snprintf(detail.message, sizeof(detail.message),
                     "Parameter response too short: got %zu bytes, expected %u",
                     data_len, rx_chunk);
            handleError(ErrorCode::DataLengthError, detail);
            return;
        }

        // Advance TX index FIRST (the slave has already received and
        // echoed this cycle's TX bytes).
        const uint8_t tx_chunk = std::min(static_cast<size_t>(config_.output_size),
                                           static_cast<size_t>(CRC::MAX_PARSE_DATA_SIZE));
        param_tx_idx_ = std::min(static_cast<size_t>(param_tx_idx_) + tx_chunk,
                                  total_len);

        // Accumulate echo bytes.
        // The slave can only echo bytes it has received, which equals
        // the master's TX progress (param_tx_idx_).  So the master's
        // RX progress cannot exceed its TX progress.
        if (param_rx_idx_ < total_len) {
            const size_t rx_off = std::min(static_cast<size_t>(param_rx_idx_),
                                            total_len);
            // Valid echo bytes: limited by TX progress and rx_chunk.
            const size_t echo_avail = (param_tx_idx_ > param_rx_idx_)
                ? static_cast<size_t>(param_tx_idx_ - param_rx_idx_) : 0;
            const size_t valid = std::min(total_len - rx_off,
                                           std::min(static_cast<size_t>(rx_chunk),
                                                    echo_avail));
            for (size_t i = 0; i < valid && i < data_len; ++i) {
                param_rx_buf_[rx_off + i] = data[i];
            }
            param_rx_idx_ = std::min(static_cast<size_t>(param_rx_idx_) + valid,
                                      total_len);
        }

        // Check if all bytes have been transferred AND echoed.
        if (param_tx_idx_ >= total_len && param_rx_idx_ >= total_len) {
            // Validate the accumulated echo against what we sent.
            for (size_t i = 0; i < total_len; ++i) {
                if (param_rx_buf_[i] != param_tx_buf_[i]) {
                    trace("RX Parameter(0x52): echo mismatch at byte %zu "
                          "(expected 0x%02X got 0x%02X)",
                          i, param_tx_buf_[i], param_rx_buf_[i]);
                    FSoEErrorDetail detail;
                    snprintf(detail.message, sizeof(detail.message),
                             "Parameter echo mismatch at byte %zu: "
                             "expected 0x%02X got 0x%02X",
                             i, param_tx_buf_[i], param_rx_buf_[i]);
                    handleError(ErrorCode::ParameterError, detail);
                    return;
                }
            }
            trace("RX Parameter(0x52): all %zu bytes echoed correctly, "
                  "moving to Data", total_len);
            transitionTo(ConnectionState::Data);
        } else {
            // More cycles needed — invalidate TX cache to send next chunk.
            trace("RX Parameter(0x52): multi-cycle transfer in progress "
                  "(tx_idx=%u/%zu rx_idx=%u/%zu)",
                  param_tx_idx_, total_len, param_rx_idx_, total_len);
            tx_cache_dirty_ = true;
        }
    } else if (cmd == Command::ProcessData) {
        trace("RX ProcessData(0x36): slave skipped Parameter phase, moving to Data");
        transitionTo(ConnectionState::Data);
        handleDataState(cmd, data, data_len);
    } else if (cmd == Command::FailSafeData) {
        // Slave is aborting the handshake by sending FailSafeData.
        trace("RX FailSafeData(0x08): slave aborting in Parameter state");
        FSoEErrorDetail detail;
        snprintf(detail.message, sizeof(detail.message),
                 "Slave sent FailSafeData in Parameter state (handshake abort)");
        handleError(ErrorCode::ApplicationError, detail);
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
    // ETG.5100 S (D) V1.2.0, §8.2.2.6: The Data state uses two commands:
    // ProcessData (valid data) and FailSafeData (safe state).  The choice
    // between them is INDEPENDENT in each direction — the master may send
    // ProcessData while the slave sends FailSafeData, or vice versa.
    // The master must accept FailSafeData from the slave without entering
    // fail-safe itself (the decision is purely local).
    // See: https://techoverflow.net/2026/08/12/fsoe-data-pdu-master-and-slave-structure/

    if (cmd == Command::Reset) {
        trace("RX Reset(0x2A): slave requested reset in Data state, resetting connection");
        resetConnection();
        return;
    }

    if (cmd == Command::FailSafeData) {
        // Slave is sending FailSafeData — its SafeInputs are all zeros
        // (ETG.5100 Table 26).  The master accepts this without entering
        // fail-safe itself; the choice between ProcessData and FailSafeData
        // is independent in each direction.
        //
        // Per spec, the FailSafeData PDU has the SAME structure as ProcessData
        // (no error code field).  All SafeData octets are 0.
        if (data_len < config_.input_size) {
            stats_.invalid_frames++;
            trace("RX FailSafeData(0x08): frame too short (%zu bytes, expected %u)",
                  data_len, config_.input_size);
            FSoEErrorDetail detail;
            snprintf(detail.message, sizeof(detail.message),
                     "FailSafeData frame too short: got %zu bytes, expected %u",
                     data_len, config_.input_size);
            handleError(ErrorCode::DataLengthError, detail);
            return;
        }

        // Copy the slave's fail-safe inputs (all zeros per spec) to
        // safe_inputs_ and mark data as not valid.
        std::copy(data, data + config_.input_size, safe_inputs_.begin());
        status_.data_valid = false;
        trace("RX FailSafeData(0x08): slave in fail-safe, %u bytes of "
              "zero safe inputs (state=Data)", config_.input_size);

        if (data_callback_) {
            data_callback_(safe_inputs_.data(), config_.input_size);
        }
        return;
    }

    if (cmd != Command::ProcessData) {
        trace("RX %s: unexpected in Data state (expected ProcessData or FailSafeData)",
              commandName(cmd));
        FSoEErrorDetail detail;
        snprintf(detail.message, sizeof(detail.message),
                 "Unexpected command 0x%02X in Data state (expected ProcessData or FailSafeData)",
                 cmd);
        handleError(ErrorCode::CommandError, detail);
        return;
    }

    // Normal ProcessData command
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
    // Legacy FailSafe state handler — the master no longer transitions to
    // this state (fail-safe is now a flag within the Data state per
    // ETG.5100 §8.2.2.6).  This handler is kept for backwards compatibility.
    // See: https://techoverflow.net/2026/08/12/fsoe-data-pdu-master-and-slave-structure/
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
        // Per ETG.5100 Table 26, FailSafeData has the same structure as
        // ProcessData (all SafeData = 0, no error code field).
        trace("RX FailSafeData(0x08): slave also in fail-safe");
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
    // Reset frame: full PDO-size frame with error code in SafeData[0].
    //
    // The FSoE frame is ALWAYS fixed-length (= fsoeFrameSize(output_size)).
    // ETG.5100 S (D) V1.2.0, §8.2.2.2:
    // Conn_Id is unused and set to 0 in Reset state — the connection has
    // not been established yet, so there is no Connection ID to check.
    // See: https://techoverflow.net/2026/08/12/fsoe-session-pdu-master-and-slave-structure/
    //
    // Reset frames reset the CRC chain AND the sequence number:
    //   - start_crc = 0 (CRC chain reset)
    //   - seq = config_.initial_seq_no (0 for Synapticon, 1 per ETG.5100)
    // last_tx_crc0_ is NOT updated (stays at 0 for the next frame).
    // last_tx_seq_no_ is set for diagnostics.
    uint8_t payload[CRC::MAX_PARSE_DATA_SIZE] = {0};
    payload[0] = ResetErrorCode::None;  // Local reset
    size_t needed = CRC::fsoeFrameSize(config_.output_size);
    if (max_len < needed) return 0;
    uint16_t seq_used = 0;
    size_t result = CRC::buildFSoEFrameWithCollisionAvoidance(
        data, Command::Reset, payload, config_.output_size,
        0,  // Conn_Id = 0 in Reset state (ETG.5100 §8.2.2.2)
        0,  // start_crc = 0 (Reset resets CRC chain)
        config_.initial_seq_no,
        nullptr,  // don't update CRC chain (Reset resets it)
        &seq_used);
    tx_seq_no_ = seq_used;
    last_tx_seq_no_ = seq_used;
    return result;
}

size_t FSoEMasterConnection::buildSessionResetFrame(uint8_t* data, size_t max_len)
{
    // ETG.5100 S (D) V1.2.0, §8.2.2.3, Table 13:
    // Session frame carries the Master Session ID in SafeData[0..1].
    // All other SafeData octets are 0.  Conn_Id is unused and set to 0
    // (the connection has not been established yet).
    // See: https://techoverflow.net/2026/08/12/fsoe-session-pdu-master-and-slave-structure/
    uint8_t payload[CRC::MAX_PARSE_DATA_SIZE] = {0};
    if (config_.output_size >= 2) {
        // Safety data length >= 2: both Session ID octets fit in one PDU.
        payload[0] = static_cast<uint8_t>(status_.session_id & 0xFF);
        payload[1] = static_cast<uint8_t>((status_.session_id >> 8) & 0xFF);
    } else {
        // Safety data length == 1: transfer Session ID in two successive
        // PDUs (low byte first, then high byte).  ETG.5100 §8.2.2.3.
        payload[0] = (session_octet_idx_ == 0)
            ? static_cast<uint8_t>(status_.session_id & 0xFF)
            : static_cast<uint8_t>((status_.session_id >> 8) & 0xFF);
    }
    size_t needed = CRC::fsoeFrameSize(config_.output_size);
    if (max_len < needed) return 0;
    uint16_t seq_used = 0;
    size_t result = CRC::buildFSoEFrameWithCollisionAvoidance(
        data, Command::Session, payload, config_.output_size,
        0,  // Conn_Id = 0 in Session state (ETG.5100 §8.2.2.3)
        last_tx_crc0_, tx_seq_no_, &last_tx_crc0_, &seq_used);
    tx_seq_no_ = seq_used;
    last_tx_seq_no_ = seq_used;
    return result;
}

size_t FSoEMasterConnection::buildConnectionFrame(uint8_t* data, size_t max_len)
{
    // ETG.5100 S (D) V1.2.0, §8.2.2.4, Tables 15-16:
    //   SafeData[0] = Connection ID, low octet
    //   SafeData[1] = Connection ID, high octet
    //   SafeData[2] = FSoE Slave Address, low octet
    //   SafeData[3] = FSoE Slave Address, high octet
    // Conn_Id field = actual Connection ID (no longer 0 as in Reset/Session).
    //
    // Multi-cycle transfer: when output_size < 4, the 4-byte payload is
    // transferred in output_size-sized chunks over multiple cycles.
    //   4 octets → 1 cycle, 2 octets → 2 cycles, 1 octet → 4 cycles
    // See: https://techoverflow.net/2026/08/12/fsoe-connection-pdu-master-and-slave-structure/
    uint8_t payload[CRC::MAX_PARSE_DATA_SIZE] = {0};
    // TX offset: clamped at 4.  Once all bytes are sent, the master
    // sends zero-padded frames while waiting for the slave's echo.
    const uint8_t tx_off = std::min(connection_tx_idx_,
                                     static_cast<uint8_t>(4));
    // Valid bytes: remaining Connection data in this chunk.
    const uint8_t valid = std::min(static_cast<uint8_t>(4 - tx_off),
                                    config_.output_size);
    for (uint8_t i = 0; i < valid; ++i) {
        payload[i] = connection_tx_buf_[tx_off + i];
    }
    // Remaining payload bytes are 0 (padding).
    size_t needed = CRC::fsoeFrameSize(config_.output_size);
    if (max_len < needed) return 0;
    uint16_t seq_used = 0;
    size_t result = CRC::buildFSoEFrameWithCollisionAvoidance(
        data, Command::Connection, payload, config_.output_size,
        config_.connection_id,
        last_tx_crc0_, tx_seq_no_, &last_tx_crc0_, &seq_used);
    tx_seq_no_ = seq_used;
    last_tx_seq_no_ = seq_used;
    return result;
}

size_t FSoEMasterConnection::buildParameterFrame(uint8_t* data, size_t max_len)
{
    // ETG.5100 S (D) V1.2.0, §8.2.2.5, Table 18:
    //   SafeData[0-1]: comm param length (always 2, LE)
    //   SafeData[2-3]: FSoE watchdog (ms, LE)
    //   SafeData[4-5]: app param length (LE)
    //   SafeData[6+]:  app param bytes
    // Total payload = 6 + app_parameters.size().
    // Multi-cycle: transferred in ceil(payload_len / output_size) cycles.
    // Unused octets in the last cycle are set to 0.
    // See: https://techoverflow.net/2026/08/12/fsoe-parameter-pdu-master-and-slave-structure/
    uint8_t payload[CRC::MAX_PARSE_DATA_SIZE] = {0};
    const size_t total_len = param_tx_buf_.size();
    const uint8_t chunk = std::min(static_cast<size_t>(config_.output_size),
                                    static_cast<size_t>(CRC::MAX_PARSE_DATA_SIZE));
    // TX offset: clamped at total_len.  Once all bytes are sent, the
    // master sends zero-padded frames while waiting for the final echo.
    const size_t tx_off = std::min(static_cast<size_t>(param_tx_idx_), total_len);
    const size_t valid = std::min(total_len - tx_off, static_cast<size_t>(chunk));
    for (size_t i = 0; i < valid; ++i) {
        payload[i] = param_tx_buf_[tx_off + i];
    }
    // Remaining payload bytes are 0 (padding).
    size_t needed = CRC::fsoeFrameSize(config_.output_size);
    if (max_len < needed) return 0;
    uint16_t seq_used = 0;
    size_t result = CRC::buildFSoEFrameWithCollisionAvoidance(
        data, Command::Parameter, payload, config_.output_size,
        config_.connection_id,
        last_tx_crc0_, tx_seq_no_, &last_tx_crc0_, &seq_used);
    tx_seq_no_ = seq_used;
    last_tx_seq_no_ = seq_used;
    return result;
}

size_t FSoEMasterConnection::buildDataFrame(uint8_t* data, size_t max_len)
{
    // Data frame: CMD + safe_outputs + ConnID
    size_t needed = CRC::fsoeFrameSize(config_.output_size);
    if (max_len < needed) return 0;
    uint16_t seq_used = 0;
    size_t result = CRC::buildFSoEFrameWithCollisionAvoidance(
        data, Command::ProcessData,
        safe_outputs_.data(), config_.output_size,
        config_.connection_id,
        last_tx_crc0_, tx_seq_no_, &last_tx_crc0_, &seq_used);
    tx_seq_no_ = seq_used;
    last_tx_seq_no_ = seq_used;
    return result;
}

size_t FSoEMasterConnection::buildFailSafeFrame(uint8_t* data, size_t max_len)
{
    // ETG.5100 S (D) V1.2.0, §8.2.2.6, Table 25:
    // FailSafeData Master PDU: all SafeData octets are set to 0.
    // The fail-safe data carries no useful payload; it signals that
    // the sender has switched its outputs to the safe state.
    // Conn_Id and CRC fields behave exactly as in the ProcessData PDU.
    // See: https://techoverflow.net/2026/08/12/fsoe-data-pdu-master-and-slave-structure/
    static const std::array<uint8_t, 16> zero_safe_data = {0};
    size_t needed = CRC::fsoeFrameSize(config_.output_size);
    if (max_len < needed) return 0;
    uint16_t seq_used = 0;
    size_t result = CRC::buildFSoEFrameWithCollisionAvoidance(
        data, Command::FailSafeData,
        zero_safe_data.data(), config_.output_size,
        config_.connection_id,
        last_tx_crc0_, tx_seq_no_, &last_tx_crc0_, &seq_used);
    tx_seq_no_ = seq_used;
    last_tx_seq_no_ = seq_used;
    return result;
}

// ============================================================================
// Frame Validation
// ============================================================================

bool FSoEMasterConnection::validateCRC(const uint8_t* data, size_t len) const
{
    uint8_t cmd = 0;
    size_t data_len = 0;
    uint16_t conn_id = 0;
    // Self-inheriting RX: verify using own last TX CRC0 and tx_seq_no_
    // (the incremented value, matching the slave's rx_seq_no_ after RX).
    const bool is_reset_frame = (!data || len == 0) ? false : (data[0] == Command::Reset);
    return CRC::parseFSoEFrameWithCollisionAvoidance(
        data, len, cmd, nullptr, data_len, conn_id,
        is_reset_frame ? 0 : last_tx_crc0_,
        is_reset_frame ? CRC::incrementSeqNo(config_.initial_seq_no) : tx_seq_no_);
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

void FSoEMasterConnection::initConnectionTxBuf()
{
    // ETG.5100 §8.2.2.4 Table 15: the Connection state transfers 4 bytes:
    //   byte 0: Connection ID low octet
    //   byte 1: Connection ID high octet
    //   byte 2: FSoE Slave Address low octet
    //   byte 3: FSoE Slave Address high octet
    connection_tx_buf_[0] = static_cast<uint8_t>(config_.connection_id & 0xFF);
    connection_tx_buf_[1] = static_cast<uint8_t>((config_.connection_id >> 8) & 0xFF);
    connection_tx_buf_[2] = static_cast<uint8_t>(config_.slave_safety_addr & 0xFF);
    connection_tx_buf_[3] = static_cast<uint8_t>((config_.slave_safety_addr >> 8) & 0xFF);
    connection_tx_idx_ = 0;
    connection_rx_idx_ = 0;
    memset(connection_rx_buf_, 0, sizeof(connection_rx_buf_));
}

void FSoEMasterConnection::initParameterTxBuf()
{
    // ETG.5100 S (D) V1.2.0, §8.2.2.5, Table 18:
    //   octets 0-1: comm param length (always 2, LE)
    //   octets 2-3: FSoE watchdog (ms, LE)
    //   octets 4-5: app param length (LE)
    //   octets 6+:  app param bytes
    param_tx_buf_.clear();
    param_tx_buf_.reserve(6 + config_.app_parameters.size());
    // Comm param length = 2 (always, just the watchdog)
    param_tx_buf_.push_back(0x02);
    param_tx_buf_.push_back(0x00);
    // FSoE watchdog (ms, LE)
    param_tx_buf_.push_back(static_cast<uint8_t>(config_.watchdog_timeout_ms & 0xFF));
    param_tx_buf_.push_back(static_cast<uint8_t>((config_.watchdog_timeout_ms >> 8) & 0xFF));
    // App param length (LE)
    uint16_t app_len = static_cast<uint16_t>(config_.app_parameters.size());
    param_tx_buf_.push_back(static_cast<uint8_t>(app_len & 0xFF));
    param_tx_buf_.push_back(static_cast<uint8_t>((app_len >> 8) & 0xFF));
    // App param bytes
    for (uint8_t b : config_.app_parameters) {
        param_tx_buf_.push_back(b);
    }
    // Reset indices
    param_tx_idx_ = 0;
    param_rx_idx_ = 0;
    param_rx_buf_.assign(param_tx_buf_.size(), 0);
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

    // Only invalidate the TX cache if the data actually changed.
    // The typed view calls write() (→ setSafeOutputs) every cycle,
    // even when the payload is identical.  Without this check, the
    // cache would be invalidated every cycle, causing the master to
    // rebuild the frame (advancing the CRC chain) every cycle —
    // breaking CRC synchronization with slow slaves.
    //
    // NOTE: safe_outputs_ is a std::array<uint8_t, 16>, so its size()
    // is always 16, not config_.output_size.  Compare only the
    // relevant bytes.
    const bool changed = (std::memcmp(data, safe_outputs_.data(), len) != 0);
    std::copy(data, data + len, safe_outputs_.begin());
    if (changed) {
        tx_cache_dirty_ = true;
    }
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
// PDO exchange
// ============================================================================
//
// The FSoE frame is ALWAYS fixed-length:
//   TX (master→slave): fsoeFrameSize(output_size) = PDO size of RxPDO
//   RX (slave→master): fsoeFrameSize(input_size)  = PDO size of TxPDO
//
// ConnID is ALWAYS at the last 2 bytes of the frame (= last 2 bytes of PDO).
// The frame maps directly to the PDO buffer — no translation needed.

bool FSoEMasterConnection::exchangeViaPDO(uint8_t* rx_pdo_out, size_t rx_pdo_max,
                                          const uint8_t* tx_pdo_in, size_t tx_pdo_len,
                                          uint64_t current_time_ms)
{
    std::lock_guard<std::recursive_mutex> lock(mutex_);

    // --- Sequence trace setup (--debug fsoe-sequence) ---
    // Capture state at entry; emit one structured summary at exit.
    const uint8_t seq_state_before = status_.state;
    bool seq_accepted = false;
    bool seq_tx_rebuilt = false;
    uint8_t seq_rx_cmd = 0;
    const char* seq_reason = "no rx";
    const bool seq_trace = static_cast<bool>(sequence_trace_callback_);
    // RAII emitter: fires the callback when the function returns.
    struct SeqEmitter {
        FSoEMasterConnection* self;
        bool active;
        uint8_t state_before;
        bool& accepted;
        bool& tx_rebuilt;
        uint8_t& rx_cmd;
        const char*& reason;
        uint32_t cycle;
        ~SeqEmitter() {
            if (!active) return;
            uint8_t state_after = self->status_.state;
            SequenceTraceInfo info{};
            info.cycle = cycle;
            info.state_before = state_before;
            info.state_after = state_after;
            info.state_changed = (state_before != state_after);
            info.frame_accepted = accepted;
            info.tx_rebuilt = tx_rebuilt;
            info.rx_cmd = rx_cmd;
            info.reason = reason;
            self->sequence_trace_callback_(info);
        }
    } seq_emitter{this, seq_trace, seq_state_before,
                  seq_accepted, seq_tx_rebuilt, seq_rx_cmd, seq_reason,
                  pdo_tx_count_ + 1};

    // Run the FSoE state machine (watchdog, phase timeouts, auto-recovery).
    update(current_time_ms);

    // TX path: build the FSoE frame directly into the RxPDO buffer.
    // The frame is always fsoeFrameSize(output_size) bytes and maps 1:1
    // to the PDO.  ConnID is at the last 2 bytes of the frame/PDO.
    //
    // CRITICAL: In the PDO path, the master must send the SAME frame bytes
    // every cycle (same CRC, same seq) while in the same state.  This is
    // because the FSoE CRC chain advances with every frame built, and both
    // sides must process the same sequence of frames to keep their CRC
    // chains in sync.  If the master rebuilt the frame every cycle
    // (advancing TX CRC), a slave that doesn't process every frame (e.g.
    // Synapticon's FSoE task runs every 8 cycles) would have its RX CRC
    // fall behind, causing CRC mismatches.
    //
    // Solution: cache the TX frame for the current state.  Only rebuild
    // when the state transitions (which changes the command byte and
    // requires a new CRC chain entry).  Between transitions, resend the
    // exact same frame bytes.
    size_t tx_len = 0;
    const uint8_t current_state = status_.state;
    bool frame_rebuilt = false;  // True if we built a new frame this cycle

    // Cache TX frames in ALL states (including Data).  The master sends
    // the SAME frame bytes every cycle until the state changes or the
    // safe outputs change.  This is essential for slaves with slow FSoE
    // task rates (e.g. Synapticon): if the master rebuilt the frame every
    // cycle, the CRC chain would advance on the master side but not on
    // the slave side (which only processes every N cycles), causing CRC
    // divergence.
    //
    // In Data state, the cache is invalidated when setSafeOutputs() is
    // called (via the tx_cache_dirty_ flag), so changing safe outputs
    // triggers a frame rebuild.
    const bool can_cache = true;

    if (can_cache && !cached_tx_pdo_.empty() && cached_tx_pdo_state_ == current_state &&
        !tx_cache_dirty_) {
        // State hasn't changed — resend the cached frame.
        // This does NOT advance last_tx_crc0_ or tx_seq_no_, keeping
        // the CRC chain in sync with slaves that process at a slower
        // rate.
        tx_len = cached_tx_pdo_.size();
        if (rx_pdo_max >= tx_len) {
            std::memcpy(rx_pdo_out, cached_tx_pdo_.data(), tx_len);
        } else {
            // Buffer too small — fall back to rebuilding
            cached_tx_pdo_.clear();
            cached_tx_pdo_state_ = 0xFF;
            tx_len = 0;
        }
    }

    if (tx_len == 0) {
        // State changed (or first frame, or Data state, or buffer was
        // too small) — build a new frame.  This advances last_tx_crc0_
        // and tx_seq_no_.
        // Also clear last_rx_frame_ so the next received frame is not
        // considered a duplicate (the master is sending a new frame).
        last_rx_frame_.clear();
        frame_rebuilt = true;
        seq_tx_rebuilt = true;
        tx_len = prepareTxFrame(rx_pdo_out, rx_pdo_max);
        if (tx_len == 0) {
            trace("PDO TX: prepareTxFrame returned 0 (state=%s, rx_pdo_max=%zu)",
                  stateName(status_.state), rx_pdo_max);
            seq_reason = "tx build failed";
            return false;
        }
        // Cache the new frame (only for handshake states)
        if (can_cache) {
            cached_tx_pdo_.assign(rx_pdo_out, rx_pdo_out + tx_len);
            cached_tx_pdo_state_ = current_state;
            tx_cache_dirty_ = false;
        }
    }

    // Zero-fill any remaining PDO bytes after the frame.
    if (tx_len < rx_pdo_max) {
        std::fill(rx_pdo_out + tx_len, rx_pdo_out + rx_pdo_max, 0x00);
    }

    // Store the raw TxPDO for timeout diagnostics.
    if (tx_pdo_in && tx_pdo_len > 0) {
        last_txpdo_.assign(tx_pdo_in, tx_pdo_in + tx_pdo_len);
    }

    pdo_tx_count_++;

    // ====================================================================
    // RX change detection (requirements a, b, c)
    // ====================================================================
    //
    // (a) In a simultaneous PDO exchange, the RxPDO frame CANNOT be the
    //     response to the TxPDO frame sent in the same cycle — the slave
    //     has not had time to process it.  The RX is always a response
    //     to a PREVIOUS TX.  We therefore NEVER treat the current RX as
    //     a response to the current TX.
    //
    // (b) No hardcoded frame-count assumptions are made.  The only timing
    //     backstop is the configured FSoE timeout (watchdog / conn_timeout).
    //
    // (c) When the master's TX changes (state transition or safe-output
    //     change), the slave's RX will still be the response to the OLD
    //     TX for some cycles (pipeline + processing delay).  We use
    //     change detection: compare the current RX to the "baseline" RX
    //     (captured when TX changed).  If the RX hasn't changed after
    //     `slave_response_delay_cycles` stale frames, it's an error →
    //     fail-safe.  The FSoE timeout is the ultimate backstop.
    //
    // When TX is rebuilt (frame_rebuilt == true), the current RX is the
    // last response to the OLD TX.  We capture it as the baseline and
    // enter "expecting_rx_change" mode.  In this mode, we skip stale
    // RX frames (identical to baseline) up to the configured budget.
    // When the RX changes, we process it.  When the budget is exhausted,
    // we trigger fail-safe.
    //
    // EXCEPTION: In Reset state, the slave is already sending a valid
    // Reset response — it doesn't need to "change" its TxPDO in response
    // to the master's Reset.  The master should process the slave's
    // Reset response directly (checking length and CRC), then advance
    // to Session.  Change-detection would incorrectly skip the valid
    // Reset response as "stale".

    if (frame_rebuilt && status_.state != ConnectionState::Reset) {
        // TX changed — capture current RX as the baseline (last response
        // to the old TX) and enter change-detection mode.  The current
        // cycle's RX is the old response by definition (the slave hasn't
        // seen the new TX yet), so we skip it immediately without
        // counting it as a stale frame.  Stale counting starts from the
        // NEXT cycle.
        if (tx_pdo_in && tx_pdo_len > 0) {
            baseline_rx_.assign(tx_pdo_in, tx_pdo_in + tx_pdo_len);
        } else {
            baseline_rx_.clear();
        }
        expecting_rx_change_ = true;
        stale_rx_count_ = 0;
        trace("PDO RX: TX changed, capturing baseline (%zu bytes), "
              "entering change-detection mode (max stale=%u)",
              baseline_rx_.size(), config_.slave_response_delay_cycles);
        // Skip this cycle's RX — it's the old response by definition.
        seq_reason = "tx rebuilt, capturing baseline";
        return false;
    }

    // NOTE: last_rx_frame_ is NOT set here — it is managed by
    // processRxFrame (set on successful processing) and prepareTxFrame
    // (cleared when a new TX is built).  Setting it here would break
    // processRxFrame's own duplicate detection for handshake states.
    // For diagnostics, last_txpdo_ (set above) stores the raw RxPDO bytes.

    if (tx_pdo_len == 0 || tx_pdo_in == nullptr) {
        trace("PDO RX: empty TxPDO (tx_pdo_len=%zu)", tx_pdo_len);
        seq_reason = "empty txpdo";
        return false;
    }

    // --- Change detection: skip stale RX frames ---
    //
    // This check runs BEFORE the valid-command check so that stale
    // frames with invalid command bytes (e.g. all-zeros from a previous
    // connection) are still counted as stale.  This ensures the stale
    // budget is exhausted correctly even when the slave's old response
    // has an invalid command byte.
    //
    // In "expecting_rx_change" mode, compare the current RX to the
    // baseline.  If identical, the slave hasn't processed the new TX yet
    // — skip and count.  If the stale count exceeds the configured
    // budget, trigger fail-safe (the slave is not responding to the
    // new TX).
    if (expecting_rx_change_) {
        const bool rx_is_stale =
            !baseline_rx_.empty() &&
            baseline_rx_.size() == tx_pdo_len &&
            std::memcmp(baseline_rx_.data(), tx_pdo_in, tx_pdo_len) == 0;

        if (rx_is_stale) {
            stale_rx_count_++;
            if (stale_rx_count_ > config_.slave_response_delay_cycles) {
                // Stale budget exhausted — the slave has not updated its
                // response within the allowed frame budget.  This is an
                // error → fail-safe.
                trace("PDO RX: stale budget exhausted (%u stale frames > %u max), "
                      "triggering fail-safe (TX cmd=%s, RX still baseline)",
                      stale_rx_count_, config_.slave_response_delay_cycles,
                      commandName(rx_pdo_out[0]));
                FSoEErrorDetail detail;
                snprintf(detail.message, sizeof(detail.message),
                         "Slave response stale after %u cycles (budget %u)",
                         stale_rx_count_, config_.slave_response_delay_cycles);
                handleError(ErrorCode::TimeoutError, detail);
                seq_reason = "stale budget exhausted";
                return false;
            }
            trace("PDO RX: stale frame (%u/%u), skipping (TX changed, "
                  "slave hasn't responded yet)",
                  stale_rx_count_, config_.slave_response_delay_cycles);
            seq_reason = "stale frame";
            return false;
        }

        // RX changed — slave has processed the new TX.  Exit
        // change-detection mode and process the frame.
        trace("PDO RX: frame changed after %u stale cycle(s), processing",
              stale_rx_count_);
        expecting_rx_change_ = false;
        stale_rx_count_ = 0;
        seq_reason = "frame changed after stale";
    }

    // Check for stale/empty TxPDO (all zeros or invalid command byte).
    const uint8_t rx_cmd = tx_pdo_in[0];
    if (!isValidCommand(rx_cmd)) {
        stats_.invalid_frames++;
        trace("PDO RX: cmd=0x%02X not a valid FSoE command, skipping (stale PDO?)",
              rx_cmd);
        seq_rx_cmd = rx_cmd;
        seq_reason = "invalid command";
        return false;
    }

    // In Reset state, only accept Reset (0x2A) and Session (0x4E)
    // responses.  The slave acknowledges a Reset by either echoing Reset
    // or transitioning to Session and responding with Session.  Other
    // commands (e.g. ProcessData from a previous connection) would cause
    // CRC errors and trigger fail-safe.  Skip them silently — the slave
    // will eventually process the master's Reset and respond correctly.
    if (status_.state == ConnectionState::Reset &&
        rx_cmd != Command::Reset && rx_cmd != Command::Session) {
        trace("PDO RX: in Reset state, skipping cmd=%s(0x%02X) "
              "(stale response from previous connection)",
              commandName(rx_cmd), rx_cmd);
        seq_rx_cmd = rx_cmd;
        seq_reason = "wrong cmd in Reset";
        return false;
    }

    trace("PDO RX: processing frame cmd=%s(0x%02X) len=%zu state=%s",
          commandName(rx_cmd), rx_cmd, tx_pdo_len, stateName(status_.state));

    // Duplicate detection for Data state in the PDO path.
    //
    // processRxFrame already does duplicate detection for handshake
    // states (Session, Connection, Parameter).  In Data state, when
    // the master's TX is unchanged (cached), the slave sends the same
    // response every cycle.  The master must skip these duplicates to
    // avoid CRC chain divergence (the slave's TX CRC only advances
    // when it processes a new master frame).
    //
    // This check is ONLY in the PDO path (not in processRxFrame) so
    // that the direct exchange path (exchangeWith) still processes
    // duplicates in Data state (needed by DataStateDuplicateIsProcessed).
    //
    // The watchdog timestamp IS updated — a duplicate frame proves the
    // slave is still alive and communicating.
    if (status_.state == ConnectionState::Data &&
        !last_rx_frame_.empty() &&
        last_rx_frame_.size() == tx_pdo_len &&
        std::memcmp(last_rx_frame_.data(), tx_pdo_in, tx_pdo_len) == 0) {
        stats_.duplicate_frames++;
        status_.last_valid_frame_ms = current_time_ms_;
        trace("PDO RX: duplicate %s frame (slave re-sent, skipping) (state=Data)",
              commandName(rx_cmd));
        seq_rx_cmd = rx_cmd;
        seq_reason = "duplicate (Data)";
        return false;
    }

    // RX path: the FSoE frame is the entire TxPDO buffer.
    // Duplicate detection for handshake states is handled inside
    // processRxFrame (it checks last_rx_frame_ which is cleared by
    // prepareTxFrame above when a new frame is built).
    seq_rx_cmd = rx_cmd;
    const bool rx_ok = processRxFrame(tx_pdo_in, tx_pdo_len);
    if (rx_ok) {
        seq_accepted = true;
        seq_reason = "processed";
    } else {
        seq_reason = "processRxFrame rejected";
    }
    return rx_ok;
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

    const uint8_t old = safe_outputs_[byte_idx];
    if (value) {
        safe_outputs_[byte_idx] |= (1 << bit_pos);
    } else {
        safe_outputs_[byte_idx] &= ~(1 << bit_pos);
    }

    // Only invalidate TX cache if the bit actually changed
    if (safe_outputs_[byte_idx] != old) {
        tx_cache_dirty_ = true;
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

void FSoEMasterConnection::setSequenceTraceCallback(SequenceTraceCallback callback)
{
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    sequence_trace_callback_ = std::move(callback);
}

void FSoEMasterConnection::setCrcTraceCallback(CrcTraceCallback callback)
{
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    crc_trace_callback_ = std::move(callback);
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

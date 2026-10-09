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

/// Decode the slave's Reset reason code (SafeData[0] of the slave's Reset PDU).
/// See ETG.5100 S (D) V1.2.0, §8.2.2.2 and ResetErrorCode in FSoEDefs.hpp.
const char* resetReasonName(uint8_t reason) {
    switch (reason) {
        case ResetErrorCode::None:               return "None(0x00)";
        case ResetErrorCode::InvalidCommand:     return "InvalidCommand(0x01)";
        case ResetErrorCode::UnknownCommand:     return "UnknownCommand(0x02)";
        case ResetErrorCode::InvalidConnID:      return "InvalidConnID(0x03)";
        case ResetErrorCode::InvalidCRC:         return "InvalidCRC(0x04)";
        case ResetErrorCode::WatchdogExpired:    return "WatchdogExpired(0x05)";
        case ResetErrorCode::InvalidAddress:     return "InvalidAddress(0x06)";
        case ResetErrorCode::InvalidData:        return "InvalidData(0x07)";
        case ResetErrorCode::InvalidCommParaLen: return "InvalidCommParaLen(0x08)";
        case ResetErrorCode::InvalidCommPara:    return "InvalidCommPara(0x09)";
        case ResetErrorCode::InvalidUserParaLen: return "InvalidUserParaLen(0x0A)";
        default:                                  return "Unknown";
    }
}

/// Extract the slave's reset reason from a parsed Reset frame.
/// Returns ResetErrorCode::None if data is null or empty.
uint8_t extractResetReason(const uint8_t* data, size_t data_len) {
    if (data && data_len >= 1) return data[0];
    return ResetErrorCode::None;
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
    session_first_tx_done_ = false;
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
    session_first_tx_done_ = false;

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
    // WITHIN the Data state, NOT a separate state.  The master can only
    // send FailSafeData when it is already in Data state.
    //
    // FSoE state machine (ETG.5100, see
    // https://techoverflow.net/2026/08/12/all-the-states-of-the-fsoe-state-machine/):
    //   Reset → Session → Connection → Parameter → Data
    //
    // A NOT_OK transition (error/CRC failure) from ANY state returns to
    // Reset — it does NOT skip to Data.  Jumping from Reset to Data
    // violates the protocol: the slave would never see the Session,
    // Connection, and Parameter handshakes, and the CRC chains would
    // never be established.
    //
    // If triggerFailSafe is called from a non-Data state (e.g. a CRC
    // error during handshake), go back to Reset (NOT_OK transition).
    // Only stay in Data if already there.
    if (status_.state != ConnectionState::Data) {
        trace("triggerFailSafe: in %s state, going back to Reset (NOT_OK)",
              stateName(status_.state));
        resetConnection();
        // Preserve error code after reset (resetConnection clears it).
        // The master includes this as the reset reason in its Reset PDU.
        if (error_code != ErrorCode::NoError) {
            status_.error_code = error_code;
        }
        return;
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
    // CRC inheritance for RX parsing:
    //   - Reset frames: start_crc=0, seq=initial_seq_no (Reset resets chain)
    //     BUT we still capture last_rx_crc0_ so the next TX (Session) can
    //     chain from the slave's Reset response CRC0 (cross-direction).
    //   - Non-Reset frames: start_crc=last_tx_crc0_, seq=last_tx_seq_no_
    //     (the slave's TX inherits from the master's last TX CRC0 and seq,
    //      so the master verifies using its own last TX CRC0 and seq)
    uint8_t cmd = 0;
    uint8_t frame_data[CRC::MAX_PARSE_DATA_SIZE] = {0};
    size_t data_len = 0;
    uint16_t conn_id = 0;
    CRC::CrcErrorDetail crc_error_detail{};

    const bool is_reset_frame = (data[0] == Command::Reset);
    // Reset frames reset the CRC chain: start_crc=0, seq=initial.
    // The slave's Reset response uses its OWN initial seq, which is the
    // SAME value as the master's initial_seq_no (master and slave have
    // independent counters that both start at the configured initial value).
    // All other frames (Session, Connection, Parameter, Data) use
    // cross-direction inheritance: start_crc=last_tx_crc0_, seq=last_tx_seq_no_
    // (the slave's TX inherits from the master's last TX CRC0 and seq,
    //  so the master verifies using its own last TX CRC0 and seq).
    const uint16_t parse_start_crc = is_reset_frame ? 0 : last_tx_crc0_;
    const uint16_t parse_seq_no = is_reset_frame
        ? config_.initial_seq_no
        : last_tx_seq_no_;
    uint16_t seq_used = 0;

    // CRC trace tracking (for --debug fsoe-crc)
    bool crc_trace_fb_used = false;
    int  crc_trace_fb_delta = 0;
    bool crc_trace_ok = false;
    uint16_t crc_trace_crc0 = 0;

    if (!CRC::parseFSoEFrameWithCollisionAvoidance(
            data, len, cmd, frame_data, data_len, conn_id,
            parse_start_crc, parse_seq_no,
            &last_rx_crc0_,  // always capture CRC0 for cross-direction chaining
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
            // Extract data bytes even on CRC failure (for diagnosis).
            if (data_len > 0 && data_len <= sizeof(info.data)) {
                std::memcpy(info.data, frame_data, data_len);
            }
            // Extract actual CRC0 from the frame bytes (the received CRC0
            // that failed to match the expected CRC0).
            if (len > CRC::MIN_FSOE_FRAME_SIZE) {
                const size_t crc0_off = (len <= 6) ? 2 : 3;
                info.crc0 = static_cast<uint16_t>(data[crc0_off]) |
                            (static_cast<uint16_t>(data[crc0_off + 1]) << 8);
            }
            crc_trace_callback_(info);
        }

        handleError(ErrorCode::CRCError, detail);

        // ConnID recovery: if this identical frame was already resolved,
        // emit the recovered safety address as a question right away;
        // otherwise hand it to the background resolver.
        if (crc_error_detail.valid) {
            bool emitted_cached = false;
            {
                std::lock_guard<std::mutex> g(addr_hint_->m);
                if (addr_hint_->ready && error_callback_ &&
                    addr_hint_->frame.size() == len &&
                    std::memcmp(addr_hint_->frame.data(), data, len) == 0) {
                    FSoEErrorDetail d{};
                    const auto& r = addr_hint_->result;
                    if (r.found) {
                        snprintf(d.message, sizeof(d.message),
                                 "safety address 0x%04X would lead to a "
                                 "correct CRC — are you sure that safety "
                                 "address 0x%04X is correct?",
                                 r.conn_id, config_.connection_id);
                    } else {
                        snprintf(d.message, sizeof(d.message),
                                 "no connID validates frame (trailer "
                                 "0x%04X) — corrupt or seq desync?",
                                 r.frame_conn_id);
                    }
                    error_callback_(ErrorCode::CRCError, d);
                    emitted_cached = true;
                }
            }
            if (!emitted_cached) {
                SafetyAddressResolver::Job job;
                job.frame.assign(data, data + len);
                job.start_crc = parse_start_crc;
                job.seq_no = parse_seq_no;
                auto hint = addr_hint_;
                const auto frame_copy = job.frame;
                const auto expected_conn_id = config_.connection_id;
                const ErrorCallback report = error_callback_;
                addr_resolver_.submit(std::move(job),
                    [hint, report, frame_copy, expected_conn_id]
                    (const SafetyAddressResolver::Result& r) {
                        {
                            std::lock_guard<std::mutex> g(hint->m);
                            hint->ready = true;
                            hint->result = r;
                            hint->frame = frame_copy;
                        }
                        if (!report) return;
                        FSoEErrorDetail d{};
                        if (r.found) {
                            snprintf(d.message, sizeof(d.message),
                                     "safety address 0x%04X would lead to a "
                                     "correct CRC — are you sure that "
                                     "safety address 0x%04X is correct?",
                                     r.conn_id, expected_conn_id);
                        } else {
                            snprintf(d.message, sizeof(d.message),
                                     "no connID validates frame (trailer "
                                     "0x%04X) — corrupt or seq desync?",
                                     r.frame_conn_id);
                        }
                        report(ErrorCode::CRCError, d);
                    });
            }
        }
        return false;
    }

    // Frame verified with the expected seq — reset the fallback counter.
    consecutive_seq_fallback_ = 0;
    {
        std::lock_guard<std::mutex> g(addr_hint_->m);
        addr_hint_->ready = false;
        addr_hint_->frame.clear();
    }
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
        if (data_len > 0 && data_len <= sizeof(info.data)) {
            std::memcpy(info.data, frame_data, data_len);
        }
        // Extract actual CRC0 from the frame bytes for the trace.
        // crc_trace_crc0 already holds last_rx_crc0_ (now updated for all
        // frames including Reset), but reading from the frame bytes gives
        // the true value even for 3-byte Reset frames (no CRC).
        // Layout: [CMD][D0][CRC0(2)]... (pduSize <= 6)
        //         [CMD][D0][D1][CRC0(2)]... (pduSize > 6)
        // 3-byte Reset frames (len == MIN_FSOE_FRAME_SIZE) have no CRC.
        if (len > CRC::MIN_FSOE_FRAME_SIZE) {
            const size_t crc0_off = (len <= 6) ? 2 : 3;
            info.crc0 = static_cast<uint16_t>(data[crc0_off]) |
                        (static_cast<uint16_t>(data[crc0_off + 1]) << 8);
        }
        crc_trace_callback_(info);
    }

    // Save the seq that matched (after collision avoidance) for diagnostics.
    // The slave has its own independent TX seq counter; we don't use this
    // for any CRC chain — it's purely informational.
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
                const uint8_t reason = extractResetReason(frame_data, data_len);
                if (config_.auto_recovery_enabled) {
                    trace("RX Reset(0x2A): recovering from Error state "
                          "(reset_reason=%s, data[0]=0x%02X)",
                          resetReasonName(reason), reason);
                    resetConnection();
                    stats_.successful_recoveries++;
                } else {
                    trace("RX Reset(0x2A): auto-recovery disabled, staying in Error "
                          "(reset_reason=%s, data[0]=0x%02X)",
                          resetReasonName(reason), reason);
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
            // Extract actual SafeData, conn_id, and CRC0 from the built
            // frame.  This is important because the build* functions for
            // Reset and Session frames use conn_id=0 (not config_.connection_id),
            // and buildResetFrame passes nullptr for out_crc0 (so
            // last_tx_crc0_ is not updated).  Reading from the frame bytes
            // gives the true values that the slave will see on the wire.
            {
                uint8_t trace_cmd = 0;
                size_t trace_dl = 0;
                uint16_t trace_cid = 0;
                CRC::extractFSoEFrame(data, len, trace_cmd, info.data,
                                      trace_dl, trace_cid);
                info.data_len = trace_dl;
                info.conn_id = trace_cid;
                // Extract CRC0 from the frame bytes.
                // Layout: [CMD][D0][CRC0(2)]... (pduSize <= 6)
                //         [CMD][D0][D1][CRC0(2)]... (pduSize > 6)
                // Reset frames (len == MIN_FSOE_FRAME_SIZE == 3) have no CRC.
                if (len > CRC::MIN_FSOE_FRAME_SIZE) {
                    const size_t crc0_off = (len <= 6) ? 2 : 3;
                    info.crc0 = static_cast<uint16_t>(data[crc0_off]) |
                                (static_cast<uint16_t>(data[crc0_off + 1]) << 8);
                }
            }
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
} // namespace FSoE

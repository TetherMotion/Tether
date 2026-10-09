/**
 * @file FSoEMasterConnection_data.cpp
 * @brief FSoEMasterConnection — safe data access, PDO exchange, diagnostics.
 *
 * TU split out of FSoEMasterConnection.cpp.
 */

#include "fsoe/FSoEMasterConnection.hpp"
#include "fsoe/FSoESlave.hpp"
#include "fsoe/FSoECRC.hpp"
#include <cstring>
#include <algorithm>
#include <cstdio>
#include <cstdarg>

namespace FSoE {

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
    //
    // IMPORTANT: When in change-detection mode (expecting_rx_change_),
    // we must NOT rebuild the frame even if tx_cache_dirty_ is true.
    // Rebuilding advances the CRC/seq, but the slave hasn't processed
    // the current cached frame yet.  If we rebuild now, the master's
    // CRC/seq will be ahead of the slave's, causing a CRC mismatch when
    // the slave eventually processes the new frame.  Instead, keep
    // sending the cached frame until the slave confirms processing (RX
    // changes), then rebuild with the new outputs.
    const bool can_cache = true;
    const bool suppress_rebuild = expecting_rx_change_;

    if (can_cache && !cached_tx_pdo_.empty() && cached_tx_pdo_state_ == current_state &&
        (!tx_cache_dirty_ || suppress_rebuild)) {
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

        // In Data state, a non-duplicate RX means the slave has processed
        // our TX and advanced its CRC chain.  Invalidate the TX cache so
        // the next frame is rebuilt with the advanced CRC, keeping both
        // sides in lockstep.  Without this, the master keeps sending the
        // same cached frame (same CRC), but the slave's CRC has moved on,
        // causing CRC mismatches and eventual watchdog timeout.
        if (status_.state == ConnectionState::Data) {
            tx_cache_dirty_ = true;
        }
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

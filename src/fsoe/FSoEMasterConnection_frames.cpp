/**
 * @file FSoEMasterConnection_frames.cpp
 * @brief FSoEMasterConnection — frame building/validation + transitions.
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
    // CRC inheritance for the Session frame:
    // - First Session TX: state-transition reset (start_crc=0, seq=initial).
    //   The Synapticon slave resets the CRC chain AND seq at the Session
    //   state transition.
    // - Subsequent Session TXs (1-octet safety data, high byte): cross-
    //   direction inheritance (start_crc=last_rx_crc0_, seq=tx_seq_no_).
    //   The CRC chain continues from the slave's last TX (which the master
    //   received and validated).  The seq continues from the master's own
    //   tx_seq_no_ counter (which was incremented after the first TX).
    const bool first_tx = !session_first_tx_done_;
    const uint16_t start_crc = first_tx ? 0 : last_rx_crc0_;
    const uint16_t seq_no = first_tx ? config_.initial_seq_no : tx_seq_no_;
    size_t result = CRC::buildFSoEFrameWithCollisionAvoidance(
        data, Command::Session, payload, config_.output_size,
        0,  // Conn_Id = 0 in Session state (ETG.5100 §8.2.2.3)
        start_crc, seq_no,
        &last_tx_crc0_, &seq_used);
    tx_seq_no_ = seq_used;
    last_tx_seq_no_ = seq_used;
    session_first_tx_done_ = true;
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
    // Cross-direction CRC inheritance: the master's TX chains from the
    // slave's last TX CRC0 (last_rx_crc0_), not from the master's own
    // last TX CRC0.  The slave verifies the master's TX using its own
    // last TX CRC0 as start_crc.
    size_t result = CRC::buildFSoEFrameWithCollisionAvoidance(
        data, Command::Connection, payload, config_.output_size,
        config_.connection_id,
        last_rx_crc0_, tx_seq_no_, &last_tx_crc0_, &seq_used);
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
    // Cross-direction CRC inheritance (see buildConnectionFrame).
    size_t result = CRC::buildFSoEFrameWithCollisionAvoidance(
        data, Command::Parameter, payload, config_.output_size,
        config_.connection_id,
        last_rx_crc0_, tx_seq_no_, &last_tx_crc0_, &seq_used);
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
    // Cross-direction CRC inheritance (see buildConnectionFrame).
    size_t result = CRC::buildFSoEFrameWithCollisionAvoidance(
        data, Command::ProcessData,
        safe_outputs_.data(), config_.output_size,
        config_.connection_id,
        last_rx_crc0_, tx_seq_no_, &last_tx_crc0_, &seq_used);
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
    // Cross-direction CRC inheritance (see buildConnectionFrame).
    size_t result = CRC::buildFSoEFrameWithCollisionAvoidance(
        data, Command::FailSafeData,
        zero_safe_data.data(), config_.output_size,
        config_.connection_id,
        last_rx_crc0_, tx_seq_no_, &last_tx_crc0_, &seq_used);
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
        data, len, cmd, {}, data_len, conn_id,
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

    // FSoE state machine: a NOT_OK transition (error/CRC/watchdog failure)
    // from ANY state returns to Reset.  Only when already in Data state
    // does an error trigger fail-safe (FailSafeData command within Data).
    //
    // See: https://techoverflow.net/2026/08/12/all-the-states-of-the-fsoe-state-machine/
    //   Reset → Session → Connection → Parameter → Data
    //   NOT_OK from any state → Reset
    if (config_.auto_fail_safe_on_error) {
        if (status_.state == ConnectionState::Data) {
            // Already in Data — enter fail-safe (send FailSafeData)
            triggerFailSafe(error_code);
        } else {
            // In handshake state — NOT_OK transition back to Reset
            trace("handleError: in %s state, going back to Reset (NOT_OK)",
                  stateName(status_.state));
            resetConnection();
            // Preserve error code after reset (resetConnection clears it).
            status_.error_code = error_code;
        }
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
} // namespace FSoE

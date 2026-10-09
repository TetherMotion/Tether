/**
 * @file FSoEMasterConnection_handlers.cpp
 * @brief FSoEMasterConnection — state handlers.
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

// State Handlers
// ============================================================================

void FSoEMasterConnection::handleResetState(uint8_t cmd, const uint8_t* data, size_t data_len)
{
    // The master sent a Reset command (0x2A) to force the slave back to its
    // initial state.  The slave acknowledges by transitioning to Session and
    // responding with a Session frame (0x4E).  When we see that response,
    // generate a fresh session ID and transition to Session to begin the
    // handshake.  A Reset response (0x2A) is also accepted — some slaves
    // echo the Reset command before switching to Session.
    if (cmd == Command::Session || cmd == Command::Reset) {
        if (cmd == Command::Reset) {
            const uint8_t reason = extractResetReason(data, data_len);
            trace("RX Reset(0x2A): slave acknowledged reset "
                  "(reset_reason=%s, data[0]=0x%02X), starting session handshake",
                  resetReasonName(reason), reason);
        } else {
            trace("RX %s: slave acknowledged reset, starting session handshake",
                  commandName(cmd));
        }
        // CRC chain is already at 0 — Reset frames don't update it.
        requestSessionReset();
        // With 1-octet safety data, the slave's Session response carries
        // the low byte of its Session ID.  requestSessionReset() reset
        // session_octet_idx_ to 0, so restore the low byte and advance
        // the index.  The high byte will be received in handleSessionState.
        // Without this, the low byte is lost and the bytes get swapped.
        if (cmd == Command::Session && config_.input_size < 2 &&
            data && data_len >= 1) {
            status_.slave_session_id =
                static_cast<uint16_t>(data[0]);
            session_octet_idx_ = 1;
            trace("RX Session(0x4E) in Reset: slave session_id low byte=0x%02X, "
                  "waiting for high byte", data[0]);
        }
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
        const uint8_t reason = extractResetReason(data, data_len);
        trace("RX Reset(0x2A): slave requested reset in Session state "
              "(reset_reason=%s, data[0]=0x%02X), resetting connection",
              resetReasonName(reason), reason);
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
        const uint8_t reason = extractResetReason(data, data_len);
        trace("RX Reset(0x2A): slave requested reset in Connection state "
              "(reset_reason=%s, data[0]=0x%02X), resetting connection",
              resetReasonName(reason), reason);
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
        const uint8_t reason = extractResetReason(data, data_len);
        trace("RX Reset(0x2A): slave requested reset in Parameter state "
              "(reset_reason=%s, data[0]=0x%02X), resetting connection",
              resetReasonName(reason), reason);
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
        const uint8_t reason = extractResetReason(data, data_len);
        trace("RX Reset(0x2A): slave requested reset in Data state "
              "(reset_reason=%s, data[0]=0x%02X), resetting connection",
              resetReasonName(reason), reason);
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
        const uint8_t reason = extractResetReason(data, data_len);
        if (config_.auto_recovery_enabled) {
            trace("RX Reset(0x2A): slave ready to recover "
                  "(reset_reason=%s, data[0]=0x%02X), resetting connection",
                  resetReasonName(reason), reason);
            stats_.successful_recoveries++;
            resetConnection();
        } else {
            trace("RX Reset(0x2A): slave ready to recover, but auto-recovery disabled "
                  "(reset_reason=%s, data[0]=0x%02X)",
                  resetReasonName(reason), reason);
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
} // namespace FSoE

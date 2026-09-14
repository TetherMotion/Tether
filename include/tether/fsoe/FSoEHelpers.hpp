#pragma once

#include <cstdint>
#include <cstring>
#include <string>

#include "tether/fsoe/FSoEDefs.hpp"

namespace FSoE {

/// Decode the FSoE command byte to a human-readable name (with hex code).
inline const char* fsoeCommandName(uint8_t cmd) {
    switch (cmd) {
        case FSoE::Command::ProcessData:   return "ProcessData(0x36)";
        case FSoE::Command::Reset:         return "Reset(0x2A)";
        case FSoE::Command::Session:       return "Session(0x4E)";
        case FSoE::Command::Connection:    return "Connection(0x64)";
        case FSoE::Command::Parameter:     return "Parameter(0x52)";
        case FSoE::Command::FailSafeData:  return "FailSafeData(0x08)";
        default:                            return "Unknown";
    }
}

/// Decode the FSoE command byte to a plain name (no hex suffix).
inline const char* fsoeCommandShortName(uint8_t cmd) {
    switch (cmd) {
        case FSoE::Command::ProcessData:   return "ProcessData";
        case FSoE::Command::Reset:         return "Reset";
        case FSoE::Command::Session:       return "Session";
        case FSoE::Command::Connection:    return "Connection";
        case FSoE::Command::Parameter:     return "Parameter";
        case FSoE::Command::FailSafeData:  return "FailSafeData";
        default:                            return "Unknown";
    }
}

/// Decode the FSoE connection state to a human-readable name.
inline const char* fsoeStateName(uint8_t state) {
    switch (state) {
        case FSoE::ConnectionState::Reset:      return "RESET";
        case FSoE::ConnectionState::Session:    return "SESSION";
        case FSoE::ConnectionState::Connection: return "CONNECTION";
        case FSoE::ConnectionState::Parameter:  return "PARAMETER";
        case FSoE::ConnectionState::Data:       return "DATA";
        case FSoE::ConnectionState::FailSafe:   return "FAILSAFE";
        case FSoE::ConnectionState::Error:      return "ERROR";
        default:                                return "UNKNOWN";
    }
}

/// Decode the FSoE error code to a human-readable name.
inline const char* fsoeErrorName(uint16_t code) {
    switch (code) {
        case FSoE::ErrorCode::NoError:           return "NoError";
        case FSoE::ErrorCode::CommandError:      return "CommandError";
        case FSoE::ErrorCode::CRCError:          return "CRCError";
        case FSoE::ErrorCode::WatchdogError:     return "WatchdogError";
        case FSoE::ErrorCode::SequenceError:     return "SequenceError";
        case FSoE::ErrorCode::ConnectionIDError: return "ConnectionIDError";
        case FSoE::ErrorCode::DataLengthError:   return "DataLengthError";
        case FSoE::ErrorCode::ParameterError:    return "ParameterError";
        case FSoE::ErrorCode::ApplicationError:  return "ApplicationError";
        case FSoE::ErrorCode::TimeoutError:      return "TimeoutError";
        case FSoE::ErrorCode::UnexpectedData:    return "UnexpectedData";
        case FSoE::ErrorCode::SessionError:      return "SessionError";
        case FSoE::ErrorCode::MasterTimeout:     return "MasterTimeout";
        case FSoE::ErrorCode::SlaveTimeout:      return "SlaveTimeout";
        case FSoE::ErrorCode::StartupError:      return "StartupError";
        case FSoE::ErrorCode::CommChannelError:  return "CommChannelError";
        default:                                 return "Unknown";
    }
}

/// Decode the FSoE Reset PDU error code (SafeData[0] in a Reset frame,
/// ETG.5100) to a human-readable name.
/// A value of 0x00 means local reset or acknowledgement.
/// Values 0x80–0xFF are device-specific (Invalid SafePara).
inline const char* fsoeResetErrorCodeName(uint8_t code) {
    switch (code) {
        case FSoE::ResetErrorCode::None:               return "None (local reset/ack)";
        case FSoE::ResetErrorCode::InvalidCommand:     return "InvalidCommand (INVALID_CMD)";
        case FSoE::ResetErrorCode::UnknownCommand:     return "UnknownCommand (UNKNOWN_CMD)";
        case FSoE::ResetErrorCode::InvalidConnID:      return "InvalidConnID (INVALID_CONNID)";
        case FSoE::ResetErrorCode::InvalidCRC:         return "InvalidCRC (INVALID_CRC)";
        case FSoE::ResetErrorCode::WatchdogExpired:    return "WatchdogExpired (WD_EXPIRED)";
        case FSoE::ResetErrorCode::InvalidAddress:     return "InvalidAddress (INVALID_ADDRESS)";
        case FSoE::ResetErrorCode::InvalidData:        return "InvalidData (INVALID_DATA)";
        case FSoE::ResetErrorCode::InvalidCommParaLen: return "InvalidCommParaLen (INVALID_COMPARALEN)";
        case FSoE::ResetErrorCode::InvalidCommPara:    return "InvalidCommPara (INVALID_COMPARA)";
        case FSoE::ResetErrorCode::InvalidUserParaLen: return "InvalidUserParaLen (INVALID_USERPARALEN)";
        case FSoE::ResetErrorCode::InvalidUserPara:    return "InvalidUserPara (INVALID_USERPARA)";
        default:
            if (code >= 0x80) return "InvalidSafePara (device-specific)";
            return "Unknown";
    }
}

/// Decode the FSoE Reset PDU error code to a plain short name
/// (no parenthetical ETG.5100 code suffix).
inline const char* fsoeResetReasonName(uint8_t code) {
    switch (code) {
        case FSoE::ResetErrorCode::None:               return "None";
        case FSoE::ResetErrorCode::InvalidCommand:     return "InvalidCommand";
        case FSoE::ResetErrorCode::UnknownCommand:     return "UnknownCommand";
        case FSoE::ResetErrorCode::InvalidConnID:      return "InvalidConnID";
        case FSoE::ResetErrorCode::InvalidCRC:         return "InvalidCRC";
        case FSoE::ResetErrorCode::WatchdogExpired:    return "WatchdogExpired";
        case FSoE::ResetErrorCode::InvalidAddress:     return "InvalidAddress";
        case FSoE::ResetErrorCode::InvalidData:        return "InvalidData";
        case FSoE::ResetErrorCode::InvalidCommParaLen: return "InvalidCommParaLen";
        case FSoE::ResetErrorCode::InvalidCommPara:    return "InvalidCommPara";
        case FSoE::ResetErrorCode::InvalidUserParaLen: return "InvalidUserParaLen";
        case FSoE::ResetErrorCode::InvalidUserPara:    return "InvalidUserPara";
        default:                                       return "Reserved";
    }
}

/// Compact per-frame tag for stream lines: "cmd=<name>", plus
/// " reason=<name>" when the frame is a Reset command.
/// @param frame  Raw FSoE frame (byte 0 = command, byte 1 = reset reason).
inline std::string fsoeCommandTag(const uint8_t* frame) {
    std::string s = "cmd=";
    s += fsoeCommandShortName(frame[0]);
    if (frame[0] == FSoE::Command::Reset) {
        s += " reason=";
        s += fsoeResetReasonName(frame[1]);
    }
    return s;
}

// ============================================================================
// PDO priming helpers
// ============================================================================

/// Build an FSoE Reset PDU that fills a PDO buffer.
///
/// Writes the FSoE Reset command (0x2A) at byte 0, zero-fills all data/CRC
/// bytes, and writes the connection ID at the last two bytes of the buffer.
/// This is intended for pre-priming the PDO buffer before the FSoE master
/// cyclic task takes over — the slave's safety module sees a valid FSoE
/// command on the wire immediately rather than an uninitialized buffer.
///
/// For a PDO buffer sized to a full FSoE frame (e.g. 11 bytes for the
/// Synapticon 0x1700 RxPDO), the layout is:
///   [CMD(0x2A)] [zeros(data+CRCs)] [ConnID_lo] [ConnID_hi]
///
/// For a minimal 3-byte buffer (no data bytes), the layout is:
///   [CMD(0x2A)] [ConnID_lo] [ConnID_hi]
///
/// @param buf      Output buffer (PDO region to fill).
/// @param buf_size Total size of the PDO region in bytes.
/// @param conn_id  FSoE Connection ID.
/// @return Number of bytes written (equal to buf_size).
inline size_t buildResetPdu(uint8_t* buf, size_t buf_size, uint16_t conn_id) {
    if (!buf || buf_size < 3) return 0;
    std::memset(buf, 0, buf_size);
    buf[0] = FSoE::Command::Reset;
    buf[buf_size - 2] = static_cast<uint8_t>(conn_id & 0xFF);
    buf[buf_size - 1] = static_cast<uint8_t>((conn_id >> 8) & 0xFF);
    return buf_size;
}

} // namespace FSoE

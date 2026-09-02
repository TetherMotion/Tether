#pragma once

#include <cstdint>

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

} // namespace FSoE

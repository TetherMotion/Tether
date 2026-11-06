/**
 * @file ExceptionCode.hpp
 * @brief Modbus exception response codes (plan §3.4).
 */
#pragma once

#include <cstdint>
#include <string_view>

#include "tether/modbus/ModbusError.hpp"

namespace tether::modbus {

/**@brief Standard Modbus exception codes (wire values). */
enum class ExceptionCode : uint8_t {
    IllegalFunction            = 0x01,
    IllegalDataAddress         = 0x02,
    IllegalDataValue           = 0x03,
    ServerDeviceFailure        = 0x04,
    Acknowledge                = 0x05,  ///< long-running; retry later
    ServerDeviceBusy           = 0x06,
    NegativeAcknowledge        = 0x07,
    MemoryParityError          = 0x08,
    GatewayPathUnavailable     = 0x0A,
    GatewayTargetFailedToRespond = 0x0B,
};

constexpr std::string_view exceptionName(uint8_t code) {
    switch (code) {
        case 0x01: return "IllegalFunction";
        case 0x02: return "IllegalDataAddress";
        case 0x03: return "IllegalDataValue";
        case 0x04: return "ServerDeviceFailure";
        case 0x05: return "Acknowledge";
        case 0x06: return "ServerDeviceBusy";
        case 0x07: return "NegativeAcknowledge";
        case 0x08: return "MemoryParityError";
        case 0x0A: return "GatewayPathUnavailable";
        case 0x0B: return "GatewayTargetFailedToRespond";
        default:   return "UnknownException";
    }
}

/// Wire exception byte → ModbusError (retryable for 0x05/0x06 by policy).
inline ModbusError exceptionToError(uint8_t code) {
    return ModbusError::exception(
        code, std::string(exceptionName(code)));
}

/// True for codes where a retry may plausibly succeed.
constexpr bool exceptionRetryable(uint8_t code) {
    return code == 0x05 || code == 0x06;
}

} // namespace tether::modbus

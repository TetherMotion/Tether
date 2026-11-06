/**
 * @file Codec.hpp
 * @brief Request/response encode+decode for all function codes.
 *
 * Pure functions: wire bytes ↔ typed records. Validation applies the
 * per-FC ceilings + field rules from plan AK.20. Errors are Codec or
 * Modbus(exception) via Expected.
 */
#pragma once

#include <cstdint>
#include <span>

#include "tether/modbus/ModbusError.hpp"
#include "tether/modbus/protocol/FunctionCode.hpp"
#include "tether/modbus/protocol/Pdu.hpp"
#include "tether/modbus/protocol/Requests.hpp"
#include "tether/modbus/protocol/Responses.hpp"

namespace tether::modbus {

// ---- request decode ---------------------------------------------------------
/// Decode request body (no fc byte) for `fc`. Errors → Codec error.
Expected<Request> decodeRequest(FunctionCode fc,
                                std::span<const uint8_t> body);

/// Encode a typed request into its wire body (no fc byte).
Expected<std::vector<uint8_t>> encodeRequest(FunctionCode fc,
                                             const Request& req);

/// Convenience: full PDU bytes for a typed request.
Expected<Pdu> requestPdu(FunctionCode fc, const Request& req);

// ---- response decode --------------------------------------------------------
/// Decode a response PDU for the given request `fc`. Exception PDUs are
/// handled by the caller (see parsePduHead); this decodes normal PDUs.
Expected<Response> decodeResponse(FunctionCode fc,
                                  std::span<const uint8_t> pduBody);

/// Encode a typed server response into its wire body.
Expected<std::vector<uint8_t>> encodeResponse(FunctionCode fc,
                                              const Response& resp,
                                              const Request& reqCtx);

/// Validate echo responses (FC5/6/22/21): body must equal request body.
Expected<void> verifyEcho(FunctionCode fc,
                          std::span<const uint8_t> reqBody,
                          std::span<const uint8_t> rspBody);

// ---- helpers -----------------------------------------------------------------
/// byteCount field for a bit quantity.
constexpr uint8_t bitByteCount(uint16_t qty) {
    return static_cast<uint8_t>((qty + 7) / 8);
}
/// Read a big-endian u16 at `off` (bounds-checked by caller).
inline uint16_t be16(std::span<const uint8_t> b, size_t off) {
    return static_cast<uint16_t>((b[off] << 8) | b[off + 1]);
}
inline uint32_t be32(std::span<const uint8_t> b, size_t off) {
    return (uint32_t(be16(b, off)) << 16) | be16(b, off + 2);
}

} // namespace tether::modbus

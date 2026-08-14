#pragma once

/// @file ResponseBuilder.hpp
/// @brief Helpers for building Moonraker-compatible HTTP/JSON-RPC responses.

#include "tether/klipper/klippy/JsonValue.hpp"
#include "tether/klipper/http/GlazeAdapter.hpp"

#include <string>
#include <string_view>

namespace tether::klipper::http {

/// @brief Moonraker JSON-RPC error codes.
enum class RpcErrorCode : int {
    ParseError = -32700,
    InvalidRequest = -32600,
    MethodNotFound = -32601,
    InvalidParams = -32602,
    InternalError = -32603,
    // Moonraker-specific (HTTP-style codes in JSON-RPC)
    BadRequest = 400,
    Unauthorized = 401,
    Forbidden = 403,
    NotFound = 404,
    Conflict = 409,
    TooManyRequests = 429,
    InternalServerError = 500,
    ServiceUnavailable = 503,
};

/// @brief Build a Moonraker-style success response envelope: {"result": ...}
inline std::string buildSuccessResponse(const klippy::JsonValue& result) {
    std::map<std::string, klippy::JsonValue> envelope;
    envelope["result"] = result;
    return dumpJson(klippy::JsonValue(envelope));
}

/// @brief Build a Moonraker-style error response envelope: {"error": {...}}
inline std::string buildErrorResponse(int code, const std::string& message) {
    std::map<std::string, klippy::JsonValue> err;
    err["code"] = klippy::JsonValue(static_cast<int64_t>(code));
    err["message"] = klippy::JsonValue(message);
    std::map<std::string, klippy::JsonValue> envelope;
    envelope["error"] = klippy::JsonValue(err);
    return dumpJson(klippy::JsonValue(envelope));
}

/// @brief Build a JSON-RPC 2.0 success response.
inline std::string buildJsonRpcSuccess(const klippy::JsonValue& id,
                                        const klippy::JsonValue& result) {
    std::map<std::string, klippy::JsonValue> resp;
    resp["jsonrpc"] = klippy::JsonValue("2.0");
    resp["id"] = id;
    resp["result"] = result;
    return dumpJson(klippy::JsonValue(resp));
}

/// @brief Build a JSON-RPC 2.0 error response.
inline std::string buildJsonRpcError(const klippy::JsonValue& id,
                                      int code, const std::string& message) {
    std::map<std::string, klippy::JsonValue> err;
    err["code"] = klippy::JsonValue(static_cast<int64_t>(code));
    err["message"] = klippy::JsonValue(message);
    std::map<std::string, klippy::JsonValue> resp;
    resp["jsonrpc"] = klippy::JsonValue("2.0");
    resp["id"] = id;
    resp["error"] = klippy::JsonValue(err);
    return dumpJson(klippy::JsonValue(resp));
}

/// @brief Build a JSON-RPC 2.0 notification (no id, method + params).
inline std::string buildJsonRpcNotification(const std::string& method,
                                             const klippy::JsonValue& params) {
    std::map<std::string, klippy::JsonValue> msg;
    msg["jsonrpc"] = klippy::JsonValue("2.0");
    msg["method"] = klippy::JsonValue(method);
    msg["params"] = params;
    return dumpJson(klippy::JsonValue(msg));
}

} // namespace tether::klipper::http

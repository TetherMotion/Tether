#pragma once

/// @file JsonRpcDispatcher.hpp
/// @brief JSON-RPC 2.0 dispatcher that maps dotted method names to UDS endpoints.
///
/// Moonraker's JSON-RPC protocol uses dotted method names (e.g. "server.info",
/// "printer.objects.query") while the existing KlippyUdsServer uses slash-style
/// method names (e.g. "server/info", "objects/query"). This dispatcher converts
/// between the two and delegates to KlippyUdsServer::callEndpoint().

#include "tether/klipper/klippy/JsonValue.hpp"
#include "tether/klipper/klippy/UdsTypes.hpp"
#include "tether/klipper/http/ResponseBuilder.hpp"

#include <functional>
#include <optional>
#include <string>
#include <string_view>

namespace tether::klipper::http {

/// @brief Type of the endpoint call function.
/// Takes a method name and params, returns the result JsonValue.
using EndpointCallable = std::function<klippy::JsonValue(
    const std::string& method, const klippy::JsonValue& params)>;

/// @brief JSON-RPC 2.0 dispatcher.
///
/// Converts dotted JSON-RPC method names to slash-style UDS method names
/// and dispatches to the provided endpoint callable.
class JsonRpcDispatcher {
public:
    /// @brief Construct with an endpoint callable.
    /// @param endpointCall Function that takes (method, params) and returns result.
    explicit JsonRpcDispatcher(EndpointCallable endpointCall)
        : endpointCall_(std::move(endpointCall)) {}

    /// @brief Convert a dotted JSON-RPC method name to a slash-style UDS method.
    /// @param dotted The dotted method name (e.g. "server.info").
    /// @return The slash-style method name (e.g. "server/info").
    static std::string dottedToSlash(const std::string& dotted) {
        std::string result = dotted;
        for (auto& c : result) {
            if (c == '.') c = '/';
        }
        return result;
    }

    /// @brief Convert a slash-style UDS method name to a dotted JSON-RPC method.
    /// @param slash The slash-style method name (e.g. "server/info").
    /// @return The dotted method name (e.g. "server.info").
    static std::string slashToDotted(const std::string& slash) {
        std::string result = slash;
        for (auto& c : result) {
            if (c == '/') c = '.';
        }
        return result;
    }

    /// @brief Dispatch a JSON-RPC 2.0 request.
    /// @param json The raw JSON-RPC request string.
    /// @return The JSON-RPC 2.0 response string.
    std::string dispatch(std::string_view json) {
        auto parsed = parseJson(json);
        if (!parsed || !parsed->isObject()) {
            return buildJsonRpcError(klippy::JsonValue{}, 
                static_cast<int>(RpcErrorCode::ParseError),
                "Parse error");
        }

        const auto& obj = parsed->asObject();

        // Check jsonrpc version
        auto it = obj.find("jsonrpc");
        if (it == obj.end() || !it->second.isString() || it->second.asString() != "2.0") {
            return buildJsonRpcError(klippy::JsonValue{},
                static_cast<int>(RpcErrorCode::InvalidRequest),
                "Invalid Request: missing or wrong jsonrpc version");
        }

        // Get method
        auto methodIt = obj.find("method");
        if (methodIt == obj.end() || !methodIt->second.isString()) {
            return buildJsonRpcError(klippy::JsonValue{},
                static_cast<int>(RpcErrorCode::InvalidRequest),
                "Invalid Request: missing method");
        }
        std::string method = methodIt->second.asString();

        // Get params (optional, defaults to empty object)
        klippy::JsonValue params = klippy::JsonValue(std::map<std::string, klippy::JsonValue>{});
        auto paramsIt = obj.find("params");
        if (paramsIt != obj.end()) {
            params = paramsIt->second;
        }

        // Get id (optional — if missing, it's a notification)
        klippy::JsonValue id = klippy::JsonValue{};
        bool hasId = false;
        auto idIt = obj.find("id");
        if (idIt != obj.end()) {
            id = idIt->second;
            hasId = true;
        }

        // Handle special methods locally
        if (method == "server.connection.identify" || method == "server.websocket.id") {
            if (hasId) {
                std::map<std::string, klippy::JsonValue> result;
                result["connection_id"] = klippy::JsonValue(static_cast<int64_t>(nextConnectionId_++));
                result["websocket_id"] = klippy::JsonValue(static_cast<int64_t>(nextConnectionId_));
                return buildJsonRpcSuccess(id, klippy::JsonValue(result));
            }
            return "";
        }

        // Convert dotted method to slash-style
        std::string slashMethod = dottedToSlash(method);

        // Dispatch to endpoint
        try {
            klippy::JsonValue result = endpointCall_(slashMethod, params);

            // Check if the result is an error
            if (result.isObject() && result.has("error")) {
                const auto& errVal = *result.find("error");
                int code = 500;
                std::string message = "Internal error";
                if (errVal.isString()) {
                    message = errVal.asString();
                } else if (errVal.isObject()) {
                    if (errVal.has("message") && errVal.find("message")->isString())
                        message = errVal.find("message")->asString();
                    if (errVal.has("code") && errVal.find("code")->isInt())
                        code = static_cast<int>(errVal.find("code")->asInt());
                }
                if (hasId) {
                    return buildJsonRpcError(id, code, message);
                }
                return "";
            }

            // Unwrap UDS-style { "result": ... } envelope.  The endpoint
            // callable returns UDS format, but JSON-RPC wraps the inner
            // value directly in its own "result" field.  Without unwrapping,
            // the client gets result.result.X (double-wrapped).
            // Copy first to avoid self-assignment (find returns a pointer
            // into result's own map, which would be destroyed mid-copy).
            if (result.isObject() && result.has("result")) {
                klippy::JsonValue inner = *result.find("result");
                result = std::move(inner);
            }

            if (hasId) {
                return buildJsonRpcSuccess(id, result);
            }
            // Notification — no response
            return "";
        } catch (const klippy::EndpointError& e) {
            if (hasId) {
                return buildJsonRpcError(id,
                    static_cast<int>(RpcErrorCode::InvalidParams), e.what());
            }
            return "";
        } catch (const std::exception& e) {
            if (hasId) {
                return buildJsonRpcError(id,
                    static_cast<int>(RpcErrorCode::InternalError), e.what());
            }
            return "";
        }
    }

    /// @brief Dispatch a JSON-RPC 2.0 request from an already-parsed JsonValue.
    /// @param frame The parsed JSON-RPC request object.
    /// @return The JSON-RPC 2.0 response string, or empty for notifications.
    std::string dispatchParsed(const klippy::JsonValue& frame) {
        return dispatch(frame.dump());
    }

private:
    EndpointCallable endpointCall_;
    int64_t nextConnectionId_ = 1;
};

} // namespace tether::klipper::http

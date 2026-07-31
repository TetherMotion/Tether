/**
 * @file CommandTable.hpp
 * @brief Dispatch table mapping msgids to typed command/response handlers.
 *
 * @details
 * The CommandTable binds a DataDictionary to runtime handlers. On the device
 * side, each command msgid is associated with a handler callable that receives
 * the decoded parameter values. On the host side, each response msgid is
 * associated with a handler that receives decoded response values.
 *
 * Encoding/decoding of message content (msgid + parameters) to/from a content
 * byte buffer is also provided here, using the DataDictionary's format specs
 * and the VLQ/ParameterFormat primitives.
 */

#pragma once

#include "tether/klipper/protocol/DataDictionary.hpp"
#include "tether/klipper/protocol/Vlq.hpp"
#include "tether/klipper/protocol/ParameterFormat.hpp"

#include <cstdint>
#include <functional>
#include <vector>
#include <string>
#include <unordered_map>
#include <span>
#include <optional>

namespace tether::klipper::protocol {

/// @brief A decoded parameter value (integer or string/buffer bytes).
struct ParamValue {
    int32_t integer = 0;            ///< Valid for integer types
    std::vector<uint8_t> bytes;     ///< Valid for string/buffer types
    bool isInteger = true;
};

/// @brief A decoded message: msgid + ordered parameter values.
struct DecodedMessage {
    uint16_t msgid = 0;
    std::vector<ParamValue> params;
};

/// @brief Command handler signature (device side). Receives decoded params.
using CommandHandler = std::function<void(const std::vector<ParamValue>&)>;

/// @brief Response handler signature (host side). Receives decoded params.
using ResponseHandler = std::function<void(const std::vector<ParamValue>&)>;

/**
 * @brief Encode a message (msgid + parameters) into a content byte buffer.
 *
 * @param dict   Data dictionary (for parameter types).
 * @param msgid  Message id.
 * @param params Parameter values (must match the format spec arity).
 * @param out    Output content buffer (appended to).
 * @return true on success.
 */
bool encodeMessage(const DataDictionary& dict, uint16_t msgid,
                   std::span<const ParamValue> params, std::vector<uint8_t>& out);

/**
 * @brief Decode a content buffer into one or more messages.
 *
 * Reads msgid + parameters repeatedly until the content is exhausted.
 *
 * @param dict    Data dictionary.
 * @param content Content bytes.
 * @return Decoded messages (may be more than one for host->MCU blocks).
 */
std::vector<DecodedMessage> decodeMessages(const DataDictionary& dict,
                                           std::span<const uint8_t> content);

/**
 * @brief Dispatch table binding msgids to handlers.
 *
 * The device side registers CommandHandlers; the host side registers
 * ResponseHandlers. Dispatch is keyed by msgid.
 */
class CommandTable {
public:
    explicit CommandTable(const DataDictionary& dict) : dict_(dict) {}

    /// @brief Register a command handler (device side).
    void registerCommand(uint16_t msgid, CommandHandler handler);

    /// @brief Register a response handler (host side).
    void registerResponse(uint16_t msgid, ResponseHandler handler);

    /// @brief Dispatch a decoded command (device side). No-op if unregistered.
    void dispatchCommand(const DecodedMessage& msg) const;

    /// @brief Dispatch a decoded response (host side). No-op if unregistered.
    void dispatchResponse(const DecodedMessage& msg) const;

    /// @return The bound data dictionary.
    const DataDictionary& dictionary() const { return dict_; }

    /// @brief Clear all handlers.
    void clear() { commandHandlers_.clear(); responseHandlers_.clear(); }

private:
    const DataDictionary& dict_;
    std::unordered_map<uint16_t, CommandHandler> commandHandlers_;
    std::unordered_map<uint16_t, ResponseHandler> responseHandlers_;
};

} // namespace tether::klipper::protocol

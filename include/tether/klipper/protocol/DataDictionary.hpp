/**
 * @file DataDictionary.hpp
 * @brief Klipper data dictionary: command/response/enum/constant registry.
 *
 * @details
 * The data dictionary maps human-readable command and response format strings
 * to integer message IDs (msgids), and holds enumerations (pin/spi_bus/etc.)
 * and constants (CLOCK_FREQ, SERIAL_BAUD, etc.). Both ends of a connection
 * must agree on the dictionary; the host downloads it from the device via the
 * `identify` handshake, while the device builds it from code-as-config
 * declarations (see config/KlipperConfig.hpp).
 *
 * Message IDs are assigned sequentially starting at kFirstDynamicMsgId (2);
 * msgids 0 (identify_response) and 1 (identify) are hard-coded. Commands,
 * responses, and outputs share a single sequential ID space.
 *
 * The dictionary can be serialised to/from JSON (the on-wire format is a
 * zlib-compressed JSON string served in chunks by the `identify` command). For
 * tether_klipper the JSON is produced from code-as-config; compression is
 * optional and minimal so no external zlib dependency is required for the
 * common in-process/loopback case. A real-zlib codec is provided when
 * TETHER_KLIPPER_HAS_ZLIB is defined (enabled for interop with real Klipper
 * hosts).
 */

#pragma once

#include "tether/klipper/protocol/Constants.hpp"
#include "tether/klipper/protocol/ParameterFormat.hpp"

#include <cstdint>
#include <string>
#include <string_view>
#include <vector>
#include <unordered_map>
#include <map>
#include <optional>
#include <variant>

namespace tether::klipper::protocol {

/// @brief An enumeration entry: either a simple value or a range [start, count].
struct EnumEntry {
    std::string key;
    /// For a simple enum: just `start`. For a range enum: [start, count].
    uint32_t start = 0;
    uint32_t count = 1;  ///< 1 = simple enum, >1 = range enum
};

/// @brief A named enumeration (e.g. "pin", "spi_bus").
struct Enumeration {
    std::string name;
    std::vector<EnumEntry> entries;
};

/// @brief A constant value (integer or string).
using ConstantValue = std::variant<int64_t, std::string>;

/// @brief Direction of a message in the dictionary.
enum class MessageDirection : uint8_t {
    Command,  ///< host -> MCU
    Response, ///< MCU -> host
    Output,   ///< free-form debug (MCU -> host)
};

/// @brief A registered message (command/response/output).
struct MessageEntry {
    uint16_t msgid = 0;             ///< Wire msgid
    MessageDirection direction;
    FormatSpec format;              ///< Parsed format string
};

/**
 * @brief The data dictionary: maps format strings to msgids and holds
 *        enumerations and constants.
 *
 * Built on the device side from code-as-config declarations; downloaded and
 * parsed on the host side from the JSON served by the `identify` handshake.
 */
class DataDictionary {
public:
    /// @return The dictionary version string (e.g. "tether_klipper-1.0.0").
    const std::string& version() const { return version_; }
    void setVersion(std::string v) { version_ = std::move(v); }

    /// @return The application name ("tether_klipper").
    const std::string& app() const { return app_; }
    void setApp(std::string a) { app_ = std::move(a); }

    /// @return Build/toolchain version string.
    const std::string& buildVersions() const { return buildVersions_; }
    void setBuildVersions(std::string v) { buildVersions_ = std::move(v); }

    // --- Message registration (device side, code-as-config) ----------------

    /**
     * @brief Register a command format string and assign it the next msgid.
     * @return The assigned msgid, or 0 if the format is invalid / already present.
     */
    uint16_t addCommand(std::string_view formatStr);

    /**
     * @brief Register a response format string and assign it the next msgid.
     */
    uint16_t addResponse(std::string_view formatStr);

    /**
     * @brief Register an output format string and assign it the next msgid.
     */
    uint16_t addOutput(std::string_view formatStr);

    // --- Lookups ------------------------------------------------------------

    /// @return The message entry for a msgid, or nullptr.
    const MessageEntry* lookupMsgid(uint16_t msgid) const;

    /// @return The msgid for a command format string, or std::nullopt.
    std::optional<uint16_t> lookupCommand(std::string_view formatStr) const;

    /// @return The msgid for a response format string, or std::nullopt.
    std::optional<uint16_t> lookupResponse(std::string_view formatStr) const;

    /// @return The msgid for an output format string, or std::nullopt.
    std::optional<uint16_t> lookupOutput(std::string_view formatStr) const;

    /// @return All registered messages (ordered by msgid).
    const std::map<uint16_t, MessageEntry>& messages() const { return messages_; }

    // --- Enumerations -------------------------------------------------------

    /// @brief Add a simple enumeration entry (key -> value).
    void addEnumValue(std::string_view enumName, std::string_view key, uint32_t value);

    /// @brief Add a range enumeration entry (key -> [start, start+count-1]).
    void addEnumRange(std::string_view enumName, std::string_view key,
                      uint32_t start, uint32_t count);

    /// @return The enumeration named @p enumName, or nullptr.
    const Enumeration* lookupEnum(std::string_view enumName) const;

    /// @brief Resolve a string value to its integer for an enumeration.
    ///        Matches by exact key or by range expansion (e.g. "PA3" -> 3).
    std::optional<uint32_t> resolveEnum(std::string_view enumName, std::string_view key) const;

    /// @return All enumerations.
    const std::vector<Enumeration>& enumerations() const { return enumerations_; }

    // --- Constants ----------------------------------------------------------

    /// @brief Add an integer constant.
    void addConstant(std::string_view name, int64_t value);

    /// @brief Add a string constant.
    void addConstantString(std::string_view name, std::string_view value);

    /// @return The constant value, or std::nullopt.
    std::optional<ConstantValue> lookupConstant(std::string_view name) const;

    /// @return All constants.
    const std::map<std::string, ConstantValue>& constants() const { return constants_; }

    // --- Serialisation ------------------------------------------------------

    /**
     * @brief Serialise the dictionary to a JSON string (uncompressed).
     *
     * The format matches the protocol's data-dictionary JSON schema:
     *   { "app", "version", "build_versions", "license",
     *     "commands": {fmt -> msgid}, "responses": {fmt -> msgid},
     *     "output": {fmt -> msgid}, "config": {name -> value},
     *     "enumerations": {name -> {key -> value | [start,count]}} }
     */
    std::string toJson() const;

    /**
     * @brief Parse a JSON dictionary string into this object (replacing
     *        current contents).
     * @return true on success.
     */
    bool fromJson(std::string_view json);

    /**
     * @brief Compress the JSON to the wire format (zlib deflate when
     *        TETHER_KLIPPER_HAS_ZLIB is defined; otherwise stores the raw
     *        JSON bytes prefixed with a 1-byte marker).
     * @return Compressed bytes.
     */
    std::vector<uint8_t> toWire() const;

    /**
     * @brief Decompress the wire-format bytes back to JSON.
     * @return JSON string, or empty on error.
     */
    static std::string fromWire(std::span<const uint8_t> wire);

    /// @return The next msgid that would be assigned (for testing/inspection).
    uint16_t nextMsgid() const { return nextMsgid_; }

private:
    uint16_t addMessage(std::string_view formatStr, MessageDirection dir);

    std::string app_{"tether_klipper"};
    std::string version_{"1.0.0"};
    std::string buildVersions_{"tether_klipper"};
    std::string license_{"MIT"};

    uint16_t nextMsgid_ = kFirstDynamicMsgId;
    std::map<uint16_t, MessageEntry> messages_;                 // msgid -> entry
    std::unordered_map<std::string, uint16_t> commandIndex_;   // fmt -> msgid
    std::unordered_map<std::string, uint16_t> responseIndex_;  // fmt -> msgid
    std::unordered_map<std::string, uint16_t> outputIndex_;    // fmt -> msgid

    std::vector<Enumeration> enumerations_;
    std::unordered_map<std::string, size_t> enumIndex_;         // name -> index

    std::map<std::string, ConstantValue> constants_;
};

} // namespace tether::klipper::protocol

/**
 * @file KlipperConfig.hpp
 * @brief Code-as-config builder for the Klipper data dictionary and device.
 *
 * @details
 * Following Tether's "code-as-config" approach (like the EtherCAT stack), the
 * KlipperConfig builder lets the device declare its commands, responses,
 * enumerations, and constants in code rather than external config files. The
 * builder constructs a DataDictionary that is served to the host via the
 * identify handshake.
 *
 * Usage:
 *   KlipperConfig config;
 *   config.addCommand("queue_step oid=%c interval=%u count=%hu add=%hi");
 *   config.addResponse("clock clock=%u");
 *   config.addConstant("CLOCK_FREQ", 180000000);
 *   config.addEnumRange("pin", "PA0", 0, 16);
 *   DataDictionary dict = config.build();
 */

#pragma once

#include "tether/klipper/protocol/DataDictionary.hpp"

#include <string>
#include <string_view>
#include <vector>
#include <functional>

namespace tether::klipper::config {

/**
 * @brief Code-as-config builder for the data dictionary.
 *
 * Declares commands, responses, outputs, enumerations, and constants in code.
 * The build() method produces a DataDictionary ready to be served by the
 * device.
 */
class KlipperConfig {
public:
    KlipperConfig() {
        dict_.setApp("tether_klipper");
        dict_.setVersion("1.0.0");
        dict_.setBuildVersions("tether_klipper");
    }

    /// @brief Set the application name.
    KlipperConfig& app(std::string_view name) { dict_.setApp(std::string(name)); return *this; }

    /// @brief Set the version string.
    KlipperConfig& version(std::string_view v) { dict_.setVersion(std::string(v)); return *this; }

    /// @brief Set the build versions string.
    KlipperConfig& buildVersions(std::string_view v) { dict_.setBuildVersions(std::string(v)); return *this; }

    /// @brief Register a command format string.
    KlipperConfig& addCommand(std::string_view fmt) { dict_.addCommand(fmt); return *this; }

    /// @brief Register a response format string.
    KlipperConfig& addResponse(std::string_view fmt) { dict_.addResponse(fmt); return *this; }

    /// @brief Register an output format string.
    KlipperConfig& addOutput(std::string_view fmt) { dict_.addOutput(fmt); return *this; }

    /// @brief Add an integer constant.
    KlipperConfig& addConstant(std::string_view name, int64_t value) {
        dict_.addConstant(name, value);
        return *this;
    }

    /// @brief Add a string constant.
    KlipperConfig& addConstantString(std::string_view name, std::string_view value) {
        dict_.addConstantString(name, value);
        return *this;
    }

    /// @brief Add a simple enumeration value.
    KlipperConfig& addEnumValue(std::string_view enumName, std::string_view key, uint32_t value) {
        dict_.addEnumValue(enumName, key, value);
        return *this;
    }

    /// @brief Add a range enumeration (key -> [start, start+count-1]).
    KlipperConfig& addEnumRange(std::string_view enumName, std::string_view key,
                                uint32_t start, uint32_t count) {
        dict_.addEnumRange(enumName, key, start, count);
        return *this;
    }

    /// @brief Build the data dictionary.
    protocol::DataDictionary build() { return dict_; }

    /// @return The underlying dictionary (for inspection).
    const protocol::DataDictionary& dictionary() const { return dict_; }

private:
    protocol::DataDictionary dict_;
};

} // namespace tether::klipper::config

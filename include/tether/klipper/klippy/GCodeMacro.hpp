#pragma once

/// @file GCodeMacro.hpp
/// @brief G-code macro definition and registry

#include <algorithm>
#include <map>
#include <string>
#include <vector>

namespace tether::klipper::klippy {

// ============================================================================
// G-code Macro System
// ============================================================================

/// @brief A G-code macro definition.
struct GcodeMacro {
    std::string name;                           ///< Macro name (e.g. "START_PRINT")
    std::string gcode;                          ///< G-code template
    std::map<std::string, std::string> defaults; ///< Default parameter values
    std::string description;                    ///< Help text
};

/// @brief G-code macro registry and expansion.
class GcodeMacroRegistry {
public:
    /// @brief Register a macro.
    void registerMacro(const GcodeMacro& macro) {
        std::string upperName = toUpper(macro.name);
        macros_[upperName] = macro;
    }

    /// @brief Unregister a macro.
    void unregisterMacro(const std::string& name) {
        std::string upperName = toUpper(name);
        macros_.erase(upperName);
    }

    /// @brief Check if a macro exists.
    bool hasMacro(const std::string& name) const {
        std::string upperName = toUpper(name);
        return macros_.find(upperName) != macros_.end();
    }

    /// @brief Get a macro by name.
    const GcodeMacro* getMacro(const std::string& name) const {
        std::string upperName = toUpper(name);
        auto it = macros_.find(upperName);
        return it == macros_.end() ? nullptr : &it->second;
    }

    /// @brief List all registered macros.
    std::vector<std::string> listMacros() const {
        std::vector<std::string> result;
        for (const auto& [name, _] : macros_) result.push_back(name);
        return result;
    }

    /// @brief Expand a macro call into G-code.
    /// @param name Macro name.
    /// @param params Parameters from the G-code line (e.g. TEMP=200).
    /// @return Expanded G-code string, or empty if macro not found.
    std::string expandMacro(const std::string& name,
                            const std::map<std::string, std::string>& params) const {
        std::string upperName = toUpper(name);
        auto it = macros_.find(upperName);
        if (it == macros_.end()) return {};
        const GcodeMacro& macro = it->second;

        // Merge defaults with provided params
        std::map<std::string, std::string> allParams = macro.defaults;
        for (const auto& [k, v] : params) allParams[k] = v;

        // Expand template: replace {param} with values
        std::string result = macro.gcode;
        for (const auto& [key, value] : allParams) {
            std::string placeholder = "{" + key + "}";
            size_t pos = 0;
            while ((pos = result.find(placeholder, pos)) != std::string::npos) {
                result.replace(pos, placeholder.size(), value);
                pos += value.size();
            }
        }
        return result;
    }

private:
    static std::string toUpper(const std::string& s) {
        std::string result = s;
        std::transform(result.begin(), result.end(), result.begin(),
                       [](unsigned char c) { return std::toupper(c); });
        return result;
    }

    std::map<std::string, GcodeMacro> macros_;
};

} // namespace tether::klipper::klippy

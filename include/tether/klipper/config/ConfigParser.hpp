/**
 * @file ConfigParser.hpp
 * @brief printer.cfg configuration file parser.
 *
 * Parses Klipper-style INI configuration files with sections like:
 *   [stepper_x]
 *   step_pin: PA0
 *   dir_pin: PA1
 *   ...
 */

#pragma once

#include <cstdint>
#include <fstream>
#include <map>
#include <optional>
#include <sstream>
#include <string>
#include <string_view>
#include <vector>

namespace tether::klipper::config {

/**@brief A configuration section (e.g. [stepper_x]). */
struct ConfigSection {
    std::string name; /**< Section name (without brackets) */
    std::map<std::string, std::string> values; /**< Key-value pairs */
    std::vector<std::string> multiValues; /**< For multi-line values */

    /**
     * @brief Check if a key exists.
     * @param key Option name within the section.
     * @return true when the key is present.
     */
    bool has(const std::string& key) const {
        return values.count(key) > 0;
    }

    /**
     * @brief Get a value as string.
     * @param key Option name within the section.
     * @param defaultVal Value returned when the key is absent.
     * @return Raw string value.
     */
    std::string get(const std::string& key, const std::string& defaultVal = "") const {
        auto it = values.find(key);
        return it == values.end() ? defaultVal : it->second;
    }

    /**
     * @brief Get a value as integer.
     * @param key Option name within the section.
     * @param defaultVal Value returned when absent or unparseable.
     * @return Parsed integer.
     */
    int64_t getInt(const std::string& key, int64_t defaultVal = 0) const {
        auto it = values.find(key);
        if (it == values.end()) return defaultVal;
        try { return std::stoll(it->second); }
        catch (...) { return defaultVal; }
    }

    /**
     * @brief Get a value as double.
     * @param key Option name within the section.
     * @param defaultVal Value returned when absent or unparseable.
     * @return Parsed floating-point value.
     */
    double getDouble(const std::string& key, double defaultVal = 0.0) const {
        auto it = values.find(key);
        if (it == values.end()) return defaultVal;
        try { return std::stod(it->second); }
        catch (...) { return defaultVal; }
    }

    /**
     * @brief Get a value as boolean.
     * @param key Option name within the section.
     * @param defaultVal Value returned when the key is absent.
     * @return Parsed boolean (true/1/yes).
     */
    bool getBool(const std::string& key, bool defaultVal = false) const {
        auto it = values.find(key);
        if (it == values.end()) return defaultVal;
        std::string v = it->second;
        return v == "true" || v == "True" || v == "1" || v == "yes" || v == "Yes";
    }

    /**
     * @brief Get a comma-separated list.
     * @param key Option name within the section.
     * @param defaultVal Comma-separated fallback when the key is absent.
     * @return Trimmed list elements.
     */
    std::vector<std::string> getList(const std::string& key,
                                      const std::string& defaultVal = "") const {
        auto it = values.find(key);
        if (it == values.end()) {
            if (defaultVal.empty()) return {};
            std::vector<std::string> result;
            std::stringstream ss(defaultVal);
            std::string item;
            while (std::getline(ss, item, ',')) {
                // Trim whitespace
                auto start = item.find_first_not_of(" \t");
                auto end = item.find_last_not_of(" \t");
                if (start != std::string::npos) {
                    result.push_back(item.substr(start, end - start + 1));
                }
            }
            return result;
        }
        std::vector<std::string> result;
        std::stringstream ss(it->second);
        std::string item;
        while (std::getline(ss, item, ',')) {
            auto start = item.find_first_not_of(" \t");
            auto end = item.find_last_not_of(" \t");
            if (start != std::string::npos) {
                result.push_back(item.substr(start, end - start + 1));
            }
        }
        return result;
    }
};

/**@brief Configuration file parser. */
class ConfigParser {
public:
    /**
     * @brief Parse a configuration string.
     * Supports include directives ([include file.cfg]), variable substitution
     * ({variable}), and multi-line values (backslash continuation).
     * @param content Configuration text.
     * @return true when at least one section was parsed.
     */
    bool parse(const std::string& content) {
        sections_.clear();
        includedFiles_.clear();
        // Note: variables_ are NOT cleared so setVariable() can be called before parse()
        parseContent(content, "");
        applyVariableSubstitution();
        return !sections_.empty();
    }

    /**
     * @brief Parse a configuration file.
     * @param path Path of the file to read.
     * @return true when the file was read and parsed.
     */
    bool parseFile(const std::string& path) {
        std::ifstream file(path);
        if (!file.is_open()) return false;
        std::stringstream ss;
        ss << file.rdbuf();
        // Extract directory for relative includes
        std::string dir;
        auto lastSlash = path.find_last_of('/');
        if (lastSlash != std::string::npos) {
            dir = path.substr(0, lastSlash + 1);
        }
        sections_.clear();
        includedFiles_.clear();
        // Note: variables_ are NOT cleared so setVariable() can be called before parseFile()
        parseContent(ss.str(), dir);
        applyVariableSubstitution();
        return !sections_.empty();
    }

    /**
     * @brief Get all sections.
     * @return Parsed sections in file order.
     */
    const std::vector<ConfigSection>& sections() const { return sections_; }

    /**
     * @brief Get sections by name.
     * @param name Section name (e.g. "stepper_x").
     * @return Every section with that name.
     */
    std::vector<const ConfigSection*> getSections(const std::string& name) const {
        std::vector<const ConfigSection*> result;
        for (const auto& s : sections_) {
            if (s.name == name) result.push_back(&s);
        }
        return result;
    }

    /**
     * @brief Get the first section with a given name.
     * @param name Section name.
     * @return Section, or nullptr when absent.
     */
    const ConfigSection* getSection(const std::string& name) const {
        for (const auto& s : sections_) {
            if (s.name == name) return &s;
        }
        return nullptr;
    }

    /**
     * @brief Check if a section exists.
     * @param name Section name.
     * @return true when the section is present.
     */
    bool hasSection(const std::string& name) const {
        return getSection(name) != nullptr;
    }

    /**
     * @brief Get all section names.
     * @return Unique section names in first-seen order.
     */
    std::vector<std::string> sectionNames() const {
        std::vector<std::string> names;
        for (const auto& s : sections_) {
            if (std::find(names.begin(), names.end(), s.name) == names.end()) {
                names.push_back(s.name);
            }
        }
        return names;
    }

    /**
     * @brief Get the list of included files.
     * @return Paths of files pulled in via [include ...].
     */
    const std::vector<std::string>& includedFiles() const { return includedFiles_; }

    /**
     * @brief Set a variable for substitution.
     * @param key Variable name (referenced as {key}).
     * @param value Replacement text.
     */
    void setVariable(const std::string& key, const std::string& value) {
        variables_[key] = value;
    }

private:
    std::vector<ConfigSection> sections_;
    std::map<std::string, std::string> variables_;
    std::vector<std::string> includedFiles_;

    /**@brief Parse content, handling includes and multi-line values. */
    void parseContent(const std::string& content, const std::string& baseDir) {
        std::stringstream ss(content);
        std::string line;
        ConfigSection* currentSection = nullptr;
        std::string pendingKey;
        std::string pendingValue;
        bool inMultiLine = false;

        while (std::getline(ss, line)) {
            // If we're in a multi-line value, append
            if (inMultiLine) {
                // Strip comments
                auto hash = line.find('#');
                if (hash != std::string::npos) {
                    line = line.substr(0, hash);
                }
                // Trim
                auto ls = line.find_first_not_of(" \t\r\n");
                auto le = line.find_last_not_of(" \t\r\n");
                if (ls != std::string::npos) {
                    line = line.substr(ls, le - ls + 1);
                } else {
                    line = "";
                }
                // Check for continuation
                if (!line.empty() && line.back() == '\\') {
                    pendingValue += line.substr(0, line.size() - 1);
                    continue;
                }
                pendingValue += line;
                if (currentSection) {
                    currentSection->values[pendingKey] = pendingValue;
                }
                inMultiLine = false;
                continue;
            }

            // Strip comments
            auto hash = line.find('#');
            if (hash != std::string::npos) {
                line = line.substr(0, hash);
            }

            // Trim whitespace
            auto start = line.find_first_not_of(" \t\r\n");
            if (start == std::string::npos) continue; // Empty line
            auto end = line.find_last_not_of(" \t\r\n");
            line = line.substr(start, end - start + 1);

            if (line.empty()) continue;

            // Check for section header
            if (line[0] == '[' && line.back() == ']') {
                std::string name = line.substr(1, line.size() - 2);
                // Trim whitespace from name
                auto ns = name.find_first_not_of(" \t");
                auto ne = name.find_last_not_of(" \t");
                if (ns != std::string::npos) {
                    name = name.substr(ns, ne - ns + 1);
                }

                // Handle include directive
                if (name.substr(0, 8) == "include ") {
                    std::string includePath = name.substr(8);
                    // Trim
                    auto ips = includePath.find_first_not_of(" \t");
                    auto ipe = includePath.find_last_not_of(" \t");
                    if (ips != std::string::npos) {
                        includePath = includePath.substr(ips, ipe - ips + 1);
                    }
                    // Resolve relative to base directory
                    std::string fullPath = includePath;
                    if (!baseDir.empty() && includePath[0] != '/') {
                        fullPath = baseDir + includePath;
                    }
                    // Avoid circular includes
                    if (std::find(includedFiles_.begin(), includedFiles_.end(),
                                  fullPath) == includedFiles_.end()) {
                        includedFiles_.push_back(fullPath);
                        std::ifstream incFile(fullPath);
                        if (incFile.is_open()) {
                            std::stringstream incss;
                            incss << incFile.rdbuf();
                            std::string incDir;
                            auto lastSlash = fullPath.find_last_of('/');
                            if (lastSlash != std::string::npos) {
                                incDir = fullPath.substr(0, lastSlash + 1);
                            }
                            parseContent(incss.str(), incDir);
                        }
                    }
                    currentSection = nullptr;
                    continue;
                }

                // Handle variable sections [section_name] with variables
                sections_.push_back({name, {}, {}});
                currentSection = &sections_.back();
                continue;
            }

            // Parse key: value or key = value
            if (!currentSection) continue; // Key-value outside section
            // Prefer '=' over ':' when both are present (e.g. URLs contain ':')
            auto eqPos = line.find('=');
            auto colonPos = line.find(':');
            size_t sep;
            if (eqPos != std::string::npos &&
                (colonPos == std::string::npos || eqPos < colonPos)) {
                sep = eqPos;
            } else if (colonPos != std::string::npos) {
                sep = colonPos;
            } else {
                continue;
            }
            std::string key = line.substr(0, sep);
            std::string value = line.substr(sep + 1);
            // Trim
            auto ks = key.find_first_not_of(" \t");
            auto ke = key.find_last_not_of(" \t");
            if (ks != std::string::npos) key = key.substr(ks, ke - ks + 1);
            auto vs = value.find_first_not_of(" \t");
            auto ve = value.find_last_not_of(" \t");
            if (vs != std::string::npos) {
                value = value.substr(vs, ve - vs + 1);
            } else {
                value = "";
            }

            // Check for multi-line continuation
            if (!value.empty() && value.back() == '\\') {
                pendingKey = key;
                pendingValue = value.substr(0, value.size() - 1);
                inMultiLine = true;
                continue;
            }

            currentSection->values[key] = value;
        }
    }

    /**@brief Apply {variable} substitution to all values. */
    void applyVariableSubstitution() {
        for (auto& section : sections_) {
            for (auto& [key, value] : section.values) {
                value = substituteVariables(value);
            }
        }
    }

    /**@brief Substitute {variable} references in a string. */
    std::string substituteVariables(const std::string& input) const {
        std::string result = input;
        size_t pos = 0;
        while ((pos = result.find('{', pos)) != std::string::npos) {
            auto end = result.find('}', pos);
            if (end == std::string::npos) break;
            std::string varName = result.substr(pos + 1, end - pos - 1);
            auto it = variables_.find(varName);
            if (it != variables_.end()) {
                result.replace(pos, end - pos + 1, it->second);
                pos += it->second.size();
            } else {
                pos = end + 1; // Skip unknown variable
            }
        }
        return result;
    }
};

// ============================================================================
// Config Validator — validates Klipper config sections
// ============================================================================

/**@brief Validation result for a single section. */
struct ConfigValidationResult {
    std::string sectionName;
    bool valid = true;
    std::vector<std::string> errors;
    std::vector<std::string> warnings;
};

/**
 * @brief Validates Klipper configuration sections.
 *
 * Checks that required keys are present and that values are within
 * reasonable ranges for each known Klipper section type.
 */
class ConfigValidator {
public:
    /**
     * @brief Validate all sections in a parsed config.
     * @param parser Parsed configuration to check.
     * @return One result per section, in file order.
     */
    std::vector<ConfigValidationResult> validate(const ConfigParser& parser) const {
        std::vector<ConfigValidationResult> results;
        for (const auto& section : parser.sections()) {
            results.push_back(validateSection(section));
        }
        return results;
    }

    /**
     * @brief Validate a single section.
     * @param section Section to check.
     * @return Validation result with errors and warnings.
     */
    ConfigValidationResult validateSection(const ConfigSection& section) const {
        ConfigValidationResult result;
        result.sectionName = section.name;

        if (section.name == "stepper_x" || section.name == "stepper_y" ||
            section.name == "stepper_z" || section.name == "stepper_z1" ||
            section.name == "stepper_z2" || section.name == "stepper_z3") {
            validateStepper(section, result);
        } else if (section.name == "extruder") {
            validateExtruder(section, result);
        } else if (section.name == "heater_bed") {
            validateHeaterBed(section, result);
        } else if (section.name == "fan") {
            validateFan(section, result);
        } else if (section.name == "probe") {
            validateProbe(section, result);
        } else if (section.name == "bed_mesh") {
            validateBedMesh(section, result);
        } else if (section.name == "printer") {
            validatePrinter(section, result);
        } else if (section.name == "mcu") {
            validateMcu(section, result);
        } else if (section.name == "idle_timeout") {
            validateIdleTimeout(section, result);
        } else if (section.name == "safe_z_home") {
            validateSafeZHome(section, result);
        } else if (section.name == "bed_tilt") {
            validateBedTilt(section, result);
        } else if (section.name == "bed_screws") {
            validateBedScrews(section, result);
        } else if (section.name == "screws_tilt_adjust") {
            validateScrewsTiltAdjust(section, result);
        } else if (section.name == "display") {
            validateDisplay(section, result);
        } else if (section.name.substr(0, 8) == "tmc2209 " ||
                   section.name.substr(0, 8) == "tmc2208 " ||
                   section.name.substr(0, 8) == "tmc2660 " ||
                   section.name.substr(0, 8) == "tmc5160 " ||
                   section.name.substr(0, 8) == "tmc2130 ") {
            validateTmc(section, result);
        } else if (section.name == "adxl345") {
            validateAdxl345(section, result);
        } else if (section.name == "bltouch") {
            validateBltouch(section, result);
        } else if (section.name == "output_pin") {
            validateOutputPin(section, result);
        } else if (section.name.substr(0, 5) == "heater") {
            validateHeaterGeneric(section, result);
        } else if (section.name == "multi_mcu") {
            validateMultiMcu(section, result);
        } else if (section.name.substr(0, 5) == "delta") {
            validateDelta(section, result);
        } else {
            // Unknown section — not an error, just a warning
            result.warnings.push_back("Unknown section type: [" + section.name + "]");
        }

        return result;
    }

    /**
     * @brief Check if all sections are valid.
     * @param results Validation results to inspect.
     * @return true when no result reports an error.
     */
    bool allValid(const std::vector<ConfigValidationResult>& results) const {
        for (const auto& r : results) {
            if (!r.valid) return false;
        }
        return true;
    }

    /**
     * @brief Get all errors as a single string.
     * @param results Validation results to format.
     * @return Human-readable error summary (empty when all valid).
     */
    std::string formatErrors(const std::vector<ConfigValidationResult>& results) const {
        std::ostringstream ss;
        for (const auto& r : results) {
            for (const auto& err : r.errors) {
                ss << "[" << r.sectionName << "] ERROR: " << err << "\n";
            }
            for (const auto& warn : r.warnings) {
                ss << "[" << r.sectionName << "] WARNING: " << warn << "\n";
            }
        }
        return ss.str();
    }

private:
    void validateStepper(const ConfigSection& s, ConfigValidationResult& r) const {
        requireKey(s, "step_pin", r);
        requireKey(s, "dir_pin", r);
        requireKey(s, "rotation_distance", r);
        if (s.has("microsteps")) {
            int ms = static_cast<int>(s.getInt("microsteps"));
            if (ms < 1 || ms > 256) {
                r.errors.push_back("microsteps must be between 1 and 256");
                r.valid = false;
            }
        }
        if (s.has("rotation_distance")) {
            double rd = s.getDouble("rotation_distance");
            if (rd <= 0) {
                r.errors.push_back("rotation_distance must be positive");
                r.valid = false;
            }
        }
        if (s.has("max_velocity")) {
            double mv = s.getDouble("max_velocity");
            if (mv <= 0) {
                r.errors.push_back("max_velocity must be positive");
                r.valid = false;
            }
        }
    }

    void validateExtruder(const ConfigSection& s, ConfigValidationResult& r) const {
        requireKey(s, "step_pin", r);
        requireKey(s, "dir_pin", r);
        requireKey(s, "rotation_distance", r);
        requireKey(s, "nozzle_diameter", r);
        requireKey(s, "filament_diameter", r);
        if (s.has("filament_diameter") && s.has("nozzle_diameter")) {
            double fd = s.getDouble("filament_diameter");
            double nd = s.getDouble("nozzle_diameter");
            if (fd <= 0 || nd <= 0) {
                r.errors.push_back("filament_diameter and nozzle_diameter must be positive");
                r.valid = false;
            }
        }
        validateHeaterCommon(s, r);
    }

    void validateHeaterBed(const ConfigSection& s, ConfigValidationResult& r) const {
        requireKey(s, "heater_pin", r);
        requireKey(s, "sensor_type", r);
        requireKey(s, "sensor_pin", r);
        validateHeaterCommon(s, r);
    }

    void validateHeaterGeneric(const ConfigSection& s, ConfigValidationResult& r) const {
        requireKey(s, "heater_pin", r);
        requireKey(s, "sensor_type", r);
        validateHeaterCommon(s, r);
    }

    void validateHeaterCommon(const ConfigSection& s, ConfigValidationResult& r) const {
        if (s.has("min_temp") && s.has("max_temp")) {
            double minT = s.getDouble("min_temp");
            double maxT = s.getDouble("max_temp");
            if (minT >= maxT) {
                r.errors.push_back("min_temp must be less than max_temp");
                r.valid = false;
            }
        }
        if (s.has("max_temp")) {
            double maxT = s.getDouble("max_temp");
            if (maxT > 500) {
                r.warnings.push_back("max_temp > 500C is unusually high");
            }
        }
    }

    void validateFan(const ConfigSection& s, ConfigValidationResult& r) const {
        requireKey(s, "pin", r);
        if (s.has("max_power")) {
            double mp = s.getDouble("max_power");
            if (mp < 0 || mp > 1.0) {
                r.errors.push_back("max_power must be between 0.0 and 1.0");
                r.valid = false;
            }
        }
        if (s.has("cycle_time")) {
            double ct = s.getDouble("cycle_time");
            if (ct <= 0) {
                r.errors.push_back("cycle_time must be positive");
                r.valid = false;
            }
        }
    }

    void validateProbe(const ConfigSection& s, ConfigValidationResult& r) const {
        requireKey(s, "z_offset", r);
        if (s.has("z_offset")) {
            double zo = s.getDouble("z_offset");
            if (zo < -10 || zo > 10) {
                r.warnings.push_back("z_offset outside typical range (-10 to 10)");
            }
        }
        if (s.has("sample_count")) {
            int sc = static_cast<int>(s.getInt("sample_count"));
            if (sc < 1 || sc > 20) {
                r.warnings.push_back("sample_count outside typical range (1-20)");
            }
        }
    }

    void validateBedMesh(const ConfigSection& s, ConfigValidationResult& r) const {
        requireKey(s, "mesh_min", r);
        requireKey(s, "mesh_max", r);
        requireKey(s, "probe_count", r);
        if (s.has("probe_count")) {
            auto pc = s.getList("probe_count");
            if (pc.size() != 2) {
                r.errors.push_back("probe_count must be two comma-separated values");
                r.valid = false;
            }
        }
        if (s.has("mesh_speed")) {
            double ms = s.getDouble("mesh_speed");
            if (ms <= 0) {
                r.errors.push_back("mesh_speed must be positive");
                r.valid = false;
            }
        }
    }

    void validatePrinter(const ConfigSection& s, ConfigValidationResult& r) const {
        requireKey(s, "kinematics", r);
        if (s.has("kinematics")) {
            std::string kin = s.get("kinematics");
            if (kin != "cartesian" && kin != "delta" && kin != "corexy" &&
                kin != "corexz" && kin != "hybrid_corexy" && kin != "hybrid_corexz" &&
                kin != "rotary_delta" && kin != "polar" && kin != "winch" &&
                kin != "none") {
                r.warnings.push_back("Unknown kinematics type: " + kin);
            }
        }
        requireKey(s, "max_velocity", r);
        requireKey(s, "max_accel", r);
        if (s.has("max_velocity")) {
            double mv = s.getDouble("max_velocity");
            if (mv <= 0 || mv > 10000) {
                r.warnings.push_back("max_velocity outside typical range");
            }
        }
        if (s.has("max_accel")) {
            double ma = s.getDouble("max_accel");
            if (ma <= 0 || ma > 100000) {
                r.warnings.push_back("max_accel outside typical range");
            }
        }
    }

    void validateMcu(const ConfigSection& s, ConfigValidationResult& r) const {
        requireKey(s, "serial", r);
        if (s.has("baud")) {
            int baud = static_cast<int>(s.getInt("baud"));
            if (baud != 250000 && baud != 115200 && baud != 57600 &&
                baud != 38400 && baud != 9600) {
                r.warnings.push_back("Non-standard baud rate: " + std::to_string(baud));
            }
        }
    }

    void validateIdleTimeout(const ConfigSection& s, ConfigValidationResult& r) const {
        if (s.has("timeout")) {
            int t = static_cast<int>(s.getInt("timeout"));
            if (t < 1) {
                r.errors.push_back("timeout must be at least 1 second");
                r.valid = false;
            }
        }
    }

    void validateSafeZHome(const ConfigSection& s, ConfigValidationResult& r) const {
        requireKey(s, "home_xy_position", r);
        requireKey(s, "z_hop", r);
    }

    void validateBedTilt(const ConfigSection& s, ConfigValidationResult& r) const {
        requireKey(s, "points", r);
    }

    void validateBedScrews(const ConfigSection& s, ConfigValidationResult& r) const {
        requireKey(s, "screws", r);
    }

    void validateScrewsTiltAdjust(const ConfigSection& s, ConfigValidationResult& r) const {
        requireKey(s, "screws", r);
        requireKey(s, "screw_thread", r);
    }

    void validateDisplay(const ConfigSection& s, ConfigValidationResult& r) const {
        requireKey(s, "display_type", r);
    }

    void validateTmc(const ConfigSection& s, ConfigValidationResult& r) const {
        requireKey(s, "uart_pin", r);
        requireKey(s, "run_current", r);
        if (s.has("run_current")) {
            double rc = s.getDouble("run_current");
            if (rc <= 0 || rc > 3000) {
                r.warnings.push_back("run_current outside typical range (0-3000mA)");
            }
        }
    }

    void validateAdxl345(const ConfigSection& s, ConfigValidationResult& r) const {
        requireKey(s, "cs_pin", r);
        requireKey(s, "spi_bus", r);
    }

    void validateBltouch(const ConfigSection& s, ConfigValidationResult& r) const {
        requireKey(s, "control_pin", r);
        requireKey(s, "sensor_pin", r);
    }

    void validateOutputPin(const ConfigSection& s, ConfigValidationResult& r) const {
        requireKey(s, "pin", r);
    }

    void validateMultiMcu(const ConfigSection& s, ConfigValidationResult& r) const {
        // Primary MCU section is just [mcu], secondary MCUs are [mcu name]
        // No specific required keys for multi_mcu coordinator
        (void)s;
    }

    void validateDelta(const ConfigSection& s, ConfigValidationResult& r) const {
        if (s.name == "delta_calibrate") {
            requireKey(s, "radius", r);
        }
    }

    void requireKey(const ConfigSection& s, const std::string& key,
                    ConfigValidationResult& r) const {
        if (!s.has(key)) {
            r.errors.push_back("Missing required key: " + key);
            r.valid = false;
        }
    }
};

} // namespace tether::klipper::config

#pragma once

/**
 * @file TmcDriverConfig.hpp
 * @brief TMC driver configuration manager
 */

#include <map>
#include <string>
#include <vector>

namespace tether::klipper::klippy {

/**@brief TMC driver parameters for a single axis. */
struct TmcDriverParams {
    double runCurrent = 800.0; /**< Run current in mA (M907/M908) */
    double holdCurrent = 400.0; /**< Hold current in mA (M909) */
    bool stealthChop = false; /**< StealthChop mode (M911) */
    double spreadThreshold = 0.0; /**< SpreadCycle threshold (M912) */
    int bumpSensitivity = 0; /**< Bump sensitivity (M913) */
    int diagPin = 0; /**< Diag pin (M914) */
};

/**@brief TMC driver configuration manager. */
class TmcDriverConfig {
public:
    /**
     * @brief Set run current for an axis (M907/M908).
     * @param axis       Axis name (e.g. "x", "e0").
     * @param currentMa  Run current in milliamps.
     */
    void setRunCurrent(const std::string& axis, double currentMa) {
        params_[axis].runCurrent = currentMa;
    }

    /**
     * @brief Set hold current for an axis (M909).
     * @param axis       Axis name.
     * @param currentMa  Hold current in milliamps.
     */
    void setHoldCurrent(const std::string& axis, double currentMa) {
        params_[axis].holdCurrent = currentMa;
    }

    /**
     * @brief Enable/disable StealthChop for an axis (M911).
     * @param axis    Axis name.
     * @param enable  true for StealthChop, false for SpreadCycle.
     */
    void setStealthChop(const std::string& axis, bool enable) {
        params_[axis].stealthChop = enable;
    }

    /**
     * @brief Set SpreadCycle threshold for an axis (M912).
     * @param axis       Axis name.
     * @param threshold  Velocity threshold (driver-specific units).
     */
    void setSpreadThreshold(const std::string& axis, double threshold) {
        params_[axis].spreadThreshold = threshold;
    }

    /**
     * @brief Set bump sensitivity for an axis (M913).
     * @param axis         Axis name.
     * @param sensitivity  StallGuard bump sensitivity value.
     */
    void setBumpSensitivity(const std::string& axis, int sensitivity) {
        params_[axis].bumpSensitivity = sensitivity;
    }

    /**
     * @brief Set diag pin for an axis (M914).
     * @param axis  Axis name.
     * @param diag  Diag pin identifier.
     */
    void setDiagPin(const std::string& axis, int diag) {
        params_[axis].diagPin = diag;
    }

    /**
     * @brief Get parameters for an axis.
     * @param axis  Axis name.
     * @return stored params, or a default-constructed set for axes never
     *         configured (returned reference is to a static default).
     */
    const TmcDriverParams& params(const std::string& axis) const {
        static const TmcDriverParams defaultParams;
        auto it = params_.find(axis);
        return it != params_.end() ? it->second : defaultParams;
    }

    /**
     * @brief Get all configured axes.
     * @return axis names that have at least one stored parameter.
     */
    std::vector<std::string> axes() const {
        std::vector<std::string> result;
        for (const auto& [axis, _] : params_) result.push_back(axis);
        return result;
    }

private:
    std::map<std::string, TmcDriverParams> params_;
};

} // namespace tether::klipper::klippy

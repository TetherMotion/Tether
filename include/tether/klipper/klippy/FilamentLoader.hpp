#pragma once

/// @file FilamentLoader.hpp
/// @brief Filament load/unload state and operations

#include <functional>
#include <map>
#include <sstream>
#include <string>

namespace tether::klipper::klippy {

/// @brief Filament load/unload state and operations.
class FilamentLoader {
public:
    /// @brief Set the load/unload callback.
    using FilamentCallback = std::function<void(int extruder, double length, double speed)>;

    void setLoadCallback(FilamentCallback cb) { loadCb_ = std::move(cb); }
    void setUnloadCallback(FilamentCallback cb) { unloadCb_ = std::move(cb); }
    void setPurgeCallback(FilamentCallback cb) { purgeCb_ = std::move(cb); }
    void setRetractCallback(FilamentCallback cb) { retractCb_ = std::move(cb); }

    /// @brief Load filament (M701).
    void loadFilament(int extruder) {
        if (loadCb_) loadCb_(extruder, loadLength_, loadSpeed_);
        filamentLoaded_[extruder] = true;
    }

    /// @brief Unload filament (M702).
    void unloadFilament(int extruder) {
        if (unloadCb_) unloadCb_(extruder, loadLength_, loadSpeed_);
        filamentLoaded_[extruder] = false;
    }

    /// @brief Load filament into tool (M703).
    void loadToTool(int tool) {
        if (loadCb_) loadCb_(tool, toolLoadLength_, loadSpeed_);
    }

    /// @brief Unload filament from tool (M704).
    void unloadFromTool(int tool) {
        if (unloadCb_) unloadCb_(tool, toolLoadLength_, loadSpeed_);
    }

    /// @brief Purge filament (M705).
    void purge(int extruder) {
        if (purgeCb_) purgeCb_(extruder, purgeLength_, purgeSpeed_);
    }

    /// @brief Retract filament (M706).
    void retract(int extruder) {
        if (retractCb_) retractCb_(extruder, retractLength_, retractSpeed_);
    }

    /// @brief Set filament sensor state (M707).
    void setSensorState(int sensor, bool enabled) {
        sensorEnabled_[sensor] = enabled;
    }

    /// @brief Report filament sensor state (M708).
    std::string reportSensorState() const {
        std::ostringstream ss;
        ss << "Filament sensors:";
        for (const auto& [sensor, enabled] : sensorEnabled_) {
            ss << " S" << sensor << ":" << (enabled ? "ON" : "OFF");
        }
        if (sensorEnabled_.empty()) ss << " none configured";
        return ss.str();
    }

    /// @brief Check if filament is loaded for an extruder.
    bool isLoaded(int extruder) const {
        auto it = filamentLoaded_.find(extruder);
        return it != filamentLoaded_.end() && it->second;
    }

    /// @brief Set load parameters.
    void setLoadLength(double length) { loadLength_ = length; }
    void setToolLoadLength(double length) { toolLoadLength_ = length; }
    void setLoadSpeed(double speed) { loadSpeed_ = speed; }
    void setPurgeLength(double length) { purgeLength_ = length; }
    void setPurgeSpeed(double speed) { purgeSpeed_ = speed; }
    void setRetractLength(double length) { retractLength_ = length; }
    void setRetractSpeed(double speed) { retractSpeed_ = speed; }

private:
    FilamentCallback loadCb_;
    FilamentCallback unloadCb_;
    FilamentCallback purgeCb_;
    FilamentCallback retractCb_;
    std::map<int, bool> filamentLoaded_;
    std::map<int, bool> sensorEnabled_;
    double loadLength_ = 50.0;
    double toolLoadLength_ = 20.0;
    double loadSpeed_ = 600.0;  // mm/min
    double purgeLength_ = 10.0;
    double purgeSpeed_ = 300.0;
    double retractLength_ = 5.0;
    double retractSpeed_ = 600.0;
};

} // namespace tether::klipper::klippy

#pragma once

/**
 * @file FilamentLoader.hpp
 * @brief Filament load/unload state and operations
 */

#include <functional>
#include <map>
#include <sstream>
#include <string>

namespace tether::klipper::klippy {

/**@brief Filament load/unload state and operations. */
class FilamentLoader {
public:
    /**@brief Set the load/unload callback. */
    using FilamentCallback = std::function<void(int extruder, double length, double speed)>;

    void setLoadCallback(FilamentCallback cb) { loadCb_ = std::move(cb); }
    void setUnloadCallback(FilamentCallback cb) { unloadCb_ = std::move(cb); }
    void setPurgeCallback(FilamentCallback cb) { purgeCb_ = std::move(cb); }
    void setRetractCallback(FilamentCallback cb) { retractCb_ = std::move(cb); }

    /**@brief Load filament (M701). */
    /**
     * @brief Feed filament into the given extruder.
     *
     * Queues a forward extrusion move of loadLength_ at loadSpeed_,
     * then marks the extruder loaded.
     * @param extruder  Extruder/tool index.
     */
    void loadFilament(int extruder) {
        if (loadCb_) loadCb_(extruder, loadLength_, loadSpeed_);
        filamentLoaded_[extruder] = true;
    }

    /**@brief Unload filament (M702). */
    /**
     * @brief Retract filament out of the given extruder.
     *
     * Queues a reverse move of loadLength_ and marks the extruder
     * unloaded.
     * @param extruder  Extruder/tool index.
     */
    void unloadFilament(int extruder) {
        if (unloadCb_) unloadCb_(extruder, loadLength_, loadSpeed_);
        filamentLoaded_[extruder] = false;
    }

    /**@brief Load filament into tool (M703). */
    /**
     * @brief Feed filament to the toolhead after a tool change.
     *
     * Shorter move (toolLoadLength_) than a cold loadToTool-less load —
     * covers the distance from the tool-change park position to the
     * nozzle.
     * @param tool  Tool index being loaded.
     */
    void loadToTool(int tool) {
        if (loadCb_) loadCb_(tool, toolLoadLength_, loadSpeed_);
    }

    /**@brief Unload filament from tool (M704). */
    /**
     * @brief Retract filament from the toolhead for a tool change.
     *
     * Reverses toolLoadLength_ so the tool can be parked without
     * drawing filament out of the extruder path entirely.
     * @param tool  Tool index being unloaded.
     */
    void unloadFromTool(int tool) {
        if (unloadCb_) unloadCb_(tool, toolLoadLength_, loadSpeed_);
    }

    /**@brief Purge filament (M705). */
    /**
     * @brief Purge filament through the nozzle.
     *
     * Extrudes purgeLength_ at purgeSpeed_ to flush residual or
     * color-contaminated material before a print.
     * @param extruder  Extruder/tool index.
     */
    void purge(int extruder) {
        if (purgeCb_) purgeCb_(extruder, purgeLength_, purgeSpeed_);
    }

    /**@brief Retract filament (M706). */
    /**
     * @brief Retract to relieve nozzle pressure.
     *
     * Small reverse move (retractLength_ at retractSpeed_) used before
     * travel moves to prevent oozing.
     * @param extruder  Extruder/tool index.
     */
    void retract(int extruder) {
        if (retractCb_) retractCb_(extruder, retractLength_, retractSpeed_);
    }

    /**@brief Set filament sensor state (M707). */
    /**
     * @brief Enable or disable a filament sensor.
     *
     * Disabled sensors are ignored by the runout check — their state is
     * treated as "filament present".
     * @param sensor   Sensor index.
     * @param enabled  true to arm the sensor, false to bypass it.
     */
    void setSensorState(int sensor, bool enabled) {
        sensorEnabled_[sensor] = enabled;
    }

    /**@brief Report filament sensor state (M708). */
    std::string reportSensorState() const {
        std::ostringstream ss;
        ss << "Filament sensors:";
        for (const auto& [sensor, enabled] : sensorEnabled_) {
            ss << " S" << sensor << ":" << (enabled ? "ON" : "OFF");
        }
        if (sensorEnabled_.empty()) ss << " none configured";
        return ss.str();
    }

    /**@brief Check if filament is loaded for an extruder. */
    /**
     * @brief Check whether the extruder is marked loaded.
     * @param extruder  Extruder/tool index.
     * @return true if loadFilament() completed for this extruder;
     *         false for unknown indices or after unloadFilament().
     */
    bool isLoaded(int extruder) const {
        auto it = filamentLoaded_.find(extruder);
        return it != filamentLoaded_.end() && it->second;
    }

    /**@brief Set load parameters. */
    /**
     * @brief Set the cold-load feed distance.
     * @param length  Filament length to feed on loadFilament() [mm].
     */
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

/**
 * @file KlippyUdsServerC1Helpers.cpp
 * @brief C1 helper methods (registration, database, job queue, etc.)
 */

#include "tether/klipper/klippy/KlippyServer.hpp"
#include "tether/klipper/klippy/AdvancedObjects.hpp"

#include <algorithm>
#include <chrono>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <set>
#include <sstream>
#include <sys/stat.h>

namespace tether::klipper::klippy {

// ============================================================================
// C1: Helper methods for new state
// ============================================================================

void KlippyServer::registerWebcam(const std::string& name, const std::string& url,
                                       const std::string& service) {
    Webcam cam;
    cam.name = name;
    cam.url = url;
    cam.service = service;
    webcams_[name] = cam;
}

void KlippyServer::registerService(const std::string& name,
                                        const std::string& activeState,
                                        const std::string& subState) {
    services_[name] = {name, activeState, subState};
}

void KlippyServer::registerFileRoot(const std::string& name, const std::string& path,
                                         bool writable) {
    fileRoots_[name] = {name, path, writable};
}

void KlippyServer::initServerConfig() {
    // Emulate a Moonraker server config that reflects the loaded Klipper config
    // This is what frontends like Mainsail/Fluidd query via server/config

    // Build the config from the ConfigParser if available
    std::map<std::string, JsonValue> printerConfig;
    if (!configParser_.sections().empty()) {
        for (const auto& section : configParser_.sections()) {
            std::map<std::string, JsonValue> sectionData;
            for (const auto& [key, value] : section.values) {
                sectionData[key] = JsonValue(value);
            }
            printerConfig[section.name] = JsonValue(sectionData);
        }
    } else {
        // Provide a default emulated Klipper config
        // [printer] section
        std::map<std::string, JsonValue> printer;
        printer["kinematics"] = JsonValue("cartesian");
        printer["max_velocity"] = JsonValue(500);
        printer["max_accel"] = JsonValue(3000);
        printer["max_accel_to_decel"] = JsonValue(1500);
        printerConfig["printer"] = JsonValue(printer);

        // [stepper_x] section
        std::map<std::string, JsonValue> stepperX;
        stepperX["step_pin"] = JsonValue("PF0");
        stepperX["dir_pin"] = JsonValue("PF1");
        stepperX["enable_pin"] = JsonValue("!PF2");
        stepperX["microsteps"] = JsonValue(16);
        stepperX["rotation_distance"] = JsonValue(40);
        stepperX["endstop_pin"] = JsonValue("PF3");
        stepperX["position_min"] = JsonValue(0);
        stepperX["position_endstop"] = JsonValue(200);
        stepperX["position_max"] = JsonValue(200);
        stepperX["homing_speed"] = JsonValue(50);
        printerConfig["stepper_x"] = JsonValue(stepperX);

        // [stepper_y] section
        std::map<std::string, JsonValue> stepperY;
        stepperY["step_pin"] = JsonValue("PF4");
        stepperY["dir_pin"] = JsonValue("PF5");
        stepperY["enable_pin"] = JsonValue("!PF6");
        stepperY["microsteps"] = JsonValue(16);
        stepperY["rotation_distance"] = JsonValue(40);
        stepperY["endstop_pin"] = JsonValue("PF7");
        stepperY["position_min"] = JsonValue(0);
        stepperY["position_endstop"] = JsonValue(200);
        stepperY["position_max"] = JsonValue(200);
        stepperY["homing_speed"] = JsonValue(50);
        printerConfig["stepper_y"] = JsonValue(stepperY);

        // [stepper_z] section
        std::map<std::string, JsonValue> stepperZ;
        stepperZ["step_pin"] = JsonValue("PL0");
        stepperZ["dir_pin"] = JsonValue("!PL1");
        stepperZ["enable_pin"] = JsonValue("!PL2");
        stepperZ["microsteps"] = JsonValue(16);
        stepperZ["rotation_distance"] = JsonValue(8);
        stepperZ["endstop_pin"] = JsonValue("probe:z_virtual_endstop");
        stepperZ["position_min"] = JsonValue(-5);
        stepperZ["position_max"] = JsonValue(200);
        printerConfig["stepper_z"] = JsonValue(stepperZ);

        // [extruder] section
        std::map<std::string, JsonValue> extruder;
        extruder["step_pin"] = JsonValue("PL3");
        extruder["dir_pin"] = JsonValue("PL4");
        extruder["enable_pin"] = JsonValue("!PL5");
        extruder["microsteps"] = JsonValue(16);
        extruder["rotation_distance"] = JsonValue(33.5);
        extruder["nozzle_diameter"] = JsonValue(0.4);
        extruder["filament_diameter"] = JsonValue(1.75);
        extruder["heater_pin"] = JsonValue("PE4");
        extruder["sensor_type"] = JsonValue("EPCOS 100K B57560G104F");
        extruder["sensor_pin"] = JsonValue("PK0");
        extruder["control"] = JsonValue("pid");
        extruder["pid_kp"] = JsonValue(22.2);
        extruder["pid_ki"] = JsonValue(1.08);
        extruder["pid_kd"] = JsonValue(114.0);
        extruder["min_temp"] = JsonValue(0);
        extruder["max_temp"] = JsonValue(250);
        printerConfig["extruder"] = JsonValue(extruder);

        // [heater_bed] section
        std::map<std::string, JsonValue> heaterBed;
        heaterBed["heater_pin"] = JsonValue("PG5");
        heaterBed["sensor_type"] = JsonValue("EPCOS 100K B57560G104F");
        heaterBed["sensor_pin"] = JsonValue("PK1");
        heaterBed["control"] = JsonValue("pid");
        heaterBed["pid_kp"] = JsonValue(54.0);
        heaterBed["pid_ki"] = JsonValue(0.77);
        heaterBed["pid_kd"] = JsonValue(924.0);
        heaterBed["min_temp"] = JsonValue(0);
        heaterBed["max_temp"] = JsonValue(120);
        printerConfig["heater_bed"] = JsonValue(heaterBed);

        // [fan] section
        std::map<std::string, JsonValue> fan;
        fan["pin"] = JsonValue("PH4");
        printerConfig["fan"] = JsonValue(fan);

        // [mcu] section
        std::map<std::string, JsonValue> mcu;
        mcu["serial"] = JsonValue("/dev/serial/by-id/usb-tether-mcu");
        mcu["baud"] = JsonValue(250000);
        printerConfig["mcu"] = JsonValue(mcu);

        // [probe] section
        std::map<std::string, JsonValue> probe;
        probe["pin"] = JsonValue("ar15");
        probe["z_offset"] = JsonValue(0.0);
        probe["x_offset"] = JsonValue(0.0);
        probe["y_offset"] = JsonValue(0.0);
        probe["samples"] = JsonValue(3);
        probe["sample_retract_dist"] = JsonValue(2.0);
        probe["speed"] = JsonValue(10.0);
        printerConfig["probe"] = JsonValue(probe);

        // [bed_mesh] section
        std::map<std::string, JsonValue> bedMesh;
        bedMesh["mesh_min"] = JsonValue("20, 20");
        bedMesh["mesh_max"] = JsonValue("180, 180");
        bedMesh["probe_count"] = JsonValue("3, 3");
        bedMesh["algorithm"] = JsonValue("bicubic");
        printerConfig["bed_mesh"] = JsonValue(bedMesh);

        // [idle_timeout] section
        std::map<std::string, JsonValue> idleTimeout;
        idleTimeout["timeout"] = JsonValue(600);
        printerConfig["idle_timeout"] = JsonValue(idleTimeout);

        // [safe_z_home] section
        std::map<std::string, JsonValue> safeZHome;
        safeZHome["home_xy_position"] = JsonValue("100, 100");
        safeZHome["z_hop"] = JsonValue(10);
        safeZHome["z_hop_speed"] = JsonValue(20);
        printerConfig["safe_z_home"] = JsonValue(safeZHome);

        // [display_status] section
        printerConfig["display_status"] = JsonValue(std::map<std::string, JsonValue>{});

        // [pause_resume] section
        printerConfig["pause_resume"] = JsonValue(std::map<std::string, JsonValue>{});

        // [virtual_sdcard] section
        std::map<std::string, JsonValue> virtualSdcard;
        virtualSdcard["path"] = JsonValue(fileRoot_);
        printerConfig["virtual_sdcard"] = JsonValue(virtualSdcard);
    }

    serverConfig_ = printerConfig;
}

void KlippyServer::databasePut(const std::string& ns, const std::string& key,
                                    const JsonValue& value) {
    database_[ns][key] = value;
}

std::optional<JsonValue> KlippyServer::databaseGet(const std::string& ns,
                                                        const std::string& key) {
    auto it = database_.find(ns);
    if (it == database_.end()) return std::nullopt;
    auto kit = it->second.find(key);
    if (kit == it->second.end()) return std::nullopt;
    return kit->second;
}

bool KlippyServer::databaseDelete(const std::string& ns, const std::string& key) {
    auto it = database_.find(ns);
    if (it == database_.end()) return false;
    auto kit = it->second.find(key);
    if (kit == it->second.end()) return false;
    it->second.erase(kit);
    return true;
}

void KlippyServer::jobQueueAdd(const std::string& filename) {
    jobQueue_.push_back(filename);
}

int64_t KlippyServer::jobHistoryAdd(const std::string& filename,
                                         const std::string& status) {
    JobHistoryEntry entry;
    entry.jobId = nextJobId_++;
    entry.filename = filename;
    entry.status = status;
    entry.startTime = static_cast<double>(
        std::chrono::duration_cast<std::chrono::seconds>(
            std::chrono::system_clock::now().time_since_epoch()).count());
    std::vector<HistoryChangedCallback> cbs;
    {
        std::lock_guard<std::recursive_mutex> lock(mutex_);
        jobHistory_.push_back(entry);
        cbs = historyChangedCbs_;
    }
    for (const auto& cb : cbs) {
        if (cb) cb("add", entry.jobId);
    }
    return entry.jobId;
}

void KlippyServer::announcementAdd(const std::string& entryId,
                                        const std::string& title,
                                        const std::string& description,
                                        const std::string& severity) {
    Announcement a;
    a.entryId = entryId;
    a.title = title;
    a.description = description;
    a.severity = severity;
    announcements_.push_back(a);
}

} // namespace tether::klipper::klippy

/**
 * @file KlippyUdsServerC1Endpoints.cpp
 * @brief C1 Moonraker-compatible endpoint handlers
 */

#include "tether/klipper/klippy/KlippyUdsServer.hpp"
#include "tether/klipper/klippy/AdvancedObjects.hpp"
#include "UdsConnection_internal.hpp"

#include <algorithm>
#include <cerrno>
#include <chrono>
#include <cstring>
#include <ctime>
#include <fcntl.h>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <netinet/in.h>
#include <set>
#include <signal.h>
#include <sstream>
#include <sys/socket.h>
#include <sys/stat.h>
#include <sys/un.h>
#include <unistd.h>

namespace tether::klipper::klippy {

// ============================================================================
// C1: New Moonraker-compatible endpoint handlers
// ============================================================================

JsonValue KlippyUdsServer::handleServerConfig(const JsonValue& params) {
    return withLockResult(serverConfig_);
}

JsonValue KlippyUdsServer::handleServerFilesRoots(const JsonValue& params) {
    return withLock([&]() {
        std::map<std::string, JsonValue> result;
        std::map<std::string, JsonValue> roots;
        for (const auto& [name, root] : fileRoots_) {
            std::map<std::string, JsonValue> rootInfo;
            rootInfo["path"] = JsonValue(root.path);
            rootInfo["writable"] = JsonValue(root.writable);
            roots[name] = JsonValue(rootInfo);
        }
        result["result"] = JsonValue(roots);
        return result;
    });
}

JsonValue KlippyUdsServer::handleServerFilesCreateDir(const JsonValue& params) {
    std::map<std::string, JsonValue> result;
    std::string path = params.has("path") && params.find("path")->isString()
        ? params.find("path")->asString() : "";

    if (!path.empty()) {
        namespace fs = std::filesystem;
        try {
            fs::path fullPath = fs::path(fileRoot_) / path;
            fs::create_directories(fullPath);
            result["result"] = JsonValue(true);
        } catch (...) {
            result["error"] = JsonValue("Failed to create directory");
        }
    } else {
        result["error"] = JsonValue("Path required");
    }
    return JsonValue(result);
}

JsonValue KlippyUdsServer::handleServerFilesMetascan(const JsonValue& params) {
    std::map<std::string, JsonValue> result;
    std::string filename = params.has("filename") && params.find("filename")->isString()
        ? params.find("filename")->asString() : "";

    if (filename.empty()) {
        result["error"] = JsonValue("Filename required");
        return JsonValue(result);
    }

    namespace fs = std::filesystem;
    fs::path fullPath = fs::path(fileRoot_) / filename;
    if (!fs::exists(fullPath)) {
        result["error"] = JsonValue("File not found");
        return JsonValue(result);
    }

    // Parse G-code metadata from the file
    std::map<std::string, JsonValue> metadata;
    metadata["filename"] = JsonValue(filename);
    metadata["size"] = JsonValue(static_cast<int64_t>(fs::file_size(fullPath)));
    metadata["modified"] = JsonValue(static_cast<int64_t>(
        std::chrono::duration_cast<std::chrono::seconds>(
            fs::last_write_time(fullPath).time_since_epoch()).count()));

    // Try to extract G-code metadata from comments
    std::ifstream f(fullPath);
    std::string line;
    double printTime = 0.0;
    double filamentUsed = 0.0;
    int layerCount = 0;
    bool slicerFound = false;
    std::string slicer;
    while (std::getline(f, line) && layerCount == 0) {
        if (line.empty() || line[0] != ';') continue;
        // Look for common metadata patterns
        if (line.find("estimated printing time") != std::string::npos) {
            // Simple extraction
            slicerFound = true;
        }
        if (line.find("filament used") != std::string::npos ||
            line.find("filament used") != std::string::npos) {
            slicerFound = true;
        }
        if (line.find("layers:") != std::string::npos) {
            // Try to extract layer count
            auto pos = line.find("layers:");
            if (pos != std::string::npos) {
                std::string numStr = line.substr(pos + 7);
                // Extract first number
                for (char c : numStr) {
                    if (c >= '0' && c <= '9') {
                        layerCount = layerCount * 10 + (c - '0');
                    } else if (layerCount > 0) break;
                }
            }
        }
        if (line.find("Cura") != std::string::npos) slicer = "Cura";
        else if (line.find("PrusaSlicer") != std::string::npos) slicer = "PrusaSlicer";
        else if (line.find("Slic3r") != std::string::npos) slicer = "Slic3r";
    }
    if (!slicer.empty()) metadata["slicer"] = JsonValue(slicer);
    if (layerCount > 0) metadata["layer_count"] = JsonValue(static_cast<int64_t>(layerCount));

    result["result"] = JsonValue(metadata);
    return JsonValue(result);
}

JsonValue KlippyUdsServer::handleServerFilesThumbnails(const JsonValue& params) {
    std::map<std::string, JsonValue> result;
    std::string filename = params.has("filename") && params.find("filename")->isString()
        ? params.find("filename")->asString() : "";

    // Thumbnails are typically embedded in G-code as base64 images
    // Return empty array if no thumbnails found
    std::vector<JsonValue> thumbnails;
    (void)filename; // Would scan the G-code file for thumbnail data
    result["result"] = JsonValue(thumbnails);
    return JsonValue(result);
}

JsonValue KlippyUdsServer::handleServerLogsRollover(const JsonValue& params) {
    std::map<std::string, JsonValue> result;
    // In a real system, this would rotate log files
    result["result"] = JsonValue(true);
    return JsonValue(result);
}

JsonValue KlippyUdsServer::handleServerKlippyLog(const JsonValue& params) {
    std::map<std::string, JsonValue> result;
    // Return log file path or content summary
    std::map<std::string, JsonValue> logInfo;
    logInfo["path"] = JsonValue(config_.logFile);
    namespace fs = std::filesystem;
    if (fs::exists(config_.logFile)) {
        logInfo["size"] = JsonValue(static_cast<int64_t>(fs::file_size(config_.logFile)));
    }
    result["result"] = JsonValue(logInfo);
    return JsonValue(result);
}

JsonValue KlippyUdsServer::handleServerMoonrakerLog(const JsonValue& params) {
    std::map<std::string, JsonValue> result;
    std::map<std::string, JsonValue> logInfo;
    std::string moonrakerLog = "/var/log/tether-moonraker.log";
    logInfo["path"] = JsonValue(moonrakerLog);
    namespace fs = std::filesystem;
    if (fs::exists(moonrakerLog)) {
        logInfo["size"] = JsonValue(static_cast<int64_t>(fs::file_size(moonrakerLog)));
    }
    result["result"] = JsonValue(logInfo);
    return JsonValue(result);
}

JsonValue KlippyUdsServer::handleMachineServicesList(const JsonValue& params) {
    return withLock([&]() {
        std::map<std::string, JsonValue> result;
        std::map<std::string, JsonValue> services;
        for (const auto& [name, svc] : services_) {
            std::map<std::string, JsonValue> svcInfo;
            svcInfo["active_state"] = JsonValue(svc.activeState);
            svcInfo["sub_state"] = JsonValue(svc.subState);
            services[name] = JsonValue(svcInfo);
        }
        result["result"] = JsonValue(services);
        return result;
    });
}

JsonValue KlippyUdsServer::handleMachineServiceAction(const JsonValue& params) {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    std::map<std::string, JsonValue> result;
    std::string service = params.has("service") && params.find("service")->isString()
        ? params.find("service")->asString() : "";

    if (service.empty()) {
        result["error"] = JsonValue("Service name required");
        return JsonValue(result);
    }

    auto it = services_.find(service);
    if (it == services_.end()) {
        result["error"] = JsonValue("Service not found");
        return JsonValue(result);
    }

    // In a real system, this would call systemctl restart/stop/start
    result["result"] = JsonValue(true);
    return JsonValue(result);
}

JsonValue KlippyUdsServer::handleMachineUpdateList(const JsonValue& params) {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    std::map<std::string, JsonValue> result;
    std::map<std::string, JsonValue> updates;
    for (const auto& [component, status] : updateStatus_) {
        std::map<std::string, JsonValue> info;
        info["status"] = JsonValue(status);
        updates[component] = JsonValue(info);
    }
    result["result"] = JsonValue(updates);
    return JsonValue(result);
}

JsonValue KlippyUdsServer::handleMachineUpdateRefresh(const JsonValue& params) {
    std::map<std::string, JsonValue> result;
    result["result"] = JsonValue(true);
    return JsonValue(result);
}

JsonValue KlippyUdsServer::handleMachineUpdateUpdate(const JsonValue& params) {
    std::map<std::string, JsonValue> result;
    std::string component = params.has("component") && params.find("component")->isString()
        ? params.find("component")->asString() : "";
    (void)component;
    result["result"] = JsonValue(true);
    return JsonValue(result);
}

JsonValue KlippyUdsServer::handleMachineUpdateRecover(const JsonValue& params) {
    std::map<std::string, JsonValue> result;
    std::string component = params.has("component") && params.find("component")->isString()
        ? params.find("component")->asString() : "";
    (void)component;
    result["result"] = JsonValue(true);
    return JsonValue(result);
}

JsonValue KlippyUdsServer::handleDatabaseList(const JsonValue& params) {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    std::map<std::string, JsonValue> result;
    std::string ns = params.has("namespace") && params.find("namespace")->isString()
        ? params.find("namespace")->asString() : "";

    std::vector<JsonValue> keys;
    auto it = database_.find(ns);
    if (it != database_.end()) {
        for (const auto& [key, val] : it->second) {
            keys.push_back(JsonValue(key));
        }
    }
    result["result"] = JsonValue(keys);
    return JsonValue(result);
}

JsonValue KlippyUdsServer::handleDatabaseGet(const JsonValue& params) {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    std::map<std::string, JsonValue> result;
    std::string ns = params.has("namespace") && params.find("namespace")->isString()
        ? params.find("namespace")->asString() : "";
    std::string key = params.has("key") && params.find("key")->isString()
        ? params.find("key")->asString() : "";

    auto val = databaseGet(ns, key);
    if (val) {
        result["result"] = *val;
    } else {
        result["error"] = JsonValue("Key not found");
    }
    return JsonValue(result);
}

JsonValue KlippyUdsServer::handleDatabasePut(const JsonValue& params) {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    std::map<std::string, JsonValue> result;
    std::string ns = params.has("namespace") && params.find("namespace")->isString()
        ? params.find("namespace")->asString() : "";
    std::string key = params.has("key") && params.find("key")->isString()
        ? params.find("key")->asString() : "";

    const JsonValue* valPtr = params.find("value");
    if (valPtr) {
        databasePut(ns, key, *valPtr);
        result["result"] = JsonValue(true);
    } else {
        result["error"] = JsonValue("Value required");
    }
    return JsonValue(result);
}

JsonValue KlippyUdsServer::handleDatabaseDelete(const JsonValue& params) {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    std::map<std::string, JsonValue> result;
    std::string ns = params.has("namespace") && params.find("namespace")->isString()
        ? params.find("namespace")->asString() : "";
    std::string key = params.has("key") && params.find("key")->isString()
        ? params.find("key")->asString() : "";

    if (databaseDelete(ns, key)) {
        result["result"] = JsonValue(true);
    } else {
        result["error"] = JsonValue("Key not found");
    }
    return JsonValue(result);
}

JsonValue KlippyUdsServer::handleJobQueueStatus(const JsonValue& params) {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    std::map<std::string, JsonValue> result;
    std::vector<JsonValue> queue;
    for (const auto& filename : jobQueue_) {
        std::map<std::string, JsonValue> job;
        job["filename"] = JsonValue(filename);
        queue.push_back(JsonValue(job));
    }
    result["result"] = JsonValue(queue);
    return JsonValue(result);
}

JsonValue KlippyUdsServer::handleJobQueuePost(const JsonValue& params) {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    std::map<std::string, JsonValue> result;
    // Accept either a single filename or a list
    const JsonValue* filenames = params.find("filenames");
    if (filenames && filenames->isArray()) {
        for (const auto& f : filenames->asArray()) {
            if (f.isString()) jobQueue_.push_back(f.asString());
        }
    } else {
        std::string filename = params.has("filename") && params.find("filename")->isString()
            ? params.find("filename")->asString() : "";
        if (!filename.empty()) {
            jobQueue_.push_back(filename);
        }
    }
    result["result"] = JsonValue(static_cast<int64_t>(jobQueue_.size()));
    return JsonValue(result);
}

JsonValue KlippyUdsServer::handleJobQueueDelete(const JsonValue& params) {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    std::map<std::string, JsonValue> result;
    std::string filename = params.has("filename") && params.find("filename")->isString()
        ? params.find("filename")->asString() : "";
    if (!filename.empty()) {
        jobQueue_.erase(
            std::remove(jobQueue_.begin(), jobQueue_.end(), filename),
            jobQueue_.end());
    } else {
        // Clear all
        jobQueue_.clear();
    }
    result["result"] = JsonValue(true);
    return JsonValue(result);
}

JsonValue KlippyUdsServer::handleJobHistoryList(const JsonValue& params) {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    std::map<std::string, JsonValue> result;
    std::vector<JsonValue> jobs;
    for (const auto& entry : jobHistory_) {
        std::map<std::string, JsonValue> job;
        job["job_id"] = JsonValue(entry.jobId);
        job["filename"] = JsonValue(entry.filename);
        job["status"] = JsonValue(entry.status);
        job["start_time"] = JsonValue(entry.startTime);
        job["end_time"] = JsonValue(entry.endTime);
        job["print_duration"] = JsonValue(entry.printDuration);
        job["total_duration"] = JsonValue(entry.totalDuration);
        job["filament_used"] = JsonValue(entry.filamentUsed);
        jobs.push_back(JsonValue(job));
    }
    result["result"] = JsonValue(jobs);
    return JsonValue(result);
}

JsonValue KlippyUdsServer::handleJobHistoryGet(const JsonValue& params) {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    std::map<std::string, JsonValue> result;
    int64_t jobId = params.has("job_id") && params.find("job_id")->isInt()
        ? params.find("job_id")->asInt() : -1;

    for (const auto& entry : jobHistory_) {
        if (entry.jobId == jobId) {
            std::map<std::string, JsonValue> job;
            job["job_id"] = JsonValue(entry.jobId);
            job["filename"] = JsonValue(entry.filename);
            job["status"] = JsonValue(entry.status);
            job["start_time"] = JsonValue(entry.startTime);
            job["end_time"] = JsonValue(entry.endTime);
            job["print_duration"] = JsonValue(entry.printDuration);
            job["total_duration"] = JsonValue(entry.totalDuration);
            job["filament_used"] = JsonValue(entry.filamentUsed);
            result["result"] = JsonValue(job);
            return JsonValue(result);
        }
    }
    result["error"] = JsonValue("Job not found");
    return JsonValue(result);
}

JsonValue KlippyUdsServer::handleAnnouncementsList(const JsonValue& params) {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    std::map<std::string, JsonValue> result;
    std::vector<JsonValue> entries;
    for (const auto& a : announcements_) {
        if (a.dismissed) continue;
        std::map<std::string, JsonValue> entry;
        entry["entry_id"] = JsonValue(a.entryId);
        entry["title"] = JsonValue(a.title);
        entry["description"] = JsonValue(a.description);
        entry["url"] = JsonValue(a.url);
        entry["date"] = JsonValue(a.date);
        entry["severity"] = JsonValue(a.severity);
        entries.push_back(JsonValue(entry));
    }
    result["result"] = JsonValue(entries);
    return JsonValue(result);
}

JsonValue KlippyUdsServer::handleAnnouncementsUpdate(const JsonValue& params) {
    std::map<std::string, JsonValue> result;
    result["result"] = JsonValue(true);
    return JsonValue(result);
}

JsonValue KlippyUdsServer::handleAnnouncementsDismiss(const JsonValue& params) {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    std::map<std::string, JsonValue> result;
    std::string entryId = params.has("entry_id") && params.find("entry_id")->isString()
        ? params.find("entry_id")->asString() : "";
    for (auto& a : announcements_) {
        if (a.entryId == entryId) {
            a.dismissed = true;
            result["result"] = JsonValue(true);
            return JsonValue(result);
        }
    }
    result["error"] = JsonValue("Announcement not found");
    return JsonValue(result);
}

JsonValue KlippyUdsServer::handleWebcamsList(const JsonValue& params) {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    std::map<std::string, JsonValue> result;
    std::map<std::string, JsonValue> webcams;
    for (const auto& [name, cam] : webcams_) {
        std::map<std::string, JsonValue> camInfo;
        camInfo["url"] = JsonValue(cam.url);
        camInfo["service"] = JsonValue(cam.service);
        camInfo["enabled"] = JsonValue(cam.enabled);
        camInfo["rotation"] = JsonValue(cam.rotation);
        camInfo["aspect_ratio"] = JsonValue(cam.aspectRatio);
        camInfo["source"] = JsonValue(cam.source);
        webcams[name] = JsonValue(camInfo);
    }
    result["result"] = JsonValue(webcams);
    return JsonValue(result);
}

JsonValue KlippyUdsServer::handleWebcamsGet(const JsonValue& params) {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    std::map<std::string, JsonValue> result;
    std::string name = params.has("name") && params.find("name")->isString()
        ? params.find("name")->asString() : "";
    auto it = webcams_.find(name);
    if (it != webcams_.end()) {
        std::map<std::string, JsonValue> camInfo;
        camInfo["url"] = JsonValue(it->second.url);
        camInfo["service"] = JsonValue(it->second.service);
        camInfo["enabled"] = JsonValue(it->second.enabled);
        camInfo["rotation"] = JsonValue(it->second.rotation);
        camInfo["aspect_ratio"] = JsonValue(it->second.aspectRatio);
        camInfo["source"] = JsonValue(it->second.source);
        result["result"] = JsonValue(camInfo);
    } else {
        result["error"] = JsonValue("Webcam not found");
    }
    return JsonValue(result);
}

JsonValue KlippyUdsServer::handleWebcamsTest(const JsonValue& params) {
    std::map<std::string, JsonValue> result;
    std::string name = params.has("name") && params.find("name")->isString()
        ? params.find("name")->asString() : "";
    auto it = webcams_.find(name);
    if (it != webcams_.end()) {
        // Return a test result
        std::map<std::string, JsonValue> testResult;
        testResult["status"] = JsonValue("ok");
        testResult["url"] = JsonValue(it->second.url);
        result["result"] = JsonValue(testResult);
    } else {
        result["error"] = JsonValue("Webcam not found");
    }
    return JsonValue(result);
}

JsonValue KlippyUdsServer::handleDevicesList(const JsonValue& params) {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    std::map<std::string, JsonValue> result;
    // Devices include power devices, sensors, etc.
    std::map<std::string, JsonValue> devices;
    // Add power devices
    for (const auto& [name, dev] : powerDevices_) {
        std::map<std::string, JsonValue> devInfo;
        devInfo["type"] = JsonValue("power");
        devInfo["state"] = JsonValue(dev.state);
        devices[name] = JsonValue(devInfo);
    }
    result["result"] = JsonValue(devices);
    return JsonValue(result);
}

JsonValue KlippyUdsServer::handleDevicesGet(const JsonValue& params) {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    std::map<std::string, JsonValue> result;
    std::string name = params.has("device") && params.find("device")->isString()
        ? params.find("device")->asString() : "";
    auto it = powerDevices_.find(name);
    if (it != powerDevices_.end()) {
        std::map<std::string, JsonValue> devInfo;
        devInfo["type"] = JsonValue("power");
        devInfo["state"] = JsonValue(it->second.state);
        devInfo["locked"] = JsonValue(it->second.locked);
        result["result"] = JsonValue(devInfo);
    } else {
        result["error"] = JsonValue("Device not found");
    }
    return JsonValue(result);
}

} // namespace tether::klipper::klippy

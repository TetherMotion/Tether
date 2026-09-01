/**
 * @file KlippyUdsServerFileEndpoints.cpp
 * @brief File operation and server info endpoint handlers
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
// Additional Moonraker-compatible endpoint handlers
// ============================================================================

JsonValue KlippyServer::handleServerInfo(const JsonValue& params) {
    std::map<std::string, JsonValue> result;
    std::map<std::string, JsonValue> info;
    info["version"] = JsonValue(config_.softwareVersion);
    // Fluidd reads klippy_state (not state) from server.info.
    info["klippy_state"] = JsonValue(stateToString(state_));
    info["state"] = JsonValue(stateToString(state_));
    info["klippy_connected"] = JsonValue(state_ != PrinterState::Shutdown);
    info["cpu_info"] = JsonValue("tether-klipper");
    // Moonraker's server.info includes these array fields; Fluidd accesses
    // payload.components.length unconditionally, so they must always be arrays.
    info["components"] = JsonValue(std::vector<JsonValue>{});
    info["failed_components"] = JsonValue(std::vector<JsonValue>{});
    info["registered_directories"] = JsonValue(std::vector<JsonValue>{
        JsonValue("config"), JsonValue("logs"), JsonValue("gcodes")
    });
    info["warnings"] = JsonValue(std::vector<JsonValue>{});
    info["moonraker_version"] = JsonValue("tether-1.0.0");
    std::vector<JsonValue> apiVersion;
    apiVersion.push_back(JsonValue(static_cast<int64_t>(1)));
    apiVersion.push_back(JsonValue(static_cast<int64_t>(0)));
    apiVersion.push_back(JsonValue(static_cast<int64_t>(0)));
    info["api_version"] = JsonValue(apiVersion);
    info["api_version_string"] = JsonValue("1.0.0");
    result["result"] = JsonValue(info);
    return JsonValue(result);
}

JsonValue KlippyServer::handleServerFilesList(const JsonValue& params) {
    std::map<std::string, JsonValue> result;
    std::vector<JsonValue> files;

    // Determine which root to list from.  Default to "gcodes" for backward
    // compatibility, but honour the "root" parameter (e.g. "config", "logs").
    std::string rootName = "gcodes";
    if (params.isObject() && params.has("root") &&
        params.find("root")->isString()) {
        rootName = params.find("root")->asString();
    }

    std::string rootPath = fileRoot_;
    auto rootIt = fileRoots_.find(rootName);
    if (rootIt != fileRoots_.end()) {
        rootPath = rootIt->second.path;
    }

    namespace fs = std::filesystem;
    if (fs::exists(rootPath)) {
        try {
            for (const auto& entry : fs::recursive_directory_iterator(rootPath)) {
                if (!entry.is_regular_file()) continue;
                std::map<std::string, JsonValue> fileInfo;
                std::string relPath = fs::relative(entry.path(), rootPath).string();
                fileInfo["path"] = JsonValue(relPath);
                fileInfo["size"] = JsonValue(static_cast<int64_t>(entry.file_size()));
                auto ftime = entry.last_write_time();
                auto sctp = std::chrono::time_point_cast<std::chrono::seconds>(
                    ftime);
                fileInfo["modified"] = JsonValue(static_cast<double>(
                    sctp.time_since_epoch().count()));
                files.push_back(JsonValue(fileInfo));
            }
        } catch (const std::filesystem::filesystem_error&) {
            // Ignore directory errors
        }
    }

    result["result"] = JsonValue(files);
    return JsonValue(result);
}

JsonValue KlippyServer::handleServerFilesMetadata(const JsonValue& params) {
    std::map<std::string, JsonValue> result;
    std::map<std::string, JsonValue> metadata;

    if (params.has("filename") && params.find("filename")->isString()) {
        std::string filename = params.find("filename")->asString();
        std::string fullPath = fileRoot_ + "/" + filename;

        namespace fs = std::filesystem;
        if (fs::exists(fullPath)) {
            metadata["filename"] = JsonValue(filename);
            metadata["size"] = JsonValue(static_cast<int64_t>(fs::file_size(fullPath)));
            auto ftime = fs::last_write_time(fullPath);
            auto sctp = std::chrono::time_point_cast<std::chrono::seconds>(ftime);
            metadata["modified"] = JsonValue(static_cast<double>(
                sctp.time_since_epoch().count()));

            // Parse slicer info from G-code header
            std::string slicer = "Unknown";
            std::ifstream f(fullPath);
            if (f) {
                std::string line;
                int lineCount = 0;
                while (std::getline(f, line) && lineCount < 50) {
                    if (line.find(";Generated with") != std::string::npos ||
                        line.find(";Slicer:") != std::string::npos ||
                        line.find("; slicer") != std::string::npos) {
                        // Extract slicer name
                        auto pos = line.find(':');
                        if (pos != std::string::npos) {
                            slicer = line.substr(pos + 1);
                            while (!slicer.empty() && std::isspace(slicer.front()))
                                slicer.erase(0, 1);
                        }
                        break;
                    }
                    ++lineCount;
                }
            }
            metadata["slicer"] = JsonValue(slicer);
        } else {
            metadata["filename"] = JsonValue(filename);
            metadata["size"] = JsonValue(static_cast<int64_t>(0));
            metadata["modified"] = JsonValue(0.0);
            metadata["slicer"] = JsonValue("Unknown");
        }
    }
    result["result"] = JsonValue(metadata);
    return JsonValue(result);
}

JsonValue KlippyServer::handleMachineSystemInfo(const JsonValue& params) {
    std::map<std::string, JsonValue> result;
    std::map<std::string, JsonValue> info;

    // Read CPU info from /proc/cpuinfo
    std::string cpuModel = "unknown";
    std::string processor = "unknown";
    std::string hardwareDesc = "unknown";
    std::string serialNumber = "unknown";
    int cpuCount = 1;
    {
        std::ifstream cpuinfo("/proc/cpuinfo");
        if (cpuinfo) {
            std::string line;
            int procCount = 0;
            while (std::getline(cpuinfo, line)) {
                if (line.size() >= 10 && line.compare(0, 10, "model name") == 0) {
                    auto pos = line.find(':');
                    if (pos != std::string::npos) {
                        cpuModel = line.substr(pos + 2);
                    }
                } else if (line.size() >= 9 && line.compare(0, 9, "processor") == 0) {
                    procCount++;
                } else if (line.size() >= 9 && line.compare(0, 9, "Processor") == 0) {
                    auto pos = line.find(':');
                    if (pos != std::string::npos) processor = line.substr(pos + 2);
                } else if (line.size() >= 8 && line.compare(0, 8, "Hardware") == 0) {
                    auto pos = line.find(':');
                    if (pos != std::string::npos) hardwareDesc = line.substr(pos + 2);
                } else if (line.size() >= 6 && line.compare(0, 6, "Serial") == 0) {
                    auto pos = line.find(':');
                    if (pos != std::string::npos) serialNumber = line.substr(pos + 2);
                }
            }
            cpuCount = std::max(1, procCount);
        }
    }

    // Read OS info from /etc/os-release
    std::string osName = "Linux";
    std::string osId = "linux";
    std::string osVersion = "";
    std::string osCodename = "";
    {
        std::ifstream osrelease("/etc/os-release");
        if (osrelease) {
            std::string line;
            while (std::getline(osrelease, line)) {
                if (line.size() >= 11 && line.compare(0, 11, "PRETTY_NAME") == 0) {
                    auto pos = line.find('=');
                    if (pos != std::string::npos) {
                        osName = line.substr(pos + 1);
                        if (!osName.empty() && osName.front() == '"') osName.erase(0, 1);
                        if (!osName.empty() && osName.back() == '"') osName.pop_back();
                    }
                } else if (line.size() >= 3 && line.compare(0, 3, "ID=") == 0) {
                    osId = line.substr(3);
                    if (!osId.empty() && osId.front() == '"') osId.erase(0, 1);
                    if (!osId.empty() && osId.back() == '"') osId.pop_back();
                } else if (line.size() >= 8 && line.compare(0, 8, "VERSION=") == 0) {
                    osVersion = line.substr(8);
                    if (!osVersion.empty() && osVersion.front() == '"') osVersion.erase(0, 1);
                    if (!osVersion.empty() && osVersion.back() == '"') osVersion.pop_back();
                } else if (line.size() >= 9 && line.compare(0, 9, "CODENAME=") == 0) {
                    osCodename = line.substr(9);
                    if (!osCodename.empty() && osCodename.front() == '"') osCodename.erase(0, 1);
                    if (!osCodename.empty() && osCodename.back() == '"') osCodename.pop_back();
                }
            }
        }
    }

    // Build cpu_info object (Mainsail expects an object, not a string)
    std::map<std::string, JsonValue> cpuInfo;
    cpuInfo["bits"] = JsonValue("64bit");
    cpuInfo["cpu_count"] = JsonValue(static_cast<int64_t>(cpuCount));
    cpuInfo["cpu_desc"] = JsonValue("");
    cpuInfo["serial_number"] = JsonValue(serialNumber);
    cpuInfo["hardware_desc"] = JsonValue(hardwareDesc);
    cpuInfo["memory_units"] = JsonValue("kB");
    cpuInfo["model"] = JsonValue(cpuModel);
    cpuInfo["processor"] = JsonValue(processor);
    cpuInfo["total_memory"] = JsonValue(static_cast<int64_t>(0));

    // Build distribution object
    std::map<std::string, JsonValue> distribution;
    distribution["codename"] = JsonValue(osCodename);
    distribution["id"] = JsonValue(osId);
    distribution["like"] = JsonValue("");
    distribution["name"] = JsonValue(osName);
    distribution["version"] = JsonValue(osVersion);
    std::map<std::string, JsonValue> versionParts;
    versionParts["build_number"] = JsonValue("0");
    versionParts["major"] = JsonValue("0");
    versionParts["minor"] = JsonValue("0");
    distribution["version_parts"] = JsonValue(versionParts);

    // Build sd_info object
    std::map<std::string, JsonValue> sdInfo;
    sdInfo["capacity"] = JsonValue("");
    sdInfo["manufacturer"] = JsonValue("");
    sdInfo["manufacturer_date"] = JsonValue("");
    sdInfo["manufacturer_id"] = JsonValue("");
    sdInfo["oem_id"] = JsonValue("");
    sdInfo["product_name"] = JsonValue("");
    sdInfo["product_revision"] = JsonValue("");
    sdInfo["serial_number"] = JsonValue("");
    sdInfo["total_bytes"] = JsonValue(static_cast<int64_t>(0));

    // Build python object
    std::map<std::string, JsonValue> python;
    python["version"] = JsonValue(std::vector<JsonValue>{});
    python["version_string"] = JsonValue("3.x");

    // Build service_state object
    std::map<std::string, JsonValue> serviceState;
    for (const auto& [name, svc] : services_) {
        std::map<std::string, JsonValue> svcState;
        svcState["active_state"] = JsonValue(svc.activeState);
        svcState["sub_state"] = JsonValue(svc.subState);
        serviceState[name] = JsonValue(svcState);
    }

    // System uptime
    double uptime = 0.0;
    {
        std::ifstream uptimeFile("/proc/uptime");
        if (uptimeFile) {
            uptimeFile >> uptime;
        }
    }

    // Instance IDs
    std::map<std::string, JsonValue> instanceIds;
    instanceIds["moonraker"] = JsonValue("");
    instanceIds["klipper"] = JsonValue("");

    info["available_services"] = JsonValue(std::vector<JsonValue>{});
    info["cpu_info"] = JsonValue(cpuInfo);
    info["distribution"] = JsonValue(distribution);
    info["sd_info"] = JsonValue(sdInfo);
    info["service_state"] = JsonValue(serviceState);
    info["python"] = JsonValue(python);
    info["network"] = JsonValue(std::map<std::string, JsonValue>{});
    info["system_uptime"] = JsonValue(uptime);
    info["instance_ids"] = JsonValue(instanceIds);

    result["result"] = JsonValue(info);
    return JsonValue(result);
}

JsonValue KlippyServer::handleMachineProcstats(const JsonValue& params) {
    std::map<std::string, JsonValue> result;
    std::map<std::string, JsonValue> stats;

    // Read CPU usage from /proc/stat
    double cpuUsage = 0.0;
    std::ifstream stat("/proc/stat");
    if (stat) {
        std::string label;
        long user, nice, system, idle;
        stat >> label >> user >> nice >> system >> idle;
        long total = user + nice + system + idle;
        if (total > 0) {
            cpuUsage = static_cast<double>(total - idle) / total * 100.0;
        }
    }

    // Read memory from /proc/meminfo
    int64_t memUsage = 0;
    std::ifstream meminfo("/proc/meminfo");
    if (meminfo) {
        std::string line;
        long memTotal = 0, memAvail = 0;
        while (std::getline(meminfo, line)) {
            if (line.size() >= 9 && line.compare(0, 9, "MemTotal:") == 0) {
                std::istringstream iss(line.substr(9));
                iss >> memTotal;
            } else if (line.size() >= 13 && line.compare(0, 13, "MemAvailable:") == 0) {
                std::istringstream iss(line.substr(13));
                iss >> memAvail;
            }
        }
        if (memTotal > 0) {
            memUsage = memTotal - memAvail;
        }
    }

    // Read CPU temperature from thermal zone
    double cpuTemp = 0.0;
    std::ifstream tempFile("/sys/class/thermal/thermal_zone0/temp");
    if (tempFile) {
        tempFile >> cpuTemp;
        cpuTemp /= 1000.0;
    }

    // System CPU usage breakdown (Mainsail expects system_cpu_usage as an object)
    std::map<std::string, JsonValue> systemCpuUsage;
    systemCpuUsage["cpu"] = JsonValue(cpuUsage);

    // Moonraker stats (Mainsail expects moonraker_stats with time field)
    std::map<std::string, JsonValue> moonrakerStats;
    moonrakerStats["cpu_usage"] = JsonValue(cpuUsage);
    moonrakerStats["mem_units"] = JsonValue("kB");
    moonrakerStats["memory"] = JsonValue(static_cast<int64_t>(memUsage));
    auto now = std::chrono::duration<double>(
        std::chrono::system_clock::now().time_since_epoch()).count();
    moonrakerStats["time"] = JsonValue(now);

    stats["cpu_usage"] = JsonValue(cpuUsage);
    stats["memory_usage"] = JsonValue(memUsage);
    stats["webhooks_connections"] = JsonValue(static_cast<int64_t>(0));
    stats["cpu_temp"] = JsonValue(cpuTemp);
    stats["system_cpu_usage"] = JsonValue(systemCpuUsage);
    stats["moonraker_stats"] = JsonValue(moonrakerStats);
    result["result"] = JsonValue(stats);
    return JsonValue(result);
}

// ============================================================================
// New Moonraker-compatible endpoint handlers
// ============================================================================

JsonValue KlippyServer::handleServerTemperatureStore(const JsonValue& params) {
    std::map<std::string, JsonValue> result;
    std::map<std::string, JsonValue> store;

    for (const auto& [heater, entries] : tempStore_) {
        std::map<std::string, JsonValue> heaterData;
        std::vector<JsonValue> temps, targets;
        for (const auto& e : entries) {
            temps.push_back(JsonValue(e.temperature));
            targets.push_back(JsonValue(e.target));
        }
        heaterData["temperatures"] = JsonValue(temps);
        heaterData["targets"] = JsonValue(targets);
        store[heater] = JsonValue(heaterData);
    }

    result["result"] = JsonValue(store);
    return JsonValue(result);
}

JsonValue KlippyServer::handleServerGcodeStore(const JsonValue& params) {
    // Moonraker returns { "gcode_store": [ { "message": ..., "time": ..., "type": ... }, ... ] }
    std::vector<JsonValue> gcodes;

    int count = -1;
    if (params.has("count") && params.find("count")->isInt()) {
        count = static_cast<int>(params.find("count")->asInt());
    }

    int n = 0;
    for (const auto& entry : gcodeStore_) {
        if (count >= 0 && n >= count) break;
        std::map<std::string, JsonValue> gc;
        gc["message"] = JsonValue(entry.message);
        auto secs = std::chrono::duration<double>(
            entry.timestamp.time_since_epoch()).count();
        gc["time"] = JsonValue(secs);
        gc["type"] = JsonValue(entry.type);
        gcodes.push_back(JsonValue(gc));
        n++;
    }

    std::map<std::string, JsonValue> result;
    result["gcode_store"] = JsonValue(gcodes);
    return JsonValue(result);
}

JsonValue KlippyServer::handleServerFilesDirectory(const JsonValue& params) {
    std::map<std::string, JsonValue> result;
    std::map<std::string, JsonValue> dirInfo;

    std::string path = params.has("path") && params.find("path")->isString()
        ? params.find("path")->asString() : "/";
    std::string action = params.has("action") && params.find("action")->isString()
        ? params.find("action")->asString() : "list";
    std::string rootName = params.has("root") && params.find("root")->isString()
        ? params.find("root")->asString() : "gcodes";

    // Resolve the root path from the registered file roots
    std::string rootPath = fileRoot_;
    auto rootIt = fileRoots_.find(rootName);
    if (rootIt != fileRoots_.end()) {
        rootPath = rootIt->second.path;
    }

    // Normalise the sub-path (strip leading slashes)
    while (!path.empty() && path.front() == '/') path.erase(0, 1);

    std::string fullPath = path.empty() ? rootPath : (rootPath + "/" + path);
    namespace fs = std::filesystem;

    if (action == "list") {
        std::vector<JsonValue> dirs, files;
        if (fs::exists(fullPath) && fs::is_directory(fullPath)) {
            try {
                for (const auto& entry : fs::directory_iterator(fullPath)) {
                    std::map<std::string, JsonValue> info;
                    info["path"] = JsonValue(entry.path().filename().string());
                    if (entry.is_directory()) {
                        info["size"] = JsonValue(static_cast<int64_t>(0));
                        info["modified"] = JsonValue(0.0);
                        dirs.push_back(JsonValue(info));
                    } else {
                        info["size"] = JsonValue(static_cast<int64_t>(entry.file_size()));
                        auto ftime = entry.last_write_time();
                        auto sctp = std::chrono::time_point_cast<std::chrono::seconds>(ftime);
                        info["modified"] = JsonValue(static_cast<double>(
                            sctp.time_since_epoch().count()));
                        files.push_back(JsonValue(info));
                    }
                }
            } catch (const std::filesystem::filesystem_error&) {}
        }
        dirInfo["dirs"] = JsonValue(dirs);
        dirInfo["files"] = JsonValue(files);
        result["result"] = JsonValue(dirInfo);
    } else if (action == "create_dir") {
        try {
            fs::create_directories(fullPath);
            result["result"] = JsonValue(true);
            notifyFilelistChanged("create_dir", path, "gcodes");
        } catch (const std::exception& e) {
            result["error"] = JsonValue(std::string("Create directory failed: ") + e.what());
        }
    } else if (action == "delete_dir") {
        try {
            fs::remove_all(fullPath);
            result["result"] = JsonValue(true);
            notifyFilelistChanged("delete_dir", path, "gcodes");
        } catch (const std::exception& e) {
            result["error"] = JsonValue(std::string("Delete directory failed: ") + e.what());
        }
    } else {
        result["error"] = JsonValue("Invalid action: " + action + " (expected 'list', 'create_dir', or 'delete_dir')");
    }
    return JsonValue(result);
}

JsonValue KlippyServer::handleServerFilesMove(const JsonValue& params) {
    std::map<std::string, JsonValue> result;
    std::string src = params.has("source") && params.find("source")->isString()
        ? params.find("source")->asString() : "";
    std::string dst = params.has("dest") && params.find("dest")->isString()
        ? params.find("dest")->asString() : "";

    if (!src.empty() && !dst.empty()) {
        namespace fs = std::filesystem;
        try {
            fs::rename(fileRoot_ + "/" + src, fileRoot_ + "/" + dst);
            result["result"] = JsonValue(true);
            notifyFilelistChanged("move", dst, "gcodes");
        } catch (const std::exception& e) {
            result["error"] = JsonValue(std::string("Move failed: ") + e.what());
        }
    } else {
        result["error"] = JsonValue("Missing 'source' or 'dest' parameter");
    }
    return JsonValue(result);
}

JsonValue KlippyServer::handleServerFilesCopy(const JsonValue& params) {
    std::map<std::string, JsonValue> result;
    std::string src = params.has("source") && params.find("source")->isString()
        ? params.find("source")->asString() : "";
    std::string dst = params.has("dest") && params.find("dest")->isString()
        ? params.find("dest")->asString() : "";

    if (!src.empty() && !dst.empty()) {
        namespace fs = std::filesystem;
        try {
            fs::copy(fileRoot_ + "/" + src, fileRoot_ + "/" + dst,
                     fs::copy_options::recursive);
            result["result"] = JsonValue(true);
            notifyFilelistChanged("copy", dst, "gcodes");
        } catch (const std::exception& e) {
            result["error"] = JsonValue(std::string("Copy failed: ") + e.what());
        }
    } else {
        result["error"] = JsonValue("Missing 'source' or 'dest' parameter");
    }
    return JsonValue(result);
}

JsonValue KlippyServer::handleServerFilesDelete(const JsonValue& params) {
    std::map<std::string, JsonValue> result;
    std::string path = params.has("path") && params.find("path")->isString()
        ? params.find("path")->asString() : "";

    if (!path.empty()) {
        namespace fs = std::filesystem;
        try {
            fs::remove_all(fileRoot_ + "/" + path);
            result["result"] = JsonValue(true);
            notifyFilelistChanged("delete", path, "gcodes");
        } catch (const std::exception& e) {
            result["error"] = JsonValue(std::string("Delete failed: ") + e.what());
        }
    } else {
        result["error"] = JsonValue("Missing 'path' parameter");
    }
    return JsonValue(result);
}

JsonValue KlippyServer::handleServerFilesUpload(const JsonValue& params) {
    // File upload is typically done via HTTP multipart in Moonraker.
    // For UDS, we accept a simple path + content.
    std::map<std::string, JsonValue> result;
    std::string path = params.has("path") && params.find("path")->isString()
        ? params.find("path")->asString() : "";
    std::string content = params.has("content") && params.find("content")->isString()
        ? params.find("content")->asString() : "";

    if (!path.empty()) {
        namespace fs = std::filesystem;
        try {
            fs::path fullPath = fs::path(fileRoot_) / path;
            fs::create_directories(fullPath.parent_path());
            std::ofstream f(fullPath, std::ios::binary);
            if (f) {
                f << content;
                result["result"] = JsonValue(true);
                notifyFilelistChanged("upload", path, "gcodes");
            } else {
                result["error"] = JsonValue("Failed to open file for writing");
            }
        } catch (const std::exception& e) {
            result["error"] = JsonValue(std::string("Upload failed: ") + e.what());
        }
    } else {
        result["error"] = JsonValue("Missing 'path' parameter");
    }
    return JsonValue(result);
}

void KlippyServer::notifyFilelistChanged(const std::string& action,
                                          const std::string& path,
                                          const std::string& root) {
    std::vector<FilelistChangedCallback> cbs;
    {
        std::lock_guard<std::recursive_mutex> lock(mutex_);
        cbs = filelistChangedCbs_;
    }
    for (const auto& cb : cbs) {
        if (cb) cb(action, path, root);
    }
}

} // namespace tether::klipper::klippy

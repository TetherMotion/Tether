/**
 * @file KlippyUdsServerFileEndpoints.cpp
 * @brief File operation and server info endpoint handlers
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
// Additional Moonraker-compatible endpoint handlers
// ============================================================================

JsonValue KlippyUdsServer::handleServerInfo(const JsonValue& params) {
    std::map<std::string, JsonValue> result;
    std::map<std::string, JsonValue> info;
    info["version"] = JsonValue(config_.softwareVersion);
    info["state"] = JsonValue(stateToString(state_));
    info["state_message"] = JsonValue(stateMessage_);
    info["klippy_connected"] = JsonValue(state_ != PrinterState::Shutdown);
    info["cpu_info"] = JsonValue("tether-klipper");
    result["result"] = JsonValue(info);
    return JsonValue(result);
}

JsonValue KlippyUdsServer::handleServerFilesList(const JsonValue& params) {
    std::map<std::string, JsonValue> result;
    std::vector<JsonValue> files;

    // List files from the file root directory
    namespace fs = std::filesystem;
    if (fs::exists(fileRoot_)) {
        try {
            for (const auto& entry : fs::recursive_directory_iterator(fileRoot_)) {
                if (!entry.is_regular_file()) continue;
                std::map<std::string, JsonValue> fileInfo;
                std::string relPath = fs::relative(entry.path(), fileRoot_).string();
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

JsonValue KlippyUdsServer::handleServerFilesMetadata(const JsonValue& params) {
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

JsonValue KlippyUdsServer::handleMachineSystemInfo(const JsonValue& params) {
    std::map<std::string, JsonValue> result;
    std::map<std::string, JsonValue> info;

    // Read CPU info from /proc/cpuinfo
    std::string cpuModel = "unknown";
    std::ifstream cpuinfo("/proc/cpuinfo");
    if (cpuinfo) {
        std::string line;
        while (std::getline(cpuinfo, line)) {
            if (line.substr(0, 10) == "model name") {
                auto pos = line.find(':');
                if (pos != std::string::npos) {
                    cpuModel = line.substr(pos + 2);
                    break;
                }
            }
        }
    }

    // Read OS info from /etc/os-release
    std::string osName = "Linux";
    std::ifstream osrelease("/etc/os-release");
    if (osrelease) {
        std::string line;
        while (std::getline(osrelease, line)) {
            if (line.substr(0, 11) == "PRETTY_NAME") {
                auto pos = line.find('=');
                if (pos != std::string::npos) {
                    osName = line.substr(pos + 1);
                    // Strip quotes
                    if (!osName.empty() && osName.front() == '"') osName.erase(0, 1);
                    if (!osName.empty() && osName.back() == '"') osName.pop_back();
                    break;
                }
            }
        }
    }

    info["cpu_info"] = JsonValue(cpuModel);
    info["python_version"] = JsonValue("3.x");
    info["system_os"] = JsonValue(osName);
    info["hostname"] = JsonValue("tether-klipper");
    result["result"] = JsonValue(info);
    return JsonValue(result);
}

JsonValue KlippyUdsServer::handleMachineProcstats(const JsonValue& params) {
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
            if (line.substr(0, 9) == "MemTotal:") {
                std::istringstream iss(line.substr(9));
                iss >> memTotal;
            } else if (line.substr(0, 13) == "MemAvailable:") {
                std::istringstream iss(line.substr(13));
                iss >> memAvail;
            }
        }
        if (memTotal > 0) {
            memUsage = memTotal - memAvail;
        }
    }

    stats["cpu_usage"] = JsonValue(cpuUsage);
    stats["memory_usage"] = JsonValue(memUsage);
    stats["webhooks_connections"] = JsonValue(static_cast<int64_t>(connections_.size()));
    result["result"] = JsonValue(stats);
    return JsonValue(result);
}

// ============================================================================
// New Moonraker-compatible endpoint handlers
// ============================================================================

JsonValue KlippyUdsServer::handleServerTemperatureStore(const JsonValue& params) {
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

JsonValue KlippyUdsServer::handleServerGcodeStore(const JsonValue& params) {
    std::map<std::string, JsonValue> result;
    std::vector<JsonValue> gcodes;

    for (const auto& entry : gcodeStore_) {
        std::map<std::string, JsonValue> gc;
        gc["message"] = JsonValue(entry.message);
        auto secs = std::chrono::duration_cast<std::chrono::seconds>(
            entry.timestamp.time_since_epoch()).count();
        gc["time"] = JsonValue(static_cast<double>(secs));
        gcodes.push_back(JsonValue(gc));
    }

    result["result"] = JsonValue(gcodes);
    return JsonValue(result);
}

JsonValue KlippyUdsServer::handleServerFilesDirectory(const JsonValue& params) {
    std::map<std::string, JsonValue> result;
    std::map<std::string, JsonValue> dirInfo;

    std::string path = params.has("path") && params.find("path")->isString()
        ? params.find("path")->asString() : "/";
    std::string action = params.has("action") && params.find("action")->isString()
        ? params.find("action")->asString() : "list";

    std::string fullPath = fileRoot_ + "/" + path;
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
        } catch (...) {
            result["result"] = JsonValue(false);
        }
    } else if (action == "delete_dir") {
        try {
            fs::remove_all(fullPath);
            result["result"] = JsonValue(true);
        } catch (...) {
            result["result"] = JsonValue(false);
        }
    }
    return JsonValue(result);
}

JsonValue KlippyUdsServer::handleServerFilesMove(const JsonValue& params) {
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
        } catch (...) {
            result["result"] = JsonValue(false);
        }
    } else {
        result["result"] = JsonValue(false);
    }
    return JsonValue(result);
}

JsonValue KlippyUdsServer::handleServerFilesCopy(const JsonValue& params) {
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
        } catch (...) {
            result["result"] = JsonValue(false);
        }
    } else {
        result["result"] = JsonValue(false);
    }
    return JsonValue(result);
}

JsonValue KlippyUdsServer::handleServerFilesDelete(const JsonValue& params) {
    std::map<std::string, JsonValue> result;
    std::string path = params.has("path") && params.find("path")->isString()
        ? params.find("path")->asString() : "";

    if (!path.empty()) {
        namespace fs = std::filesystem;
        try {
            fs::remove_all(fileRoot_ + "/" + path);
            result["result"] = JsonValue(true);
        } catch (...) {
            result["result"] = JsonValue(false);
        }
    } else {
        result["result"] = JsonValue(false);
    }
    return JsonValue(result);
}

JsonValue KlippyUdsServer::handleServerFilesUpload(const JsonValue& params) {
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
            } else {
                result["result"] = JsonValue(false);
            }
        } catch (...) {
            result["result"] = JsonValue(false);
        }
    } else {
        result["result"] = JsonValue(false);
    }
    return JsonValue(result);
}

} // namespace tether::klipper::klippy

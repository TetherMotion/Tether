/**
 * @file KlippyServerC1Endpoints.cpp
 * @brief C1 Moonraker-compatible endpoint handlers
 */

#include "tether/klipper/klippy/KlippyServer.hpp"
#include "tether/klipper/klippy/AdvancedObjects.hpp"

#include <algorithm>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <set>
#include <sstream>
#include <sys/stat.h>
#include <sys/wait.h>

namespace tether::klipper::klippy {

namespace {

/**
 * @brief Run a shell command, returning its exit status (or -1 on spawn
 *        failure).  When @p output is non-null it receives combined
 *        stdout+stderr.
 */
int runCommand(const std::string& cmd, std::string* output = nullptr) {
    FILE* fp = popen((cmd + " 2>&1").c_str(), "r");
    if (!fp) return -1;
    char buf[512];
    std::string out;
    while (fgets(buf, sizeof(buf), fp)) out += buf;
    int status = pclose(fp);
    if (output) *output = out;
    if (status < 0) return -1;
    if (WIFEXITED(status)) return WEXITSTATUS(status);
    return -1;
}

/**@brief Whether the file looks like a git work tree. */
bool isGitRepo(const std::string& dir) {
    if (dir.empty()) return false;
    return runCommand("git -C \"" + dir + "\" rev-parse --is-inside-work-tree") == 0;
}

/**@brief Resolve an update component name to its source directory.
 *        Returns empty when the component is unknown. */
std::string updateComponentDir(const UdsServerConfig& cfg,
                               const std::string& component) {
    // The only component with a known on-disk source tree is the klipper
    // firmware itself (Tether's klipperPath).
    if (component == "klipper" || component == "tether") {
        return cfg.klipperPath;
    }
    return {};
}

/**@brief Standard base64 decode (RFC 4648, with padding).  Returns empty on
 *        malformed input. */
std::string base64Decode(const std::string& in) {
    static const std::string tbl =
        "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
    std::string out;
    out.reserve(in.size() * 3 / 4);
    int val = 0, bits = -8;
    for (unsigned char c : in) {
        if (c == '=') break;
        size_t pos = tbl.find(static_cast<char>(c));
        if (pos == std::string::npos) {
            if (c == '\n' || c == '\r' || c == ' ' || c == '\t') continue;
            return {};
        }
        val = (val << 6) + static_cast<int>(pos);
        bits += 6;
        if (bits >= 0) {
            out.push_back(static_cast<char>((val >> bits) & 0xFF));
            bits -= 8;
        }
    }
    return out;
}

/**@brief Extract `; THUMBNAIL` blocks from G-code source text.
 *        Returns a list of {width, height, mime, data(base64)}. */
struct EmbeddedThumbnail {
    int width{0};
    int height{0};
    std::string mimeType;
    std::string base64;
};

std::vector<EmbeddedThumbnail> extractThumbnails(const std::string& path) {
    std::vector<EmbeddedThumbnail> thumbs;
    std::ifstream f(path);
    if (!f) return thumbs;
    std::string line;
    EmbeddedThumbnail* cur = nullptr;
    std::string b64;
    while (std::getline(f, line)) {
        if (line.rfind("; THUMBNAIL begin", 0) == 0) {
            // "; THUMBNAIL begin <w>x<h> <size>"
            EmbeddedThumbnail t;
            auto bpos = line.find("begin");
            if (bpos != std::string::npos) {
                auto rest = line.substr(bpos + 5);
                auto xpos = rest.find('x');
                if (xpos != std::string::npos) {
                    t.width = std::atoi(rest.c_str());
                    t.height = std::atoi(rest.c_str() + xpos + 1);
                }
            }
            thumbs.push_back(t);
            cur = &thumbs.back();
            b64.clear();
        } else if (cur && line.rfind("; THUMBNAIL end", 0) == 0) {
            std::string decoded = base64Decode(b64);
            if (!decoded.empty()) {
                if (decoded.size() > 8 && decoded[0] == '\x89' &&
                    decoded.compare(1, 3, "PNG") == 0) {
                    cur->mimeType = "image/png";
                } else if (decoded.size() > 3 &&
                           static_cast<unsigned char>(decoded[0]) == 0xFF &&
                           static_cast<unsigned char>(decoded[1]) == 0xD8) {
                    cur->mimeType = "image/jpeg";
                } else {
                    cur->mimeType = "image/png";
                }
                cur->base64 = b64;
            }
            cur = nullptr;
            b64.clear();
        } else if (cur) {
            // Base64 payload lines (may be raw or "; "-prefixed depending
            // on the slicer).
            std::string payload = line;
            if (payload.rfind(";", 0) == 0) {
                payload = payload.substr(1);
                if (!payload.empty() && payload[0] == ' ') payload.erase(0, 1);
            }
            for (char c : payload) {
                if (!std::isspace(static_cast<unsigned char>(c))) b64 += c;
            }
        }
    }
    // Drop entries whose payload never terminated/decoded.
    thumbs.erase(std::remove_if(thumbs.begin(), thumbs.end(),
                                [](const EmbeddedThumbnail& t) {
                                    return t.base64.empty();
                                }),
                 thumbs.end());
    return thumbs;
}

} // namespace

// ============================================================================
// C1: New Moonraker-compatible endpoint handlers
// ============================================================================

JsonValue KlippyServer::handleServerConfig(const JsonValue& params) {
    return withLockResult(serverConfig_);
}

JsonValue KlippyServer::handleServerFilesRoots(const JsonValue& params) {
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

JsonValue KlippyServer::handleServerFilesCreateDir(const JsonValue& params) {
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

JsonValue KlippyServer::handleServerFilesMetascan(const JsonValue& params) {
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

JsonValue KlippyServer::handleServerFilesThumbnails(const JsonValue& params) {
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

    // Extract embedded "; THUMBNAIL" blocks (slicer-embedded PNG/JPEG).
    auto thumbs = extractThumbnails(fullPath.string());
    std::vector<JsonValue> arr;
    for (const auto& t : thumbs) {
        std::map<std::string, JsonValue> info;
        info["width"] = JsonValue(static_cast<int64_t>(t.width));
        info["height"] = JsonValue(static_cast<int64_t>(t.height));
        info["size"] = JsonValue(static_cast<int64_t>(
            base64Decode(t.base64).size()));
        info["mime_type"] = JsonValue(t.mimeType);
        info["data"] = JsonValue(t.base64);
        arr.push_back(JsonValue(info));
    }
    std::map<std::string, JsonValue> payload;
    payload["thumbnails"] = JsonValue(arr);
    result["result"] = JsonValue(payload);
    return JsonValue(result);
}

JsonValue KlippyServer::handleServerLogsRollover(const JsonValue& params) {
    std::map<std::string, JsonValue> result;
    namespace fs = std::filesystem;

    // Moonraker rotates klippy.log -> klippy.log.1 (single generation kept).
    const std::string& logPath = config_.logFile;
    if (logPath.empty() || !fs::exists(logPath)) {
        result["error"] = JsonValue("Log file not found");
        return JsonValue(result);
    }
    std::error_code ec;
    fs::path rotated = fs::path(logPath + ".1");
    fs::rename(logPath, rotated, ec);
    if (ec) {
        result["error"] = JsonValue("Failed to rotate log: " + ec.message());
        return JsonValue(result);
    }
    // Recreate an empty log so writers can continue.
    std::ofstream fresh(logPath, std::ios::trunc);
    if (!fresh) {
        result["error"] = JsonValue("Failed to recreate log file");
        return JsonValue(result);
    }
    result["result"] = JsonValue(true);
    return JsonValue(result);
}

JsonValue KlippyServer::handleServerKlippyLog(const JsonValue& params) {
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

JsonValue KlippyServer::handleServerMoonrakerLog(const JsonValue& params) {
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

JsonValue KlippyServer::handleMachineServicesList(const JsonValue& params) {
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

JsonValue KlippyServer::handleMachineServiceAction(const JsonValue& params,
                                                   const std::string& action) {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    std::map<std::string, JsonValue> result;
    std::string service = params.has("service") && params.find("service")->isString()
        ? params.find("service")->asString() : "";
    // Moonraker also accepts an explicit "action" parameter.
    std::string act = action;
    if (act.empty() && params.has("action") &&
        params.find("action")->isString()) {
        act = params.find("action")->asString();
    }

    if (service.empty()) {
        result["error"] = JsonValue("Service name required");
        return JsonValue(result);
    }
    if (act.empty()) {
        result["error"] = JsonValue("Service action required");
        return JsonValue(result);
    }
    if (act != "start" && act != "stop" && act != "restart") {
        result["error"] = JsonValue("Invalid service action '" + act + "'");
        return JsonValue(result);
    }

    auto it = services_.find(service);
    if (it == services_.end()) {
        result["error"] = JsonValue("Service not found");
        return JsonValue(result);
    }

    // Drive the real systemd unit when available.  Moonraker service names
    // map to systemd units; append ".service" when no suffix is present.
    std::string unit = service;
    if (unit.find('.') == std::string::npos) unit += ".service";
    std::string output;
    int rc = runCommand("systemctl " + act + " \"" + unit + "\"", &output);
    if (rc != 0) {
        result["error"] = JsonValue("systemctl " + act + " " + unit +
                                    " failed (rc=" + std::to_string(rc) + ")" +
                                    (output.empty() ? "" : ": " + output));
        return JsonValue(result);
    }

    // Reflect the new state in the service table.
    if (act == "stop") {
        it->second.activeState = "inactive";
        it->second.subState = "dead";
    } else {
        it->second.activeState = "active";
        it->second.subState = "running";
    }
    result["result"] = JsonValue(true);
    return JsonValue(result);
}

JsonValue KlippyServer::handleMachineUpdateList(const JsonValue& params) {
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

JsonValue KlippyServer::handleMachineUpdateRefresh(const JsonValue& params) {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    std::map<std::string, JsonValue> result;

    // Refresh every tracked component that maps to a local git checkout:
    // fetch and compare HEAD against its upstream.
    bool any = false;
    for (const auto& [component, _] : updateStatus_) {
        std::string dir = updateComponentDir(config_, component);
        if (dir.empty() || !isGitRepo(dir)) continue;
        any = true;
        runCommand("git -C \"" + dir + "\" fetch --quiet");
        std::string out;
        int rc = runCommand(
            "git -C \"" + dir + "\" rev-list --count HEAD..@{u} 2>/dev/null",
            &out);
        int behind = (rc == 0) ? std::atoi(out.c_str()) : 0;
        updateStatus_[component] = (behind > 0)
            ? "update-available" : "up-to-date";
    }
    if (!any && updateStatus_.empty()) {
        // Nothing tracked — still a successful no-op refresh.
        result["result"] = JsonValue(true);
        return JsonValue(result);
    }
    result["result"] = JsonValue(true);
    return JsonValue(result);
}

JsonValue KlippyServer::handleMachineUpdateUpdate(const JsonValue& params) {
    std::map<std::string, JsonValue> result;
    std::string component = params.has("component") && params.find("component")->isString()
        ? params.find("component")->asString() : "";
    if (component.empty()) {
        result["error"] = JsonValue("Missing 'component' parameter");
        return JsonValue(result);
    }
    std::string dir = updateComponentDir(config_, component);
    if (dir.empty()) {
        result["error"] = JsonValue("Unknown update component '" + component + "'");
        return JsonValue(result);
    }
    if (!isGitRepo(dir)) {
        result["error"] = JsonValue("Component '" + component +
                                    "' is not a git checkout: " + dir);
        return JsonValue(result);
    }

    std::string output;
    int rc = runCommand("git -C \"" + dir + "\" pull --ff-only", &output);
    if (rc != 0) {
        result["error"] = JsonValue("git pull failed (rc=" +
                                    std::to_string(rc) + ")" +
                                    (output.empty() ? "" : ": " + output));
        return JsonValue(result);
    }
    {
        std::lock_guard<std::recursive_mutex> lock(mutex_);
        updateStatus_[component] = "up-to-date";
    }
    result["result"] = JsonValue(true);
    return JsonValue(result);
}

JsonValue KlippyServer::handleMachineUpdateRecover(const JsonValue& params) {
    std::map<std::string, JsonValue> result;
    std::string component = params.has("component") && params.find("component")->isString()
        ? params.find("component")->asString() : "";
    if (component.empty()) {
        result["error"] = JsonValue("Missing 'component' parameter");
        return JsonValue(result);
    }
    std::string dir = updateComponentDir(config_, component);
    if (dir.empty()) {
        result["error"] = JsonValue("Unknown update component '" + component + "'");
        return JsonValue(result);
    }
    if (!isGitRepo(dir)) {
        result["error"] = JsonValue("Component '" + component +
                                    "' is not a git checkout: " + dir);
        return JsonValue(result);
    }

    // Recovery = restore tracked files to HEAD and remove untracked
    // artifacts, matching Moonraker's "recover to last known good state".
    std::string output;
    int rc = runCommand("git -C \"" + dir + "\" reset --hard HEAD && "
                        "git -C \"" + dir + "\" clean -fd", &output);
    if (rc != 0) {
        result["error"] = JsonValue("git recovery failed (rc=" +
                                    std::to_string(rc) + ")" +
                                    (output.empty() ? "" : ": " + output));
        return JsonValue(result);
    }
    {
        std::lock_guard<std::recursive_mutex> lock(mutex_);
        updateStatus_[component] = "up-to-date";
    }
    result["result"] = JsonValue(true);
    return JsonValue(result);
}

JsonValue KlippyServer::handleDatabaseList(const JsonValue& params) {
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

JsonValue KlippyServer::handleDatabaseGet(const JsonValue& params) {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    std::string ns = params.has("namespace") && params.find("namespace")->isString()
        ? params.find("namespace")->asString() : "";
    std::string key = params.has("key") && params.find("key")->isString()
        ? params.find("key")->asString() : "";

    auto val = databaseGet(ns, key);
    // Moonraker returns { namespace, key, value } for get_item.
    // For missing keys, return an empty object so Mainsail can use the
    // 'in' operator on it (returning null causes a TypeError).
    std::map<std::string, JsonValue> result;
    if (val) {
        result["value"] = *val;
    } else {
        result["value"] = JsonValue(std::map<std::string, JsonValue>{});
    }
    result["namespace"] = JsonValue(ns);
    result["key"] = JsonValue(key);
    return JsonValue(result);
}

JsonValue KlippyServer::handleDatabasePut(const JsonValue& params) {
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

JsonValue KlippyServer::handleDatabaseDelete(const JsonValue& params) {
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

JsonValue KlippyServer::handleJobQueueStatus(const JsonValue& params) {
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

JsonValue KlippyServer::handleJobQueuePost(const JsonValue& params) {
    std::vector<JobQueueChangedCallback> cbs;
    {
        std::lock_guard<std::recursive_mutex> lock(mutex_);
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
        cbs = jobQueueChangedCbs_;
    }
    for (const auto& cb : cbs) {
        if (cb) cb("post");
    }
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    std::map<std::string, JsonValue> result;
    result["result"] = JsonValue(static_cast<int64_t>(jobQueue_.size()));
    return JsonValue(result);
}

JsonValue KlippyServer::handleJobQueueDelete(const JsonValue& params) {
    std::vector<JobQueueChangedCallback> cbs;
    {
        std::lock_guard<std::recursive_mutex> lock(mutex_);
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
        cbs = jobQueueChangedCbs_;
    }
    for (const auto& cb : cbs) {
        if (cb) cb("delete");
    }
    std::map<std::string, JsonValue> result;
    result["result"] = JsonValue(true);
    return JsonValue(result);
}

JsonValue KlippyServer::handleJobHistoryList(const JsonValue& params) {
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

JsonValue KlippyServer::handleJobHistoryGet(const JsonValue& params) {
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

JsonValue KlippyServer::handleAnnouncementsList(const JsonValue& params) {
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

JsonValue KlippyServer::handleAnnouncementsUpdate(const JsonValue& params) {
    std::map<std::string, JsonValue> result;

    // Announcement sources, in priority order:
    //  1. explicit "path" parameter (local JSON file)
    //  2. announcements.json next to the printer config
    //  3. announcements.json inside klipperPath
    std::string path;
    if (params.has("path") && params.find("path")->isString()) {
        path = params.find("path")->asString();
    }
    if (path.empty()) {
        namespace fs = std::filesystem;
        fs::path cfgDir = fs::path(config_.configFile).parent_path();
        fs::path candidate = cfgDir / "announcements.json";
        if (fs::exists(candidate)) path = candidate.string();
    }
    if (path.empty() && !config_.klipperPath.empty()) {
        namespace fs = std::filesystem;
        fs::path candidate =
            fs::path(config_.klipperPath) / "announcements.json";
        if (fs::exists(candidate)) path = candidate.string();
    }
    if (path.empty()) {
        result["error"] = JsonValue(
            "No announcement source: set 'path' or provide announcements.json");
        return JsonValue(result);
    }

    std::ifstream f(path);
    if (!f) {
        result["error"] = JsonValue("Cannot open announcement source: " + path);
        return JsonValue(result);
    }
    std::stringstream ss;
    ss << f.rdbuf();
    auto doc = JsonValue::parse(ss.str());
    if (!doc || !doc->isArray()) {
        result["error"] = JsonValue("Invalid announcement feed format");
        return JsonValue(result);
    }

    std::lock_guard<std::recursive_mutex> lock(mutex_);
    size_t added = 0;
    for (const auto& item : doc->asArray()) {
        if (!item.isObject()) continue;
        auto getStr = [&](const char* k) -> std::string {
            const JsonValue* v = item.find(k);
            return (v && v->isString()) ? v->asString() : "";
        };
        std::string entryId = getStr("entry_id");
        if (entryId.empty()) entryId = getStr("id");
        if (entryId.empty()) continue;
        bool exists = false;
        for (const auto& a : announcements_) {
            if (a.entryId == entryId) { exists = true; break; }
        }
        if (exists) continue;
        Announcement a;
        a.entryId = entryId;
        a.title = getStr("title");
        a.description = getStr("description");
        a.url = getStr("url");
        a.date = getStr("date");
        a.severity = getStr("severity");
        if (a.severity.empty()) a.severity = "info";
        announcements_.push_back(a);
        ++added;
    }
    result["result"] = JsonValue(static_cast<int64_t>(added));
    return JsonValue(result);
}

JsonValue KlippyServer::handleAnnouncementsDismiss(const JsonValue& params) {
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

JsonValue KlippyServer::handleWebcamsList(const JsonValue& params) {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    // Moonraker returns { "webcams": [ { ... }, ... ] }
    std::vector<JsonValue> webcamArray;
    for (const auto& [name, cam] : webcams_) {
        std::map<std::string, JsonValue> camInfo;
        camInfo["name"] = JsonValue(cam.name);
        camInfo["location"] = JsonValue(cam.location);
        camInfo["service"] = JsonValue(cam.service);
        camInfo["enabled"] = JsonValue(cam.enabled);
        camInfo["icon"] = JsonValue(cam.icon);
        camInfo["target_fps"] = JsonValue(static_cast<int64_t>(cam.targetFps));
        camInfo["target_fps_idle"] = JsonValue(static_cast<int64_t>(cam.targetFpsIdle));
        camInfo["stream_url"] = JsonValue(cam.streamUrl);
        camInfo["snapshot_url"] = JsonValue(cam.snapshotUrl);
        camInfo["flip_horizontal"] = JsonValue(cam.flipHorizontal);
        camInfo["flip_vertical"] = JsonValue(cam.flipVertical);
        camInfo["rotation"] = JsonValue(static_cast<int64_t>(cam.rotation));
        camInfo["aspect_ratio"] = JsonValue(cam.aspectRatio);
        camInfo["extra_data"] = JsonValue(std::map<std::string, JsonValue>{});
        camInfo["source"] = JsonValue(cam.source);
        camInfo["uid"] = JsonValue(cam.uid);
        webcamArray.push_back(JsonValue(camInfo));
    }
    std::map<std::string, JsonValue> result;
    result["webcams"] = JsonValue(webcamArray);
    return JsonValue(result);
}

JsonValue KlippyServer::handleWebcamsGet(const JsonValue& params) {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    std::string name = params.has("name") && params.find("name")->isString()
        ? params.find("name")->asString() : "";
    auto it = webcams_.find(name);
    if (it != webcams_.end()) {
        const auto& cam = it->second;
        std::map<std::string, JsonValue> camInfo;
        camInfo["name"] = JsonValue(cam.name);
        camInfo["location"] = JsonValue(cam.location);
        camInfo["service"] = JsonValue(cam.service);
        camInfo["enabled"] = JsonValue(cam.enabled);
        camInfo["icon"] = JsonValue(cam.icon);
        camInfo["target_fps"] = JsonValue(static_cast<int64_t>(cam.targetFps));
        camInfo["target_fps_idle"] = JsonValue(static_cast<int64_t>(cam.targetFpsIdle));
        camInfo["stream_url"] = JsonValue(cam.streamUrl);
        camInfo["snapshot_url"] = JsonValue(cam.snapshotUrl);
        camInfo["flip_horizontal"] = JsonValue(cam.flipHorizontal);
        camInfo["flip_vertical"] = JsonValue(cam.flipVertical);
        camInfo["rotation"] = JsonValue(static_cast<int64_t>(cam.rotation));
        camInfo["aspect_ratio"] = JsonValue(cam.aspectRatio);
        camInfo["extra_data"] = JsonValue(std::map<std::string, JsonValue>{});
        camInfo["source"] = JsonValue(cam.source);
        camInfo["uid"] = JsonValue(cam.uid);
        std::map<std::string, JsonValue> result;
        result["webcam"] = JsonValue(camInfo);
        return JsonValue(result);
    } else {
        std::map<std::string, JsonValue> result;
        result["error"] = JsonValue("Webcam not found");
        return JsonValue(result);
    }
}

JsonValue KlippyServer::handleWebcamsTest(const JsonValue& params) {
    std::map<std::string, JsonValue> result;
    std::string name = params.has("name") && params.find("name")->isString()
        ? params.find("name")->asString() : "";
    auto it = webcams_.find(name);
    if (it != webcams_.end()) {
        // Return a test result
        std::map<std::string, JsonValue> testResult;
        testResult["status"] = JsonValue("ok");
        testResult["url"] = JsonValue(it->second.streamUrl);
        result["result"] = JsonValue(testResult);
    } else {
        result["error"] = JsonValue("Webcam not found");
    }
    return JsonValue(result);
}

JsonValue KlippyServer::handleDevicesList(const JsonValue& params) {
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

JsonValue KlippyServer::handleDevicesGet(const JsonValue& params) {
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

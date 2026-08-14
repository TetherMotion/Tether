/**
 * @file KlippyUdsServerE1Endpoints.cpp
 * @brief E1 additional endpoint handlers (job queue, webcams, power, access, bots, notepad, spoolman, devices)
 */

#include "tether/klipper/klippy/KlippyServer.hpp"
#include "tether/klipper/klippy/AdvancedObjects.hpp"

#include <algorithm>
#include <chrono>
#include <ctime>
#include <format>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <netdb.h>
#include <set>
#include <sstream>
#include <sys/stat.h>
#include <cstring>

namespace tether::klipper::klippy {

// ============================================================================
// E1-High: Additional Moonraker-compatible endpoint handlers
// ============================================================================

JsonValue KlippyServer::handleJobQueuePause(const JsonValue& params) {
    std::vector<JobQueueChangedCallback> cbs;
    {
        std::lock_guard<std::recursive_mutex> lock(mutex_);
        jobQueuePaused_ = true;
        cbs = jobQueueChangedCbs_;
    }
    for (const auto& cb : cbs) {
        if (cb) cb("pause");
    }
    std::map<std::string, JsonValue> result;
    result["result"] = JsonValue("paused");
    return JsonValue(result);
}

JsonValue KlippyServer::handleJobQueueStart(const JsonValue& params) {
    std::vector<JobQueueChangedCallback> cbs;
    {
        std::lock_guard<std::recursive_mutex> lock(mutex_);
        jobQueuePaused_ = false;
        cbs = jobQueueChangedCbs_;
    }
    for (const auto& cb : cbs) {
        if (cb) cb("start");
    }
    std::map<std::string, JsonValue> result;
    result["result"] = JsonValue("started");
    return JsonValue(result);
}

JsonValue KlippyServer::handleJobQueueJumpTo(const JsonValue& params) {
    std::vector<JobQueueChangedCallback> cbs;
    {
        std::lock_guard<std::recursive_mutex> lock(mutex_);
        std::map<std::string, JsonValue> result;
        const JsonValue* jobIdVal = params.find("job_id");
        if (jobIdVal && jobIdVal->isInt()) {
            int64_t jobId = jobIdVal->asInt();
            // Find the job with this ID in the queue
            if (jobId >= 0 && static_cast<size_t>(jobId) < jobQueue_.size()) {
                jobQueueCurrentIndex_ = static_cast<size_t>(jobId);
                cbs = jobQueueChangedCbs_;
            } else {
                result["error"] = JsonValue("Job ID out of range");
                return JsonValue(result);
            }
        } else {
            result["error"] = JsonValue("Missing or invalid 'job_id' parameter");
            return JsonValue(result);
        }
    }
    for (const auto& cb : cbs) {
        if (cb) cb("jump_to");
    }
    std::map<std::string, JsonValue> result;
    result["result"] = JsonValue("ok");
    return JsonValue(result);
}

JsonValue KlippyServer::handleJobHistoryDelete(const JsonValue& params) {
    std::vector<HistoryChangedCallback> cbs;
    bool changed = false;
    int64_t deletedJobId = -1;
    {
        std::lock_guard<std::recursive_mutex> lock(mutex_);
        std::map<std::string, JsonValue> result;
        const JsonValue* jobIdVal = params.find("job_id");
        if (jobIdVal && jobIdVal->isInt()) {
            int64_t jobId = jobIdVal->asInt();
            auto it = std::remove_if(jobHistory_.begin(), jobHistory_.end(),
                [jobId](const JobHistoryEntry& e) { return e.jobId == jobId; });
            if (it != jobHistory_.end()) {
                jobHistory_.erase(it, jobHistory_.end());
                changed = true;
                deletedJobId = jobId;
                cbs = historyChangedCbs_;
                result["result"] = JsonValue(true);
            } else {
                result["error"] = JsonValue("Job ID not found");
                return JsonValue(result);
            }
        } else {
            const JsonValue* allVal = params.find("all");
            if (allVal && allVal->isBool() && allVal->asBool()) {
                // Delete all
                jobHistory_.clear();
                changed = true;
                cbs = historyChangedCbs_;
                result["result"] = JsonValue(true);
            } else {
                result["error"] = JsonValue("Missing 'job_id' parameter");
                return JsonValue(result);
            }
        }
    }
    if (changed) {
        for (const auto& cb : cbs) {
            if (cb) cb("delete", deletedJobId);
        }
    }
    std::map<std::string, JsonValue> result;
    result["result"] = JsonValue(true);
    return JsonValue(result);
}

JsonValue KlippyServer::handleWebcamsUpdate(const JsonValue& params) {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    std::map<std::string, JsonValue> result;
    std::string name = params.has("name") && params.find("name")->isString()
        ? params.find("name")->asString() : "";
    if (name.empty()) {
        result["error"] = JsonValue("Missing 'name' parameter");
        return JsonValue(result);
    }
    auto it = webcams_.find(name);
    if (it == webcams_.end()) {
        result["error"] = JsonValue("Webcam not found");
        return JsonValue(result);
    }
    // Update fields if present
    const JsonValue* urlVal = params.find("url");
    if (urlVal && urlVal->isString()) it->second.url = urlVal->asString();
    const JsonValue* serviceVal = params.find("service");
    if (serviceVal && serviceVal->isString()) it->second.service = serviceVal->asString();
    const JsonValue* enabledVal = params.find("enabled");
    if (enabledVal && enabledVal->isBool()) it->second.enabled = enabledVal->asBool();
    const JsonValue* rotationVal = params.find("rotation");
    if (rotationVal && rotationVal->isInt()) it->second.rotation = rotationVal->asInt();
    // Return updated webcam
    std::map<std::string, JsonValue> webcam;
    webcam["name"] = JsonValue(it->second.name);
    webcam["url"] = JsonValue(it->second.url);
    webcam["service"] = JsonValue(it->second.service);
    webcam["enabled"] = JsonValue(it->second.enabled);
    webcam["rotation"] = JsonValue(it->second.rotation);
    webcam["aspect_ratio"] = JsonValue(it->second.aspectRatio);
    webcam["source"] = JsonValue(it->second.source);
    result["result"] = JsonValue(webcam);
    return JsonValue(result);
}

JsonValue KlippyServer::handleWebcamsDelete(const JsonValue& params) {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    std::map<std::string, JsonValue> result;
    std::string name = params.has("name") && params.find("name")->isString()
        ? params.find("name")->asString() : "";
    if (name.empty()) {
        result["error"] = JsonValue("Missing 'name' parameter");
        return JsonValue(result);
    }
    auto it = webcams_.find(name);
    if (it == webcams_.end()) {
        result["error"] = JsonValue("Webcam not found");
        return JsonValue(result);
    }
    webcams_.erase(it);
    result["result"] = JsonValue(true);
    return JsonValue(result);
}

JsonValue KlippyServer::handleDevicePowerOn(const JsonValue& params) {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    std::map<std::string, JsonValue> result;
    std::string device = params.has("device") && params.find("device")->isString()
        ? params.find("device")->asString() : "";
    auto it = powerDevices_.find(device);
    if (it == powerDevices_.end()) {
        result["error"] = JsonValue("Device not found");
        return JsonValue(result);
    }
    it->second.state = "on";
    std::map<std::string, JsonValue> devInfo;
    devInfo["device"] = JsonValue(device);
    devInfo["state"] = JsonValue("on");
    result["result"] = JsonValue(devInfo);
    return JsonValue(result);
}

JsonValue KlippyServer::handleDevicePowerOff(const JsonValue& params) {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    std::map<std::string, JsonValue> result;
    std::string device = params.has("device") && params.find("device")->isString()
        ? params.find("device")->asString() : "";
    auto it = powerDevices_.find(device);
    if (it == powerDevices_.end()) {
        result["error"] = JsonValue("Device not found");
        return JsonValue(result);
    }
    it->second.state = "off";
    std::map<std::string, JsonValue> devInfo;
    devInfo["device"] = JsonValue(device);
    devInfo["state"] = JsonValue("off");
    result["result"] = JsonValue(devInfo);
    return JsonValue(result);
}

JsonValue KlippyServer::handleDevicePowerToggle(const JsonValue& params) {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    std::map<std::string, JsonValue> result;
    std::string device = params.has("device") && params.find("device")->isString()
        ? params.find("device")->asString() : "";
    auto it = powerDevices_.find(device);
    if (it == powerDevices_.end()) {
        result["error"] = JsonValue("Device not found");
        return JsonValue(result);
    }
    it->second.state = (it->second.state == "on") ? "off" : "on";
    std::map<std::string, JsonValue> devInfo;
    devInfo["device"] = JsonValue(device);
    devInfo["state"] = JsonValue(it->second.state);
    result["result"] = JsonValue(devInfo);
    return JsonValue(result);
}

JsonValue KlippyServer::handleMachineSystemPerms(const JsonValue& params) {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    std::map<std::string, JsonValue> result;
    std::map<std::string, JsonValue> perms;
    for (const auto& [resource, permList] : systemPerms_) {
        std::vector<JsonValue> arr;
        for (const auto& p : permList) arr.push_back(JsonValue(p));
        perms[resource] = JsonValue(arr);
    }
    result["result"] = JsonValue(perms);
    return JsonValue(result);
}

JsonValue KlippyServer::handleAnnouncementsFeed(const JsonValue& params) {
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

JsonValue KlippyServer::handleServerFilesGet(const JsonValue& params) {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    std::map<std::string, JsonValue> result;
    std::string root = params.has("root") && params.find("root")->isString()
        ? params.find("root")->asString() : "gcodes";
    std::string filename = params.has("filename") && params.find("filename")->isString()
        ? params.find("filename")->asString() : "";
    if (filename.empty()) {
        result["error"] = JsonValue("Missing 'filename' parameter");
        return JsonValue(result);
    }
    // Find the root path
    auto rootIt = fileRoots_.find(root);
    std::string basePath = (rootIt != fileRoots_.end()) ? rootIt->second.path : "";
    std::string fullPath = basePath + "/" + filename;
    // Check file size before reading (limit to 16MB to prevent memory exhaustion)
    namespace fs = std::filesystem;
    std::error_code ec;
    auto fileSize = fs::file_size(fullPath, ec);
    if (ec) {
        result["error"] = JsonValue("File not found or inaccessible: " + fullPath);
        return JsonValue(result);
    }
    if (fileSize > 16 * 1024 * 1024) {
        result["error"] = JsonValue("File too large (max 16MB): " + std::to_string(fileSize) + " bytes");
        return JsonValue(result);
    }
    // Read file contents
    std::ifstream f(fullPath, std::ios::binary);
    if (!f.is_open()) {
        result["error"] = JsonValue("File not found: " + fullPath);
        return JsonValue(result);
    }
    std::string content((std::istreambuf_iterator<char>(f)),
                         std::istreambuf_iterator<char>());
    result["result"] = JsonValue(content);
    return JsonValue(result);
}

JsonValue KlippyServer::handleServerLogsList(const JsonValue& params) {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    std::map<std::string, JsonValue> result;
    std::vector<JsonValue> logs;
    for (const auto& lf : logFiles_) {
        std::map<std::string, JsonValue> logInfo;
        logInfo["name"] = JsonValue(lf.name);
        logInfo["path"] = JsonValue(lf.path);
        logInfo["size"] = JsonValue(lf.size);
        logInfo["modified"] = JsonValue(lf.modified);
        logs.push_back(JsonValue(logInfo));
    }
    // Also add default logs if not already present
    if (logFiles_.empty()) {
        std::map<std::string, JsonValue> klippyLog;
        klippyLog["name"] = JsonValue("klippy.log");
        klippyLog["path"] = JsonValue(config_.logFile);
        klippyLog["size"] = JsonValue(static_cast<int64_t>(0));
        klippyLog["modified"] = JsonValue(static_cast<int64_t>(0));
        logs.push_back(JsonValue(klippyLog));

        std::map<std::string, JsonValue> moonrakerLog;
        moonrakerLog["name"] = JsonValue("moonraker.log");
        moonrakerLog["path"] = JsonValue("/tmp/moonraker.log");
        moonrakerLog["size"] = JsonValue(static_cast<int64_t>(0));
        moonrakerLog["modified"] = JsonValue(static_cast<int64_t>(0));
        logs.push_back(JsonValue(moonrakerLog));
    }
    result["result"] = JsonValue(logs);
    return JsonValue(result);
}

// ============================================================================
// E1-Low: Access endpoint handlers
// ============================================================================

JsonValue KlippyServer::handleAccessLogin(const JsonValue& params) {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    std::map<std::string, JsonValue> result;
    std::string username = params.has("username") && params.find("username")->isString()
        ? params.find("username")->asString() : "";
    std::string password = params.has("password") && params.find("password")->isString()
        ? params.find("password")->asString() : "";
    auto it = users_.find(username);
    if (it == users_.end() || it->second.password != password) {
        result["error"] = JsonValue("Invalid credentials");
        return JsonValue(result);
    }
    // Generate a simple token
    std::string token = std::format("{}_jwt_token_{}", username, std::time(nullptr));
    std::map<std::string, JsonValue> loginResult;
    loginResult["token"] = JsonValue(token);
    loginResult["username"] = JsonValue(username);
    loginResult["source"] = JsonValue(it->second.source);
    std::vector<JsonValue> perms;
    for (const auto& p : it->second.permissions) perms.push_back(JsonValue(p));
    loginResult["permissions"] = JsonValue(perms);
    result["result"] = JsonValue(loginResult);
    return JsonValue(result);
}

JsonValue KlippyServer::handleAccessLogout(const JsonValue& params) {
    // Token invalidation is handled by the HTTP transport layer (clears client-side token)
    std::map<std::string, JsonValue> result;
    result["result"] = JsonValue("ok");
    return JsonValue(result);
}

JsonValue KlippyServer::handleAccessUser(const JsonValue& params) {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    std::map<std::string, JsonValue> result;
    std::vector<JsonValue> users;
    for (const auto& [name, user] : users_) {
        std::map<std::string, JsonValue> userInfo;
        userInfo["username"] = JsonValue(name);
        userInfo["source"] = JsonValue(user.source);
        std::vector<JsonValue> perms;
        for (const auto& p : user.permissions) perms.push_back(JsonValue(p));
        userInfo["permissions"] = JsonValue(perms);
        users.push_back(JsonValue(userInfo));
    }
    result["result"] = JsonValue(users);
    return JsonValue(result);
}

JsonValue KlippyServer::handleAccessRefreshJwt(const JsonValue& params) {
    std::map<std::string, JsonValue> result;
    std::string username = params.has("username") && params.find("username")->isString()
        ? params.find("username")->asString() : "";
    if (username.empty()) {
        result["error"] = JsonValue("Missing 'username' parameter");
        return JsonValue(result);
    }
    // Validate user exists
    {
        std::lock_guard<std::recursive_mutex> lock(mutex_);
        auto it = users_.find(username);
        if (it == users_.end()) {
            result["error"] = JsonValue("User not found");
            return JsonValue(result);
        }
    }
    // Generate a refreshed JWT token
    std::string token = std::format("{}_jwt_refreshed_{}", username, std::time(nullptr));
    std::map<std::string, JsonValue> refreshResult;
    refreshResult["token"] = JsonValue(token);
    refreshResult["username"] = JsonValue(username);
    result["result"] = JsonValue(refreshResult);
    return JsonValue(result);
}

JsonValue KlippyServer::handleAccessApiKey(const JsonValue& params) {
    std::map<std::string, JsonValue> result;
    result["result"] = JsonValue(apiKey_);
    return JsonValue(result);
}

JsonValue KlippyServer::handleAccessOneshotToken(const JsonValue& params) {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    std::map<std::string, JsonValue> result;
    std::string token = generateOneshotToken();
    result["result"] = JsonValue(token);
    return JsonValue(result);
}

// ============================================================================
// E1-Low: Bot endpoint handlers
// ============================================================================

JsonValue KlippyServer::handleBotList(const JsonValue& params) {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    std::map<std::string, JsonValue> result;
    std::vector<JsonValue> botList;
    for (const auto& [name, bot] : bots_) {
        std::map<std::string, JsonValue> botInfo;
        botInfo["name"] = JsonValue(name);
        botInfo["type"] = JsonValue(bot.type);
        botInfo["enabled"] = JsonValue(bot.enabled);
        botList.push_back(JsonValue(botInfo));
    }
    result["result"] = JsonValue(botList);
    return JsonValue(result);
}

JsonValue KlippyServer::handleBotGet(const JsonValue& params) {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    std::map<std::string, JsonValue> result;
    std::string name = params.has("name") && params.find("name")->isString()
        ? params.find("name")->asString() : "";
    auto it = bots_.find(name);
    if (it == bots_.end()) {
        result["error"] = JsonValue("Bot not found");
        return JsonValue(result);
    }
    std::map<std::string, JsonValue> botInfo;
    botInfo["name"] = JsonValue(it->second.name);
    botInfo["type"] = JsonValue(it->second.type);
    botInfo["enabled"] = JsonValue(it->second.enabled);
    result["result"] = JsonValue(botInfo);
    return JsonValue(result);
}

JsonValue KlippyServer::handleBotUpdate(const JsonValue& params) {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    std::map<std::string, JsonValue> result;
    std::string name = params.has("name") && params.find("name")->isString()
        ? params.find("name")->asString() : "";
    auto it = bots_.find(name);
    if (it == bots_.end()) {
        result["error"] = JsonValue("Bot not found");
        return JsonValue(result);
    }
    const JsonValue* enabledVal = params.find("enabled");
    if (enabledVal && enabledVal->isBool()) it->second.enabled = enabledVal->asBool();
    const JsonValue* tokenVal = params.find("token");
    if (tokenVal && tokenVal->isString()) it->second.token = tokenVal->asString();
    const JsonValue* chatIdVal = params.find("chat_id");
    if (chatIdVal && chatIdVal->isString()) it->second.chatId = chatIdVal->asString();
    result["result"] = JsonValue(true);
    return JsonValue(result);
}

JsonValue KlippyServer::handleBotDelete(const JsonValue& params) {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    std::map<std::string, JsonValue> result;
    std::string name = params.has("name") && params.find("name")->isString()
        ? params.find("name")->asString() : "";
    auto it = bots_.find(name);
    if (it == bots_.end()) {
        result["error"] = JsonValue("Bot not found");
        return JsonValue(result);
    }
    bots_.erase(it);
    result["result"] = JsonValue(true);
    return JsonValue(result);
}

// ============================================================================
// E1-Low: Notepad endpoint handlers
// ============================================================================

JsonValue KlippyServer::handleNotepadList(const JsonValue& params) {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    std::map<std::string, JsonValue> result;
    std::vector<JsonValue> notes;
    for (const auto& [key, entry] : notepad_) {
        std::map<std::string, JsonValue> note;
        note["key"] = JsonValue(key);
        note["value"] = JsonValue(entry.value);
        note["last_modified"] = JsonValue(entry.lastModified);
        notes.push_back(JsonValue(note));
    }
    result["result"] = JsonValue(notes);
    return JsonValue(result);
}

JsonValue KlippyServer::handleNotepadGet(const JsonValue& params) {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    std::map<std::string, JsonValue> result;
    std::string key = params.has("key") && params.find("key")->isString()
        ? params.find("key")->asString() : "";
    auto val = notepadGet(key);
    if (val.has_value()) {
        result["result"] = JsonValue(*val);
    } else {
        result["error"] = JsonValue("Notepad entry not found");
    }
    return JsonValue(result);
}

JsonValue KlippyServer::handleNotepadPut(const JsonValue& params) {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    std::map<std::string, JsonValue> result;
    std::string key = params.has("key") && params.find("key")->isString()
        ? params.find("key")->asString() : "";
    std::string value = params.has("value") && params.find("value")->isString()
        ? params.find("value")->asString() : "";
    if (key.empty()) {
        result["error"] = JsonValue("Missing 'key' parameter");
        return JsonValue(result);
    }
    notepadPut(key, value);
    result["result"] = JsonValue(true);
    return JsonValue(result);
}

JsonValue KlippyServer::handleNotepadDelete(const JsonValue& params) {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    std::map<std::string, JsonValue> result;
    std::string key = params.has("key") && params.find("key")->isString()
        ? params.find("key")->asString() : "";
    if (key.empty()) {
        result["error"] = JsonValue("Missing 'key' parameter");
        return JsonValue(result);
    }
    auto it = notepad_.find(key);
    if (it == notepad_.end()) {
        result["error"] = JsonValue("Notepad entry not found");
        return JsonValue(result);
    }
    notepad_.erase(it);
    result["result"] = JsonValue(true);
    return JsonValue(result);
}

// ============================================================================
// E1-Low: Spoolman endpoint handlers
// ============================================================================

JsonValue KlippyServer::handleSpoolmanInfo(const JsonValue& params) {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    std::map<std::string, JsonValue> result;
    std::map<std::string, JsonValue> info;
    info["connected"] = JsonValue(spoolmanConnected_);
    info["url"] = JsonValue(spoolmanUrl_);
    info["spool_id"] = JsonValue(currentSpoolId_);
    for (const auto& [k, v] : spoolmanInfo_) info[k] = v;
    result["result"] = JsonValue(info);
    return JsonValue(result);
}

JsonValue KlippyServer::handleSpoolmanSpoolId(const JsonValue& params) {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    std::map<std::string, JsonValue> result;
    const JsonValue* spoolIdVal = params.find("spool_id");
    if (spoolIdVal && spoolIdVal->isInt()) {
        currentSpoolId_ = spoolIdVal->asInt();
        result["result"] = JsonValue(true);
    } else {
        // Return current spool ID
        result["result"] = JsonValue(currentSpoolId_);
    }
    return JsonValue(result);
}

JsonValue KlippyServer::handleSpoolmanProxy(const JsonValue& params) {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    std::map<std::string, JsonValue> result;

    // If no Spoolman URL is configured, return not_connected
    if (spoolmanUrl_.empty()) {
        std::map<std::string, JsonValue> proxyResult;
        proxyResult["status"] = JsonValue("not_connected");
        proxyResult["message"] = JsonValue("Spoolman URL not configured");
        result["result"] = JsonValue(proxyResult);
        return JsonValue(result);
    }

    // Parse the Spoolman URL to extract host, port, and path
    // Expected format: http://hostname:port or http://hostname:port/api
    std::string url = spoolmanUrl_;
    std::string host, path = "/";
    int port = 80;

    // Remove protocol prefix
    if (url.size() >= 7 && url.compare(0, 7, "http://") == 0) {
        url = url.substr(7);
    } else if (url.size() >= 8 && url.compare(0, 8, "https://") == 0) {
        // HTTPS not supported via raw socket — return error
        std::map<std::string, JsonValue> errResult;
        errResult["error"] = JsonValue("HTTPS Spoolman URLs not supported (use HTTP)");
        errResult["code"] = JsonValue(400);
        result["result"] = JsonValue(errResult);
        return JsonValue(result);
    }

    // Extract host:port and path
    auto slashPos = url.find('/');
    if (slashPos != std::string::npos) {
        host = url.substr(0, slashPos);
        path = url.substr(slashPos);
    } else {
        host = url;
    }

    // Extract port from host
    auto colonPos = host.find(':');
    if (colonPos != std::string::npos) {
        try { port = std::stoi(host.substr(colonPos + 1)); } catch (...) {}
        host = host.substr(0, colonPos);
    }

    // Build the HTTP request path from the proxy params
    // The params should contain a "path" field with the API endpoint
    std::string requestPath = path;
    if (params.isObject()) {
        const auto* pathVal = params.find("path");
        if (pathVal && pathVal->isString()) {
            std::string p = pathVal->asString();
            if (!p.empty()) {
                if (p[0] != '/') requestPath += "/";
                requestPath += p;
            }
        }
    }
    // Ensure path starts with /v1 if not already prefixed
    if (requestPath.find("/v1") != 0 && requestPath.find("/api") != 0) {
        requestPath = "/v1" + requestPath;
    }

    // Create a TCP socket and connect to the Spoolman server
    int sock = ::socket(AF_INET, SOCK_STREAM, 0);
    if (sock < 0) {
        std::map<std::string, JsonValue> errResult;
        errResult["error"] = JsonValue("Failed to create socket: " + std::string(strerror(errno)));
        errResult["code"] = JsonValue(500);
        result["result"] = JsonValue(errResult);
        return JsonValue(result);
    }

    // Set a 5-second timeout
    struct timeval tv;
    tv.tv_sec = 5;
    tv.tv_usec = 0;
    if (setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv)) < 0 ||
        setsockopt(sock, SOL_SOCKET, SO_SNDTIMEO, &tv, sizeof(tv)) < 0) {
        ::close(sock);
        std::map<std::string, JsonValue> errResult;
        errResult["error"] = JsonValue("Failed to set socket timeout");
        errResult["code"] = JsonValue(500);
        result["result"] = JsonValue(errResult);
        return JsonValue(result);
    }

    // Resolve host using thread-safe getaddrinfo()
    struct addrinfo hints{};
    hints.ai_family = AF_INET;
    hints.ai_socktype = SOCK_STREAM;
    struct addrinfo* res = nullptr;
    int gaiErr = getaddrinfo(host.c_str(), std::to_string(port).c_str(), &hints, &res);
    if (gaiErr != 0 || !res) {
        if (sock >= 0) ::close(sock);
        std::map<std::string, JsonValue> errResult;
        errResult["error"] = JsonValue("Failed to resolve host: " + host +
            (gaiErr != 0 ? std::string(" (") + gai_strerror(gaiErr) + ")" : ""));
        errResult["code"] = JsonValue(502);
        result["result"] = JsonValue(errResult);
        return JsonValue(result);
    }

    if (::connect(sock, res->ai_addr, res->ai_addrlen) < 0) {
        freeaddrinfo(res);
        ::close(sock);
        std::map<std::string, JsonValue> errResult;
        errResult["error"] = JsonValue("Failed to connect to Spoolman server: " + std::string(strerror(errno)));
        errResult["code"] = JsonValue(502);
        result["result"] = JsonValue(errResult);
        return JsonValue(result);
    }
    freeaddrinfo(res);

    // Build HTTP GET request
    std::string httpRequest = "GET " + requestPath + " HTTP/1.1\r\n";
    httpRequest += "Host: " + host + "\r\n";
    httpRequest += "Accept: application/json\r\n";
    httpRequest += "Connection: close\r\n";
    httpRequest += "\r\n";

    // Send request
    if (::send(sock, httpRequest.c_str(), httpRequest.size(), 0) < 0) {
        ::close(sock);
        std::map<std::string, JsonValue> errResult;
        errResult["error"] = JsonValue("Failed to send request to Spoolman");
        errResult["code"] = JsonValue(502);
        result["result"] = JsonValue(errResult);
        return JsonValue(result);
    }

    // Read response
    std::string response;
    char buffer[4096];
    ssize_t bytesRead;
    while ((bytesRead = ::recv(sock, buffer, sizeof(buffer) - 1, 0)) > 0) {
        buffer[bytesRead] = '\0';
        response += buffer;
        if (response.size() > 65536) break; // Limit response size
    }
    ::close(sock);

    // Parse HTTP response — extract body (after \r\n\r\n)
    auto headerEnd = response.find("\r\n\r\n");
    if (headerEnd == std::string::npos) {
        std::map<std::string, JsonValue> errResult;
        errResult["error"] = JsonValue("Invalid HTTP response from Spoolman");
        errResult["code"] = JsonValue(502);
        result["result"] = JsonValue(errResult);
        return JsonValue(result);
    }

    // Extract status code
    int statusCode = 0;
    auto statusLineEnd = response.find("\r\n");
    if (statusLineEnd != std::string::npos) {
        std::string statusLine = response.substr(0, statusLineEnd);
        auto sp = statusLine.find(' ');
        if (sp != std::string::npos) {
            auto sp2 = statusLine.find(' ', sp + 1);
            if (sp2 != std::string::npos) {
                try { statusCode = std::stoi(statusLine.substr(sp + 1, sp2 - sp - 1)); } catch (...) {}
            }
        }
    }

    std::string body = response.substr(headerEnd + 4);

    // Check for HTTP error status codes (4xx/5xx)
    if (statusCode >= 400) {
        std::map<std::string, JsonValue> errResult;
        errResult["error"] = JsonValue("Spoolman server returned HTTP " + std::to_string(statusCode));
        errResult["code"] = JsonValue(statusCode);
        errResult["body"] = JsonValue(body);
        result["result"] = JsonValue(errResult);
        return JsonValue(result);
    }

    // Return the response body and status
    std::map<std::string, JsonValue> proxyResult;
    proxyResult["status"] = JsonValue("connected");
    proxyResult["http_status"] = JsonValue(statusCode);
    proxyResult["body"] = JsonValue(body);
    proxyResult["url"] = JsonValue(spoolmanUrl_);
    result["result"] = JsonValue(proxyResult);
    return JsonValue(result);
}

// ============================================================================
// E1-Low: Device CRUD endpoint handlers
// ============================================================================

JsonValue KlippyServer::handleDevicesCreate(const JsonValue& params) {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    std::map<std::string, JsonValue> result;
    std::string device = params.has("device") && params.find("device")->isString()
        ? params.find("device")->asString() : "";
    std::string type = "power";
    const JsonValue* typeVal = params.find("type");
    if (typeVal && typeVal->isString()) type = typeVal->asString();
    if (device.empty()) {
        result["error"] = JsonValue("Missing 'device' parameter");
        return JsonValue(result);
    }
    // Register as a power device by default
    PowerDevice pd;
    pd.name = device;
    pd.state = "off";
    pd.locked = false;
    powerDevices_[device] = pd;
    std::map<std::string, JsonValue> devInfo;
    devInfo["device"] = JsonValue(device);
    devInfo["type"] = JsonValue(type);
    devInfo["state"] = JsonValue("off");
    result["result"] = JsonValue(devInfo);
    return JsonValue(result);
}

JsonValue KlippyServer::handleDevicesDelete(const JsonValue& params) {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    std::map<std::string, JsonValue> result;
    std::string device = params.has("device") && params.find("device")->isString()
        ? params.find("device")->asString() : "";
    if (device.empty()) {
        result["error"] = JsonValue("Missing 'device' parameter");
        return JsonValue(result);
    }
    auto it = powerDevices_.find(device);
    if (it == powerDevices_.end()) {
        result["error"] = JsonValue("Device not found");
        return JsonValue(result);
    }
    powerDevices_.erase(it);
    result["result"] = JsonValue(true);
    return JsonValue(result);
}

// ============================================================================
// E1-Low: Database namespace endpoint handler
// ============================================================================

JsonValue KlippyServer::handleDatabaseNs(const JsonValue& params) {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    std::map<std::string, JsonValue> result;
    // List all namespaces
    std::vector<JsonValue> namespaces;
    for (const auto& [ns, _] : database_) {
        namespaces.push_back(JsonValue(ns));
    }
    result["result"] = JsonValue(namespaces);
    return JsonValue(result);
}

} // namespace tether::klipper::klippy

/**
 * @file KlippyUdsServerE1Helpers.cpp
 * @brief E1 public helper methods (user/bot/notepad/spoolman registration, etc.)
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

JsonValue KlippyUdsServer::callEndpoint(const std::string& method, const JsonValue& params) {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    auto it = endpoints_.find(method);
    if (it == endpoints_.end()) {
        std::map<std::string, JsonValue> err;
        err["error"] = JsonValue("Endpoint not found: " + method);
        err["code"] = JsonValue(404);
        return JsonValue(err);
    }
    return it->second(params);
}

// ============================================================================
// E1: Public helper methods
// ============================================================================

void KlippyUdsServer::registerUser(const std::string& username, const std::string& password,
                                     const std::vector<std::string>& permissions) {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    User user;
    user.username = username;
    user.password = password;
    user.permissions = permissions;
    users_[username] = user;
}

void KlippyUdsServer::registerBot(const std::string& name, const std::string& type,
                                    const std::string& token, bool enabled) {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    Bot bot;
    bot.name = name;
    bot.type = type;
    bot.token = token;
    bot.enabled = enabled;
    bots_[name] = bot;
}

void KlippyUdsServer::notepadPut(const std::string& key, const std::string& value) {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    NotepadEntry entry;
    entry.key = key;
    entry.value = value;
    entry.lastModified = std::time(nullptr);
    notepad_[key] = entry;
}

std::optional<std::string> KlippyUdsServer::notepadGet(const std::string& key) const {
    auto it = notepad_.find(key);
    if (it == notepad_.end()) return std::nullopt;
    return it->second.value;
}

void KlippyUdsServer::setSpoolmanConnected(bool connected, const std::string& url) {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    spoolmanConnected_ = connected;
    spoolmanUrl_ = url;
}

void KlippyUdsServer::setSpoolId(int64_t id) {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    currentSpoolId_ = id;
}

void KlippyUdsServer::addLogFile(const std::string& name, const std::string& path) {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    LogFile lf;
    lf.name = name;
    lf.path = path;
    logFiles_.push_back(lf);
}

void KlippyUdsServer::setSystemPerms(const std::string& resource,
                                       const std::vector<std::string>& perms) {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    systemPerms_[resource] = perms;
}

} // namespace tether::klipper::klippy

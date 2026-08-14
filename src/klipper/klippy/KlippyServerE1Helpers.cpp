/**
 * @file KlippyUdsServerE1Helpers.cpp
 * @brief E1 public helper methods (user/bot/notepad/spoolman registration, etc.)
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
// E1: Public helper methods
// ============================================================================

void KlippyServer::registerUser(const std::string& username, const std::string& password,
                                     const std::vector<std::string>& permissions) {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    User user;
    user.username = username;
    user.password = password;
    user.permissions = permissions;
    users_[username] = user;
}

void KlippyServer::registerBot(const std::string& name, const std::string& type,
                                    const std::string& token, bool enabled) {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    Bot bot;
    bot.name = name;
    bot.type = type;
    bot.token = token;
    bot.enabled = enabled;
    bots_[name] = bot;
}

void KlippyServer::notepadPut(const std::string& key, const std::string& value) {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    NotepadEntry entry;
    entry.key = key;
    entry.value = value;
    entry.lastModified = std::time(nullptr);
    notepad_[key] = entry;
}

std::optional<std::string> KlippyServer::notepadGet(const std::string& key) const {
    auto it = notepad_.find(key);
    if (it == notepad_.end()) return std::nullopt;
    return it->second.value;
}

void KlippyServer::setSpoolmanConnected(bool connected, const std::string& url) {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    spoolmanConnected_ = connected;
    spoolmanUrl_ = url;
}

void KlippyServer::setSpoolId(int64_t id) {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    currentSpoolId_ = id;
}

void KlippyServer::addLogFile(const std::string& name, const std::string& path) {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    LogFile lf;
    lf.name = name;
    lf.path = path;
    logFiles_.push_back(lf);
}

void KlippyServer::setSystemPerms(const std::string& resource,
                                       const std::vector<std::string>& perms) {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    systemPerms_[resource] = perms;
}

} // namespace tether::klipper::klippy

/**
 * @file SlaveLogger.cpp
 * @brief Realtime-safe logging implementation for EtherCAT slave
 */

#include "slave/logging/SlaveLogger.hpp"
#include <cstring>
#include <chrono>

namespace EtherCAT {
namespace slave {

// ============================================================================
// Helper Functions
// ============================================================================

static const char* getCategoryName(SlaveLogCategory cat) {
    switch (cat) {
        case SlaveLogCategory::StateMachine: return "SM";
        case SlaveLogCategory::Register: return "REG";
        case SlaveLogCategory::FMMU: return "FMMU";
        case SlaveLogCategory::SyncManager: return "SYNCM";
        case SlaveLogCategory::DC: return "DC";
        case SlaveLogCategory::Watchdog: return "WDG";
        case SlaveLogCategory::SII: return "SII";
        case SlaveLogCategory::FrameRx: return "FRM_RX";
        case SlaveLogCategory::FrameTx: return "FRM_TX";
        case SlaveLogCategory::Datagram: return "DG";
        case SlaveLogCategory::LogicalAddr: return "LADDR";
        case SlaveLogCategory::Mailbox: return "MBX";
        case SlaveLogCategory::CoE: return "CoE";
        case SlaveLogCategory::FoE: return "FoE";
        case SlaveLogCategory::EoE: return "EoE";
        case SlaveLogCategory::VoE: return "VoE";
        case SlaveLogCategory::SoE: return "SoE";
        case SlaveLogCategory::AoE: return "AoE";
        case SlaveLogCategory::PDO: return "PDO";
        case SlaveLogCategory::TxPDO: return "TxPDO";
        case SlaveLogCategory::RxPDO: return "RxPDO";
        case SlaveLogCategory::CiA401: return "401";
        case SlaveLogCategory::CiA402: return "402";
        default: return "???";
    }
}

static const char* getLevelName(SlaveLogLevel level) {
    switch (level) {
        case SlaveLogLevel::Trace: return "TRC";
        case SlaveLogLevel::Debug: return "DBG";
        case SlaveLogLevel::Info: return "INF";
        case SlaveLogLevel::Warning: return "WRN";
        case SlaveLogLevel::Error: return "ERR";
        case SlaveLogLevel::Critical: return "CRT";
        default: return "???";
    }
}

static const char* getLevelColor(SlaveLogLevel level) {
    switch (level) {
        case SlaveLogLevel::Trace: return "\033[90m";
        case SlaveLogLevel::Debug: return "\033[36m";
        case SlaveLogLevel::Info: return "\033[32m";
        case SlaveLogLevel::Warning: return "\033[33m";
        case SlaveLogLevel::Error: return "\033[31m";
        case SlaveLogLevel::Critical: return "\033[1;31m";
        default: return "";
    }
}

// ============================================================================
// SlaveLogger Implementation
// ============================================================================

SlaveLogger::SlaveLogger(const SlaveLogConfig& config)
    : config_(config)
    , enabledCategories_(config.enabledCategories)
{
    queue_.resize(config_.queueSize);
}

SlaveLogger::~SlaveLogger() {
    stop();
}

void SlaveLogger::start() {
    if (running_.exchange(true)) return;
    
    // Open log file if enabled
    if (config_.fileEnabled && !config_.logFilePath.empty()) {
        logFile_ = std::fopen(config_.logFilePath.c_str(), "a");
    }
    
    // Start processing thread
    processingThread_ = std::make_unique<std::thread>(&SlaveLogger::processingThread, this);
}

void SlaveLogger::stop() {
    if (!running_.exchange(false)) return;
    
    if (processingThread_ && processingThread_->joinable()) {
        processingThread_->join();
    }
    processingThread_.reset();
    
    // Flush remaining entries
    flush();
    
    if (logFile_) {
        std::fclose(logFile_);
        logFile_ = nullptr;
    }
}

void SlaveLogger::flush() {
    size_t head = queueHead_.load();
    size_t tail = queueTail_.load();
    
    while (tail != head) {
        writeEntry(queue_[tail % queue_.size()]);
        tail++;
    }
    queueTail_.store(tail);
}

void SlaveLogger::setEnabledCategories(SlaveLogCategory categories) {
    enabledCategories_.store(categories);
}

void SlaveLogger::setCategoryEnabled(SlaveLogCategory category, bool enabled) {
    auto current = enabledCategories_.load();
    if (enabled) {
        enabledCategories_.store(current | category);
    } else {
        enabledCategories_.store(current & ~category);
    }
}

bool SlaveLogger::isCategoryEnabled(SlaveLogCategory category) const {
    auto enabled = enabledCategories_.load();
    return (static_cast<uint32_t>(enabled) & static_cast<uint32_t>(category)) != 0;
}

void SlaveLogger::setMinLevel(SlaveLogLevel level) {
    std::lock_guard<std::mutex> lock(mutex_);
    config_.minLevel = level;
}

void SlaveLogger::log(SlaveLogCategory category, SlaveLogLevel level, const char* format, ...) {
    va_list args;
    va_start(args, format);
    logv(category, level, format, args);
    va_end(args);
}

void SlaveLogger::logv(SlaveLogCategory category, SlaveLogLevel level, const char* format, va_list args) {
    if (level < config_.minLevel) return;
    if (!isCategoryEnabled(category)) return;
    
    SlaveLogEntry entry;
    entry.timestamp = getTimestamp();
    entry.category = category;
    entry.level = level;
    entry.slaveAddress = slaveAddress_;
    entry.dataLen = 0;
    
    std::vsnprintf(entry.message, sizeof(entry.message), format, args);
    
    // Try to enqueue
    size_t head = queueHead_.load(std::memory_order_relaxed);
    size_t tail = queueTail_.load(std::memory_order_acquire);
    
    if (head - tail >= queue_.size()) {
        dropCount_.fetch_add(1);
        return;
    }
    
    queue_[head % queue_.size()] = entry;
    queueHead_.store(head + 1, std::memory_order_release);
    logCount_.fetch_add(1);
}

void SlaveLogger::logFrame(SlaveLogCategory category, const uint8_t* frame, size_t length, const char* description) {
    SlaveLogEntry entry;
    entry.timestamp = getTimestamp();
    entry.category = category;
    entry.level = SlaveLogLevel::Debug;
    entry.slaveAddress = slaveAddress_;
    
    if (description) {
        std::snprintf(entry.message, sizeof(entry.message), "%s (%zu bytes)", description, length);
    } else {
        std::snprintf(entry.message, sizeof(entry.message), "Frame (%zu bytes)", length);
    }
    
    entry.dataLen = std::min(length, sizeof(entry.data));
    if (frame && entry.dataLen > 0) {
        std::memcpy(entry.data, frame, entry.dataLen);
    }
    
    size_t head = queueHead_.load(std::memory_order_relaxed);
    size_t tail = queueTail_.load(std::memory_order_acquire);
    
    if (head - tail >= queue_.size()) {
        dropCount_.fetch_add(1);
        return;
    }
    
    queue_[head % queue_.size()] = entry;
    queueHead_.store(head + 1, std::memory_order_release);
    logCount_.fetch_add(1);
}

void SlaveLogger::logHex(SlaveLogCategory category, SlaveLogLevel level,
                         const uint8_t* data, size_t length, const char* prefix) {
    if (level < config_.minLevel) return;
    if (!isCategoryEnabled(category)) return;
    
    char hexBuf[256];
    size_t pos = 0;
    if (prefix) {
        pos = std::snprintf(hexBuf, sizeof(hexBuf), "%s: ", prefix);
    }
    
    for (size_t i = 0; i < length && pos < sizeof(hexBuf) - 4; i++) {
        pos += std::snprintf(hexBuf + pos, sizeof(hexBuf) - pos, "%02X ", data[i]);
    }
    
    log(category, level, "%s", hexBuf);
}

void SlaveLogger::trace(SlaveLogCategory cat, const char* fmt, ...) {
    va_list args; va_start(args, fmt); logv(cat, SlaveLogLevel::Trace, fmt, args); va_end(args);
}

void SlaveLogger::debug(SlaveLogCategory cat, const char* fmt, ...) {
    va_list args; va_start(args, fmt); logv(cat, SlaveLogLevel::Debug, fmt, args); va_end(args);
}

void SlaveLogger::info(SlaveLogCategory cat, const char* fmt, ...) {
    va_list args; va_start(args, fmt); logv(cat, SlaveLogLevel::Info, fmt, args); va_end(args);
}

void SlaveLogger::warn(SlaveLogCategory cat, const char* fmt, ...) {
    va_list args; va_start(args, fmt); logv(cat, SlaveLogLevel::Warning, fmt, args); va_end(args);
}

void SlaveLogger::error(SlaveLogCategory cat, const char* fmt, ...) {
    va_list args; va_start(args, fmt); logv(cat, SlaveLogLevel::Error, fmt, args); va_end(args);
}

void SlaveLogger::critical(SlaveLogCategory cat, const char* fmt, ...) {
    va_list args; va_start(args, fmt); logv(cat, SlaveLogLevel::Critical, fmt, args); va_end(args);
}

void SlaveLogger::logFrameToPcap(const uint8_t* frame, size_t length,
                                  HAL::FrameDirection direction, uint64_t timestamp) {
    if (pcapLogger_) {
        pcapLogger_->logFrame(frame, length, direction, timestamp ? timestamp : getTimestamp());
    }
}

void SlaveLogger::resetStats() {
    logCount_.store(0);
    dropCount_.store(0);
}

void SlaveLogger::processingThread() {
    while (running_.load()) {
        size_t head = queueHead_.load(std::memory_order_acquire);
        size_t tail = queueTail_.load(std::memory_order_relaxed);
        
        if (tail < head) {
            writeEntry(queue_[tail % queue_.size()]);
            queueTail_.store(tail + 1, std::memory_order_release);
        } else {
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
    }
}

void SlaveLogger::writeEntry(const SlaveLogEntry& entry) {
    if (config_.consoleEnabled) writeToConsole(entry);
    if (config_.fileEnabled && logFile_) writeToFile(entry);
    if (config_.customCallback) writeToCallback(entry);
}

void SlaveLogger::writeToConsole(const SlaveLogEntry& entry) {
    const char* color = config_.consoleColors ? getLevelColor(entry.level) : "";
    const char* reset = config_.consoleColors ? "\033[0m" : "";
    
    std::fprintf(stderr, "[%8lu.%03lu] %s[%-3s]%s [%-6s] %s\n",
                 static_cast<unsigned long>(entry.timestamp / 1000000000ULL),
                 static_cast<unsigned long>((entry.timestamp / 1000000ULL) % 1000),
                 color, getLevelName(entry.level), reset,
                 getCategoryName(entry.category), entry.message);
    
    if (entry.dataLen > 0) {
        std::fprintf(stderr, "  ");
        for (size_t i = 0; i < entry.dataLen; i++) {
            std::fprintf(stderr, "%02X ", entry.data[i]);
        }
        std::fprintf(stderr, "\n");
    }
}

void SlaveLogger::writeToFile(const SlaveLogEntry& entry) {
    if (!logFile_) return;
    
    std::fprintf(logFile_, "[%lu.%03lu] [%s] [%s] %s\n",
                 static_cast<unsigned long>(entry.timestamp / 1000000000ULL),
                 static_cast<unsigned long>((entry.timestamp / 1000000ULL) % 1000),
                 getLevelName(entry.level),
                 getCategoryName(entry.category),
                 entry.message);
    
    currentFileSize_ += std::strlen(entry.message) + 50;
    
    if (currentFileSize_ > config_.maxFileSize && config_.maxRotatedFiles > 0) {
        std::fclose(logFile_);
        logFile_ = nullptr;
        
        // Simple rotation
        for (int i = config_.maxRotatedFiles - 1; i >= 0; i--) {
            std::string oldName = config_.logFilePath + (i > 0 ? "." + std::to_string(i) : "");
            std::string newName = config_.logFilePath + "." + std::to_string(i + 1);
            std::rename(oldName.c_str(), newName.c_str());
        }
        
        logFile_ = std::fopen(config_.logFilePath.c_str(), "a");
        currentFileSize_ = 0;
    }
}

void SlaveLogger::writeToCallback(const SlaveLogEntry& entry) {
    if (config_.customCallback) config_.customCallback(entry);
}

uint64_t SlaveLogger::getTimestamp() const {
    auto now = std::chrono::steady_clock::now();
    return static_cast<uint64_t>(
        std::chrono::duration_cast<std::chrono::nanoseconds>(now.time_since_epoch()).count());
}

}  // namespace slave
}  // namespace EtherCAT

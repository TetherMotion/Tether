/**
 * @file DataDictionary.cpp
 * @brief DataDictionary implementation using Glaze for JSON (de)serialisation.
 */

#include "tether/klipper/protocol/DataDictionary.hpp"
#include "tether/klipper/KlipperLog.hpp"

#include <format>

#ifdef TETHER_KLIPPER_HAS_ZLIB
#include <zlib.h>
#endif

namespace tether::klipper::protocol {

// ============================================================================
// Message registration
// ============================================================================

uint16_t DataDictionary::addMessage(std::string_view formatStr, MessageDirection dir) {
    std::string key(formatStr);
    if (dir == MessageDirection::Command && commandIndex_.count(key)) {
        KLIPPER_LOG_WARN("Duplicate command format string ignored: " + key);
        return 0;
    }
    if (dir == MessageDirection::Response && responseIndex_.count(key)) {
        KLIPPER_LOG_WARN("Duplicate response format string ignored: " + key);
        return 0;
    }
    if (dir == MessageDirection::Output && outputIndex_.count(key)) {
        KLIPPER_LOG_WARN("Duplicate output format string ignored: " + key);
        return 0;
    }
    auto spec = parseFormatString(formatStr);
    if (!spec) {
        KLIPPER_LOG_ERROR("Failed to parse format string: " + key);
        return 0;
    }
    if (nextMsgid_ > kMaxMsgId) {
        KLIPPER_LOG_ERROR(std::format("DataDictionary msgid exhausted (max={})", kMaxMsgId));
        return 0;
    }
    uint16_t id = nextMsgid_++;
    MessageEntry e;
    e.msgid = id;
    e.direction = dir;
    e.format = std::move(*spec);
    messages_[id] = std::move(e);
    if (dir == MessageDirection::Command) commandIndex_[key] = id;
    else if (dir == MessageDirection::Response) responseIndex_[key] = id;
    else outputIndex_[key] = id;
    return id;
}

uint16_t DataDictionary::addCommand(std::string_view f)   { return addMessage(f, MessageDirection::Command); }
uint16_t DataDictionary::addResponse(std::string_view f)  { return addMessage(f, MessageDirection::Response); }
uint16_t DataDictionary::addOutput(std::string_view f)    { return addMessage(f, MessageDirection::Output); }

// ============================================================================
// Lookups
// ============================================================================

const MessageEntry* DataDictionary::lookupMsgid(uint16_t msgid) const {
    auto it = messages_.find(msgid);
    return it == messages_.end() ? nullptr : &it->second;
}
std::optional<uint16_t> DataDictionary::lookupCommand(std::string_view f) const {
    auto it = commandIndex_.find(std::string(f));
    if (it == commandIndex_.end()) return std::nullopt;
    return it->second;
}
std::optional<uint16_t> DataDictionary::lookupResponse(std::string_view f) const {
    auto it = responseIndex_.find(std::string(f));
    if (it == responseIndex_.end()) return std::nullopt;
    return it->second;
}
std::optional<uint16_t> DataDictionary::lookupOutput(std::string_view f) const {
    auto it = outputIndex_.find(std::string(f));
    if (it == outputIndex_.end()) return std::nullopt;
    return it->second;
}

// ============================================================================
// Enumerations
// ============================================================================

void DataDictionary::addEnumValue(std::string_view enumName, std::string_view key, uint32_t value) {
    auto& idx = enumIndex_[std::string(enumName)];
    bool found = false;
    for (size_t i = 0; i < enumerations_.size(); ++i) {
        if (enumerations_[i].name == enumName) { idx = i + 1; found = true; break; }
    }
    if (!found) {
        enumerations_.push_back(Enumeration{std::string(enumName), {}});
        idx = enumerations_.size();
    }
    auto& e = enumerations_[idx - 1];
    e.entries.push_back(EnumEntry{std::string(key), value, 1});
}

void DataDictionary::addEnumRange(std::string_view enumName, std::string_view key,
                                  uint32_t start, uint32_t count) {
    auto& idx = enumIndex_[std::string(enumName)];
    bool found = false;
    for (size_t i = 0; i < enumerations_.size(); ++i) {
        if (enumerations_[i].name == enumName) { idx = i + 1; found = true; break; }
    }
    if (!found) {
        enumerations_.push_back(Enumeration{std::string(enumName), {}});
        idx = enumerations_.size();
    }
    auto& e = enumerations_[idx - 1];
    e.entries.push_back(EnumEntry{std::string(key), start, count});
}

const Enumeration* DataDictionary::lookupEnum(std::string_view enumName) const {
    for (const auto& e : enumerations_) {
        if (e.name == enumName) return &e;
    }
    return nullptr;
}

std::optional<uint32_t> DataDictionary::resolveEnum(std::string_view enumName, std::string_view key) const {
    const Enumeration* e = lookupEnum(enumName);
    if (!e) return std::nullopt;
    // Exact key match.
    for (const auto& en : e->entries) {
        if (en.count == 1 && en.key == key) return en.start;
    }
    // Range expansion: split trailing digits from the declared key.
    for (const auto& en : e->entries) {
        if (en.count <= 1) continue;
        size_t digitStart = en.key.size();
        while (digitStart > 0 && std::isdigit(static_cast<unsigned char>(en.key[digitStart - 1]))) --digitStart;
        std::string root = en.key.substr(0, digitStart);
        uint32_t baseIdx = 0;
        if (digitStart < en.key.size()) {
            auto [ptr, ec] = std::from_chars(en.key.data() + digitStart,
                                             en.key.data() + en.key.size(), baseIdx);
            if (ec != std::errc{}) continue;
        }
        if (key.size() <= root.size()) continue;
        if (key.substr(0, root.size()) != root) continue;
        size_t kDigitStart = root.size();
        bool allDigits = true;
        for (size_t i = kDigitStart; i < key.size(); ++i) {
            if (!std::isdigit(static_cast<unsigned char>(key[i]))) { allDigits = false; break; }
        }
        if (!allDigits) continue;
        uint32_t kIdx = 0;
        auto [p2, ec2] = std::from_chars(key.data() + kDigitStart, key.data() + key.size(), kIdx);
        if (ec2 != std::errc{}) continue;
        if (kIdx >= baseIdx && kIdx < baseIdx + en.count) {
            return en.start + (kIdx - baseIdx);
        }
    }
    return std::nullopt;
}

// ============================================================================
// Constants
// ============================================================================

void DataDictionary::addConstant(std::string_view name, int64_t value) {
    constants_[std::string(name)] = value;
}
void DataDictionary::addConstantString(std::string_view name, std::string_view value) {
    constants_[std::string(name)] = std::string(value);
}
std::optional<ConstantValue> DataDictionary::lookupConstant(std::string_view name) const {
    auto it = constants_.find(std::string(name));
    if (it == constants_.end()) return std::nullopt;
    return it->second;
}

// ============================================================================
// Wire (de)compression
// ============================================================================

std::string DataDictionary::fromWire(std::span<const uint8_t> wire) {
    if (wire.empty()) return {};
#ifdef TETHER_KLIPPER_HAS_ZLIB
    // Heuristic: zlib streams start with 0x78 (most common deflate headers).
    if (wire[0] == 0x78 || wire[0] == 0x68 || wire[0] == 0x08 || wire[0] == 0x28 ||
        wire[0] == 0x38 || wire[0] == 0x48 || wire[0] == 0x58) {
        // Iteratively try larger buffers for decompression
        for (size_t attempt = 0; attempt < 5; ++attempt) {
            std::string out;
            uLongf destLen = static_cast<uLongf>(wire.size() * (8 << attempt) + 1024);
            out.resize(destLen);
            int ret = uncompress(reinterpret_cast<Bytef*>(out.data()), &destLen,
                                 reinterpret_cast<const Bytef*>(wire.data()), static_cast<uLong>(wire.size()));
            if (ret == Z_OK) {
                out.resize(destLen);
                return out;
            }
            if (ret != Z_BUF_ERROR) break; // Error other than buffer too small
        }
    }
#endif
    // Stored format: skip marker byte, return raw JSON.
    if (wire[0] == 0x00) {
        return std::string(reinterpret_cast<const char*>(wire.data() + 1), wire.size() - 1);
    }
    // Best effort: treat as raw JSON.
    return std::string(reinterpret_cast<const char*>(wire.data()), wire.size());
}

} // namespace tether::klipper::protocol

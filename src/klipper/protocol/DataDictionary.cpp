/**
 * @file DataDictionary.cpp
 * @brief DataDictionary implementation using Glaze for JSON (de)serialisation.
 */

#include "tether/klipper/protocol/DataDictionary.hpp"
#include "tether/klipper/KlipperLog.hpp"

#include <glaze/glaze.hpp>

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
        KLIPPER_LOG_ERROR("DataDictionary msgid exhausted (max=" + std::to_string(kMaxMsgId) + ")");
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
// JSON serialisation via Glaze (glz::generic dynamic JSON)
// ============================================================================

std::string DataDictionary::toJson() const {
    glz::generic doc;
    doc["app"] = app_;
    doc["version"] = version_;
    doc["build_versions"] = buildVersions_;
    doc["license"] = license_;

    // operator[] auto-converts null to object_t on first key access.
    for (const auto& [id, e] : messages_) {
        std::string fmt = e.format.toString();
        if (e.direction == MessageDirection::Command) doc["commands"][fmt] = static_cast<double>(id);
        else if (e.direction == MessageDirection::Response) doc["responses"][fmt] = static_cast<double>(id);
        else doc["output"][fmt] = static_cast<double>(id);
    }
    // Ensure empty maps exist even when there are no entries.
    if (!doc.contains("commands")) doc["commands"] = glz::generic::object_t{};
    if (!doc.contains("responses")) doc["responses"] = glz::generic::object_t{};
    if (!doc.contains("output")) doc["output"] = glz::generic::object_t{};

    for (const auto& [name, val] : constants_) {
        if (std::holds_alternative<int64_t>(val)) {
            doc["config"][name] = static_cast<double>(std::get<int64_t>(val));
        } else {
            doc["config"][name] = std::get<std::string>(val);
        }
    }
    if (!doc.contains("config")) doc["config"] = glz::generic::object_t{};

    for (const auto& e : enumerations_) {
        glz::generic enObj;
        for (const auto& en : e.entries) {
            if (en.count <= 1) {
                enObj[en.key] = static_cast<double>(en.start);
            } else {
                enObj[en.key] = std::vector<double>{static_cast<double>(en.start),
                                                   static_cast<double>(en.count)};
            }
        }
        doc["enumerations"][e.name] = std::move(enObj);
    }
    if (!doc.contains("enumerations")) doc["enumerations"] = glz::generic::object_t{};

    auto ws = glz::write_json(doc);
    return ws.value_or(std::string{});
}

bool DataDictionary::fromJson(std::string_view json) {
    auto rd = glz::read_json<glz::generic>(json);
    if (!rd) {
        KLIPPER_LOG_ERROR("DataDictionary::fromJson() failed to parse JSON");
        return false;
    }
    glz::generic& doc = *rd;
    // Reset state.
    *this = DataDictionary{};

    auto getString = [&](const char* key) -> std::string {
        if (!doc.contains(key)) return {};
        if (auto* s = doc[key].get_if<std::string>()) return *s;
        return {};
    };

    app_ = getString("app");
    version_ = getString("version");
    buildVersions_ = getString("build_versions");
    license_ = getString("license");

    auto loadMsgMap = [&](const char* key, MessageDirection dir) {
        if (!doc.contains(key)) return;
        auto* obj = doc[key].get_if<glz::generic::object_t>();
        if (!obj) return;
        for (auto& [fmt, val] : *obj) {
            int64_t id = 0;
            if (auto* d = val.get_if<double>()) id = static_cast<int64_t>(*d);
            
            MessageEntry e;
            e.msgid = static_cast<uint16_t>(id);
            e.direction = dir;
            auto spec = parseFormatString(fmt);
            if (spec) e.format = std::move(*spec);
            messages_[e.msgid] = std::move(e);
            if (dir == MessageDirection::Command) commandIndex_[fmt] = static_cast<uint16_t>(id);
            else if (dir == MessageDirection::Response) responseIndex_[fmt] = static_cast<uint16_t>(id);
            else outputIndex_[fmt] = static_cast<uint16_t>(id);
        }
    };
    loadMsgMap("commands", MessageDirection::Command);
    loadMsgMap("responses", MessageDirection::Response);
    loadMsgMap("output", MessageDirection::Output);

    if (doc.contains("config")) {
        if (auto* obj = doc["config"].get_if<glz::generic::object_t>()) {
            for (auto& [name, val] : *obj) {
                if (auto* d = val.get_if<double>()) {
                    constants_[name] = static_cast<int64_t>(*d);
                } else if (auto* s = val.get_if<std::string>()) {
                    constants_[name] = *s;
                }
            }
        }
    }

    if (doc.contains("enumerations")) {
        if (auto* obj = doc["enumerations"].get_if<glz::generic::object_t>()) {
            for (auto& [enName, enVal] : *obj) {
                Enumeration en;
                en.name = enName;
                if (auto* enObj = enVal.get_if<glz::generic::object_t>()) {
                    for (auto& [ekey, eval] : *enObj) {
                        if (auto* arr = eval.get_if<glz::generic::array_t>()) {
                            if (arr->size() >= 2) {
                                uint32_t start = 0, count = 0;
                                if (auto* d = (*arr)[0].get_if<double>()) start = static_cast<uint32_t>(*d);
                                if (auto* d = (*arr)[1].get_if<double>()) count = static_cast<uint32_t>(*d);
                                en.entries.push_back(EnumEntry{ekey, start, count});
                            }
                        } else {
                            uint32_t v = 0;
                            if (auto* d = eval.get_if<double>()) v = static_cast<uint32_t>(*d);
                            en.entries.push_back(EnumEntry{ekey, v, 1});
                        }
                    }
                }
                enumerations_.push_back(std::move(en));
            }
        }
    }

    nextMsgid_ = kFirstDynamicMsgId;
    for (const auto& [id, e] : messages_) {
        if (id >= nextMsgid_) nextMsgid_ = static_cast<uint16_t>(id + 1);
    }
    return true;
}

// ============================================================================
// Wire (de)compression
// ============================================================================

std::vector<uint8_t> DataDictionary::toWire() const {
    std::string json = toJson();
#ifdef TETHER_KLIPPER_HAS_ZLIB
    std::vector<uint8_t> out;
    out.resize(json.size() + 64);
    uLongf destLen = out.size();
    if (compress2(reinterpret_cast<Bytef*>(out.data()), &destLen,
                  reinterpret_cast<const Bytef*>(json.data()), json.size(), 9) == Z_OK) {
        out.resize(destLen);
        return out;
    }
#endif
    // Stored format: marker byte 0x00 followed by raw JSON bytes.
    std::vector<uint8_t> out;
    out.reserve(json.size() + 1);
    out.push_back(0x00);
    out.insert(out.end(), json.begin(), json.end());
    return out;
}

std::string DataDictionary::fromWire(std::span<const uint8_t> wire) {
    if (wire.empty()) return {};
#ifdef TETHER_KLIPPER_HAS_ZLIB
    // Heuristic: zlib streams start with 0x78 (most common deflate headers).
    if (wire[0] == 0x78 || wire[0] == 0x68 || wire[0] == 0x08 || wire[0] == 0x28 ||
        wire[0] == 0x38 || wire[0] == 0x48 || wire[0] == 0x58) {
        std::string out;
        out.resize(wire.size() * 8 + 1024);
        uLongf destLen = out.size();
        if (uncompress(reinterpret_cast<Bytef*>(out.data()), &destLen,
                       reinterpret_cast<const Bytef*>(wire.data()), wire.size()) == Z_OK) {
            out.resize(destLen);
            return out;
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

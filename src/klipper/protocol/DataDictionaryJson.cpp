/**
 * @file DataDictionaryJson.cpp
 * @brief Glaze-based JSON serialisation for the Klipper DataDictionary.
 */

#include "tether/klipper/protocol/DataDictionary.hpp"
#include "tether/klipper/KlipperLog.hpp"

#include <glaze/glaze.hpp>
#include <format>

#ifdef TETHER_KLIPPER_HAS_ZLIB
#include <zlib.h>
#endif

namespace tether::klipper::protocol {

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
// Wire compression
// ============================================================================

std::vector<uint8_t> DataDictionary::toWire() const {
    std::string json = toJson();
#ifdef TETHER_KLIPPER_HAS_ZLIB
    // Allocate buffer large enough for worst case (input + input/1000 + 12 per zlib docs)
    std::vector<uint8_t> out;
    uLongf destLen = compressBound(static_cast<uLong>(json.size()));
    out.resize(destLen);
    if (compress2(reinterpret_cast<Bytef*>(out.data()), &destLen,
                  reinterpret_cast<const Bytef*>(json.data()), static_cast<uLong>(json.size()), 9) == Z_OK) {
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

} // namespace tether::klipper::protocol

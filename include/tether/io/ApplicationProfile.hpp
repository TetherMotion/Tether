#pragma once

// Application profile integrity + in-memory recipe store for the optional
// machine.app.profile / machine.recipe.* surfaces.
//
// Profile signing uses keyed BLAKE3 (a real MAC, not a plain hash): the
// server holds the 32-byte key, `signProfileDocument` produces the MAC that
// is stored in the AppProfileV1 signal, and `StaticAppProfileSource` refuses
// to install a profile whose MAC does not verify. The document itself is
// opaque to the framework — the dashboard interprets it as the declarative
// widget/panel contract documented in docs/WebInterfaceContract.md.

#include "tether/io/MachineService.hpp"

#include <array>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <map>
#include <string>
#include <string_view>
#include <unordered_set>
#include <vector>

#include "blake3.h"

#if defined(TETHER_IO_HAS_GLAZE)
#include <glaze/glaze.hpp>
#endif

namespace tether::io::machine {

using ProfileKey = std::array<uint8_t, 32>;
using ProfileMac = std::array<uint8_t, 32>;

/// Keyed BLAKE3 MAC of a profile document.
inline ProfileMac signProfileDocument(const void* document, size_t size,
                                      const ProfileKey& key) {
    blake3_hasher hasher;
    blake3_hasher_init_keyed(&hasher, key.data());
    blake3_hasher_update(&hasher, document, size);
    ProfileMac mac{};
    blake3_hasher_finalize(&hasher, mac.data(), mac.size());
    return mac;
}

/// Constant-time MAC comparison.
inline bool verifyProfileDocument(const void* document, size_t size,
                                  const ProfileKey& key, const ProfileMac& mac) {
    const ProfileMac expected = signProfileDocument(document, size, key);
    uint8_t diff = 0;
    for (size_t i = 0; i < mac.size(); ++i) diff |= mac[i] ^ expected[i];
    return diff == 0;
}

/// Application profile document schema (server-side startup validation,
/// plan item 86). Mirrors the widget contract the dashboard parses in
/// `domain/app-profile.ts`: format marker, panels of widgets, and per-kind
/// required fields. Validation is compile-time gated on Glaze; when Glaze is
/// unavailable `install` fails closed and the machine.app.profile surface is
/// simply absent.
inline constexpr std::string_view kAppProfileFormat = "tether.app.profile.v1";
inline constexpr size_t kMaxProfileDocumentBytes = 256 * 1024;
inline constexpr size_t kMaxProfilePanels = 64;
inline constexpr size_t kMaxProfileWidgets = 64;
inline constexpr size_t kMaxProfileString = 128;

#if defined(TETHER_IO_HAS_GLAZE)
namespace detail {
inline bool profileString(const glz::generic& value, std::string_view& out,
                          size_t maxLength) {
    const auto* str = value.get_if<std::string>();
    if (!str || str->empty() || str->size() > maxLength) return false;
    out = *str;
    return true;
}

inline bool profileWidgetValid(const glz::generic& widget) {
    const auto* object = widget.get_if<glz::generic::object_t>();
    if (!object) return false;
    const auto find = [&](std::string_view key) -> const glz::generic* {
        const auto it = object->find(std::string(key));
        return it == object->end() ? nullptr : &it->second;
    };
    std::string_view label, kind;
    if (!find("kind") || !profileString(*find("kind"), kind, 32)) return false;
    if (!find("label") || !profileString(*find("label"), label, kMaxProfileString))
        return false;
    static const std::unordered_set<std::string_view> reading = {
        "value", "bar", "gauge", "state", "lamp", "dro", "sparkline"};
    const auto hasString = [&](std::string_view key) {
        const auto* value = find(key);
        std::string_view ignored;
        return value && profileString(*value, ignored, kMaxProfileString);
    };
    if (reading.contains(kind)) return hasString("entry");
    if (kind == "jog") return hasString("axis");
    if (kind == "command") {
        const auto* action = find("action");
        return hasString("axis") && action && action->get_if<double>() &&
               *action->get_if<double>() >= 0 &&
               *action->get_if<double>() == std::floor(*action->get_if<double>());
    }
    if (kind == "button") return hasString("fn");
    return false;
}
} // namespace detail

/// Validate the declarative profile document against the widget contract.
/// Returns false for malformed JSON, a wrong format marker, unbounded
/// documents, or widgets missing their kind-required fields.
inline bool validateAppProfileDocument(const void* document, size_t size) {
    if (!document || size == 0 || size > kMaxProfileDocumentBytes) return false;
    auto parsed = glz::read_json<glz::generic>(
        std::string_view(static_cast<const char*>(document), size));
    if (!parsed) return false;
    const auto* root = parsed->get_if<glz::generic::object_t>();
    if (!root) return false;
    const auto format = root->find("format");
    std::string_view formatString;
    if (format == root->end() ||
        !detail::profileString(format->second, formatString, kMaxProfileString) ||
        formatString != kAppProfileFormat)
        return false;
    const auto panels = root->find("panels");
    if (panels == root->end()) return true;  // panel-less profile is valid
    const auto* panelList = panels->second.get_if<glz::generic::array_t>();
    if (!panelList || panelList->size() > kMaxProfilePanels) return false;
    std::unordered_set<std::string_view> ids;
    for (const auto& panel : *panelList) {
        const auto* object = panel.get_if<glz::generic::object_t>();
        if (!object) return false;
        const auto id = object->find("id");
        const auto title = object->find("title");
        std::string_view idString, titleString;
        if (id == object->end() || title == object->end() ||
            !detail::profileString(id->second, idString, kMaxProfileString) ||
            !detail::profileString(title->second, titleString, kMaxProfileString))
            return false;
        if (!ids.insert(idString).second) return false;
        const auto widgets = object->find("widgets");
        if (widgets == object->end()) continue;
        const auto* widgetList = widgets->second.get_if<glz::generic::array_t>();
        if (!widgetList || widgetList->size() > kMaxProfileWidgets) return false;
        for (const auto& widget : *widgetList)
            if (!detail::profileWidgetValid(widget)) return false;
    }
    return true;
}
#else
/// Without Glaze the document cannot be validated; fail closed so an
/// unverifiable profile never reaches the wire.
inline bool validateAppProfileDocument(const void*, size_t) { return false; }
#endif

/// In-memory application profile source. `install` verifies the MAC against
/// the key and validates the document against the profile contract; a
/// tampered, mismatched, or malformed document never reaches the wire.
class StaticAppProfileSource final : public IMachineAppProfileSource {
public:
    explicit StaticAppProfileSource(ProfileKey key) : key_(key) {}

    /// Verify + validate + install the profile document. Returns false when
    /// `mac` does not match `document` under this source's key or the
    /// document violates the profile contract.
    bool install(std::string name, std::string version,
                 std::vector<uint8_t> document, const ProfileMac& mac) {
        if (!verifyProfileDocument(document.data(), document.size(), key_, mac))
            return false;
        if (!validateAppProfileDocument(document.data(), document.size()))
            return false;
        profile_ = MachineAppProfile{std::move(name), std::move(version),
                                     std::move(document), mac};
        installed_ = true;
        return true;
    }

    bool installed() const { return installed_; }
    const MachineAppProfile& profile() override { return profile_; }

private:
    ProfileKey key_;
    MachineAppProfile profile_;
    bool installed_ = false;
};

/// In-memory recipe store: `add` registers a named set of ConfigWrites that
/// machine.recipe.apply stages through the normal configuration transaction.
class InMemoryRecipeStore final : public IMachineRecipeStore {
public:
    void add(std::string name, std::string description,
             std::vector<ConfigWrite> entries) {
        recipes_[name] = Recipe{std::move(description), std::move(entries)};
    }

    std::vector<RecipeInfo> list() override {
        std::vector<RecipeInfo> out;
        out.reserve(recipes_.size());
        for (const auto& [name, recipe] : recipes_)
            out.push_back({name, recipe.description,
                           static_cast<uint32_t>(recipe.entries.size())});
        return out;
    }

    bool recipe(std::string_view name,
                std::vector<ConfigWrite>& writes) override {
        const auto it = recipes_.find(std::string(name));
        if (it == recipes_.end()) return false;
        writes = it->second.entries;
        return true;
    }

private:
    struct Recipe {
        std::string description;
        std::vector<ConfigWrite> entries;
    };
    std::map<std::string, Recipe> recipes_;
};

} // namespace tether::io::machine

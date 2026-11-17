#pragma once

/**
 * @file Spinner.hpp
 * @brief Toolkit-agnostic "work in flight" glyphs + compact duration
 *        formatting — the recurring little helpers every monitor TUI
 *        ends up re-implementing.
 */

#include <cstdint>
#include <cstdio>
#include <string>
#include <string_view>

namespace Tether {
namespace UI {

/// Braille spinner — cycle `tick` (e.g. a ~50 ms frame counter).
inline std::string_view spinnerFrame(uint64_t tick) {
    static constexpr std::string_view kFrames[] = {
        "⠋", "⠙", "⠹", "⠸", "⠼", "⠴", "⠦", "⠧", "⠇", "⠏",
    };
    return kFrames[tick % 10];
}

/// ASCII fallback for terminals without UTF-8.
inline char spinnerFrameAscii(uint64_t tick) {
    static constexpr char kFrames[] = {'-', '\\', '|', '/'};
    return kFrames[tick % 4];
}

/// `3s` / `2m05s` / `1h04m` — compact elapsed-time rendering.
inline std::string formatDuration(uint64_t secs) {
    if (secs < 60) return std::to_string(secs) + "s";
    if (secs < 3600)
        return std::to_string(secs / 60) + "m" +
               (secs % 60 < 10 ? "0" : "") + std::to_string(secs % 60) + "s";
    return std::to_string(secs / 3600) + "h" +
           ((secs % 3600) / 60 < 10 ? "0" : "") +
           std::to_string((secs % 3600) / 60) + "m";
}

/// Compact count: `812` / `9.7k` / `1.2M`.
inline std::string formatCount(uint64_t v) {
    if (v < 1000) return std::to_string(v);
    if (v < 1'000'000) {
        char b[16];
        snprintf(b, sizeof(b), "%.1fk", v / 1000.0);
        return b;
    }
    char b[16];
    snprintf(b, sizeof(b), "%.1fM", v / 1'000'000.0);
    return b;
}

}  // namespace UI
}  // namespace Tether

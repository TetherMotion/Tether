#pragma once

#include <sstream>
#include <string>
#include <vector>

namespace Tether {
namespace TUI {

/// Word-wrap `text` to `width` columns: split on newlines first, then wrap
/// each line at the last space that still fits (hard-break when no space).
/// Empty input lines are preserved as empty output lines.
inline std::vector<std::string> wrapText(const std::string& text, int width) {
    std::vector<std::string> lines;
    if (width <= 0) return lines;
    std::istringstream iss(text);
    std::string line;
    while (std::getline(iss, line)) {
        size_t pos = 0;
        while (pos < line.size()) {
            size_t len = std::min<size_t>(width, line.size() - pos);
            if (pos + len < line.size()) {
                const size_t brk = line.rfind(' ', pos + len);
                if (brk != std::string::npos && brk > pos) len = brk - pos;
            }
            lines.push_back(line.substr(pos, len));
            pos += len;
            while (pos < line.size() && line[pos] == ' ') ++pos;
        }
        if (line.empty()) lines.emplace_back();
    }
    return lines;
}

/// Pad or truncate a string to a fixed display width (byte count).
inline std::string padToWidth(const std::string& s, size_t w) {
    if (s.size() >= w) return s.substr(0, w);
    return s + std::string(w - s.size(), ' ');
}

} // namespace TUI
} // namespace Tether

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

} // namespace TUI
} // namespace Tether

/**
 * @file AnsiLine.cpp
 * @brief UTF-8 measuring + ANSI SGR-to-curses line rendering.
 */

// ncurses defines OK/ERR as macros; undefine them before Tether headers
#include <ncurses.h>
#undef OK
#undef ERR

#include "tether/terminal_ui/AnsiLine.hpp"

#include <cstdlib>
#include <cwchar>
#include <vector>

#include "tether/terminal_ui/Session.hpp"

namespace Tether {
namespace TUI {

std::string utf8Trunc(const std::string& s, int max_cols) {
    if (max_cols <= 0) return {};
    std::mbstate_t st{};
    int    cols = 0;
    size_t i    = 0;
    while (i < s.size() && cols < max_cols) {
        wchar_t      wc = 0;
        const size_t n  = std::mbrtowc(&wc, s.data() + i, s.size() - i, &st);
        if (n == (size_t)-1 || n == (size_t)-2) { ++i; ++cols; continue; }
        if (n == 0) { ++i; continue; }
        i += n;
        ++cols;
    }
    return s.substr(0, i);
}

int utf8Cols(const std::string& s) {
    std::mbstate_t st{};
    int    cols = 0;
    size_t i    = 0;
    while (i < s.size()) {
        wchar_t      wc = 0;
        const size_t n  = std::mbrtowc(&wc, s.data() + i, s.size() - i, &st);
        if (n == (size_t)-1 || n == (size_t)-2) { ++i; ++cols; continue; }
        if (n == 0) { ++i; continue; }
        i += n;
        ++cols;
    }
    return cols;
}

short ansiFgPair(int c) {
    switch (c) {
        case 0: return PalMuted;      // black — invisible on black, mute it
        case 1: return PalError;      // red
        case 2: return PalValue;      // green
        case 3: return PalHint;       // yellow
        case 4: return PalInfo;       // blue
        case 5: return PalSelected;   // magenta
        case 6: return PalHeader;     // cyan
        default: return PalMuted;     // white / out of range
    }
}

// Apply one SGR parameter list (text between "ESC[" and "m") to the
// current render state.  `base_pair` acts as the default foreground
// (SGR 0 / 39).
static void applySgr(const std::string& params, short& pair, bool& bold,
                     short base_pair) {
    std::vector<int> ps;
    size_t pos = 0;
    while (pos <= params.size()) {
        const size_t sep = params.find(';', pos);
        const std::string tok = params.substr(
            pos, sep == std::string::npos ? std::string::npos : sep - pos);
        ps.push_back(tok.empty() ? 0 : std::atoi(tok.c_str()));
        if (sep == std::string::npos) break;
        pos = sep + 1;
    }
    for (size_t k = 0; k < ps.size(); ++k) {
        const int c = ps[k];
        if (c == 0) { pair = base_pair; bold = false; }
        else if (c == 1)  bold = true;
        else if (c == 22) bold = false;
        else if (c >= 30 && c <= 37) pair = ansiFgPair(c - 30);
        else if (c == 39) pair = base_pair;
        else if (c >= 90 && c <= 97) { pair = ansiFgPair(c - 90); bold = true; }
        else if ((c == 38 || c == 48) && k + 1 < ps.size()) {
            // Extended color — approximate 256-color fg to the basic
            // palette; truecolor and background are dropped.
            if (ps[k + 1] == 5 && k + 2 < ps.size()) {
                if (c == 38) {
                    pair = ansiFgPair(ps[k + 2] & 7);
                    if (ps[k + 2] >= 8) bold = true;
                }
                k += 2;
            } else if (ps[k + 1] == 2 && k + 4 < ps.size()) {
                k += 4;
            }
        }
    }
}

void renderAnsiLine(WINDOW* win, int row, int col, int width,
                    const std::string& s, short base_pair) {
    short pair = base_pair;
    bool  bold = false;
    const int x0 = col;
    size_t i = 0;
    while (i < s.size() && col - x0 < width) {
        const auto ch = static_cast<unsigned char>(s[i]);
        if (ch == 0x1B && i + 1 < s.size() && s[i + 1] == '[') {
            // CSI sequence: parameter bytes until the final byte (@-~).
            size_t j = i + 2;
            while (j < s.size()) {
                const auto fc = static_cast<unsigned char>(s[j]);
                if (fc >= 0x40 && fc <= 0x7E) break;
                ++j;
            }
            if (j >= s.size()) break;   // unterminated — drop the tail
            if (s[j] == 'm')
                applySgr(s.substr(i + 2, j - i - 2), pair, bold, base_pair);
            i = j + 1;
            continue;
        }
        // Plain run up to the next escape or end of line.
        size_t j = s.find('\x1B', i);
        if (j == std::string::npos) j = s.size();
        const std::string out =
            utf8Trunc(s.substr(i, j - i), width - (col - x0));
        const attr_t attr =
            (pair != PalNone ? COLOR_PAIR(pair) : 0) |
            (bold ? A_BOLD : 0);
        if (attr) wattron(win, attr);
        mvwprintw(win, row, col, "%s", out.c_str());
        if (attr) wattroff(win, attr);
        col += utf8Cols(out);
        i = j;
    }
}

} // namespace TUI
} // namespace Tether

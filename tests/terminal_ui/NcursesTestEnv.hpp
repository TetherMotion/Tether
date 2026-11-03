/**
 * @file NcursesTestEnv.hpp
 * @brief Shared headless-ncurses fixture for terminal_ui tests.
 *
 * newterm() accepts arbitrary FILE* pairs, so a real SCREEN runs on
 * /dev/null with no tty. Widget draws land in WINDOW buffers and are
 * verified via mvwinnstr()/mvwinch()/mvwin_wch().
 *
 * Include AFTER <ncurses.h> and the widget headers (include order per
 * AGENTS.md: standard → UI models → ncurses → widgets → this header).
 */

#pragma once

#include <gtest/gtest.h>

#include <clocale>
#include <cstdio>
#include <cstdlib>
#include <string>
#include <vector>

namespace NcursesTest {

using Tether::TUI::PalValue; using Tether::TUI::PalHeader; using Tether::TUI::PalHint;
using Tether::TUI::PalError; using Tether::TUI::PalSelected; using Tether::TUI::PalMuted;
using Tether::TUI::PalInfo; using Tether::TUI::PalNone;

struct Ctx {
    FILE* out = nullptr;
    FILE* in  = nullptr;
    SCREEN* scr = nullptr;
    bool ok = false;

    static Ctx& get() {
        static Ctx ctx;
        if (!ctx.scr) ctx.init();
        return ctx;
    }
    void init() {
        setenv("TERM", "xterm-256color", 1);
        setlocale(LC_ALL, "C.UTF-8");
        in  = fopen("/dev/null", "r");
        out = fopen("/dev/null", "w");
        scr = newterm(nullptr, out, in);
        if (!scr) return;
        set_term(scr);
        cbreak();
        noecho();
        nonl();
        curs_set(0);
        if (has_colors()) {
            start_color();
            use_default_colors();
            init_pair(PalValue,    COLOR_GREEN,   -1);
            init_pair(PalHeader,   COLOR_CYAN,    -1);
            init_pair(PalHint,     COLOR_YELLOW,  -1);
            init_pair(PalError,    COLOR_RED,     -1);
            init_pair(PalSelected, COLOR_MAGENTA, -1);
            init_pair(PalMuted,    COLOR_WHITE,   -1);
            init_pair(PalInfo,     COLOR_BLUE,    -1);
        }
        ok = true;
    }
};

/// RAII test window.
struct W {
    WINDOW* w;
    explicit W(int h = 12, int wd = 60) : w(newwin(h, wd, 0, 0)) {}
    ~W() { delwin(w); }
    operator WINDOW*() const { return w; }
};

/// Text of row y, cols [x0, x1) — trailing blanks trimmed.
inline std::string rowText(WINDOW* w, int y, int x0 = 0, int x1 = 60) {
    std::vector<char> buf(x1 - x0 + 1, 0);
    mvwinnstr(w, y, x0, buf.data(), x1 - x0);
    std::string s(buf.data());
    const auto last = s.find_last_not_of(' ');
    return last == std::string::npos ? "" : s.substr(0, last + 1);
}

inline chtype cell(WINDOW* w, int y, int x) { return mvwinch(w, y, x); }
inline char   cellCh(WINDOW* w, int y, int x) { return static_cast<char>(mvwinch(w, y, x) & A_CHARTEXT); }
inline int    cellPair(WINDOW* w, int y, int x) { return PAIR_NUMBER(mvwinch(w, y, x)); }
inline bool   cellBold(WINDOW* w, int y, int x) { return (mvwinch(w, y, x) & A_BOLD) != 0; }

/// Pair+attrs for cells that may hold wide chars — mvwinch() returns
/// garbage for them in ncursesw; mvwin_wch + getcchar is exact.
inline void cellAttrs(WINDOW* w, int y, int x, short& pair, attr_t& attrs) {
    cchar_t cc{};
    mvwin_wch(w, y, x, &cc);
    wchar_t wc[CCHARW_MAX] = {};
    short pr = 0;
    getcchar(&cc, wc, &attrs, &pr, nullptr);
    pair = pr;
}

/// Fixture — requires Ctx::get().ok in SetUp.
class NcursesWidgetTest : public ::testing::Test {
protected:
    void SetUp() override {
        ASSERT_TRUE(Ctx::get().ok) << "newterm() failed — terminfo missing?";
    }
};

} // namespace NcursesTest

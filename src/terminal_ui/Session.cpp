/**
 * @file Session.cpp
 * @brief ncurses lifecycle + availability check
 */

#include "tether/terminal_ui/Session.hpp"

#include <cstdlib>
#include <cstring>
#include <clocale>
#include <langinfo.h>

#include <ncurses.h>
#include <unistd.h>

namespace Tether {
namespace TUI {

Session::Session() {
    // Force a UTF-8 locale so wide/Unicode box-drawing and arrows render
    // correctly even when the environment is not set (e.g. LANG=C).
    static const char* const locales[] = {
        "C.UTF-8", "C.utf8", "en_US.UTF-8", "en_US.utf8", "",
    };
    for (const char* l : locales) {
        if (setlocale(LC_ALL, l) && strcasestr(nl_langinfo(CODESET), "UTF-8"))
            break;
    }
    initscr();
    noecho();
    cbreak();
    curs_set(0);
    nodelay(stdscr, TRUE);
    keypad(stdscr, TRUE);

    colors_ = has_colors();
    if (colors_) {
        start_color();
        use_default_colors();
        init_pair(PalValue,    COLOR_GREEN,   -1);
        init_pair(PalHeader,   COLOR_CYAN,    -1);
        init_pair(PalHint,     COLOR_YELLOW,  -1);
        init_pair(PalError,    COLOR_RED,     -1);
        init_pair(PalSelected, COLOR_MAGENTA, -1);
    }
}

Session::~Session() {
    endwin();
}

bool Session::available() {
    if (!isatty(STDOUT_FILENO)) return false;
    const char* term = getenv("TERM");
    if (!term || !*term || strcmp(term, "dumb") == 0) return false;
    return true;
}

int Session::rows() const { return LINES; }
int Session::cols() const { return COLS; }

int Session::pollKey(int timeout_ms) {
    if (timeout_ms < 0)      nodelay(stdscr, FALSE);
    else if (timeout_ms == 0) nodelay(stdscr, TRUE);
    else                     wtimeout(stdscr, timeout_ms);
    return getch();
}

TermWindow* Session::makeWindow(int y, int x, int h, int w) const {
    return newwin(h, w, y, x);
}

void Session::freeWindow(TermWindow* win) const {
    if (win) delwin(static_cast<WINDOW*>(win));
}

} // namespace TUI
} // namespace Tether

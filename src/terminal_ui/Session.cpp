/**
 * @file Session.cpp
 * @brief ncurses lifecycle + availability check
 */

#include "tether/terminal_ui/Session.hpp"

#include <cstdlib>
#include <cstring>
#include <string>
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
        init_pair(PalMuted,    COLOR_WHITE,   -1);
        init_pair(PalInfo,     COLOR_BLUE,    -1);
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

/**
 * Decode the bytes pending after a raw ESC.  Returns the curses key
 * code for common CSI (`ESC [ ...`) and SS3 (`ESC O X`) sequences —
 * arrows, Home/End, PgUp/PgDn, Insert/Delete, Shift-Tab, F1-F12 — or
 * 27 for a genuine lone ESC / ESC-ESC, or -1 for an unrecognized or
 * swallowed sequence.
 */
static int decodeEscapeTail(Session& session) {
    const int c1 = session.pollKey(30);
    if (c1 < 0) return 27;                    // lone ESC
    if (c1 == 27) return 27;                  // ESC ESC → quit
    if (c1 != '[' && c1 != 'O') return -1;

    if (c1 == 'O') {
        // SS3: single final byte (application keypad / xterm F-keys).
        const int f = session.pollKey(5);
        switch (f) {
            case 'A': return KEY_UP;
            case 'B': return KEY_DOWN;
            case 'C': return KEY_RIGHT;
            case 'D': return KEY_LEFT;
            case 'H': return KEY_HOME;
            case 'F': return KEY_END;
            case 'P': return KEY_F(1);
            case 'Q': return KEY_F(2);
            case 'R': return KEY_F(3);
            case 'S': return KEY_F(4);
            default:  return -1;
        }
    }

    // CSI: parameter/intermediate bytes until a final byte (@-~).
    std::string params;
    for (int i = 0; i < 16; ++i) {
        const int c = session.pollKey(5);
        if (c < 0) return -1;
        if (c >= 0x40 && c <= 0x7E) {
            if (params.empty()) {
                switch (c) {                  // CSI <final>
                    case 'A': return KEY_UP;
                    case 'B': return KEY_DOWN;
                    case 'C': return KEY_RIGHT;
                    case 'D': return KEY_LEFT;
                    case 'H': return KEY_HOME;
                    case 'F': return KEY_END;
                    case 'Z': return KEY_BTAB;
                    default:  return -1;
                }
            }
            if (c == '~') {                   // CSI <n>~
                switch (std::atoi(params.c_str())) {
                    case 1: case 7:  return KEY_HOME;
                    case 4: case 8:  return KEY_END;
                    case 2:          return KEY_IC;
                    case 3:          return KEY_DC;
                    case 5:          return KEY_PPAGE;
                    case 6:          return KEY_NPAGE;
                    case 11:         return KEY_F(1);
                    case 12:         return KEY_F(2);
                    case 13:         return KEY_F(3);
                    case 14:         return KEY_F(4);
                    case 15:         return KEY_F(5);
                    case 17:         return KEY_F(6);
                    case 18:         return KEY_F(7);
                    case 19:         return KEY_F(8);
                    case 20:         return KEY_F(9);
                    case 21:         return KEY_F(10);
                    case 23:         return KEY_F(11);
                    case 24:         return KEY_F(12);
                    default:         return -1;
                }
            }
            return -1;
        }
        params += static_cast<char>(c);
    }
    return -1;
}

int Session::pollKeyDecoded(int timeout_ms) {
    const int key = pollKey(timeout_ms);
    return (key == 27) ? decodeEscapeTail(*this) : key;
}

TermWindow* Session::makeWindow(int y, int x, int h, int w) const {
    return newwin(h, w, y, x);
}

void Session::freeWindow(TermWindow* win) const {
    if (win) delwin(static_cast<WINDOW*>(win));
}

} // namespace TUI
} // namespace Tether

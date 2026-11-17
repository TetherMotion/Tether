/**
 * @file AgentMonitorScreen.cpp
 * @brief PanelScreen wiring for UI::AgentMonitor — see the header.
 */

#include "tether/terminal_ui/AgentMonitorScreen.hpp"

#include <ncurses.h>

#include "tether/terminal_ui/PanelScreen.hpp"
#include "tether/terminal_ui/StyledText.hpp"

namespace Tether {
namespace TUI {

AgentMonitorScreen::AgentMonitorScreen(std::string title,
                                       Utils::IFramePump& child,
                                       UI::AgentMonitor& mon)
    : title_(std::move(title)), child_(&child), mon_(&mon) {}

void AgentMonitorScreen::run(std::atomic<bool>& cancel) {
    auto& child = *child_;
    auto& mon   = *mon_;

    PanelScreen screen(title_, PanelScreenHooks{
        .onTick = [&] {
            ++mon.tick;
            for (auto& f : child.poll())
                mon.onFrame(f.kind, f.payload);
            std::string err = child.drainErr();
            if (!err.empty()) mon.onStderr(err);
            if (!child.alive()) mon.onBackendExit();
            // A finished ASK menu emits its ANSWER frame here.
            if (mon.awaiting && mon.ask_menu) mon.pollAskMenu();
        },
        .renderBody = [&](int top, int bottom) {
            mon.setWidth(std::max(20, COLS - 4));
            // Body: [top, bottom-2-menu_h) log, then the option menu
            // block while one is open, bottom-2 status, bottom-1 input.
            int menu_h = mon.menuRows();
            int h = bottom - top - 2 - menu_h;
            if (h > 0) {
                int row = top;
                for (const auto* lp : mon.log.visible(h)) {
                    move(row, 0); clrtoeol();
                    short pair = logLevelPair(lp->level);
                    if (pair != PalNone) attron(COLOR_PAIR(pair));
                    mvprintw(row, 1, "%s", lp->text.c_str());
                    if (pair != PalNone) attroff(COLOR_PAIR(pair));
                    ++row;
                }
            }

            // Option menu block — live › highlight, sits above the
            // status line while an ASK question is open.
            if (mon.awaiting && mon.ask_menu) {
                auto& menu = *mon.ask_menu;
                for (int i = 0; i < menu.itemCount(); ++i) {
                    int my = bottom - 2 - menu_h + i;
                    if (my < top) continue;
                    move(my, 0); clrtoeol();
                    std::string lbl = menu.rowLabel(i);
                    if (int(lbl.size()) > COLS - 6)
                        lbl = lbl.substr(0, size_t(COLS - 7)) + "…";
                    if (i == menu.cursor()) {
                        attron(A_REVERSE | A_BOLD);
                        mvprintw(my, 1, " › %s", lbl.c_str());
                        attroff(A_REVERSE | A_BOLD);
                    } else if (menu.isCustomRow(i)) {
                        attron(COLOR_PAIR(PalMuted));
                        mvprintw(my, 3, "%s", lbl.c_str());
                        attroff(COLOR_PAIR(PalMuted));
                    } else {
                        mvprintw(my, 3, "%s", lbl.c_str());
                    }
                }
            }

            // Status line.
            int sy = bottom - 2;
            if (sy >= top) {
                move(sy, 0); clrtoeol();
                attron(A_REVERSE);
                std::string s = mon.statusLine();
                mvprintw(sy, 0, "%s", s.c_str());
                for (int x = int(s.size()); x < COLS; ++x) mvaddch(sy, x, ' ');
                attroff(A_REVERSE);
            }

            // Input row.
            int y = bottom - 1;
            if (y >= top) {
                move(y, 0); clrtoeol();
                if (mon.awaiting) {
                    bool editing = mon.ask_menu && mon.ask_menu->editingCustom();
                    if (mon.ask_menu && !editing) {
                        attron(COLOR_PAIR(PalHint));
                        mvprintw(y, 1,
                            "(↑/↓ move · Enter select · Esc skip · digits jump)");
                        attroff(COLOR_PAIR(PalHint));
                    } else {
                        attron(A_BOLD);
                        mvprintw(y, 0, "answer>");
                        attroff(A_BOLD);
                        const std::string& buf =
                            editing ? mon.ask_menu->customInput().buffer()
                                    : mon.input;
                        std::string view = buf;
                        int maxw = COLS - 10;
                        if (int(view.size()) > maxw)
                            view = "…" + view.substr(view.size() - size_t(maxw - 1));
                        mvprintw(y, 9, "%s", view.c_str());
                        move(y, 9 + int(view.size()));
                    }
                } else {
                    attron(COLOR_PAIR(PalHint));
                    mvprintw(y, 1, "PgUp/PgDn: scroll   q: quit");
                    attroff(COLOR_PAIR(PalHint));
                }
            }
        },
        .showLogView = false,
        .onKey = [&](int ch) -> bool {
            // An open option menu is modal — every key goes to it.
            if (mon.awaiting && mon.ask_menu) {
                mon.ask_menu->handleKey(ch);
                mon.pollAskMenu();   // emits ANSWER when the menu finished
                return true;
            }
            switch (ch) {
                case '\n': case '\r': case KEY_ENTER:
                    mon.submitInput();
                    return true;
                case 27:  // Esc — skip a free-text question, else quit
                    if (mon.awaiting) { mon.skipInput(); return true; }
                    return false;
                case KEY_BACKSPACE: case 127: case 8:
                    mon.inputBackspace();
                    return true;
                case KEY_PPAGE: mon.log.scrollBy(10); return true;
                case KEY_NPAGE: mon.log.scrollBy(-10); return true;
                case KEY_END:   mon.log.scrollEnd(); return true;
                default:
                    if (mon.awaiting && ch >= 32 && ch < 127) {
                        mon.inputChar(char(ch));
                        return true;
                    }
                    return false;
            }
        },
        // While a question is open the menu/input owns every key —
        // including digits, which the screen would treat as view switching.
        .wantsKey = [&](int) -> bool { return mon.awaiting; },
        .keyHints = "PgUp/PgDn: scroll   q: quit",
    });

    screen.run(cancel);
}

} // namespace TUI
} // namespace Tether

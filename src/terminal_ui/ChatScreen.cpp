/**
 * @file ChatScreen.cpp
 * @brief PanelScreen wiring for UI::ChatSession — see the header.
 */

#include "tether/terminal_ui/ChatScreen.hpp"

#include <ncurses.h>

#include "tether/terminal_ui/PanelScreen.hpp"
#include "tether/terminal_ui/StyledText.hpp"

namespace Tether {
namespace TUI {

ChatScreen::ChatScreen(std::string title,
                       Utils::FramedChild& child,
                       UI::ChatSession& ses)
    : title_(std::move(title)), child_(&child), ses_(&ses) {}

void ChatScreen::run(std::atomic<bool>& cancel) {
    auto& child = *child_;
    auto& ses   = *ses_;

    PanelScreen screen(title_, PanelScreenHooks{
        .onTick = [&] {
            for (auto& f : child.poll())
                ses.onFrame(f.kind, f.payload);
            if (!child.alive()) ses.onBackendExit();
        },
        .renderBody = [&](int top, int bottom) {
            ses.setWidth(std::max(20, COLS - 4));
            // Transcript: rows [top, bottom-1); input on bottom-1.
            int h = bottom - top - 1;
            if (h < 1) return;
            int row = top;
            for (const auto* lp : ses.transcript.visible(h)) {
                move(row, 0); clrtoeol();
                short pair = logLevelPair(lp->level);
                if (pair != PalNone) attron(COLOR_PAIR(pair));
                mvprintw(row, 1, "%s", lp->text.c_str());
                if (pair != PalNone) attroff(COLOR_PAIR(pair));
                ++row;
            }
            // Input row.
            int y = bottom - 1;
            move(y, 0); clrtoeol();
            attron(A_BOLD);
            mvprintw(y, 0, ses.busy ? "…" : ">");
            attroff(A_BOLD);
            std::string view = ses.input;
            int maxw = COLS - 3;
            if (int(view.size()) > maxw)
                view = "…" + view.substr(view.size() - size_t(maxw - 1));
            mvprintw(y, 2, "%s", view.c_str());
            if (!ses.ready)
                mvprintw(y, 2, "(connecting to backend…)");
            move(y, 2 + int(view.size()));
        },
        .showLogView = false,
        .onKey = [&](int ch) -> bool {
            switch (ch) {
                case '\n': case '\r': case KEY_ENTER:
                    ses.submitInput();
                    return true;
                case KEY_BACKSPACE: case 127: case 8:
                    ses.inputBackspace();
                    return true;
                case KEY_PPAGE: ses.transcript.scrollBy(10); return true;
                case KEY_NPAGE: ses.transcript.scrollBy(-10); return true;
                case KEY_END:   ses.transcript.scrollEnd(); return true;
                case 4: // Ctrl-D — quit when the prompt is empty
                    if (ses.input.empty()) return false;
                    return true;
                default:
                    if (ch >= 32 && ch < 127) {
                        ses.inputChar(char(ch));
                        return true;
                    }
                    return false;
            }
        },
        .keyHints = "type + Enter: send   PgUp/PgDn: scroll   q: quit",
    });

    screen.run(cancel);
}

} // namespace TUI
} // namespace Tether

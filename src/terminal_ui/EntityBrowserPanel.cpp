/**
 * @file EntityBrowserPanel.cpp
 * @brief Standard entities+registers view widget.
 */

#include <ncurses.h>

#include <cstdio>

#include "tether/terminal_ui/EntityBrowserPanel.hpp"
#include "tether/terminal_ui/TablePanel.hpp"

namespace Tether {
namespace TUI {

EntityBrowserPanel::EntityBrowserPanel(UI::EntityBrowser& browser)
    : b_(browser) {
    b_.entities().setColumns(
        {"idx", "name", "state", "code", "err", "susp", "detail"});
    b_.registers().setColumns({"obj", "name", "value", "status"});
}

UI::TableModel& EntityBrowserPanel::focusedTable() {
    return b_.focus() == UI::EntityBrowser::Focus::Entities
               ? b_.entities()
               : b_.registers();
}

void EntityBrowserPanel::render(int top, int bottom) {
    // Throttled provider poll.
    const auto now = std::chrono::steady_clock::now();
    if (first_render_ || now - last_refresh_ >= refresh_ms_) {
        first_render_ = false;
        last_refresh_ = now;
        b_.refresh();
        if (b_.autoRead() && b_.registersStale())
            b_.refreshRegisters();
    }

    // Prompt line reserves the last body row.
    const int table_bottom = prompt_ == Prompt::None ? bottom : bottom - 1;

    // Split: entities get ~40% (min 6 rows), registers the rest.
    const int total = table_bottom - top;
    int ent_h = total * 2 / 5;
    ent_h = std::clamp(ent_h, 6, std::max(6, total - 5));

    static const std::vector<int> kEntWidths = {5, 24, 10, 8, 5, 6, 0};
    static const std::vector<int> kRegWidths = {12, 30, 24, 0};
    const int width = COLS;

    TablePanel::render(b_.entities(), top, top + ent_h, 0, width,
                       b_.focus() == UI::EntityBrowser::Focus::Entities,
                       kEntWidths);
    TablePanel::render(b_.registers(), top + ent_h, table_bottom, 0, width,
                       b_.focus() == UI::EntityBrowser::Focus::Registers,
                       kRegWidths);

    // Context line under the registers table header: which entity the
    // register list belongs to + read status.
    attron(COLOR_PAIR(PalMuted));
    if (!b_.registerStatus().empty())
        mvprintw(top + ent_h, width - std::min<int>(width, 40),
                 "%s", b_.registerStatus().c_str());
    else if (b_.registersUnread())
        mvprintw(top + ent_h, width - std::min<int>(width, 40),
                 "press Enter/r to read registers");
    attroff(COLOR_PAIR(PalMuted));

    // Overlays draw on top of everything.
    overlay_host_.render(top, bottom);

    // Modal prompt on the last row.
    if (prompt_ != Prompt::None) {
        attron(COLOR_PAIR(PalHint));
        mvprintw(bottom - 1, 0, "%s: %s_",
                 prompt_ == Prompt::Search ? "search" : "filter",
                 prompt_buf_.c_str());
        if (prompt_ == Prompt::Filter) {
            mvprintw(bottom - 1, 20 + static_cast<int>(prompt_buf_.size()),
                     "(field=v !field=v field>v text=v — Enter applies, Esc cancels)");
        }
        attroff(COLOR_PAIR(PalHint));
    }
}

void EntityBrowserPanel::commitPrompt() {
    if (prompt_ == Prompt::Search)
        focusedTable().setSearch(prompt_buf_);
    else if (prompt_ == Prompt::Filter)
        focusedTable().setFilterExpr(prompt_buf_);
    prompt_ = Prompt::None;
    prompt_buf_.clear();
}

std::string EntityBrowserPanel::keyHints() const {
    if (!overlays_.empty()) return overlay_host_.keyHints();
    std::string h = "e:focus Enter/r:read /:search f:filter x:clear a:auto";
    for (const auto& a : actions_) {
        h += ' ';
        h += a.label;
    }
    return h;
}

std::unique_ptr<UI::Overlay> makeEntityInfoOverlay(UI::EntityBrowser& b) {
    const UI::SlaveEntity* e = b.selectedEntity();
    if (!e) {
        return std::make_unique<UI::TextOverlay>(
            "Entity info", std::vector<std::string>{"no entity selected"});
    }
    std::vector<std::string> lines = {
        "index:     " + std::to_string(e->index),
        "name:      " + e->name,
        "state:     " + e->state,
        "status:    0x" + ([&]{ char b_[8]; snprintf(b_, 8, "%04X", e->status_code); return std::string{b_}; })(),
        std::string("error:     ") + (e->error ? "yes" : "no"),
        std::string("suspended: ") + (e->suspended ? "yes" : "no"),
        "",
        e->detail,
    };
    if (!b.registerStatus().empty()) {
        lines.push_back("");
        lines.push_back("registers: " + b.registerStatus());
    }
    return std::make_unique<UI::TextOverlay>(
        "Entity " + std::to_string(e->index), std::move(lines));
}

bool EntityBrowserPanel::handleKey(int key) {
    // Overlays consume all keys until closed.
    if (!overlays_.empty()) return overlay_host_.handleKey(key);

    // Prompt mode consumes everything except Enter/Esc.
    if (prompt_ != Prompt::None) {
        if (key == '\n' || key == KEY_ENTER) { commitPrompt(); return true; }
        if (key == 27) { prompt_ = Prompt::None; prompt_buf_.clear(); return true; }
        if (key == KEY_BACKSPACE || key == 127 || key == '\b') {
            if (!prompt_buf_.empty()) prompt_buf_.pop_back();
            return true;
        }
        if (key >= 32 && key < 127) {
            prompt_buf_.push_back(static_cast<char>(key));
            // Live type-ahead for quicksearch.
            if (prompt_ == Prompt::Search)
                focusedTable().setSearch(prompt_buf_);
            return true;
        }
        return true;   // swallow everything else while prompting
    }

    // App-registered actions get first pick.
    for (const auto& a : actions_) {
        if (a.key == key) {
            if (a.invoke) a.invoke();
            return true;
        }
    }

    const int page = 8;
    switch (key) {
        case KEY_UP:    focusedTable().move(-1); return true;
        case KEY_DOWN:  focusedTable().move(+1); return true;
        case KEY_PPAGE: focusedTable().pageMove(-1, page); return true;
        case KEY_NPAGE: focusedTable().pageMove(+1, page); return true;
        case KEY_HOME:  focusedTable().selectFirst(); return true;
        case KEY_END:   focusedTable().selectLast(); return true;

        case '\n':
        case KEY_ENTER:
            b_.refreshRegisters();
            b_.setFocus(UI::EntityBrowser::Focus::Registers);
            return true;
        case 'r':
        case 'R':
            b_.refreshRegisters();
            return true;

        case 'e':
            b_.setFocus(b_.focus() == UI::EntityBrowser::Focus::Entities
                            ? UI::EntityBrowser::Focus::Registers
                            : UI::EntityBrowser::Focus::Entities);
            return true;

        case KEY_LEFT:
            if (b_.focus() == UI::EntityBrowser::Focus::Registers) {
                b_.setFocus(UI::EntityBrowser::Focus::Entities);
                return true;
            }
            return false;

        case '/':
            prompt_ = Prompt::Search;
            prompt_buf_ = focusedTable().search();
            return true;
        case 'f':
            prompt_ = Prompt::Filter;
            prompt_buf_ = focusedTable().filterExpr();
            return true;
        case 'x':
            focusedTable().clearFilter();
            focusedTable().setSearch("");
            return true;
        case 'a':
            b_.setAutoRead(!b_.autoRead());
            return true;
        default:
            return false;
    }
}

} // namespace TUI
} // namespace Tether

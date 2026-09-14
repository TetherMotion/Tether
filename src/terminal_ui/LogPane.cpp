/**
 * @file LogPane.cpp
 * @brief Captured-log ring buffer implementation
 */

#include "tether/terminal_ui/LogPane.hpp"

#include <algorithm>
#include <cwchar>

#include "logging/Logger.hpp"

#include <ncurses.h>

namespace Tether {
namespace TUI {

// Truncate a UTF-8 string to at most `maxCols` display columns without
// splitting a multibyte codepoint.
static std::string utf8Trunc(const std::string& s, int maxCols) {
    if (maxCols <= 0) return {};
    std::mbstate_t st{};
    int    cols = 0;
    size_t i    = 0;
    while (i < s.size() && cols < maxCols) {
        wchar_t      wc = 0;
        const size_t n  = std::mbrtowc(&wc, s.data() + i, s.size() - i, &st);
        if (n == (size_t)-1 || n == (size_t)-2) { ++i; ++cols; continue; }
        if (n == 0) { ++i; continue; }
        i += n;
        ++cols;
    }
    return s.substr(0, i);
}

LogPane::LogPane(size_t maxLines) : maxLines_(maxLines) {}

LogPane::~LogPane() { release(); }

void LogPane::attach() {
    if (attached_) return;
    attached_ = true;
    Platform::Logger::instance().setHandler(
        [this](Platform::LogLevel level, const char* tag, const char* msg) {
            static const char* lv[] = {"", "E", "W", "I", "D", "V"};
            const char* l = lv[std::min<int>(static_cast<int>(level), 5)];
            addLine(std::string(l) + " " + tag + ": " + msg);
        });
}

void LogPane::release() {
    if (!attached_) return;
    attached_ = false;
    Platform::Logger::instance().setHandler(nullptr);
}

void LogPane::addLine(std::string line) {
    std::lock_guard<std::mutex> lock(mutex_);
    lines_.emplace_back(std::move(line));
    while (lines_.size() > maxLines_) lines_.pop_front();
}

void LogPane::render(TermWindow* w, short colorPair) {
    WINDOW* win = static_cast<WINDOW*>(w);
    werase(win);

    int h = 0, width = 0;
    getmaxyx(win, h, width);
    if (h <= 0 || width <= 0) return;

    std::deque<std::string> copy;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        copy = lines_;
    }

    // Newest at the bottom, at most `h` lines.
    const size_t start = copy.size() > static_cast<size_t>(h)
                       ? copy.size() - static_cast<size_t>(h) : 0;
    int row = 0;
    if (colorPair != PalNone) wattron(win, COLOR_PAIR(colorPair));
    for (size_t i = start; i < copy.size() && row < h; ++i, ++row) {
        mvwprintw(win, row, 0, "%s", utf8Trunc(copy[i], width - 1).c_str());
    }
    if (colorPair != PalNone) wattroff(win, COLOR_PAIR(colorPair));
}

} // namespace TUI
} // namespace Tether

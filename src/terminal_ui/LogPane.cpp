/**
 * @file LogPane.cpp
 * @brief Scrollable captured-log ring buffer implementation
 */

#include "tether/terminal_ui/LogPane.hpp"

#include <algorithm>
#include <cwchar>

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

// Map a log level to its display palette pair.
static short levelColor(Platform::LogLevel level, short infoColor) {
    switch (level) {
        case Platform::LogLevel::Error:   return PalError;
        case Platform::LogLevel::Warn:    return PalHint;
        case Platform::LogLevel::Debug:
        case Platform::LogLevel::Verbose: return PalMuted;
        default:                          return infoColor;
    }
}

namespace {
size_t g_defaultCapacity = TETHER_TUI_LOGPANE_CAPACITY;
}

LogPane::LogPane(size_t viewHeight, size_t capacity)
    : viewHeight_(viewHeight),
      capacity_(capacity ? capacity : g_defaultCapacity) {}

LogPane::~LogPane() { release(); }

void LogPane::attach() {
    if (attached_) return;
    attached_ = true;
    Platform::Logger::instance().setHandler(
        [this](Platform::LogLevel level, const char* tag, const char* msg) {
            static const char* lv[] = {"", "E", "W", "I", "D", "V"};
            const char* l = lv[std::min<int>(static_cast<int>(level), 5)];
            addLine(std::string(l) + " " + tag + ": " + msg, level);
        });
}

void LogPane::release() {
    if (!attached_) return;
    attached_ = false;
    Platform::Logger::instance().setHandler(nullptr);
}

void LogPane::addLine(std::string line) {
    addLine(std::move(line), Platform::LogLevel::Info);
}

void LogPane::addLine(std::string line, Platform::LogLevel level) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (level > min_level_) return;   // ingest filter
    lines_.push_back({std::move(line), level});
    while (lines_.size() > capacity_) lines_.pop_front();
}

void LogPane::setCapacity(size_t capacity) {
    if (capacity == 0) capacity = 1;
    std::lock_guard<std::mutex> lock(mutex_);
    capacity_ = capacity;
    while (lines_.size() > capacity_) lines_.pop_front();
}

void LogPane::setDefaultCapacity(size_t capacity) {
    g_defaultCapacity = capacity ? capacity : 1;
}

size_t LogPane::defaultCapacity() { return g_defaultCapacity; }

void LogPane::setMinLevel(Platform::LogLevel level) {
    std::lock_guard<std::mutex> lock(mutex_);
    min_level_ = level;
}

void LogPane::setViewLevel(Platform::LogLevel level) {
    view_level_ = level;
}

void LogPane::setViewTag(const std::string& substr) {
    view_tag_ = substr;
}

void LogPane::clearViewFilter() {
    view_level_ = Platform::LogLevel::None;
    view_tag_.clear();
}

void LogPane::scrollLines(int delta) {
    // Note: deliberately takes no lock — size() may race by a line or
    // two, which only shifts the clamp bound harmlessly.
    const long long total = static_cast<long long>(size());
    long long off = static_cast<long long>(scrollOffset_) -
                    static_cast<long long>(delta);
    scrollOffset_ = static_cast<size_t>(
        std::clamp<long long>(off, 0, std::max<long long>(total - 1, 0)));
}

void LogPane::scrollToEnd() { scrollOffset_ = 0; }

size_t LogPane::size() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return lines_.size();
}

void LogPane::render(TermWindow* w, short colorPair) {
    WINDOW* win = static_cast<WINDOW*>(w);
    werase(win);

    int h = 0, width = 0;
    getmaxyx(win, h, width);
    if (h <= 0 || width <= 0) return;

    std::deque<Entry> copy;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        copy = lines_;
    }
    // Apply the view filter (level + text substring).  Scroll offsets
    // count matching lines only.
    if (filtered()) {
        std::deque<Entry> kept;
        for (const auto& e : copy) {
            if (view_level_ != Platform::LogLevel::None &&
                e.level > view_level_)
                continue;
            if (!view_tag_.empty() &&
                e.text.find(view_tag_) == std::string::npos)
                continue;
            kept.push_back(e);
        }
        copy = std::move(kept);
    }
    const size_t total = copy.size();
    // Clamp the offset to what actually exists (lines may have been
    // appended or evicted since the last render).
    scrollOffset_ = std::min(scrollOffset_, total ? total - 1 : 0);

    // Slice [start, end): `end` is `scrollOffset_` lines back from the
    // newest; `start` fills the window height.
    const size_t end = total - scrollOffset_;
    const size_t start = end > static_cast<size_t>(h)
                       ? end - static_cast<size_t>(h) : 0;

    int row = 0;
    for (size_t i = start; i < end && row < h; ++i, ++row) {
        const short cp = levelColor(copy[i].level, colorPair);
        if (cp != PalNone) wattron(win, COLOR_PAIR(cp));
        mvwprintw(win, row, 0, "%s",
                  utf8Trunc(copy[i].text, width - 1).c_str());
        if (cp != PalNone) wattroff(win, COLOR_PAIR(cp));
    }
    // Scroll/filter indicator on the last row.
    if (h > 0 && (scrollOffset_ > 0 || filtered())) {
        std::string ind;
        if (scrollOffset_ > 0) ind = " scroll -" + std::to_string(scrollOffset_) + " ";
        if (filtered())        ind += " filtered ";
        wattron(win, COLOR_PAIR(PalHint) | A_REVERSE);
        mvwprintw(win, h - 1,
                  width > static_cast<int>(ind.size()) + 1
                      ? width - static_cast<int>(ind.size()) - 1 : 0,
                  "%s", ind.c_str());
        wattroff(win, COLOR_PAIR(PalHint) | A_REVERSE);
    }
}

} // namespace TUI
} // namespace Tether

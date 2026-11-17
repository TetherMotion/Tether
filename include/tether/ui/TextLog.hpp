#pragma once

/**
 * @file TextLog.hpp
 * @brief Toolkit-agnostic bounded text transcript — the scrollback
 *        model behind chat/monitor TUIs. Owns word-wrap at a set
 *        width, a size cap, and a "lines above bottom" scroll offset;
 *        renderers pull a visible slice of `const Line*`.
 *
 * Not curses: lives in tether/ui next to the other models so a
 * different frontend (Qt, web, test) can drive it identically.
 */

#include <algorithm>
#include <cstddef>
#include <deque>
#include <string>
#include <vector>

#include <logging/Logger.hpp>  // Tether::Platform::LogLevel

namespace Tether {
namespace UI {

class TextLog {
public:
    struct Line {
        std::string text;
        Platform::LogLevel level;
    };

    explicit TextLog(size_t cap = 4000) : cap_(cap) {}

    /**Word-wrap `text` at the current width and append the rows. */
    void add(const std::string& text, Platform::LogLevel level) {
        for (auto& w : wrap(text, width_))
            lines_.push_back({std::move(w), level});
        while (lines_.size() > cap_) lines_.pop_front();
        dirty_ = true;
    }

    /// Append raw lines (no wrapping — already laid out).
    void addRaw(const std::string& line, Platform::LogLevel level) {
        lines_.push_back({line, level});
        while (lines_.size() > cap_) lines_.pop_front();
        dirty_ = true;
    }

    void clear() { lines_.clear(); scroll_ = 0; }
    size_t size() const { return lines_.size(); }
    bool empty() const { return lines_.empty(); }
    const Line& back() const { return lines_.back(); }

    void setWidth(int w) { width_ = std::max(20, w); }
    int width() const { return width_; }

    /// Scroll offset in lines above the bottom (0 = following tail).
    int scroll() const { return scroll_; }
    void scrollBy(int n) {
        scroll_ = std::clamp(scroll_ + n, 0, int(lines_.size()));
    }
    void scrollEnd() { scroll_ = 0; }

    /// True when new content arrived since the last render pass.
    bool dirty() const { return dirty_; }
    void clearDirty() { dirty_ = false; }

    /**The at-most-`height` visible rows honoring the scroll offset. */
    std::vector<const Line*> visible(int height) const {
        std::vector<const Line*> vis;
        if (height <= 0) return vis;
        int end = int(lines_.size()) - scroll_;
        int start = std::max(0, end - height);
        end = std::max(0, end);
        vis.reserve(size_t(end - start));
        for (int i = start; i < end; ++i) vis.push_back(&lines_[size_t(i)]);
        return vis;
    }

    /**Word-wrap one string into ≤ w columns (shared with callers who
     * need a one-off wrap — kept identical for every consumer). */
    static std::vector<std::string> wrap(const std::string& text, int w) {
        w = std::max(8, w);
        std::vector<std::string> lines;
        std::string cur, word;
        auto flush_word = [&] {
            while (int(word.size()) > w) {
                if (!cur.empty()) { lines.push_back(cur); cur.clear(); }
                lines.push_back(word.substr(0, size_t(w)));
                word.erase(0, size_t(w));
            }
            if (!word.empty()) {
                if (!cur.empty() && int(cur.size() + word.size()) + 1 > w) {
                    lines.push_back(cur);
                    cur.clear();
                }
                if (!cur.empty()) cur += ' ';
                cur += word;
                word.clear();
            }
        };
        for (char c : text) {
            if (c == '\n') { flush_word(); lines.push_back(cur); cur.clear(); }
            else if (c == ' ') flush_word();
            else if (c != '\r' && c != '\t') word += c;
        }
        flush_word();
        if (!cur.empty() || lines.empty()) lines.push_back(cur);
        return lines;
    }

private:
    std::deque<Line> lines_;
    size_t cap_;
    int width_ = 80;
    int scroll_ = 0;
    bool dirty_ = false;
};

}  // namespace UI
}  // namespace Tether

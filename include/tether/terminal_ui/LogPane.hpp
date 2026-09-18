/**
 * @file LogPane.hpp
 * @brief Scrollable ring buffer of captured log lines for display inside a TUI
 *
 * While ncurses owns the screen, console logging would corrupt the layout.
 * attach() redirects the process logger into this pane's ring buffer;
 * release() restores the console handler.  render() draws the buffered
 * lines into a TermWindow.
 *
 * The pane keeps a deep scrollback (`capacity`, default 500 lines)
 * independent of the on-screen view height (`viewHeight`, default 4).
 * Lines are colored by severity (Error=red, Warn=yellow, Debug/Verbose=
 * gray, Info=plain).  The view follows the newest line unless the user
 * scrolled up (scrollLines()/scrollToEnd()).
 */

#pragma once

#include <cstddef>
#include <deque>
#include <mutex>
#include <string>
#include <unordered_set>
#include <vector>

#include "logging/Logger.hpp"
#include "tether/terminal_ui/Session.hpp"

/// Compile-time default scrollback depth for LogPane.  Override at
/// build time (-DTETHER_TUI_LOGPANE_CAPACITY=2000) or at runtime via
/// LogPane::setDefaultCapacity() before constructing the pane.
#ifndef TETHER_TUI_LOGPANE_CAPACITY
#define TETHER_TUI_LOGPANE_CAPACITY 500
#endif

namespace Tether {
namespace TUI {

/// One captured log line with its facets (level + module tag) for
/// multi-facet filtering.
struct LogEntry {
    std::string        text;   ///< full rendered line "I tag: msg"
    Platform::LogLevel level = Platform::LogLevel::Info;
    std::string        tag;    ///< module tag ("" when unknown)
};

/// Multi-facet view filter for LogPane — all facets AND together:
///   level  : keep lines with severity >= min_level (None = no limit)
///   include_tags : empty = keep all; otherwise tag must be in the set
///   exclude_tags : tag must NOT be in the set (wins over include)
///   contains : non-empty substring the full line must contain
///
/// Applied at render time, so it filters the existing scrollback
/// (history) as well as every line captured afterwards (live).
struct LogFilter {
    Platform::LogLevel            min_level = Platform::LogLevel::None;
    std::unordered_set<std::string> include_tags;
    std::unordered_set<std::string> exclude_tags;
    std::string                   contains;

    bool active() const {
        return min_level != Platform::LogLevel::None ||
               !include_tags.empty() || !exclude_tags.empty() ||
               !contains.empty();
    }
    bool matches(const LogEntry& e) const;
    void clear() { *this = LogFilter{}; }

    /// Parse a filter expression of space-separated terms:
    ///   "level>=warn" / "min=error"   — minimum severity
    ///   "tag=fsoe" / "+tag=fsoe"      — include module tag (repeatable)
    ///   "!tag=mailbox" / "-tag=x"     — exclude module tag (repeatable)
    ///   "text=stuck"                  — substring
    ///   <bare word>                   — substring (same as text=word)
    /// Unknown level names / malformed terms are ignored.
    static LogFilter parse(const std::string& expr);
};

class LogPane {
public:
    /// @param viewHeight  On-screen height used by the screens for layout
    ///                    (maxLines() reports this for compatibility).
    /// @param capacity    Scrollback depth in lines; 0 = use the current
    ///                    default (TETHER_TUI_LOGPANE_CAPACITY or
    ///                    setDefaultCapacity()).
    explicit LogPane(size_t viewHeight = 4, size_t capacity = 0);
    ~LogPane();

    LogPane(const LogPane&) = delete;
    LogPane& operator=(const LogPane&) = delete;

    /// Redirect the global logger into this pane.
    void attach();
    /// Restore the previous (console) log handler.  Called by ~LogPane().
    void release();

    /// Append a line directly (e.g. application status text, Info level).
    void addLine(std::string line);
    /// Append a line with an explicit severity for coloring.
    void addLine(std::string line, Platform::LogLevel level);
    /// Append a fully-faceted entry (level + module tag).
    void addLine(std::string line, Platform::LogLevel level,
                 std::string tag);

    /// Scroll the view.  Positive delta moves toward newer lines,
    /// negative toward older.  Any nonzero offset disables following
    /// until scrollToEnd() (or scrolling back to the bottom).
    void scrollLines(int delta);
    /// Jump to the newest line and resume following.
    void scrollToEnd();
    /// True while the view tracks the newest line.
    bool following() const { return scrollOffset_ == 0; }
    /// Lines above the newest currently hidden by scrolling.
    size_t scrollOffset() const { return scrollOffset_; }

    /// Draw buffered lines into `win` (cleared first), newest at the
    /// bottom (or the scrolled slice).  `colorPair` colors Info-level
    /// lines; Warn/Error/Debug/Verbose use their own severity colors.
    /// Lines rejected by the view filter are skipped.
    void render(TermWindow* win, short colorPair = PalNone);

    // ---- Buffer sizing -------------------------------------------------
    /// Resize the scrollback at runtime.  Shrinking evicts the oldest
    /// lines; the scroll offset is clamped accordingly.
    void setCapacity(size_t capacity);
    /// Change the default capacity used when a pane is constructed with
    /// capacity=0.  Affects panes created afterwards only.
    static void setDefaultCapacity(size_t capacity);
    static size_t defaultCapacity();

    // ---- Filtering -----------------------------------------------------
    /// Ingest filter: drop lines below this level at capture time
    /// (they never enter the buffer).  Default Verbose = keep all.
    void setMinLevel(Platform::LogLevel level);
    /// Multi-facet view filter — applies live to new lines and
    /// retroactively to the scrollback on every render.
    void setViewFilter(const LogFilter& f);
    const LogFilter& viewFilter() const { return view_filter_; }
    /// Convenience single-facet helpers (merge into the view filter).
    void setViewLevel(Platform::LogLevel level);
    void setViewTag(const std::string& substr);
    /// Module filtering helpers: add/remove a tag in the include or
    /// exclude set of the view filter.
    void includeTag(const std::string& tag, bool on = true);
    void excludeTag(const std::string& tag, bool on = true);
    /// All module tags seen so far (for building filter UIs).
    std::vector<std::string> knownTags() const;
    /// Reset all view filters (level + tags + text).
    void clearViewFilter();
    /// True when a view filter is active.
    bool filtered() const { return view_filter_.active(); }

    /// On-screen view height (kept under the old name — the screens use
    /// it to size the log window).
    size_t maxLines() const { return viewHeight_; }
    size_t capacity() const { return capacity_; }
    size_t size() const;

private:
    std::deque<LogEntry> lines_;
    mutable std::mutex mutex_;
    size_t            viewHeight_;
    size_t            capacity_;
    Platform::LogLevel min_level_ = Platform::LogLevel::Verbose;
    LogFilter         view_filter_;
    /// Lines hidden below the view bottom (0 = following newest).
    /// Only mutated by the UI thread; read without a lock is fine.
    size_t            scrollOffset_ = 0;
    bool              attached_ = false;
};

} // namespace TUI
} // namespace Tether

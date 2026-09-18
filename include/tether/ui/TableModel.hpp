#pragma once

/**
 * @file TableModel.hpp
 * @brief Navigable, filterable, searchable table view-model.
 *
 * Pure data + navigation state — no rendering, no ncurses.  A terminal
 * widget (TablePanel) draws it today; a web UI can serialize the same
 * model later.
 *
 * Selection and scroll index refer to the *visible* row list — the
 * subset of rows passing the complex filter and the quicksearch.
 * Quicksearch is a single case-insensitive substring applied on top of
 * the complex filter (type-ahead filtering), matching any cell or field.
 */

#include <algorithm>
#include <string>
#include <vector>

#include "tether/ui/Filter.hpp"

namespace Tether {
namespace UI {

/// Semantic row colors (renderer maps them to palette entries).
enum RowColor : int {
    RowNone  = 0,
    RowOk    = 1,   ///< green — good/live state
    RowWarn  = 2,   ///< yellow — degraded/attention
    RowErr   = 3,   ///< red — error state
    RowMuted = 4,   ///< gray — inactive/invalid
    RowInfo  = 5,   ///< blue/cyan — informational
};

struct Row {
    std::vector<std::string> cells;   ///< rendered columns (strings only)
    FieldSet fields;                  ///< filterable named fields
    int tag = -1;                     ///< app payload (e.g. slave index)
    int color = RowNone;
};

class TableModel {
public:
    void setColumns(std::vector<std::string> cols) { columns_ = std::move(cols); }
    const std::vector<std::string>& columns() const { return columns_; }

    /// Replace rows; tries to keep the selection on the same `tag`.
    void setRows(std::vector<Row> rows) {
        const int tag = selectedTag();
        rows_ = std::move(rows);
        dirty_ = true;
        if (tag >= 0) {
            recompute();
            for (size_t i = 0; i < visible_.size(); ++i) {
                if (rows_[visible_[i]].tag == tag) { sel_ = i; return; }
            }
        }
        sel_ = std::min(sel_, visible_.empty() ? size_t{0} : visible_.size() - 1);
    }

    const std::vector<Row>& rows() const { return rows_; }
    size_t rowCount() const { return rows_.size(); }

    // -- visible (filtered) rows ----------------------------------------

    size_t visibleCount() {
        recompute();
        return visible_.size();
    }

    /// Row at visible index `vi` (nullptr out of range).
    const Row* visibleRow(size_t vi) {
        recompute();
        return vi < visible_.size() ? &rows_[visible_[vi]] : nullptr;
    }

    /// Scroll offset expressed in visible rows (for the renderer).
    size_t scrollOffset() const { return scroll_; }
    void setScroll(size_t s) { scroll_ = s; }

    /// Keep `sel_` inside `height` visible rows.
    void ensureVisible(int height) {
        if (height <= 0) return;
        if (sel_ < scroll_) scroll_ = sel_;
        if (sel_ >= scroll_ + static_cast<size_t>(height))
            scroll_ = sel_ - static_cast<size_t>(height) + 1;
        const size_t n = const_cast<TableModel*>(this)->visibleCount();
        const size_t max_scroll = n > static_cast<size_t>(height)
                                      ? n - static_cast<size_t>(height) : 0;
        scroll_ = std::min(scroll_, max_scroll);
    }

    // -- selection -------------------------------------------------------

    size_t selectedIndex() const { return sel_; }
    const Row* selected() {
        recompute();
        return sel_ < visible_.size() ? &rows_[visible_[sel_]] : nullptr;
    }
    int selectedTag() {
        const Row* r = const_cast<TableModel*>(this)->selected();
        return r ? r->tag : -1;
    }

    void move(int delta) {
        recompute();
        if (visible_.empty()) { sel_ = 0; return; }
        const int n = static_cast<int>(visible_.size());
        const int s = static_cast<int>(sel_) + delta;
        sel_ = static_cast<size_t>(std::clamp(s, 0, n - 1));
    }
    void pageMove(int delta, int height) { move(delta * std::max(1, height)); }
    void selectFirst() { sel_ = 0; }
    void selectLast() {
        recompute();
        sel_ = visible_.empty() ? 0 : visible_.size() - 1;
    }
    void selectByTag(int tag) {
        recompute();
        for (size_t i = 0; i < visible_.size(); ++i) {
            if (rows_[visible_[i]].tag == tag) { sel_ = i; return; }
        }
    }

    // -- filtering / quicksearch -----------------------------------------

    bool setFilterExpr(const std::string& expr) {
        filter_ = RecordFilter::parse(expr);
        filter_expr_ = expr;
        sel_ = scroll_ = 0;
        dirty_ = true;
        return true;   // parse never fails — degrade-to-substring policy
    }
    const std::string& filterExpr() const { return filter_expr_; }
    void clearFilter() {
        filter_.clear();
        filter_expr_.clear();
        dirty_ = true;
    }
    bool hasFilter() const { return filter_.active() || !search_.empty(); }

    /// Incremental quicksearch (type-ahead).  Called per keystroke.
    void setSearch(const std::string& s) {
        search_ = s;
        sel_ = scroll_ = 0;
        dirty_ = true;
    }
    const std::string& search() const { return search_; }

private:
    bool rowVisible(const Row& r) const {
        if (!search_.empty()) {
            std::string hay;
            for (const auto& c : r.cells) { hay += c; hay += ' '; }
            if (!RecordFilter::containsCi(hay, search_)) return false;
        }
        if (filter_.active()) {
            std::string hay;
            for (const auto& c : r.cells) { hay += c; hay += ' '; }
            if (!filter_.matches(r.fields, hay)) return false;
        }
        return true;
    }

    void recompute() {
        if (!dirty_) return;
        dirty_ = false;
        visible_.clear();
        for (size_t i = 0; i < rows_.size(); ++i) {
            if (rowVisible(rows_[i])) visible_.push_back(i);
        }
        if (sel_ >= visible_.size()) sel_ = visible_.empty() ? 0 : visible_.size() - 1;
    }

    std::vector<std::string> columns_;
    std::vector<Row> rows_;
    RecordFilter filter_;
    std::string filter_expr_;
    std::string search_;
    size_t sel_ = 0;
    size_t scroll_ = 0;
    std::vector<size_t> visible_;
    bool dirty_ = true;
};

} // namespace UI
} // namespace Tether

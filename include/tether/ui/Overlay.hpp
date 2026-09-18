#pragma once

/**
 * @file Overlay.hpp
 * @brief Overlay panel models — toolkit-agnostic pop-up content.
 *
 * An Overlay is a model: it exposes a title, content (either a
 * TableModel or plain text lines) and a key handler.  The frontend
 * (terminal_ui::OverlayHost today, a modal/route in a web UI later)
 * renders it on top of the owning panel and routes keys to the top of
 * the stack.
 *
 * Panels accept "actions" (PanelAction): a hotkey + label + callback,
 * typically pushing an Overlay.  The footer shows action labels
 * automatically.
 */

#include <functional>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "tether/ui/TableModel.hpp"

namespace Tether {
namespace UI {

// ---------------------------------------------------------------------------
// Panel actions
// ---------------------------------------------------------------------------

struct PanelAction {
    int key = 0;                  ///< trigger key (e.g. 'i', 'w')
    std::string label;            ///< hint text, e.g. "i:info"
    std::string description;      ///< longer text for help overlays
    std::function<void()> invoke; ///< typically pushes an Overlay
};

// ---------------------------------------------------------------------------
// Overlay model
// ---------------------------------------------------------------------------

class Overlay {
public:
    virtual ~Overlay() = default;

    /// Title shown in the overlay border.
    virtual std::string title() const { return {}; }

    /// When non-null the frontend renders this table as the content
    /// (searchable/filterable for free).  Takes precedence over lines().
    virtual TableModel* table() { return nullptr; }

    /// Text content lines (used when table() is null).
    virtual std::vector<std::string> lines() { return {}; }

    /// Keys routed to the top overlay.  Return true when consumed.
    /// Esc/close handling is the frontend's job.
    virtual bool handleKey(int /*key*/) { return false; }

    /// Footer hint text while this overlay is on top.
    virtual std::string keyHints() const { return {}; }
};

/// Simple text overlay — title + wrapped lines + optional key handler.
class TextOverlay : public Overlay {
public:
    TextOverlay(std::string title, std::vector<std::string> lines)
        : title_(std::move(title)), lines_(std::move(lines)) {}

    std::string title() const override { return title_; }
    std::vector<std::string> lines() override { return lines_; }
    std::function<bool(int)> onKey;    ///< optional
    bool handleKey(int key) override {
        return onKey ? onKey(key) : false;
    }
    std::string key_hints;
    std::string keyHints() const override { return key_hints; }

private:
    std::string title_;
    std::vector<std::string> lines_;
};

/// Overlay presenting a TableModel — gets navigation, quicksearch and
/// complex filtering through the host's standard table keys.
class TableOverlay : public Overlay {
public:
    TableOverlay(std::string title, TableModel model)
        : title_(std::move(title)), model_(std::move(model)) {}

    std::string title() const override { return title_; }
    TableModel* table() override { return &model_; }
    bool handleKey(int key) override {
        switch (key) {
            case 'k': case 'u': model_.move(-1); return true;
            case 'j': case 'd': model_.move(+1); return true;
            default: return false;
        }
    }
    std::string keyHints() const override { return "up/dn:nav Esc:close"; }

private:
    std::string title_;
    TableModel model_;
};

// ---------------------------------------------------------------------------
// Overlay stack (model side — owns overlay lifetimes)
// ---------------------------------------------------------------------------

class OverlayStack {
public:
    void push(std::unique_ptr<Overlay> o) { stack_.push_back(std::move(o)); }
    void pop() { if (!stack_.empty()) stack_.pop_back(); }
    void clear() { stack_.clear(); }
    Overlay* top() { return stack_.empty() ? nullptr : stack_.back().get(); }
    bool empty() const { return stack_.empty(); }
    size_t size() const { return stack_.size(); }

private:
    std::vector<std::unique_ptr<Overlay>> stack_;
};

} // namespace UI
} // namespace Tether

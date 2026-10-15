#pragma once

/**
 * @file KeyMap.hpp
 * @brief Key-binding registry — dispatch, footer hints and help text
 *        from one declaration.
 *
 * Views currently declare their keys twice: an `onKey` switch that
 * dispatches, and a separate `keyHints` string for the footer / `?`
 * help overlay.  `UI::KeyMap` collapses both: each `Binding` maps one
 * or more key codes to an invoke callback plus the hint/description
 * text, so `handleKey`, `keyHints()` and `helpLines()` all derive
 * from the same table and can't drift apart.
 *
 * Usage:
 *   UI::KeyMap keys;
 *   keys.add({KEY_UP, "up", "Move selection up", [&]{ m.move(-1); return true; }});
 *   keys.add({'/', "/:search", "Open the quicksearch prompt", [&]{ ... }});
 *   ...
 *   view.onKey = [&](int k){ return keys.handleKey(k); };
 *   view.keyHintsFn = [&]{ return keys.keyHints(); };
 */

#include <functional>
#include <string>
#include <vector>

namespace Tether {
namespace UI {

class KeyMap {
public:
    struct Binding {
        std::vector<int> keys;          ///< key codes (chars and KEY_*)
        std::string hint;               ///< footer hint, e.g. "up/dn:nav"
        std::string description;        ///< longer text for the ? overlay
        std::function<bool()> invoke;   ///< false = "not consumed"
    };

    /// Register a binding for `key` (single-key convenience).
    void add(int key, std::string hint, std::string description,
             std::function<bool()> invoke) {
        bindings_.push_back({{key}, std::move(hint),
                             std::move(description), std::move(invoke)});
    }

    /// Register a multi-key binding (e.g. arrows + hjkl for one action).
    void add(Binding b) { bindings_.push_back(std::move(b)); }

    /// Dispatch: first binding listing `key` wins.  A binding whose
    /// invoke returns false falls through to the next matching
    /// binding (rare) and then to "unhandled".
    bool handleKey(int key) {
        for (auto& b : bindings_)
            for (const int k : b.keys)
                if (k == key)
                    return b.invoke ? b.invoke() : true;
        return false;
    }

    /// Footer hint text — the distinct non-empty hints in
    /// registration order, space-joined.
    std::string keyHints() const {
        std::string out;
        for (const auto& b : bindings_)
            if (!b.hint.empty()) {
                if (!out.empty()) out += ' ';
                out += b.hint;
            }
        return out;
    }

    /// "hint — description" lines for a help overlay.
    std::vector<std::string> helpLines() const {
        std::vector<std::string> out;
        for (const auto& b : bindings_) {
            std::string line = b.hint.empty() ? keyName(b.keys.front())
                                              : b.hint;
            if (!b.description.empty()) {
                line += "  —  ";
                line += b.description;
            }
            out.push_back(std::move(line));
        }
        return out;
    }

    bool empty() const { return bindings_.empty(); }

private:
    /// Human-readable name for a key code (printable chars as-is).
    static std::string keyName(int key) {
        if (key >= 32 && key <= 126)
            return std::string(1, static_cast<char>(key));
        switch (key) {
            case 258: return "Down";  case 259: return "Up";
            case 260: return "Left";  case 261: return "Right";
            case 262: return "Home";  case 360: return "End";
            case 338: return "PgDn";  case 339: return "PgUp";
            case 27:  return "Esc";   case '\n': return "Enter";
            case '\t': return "Tab";  case 343:  return "Enter";
            default:  return "key#" + std::to_string(key);
        }
    }

    std::vector<Binding> bindings_;
};

} // namespace UI
} // namespace Tether

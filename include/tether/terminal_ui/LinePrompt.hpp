#pragma once

#include <functional>
#include <string>
#include <vector>

namespace Tether {
namespace TUI {

/// Specification for a modal one-line text prompt (TreeScreen::openLinePrompt).
///
/// While a prompt is open the screen routes every key to it — printable
/// characters append to the buffer, Backspace deletes, Enter commits,
/// Esc cancels.  The detail pane renders the prompt instead of the node
/// detail ("<label>: <buffer>_" plus the dim hint lines).
///
///   screen.openLinePrompt({
///       .label = "search",
///       .hint_lines = {"(type to filter — Enter applies, Esc cancels)"},
///       .on_commit = [&](const std::string& s) { applySearch(s); },
///   });
struct LinePromptSpec {
    std::string label;                            ///< input prefix, e.g. "search"
    std::string initial;                          ///< initial buffer contents
    std::vector<std::string> hint_lines;          ///< dim lines under the input
    std::function<void(const std::string&)> on_commit;  ///< Enter pressed
    std::function<void()> on_cancel;              ///< Esc pressed (optional)
};

} // namespace TUI
} // namespace Tether

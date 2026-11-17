#pragma once

/**
 * @file ChatScreen.hpp
 * @brief Ready-made PanelScreen for an agent-chat session: scrollable
 *        transcript + input line, driven by a UI::ChatSession model and
 *        a Utils::IFramePump transport (SEND/QUIT outbound).
 *
 *   ChatSession ses;  IFramePump pump;    // e.g. FramedChild or in-proc queue
 *   ChatScreen screen("kico chat — gen/", child, ses);
 *   screen.run(cancel);
 *   child.quit();
 */

#include <atomic>
#include <string>

#include "tether/ui/ChatSession.hpp"
#include "tether/utils/FramePump.hpp"

namespace Tether {
namespace TUI {

class ChatScreen {
public:
    ChatScreen(std::string title,
               Utils::IFramePump& child,
               UI::ChatSession& ses);

    /**Run the draw/dispatch loop until 'q'/Esc or `cancel`. */
    void run(std::atomic<bool>& cancel);

private:
    std::string         title_;
    Utils::IFramePump* child_;
    UI::ChatSession*    ses_;
};

} // namespace TUI
} // namespace Tether

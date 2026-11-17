#pragma once

/**
 * @file AgentMonitorScreen.hpp
 * @brief Ready-made PanelScreen for an agent-run monitor: event log,
 *        live status line, ASK option menu / free-text prompt — all
 *        driven by a UI::AgentMonitor model and a Utils::IFramePump
 *        transport.  The app supplies title + the two objects; run()
 *        owns the draw/dispatch loop.
 *
 *   AgentMonitor mon;  IFramePump pump;    // e.g. FramedChild or in-proc queue
 *   AgentMonitorScreen screen("kico generate x.slx", child, mon);
 *   screen.run(cancel);
 *   child.quit();
 */

#include <atomic>
#include <string>

#include "tether/ui/AgentMonitor.hpp"
#include "tether/utils/FramePump.hpp"

namespace Tether {
namespace TUI {

class AgentMonitorScreen {
public:
    AgentMonitorScreen(std::string title,
                       Utils::IFramePump& child,
                       UI::AgentMonitor& mon);

    /**Run the draw/dispatch loop until 'q'/Esc or `cancel`. */
    void run(std::atomic<bool>& cancel);

private:
    std::string          title_;
    Utils::IFramePump*  child_;
    UI::AgentMonitor*    mon_;
};

} // namespace TUI
} // namespace Tether

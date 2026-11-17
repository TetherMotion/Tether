#pragma once

/**
 * @file FramePump.hpp
 * @brief Frame-source interface for TUI screens — what AgentMonitorScreen
 *        and ChatScreen poll each tick.  FramedChild implements it for
 *        process backends; in-process hosts (e.g. a Rust FFI layer)
 *        implement it over their own queue.
 */

#include <string>
#include <vector>

namespace Tether {
namespace Utils {

/// One decoded frame.
struct Frame {
    std::string kind;
    std::string payload;
};

class IFramePump {
public:
    virtual ~IFramePump() = default;

    /// Drain every complete frame received since the last call.
    virtual std::vector<Frame> poll() = 0;

    /// Drain captured backend stderr (may be empty).
    virtual std::string drainErr() = 0;

    /// False once the backend is gone — screens show "[backend exited]".
    virtual bool alive() const = 0;
};

} // namespace Utils
} // namespace Tether

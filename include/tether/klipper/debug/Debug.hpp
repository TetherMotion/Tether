/**
 * @file Debug.hpp
 * @brief Debug flags and diagnostic gate for Klipper.
 *
 * Provides:
 *   - DebugFlags: bitfield of debug enable flags
 *   - DiagnosticGate: conditional logging/dump based on flags
 *
 * DebugManager is a thin wrapper over Tether::Platform::Logger.
 * Debug-flag-gated messages are forwarded to the platform Logger at
 * Debug level with a "klipper" tag, so all log routing (timestamps,
 * custom handlers, level filtering) is centralised in one place.
 */

#pragma once

#include "logging/Logger.hpp"

#include <cstdint>
#include <functional>
#include <string>
#include <string_view>

namespace tether::klipper::debug {

/// @brief Debug flag bits.
enum class DebugFlag : uint32_t {
    None        = 0,
    Commands    = 1u << 0,  ///< Log all commands
    Responses   = 1u << 1,  ///< Log all responses
    Motion      = 1u << 2,  ///< Log motion blocks
    Clock       = 1u << 3,  ///< Log clock sync
    Objects     = 1u << 4,  ///< Log object allocation
    Transports  = 1u << 5,  ///< Log transport I/O
    Thermal     = 1u << 6,  ///< Log thermal control
    Homing      = 1u << 7,  ///< Log homing
    Probing     = 1u << 8,  ///< Log probing
    Config      = 1u << 9,  ///< Log config parsing
    All         = 0xFFFFFFFFu,
};

inline DebugFlag operator|(DebugFlag a, DebugFlag b) {
    return static_cast<DebugFlag>(static_cast<uint32_t>(a) | static_cast<uint32_t>(b));
}

inline DebugFlag operator&(DebugFlag a, DebugFlag b) {
    return static_cast<DebugFlag>(static_cast<uint32_t>(a) & static_cast<uint32_t>(b));
}

inline bool hasFlag(DebugFlag flags, DebugFlag test) {
    return static_cast<uint32_t>(flags & test) != 0;
}

/// @brief Debug flags manager — thin wrapper over Tether::Platform::Logger.
class DebugManager {
public:
    /// @brief Legacy log callback type (kept for backward compatibility).
    /// If set, messages are also delivered to this callback in addition to
    /// the platform Logger.
    using LogCallback = std::function<void(std::string_view message)>;

    DebugManager() = default;

    /// @brief Set debug flags.
    void setFlags(DebugFlag flags) { flags_ = flags; }

    /// @brief Get current debug flags.
    DebugFlag flags() const { return flags_; }

    /// @brief Enable a specific debug flag.
    void enable(DebugFlag flag) { flags_ = flags_ | flag; }

    /// @brief Disable a specific debug flag.
    void disable(DebugFlag flag) {
        flags_ = static_cast<DebugFlag>(
            static_cast<uint32_t>(flags_) & ~static_cast<uint32_t>(flag));
    }

    /// @brief Check if a flag is enabled.
    bool isEnabled(DebugFlag flag) const { return hasFlag(flags_, flag); }

    /// @brief Set a legacy log callback (optional, for backward compatibility).
    /// If set, log() messages are also delivered to this callback.
    void setLogCallback(LogCallback cb) { logCb_ = std::move(cb); }

    /// @brief Log a message if the corresponding flag is enabled.
    /// Forwards to Tether::Platform::Logger at Debug level with tag "klipper".
    void log(DebugFlag flag, std::string_view message) {
        if (!isEnabled(flag)) return;
        // Forward to the platform Logger.
        Tether::Platform::Logger::instance().logFormatted(
            Tether::Platform::LogLevel::Debug, "klipper", message);
        // Also deliver to legacy callback if set.
        if (logCb_) logCb_(message);
    }

    /// @brief Check if logging is enabled for a flag.
    bool shouldLog(DebugFlag flag) const {
        return isEnabled(flag);
    }

private:
    DebugFlag flags_ = DebugFlag::None;
    LogCallback logCb_;
};

} // namespace tether::klipper::debug

/**
 * @file Debug.hpp
 * @brief Debug flags and diagnostic gate for Klipper.
 *
 * Provides:
 *   - DebugFlags: bitfield of debug enable flags
 *   - DiagnosticGate: conditional logging/dump based on flags
 */

#pragma once

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

/// @brief Debug flags manager with optional log callback.
class DebugManager {
public:
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

    /// @brief Set the log callback.
    void setLogCallback(LogCallback cb) { logCb_ = std::move(cb); }

    /// @brief Log a message if the corresponding flag is enabled.
    void log(DebugFlag flag, std::string_view message) {
        if (isEnabled(flag) && logCb_) {
            logCb_(message);
        }
    }

    /// @brief Check if logging is enabled for a flag.
    bool shouldLog(DebugFlag flag) const {
        return isEnabled(flag) && logCb_ != nullptr;
    }

private:
    DebugFlag flags_ = DebugFlag::None;
    LogCallback logCb_;
};

} // namespace tether::klipper::debug

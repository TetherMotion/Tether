#pragma once

/// @file PrinterState.hpp
/// @brief Printer state machine enum and helpers.

#include <cstdint>
#include <string>

namespace tether::klipper::klippy {

// ============================================================================
// Printer state machine
// ============================================================================

enum class PrinterState : uint8_t {
    Startup   = 0,
    Ready     = 1,
    Printing  = 2,
    Paused    = 3,
    Error     = 4,
    Shutdown  = 5,
};

/// @brief Convert PrinterState to string.
inline const char* stateToString(PrinterState s) {
    switch (s) {
        case PrinterState::Startup:  return "startup";
        case PrinterState::Ready:    return "ready";
        case PrinterState::Printing: return "printing";
        case PrinterState::Paused:   return "paused";
        case PrinterState::Error:    return "error";
        case PrinterState::Shutdown: return "shutdown";
    }
    return "unknown";
}

} // namespace tether::klipper::klippy

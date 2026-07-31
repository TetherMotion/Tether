/**
 * @file PrinterStateMachine.hpp
 * @brief Printer state machine for Klipper.
 *
 * Implements the Klipper printer state machine:
 *   startup -> ready -> printing -> ready
 *   startup -> error
 *   any -> shutdown (via M112 or fault)
 */

#pragma once

#include "tether/klipper/klippy/KlippyUdsServer.hpp"

#include <cstdint>
#include <functional>
#include <string>

namespace tether::klipper::klippy {

/// @brief Convert PrinterState to string (extended version with all states).
inline std::string printerStateToString(PrinterState s) {
    return stateToString(s);
}

/// @brief Printer state machine.
class PrinterStateMachine {
public:
    using StateChangeCallback = std::function<void(PrinterState, PrinterState)>;

    PrinterStateMachine() = default;

    /// @brief Set the state change callback.
    void setStateChangeCallback(StateChangeCallback cb) {
        stateChangeCb_ = std::move(cb);
    }

    /// @brief Get the current state.
    PrinterState state() const { return state_; }

    /// @brief Get the state message.
    const std::string& message() const { return message_; }

    /// @brief Transition to a new state.
    /// @return True if the transition was valid.
    bool transition(PrinterState newState, const std::string& message = "") {
        if (!isValidTransition(state_, newState)) return false;

        PrinterState old = state_;
        state_ = newState;
        if (!message.empty()) message_ = message;

        if (stateChangeCb_) {
            stateChangeCb_(old, newState);
        }
        return true;
    }

    /// @brief Check if a transition is valid.
    static bool isValidTransition(PrinterState from, PrinterState to) {
        // From any state to shutdown is always valid (emergency stop)
        if (to == PrinterState::Shutdown) return true;

        switch (from) {
            case PrinterState::Startup:
                return to == PrinterState::Ready ||
                       to == PrinterState::Error ||
                       to == PrinterState::Shutdown;
            case PrinterState::Ready:
                return to == PrinterState::Printing ||
                       to == PrinterState::Shutdown;
            case PrinterState::Printing:
                return to == PrinterState::Ready ||
                       to == PrinterState::Paused ||
                       to == PrinterState::Shutdown;
            case PrinterState::Paused:
                return to == PrinterState::Printing ||
                       to == PrinterState::Ready ||
                       to == PrinterState::Shutdown;
            case PrinterState::Error:
                // Error is terminal (except shutdown)
                return to == PrinterState::Shutdown;
            case PrinterState::Shutdown:
                // Shutdown is terminal
                return false;
        }
        return false;
    }

    /// @brief Check if the printer is ready for commands.
    bool isOperational() const {
        return state_ == PrinterState::Ready ||
               state_ == PrinterState::Printing ||
               state_ == PrinterState::Paused;
    }

    /// @brief Check if the printer is printing.
    bool isPrinting() const { return state_ == PrinterState::Printing; }

    /// @brief Check if the printer is in a terminal state.
    bool isTerminal() const {
        return state_ == PrinterState::Shutdown ||
               state_ == PrinterState::Error;
    }

private:
    PrinterState state_ = PrinterState::Startup;
    std::string message_ = "Printer is not ready";
    StateChangeCallback stateChangeCb_;
};

} // namespace tether::klipper::klippy

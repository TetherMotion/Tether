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

/**
 * @brief Convert PrinterState to a lowercase state name.
 *
 * Forwards to the canonical stateToString(); kept for API symmetry in
 * klippy code paths.
 * @param s  State to name.
 * @return State name string for reporting/status output.
 */
inline std::string printerStateToString(PrinterState s) {
    return stateToString(s);
}

/**@brief Printer state machine. */
class PrinterStateMachine {
public:
    using StateChangeCallback = std::function<void(PrinterState, PrinterState)>;

    PrinterStateMachine() = default;

    /**
     * @brief Set the state change callback.
     *
     * Invoked synchronously from transition() after a valid state change
     * with (old, new). Keep it non-blocking.
     * @param cb  Callback taking (oldState, newState); empty disables it.
     */
    void setStateChangeCallback(StateChangeCallback cb) {
        stateChangeCb_ = std::move(cb);
    }

    /**
     * @brief Get the current state.
     * @return Active PrinterState (Startup until first transition).
     */
    PrinterState state() const { return state_; }

    /**
     * @brief Get the state message.
     *
     * The human-readable reason text supplied with the last successful
     * transition; defaults to "Printer is not ready" at startup.
     * @return Current message string, valid until the next transition.
     */
    const std::string& message() const { return message_; }

    /**
     * @brief Transition to a new state.
     *
     * Validated against isValidTransition() — an invalid transition
     * leaves state and message untouched and returns false. On success
     * the state-change callback fires and @p message, when non-empty,
     * replaces the current message.
     * @param newState  Target state; Shutdown is always accepted (E-stop).
     * @param message   Optional reason text to store for message().
     * @return True if the transition was valid.
     */
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

    /**
     * @brief Check if a transition is valid.
     *
     * Shutdown is reachable from every state (emergency stop); Error and
     * Shutdown are terminal. Startup->Ready|Error, Ready->Printing,
     * Printing->Ready|Paused, Paused->Printing|Ready.
     * @param from  Source state.
     * @param to    Target state.
     * @return true if the transition is allowed.
     */
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

    /**
     * @brief Check if the printer is ready for commands.
     * @return true in Ready, Printing, or Paused — i.e. not Startup,
     *         Error, or Shutdown.
     */
    bool isOperational() const {
        return state_ == PrinterState::Ready ||
               state_ == PrinterState::Printing ||
               state_ == PrinterState::Paused;
    }

    /**
     * @brief Check if the printer is printing.
     * @return true only in the Printing state (Paused reports false).
     */
    bool isPrinting() const { return state_ == PrinterState::Printing; }

    /**
     * @brief Check if the printer is in a terminal state.
     * @return true in Shutdown or Error — no further transitions are
     *         possible (Error can only exit to Shutdown).
     */
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

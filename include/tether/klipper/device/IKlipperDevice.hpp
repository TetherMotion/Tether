/**
 * @file IKlipperDevice.hpp
 * @brief Interface for KlipperDevice (extracted for layer separation and testability).
 *
 * @details
 * This interface allows the klippy layer (KlippyInstance) to depend on an
 * abstraction rather than the concrete KlipperDevice implementation. This
 * breaks the direct dependency from klippy -> device and enables mocking
 * the device in tests.
 */

#pragma once

#include "tether/klipper/objects/Stepper.hpp"

#include <cstdint>
#include <memory>

namespace tether::klipper::device {

/// @brief Interface for the Klipper device's motion-related operations.
///
/// This interface exposes only the methods that KlippyInstance needs to call
/// on the device. The full KlipperDevice class implements this interface
/// plus many more methods used internally by the device layer.
class IKlipperDevice {
public:
    virtual ~IKlipperDevice() = default;

    /// @brief Open the transport and start serving.
    /// @return True if the device started successfully.
    virtual bool start() = 0;

    /// @brief Pump the event loop: read transport, parse blocks, dispatch, ack.
    virtual void pump() = 0;

    /// @brief Advance the MCU clock by @p deltaTicks.
    virtual void advanceClock(uint32_t deltaTicks) = 0;

    /// @brief Register a Stepper peripheral and auto-wire the queue_step /
    ///        set_next_step_dir / reset_step_clock handlers for its OID.
    /// @return The OID passed in, for chaining.
    virtual uint8_t registerStepper(std::shared_ptr<objects::Stepper> stepper) = 0;

    /// @brief Register all default stepper motion command handlers
    ///        (queue_step, set_next_step_dir, reset_step_clock) for every
    ///        Stepper currently registered.
    virtual void enableStepperMotion() = 0;
};

} // namespace tether::klipper::device

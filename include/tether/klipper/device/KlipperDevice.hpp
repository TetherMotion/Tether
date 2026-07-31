/**
 * @file KlipperDevice.hpp
 * @brief Klipper device: serves the data dictionary, processes commands,
 *        and executes motion via Tether's motion subsystem.
 *
 * @details
 * The KlipperDevice is the device-side entry point. It:
 *   1. Opens a transport.
 *   2. Serves the data dictionary via the identify handshake.
 *   3. Processes incoming command blocks, dispatches to peripheral handlers.
 *   4. Sends ack blocks and response blocks.
 *   5. Executes motion in one of two selectable modes:
 *        - Passthrough: steps are executed at scheduled clock times on a
 *          virtual stepper (StepExecutor).
 *        - Reconstruct+Replan: steps are fed to the MotionReconstructor and
 *          emitted as MotionBlocks for analysis.
 *
 * The device runs an event loop that pumps the transport, parses blocks,
 * dispatches commands, and advances the clock.
 */

#pragma once

#include "tether/klipper/transport/IByteStreamTransport.hpp"
#include "tether/klipper/protocol/DataDictionary.hpp"
#include "tether/klipper/protocol/CommandTable.hpp"
#include "tether/klipper/protocol/IdentifyProtocol.hpp"
#include "tether/klipper/protocol/MessageBlock.hpp"
#include "tether/klipper/clock/McuClock.hpp"
#include "tether/klipper/objects/OidAllocator.hpp"
#include "tether/klipper/objects/Stepper.hpp"
#include "tether/klipper/objects/Peripherals.hpp"
#include "tether/klipper/motion/MotionBlockSink.hpp"
#include "tether/klipper/motion/MotionReconstructor.hpp"

#include <memory>
#include <unordered_map>
#include <vector>
#include <functional>
#include <string>

namespace tether::klipper::device {

/// @brief Motion execution mode for the device.
enum class MotionMode {
    Passthrough,       ///< Execute steps directly on a virtual stepper.
    ReconstructReplan, ///< Reconstruct steps into MotionBlocks for analysis.
};

/// @brief Configuration for the Klipper device.
struct KlipperDeviceConfig {
    /// MCU clock frequency in Hz.
    uint32_t clockFreqHz = 180000000;
    /// Motion execution mode.
    MotionMode motionMode = MotionMode::Passthrough;
    /// Sink for motion blocks (required for ReconstructReplan mode).
    std::shared_ptr<motion::MotionBlockSink> motionSink;
};

/**
 * @brief Klipper device: serves the dict, processes commands, executes motion.
 */
class KlipperDevice {
public:
    KlipperDevice(std::shared_ptr<transport::IByteStreamTransport> transport,
                  protocol::DataDictionary dict,
                  KlipperDeviceConfig config = {});
    ~KlipperDevice();

    /// @brief Open the transport and start serving.
    bool start();

    /// @brief Pump the event loop: read transport, parse blocks, dispatch, ack.
    void pump();

    /// @brief Advance the MCU clock by @p deltaTicks.
    void advanceClock(uint32_t deltaTicks);

    /// @return The MCU clock.
    const clock::McuClock& clock() const { return mcuClock_; }

    /// @return The data dictionary.
    const protocol::DataDictionary& dictionary() const { return dict_; }

    /// @brief Register a peripheral object by OID.
    void registerPeripheral(uint8_t oid, std::shared_ptr<void> peripheral);

    /// @brief Register a Stepper peripheral and auto-wire the queue_step /
    ///        set_next_step_dir / reset_step_clock handlers for its OID.
    /// @return The OID passed in, for chaining.
    uint8_t registerStepper(std::shared_ptr<objects::Stepper> stepper);

    /// @brief Register all default stepper motion command handlers
    ///        (queue_step, set_next_step_dir, reset_step_clock) for every
    ///        Stepper currently registered via registerPeripheral/registerStepper.
    void enableStepperMotion();

    /// @brief Register a command handler for a format string.
    void onCommand(const std::string& formatStr, protocol::CommandHandler handler);

    /// @brief Send a response block.
    bool sendResponse(const std::string& formatStr,
                      const std::vector<protocol::ParamValue>& params);

    /// @return The last received in-order sequence (for ack building).
    uint8_t lastReceivedSeq() const { return lastRecvSeq_; }

private:
    void processBlock(const protocol::MessageBlock& block);
    void sendAck(uint8_t seq);

    std::shared_ptr<transport::IByteStreamTransport> transport_;
    protocol::DataDictionary dict_;
    KlipperDeviceConfig config_;
    clock::McuClock mcuClock_;
    std::unique_ptr<protocol::CommandTable> commandTable_;
    std::unique_ptr<protocol::IdentifyServer> identifyServer_;
    objects::OidAllocator oidAllocator_;
    std::unordered_map<uint8_t, std::shared_ptr<void>> peripherals_;
    /// Typed stepper map for default queue_step dispatch.
    std::unordered_map<uint8_t, std::shared_ptr<objects::Stepper>> steppers_;
    /// Per-OID base clock set by reset_step_clock (for queue_step scheduling).
    std::unordered_map<uint8_t, uint32_t> stepperBaseClocks_;
    uint8_t lastRecvSeq_ = 0;
    bool started_ = false;
};

} // namespace tether::klipper::device

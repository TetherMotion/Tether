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

#include "tether/klipper/device/IKlipperDevice.hpp"
#include "tether/klipper/device/KlipperDeviceConfig.hpp"
#include "tether/klipper/transport/IByteStreamTransport.hpp"
#include "tether/klipper/protocol/DataDictionary.hpp"
#include "tether/klipper/protocol/CommandTable.hpp"
#include "tether/klipper/protocol/IdentifyProtocol.hpp"
#include "tether/klipper/protocol/MessageBlock.hpp"
#include "tether/klipper/protocol/BlockReader.hpp"
#include "tether/klipper/clock/McuClock.hpp"
#include "tether/klipper/objects/OidAllocator.hpp"
#include "tether/klipper/objects/Stepper.hpp"
#include "tether/klipper/objects/Peripherals.hpp"
#include "tether/klipper/motion/MotionBlockSink.hpp"
#include "tether/klipper/motion/MotionReconstructor.hpp"
#include "tether/klipper/motion/StepScheduler.hpp"

#include <memory>
#include <unordered_map>
#include <vector>
#include <functional>
#include <string>

namespace tether::klipper::device {

/**
 * @brief Klipper device: serves the dict, processes commands, executes motion.
 */
class KlipperDevice : public IKlipperDevice {
public:
    KlipperDevice(std::shared_ptr<transport::IByteStreamTransport> transport,
                  protocol::DataDictionary dict,
                  KlipperDeviceConfig config = {});
    ~KlipperDevice() override;

    /// @brief Open the transport and start serving.
    bool start() override;

    /// @brief Pump the event loop: read transport, parse blocks, dispatch, ack.
    void pump() override;

    /// @brief Reset device-side protocol state for connection re-establishment.
    ///
    /// Clears the last-received sequence, shutdown/finalize flags, OID
    /// allocator, stepper base clocks, and re-registers default command
    /// handlers. The transport is not touched (call start() to re-open it).
    /// This mirrors the reset() pattern of the pcapng reader and BlockReader,
    /// allowing the device to recover from a host reconnect without full
    /// reconstruction.
    void reset();

    /// @brief Advance the MCU clock by @p deltaTicks.
    void advanceClock(uint32_t deltaTicks) override;

    /// @return The MCU clock.
    const clock::McuClock& clock() const { return mcuClock_; }

    /// @return The data dictionary.
    const protocol::DataDictionary& dictionary() const { return dict_; }

    /// @brief Register a peripheral object by OID.
    void registerPeripheral(uint8_t oid, std::shared_ptr<void> peripheral);

    /// @brief Register a Stepper peripheral and auto-wire the queue_step /
    ///        set_next_step_dir / reset_step_clock handlers for its OID.
    /// @return The OID passed in, for chaining.
    uint8_t registerStepper(std::shared_ptr<objects::Stepper> stepper) override;

    /// @brief Register all default stepper motion command handlers
    ///        (queue_step, set_next_step_dir, reset_step_clock) for every
    ///        Stepper currently registered via registerPeripheral/registerStepper.
    void enableStepperMotion() override;

    /// @brief Register default handlers for the 5 core device commands:
    ///        allocate_oids, get_config, get_status, shutdown, finalize_config.
    ///        Called automatically in the constructor; safe to call again
    ///        after resetting state. Application-registered handlers via
    ///        onCommand() override these defaults.
    void enableDefaultCommands();

    /// @brief Tick the real-time StepScheduler (if enabled). Call this
    ///        periodically from the main loop or a timer thread.
    /// @return Number of steps fired, or 0 if the scheduler is not enabled.
    size_t tickStepScheduler();

    /// @return The StepScheduler, or nullptr if not enabled.
    motion::StepScheduler* stepScheduler() { return stepScheduler_.get(); }

    /// @brief Wait for all scheduled steps to complete (blocks).
    /// @param maxWaitMs Maximum wait in milliseconds (0 = no limit).
    /// @return True if all steps completed, false on timeout.
    bool waitStepScheduler(uint32_t maxWaitMs = 0) {
        if (!stepScheduler_) return true;
        return stepScheduler_->wait(maxWaitMs);
    }

    /// @brief Register a command handler for a format string.
    void onCommand(const std::string& formatStr, protocol::CommandHandler handler);

    /// @brief Send a response block.
    bool sendResponse(const std::string& formatStr,
                      const std::vector<protocol::ParamValue>& params);

    /// @return The last received in-order sequence (for ack building).
    uint8_t lastReceivedSeq() const { return lastRecvSeq_; }

    /// @return True if the device is in shutdown state.
    bool isShutdown() const { return shutdown_; }

    /// @return True if finalize_config has been received.
    bool isConfigFinalized() const { return configFinalized_; }

    /// @return The config CRC (valid after get_config or finalize_config).
    uint32_t configCrc() const { return configCrc_; }

    /// @return The number of allocated OIDs.
    uint8_t allocatedOidCount() const { return oidAllocator_.nextOid(); }

    /// @return Block parse statistics from the internal BlockReader.
    const protocol::BlockParseStats& blockParseStats() const {
        return blockReader_.stats();
    }

    /// @return Number of corrupt blocks skipped by the internal BlockReader.
    size_t skippedBlockCount() const { return blockReader_.skippedBlockCount(); }

    /// @brief Enable error-recovery mode on the internal BlockReader.
    /// When enabled, corrupt blocks are skipped (with optional callback)
    /// instead of causing pump() to silently drop remaining data.
    void setBlockRecoveryMode(bool enabled,
                              protocol::BlockReader::ErrorCallback cb = nullptr) {
        blockReader_.setRecoveryMode(enabled, std::move(cb));
    }

private:
    void processBlock(const protocol::MessageBlock& block);
    void sendAck(uint8_t seq);

    std::shared_ptr<transport::IByteStreamTransport> transport_;
    protocol::DataDictionary dict_;
    KlipperDeviceConfig config_;
    clock::McuClock mcuClock_;
    std::unique_ptr<protocol::CommandTable> commandTable_;
    std::unique_ptr<protocol::IdentifyServer> identifyServer_;
    protocol::BlockReader blockReader_;
    objects::OidAllocator oidAllocator_;
    std::unordered_map<uint8_t, std::shared_ptr<void>> peripherals_;
    /// Typed stepper map for default queue_step dispatch.
    std::unordered_map<uint8_t, std::shared_ptr<objects::Stepper>> steppers_;
    /// Per-OID base clock set by reset_step_clock (for queue_step scheduling).
    std::unordered_map<uint8_t, uint32_t> stepperBaseClocks_;
    /// Real-time step scheduler (optional, enabled by config.useStepScheduler).
    std::unique_ptr<motion::StepScheduler> stepScheduler_;
    uint8_t lastRecvSeq_ = 0;
    bool started_ = false;
    bool shutdown_ = false;
    bool configFinalized_ = false;
    uint32_t configCrc_ = 0;
    bool receivedFirstBlock_ = false;
};

} // namespace tether::klipper::device

/**
 * @file KlippyHost.hpp
 * @brief Klippy host: orchestrates the connection, dict download, clock sync,
 *        and command dispatch to a Klipper device.
 *
 * @details
 * The KlippyHost is the host-side entry point. It:
 *   1. Opens a transport to the device.
 *   2. Downloads the data dictionary via the identify handshake.
 *   3. Establishes clock synchronisation via periodic get_clock queries.
 *   4. Allocates OIDs and configures peripherals.
 *   5. Sends commands (via the SerialQueue) and dispatches responses.
 *   6. Translates Tether MotionPlans into queue_step sequences and sends them.
 *
 * The host runs an event loop that pumps the transport, processes acks,
 * checks retransmit timeouts, and dispatches responses.
 */

#pragma once

#include "tether/klipper/transport/IByteStreamTransport.hpp"
#include "tether/klipper/reliability/SerialQueue.hpp"
#include "tether/klipper/protocol/DataDictionary.hpp"
#include "tether/klipper/protocol/CommandTable.hpp"
#include "tether/klipper/protocol/IdentifyProtocol.hpp"
#include "tether/klipper/clock/ClockSync.hpp"
#include "tether/klipper/objects/OidAllocator.hpp"
#include "tether/klipper/objects/Stepper.hpp"
#include "tether/klipper/objects/Peripherals.hpp"
#include "tether/klipper/motion/MotionTranslator.hpp"
#include "tether/klipper/motion/MotionBlock.hpp"

#include <memory>
#include <functional>
#include <vector>
#include <unordered_map>
#include <string>

namespace tether::klipper::klippy {

/**
 * @brief Klippy host: connects to a device, downloads the dict, syncs the
 *        clock, and dispatches commands.
 */
class KlippyHost {
public:
    explicit KlippyHost(std::shared_ptr<transport::IByteStreamTransport> transport);
    ~KlippyHost();

    /// @brief Open the transport and start the connection.
    bool connect();

    /// @brief Run the identify handshake to download the data dictionary.
    /// @param devicePump Optional callback to pump the device side (for
    ///        loopback/in-process use). Called after each host pump.
    bool downloadDictionary(std::function<void()> devicePump = nullptr);

    /// @brief Synchronise the clock by sending get_clock.
    /// @param devicePump Optional callback to pump the device side.
    bool syncClock(std::function<void()> devicePump = nullptr);

    /// @brief Allocate an OID for a peripheral type.
    uint8_t allocateOid(const std::string& type);

    /// @brief Send a command by format string with parameter values.
    bool sendCommand(const std::string& formatStr,
                     const std::vector<protocol::ParamValue>& params);

    /// @brief Send a translated step sequence to the device.
    ///
    /// Emits (per axis, in order):
    ///   - reset_step_clock oid=%c clock=<startClock>  (once at the start)
    ///   - set_next_step_dir oid=%c dir=<0|1>          (when direction changes)
    ///   - queue_step oid=%c interval=%u count=%hu add=%hi  (per StepCommand)
    ///
    /// The host must have completed downloadDictionary() first.
    /// @param pump Optional callback invoked when the serial-queue window is
    ///        full (should pump the device side + host side so acks flow back
    ///        and free window space). If null, excess commands are dropped.
    /// @return Number of queue_step commands successfully enqueued.
    size_t sendStepSequence(const motion::AxisStepSequence& seq,
                            std::function<void()> pump = nullptr);

    /// @brief Send multiple step sequences (convenience overload).
    size_t sendStepSequences(const std::vector<motion::AxisStepSequence>& seqs,
                             std::function<void()> pump = nullptr);

    /// @brief Register a response handler for a response format string.
    void onResponse(const std::string& formatStr, protocol::ResponseHandler handler);

    /// @brief Pump the event loop: read transport, process acks, dispatch responses.
    void pump();

    /// @brief Check retransmit timeouts.
    void checkTimeouts();

    /// @return The data dictionary (valid after downloadDictionary()).
    const protocol::DataDictionary& dictionary() const { return dict_; }

    /// @return The clock sync (valid after syncClock()).
    const clock::ClockSync& clockSync() const { return clockSync_; }

    /// @return The serial queue.
    reliability::SerialQueue& serialQueue() { return *serialQueue_; }

    /// @return True if the host is connected and the dictionary is downloaded.
    bool isReady() const { return connected_ && dictDownloaded_; }

private:
    std::shared_ptr<transport::IByteStreamTransport> transport_;
    std::unique_ptr<reliability::SerialQueue> serialQueue_;
    protocol::DataDictionary dict_;
    std::unique_ptr<protocol::CommandTable> commandTable_;
    clock::ClockSync clockSync_;
    objects::OidAllocator oidAllocator_;
    bool connected_ = false;
    bool dictDownloaded_ = false;

    // Pending get_clock response handling.
    clock::HostTime getClockSendTime_;
    bool getClockPending_ = false;
};

} // namespace tether::klipper::klippy

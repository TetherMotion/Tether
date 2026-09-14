#pragma once

#include <bit>
#include <cstdint>
#include <functional>
#include <optional>
#include <ranges>

#include "tether/fsoe/FSoESlave.hpp"
#include "tether/fsoe/TypedProcessData.hpp"

namespace FSoE {

/// Declarative command→status bit mirror for FSoE slave emulators.
///
/// Each entry echoes the bit value at `command_bit` (index into the
/// master→slave safe output data) verbatim into `status_bit` (index into
/// the slave→master safe input data).  Corresponding functions sit at
/// DIFFERENT bit positions in the two directions, so the table records
/// both positions explicitly.
struct BitMirror {
    uint8_t command_bit;
    uint8_t status_bit;
};

/// constexpr bit-index extraction from a single-bit mask — lets mirror
/// tables be written in terms of PDO mask constants (e.g.
/// `SOMANET_RxPDO_1700::kSTO`) instead of hardcoded bit numbers.
constexpr uint8_t bitIndexOf(uint64_t mask)
{
    return static_cast<uint8_t>(std::countr_zero(mask));
}

/// Whether a codec declares a command→status bit-mirror table as a
/// `kStatusBitMirrors` static member (any range of BitMirror).
template<typename CodecT>
concept HasStatusBitMirrors =
    std::ranges::range<decltype(CodecT::kStatusBitMirrors)>;

/// Generic, profile-parameterised FSoE slave emulator base.
///
/// Owns the low-level FSoESlave state machine and a TypedSlaveProcessDataView
/// that encodes/decodes safe data using the profile-specific Codec.  Derived
/// classes provide the profile-specific semantics by overriding the protected
/// virtual hooks:
///
///   - onInitialize()         — set initial status fields from config
///   - onCommandConsumed(cmd) — track edge-triggered command bits
///   - buildStatus(out)       — map current command + internal state → status
///   - onReset()              — reset profile-specific state to safe defaults
///
/// The Config type must provide a `toSlaveConfig()` method that returns a
/// fully populated FSoESlaveConfig (including safeInputSize / safeOutputSize
/// from the profile's Codec).
///
/// @tparam CodecT        Profile codec (must provide kMainToSlaveSize,
///                       kSlaveToMainSize, encode/decode static methods)
/// @tparam CommandT      Profile command (master→slave) payload type
/// @tparam StatusT       Profile status (slave→master) payload type
/// @tparam ConfigT       Profile-specific emulator config type
template<typename CodecT, typename CommandT, typename StatusT, typename ConfigT>
class FSoESlaveEmulator {
public:
    using Codec = CodecT;
    using Command = CommandT;
    using Status = StatusT;
    using Config = ConfigT;

    explicit FSoESlaveEmulator(const Config& config)
        : config_(config)
        , slave_config_(config.toSlaveConfig())
        , slave_(slave_config_)
        , typed_view_(slave_)
    {
    }

    virtual ~FSoESlaveEmulator() = default;

    bool initialize()
    {
        initialized_ = slave_.initialize();
        if (!initialized_) {
            return false;
        }
        published_status_ = {};
        onInitialize();
        refreshPublishedStatus();
        return true;
    }

    /// Consume the latest decoded command and republish the status.
    /// Called by the application after the FSoE slave has processed a new
    /// master→slave frame, so that the next slave→master frame reflects
    /// the latest command.
    void synchronizeCommandAndStatus()
    {
        consumeLatestCommand();
        refreshPublishedStatus();
    }

    const Status& status() const { return published_status_; }
    const Command& lastCommand() const { return last_command_; }
    bool isInitialized() const { return initialized_; }

    FSoESlave& rawSlave() { return slave_; }
    const FSoESlave& rawSlave() const { return slave_; }

protected:
    /// Called during initialize() after published_status_ is zeroed.
    /// Override to set initial status fields from the config.
    virtual void onInitialize() {}

    /// Called after a new command is decoded from the FSoE frame.
    /// Override to track edge-triggered bits (error ack, restart ack, etc.).
    virtual void onCommandConsumed(const Command& /*cmd*/) {}

    /// Called to build the current status from the command + internal state.
    /// Must be overridden by every concrete profile.
    virtual void buildStatus(Status& out) = 0;

    /// Called when the emulator is reset to the safe state.
    /// Override to reset profile-specific internal state.
    virtual void onReset() {}

    /// Consume the latest decoded command from the FSoE frame.
    /// Updates last_command_ and calls onCommandConsumed().
    /// Protected so derived classes can call it independently of
    /// refreshPublishedStatus() (e.g. in step() where position/velocity
    /// must be updated between consume and publish).
    void consumeLatestCommand()
    {
        const auto decoded = typed_view_.consume();
        if (!decoded) {
            return;
        }
        last_command_ = *decoded;
        onCommandConsumed(last_command_);
    }

    /// Build the status from current state and publish it via the codec.
    /// Protected so derived classes can call it after updating internal
    /// state (e.g. position/velocity in step()).
    void refreshPublishedStatus()
    {
        buildStatus(published_status_);
        (void)typed_view_.publish(published_status_);
        applyStatusBitMirrors();
    }

    /// Echo the codec-declared command bits (CodecT::kStatusBitMirrors)
    /// into the corresponding status bits of the safe-input image.
    /// Called automatically by refreshPublishedStatus(); a codec without
    /// a mirror table mirrors nothing.
    ///
    /// Only in Data state does the slave→master payload carry live
    /// status flags — earlier connection states use the safe-data area
    /// for handshake/parameter exchange, so nothing is mirrored there.
    void applyStatusBitMirrors()
    {
        if constexpr (HasStatusBitMirrors<CodecT>) {
            if (!slave_.isOperational()) {
                return;
            }
            for (const BitMirror& m : CodecT::kStatusBitMirrors) {
                slave_.setSafeInputBit(
                    m.status_bit, slave_.getSafeOutputBit(m.command_bit));
            }
        }
    }

    Config config_;
    FSoESlaveConfig slave_config_;
    FSoESlave slave_;
    TypedSlaveProcessDataView<Command, Status, Codec> typed_view_;
    Command last_command_{};
    Status published_status_{};
    bool initialized_ = false;
};

} // namespace FSoE

/**
 * @file CompactDriveTerminal.hpp
 * @brief Driver for Beckhoff compact-drive terminals with the CiA402-
 *        semantic "DRV" interface (EL7062, EL7411, EL72xx, ELM72xx)
 *
 * These terminals expose a CiA402-compatible control/status channel
 * through Beckhoff DRV objects rather than the canonical 0x60xx indices:
 *
 *   DRV outputs (object 0x7010, +0x70 per axis):
 *     :01 Controlword          :03 Modes of operation
 *     :05 Target position      :06 Target velocity     :09 Target torque
 *
 *   DRV inputs (object 0x6010, +0x70 per axis):
 *     :01 Statusword           :03 Modes of operation display
 *     :06 Following error      :07 Velocity actual     :08 Torque actual
 *
 *   FB inputs (object 0x6000, +0x70 per axis):
 *     :17 Position actual value
 *
 * Field offsets are resolved from the SII PDO list by (index, subindex),
 * so every device revision and every PDO-assignment variant works —
 * whichever DRV PDOs the default assignment maps determine what the
 * driver can command (hasTargetVelocity() etc. report presence).
 *
 * Controlword/statusword bits follow CiA402 semantics; driveState()
 * decodes the statusword into the CiA402 FSA states.  The driver covers
 * the cyclic part — PDO assignment changes, modes-of-operation defaults
 * and motor parameters are configured over CoE/SDO by the application.
 *
 * @code
 *   auto drv = CompactDriveTerminal::findFirst(master, Devices::EL7201);
 *   drv->start();
 *   drv->requestEnableOperation(0);          // CiA402 enable sequence
 *   drv->setTargetVelocity(0, 100000);       // cyclic velocity target
 * @endcode
 */

#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <span>
#include <vector>

#include "tether/Beckhoff/TerminalBase.hpp"
#include "tether/ethercat/PDOManager.hpp"
#include "tether/ethercat/SlaveDiscoveryManager.hpp"
#include "tether/ethercat/Types.hpp"

namespace EtherCAT {

class Master;

namespace Beckhoff {

// ============================================================================
// Known DRV-interface compact drives (from the Beckhoff ESIs)
// ============================================================================
// num_bits holds the axis count.  Product codes verified against
// Beckhoff EL7xxx.xml / EL72xx.xml / ELM72xx.xml.
// None verified on hardware yet — supported, not verified yet.

namespace Devices {

inline constexpr DeviceIdentity EL7062{0x00000002, 0x1B963052, 2, "EL7062"};
inline constexpr DeviceIdentity EL7411{0x00000002, 0x1CF33052, 1, "EL7411"};
inline constexpr DeviceIdentity EL7201{0x00000002, 0x1C213052, 1, "EL7201"};
inline constexpr DeviceIdentity EL7211{0x00000002, 0x1C2B3052, 1, "EL7211"};
inline constexpr DeviceIdentity EL7221{0x00000002, 0x1C353052, 1, "EL7221"};

// ELM72xx premium drives (Beckhoff vendor ID, ELM product-code range).
inline constexpr DeviceIdentity ELM7211{0x00000002, 0x502274B9, 1, "ELM7211"};
inline constexpr DeviceIdentity ELM7212{0x00000002, 0x502274C9, 2, "ELM7212"};
inline constexpr DeviceIdentity ELM7221{0x00000002, 0x50227559, 1, "ELM7221"};
inline constexpr DeviceIdentity ELM7222{0x00000002, 0x50227569, 2, "ELM7222"};
inline constexpr DeviceIdentity ELM7231{0x00000002, 0x502275F9, 1, "ELM7231"};

// EP/EJ variants — same DRV-interface electronics.
inline constexpr DeviceIdentity EJ7062{0x00000002, 0x1B962852, 2, "EJ7062"};
inline constexpr DeviceIdentity EJ7211{0x00000002, 0x1C2B2852, 1, "EJ7211"};
inline constexpr DeviceIdentity EJ7411{0x00000002, 0x1CF32852, 1, "EJ7411"};
inline constexpr DeviceIdentity EP7211{0x00000002, 0x1C2B4052, 1, "EP7211"};
inline constexpr DeviceIdentity EP7412{0x00000002, 0x1CF44052, 2, "EP7412"};
inline constexpr DeviceIdentity EP7402{0x00000002, 0x1CEA4052, 1, "EP7402"};
// Note: EP7342/ER7342 are DC-motor boxes — they use the MOT interface,
// not DRV, and are registered under DcMotorTerminal instead.

/// Every known DRV-interface compact-drive terminal.
inline constexpr std::array kCompactDriveTerminals{
    EL7062, EL7411, EL7201, EL7211, EL7221,
    ELM7211, ELM7212, ELM7221, ELM7222, ELM7231,
    EJ7062, EJ7211, EJ7411, EP7211, EP7412, EP7402,
};

} // namespace Devices

// ============================================================================
// CompactDriveTerminal
// ============================================================================

/// CiA402 finite-state-automaton state decoded from the statusword.
enum class DriveState : uint8_t {
    NotReadyToSwitchOn,
    SwitchOnDisabled,
    ReadyToSwitchOn,
    SwitchedOn,
    OperationEnabled,
    QuickStopActive,
    FaultReactionActive,
    Fault,
    Unknown,
};

class CompactDriveTerminal : public TerminalBase {
public:
    /// Maximum axes per device (ELM72x2 carry two).
    static constexpr size_t kMaxAxes = 4;

    // -- Construction / factories ------------------------------------------------

    CompactDriveTerminal(Master& master, uint16_t slave_index,
                         const DeviceIdentity& identity);
    CompactDriveTerminal(Master& master, const DiscoveredSlave& slave,
                         const DeviceIdentity& identity);

    ~CompactDriveTerminal() override;

    CompactDriveTerminal(CompactDriveTerminal&&) noexcept            = default;
    CompactDriveTerminal& operator=(CompactDriveTerminal&&) noexcept = default;
    CompactDriveTerminal(const CompactDriveTerminal&)                = delete;
    CompactDriveTerminal& operator=(const CompactDriveTerminal&)     = delete;

    /// True when `s` carries the given vendor/product identity.
    static bool matches(const DiscoveredSlave& s, const DeviceIdentity& id) {
        return s.hasVendorAndProduct(id.vendor_id, id.product_code);
    }

    /// Scan the bus for the first slave matching `identity`.
    static Result<CompactDriveTerminal> findFirst(
        Master& master, const DeviceIdentity& identity);
    static Result<CompactDriveTerminal> findFirst(
        Master& master, const DeviceIdentity& identity,
        std::span<const DiscoveredSlave> scan);

    // -- Bring-up ----------------------------------------------------------------------

    /// Standalone bring-up to SAFE-OP (position addressing).
    Result<> configure();

    /// configure() then OP (+ managed realtime loop by default).
    Result<> start() { return start(StartOptions()); }
    Result<> start(const StartOptions& opts);

    /// Chained-operation contract.
    Result<> prepareForLogicalExchange();
    Result<> mapLogicalAndEnterSafeOp();
    Result<> requestOp(int timeout_ms);

    // -- Axes -----------------------------------------------------------------------------

    /// Number of axes resolved from the SII layout (1 or 2).
    size_t axisCount() const { return axes_.size(); }

    // -- Control (SM2 image) ----------------------------------------------------------------

    void     setControlword(size_t axis, uint16_t cw);
    uint16_t controlword(size_t axis) const;
    bool     hasControlword(size_t axis) const;

    /// Modes-of-operation command (object :03, 8 bit) — only present when
    /// the PDO assignment maps it (ELM72xx maps it; EL72xx usually sets
    /// the mode via SDO instead).
    void setModesOfOperation(size_t axis, int8_t mode);
    bool hasModesOfOperation(size_t axis) const;

    void setTargetPosition(size_t axis, int32_t pos);
    bool hasTargetPosition(size_t axis) const;

    void setTargetVelocity(size_t axis, int32_t vel);
    bool hasTargetVelocity(size_t axis) const;

    void setTargetTorque(size_t axis, int16_t tq);
    bool hasTargetTorque(size_t axis) const;

    // -- Status (SM3 image) -------------------------------------------------------------------

    uint16_t statusword(size_t axis) const;
    bool     hasStatusword(size_t axis) const;

    int8_t modesOfOperationDisplay(size_t axis) const;
    bool   hasModesOfOperationDisplay(size_t axis) const;

    int32_t actualPosition(size_t axis) const;
    bool    hasActualPosition(size_t axis) const;

    int32_t actualVelocity(size_t axis) const;
    bool    hasActualVelocity(size_t axis) const;

    int16_t actualTorque(size_t axis) const;
    bool    hasActualTorque(size_t axis) const;

    int32_t followingError(size_t axis) const;
    bool    hasFollowingError(size_t axis) const;

    // -- CiA402 helpers --------------------------------------------------------------------------

    /// Decode the statusword into the CiA402 FSA state.
    DriveState driveState(size_t axis) const;
    static DriveState decodeState(uint16_t statusword);

    /// Statusword bit helpers (CiA402 semantics).
    bool fault(size_t axis) const;
    bool operationEnabled(size_t axis) const;
    bool warningPresent(size_t axis) const;
    bool targetReached(size_t axis) const;   ///< statusword bit 10

    // Controlword command sequences — write the command into the PDO
    // image; it is transmitted with the next cyclic exchange.  The FSA
    // needs each step to be observed by the slave, so step through them
    // one call per cycle (or call driveState() to track progress).

    /// "Shutdown" (0x06) — first step of the enable sequence.
    void requestShutdown(size_t axis);
    /// "Switch on" (0x07).
    void requestSwitchOn(size_t axis);
    /// "Enable operation" (0x0F) — final step of the enable sequence.
    void requestEnableOperation(size_t axis);
    /// "Disable voltage" (0x00) — immediate coast-down.
    void requestDisableVoltage(size_t axis);
    /// "Quick stop" (0x02).
    void requestQuickStop(size_t axis);
    /// "Fault reset" (controlword bit 7 set) — clear with clearFaultReset()
    /// once the fault clears.
    void requestFaultReset(size_t axis);
    void clearFaultReset(size_t axis);

private:
    static constexpr uint32_t kInvalid = ~0u;

    struct Field {
        uint32_t bit_off = kInvalid;
        uint8_t  bit_len = 0;
        bool present() const { return bit_off != kInvalid && bit_len != 0; }
    };

    /// All resolved fields of one axis.
    struct Axis {
        Field controlword, modes, target_pos, target_vel, target_torque;
        Field statusword, modes_disp, following_err, vel_act, torque_act;
        Field fb_position;
    };

    bool resolveLayout();
    Result<> prepare(PDO::PDOAddressMode mode);

    const Axis* axisPtr(size_t i) const {
        return i < axes_.size() ? &axes_[i] : nullptr;
    }
    Axis* axisPtr(size_t i) {
        return i < axes_.size() ? &axes_[i] : nullptr;
    }

    int64_t inSigned(const Field& f) const;
    void    outSigned(const Field& f, int32_t v);
    uint16_t inWord(const Field& f) const;

    std::vector<Axis> axes_;
};

} // namespace Beckhoff

} // namespace EtherCAT

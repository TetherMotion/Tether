/**
 * @file DcMotorTerminal.hpp
 * @brief Beckhoff EL73xx two-channel DC-motor (H-bridge) terminals
 *
 * The EL73xx family drives brushed DC motors through an H-bridge stage
 * per channel.  Their process data uses the Beckhoff "MOT" interface —
 * related to the POS stepper interface but velocity-only:
 *
 *   SM2 (outputs), per axis:
 *     MOT Control object (0x70xx): Enable, Reset, Reduce torque, ...
 *     MOT Velocity (same object, subindex 0x21/33): 16-bit signed
 *     optional ENC Control object (0x70xx): latch enables, set counter,
 *         set-counter value (subindex 0x11/17)
 *
 *   SM3 (inputs), per axis:
 *     MOT Status object (0x60xx): Ready to enable, Ready, Warning,
 *         Error, Moving +/- , Torque reduced, Digital inputs, ...
 *     optional ENC Status object (0x60xx): latch valid, set-counter
 *         done, under/overflow + counter/latch values
 *
 * Object-index layout (from the ESIs):
 *   EL7332 / EJ7332 / EJ7334 : MOT objects 0x7000/0x7010(/0x7020/0x7030),
 *                              no encoder objects
 *   EL7342 / EJ7342 / EP7342 / ER7342 / EPP7342 :
 *                              ENC objects 0x7000/0x7010,
 *                              MOT objects 0x7020/0x7030
 *
 * The driver discovers axes by scanning the SM2 PDO entries: every
 * 0x70xx object carrying a >=16-bit "Velocity" entry (subindex 0x21)
 * is one motor axis; remaining 0x70xx objects with a "Set counter"
 * bit / "Set counter value" entry are encoder objects paired to the
 * axes in index order.  All field offsets are resolved by (object
 * index, subindex) — no byte offsets are hard-coded.
 */

#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <span>
#include <vector>

#include "tether/Beckhoff/TerminalBase.hpp"
#include "tether/ethercat/PDOManager.hpp"             // PDOAddressMode
#include "tether/ethercat/SlaveDiscoveryManager.hpp"  // DiscoveredSlave
#include "tether/ethercat/Types.hpp"                  // SlaveState

namespace EtherCAT {

class Master;

namespace Beckhoff {

// ============================================================================
// Known MOT-interface DC-motor terminals (from the Beckhoff ESIs)
// ============================================================================
// num_bits holds the axis count.  All ESI-verified for shape; none
// verified on hardware yet — supported, not verified yet.

namespace Devices {

// EL73xx — 2/4-channel H-bridge DC-motor terminals.
inline constexpr DeviceIdentity EL7332{0x00000002, 0x1CA43052, 2, "EL7332"};
inline constexpr DeviceIdentity EL7342{0x00000002, 0x1CAE3052, 2, "EL7342"};

// EJ plug-in modules (0x2852 suffix).
inline constexpr DeviceIdentity EJ7332{0x00000002, 0x1CA42852, 2, "EJ7332"};
inline constexpr DeviceIdentity EJ7334{0x00000002, 0x1CA62852, 4, "EJ7334"};
inline constexpr DeviceIdentity EJ7342{0x00000002, 0x1CAE2852, 2, "EJ7342"};

// EP/ER field boxes + EtherCAT-P module.
inline constexpr DeviceIdentity EP7342 {0x00000002, 0x1CAE4052, 2, "EP7342"};
inline constexpr DeviceIdentity ER7342 {0x00000002, 0x1CAE4852, 2, "ER7342"};
inline constexpr DeviceIdentity EPP7342{0x00000002, 0x647790E9, 2, "EPP7342"};

/// Every known MOT-interface DC-motor terminal.
inline constexpr std::array kDcMotorTerminals{
    EL7332, EL7342, EJ7332, EJ7334, EJ7342, EP7342, ER7342, EPP7342,
};

} // namespace Devices

// ============================================================================
// DcMotorTerminal
// ============================================================================

class DcMotorTerminal : public TerminalBase {
public:
    static constexpr size_t kMaxAxes = 4;

    // -- Construction / factories ----------------------------------------------

    DcMotorTerminal(Master& master, uint16_t slave_index,
                    const DeviceIdentity& identity);
    DcMotorTerminal(Master& master, const DiscoveredSlave& slave,
                    const DeviceIdentity& identity);

    ~DcMotorTerminal() override;

    DcMotorTerminal(DcMotorTerminal&&) noexcept            = default;
    DcMotorTerminal& operator=(DcMotorTerminal&&) noexcept = default;
    DcMotorTerminal(const DcMotorTerminal&)                = delete;
    DcMotorTerminal& operator=(const DcMotorTerminal&)     = delete;

    /// True when `s` carries the given vendor/product identity.
    static bool matches(const DiscoveredSlave& s, const DeviceIdentity& id) {
        return s.hasVendorAndProduct(id.vendor_id, id.product_code);
    }

    /// Scan the bus for the first slave matching `identity`.
    static Result<DcMotorTerminal> findFirst(Master& master,
                                             const DeviceIdentity& identity);
    static Result<DcMotorTerminal> findFirst(
        Master& master, const DeviceIdentity& identity,
        std::span<const DiscoveredSlave> scan);

    // -- Bring-up ------------------------------------------------------------------

    /// Standalone bring-up to SAFE-OP (position addressing).
    Result<> configure();

    /// configure() then OP (+ managed realtime loop by default).
    Result<> start() { return start(StartOptions()); }
    Result<> start(const StartOptions& opts);

    /// Chained-operation contract (shared logical address space).
    Result<> prepareForLogicalExchange();
    Result<> mapLogicalAndEnterSafeOp();
    Result<> requestOp(int timeout_ms);

    // -- Axes ------------------------------------------------------------------------

    /// Number of resolved motor axes (0 before prepare()).
    size_t axisCount() const { return axes_.size(); }

    // -- Motor control (SM2 image), per axis -------------------------------------------

    /// MOT Control :01 — power-stage enable.
    void setEnable(size_t axis, bool on);
    /// MOT Control :02 — fault reset (write 1, then clear).
    void setReset(size_t axis, bool on);
    /// MOT Control :03 — reduced motor torque.
    void setReduceTorque(size_t axis, bool on);

    /**
     * @brief MOT velocity command (object 0x70xx:0x21, 16 bit signed).
     * Application units depend on the terminal's SDO configuration.
     */
    void setVelocity(size_t axis, int16_t v);

    /// Raw MOT control word (16 bit) — for bits not wrapped above.
    uint16_t motControlWord(size_t axis) const;

    // -- Motor status (SM3 image), per axis --------------------------------------------

    /// MOT Status :01 — ready to enable.
    bool readyToEnable(size_t axis) const;
    /// MOT Status :02 — ready (power stage active).
    bool ready(size_t axis) const;
    /// MOT Status :03 — warning present.
    bool warning(size_t axis) const;
    /// MOT Status :04 — error present (reset via setReset()).
    bool error(size_t axis) const;
    /// MOT Status :05/:06 — currently moving in +/- direction.
    bool movingPositive(size_t axis) const;
    bool movingNegative(size_t axis) const;
    /// MOT Status :07 — running at reduced torque.
    bool torqueReduced(size_t axis) const;
    /// MOT Status :0C/:0D — digital inputs 1/2.
    bool digitalInput1(size_t axis) const;
    bool digitalInput2(size_t axis) const;
    /// Sync error flag (object 0x1C32:32 inside the axis status PDO).
    bool syncError(size_t axis) const;

    /// Raw MOT status word (16 bit).
    uint16_t motStatusWord(size_t axis) const;

    // -- Optional encoder channel (EL7342-class devices) --------------------------------

    /// True when axis carries an ENC object pair (counter/latch).
    bool hasEncoder(size_t axis) const;

    /// ENC counter value (object 0x60xx:0x11, 16/32 bit signed).
    int32_t counterValue(size_t axis) const;
    /// ENC latch value (object 0x60xx:0x12, 16/32 bit signed).
    int32_t latchValue(size_t axis) const;
    /// ENC Status :02 — external latch captured a value.
    bool latchValid(size_t axis) const;
    /// ENC Status :03 — a set-counter command completed.
    bool setCounterDone(size_t axis) const;
    /// ENC Status :04/:05 — counter wrapped.
    bool counterUnderflow(size_t axis) const;
    bool counterOverflow(size_t axis) const;

    /// ENC set-counter preset value (object 0x70xx:0x11).
    void setCounterValue(size_t axis, int32_t v);
    /// ENC Control :03 — "Set counter" command bit.
    void setSetCounter(size_t axis, bool on);
    /// ENC Control :02/:04 — external latch on positive/negative edge.
    void setLatchExtPos(size_t axis, bool on);
    void setLatchExtNeg(size_t axis, bool on);

    /// Raw ENC status word (16 bit), 0 when no encoder.
    uint16_t encStatusWord(size_t axis) const;

private:
    static constexpr uint32_t kInvalid = ~0u;

    /// One resolved field — absolute bit offset + width (0 = absent).
    struct Field {
        uint32_t bit_off = kInvalid;
        uint8_t  bit_len = 0;
    };

    /// Resolved per-axis layout.
    struct Axis {
        // motor control (SM2)
        uint16_t mot_obj        = 0;        ///< 0x70xx motor object index
        uint32_t off_enable     = kInvalid; ///< :01
        uint32_t off_reset      = kInvalid; ///< :02
        uint32_t off_rtq        = kInvalid; ///< :03
        Field    velocity;                  ///< :21 (16b)
        // encoder control (SM2, optional)
        uint16_t enc_obj        = 0;        ///< 0x70xx encoder object index
        uint32_t off_latch_pos  = kInvalid; ///< :02
        uint32_t off_set_counter= kInvalid; ///< :03
        uint32_t off_latch_neg  = kInvalid; ///< :04
        Field    set_counter_val;           ///< :11 (16/32b)
        // motor status (SM3)
        uint16_t stat_obj       = 0;        ///< mot_obj - 0x1000
        uint32_t off_status     = kInvalid; ///< first status bit
        uint32_t off_sync_err   = kInvalid; ///< 0x1C32:32 inside PDO
        // encoder status (SM3, optional)
        uint16_t enc_stat_obj   = 0;        ///< enc_obj - 0x1000
        uint32_t off_enc_status = kInvalid; ///< first ENC status bit
        Field    counter;                   ///< :11 (16/32b)
        Field    latch;                     ///< :12 (16/32b)
    };

    bool resolveLayout();
    Result<> prepare(PDO::PDOAddressMode mode);

    // -- bit access helpers -------------------------------------------------------------

    bool    inBit(uint32_t bit_off) const;
    void    setOutBit(uint32_t bit_off, bool v);
    int64_t inSigned64(uint32_t bit_off, uint8_t bits) const;
    int32_t inSigned(uint32_t bit_off, uint8_t bits) const;
    void    outSigned(uint32_t bit_off, uint8_t bits, int32_t v);

    const Axis* axisAt(size_t axis) const {
        return axis < axes_.size() ? &axes_[axis] : nullptr;
    }
    Axis* axisAt(size_t axis) {
        return axis < axes_.size() ? &axes_[axis] : nullptr;
    }

    std::vector<Axis> axes_;
};

} // namespace Beckhoff

} // namespace EtherCAT

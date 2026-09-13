/**
 * @file StepperTerminal.hpp
 * @brief Driver for Beckhoff EL703x/EL704x stepper terminals (the "POS"
 *        interface)
 *
 * The EL7031/EL7037/EL7041/EL7047 stepper-motor terminals share one
 * default process image:
 *
 *   SM2 outputs (8 B) — PDOs 0x1600+0x1602+0x1604:
 *     ENC control byte (latch-enable / set-counter bits),
 *     set-counter value (16/32 bit),
 *     STM control byte (Enable / Reset / Reduce torque),
 *     STM velocity command (16 bit).
 *
 *   SM3 inputs (8 B) — PDOs 0x1A00+0x1A03:
 *     ENC status word + counter value + latch value,
 *     STM status word (ready/warning/error/moving/digital inputs).
 *
 * Field offsets are resolved from the SII PDO entries by object index +
 * subindex — no hard-coded byte offsets — so the compact and extended
 * PDO variants both work.  Operating-mode selection (velocity /
 * positioning / direct-step) is configured over CoE/SDO by the
 * application; this driver covers the cyclic part.
 *
 * @code
 *   auto stm = StepperTerminal::findFirst(master, Devices::EL7031);
 *   stm->start();
 *   stm->setEnable(true);
 *   stm->setVelocity(500);              // application units / cycle
 *   while (running) {
 *       int32_t pos = stm->counterValue();
 *   }
 * @endcode
 */

#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <span>

#include "tether/Beckhoff/TerminalBase.hpp"
#include "tether/ethercat/PDOManager.hpp"
#include "tether/ethercat/SlaveDiscoveryManager.hpp"
#include "tether/ethercat/Types.hpp"

namespace EtherCAT {

class Master;

namespace Beckhoff {

// ============================================================================
// Known POS-interface stepper terminals (from Beckhoff EL7xxx.xml)
// ============================================================================
// All revisions of these product codes default to the same SM2/SM3
// assignment (ENC control compact + STM control + STM velocity out,
// ENC status compact + STM status in).  None verified on hardware yet —
// supported, not verified yet.

namespace Devices {

inline constexpr DeviceIdentity EL7031{0x00000002, 0x1B773052, 1, "EL7031"};
inline constexpr DeviceIdentity EL7037{0x00000002, 0x1B7D3052, 1, "EL7037"};
inline constexpr DeviceIdentity EL7041{0x00000002, 0x1B813052, 1, "EL7041"};
inline constexpr DeviceIdentity EL7047{0x00000002, 0x1B873052, 1, "EL7047"};

// EP/ER/EJ variants — same POS-interface electronics.
inline constexpr DeviceIdentity EJ7031{0x00000002, 0x1B772852, 1, "EJ7031"};
inline constexpr DeviceIdentity EJ7037{0x00000002, 0x1B7D2852, 1, "EJ7037"};
inline constexpr DeviceIdentity EJ7041{0x00000002, 0x1B812852, 1, "EJ7041"};
inline constexpr DeviceIdentity EJ7047{0x00000002, 0x1B872852, 1, "EJ7047"};
inline constexpr DeviceIdentity EP7041{0x00000002, 0x1B814052, 1, "EP7041"};
inline constexpr DeviceIdentity ER7041{0x00000002, 0x1B814852, 1, "ER7041"};

/// Every known POS-interface stepper terminal.
inline constexpr std::array kStepperTerminals{
    EL7031, EL7037, EL7041, EL7047,
    EJ7031, EJ7037, EJ7041, EJ7047, EP7041, ER7041,
};

} // namespace Devices

// ============================================================================
// StepperTerminal
// ============================================================================

class StepperTerminal : public TerminalBase {
public:
    // -- Construction / factories ----------------------------------------------

    StepperTerminal(Master& master, uint16_t slave_index,
                    const DeviceIdentity& identity);
    StepperTerminal(Master& master, const DiscoveredSlave& slave,
                    const DeviceIdentity& identity);

    ~StepperTerminal() override;

    StepperTerminal(StepperTerminal&&) noexcept            = default;
    StepperTerminal& operator=(StepperTerminal&&) noexcept = default;
    StepperTerminal(const StepperTerminal&)                = delete;
    StepperTerminal& operator=(const StepperTerminal&)     = delete;

    /// True when `s` carries the given vendor/product identity.
    static bool matches(const DiscoveredSlave& s, const DeviceIdentity& id) {
        return s.hasVendorAndProduct(id.vendor_id, id.product_code);
    }

    /// Scan the bus for the first slave matching `identity`.
    static Result<StepperTerminal> findFirst(Master& master,
                                             const DeviceIdentity& identity);
    static Result<StepperTerminal> findFirst(
        Master& master, const DeviceIdentity& identity,
        std::span<const DiscoveredSlave> scan);

    // -- Bring-up ------------------------------------------------------------------

    /// Standalone bring-up to SAFE-OP (position addressing).
    Result<> configure();

    /// configure() then OP (+ managed realtime loop by default).
    Result<> start() { return start(StartOptions()); }
    Result<> start(const StartOptions& opts);

    /// Chained-operation contract (shared logical address space):
    /// same semantics as the other terminal drivers.
    Result<> prepareForLogicalExchange();
    Result<> mapLogicalAndEnterSafeOp();
    Result<> requestOp(int timeout_ms);

    // -- Drive control (SM2 image) ---------------------------------------------------

    /// STM Control :01 — power-stage enable.
    void setEnable(bool on);
    /// STM Control :02 — fault reset (write 1, then clear).
    void setReset(bool on);
    /// STM Control :03 — reduced motor torque.
    void setReduceTorque(bool on);

    /**
     * @brief STM velocity command (object 0x7010:33, 16 bit).
     * Application units depend on the terminal's SDO configuration.
     */
    void setVelocity(int16_t v);

    /**
     * @brief STM target position (object 0x7010:17, 32 bit) — only valid
     *        when the terminal's PDO assignment maps it (not part of the
     *        default image; add PDO 0x1603 via SDO).  hasTargetPosition()
     *        reports whether the field exists.
     */
    void setTargetPosition(int32_t pos);
    bool hasTargetPosition() const { return fld_pos_target_.bit_len != 0; }

    /**
     * @brief ENC set-counter preset value (object 0x7000:17, 16 or
     *        32 bit).  Latched into the counter while setCounter(true).
     */
    void setCounterValue(int32_t v);
    /// ENC Control :03 — "Set counter" command bit.
    void setSetCounter(bool on);
    /// ENC Control :02 — enable external latch on positive edge.
    void setLatchExtPos(bool on);
    /// ENC Control :04 — enable external latch on negative edge.
    void setLatchExtNeg(bool on);
    /// ENC Control :01 — enable latch C (present on EL7041/EL7047 only).
    void setLatchC(bool on);
    bool hasLatchC() const { return kInvalid != off_enc_latch_c_; }

    /// Raw STM control word (16 bit) — for bits not wrapped above.
    uint16_t stmControlWord() const;
    /// Raw ENC control word (16 bit).
    uint16_t encControlWord() const;

    // -- Drive status (SM3 image) ------------------------------------------------------

    /// STM Status :01 — ready to enable.
    bool readyToEnable() const;
    /// STM Status :02 — ready (power stage active).
    bool ready() const;
    /// STM Status :03 — warning present.
    bool warning() const;
    /// STM Status :04 — error present (reset via setReset()).
    bool error() const;
    /// STM Status :05/:06 — currently moving in +/- direction.
    bool movingPositive() const;
    bool movingNegative() const;
    /// STM Status :07 — running at reduced torque.
    bool torqueReduced() const;
    /// STM Status :12/:13 — digital inputs 1/2.
    bool digitalInput1() const;
    bool digitalInput2() const;

    /// ENC Status :02 — external latch captured a value.
    bool latchValid() const;
    /// ENC Status :03 — a setCounter() command completed.
    bool setCounterDone() const;
    /// ENC Status :04/:05 — counter wrapped.
    bool counterUnderflow() const;
    bool counterOverflow() const;
    /// Sync error flag (object 0x1C32:32, DC-related).
    bool syncError() const;

    /// ENC counter value (object 0x6000:17, 16 or 32 bit, signed).
    int32_t counterValue() const;
    /// ENC latch value (object 0x6000:18, 16 or 32 bit, signed).
    int32_t latchValue() const;

    /// Raw STM status word (16 bit).
    uint16_t stmStatusWord() const;
    /// Raw ENC status word (16 bit).
    uint16_t encStatusWord() const;

private:
    static constexpr uint32_t kInvalid = ~0u;

    /// One resolved field — absolute bit offset in its process image +
    /// width.  bit_len == 0 marks "not present".
    struct Field {
        uint32_t bit_off = kInvalid;
        uint8_t  bit_len = 0;
    };

    bool resolveLayout();
    Result<> prepare(PDO::PDOAddressMode mode);

    // -- bit access helpers -------------------------------------------------------------

    bool    inBit(uint32_t bit_off) const;
    void    setOutBit(uint32_t bit_off, bool v);
    int32_t inSigned(uint32_t bit_off, uint8_t bits) const;
    int64_t inSigned64(uint32_t bit_off, uint8_t bits) const;
    void    outSigned(uint32_t bit_off, uint8_t bits, int32_t v);

    // -- resolved layout (all into the process images) ------------------------------------

    // SM2 (outputs)
    uint32_t off_enc_ctrl_    = kInvalid;   // first 0x7000 bit (packed byte)
    uint32_t off_enc_latch_c_ = kInvalid;   // 0x7000:01 (absent on EL703x)
    uint32_t off_set_counter_ = kInvalid;   // 0x7000:03
    uint32_t off_latch_pos_   = kInvalid;   // 0x7000:02
    uint32_t off_latch_neg_   = kInvalid;   // 0x7000:04
    Field    fld_set_counter_val_;          // 0x7000:17 (16/32b)
    uint32_t off_stm_enable_  = kInvalid;   // 0x7010:01
    uint32_t off_stm_reset_   = kInvalid;   // 0x7010:02
    uint32_t off_stm_rtq_     = kInvalid;   // 0x7010:03
    Field    fld_velocity_;                 // 0x7010:33 (16b)
    Field    fld_pos_target_;               // 0x7010:17 (32b, optional)

    // SM3 (inputs)
    uint32_t off_enc_status_  = kInvalid;   // first 0x6000 bit (packed word)
    uint32_t off_latch_valid_ = kInvalid;   // 0x6000:02
    uint32_t off_set_done_    = kInvalid;   // 0x6000:03
    uint32_t off_underflow_   = kInvalid;   // 0x6000:04
    uint32_t off_overflow_    = kInvalid;   // 0x6000:05
    Field    fld_counter_;                  // 0x6000:17 (16/32b)
    Field    fld_latch_;                    // 0x6000:18 (16/32b)
    uint32_t off_stm_status_  = kInvalid;   // first 0x6010 bit (packed word)
    uint32_t off_sync_err_    = kInvalid;   // 0x1C32:32
};

} // namespace Beckhoff

} // namespace EtherCAT

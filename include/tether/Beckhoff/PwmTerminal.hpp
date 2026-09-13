/**
 * @file PwmTerminal.hpp
 * @brief Driver for Beckhoff PWM output terminals (EL2502, EL2535, EL2564)
 *
 * PWM channels are resolved from the SII RxPDO list: each channel owns
 * a 16-bit duty value on object 0x7000+0x10*n subindex 0x11 ("PWM
 * output") plus optional control bits:
 *
 *   :01 enable dithering   :06 enable   :07 reset        (EL2535)
 *
 * and a status word on 0x6000+0x10*n:
 *
 *   :01 digital input      :06 warning  :07 error  :16 TxPDO toggle
 *
 * EL2564 additionally maps a device-wide master gain (0xF719:17).
 *
 * The raw duty range is 0..0x7FFF (0..100%); setDuty() clamps and
 * setDutyFraction() scales 0.0..1.0 onto the configured full scale.
 *
 * @code
 *   auto pwm = PwmTerminal::findFirst(master, Devices::EL2502);
 *   pwm->start();
 *   pwm->setDutyFraction(0, 0.25);   // channel 1 at 25 %
 * @endcode
 */

#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
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
// Known PWM terminals (from Beckhoff EL25xx.xml)
// ============================================================================
// num_bits holds the channel count.  None verified on hardware yet —
// supported, not verified yet.

namespace Devices {

inline constexpr DeviceIdentity EL2502{0x00000002, 0x09C63052, 2, "EL2502"};
inline constexpr DeviceIdentity EL2535{0x00000002, 0x09E73052, 2, "EL2535"};
inline constexpr DeviceIdentity EL2564{0x00000002, 0x0A043052, 4, "EL2564"};

// EJ/EP variants — same PWM electronics.
inline constexpr DeviceIdentity EJ2502{0x00000002, 0x09C62852, 2, "EJ2502"};
inline constexpr DeviceIdentity EJ2564{0x00000002, 0x0A042852, 4, "EJ2564"};
inline constexpr DeviceIdentity EP2534{0x00000002, 0x09E64052, 2, "EP2534"};

/// Every known PWM output terminal.
inline constexpr std::array kPwmTerminals{
    EL2502, EL2535, EL2564, EJ2502, EJ2564, EP2534,
};

} // namespace Devices

// ============================================================================
// PwmTerminal
// ============================================================================

class PwmTerminal : public TerminalBase {
public:
    /// Raw duty full-scale (0..0x7FFF = 0..100 %).
    static constexpr uint16_t kDutyFullScale = 0x7FFF;
    static constexpr size_t kMaxChannels = 8;

    // -- Construction / factories ------------------------------------------------

    PwmTerminal(Master& master, uint16_t slave_index,
                const DeviceIdentity& identity);
    PwmTerminal(Master& master, const DiscoveredSlave& slave,
                const DeviceIdentity& identity);

    ~PwmTerminal() override;

    PwmTerminal(PwmTerminal&&) noexcept            = default;
    PwmTerminal& operator=(PwmTerminal&&) noexcept = default;
    PwmTerminal(const PwmTerminal&)                = delete;
    PwmTerminal& operator=(const PwmTerminal&)     = delete;

    /// True when `s` carries the given vendor/product identity.
    static bool matches(const DiscoveredSlave& s, const DeviceIdentity& id) {
        return s.hasVendorAndProduct(id.vendor_id, id.product_code);
    }

    /// Scan the bus for the first slave matching `identity`.
    static Result<PwmTerminal> findFirst(Master& master,
                                         const DeviceIdentity& identity);
    static Result<PwmTerminal> findFirst(
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

    // -- Channels --------------------------------------------------------------------------

    /// Number of PWM channels resolved from the SII layout.
    size_t channelCount() const { return channels_.size(); }

    // -- Duty cycle -------------------------------------------------------------------------

    /// Raw duty value, clamped to [0, kDutyFullScale].
    void setDuty(size_t ch, uint16_t raw);
    /// Scaled duty: 0.0 = off, 1.0 = full scale.
    void setDutyFraction(size_t ch, float fraction);
    /// Current raw duty value from the output image.
    uint16_t duty(size_t ch) const;
    /// Ramp all channels to zero (leaves enable bits untouched).
    void allOff();

    // -- Control bits (EL2535-class; no-ops when the device doesn't map them) ------------------

    void setEnable(size_t ch, bool on);          ///< :06
    void setReset(size_t ch, bool on);           ///< :07
    void setEnableDithering(size_t ch, bool on); ///< :01
    bool hasEnable(size_t ch) const;
    bool hasDithering(size_t ch) const;

    /// Device-wide master gain (EL2564 only; 0xF719:17).
    void setMasterGain(uint16_t gain);
    bool hasMasterGain() const { return master_gain_.present(); }

    // -- Status --------------------------------------------------------------------------------

    bool warning(size_t ch) const;              ///< 0x6xxx:06
    bool error(size_t ch) const;                ///< 0x6xxx:07
    bool digitalInput(size_t ch) const;         ///< 0x6xxx:01 (EL2535)
    bool hasStatus(size_t ch) const;

private:
    static constexpr uint32_t kInvalid = ~0u;

    struct Field {
        uint32_t bit_off = kInvalid;
        uint8_t  bit_len = 0;
        bool present() const { return bit_off != kInvalid && bit_len != 0; }
    };

    struct Channel {
        Field duty;              // 0x70xx:17, 16 bit
        uint32_t enable   = kInvalid;
        uint32_t reset    = kInvalid;
        uint32_t dither   = kInvalid;
        uint32_t warning  = kInvalid;
        uint32_t err      = kInvalid;
        uint32_t din      = kInvalid;
    };

    bool resolveLayout();
    Result<> prepare(PDO::PDOAddressMode mode);

    bool inBit(uint32_t bit_off) const;
    void setOutBit(uint32_t bit_off, bool v);

    std::vector<Channel> channels_;
    Field master_gain_;
};

} // namespace Beckhoff

} // namespace EtherCAT

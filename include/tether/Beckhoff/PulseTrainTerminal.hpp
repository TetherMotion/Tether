/**
 * @file PulseTrainTerminal.hpp
 * @brief Driver for Beckhoff pulse-train (PTO) terminals
 *        (EL2521 PTO variants, EL2522)
 *
 * The PTO interface generates step pulses for external stepper drivers.
 * Each channel is resolved from the SII PDO list by (index, subindex):
 *
 *   PTO control (object 0x7000+0x10*ch):
 *     :01 Frequency select    :02 Disable ramp      :03 Go counter
 *     :04 Automatic direction :05 Forward           :06 Reward
 *     :17 Frequency value (16b)  :18 Target counter value (16/32b)
 *
 *   PTO status (object 0x6000+0x10*ch):
 *     :01 Sel. ack / end counter  :02 Ramp active   :07 Error
 *     :14 Sync error
 *
 *   ENC control/status (counter readback, per device: 0x7010+0x10*ch
 *   on EL2521-0124, 0x7020+0x10*ch on EL2522):
 *     ctrl: :03 Set counter, :17 Set counter value
 *     stat: :03 Set counter done, :04/:05 under/overflow,
 *           :17 Counter value, :18 Latch value
 *
 * Early EL2521 revisions map a plain 16-bit Ctrl + 16-bit Data word
 * instead of the PTO fields — rawCtrl()/rawData() expose them; the PTO
 * helpers report presence via has*().
 *
 * @code
 *   auto pto = PulseTrainTerminal::findFirst(master, Devices::EL2522);
 *   pto->start();
 *   pto->setFrequency(0, 5000);        // pulses/s in device units
 *   pto->setGoCounter(0, true);        // run toward the target count
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
// Known pulse-train terminals (from Beckhoff EL25xx.xml)
// ============================================================================
// EL2521 revisions -0024/-0124 map the PTO interface; early revisions
// expose only the raw Ctrl/Data pair.  num_bits holds the channel count.
// None verified on hardware yet — supported, not verified yet.

namespace Devices {

inline constexpr DeviceIdentity EL2521{0x00000002, 0x09D93052, 1, "EL2521"};
inline constexpr DeviceIdentity EL2522{0x00000002, 0x09DA3052, 2, "EL2522"};

// EJ variants — same PTO electronics.
inline constexpr DeviceIdentity EJ2521{0x00000002, 0x09D92852, 1, "EJ2521"};
inline constexpr DeviceIdentity EJ2522{0x00000002, 0x09DA2852, 2, "EJ2522"};

/// Every known pulse-train terminal.
inline constexpr std::array kPulseTrainTerminals{
    EL2521, EL2522, EJ2521, EJ2522,
};

} // namespace Devices

// ============================================================================
// PulseTrainTerminal
// ============================================================================

class PulseTrainTerminal : public TerminalBase {
public:
    static constexpr size_t kMaxChannels = 4;

    // -- Construction / factories ------------------------------------------------

    PulseTrainTerminal(Master& master, uint16_t slave_index,
                       const DeviceIdentity& identity);
    PulseTrainTerminal(Master& master, const DiscoveredSlave& slave,
                       const DeviceIdentity& identity);

    ~PulseTrainTerminal() override;

    PulseTrainTerminal(PulseTrainTerminal&&) noexcept            = default;
    PulseTrainTerminal& operator=(PulseTrainTerminal&&) noexcept = default;
    PulseTrainTerminal(const PulseTrainTerminal&)                = delete;
    PulseTrainTerminal& operator=(const PulseTrainTerminal&)     = delete;

    /// True when `s` carries the given vendor/product identity.
    static bool matches(const DiscoveredSlave& s, const DeviceIdentity& id) {
        return s.hasVendorAndProduct(id.vendor_id, id.product_code);
    }

    /// Scan the bus for the first slave matching `identity`.
    static Result<PulseTrainTerminal> findFirst(
        Master& master, const DeviceIdentity& identity);
    static Result<PulseTrainTerminal> findFirst(
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

    size_t channelCount() const { return channels_.size(); }

    // -- PTO control -------------------------------------------------------------------------

    /// Frequency value (0x70xx:17, 16 bit).  Units per the terminal's
    /// SDO frequency scaling (default: pulses per second).
    void setFrequency(size_t ch, uint16_t freq);
    uint16_t frequency(size_t ch) const;
    bool hasFrequency(size_t ch) const;

    void setFrequencySelect(size_t ch, bool on);   ///< :01
    void setDisableRamp(size_t ch, bool on);       ///< :02
    void setGoCounter(size_t ch, bool on);         ///< :03
    void setAutoDirection(size_t ch, bool on);     ///< :04
    void setForward(size_t ch, bool on);           ///< :05
    void setReverse(size_t ch, bool on);           ///< :06

    /// Target counter value (0x70xx:18, 16 or 32 bit) — travel distance
    /// for "go counter" mode.
    void setTarget(size_t ch, int32_t target);
    bool hasTarget(size_t ch) const;

    /// Legacy 16-bit Ctrl / Data words (early EL2521 revisions).
    uint16_t rawCtrl(size_t ch) const;
    void setRawCtrl(size_t ch, uint16_t w);
    uint16_t rawData(size_t ch) const;
    void setRawData(size_t ch, uint16_t w);
    bool hasRawInterface(size_t ch) const;

    // -- ENC control ----------------------------------------------------------------------------

    void setEncSetCounter(size_t ch, bool on);     ///< ENC ctrl :03
    void setEncCounterValue(size_t ch, int32_t v); ///< ENC ctrl :17
    bool hasEnc(size_t ch) const;

    // -- Status ------------------------------------------------------------------------------------

    bool selAck(size_t ch) const;        ///< PTO status :01
    bool rampActive(size_t ch) const;    ///< PTO status :02
    bool error(size_t ch) const;         ///< PTO status :07
    bool syncError(size_t ch) const;     ///< PTO status :14

    bool setCounterDone(size_t ch) const;
    bool counterUnderflow(size_t ch) const;
    bool counterOverflow(size_t ch) const;
    int32_t counterValue(size_t ch) const;
    int32_t latchValue(size_t ch) const;

private:
    static constexpr uint32_t kInvalid = ~0u;

    struct Field {
        uint32_t bit_off = kInvalid;
        uint8_t  bit_len = 0;
        bool present() const { return bit_off != kInvalid && bit_len != 0; }
    };

    struct Channel {
        // PTO control (SM2)
        uint32_t freq_sel    = kInvalid;
        uint32_t dis_ramp    = kInvalid;
        uint32_t go_counter  = kInvalid;
        uint32_t auto_dir    = kInvalid;
        uint32_t fwd         = kInvalid;
        uint32_t rev         = kInvalid;
        Field    freq;
        Field    target;
        Field    raw_ctrl;   // legacy EL2521 Ctrl word
        Field    raw_data;   // legacy EL2521 Data word
        // ENC control (SM2)
        uint32_t enc_set_cnt = kInvalid;
        Field    enc_set_val;
        // PTO status (SM3)
        uint32_t sel_ack     = kInvalid;
        uint32_t ramp_active = kInvalid;
        uint32_t err         = kInvalid;
        uint32_t sync_err    = kInvalid;
        // ENC status (SM3)
        uint32_t set_done    = kInvalid;
        uint32_t underflow   = kInvalid;
        uint32_t overflow    = kInvalid;
        Field    counter;
        Field    latch;
    };

    bool resolveLayout();
    Result<> prepare(PDO::PDOAddressMode mode);

    bool inBit(uint32_t bit_off) const;
    void setOutBit(uint32_t bit_off, bool v);
    int64_t inSigned(const Field& f) const;
    void outSigned(const Field& f, int32_t v);
    uint16_t inWord(const Field& f) const;
    void outWord(const Field& f, uint16_t v);

    std::vector<Channel> channels_;
};

} // namespace Beckhoff

} // namespace EtherCAT

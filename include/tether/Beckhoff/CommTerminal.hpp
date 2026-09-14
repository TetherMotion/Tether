/**
 * @file CommTerminal.hpp
 * @brief Generic driver for Beckhoff byte-FIFO communication terminals
 *        (EL6xxx serial / IO-Link / memory / display)
 *
 * Communication terminals exchange byte streams through process data:
 * each channel maps a control/status register plus a block of 8-bit
 * data entries:
 *
 *   SM2 (outputs), per channel PDO:
 *     Ctrl word   (8 or 16 bit) — handshake bits, byte count, toggles
 *     Data Out 0..N (8 bit)     — transmit FIFO image
 *
 *   SM3 (inputs), per channel PDO:
 *     Status word (8 or 16 bit) — handshake bits, byte count, toggles
 *     Data In 0..N  (8 bit)     — receive FIFO image
 *
 * Family examples: EL6001/EL6002/EL6021/EL6022 (RS232/RS485),
 * EL6080 (memory), EL6090 (display), EL6201 (AS-i), EL6224/EP6224/
 * EJ6224 (IO-Link — channel PDOs are SDO-assigned, default image
 * carries per-channel state bytes), EL6233 (PROFINET-RT end).
 *
 * Channel pairing is by PDO order: the i-th input FIFO PDO pairs with
 * the i-th output FIFO PDO.  All offsets resolve from the SII entries;
 * the generic ctrl/status + data view stays usable for devices whose
 * handshake semantics differ.
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

namespace EtherCAT {

class Master;

namespace Beckhoff {

// ============================================================================
// Known communication terminals (from the Beckhoff ESIs)
// ============================================================================
// num_bits is informational here — the channel count is resolved from
// the PDO layout.  All ESI-verified for shape; none verified on
// hardware yet — supported, not verified yet.

namespace Devices {

// Serial interfaces (RS232 / RS422/485, TTY).
inline constexpr DeviceIdentity EL6001{0x00000002, 0x17713052, 1, "EL6001"};
inline constexpr DeviceIdentity EL6002{0x00000002, 0x17723052, 2, "EL6002"};
inline constexpr DeviceIdentity EL6021{0x00000002, 0x17853052, 1, "EL6021"};
inline constexpr DeviceIdentity EL6022{0x00000002, 0x17863052, 2, "EL6022"};

// Memory / display / license.
inline constexpr DeviceIdentity EL6070{0x00000002, 0x17B63052, 1, "EL6070"};
inline constexpr DeviceIdentity EL6080{0x00000002, 0x17C03052, 1, "EL6080"};
inline constexpr DeviceIdentity EL6090{0x00000002, 0x17CA3052, 1, "EL6090"};

// Fieldbus-on-terminal protocols.
inline constexpr DeviceIdentity EL6201{0x00000002, 0x18393052, 1, "EL6201"};
inline constexpr DeviceIdentity EL6224{0x00000002, 0x18503052, 4, "EL6224"};
inline constexpr DeviceIdentity EL6233{0x00000002, 0x18593052, 1, "EL6233"};
inline constexpr DeviceIdentity ELX6233{0x00000002, 0x970C8399, 1, "ELX6233"};

// EJ / EP / ER / EPP variants.
inline constexpr DeviceIdentity EJ6002{0x00000002, 0x17722852, 2, "EJ6002"};
inline constexpr DeviceIdentity EJ6070{0x00000002, 0x17B62852, 1, "EJ6070"};
inline constexpr DeviceIdentity EJ6080{0x00000002, 0x17C02852, 1, "EJ6080"};
inline constexpr DeviceIdentity EJ6224{0x00000002, 0x18502852, 4, "EJ6224"};
inline constexpr DeviceIdentity EP6001{0x00000002, 0x17714052, 1, "EP6001"};
inline constexpr DeviceIdentity EP6002{0x00000002, 0x17724052, 2, "EP6002"};
inline constexpr DeviceIdentity EP6070{0x00000002, 0x17B64052, 1, "EP6070"};
inline constexpr DeviceIdentity EP6080{0x00000002, 0x17C04052, 1, "EP6080"};
inline constexpr DeviceIdentity EP6090{0x00000002, 0x17CA4052, 1, "EP6090"};
inline constexpr DeviceIdentity EP6222{0x00000002, 0x184E4052, 2, "EP6222"};
inline constexpr DeviceIdentity EP6224{0x00000002, 0x18504052, 4, "EP6224"};
inline constexpr DeviceIdentity ER6001{0x00000002, 0x17714852, 1, "ER6001"};
inline constexpr DeviceIdentity ER6002{0x00000002, 0x17724852, 2, "ER6002"};
inline constexpr DeviceIdentity ERP6224{0x00000002, 0x64F63F09, 4, "ERP6224"};
inline constexpr DeviceIdentity EPP6001{0x00000002, 0x64773D19, 1, "EPP6001"};
inline constexpr DeviceIdentity EPP6002{0x00000002, 0x64773D29, 2, "EPP6002"};
inline constexpr DeviceIdentity EPP6090{0x00000002, 0x647742A9, 1, "EPP6090"};
inline constexpr DeviceIdentity EPP6224{0x00000002, 0x64774C09, 4, "EPP6224"};
inline constexpr DeviceIdentity EPP6228{0x00000002, 0x64774C69, 8, "EPP6228"};

/// Every known communication terminal.
inline constexpr std::array kCommTerminals{
    EL6001, EL6002, EL6021, EL6022, EL6070, EL6080, EL6090,
    EL6201, EL6224, EL6233, ELX6233,
    EJ6002, EJ6070, EJ6080, EJ6224,
    EP6001, EP6002, EP6070, EP6080, EP6090, EP6222, EP6224,
    ER6001, ER6002, ERP6224,
    EPP6001, EPP6002, EPP6090, EPP6224, EPP6228,
};

} // namespace Devices

// ============================================================================
// CommTerminal
// ============================================================================

class CommTerminal : public TerminalBase {
public:
    static constexpr size_t kMaxChannels   = 8;
    static constexpr size_t kMaxFifoBytes  = 256;

    // -- Construction / factories ----------------------------------------------

    CommTerminal(Master& master, uint16_t slave_index,
                 const DeviceIdentity& identity);
    CommTerminal(Master& master, const DiscoveredSlave& slave,
                 const DeviceIdentity& identity);

    ~CommTerminal() override;

    CommTerminal(CommTerminal&&) noexcept            = default;
    CommTerminal& operator=(CommTerminal&&) noexcept = default;
    CommTerminal(const CommTerminal&)                = delete;
    CommTerminal& operator=(const CommTerminal&)     = delete;

    /// True when `s` carries the given vendor/product identity.
    static bool matches(const DiscoveredSlave& s, const DeviceIdentity& id) {
        return s.hasVendorAndProduct(id.vendor_id, id.product_code);
    }

    /// Scan the bus for the first slave matching `identity`.
    static Result<CommTerminal> findFirst(Master& master,
                                          const DeviceIdentity& identity);
    static Result<CommTerminal> findFirst(
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

    // -- Channels ----------------------------------------------------------------------

    /// Number of FIFO channel pairs resolved (0 before prepare()).
    size_t channels() const { return chans_.size(); }

    /// Data capacity of the transmit/receive FIFO images, in bytes.
    size_t txCapacity(size_t ch) const;
    size_t rxCapacity(size_t ch) const;

    // -- Control / status words -----------------------------------------------------------

    /// Ctrl word of channel `ch` (8- or 16-bit field, zero-extended).
    uint16_t ctrlWord(size_t ch) const;
    /// Overwrite the channel's ctrl word.
    void setCtrlWord(size_t ch, uint16_t w);
    /// Single ctrl-word bit access (handshake toggles etc.).
    bool ctrlBit(size_t ch, uint8_t bit) const;
    void setCtrlBit(size_t ch, uint8_t bit, bool v);

    /// Status word of channel `ch`.
    uint16_t statusWord(size_t ch) const;
    bool statusBit(size_t ch, uint8_t bit) const;

    // -- FIFO data ---------------------------------------------------------------------------

    /**
     * @brief Copy `src` into the channel's transmit FIFO image.
     * @return bytes written (capped at txCapacity()).
     */
    size_t writeData(size_t ch, std::span<const uint8_t> src);

    /// Read view over the channel's receive FIFO image.
    std::span<const uint8_t> dataIn(size_t ch) const;

    /// Copy the receive FIFO image into `dst`; @return bytes copied.
    size_t readData(size_t ch, std::span<uint8_t> dst) const;

private:
    bool resolveLayout();
    Result<> prepare(PDO::PDOAddressMode mode);

    /// One resolved FIFO channel.
    struct Channel {
        uint16_t ctrl_off   = 0;    ///< byte offset in the output image
        uint8_t  ctrl_bits  = 0;
        uint16_t stat_off   = 0;    ///< byte offset in the input image
        uint8_t  stat_bits  = 0;
        uint16_t dout_off   = 0;    ///< first Data Out byte
        uint16_t dout_len   = 0;    ///< Data Out byte count
        uint16_t din_off    = 0;    ///< first Data In byte
        uint16_t din_len    = 0;
    };

    std::vector<Channel> chans_;
};

} // namespace Beckhoff

} // namespace EtherCAT

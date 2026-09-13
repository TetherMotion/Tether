/**
 * @file EL2004.hpp
 * @brief Easy-to-use driver for the Beckhoff EL2004 4-channel digital output terminal
 *
 * The EL2004 (vendor 0x00000002, product 0x07D43052) is a mailbox-less,
 * output-only EtherCAT terminal: four 24V/0.5A channels packed as four bits
 * into a single process-data byte on sync-manager channel 0.  It is one
 * member of the generic packed-output family implemented by OutputTerminal —
 * this class just binds the EL2004 identity and adds a std::bitset-flavoured
 * channel API.
 *
 * @code
 *   // Simplest possible usage — find and drive the first EL2004 on the bus:
 *   auto el = EL2004::findFirst(master);
 *   if (!el) { ... el.error() ... }
 *   el->start();                    // PRE-OP -> SAFE-OP -> OP (+ RT loop)
 *   el->set(0, true);               // channel 1 ON  (0-based channel index!)
 *   el->setChannels(0b0101);        // channels 1+3 ON
 *   el->allOff();
 * @endcode
 *
 * Output setters only touch the registered PDO buffer — the master's cyclic
 * exchange publishes them on the next cycle.  They are safe to call from any
 * thread (including while the realtime loop is running); each call is a
 * lock-free atomic store.
 *
 * Channel numbering is 0-based throughout (0 = terminal marking "1").
 *
 * For chains mixing different output terminals (EL2002, EL2008, relays, ...)
 * over one shared logical address space see MultiOutputTerminal.hpp.  MultiEL2004
 * provides the EL2004-only variant.
 */

#pragma once

#include <bitset>
#include <cstddef>
#include <cstdint>
#include <span>

#include "tether/Beckhoff/OutputTerminal.hpp"

namespace EtherCAT {
namespace Beckhoff {

// ============================================================================
// EL2004 — OutputTerminal specialization
// ============================================================================

class EL2004 : public OutputTerminal {
public:
    // -- Device identity (from the Beckhoff EL2xxx ESI) ----------------------

    static constexpr uint32_t kVendorId    = Devices::EL2004.vendor_id;
    static constexpr uint32_t kProductCode = Devices::EL2004.product_code;
    static constexpr size_t   kNumChannels = Devices::EL2004.num_bits;
    static constexpr uint16_t kRxPdoIndex  = 0x1600;  ///< first of 0x1600-0x1603

    static constexpr DeviceIdentity kIdentity = Devices::EL2004;

    /// Channel bitmask type — bit N corresponds to output channel N.
    using Channels = std::bitset<kNumChannels>;

    // Keep the old type aliases working.
    using Error        = Beckhoff::Error;
    using StartOptions = Beckhoff::StartOptions;
    template <typename T = void>
    using Result = Beckhoff::Result<T>;

    static const char* errorToString(Error e) { return Beckhoff::errorToString(e); }

    // -- Construction ----------------------------------------------------------

    EL2004(Master& master, uint16_t slave_index)
        : OutputTerminal(master, slave_index, kIdentity) {}

    EL2004(Master& master, const DiscoveredSlave& slave)
        : OutputTerminal(master, slave, kIdentity) {}

    // -- Factories ---------------------------------------------------------------

    /// True when `s` carries the EL2004 vendor/product identity.
    static bool matches(const DiscoveredSlave& s) {
        return OutputTerminal::matches(s, kIdentity);
    }

    /**
     * @brief Scan the bus and return a driver bound to the first EL2004 found.
     *
     * Runs a shallow discovery (vendor + product ID only); the selected
     * slave's SII is read later by configure().
     * @return The driver, or Error::NoDeviceFound.
     */
    static Result<EL2004> findFirst(Master& master);

    /**
     * @brief Like findFirst(master) but reuses an existing discovery result
     *        instead of scanning the bus again.
     */
    static Result<EL2004> findFirst(Master& master,
                                    std::span<const DiscoveredSlave> scan);

    // -- std::bitset channel API -------------------------------------------------
    // Generic bit access (setBit/bit/setBits/bits/setOnly/allOn/allOff) is
    // inherited from OutputTerminal.

    /// Set a single channel (0-3).  Out-of-range indices are ignored.
    void set(size_t channel, bool on) { setBit(channel, on); }

    /// Current requested state of a channel (0-3); false if out of range.
    bool get(size_t channel) const { return bit(channel); }

    /// Write all four channels at once (bit N = channel N).
    void setChannels(Channels bits) { setBits(bits.to_ullong()); }

    /// Currently requested channel states (bit N = channel N).
    Channels channels() const { return Channels(bits()); }

    /// Raw output byte (only the low 4 bits are used).
    void    setRaw(uint8_t byte) { setBits(byte); }
    uint8_t raw() const { return static_cast<uint8_t>(bits()); }
};

} // namespace Beckhoff

} // namespace EtherCAT

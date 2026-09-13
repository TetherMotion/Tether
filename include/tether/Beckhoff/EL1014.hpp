/**
 * @file EL1014.hpp
 * @brief Easy-to-use driver for the Beckhoff EL1014 4-channel digital input terminal
 *
 * The EL1014 (vendor 0x00000002, product 0x03F63052) is a mailbox-less,
 * input-only EtherCAT terminal: four 24V/10µs channels packed as four bits
 * into a single process-data byte on sync-manager channel 0.  It is one
 * member of the generic packed-input family implemented by InputTerminal —
 * this class just binds the EL1014 identity and adds a std::bitset-flavoured
 * channel API.
 *
 * @code
 *   // Simplest possible usage — find and read the first EL1014 on the bus:
 *   auto el = EL1014::findFirst(master);
 *   if (!el) { ... el.error() ... }
 *   el->start();                    // PRE-OP -> SAFE-OP -> OP (+ RT loop)
 *   bool on  = el->get(0);          // channel 1 state (0-based index!)
 *   auto all = el->channels();      // std::bitset<4>
 * @endcode
 *
 * Input getters read the registered PDO buffer as last written by the
 * cyclic exchange — safe to call from any thread while the realtime loop
 * is running (lock-free atomic load).
 *
 * Channel numbering is 0-based throughout (0 = terminal marking "1").
 *
 * For chains mixing different input terminals (EL1002, EL1008, ...) over
 * one shared logical address space see MultiInputTerminal.hpp.  MultiEL1014
 * provides the EL1014-only variant.
 */

#pragma once

#include <bitset>
#include <cstddef>
#include <cstdint>
#include <span>

#include "tether/Beckhoff/InputTerminal.hpp"

namespace EtherCAT {
namespace Beckhoff {

// ============================================================================
// EL1014 — InputTerminal specialization
// ============================================================================

class EL1014 : public InputTerminal {
public:
    // -- Device identity (from the Beckhoff EL1xxx ESI) ----------------------

    static constexpr uint32_t kVendorId    = Devices::EL1014.vendor_id;
    static constexpr uint32_t kProductCode = Devices::EL1014.product_code;
    static constexpr size_t   kNumChannels = Devices::EL1014.num_bits;
    static constexpr uint16_t kTxPdoIndex  = 0x1A00;  ///< first of 0x1A00-0x1A03

    static constexpr DeviceIdentity kIdentity = Devices::EL1014;

    /// Channel bitmask type — bit N corresponds to input channel N.
    using Channels = std::bitset<kNumChannels>;

    using Error        = Beckhoff::Error;
    using StartOptions = Beckhoff::StartOptions;
    template <typename T = void>
    using Result = Beckhoff::Result<T>;

    static const char* errorToString(Error e) { return Beckhoff::errorToString(e); }

    // -- Construction ----------------------------------------------------------

    EL1014(Master& master, uint16_t slave_index)
        : InputTerminal(master, slave_index, kIdentity) {}

    EL1014(Master& master, const DiscoveredSlave& slave)
        : InputTerminal(master, slave, kIdentity) {}

    // -- Factories ---------------------------------------------------------------

    /// True when `s` carries the EL1014 vendor/product identity.
    static bool matches(const DiscoveredSlave& s) {
        return InputTerminal::matches(s, kIdentity);
    }

    /**
     * @brief Scan the bus and return a driver bound to the first EL1014 found.
     *
     * Runs a shallow discovery (vendor + product ID only); the selected
     * slave's SII is read later by configure().
     * @return The driver, or Error::NoDeviceFound.
     */
    static Result<EL1014> findFirst(Master& master);

    /**
     * @brief Like findFirst(master) but reuses an existing discovery result
     *        instead of scanning the bus again.
     */
    static Result<EL1014> findFirst(Master& master,
                                    std::span<const DiscoveredSlave> scan);

    // -- std::bitset channel API -------------------------------------------------
    // Generic bit access (bit/bits) is inherited from InputTerminal.

    /// Current state of a channel (0-3); false if out of range.
    bool get(size_t channel) const { return bit(channel); }

    /// Current channel states (bit N = channel N).
    Channels channels() const { return Channels(bits()); }

    /// Raw input byte (only the low 4 bits are used).
    uint8_t raw() const { return static_cast<uint8_t>(bits()); }
};

} // namespace Beckhoff

} // namespace EtherCAT

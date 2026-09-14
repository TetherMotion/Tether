/**
 * @file CombinedIoTerminal.hpp
 * @brief Generic driver for Beckhoff terminals with packed-bit inputs
 *        AND packed-bit outputs
 *
 * CombinedIoTerminal covers the large family of devices whose process
 * image is a set of 1-bit channels in both directions — mixed digital
 * I/O (EL1852/EL1859, EP23xx, EK18xx coupler-I/O), output terminals
 * with per-channel diagnostics (EL2032/EL2034/EL2068), relays with
 * switching counters (ELM2642/ELM2742), LED drivers (EL2595/EL2596),
 * power-distribution terminals (EL9221/EL9222/EL9227 overcurrent,
 * EP9214/EP9224) and other bit-oriented combined devices.
 *
 * Channel numbering is per direction, 0-based, in PDO entry order:
 *   input(i)       reads the i-th 1-bit TxPDO entry
 *   setOutput(i,v) writes the i-th 1-bit RxPDO entry
 * Non-bit fields inside the image (switching counters, index selects,
 * timestamps, RGBW values) are reachable through rawInput()/rawOutput()
 * and the (object, subindex) lookup helpers fieldIn()/fieldOut().
 *
 * The DeviceIdentity's num_bits packs both directions for the no-SII
 * fallback:  low byte = input channels, high byte = output channels
 * (0x0404 = 4 in + 4 out).  With SII both counts are resolved from the
 * PDO entries.
 */

#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <span>
#include <vector>

#include "tether/Beckhoff/ICombinedIoTerminal.hpp"
#include "tether/Beckhoff/TerminalBase.hpp"
#include "tether/ethercat/PDOManager.hpp"             // PDOAddressMode
#include "tether/ethercat/SlaveDiscoveryManager.hpp"  // DiscoveredSlave
#include "tether/ethercat/Types.hpp"                  // SlaveState

namespace EtherCAT {

class Master;

namespace Beckhoff {

// ============================================================================
// Known combined packed-bit devices (from the Beckhoff ESIs)
// ============================================================================
// num_bits = (out_channels << 8) | in_channels for the no-SII fallback.
// Product codes verified against the ESIs; supported, not verified on
// hardware yet.

namespace Devices {

// -- Mixed digital I/O terminals ----------------------------------------------
inline constexpr DeviceIdentity EL1852{0x00000002, 0x073C3052, 0x0808, "EL1852"};
inline constexpr DeviceIdentity EL1859{0x00000002, 0x07433052, 0x0808, "EL1859"};
inline constexpr DeviceIdentity EJ1859{0x00000002, 0x07432852, 0x0808, "EJ1859"};
inline constexpr DeviceIdentity EP1859{0x00000002, 0x07434052, 0x0808, "EP1859"};

// -- Output terminals with per-channel diagnostics -----------------------------
inline constexpr DeviceIdentity EL2014{0x00000002, 0x07DE3052, 0x0414, "EL2014"};
inline constexpr DeviceIdentity EL2032{0x00000002, 0x07F03052, 0x0202, "EL2032"};
inline constexpr DeviceIdentity EL2034{0x00000002, 0x07F23052, 0x0404, "EL2034"};
inline constexpr DeviceIdentity EL2044{0x00000002, 0x07FC3052, 0x0414, "EL2044"};
inline constexpr DeviceIdentity EL2068{0x00000002, 0x08143052, 0x0808, "EL2068"};
inline constexpr DeviceIdentity EL2212{0x00000002, 0x08A43052, 0x060E, "EL2212"};
// EL2262 is also in the OutputTerminal registry (plain-bit access); the
// C-suffixed identity exposes the timestamp fields via fieldOut().
inline constexpr DeviceIdentity EL2262C{0x00000002, 0x08D63052, 0x0200, "EL2262"};
inline constexpr DeviceIdentity EL2642{0x00000002, 0x0A523052, 0x0802, "EL2642"};
inline constexpr DeviceIdentity EL2819{0x00000002, 0x0B033052, 0x1040, "EL2819"};
inline constexpr DeviceIdentity EL2838{0x00000002, 0x0B163052, 0x0808, "EL2838"};
inline constexpr DeviceIdentity EL2869{0x00000002, 0x0B353052, 0x1010, "EL2869"};
inline constexpr DeviceIdentity EL2878{0x00000002, 0x0B3E3052, 0x0801, "EL2878"};
inline constexpr DeviceIdentity EJ2034{0x00000002, 0x07F22852, 0x0404, "EJ2034"};
inline constexpr DeviceIdentity EJ2262{0x00000002, 0x08D62852, 0x0200, "EJ2262"};

// -- ELM relay outputs with switching counters ---------------------------------
inline constexpr DeviceIdentity ELM2642{0x00000002, 0x50215729, 0x0802, "ELM2642"};
inline constexpr DeviceIdentity ELM2644{0x00000002, 0x50215749, 0x1002, "ELM2644"};
inline constexpr DeviceIdentity ELM2742{0x00000002, 0x50215D69, 0x0802, "ELM2742"};
inline constexpr DeviceIdentity ELM2744{0x00000002, 0x50215D89, 0x1002, "ELM2744"};

// -- LED drivers (bit control; brightness via CoE) -----------------------------
inline constexpr DeviceIdentity EL2595{0x00000002, 0x0A233052, 0x0306, "EL2595"};
inline constexpr DeviceIdentity EL2596{0x00000002, 0x0A243052, 0x0506, "EL2596"};
inline constexpr DeviceIdentity EL2574{0x00000002, 0x0A0E3052, 0x0308, "EL2574"};

// -- Overcurrent / power-distribution terminals ---------------------------------
inline constexpr DeviceIdentity EL9221{0x00000002, 0x24053052, 0x020A, "EL9221"};
inline constexpr DeviceIdentity EL9222{0x00000002, 0x24063052, 0x0414, "EL9222"};
inline constexpr DeviceIdentity EL9227{0x00000002, 0x240B3052, 0x0426, "EL9227"};
inline constexpr DeviceIdentity EL9562{0x00000002, 0x255A3052, 0x0203, "EL9562"};
inline constexpr DeviceIdentity EP9214{0x00000002, 0x23FE4052, 0x1232, "EP9214"};
inline constexpr DeviceIdentity EP9224{0x00000002, 0x24084052, 0x1245, "EP9224"};

// -- Couplers with integrated digital I/O --------------------------------------
inline constexpr DeviceIdentity EK1814{0x00000002, 0x07162C52, 0x0404, "EK1814"};
inline constexpr DeviceIdentity EK1818{0x00000002, 0x071A2C52, 0x0408, "EK1818"};
inline constexpr DeviceIdentity EK1828{0x00000002, 0x07242C52, 0x0804, "EK1828"};

// -- EP/ER multi-function boxes -------------------------------------------------
inline constexpr DeviceIdentity EP1518{0x00000002, 0x05EE4052, 0x041B, "EP1518"};
inline constexpr DeviceIdentity EP1839{0x00000002, 0x072F4052, 0x0844, "EP1839"};
inline constexpr DeviceIdentity EP2308{0x00000002, 0x09044052, 0x0404, "EP2308"};
inline constexpr DeviceIdentity EP2316{0x00000002, 0x090C4052, 0x0A16, "EP2316"};
inline constexpr DeviceIdentity EP2318{0x00000002, 0x090E4052, 0x0404, "EP2318"};
inline constexpr DeviceIdentity EP2328{0x00000002, 0x09184052, 0x0404, "EP2328"};
inline constexpr DeviceIdentity EP2338{0x00000002, 0x09224052, 0x0408, "EP2338"};
inline constexpr DeviceIdentity EP2839{0x00000002, 0x0B174052, 0x1005, "EP2839"};
inline constexpr DeviceIdentity ER1518{0x00000002, 0x05EE4852, 0x041B, "ER1518"};
inline constexpr DeviceIdentity ER2308{0x00000002, 0x09044852, 0x0404, "ER2308"};
inline constexpr DeviceIdentity ER2318{0x00000002, 0x090E4852, 0x0404, "ER2318"};
inline constexpr DeviceIdentity ER2328{0x00000002, 0x09184852, 0x0404, "ER2328"};
inline constexpr DeviceIdentity ER2338{0x00000002, 0x09224852, 0x0408, "ER2338"};

// -- Timestamped / XFC digital I/O ---------------------------------------------
// Input/latch bits are the packed channels; the 64-bit latch/start-time
// timestamp entries are reachable via fieldIn()/fieldOut().  These also
// appear in the InputTerminal/OutputTerminal registries for applications
// that only need the plain bits.
inline constexpr DeviceIdentity EL1252C{0x00000002, 0x04E43052, 0x0002, "EL1252"};
inline constexpr DeviceIdentity EL1254C{0x00000002, 0x04E63052, 0x0002, "EL1254"};
inline constexpr DeviceIdentity EL1258C{0x00000002, 0x04EA3052, 0x0008, "EL1258"};
inline constexpr DeviceIdentity EL1259C{0x00000002, 0x04EB3052, 0x0008, "EL1259"};
inline constexpr DeviceIdentity EL1262C{0x00000002, 0x04EE3052, 0x0002, "EL1262"};
inline constexpr DeviceIdentity EL1264C{0x00000002, 0x04F03052, 0x0004, "EL1264"};
inline constexpr DeviceIdentity EL2252C{0x00000002, 0x08D43052, 0x0200, "EL2252"};
inline constexpr DeviceIdentity EL2258C{0x00000002, 0x08DA3052, 0x0800, "EL2258"};

// -- Multi-function / specialty -------------------------------------------------
inline constexpr DeviceIdentity EL8601{0x00000002, 0x21993052, 0x0415, "EL8601"};
inline constexpr DeviceIdentity EP8601{0x00000002, 0x21994052, 0x0415, "EP8601"};
inline constexpr DeviceIdentity EL9501{0x00000002, 0x251D3052, 0x0011, "EL9501"};
inline constexpr DeviceIdentity EL9561{0x00000002, 0x25593052, 0x0011, "EL9561"};
inline constexpr DeviceIdentity ELM9410{0x00000002, 0x5022FE29, 0x0119, "ELM9410"};
inline constexpr DeviceIdentity EP4378{0x00000002, 0x111A4052, 0x0C2C, "EP4378"};

// -- EtherCAT-P (EPP) mixed boxes ---------------------------------------------------
// EPP entries are module-style ESIs (no explicit PDO list); the SII
// resolves the same packed-field layout on hardware.
inline constexpr DeviceIdentity EPP1839{0x00000002, 0x647638F9, 0x0844, "EPP1839"};
inline constexpr DeviceIdentity EPP1859{0x00000002, 0x64763A39, 0x0808, "EPP1859"};
inline constexpr DeviceIdentity EPP2308{0x00000002, 0x64765649, 0x0404, "EPP2308"};
inline constexpr DeviceIdentity EPP2316{0x00000002, 0x647656C9, 0x0A16, "EPP2316"};
inline constexpr DeviceIdentity EPP2318{0x00000002, 0x647656E9, 0x0404, "EPP2318"};
inline constexpr DeviceIdentity EPP2328{0x00000002, 0x64765789, 0x0404, "EPP2328"};
inline constexpr DeviceIdentity EPP2334{0x00000002, 0x647657E9, 0x0404, "EPP2334"};
inline constexpr DeviceIdentity EPP2338{0x00000002, 0x64765829, 0x0408, "EPP2338"};
inline constexpr DeviceIdentity EPP2339{0x00000002, 0x64765839, 0x0808, "EPP2339"};
inline constexpr DeviceIdentity EPP2349{0x00000002, 0x647658D9, 0x0808, "EPP2349"};
inline constexpr DeviceIdentity EPP2596{0x00000002, 0x64766849, 0x0506, "EPP2596"};
inline constexpr DeviceIdentity EPP2839{0x00000002, 0x64767779, 0x1005, "EPP2839"};

/// Every known combined packed-bit device.
inline constexpr std::array kCombinedIoTerminals{
    EL1852, EL1859, EJ1859, EP1859,
    EL2014, EL2032, EL2034, EL2044, EL2068, EL2212, EL2262C, EL2642,
    EL2819, EL2838, EL2869, EL2878, EJ2034, EJ2262,
    ELM2642, ELM2644, ELM2742, ELM2744,
    EL2595, EL2596, EL2574,
    EL9221, EL9222, EL9227, EL9562, EP9214, EP9224,
    EK1814, EK1818, EK1828,
    EP1518, EP1839, EP2308, EP2316, EP2318, EP2328, EP2338, EP2839,
    ER1518, ER2308, ER2318, ER2328, ER2338,
    EL1252C, EL1254C, EL1258C, EL1259C, EL1262C, EL1264C,
    EL2252C, EL2258C,
    EL8601, EP8601, EL9501, EL9561, ELM9410, EP4378,
    EPP1839, EPP1859, EPP2308, EPP2316, EPP2318, EPP2328, EPP2334,
    EPP2338, EPP2339, EPP2349, EPP2596, EPP2839,
};

} // namespace Devices

// ============================================================================
// CombinedIoTerminal — packed-bit inputs + packed-bit outputs
// ============================================================================

class CombinedIoTerminal : public TerminalBase, public ICombinedIoTerminal {
public:
    /// Maximum channels per direction.
    static constexpr size_t kMaxBits = 256;

    CombinedIoTerminal(Master& master, uint16_t slave_index,
                       const DeviceIdentity& identity);
    CombinedIoTerminal(Master& master, const DiscoveredSlave& slave,
                       const DeviceIdentity& identity);
    ~CombinedIoTerminal() override;

    CombinedIoTerminal(CombinedIoTerminal&&) noexcept            = default;
    CombinedIoTerminal& operator=(CombinedIoTerminal&&) noexcept = default;
    CombinedIoTerminal(const CombinedIoTerminal&)                = delete;
    CombinedIoTerminal& operator=(const CombinedIoTerminal&)     = delete;

    /// True when `s` carries the given vendor/product identity.
    static bool matches(const DiscoveredSlave& s, const DeviceIdentity& id) {
        return s.hasVendorAndProduct(id.vendor_id, id.product_code);
    }

    static Result<CombinedIoTerminal> findFirst(
        Master& master, const DeviceIdentity& identity);
    static Result<CombinedIoTerminal> findFirst(
        Master& master, const DeviceIdentity& identity,
        std::span<const DiscoveredSlave> scan);

    /// Standalone bring-up: SII, identity, SM registers, mailbox, PRE-OP,
    /// PDO registration (position addressing), SAFE-OP.
    Result<> configure();

    /// configure() then OP; starts the master's RT loop by default.
    Result<> start() { return start(StartOptions()); }
    Result<> start(const StartOptions& opts);

    // -- ICombinedIoTerminal --------------------------------------------------------

    uint16_t slaveIndex() const override {
        return TerminalBase::slaveIndex();
    }
    const char* deviceName() const override {
        return TerminalBase::deviceName();
    }

    /// Chained-operation contract (shared logical address space).
    Result<> prepareForLogicalExchange() override;
    Result<> mapLogicalAndEnterSafeOp() override;
    Result<> requestOp(int timeout_ms) override;

    // -- Channel counts --------------------------------------------------------

    size_t inputChannelCount()  const override { return in_offs_.size(); }
    size_t outputChannelCount() const override { return out_offs_.size(); }

    // -- Packed-bit access -------------------------------------------------------

    /// Read input channel i (the i-th 1-bit TxPDO entry).
    bool input(size_t i) const override;

    /// Write output channel i (the i-th 1-bit RxPDO entry).
    void setOutput(size_t i, bool on) override;
    bool output(size_t i) const override;

    void allOutputsOff() override;

    // -- Raw field access (non-bit entries) ---------------------------------------

    /**
     * @brief Read a wider field from the input image by (object, subindex).
     * @param occurrence  n-th matching entry (per-channel objects repeat)
     * @return the zero-extended field value, or false when not found.
     */
    bool fieldIn(uint16_t index, int subindex, uint32_t occurrence,
                 uint64_t& value) const;

    /// Like fieldIn() for the output image.
    bool fieldOut(uint16_t index, int subindex, uint32_t occurrence,
                  uint64_t value);

    /// Bit offset of an input entry (index/subindex/occurrence), for
    /// applications that want to poll raw fields directly.
    bool findInputField(uint16_t index, int subindex, uint32_t occurrence,
                        uint32_t& bit_off) const;
    bool findOutputField(uint16_t index, int subindex, uint32_t occurrence,
                         uint32_t& bit_off) const;

private:
    bool resolveLayout();
    Result<> prepare(PDO::PDOAddressMode mode);

    /// Resolved absolute bit offsets of each 1-bit channel.
    std::vector<uint32_t> in_offs_;
    std::vector<uint32_t> out_offs_;
};

} // namespace Beckhoff

} // namespace EtherCAT

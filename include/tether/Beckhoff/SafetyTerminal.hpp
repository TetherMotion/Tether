/**
 * @file SafetyTerminal.hpp
 * @brief Driver for Beckhoff TwinSAFE terminals (EL19xx safe inputs,
 *        EL29xx safe outputs) over FSoE
 *
 * The TwinSAFE terminals embed one FSoE PDU in each direction of their
 * process image:
 *
 *   SM2 (RxPDO 0x1600 "FSoE Outputs"):
 *     FSoE command (8b) | safe output bits | FSoE CRC (16b) | conn ID (16b)
 *   SM3 (TxPDO 0x1A00 "FSoE Inputs"):
 *     FSoE command (8b) | safe input bits  | FSoE CRC (16b) | conn ID (16b)
 *
 * This driver owns an FSoEMasterConnection (from the fsoe component) and
 * pumps the FSoE state machine through the registered PDO buffers via
 * exchangeViaPDO().  The application calls exchange() once per EtherCAT
 * cycle, after the cyclic LRW, with the current monotonic time.
 *
 * The FSoE address (the safety address configured in TwinCAT Safety
 * Editor) must be supplied in Config — it is part of the slave's safety
 * project, not discoverable from the ESI.
 *
 * @code
 *   auto safe = SafetyTerminal::findFirst(master, Devices::EL1918);
 *   SafetyTerminal::Config cfg;
 *   cfg.safety_addr = 0x0001;              // from the TwinSAFE project
 *   safe->configure(cfg);
 *   safe->start();
 *   while (running) {
 *       master->cycle();                    // one LRW exchange
 *       safe->exchange(now_ms);             // pump FSoE
 *       if (safe->isOperational())
 *           bool ch1 = safe->safeInputBit(0);
 *   }
 * @endcode
 */

#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <span>
#include <vector>

#include "tether/Beckhoff/TerminalBase.hpp"
#include "tether/ethercat/PDOManager.hpp"
#include "tether/ethercat/SlaveDiscoveryManager.hpp"
#include "tether/ethercat/Types.hpp"

namespace FSoE {
class FSoEMasterConnection;
struct MasterConnectionConfig;
}

namespace EtherCAT {

class Master;

namespace Beckhoff {

// ============================================================================
// Known TwinSAFE terminals (from Beckhoff EL19xx.xml / EL29xx.xml)
// ============================================================================
// num_bits holds the safe channel count.  EL1904/EL2904 are registered
// unverified (not present in the available ESI catalog).
// None verified on hardware yet — supported, not verified yet.

namespace Devices {

inline constexpr DeviceIdentity EL1918{0x00000002, 0x077E3052, 8, "EL1918"};
inline constexpr DeviceIdentity EL2911{0x00000002, 0x0B5F3052, 1, "EL2911"};
inline constexpr DeviceIdentity EL2912{0x00000002, 0x0B603052, 2, "EL2912"};

// Not in the ESI catalog at hand — registered unverified.
inline constexpr DeviceIdentity EL1904{0x00000002, 0x076C3052, 4, "EL1904"};
inline constexpr DeviceIdentity EL2904{0x00000002, 0x0B483052, 4, "EL2904"};

/// Every known TwinSAFE terminal.
inline constexpr std::array kSafetyTerminals{
    EL1918, EL2911, EL2912, EL1904, EL2904,
};

} // namespace Devices

// ============================================================================
// SafetyTerminal
// ============================================================================

class SafetyTerminal : public TerminalBase {
public:
    /// FSoE connection parameters.  The safety address comes from the
    /// TwinSAFE project downloaded to the device.
    struct Config {
        uint16_t safety_addr = 0;      ///< FSoE slave address (required)
        uint16_t connection_id = 0;    ///< 0 = derived from safety_addr
        uint8_t  safety_level = 2;     ///< FSoE SIL level (2 or 3)

        /// Safe user-data bytes inside each FSoE frame.  0 = derived
        /// from the SM image length (image - cmd - CRC - connID).
        uint8_t  safe_input_bytes  = 0;
        uint8_t  safe_output_bytes = 0;

        std::array<uint8_t, 16> fail_safe_values = {};
        std::vector<uint8_t> app_parameters;     ///< SR-AppParams, if any
    };

    // -- Construction / factories ------------------------------------------------

    SafetyTerminal(Master& master, uint16_t slave_index,
                   const DeviceIdentity& identity);
    SafetyTerminal(Master& master, const DiscoveredSlave& slave,
                   const DeviceIdentity& identity);

    ~SafetyTerminal() override;

    SafetyTerminal(SafetyTerminal&&) noexcept            = default;
    SafetyTerminal& operator=(SafetyTerminal&&) noexcept = default;
    SafetyTerminal(const SafetyTerminal&)                = delete;
    SafetyTerminal& operator=(const SafetyTerminal&)     = delete;

    /// True when `s` carries the given vendor/product identity.
    static bool matches(const DiscoveredSlave& s, const DeviceIdentity& id) {
        return s.hasVendorAndProduct(id.vendor_id, id.product_code);
    }

    /// Scan the bus for the first slave matching `identity`.
    static Result<SafetyTerminal> findFirst(Master& master,
                                            const DeviceIdentity& identity);
    static Result<SafetyTerminal> findFirst(
        Master& master, const DeviceIdentity& identity,
        std::span<const DiscoveredSlave> scan);

    // -- Bring-up ----------------------------------------------------------------------

    /// Standalone bring-up to SAFE-OP (position addressing) and create
    /// the FSoE connection.  FSoE session establishment starts with the
    /// first exchange() call in OP.
    Result<> configure(const Config& cfg);

    /// configure() then OP (+ managed realtime loop by default).
    Result<> start(const Config& cfg, const StartOptions& opts = {});

    /// Chained-operation contract.
    Result<> prepareForLogicalExchange();
    Result<> mapLogicalAndEnterSafeOp();
    Result<> requestOp(int timeout_ms);

    // -- FSoE cyclic exchange -------------------------------------------------------------

    /**
     * @brief Pump the FSoE state machine over the PDO buffers.
     *
     * Call once per EtherCAT cycle in OP, *after* the cyclic process
     * data exchange refreshed the input image.  Builds the master→slave
     * frame into the output image (sent with the next LRW) and
     * processes the slave→master frame from the input image.
     *
     * @return true when the frame exchange was processed successfully.
     */
    bool exchange(uint64_t current_time_ms);

    /// Access the underlying FSoE connection (status, callbacks, stats).
    FSoE::FSoEMasterConnection& fsoe() { return *fsoe_; }
    const FSoE::FSoEMasterConnection& fsoe() const { return *fsoe_; }

    // -- Safe I/O ---------------------------------------------------------------------------

    /// Number of safe channels for this device (from the registry).
    size_t channelCount() const { return identity_.num_bits; }

    bool  safeInputBit(uint8_t bit) const;
    bool  setSafeOutputBit(uint8_t bit, bool value);
    uint8_t safeInputByte(uint8_t index) const;
    bool  setSafeOutputByte(uint8_t index, uint8_t value);

    // -- Connection status --------------------------------------------------------------------

    bool isOperational() const;
    bool isFailSafe() const;
    uint8_t fsoeState() const;
    uint16_t fsoeErrorCode() const;

    /// Reset the FSoE connection (e.g. after a watchdog trip).
    void resetConnection();

private:
    bool resolveLayout();
    Result<> prepare(PDO::PDOAddressMode mode);

    Config config_;
    std::unique_ptr<FSoE::FSoEMasterConnection> fsoe_;
};

} // namespace Beckhoff

} // namespace EtherCAT

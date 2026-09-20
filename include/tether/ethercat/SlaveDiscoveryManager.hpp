/**
 * @file SlaveDiscoveryManager.hpp
 * @brief High-level slave discovery API for EtherCAT masters
 *
 * @details
 * `SlaveDiscoveryManager` provides a flexible, thread-safe API for discovering
 * EtherCAT slaves on the bus. It is obtained from a `Master` via
 * `master.discovery()`, which returns a reference to the manager owned by the
 * master.
 *
 * ## Discovery options
 *
 * The caller controls what information is read from each slave by passing one
 * or more `DiscoveryOption` enum values. Options that only need the first 64
 * SII words (identity, mailbox) are "shallow" and use the prefetched cache.
 * Options that need SII categories (sync managers, PDOs, DC) are "deep" and
 * trigger a full SII parse.
 *
 * Options are combined idiomatically:
 *
 * @code
 *   // Discover everything (default)
 *   auto slaves = master.discovery().discover();
 *
 *   // Discover only vendor and product IDs
 *   auto slaves = master.discovery().discover({DiscoveryOption::VendorId,
 *                                              DiscoveryOption::ProductCode});
 *
 *   // Discover one slave
 *   auto slave = master.discovery().discoverOne(3);
 *
 *   // Discover specific slaves by index
 *   std::array<uint16_t, 2> ids = {0, 2};
 *   auto slaves = master.discovery().discover(ids);
 *
 *   // Async discovery (returns std::future)
 *   auto future = master.discovery().discoverAsync({DiscoveryOption::VendorId});
 *   // ... do other work ...
 *   auto slaves = future.get();
 * @endcode
 *
 * ## Thread model
 *
 * Synchronous discovery blocks the calling thread. If `maxThreads() > 1`,
 * per-slave SII reads are parallelised across up to `maxThreads()` threads.
 *
 * Asynchronous discovery (`discoverAsync()`, `discoverOneAsync()`) launches a
 * single worker thread that performs the discovery and fulfils the returned
 * `std::future`. The worker thread may itself spawn up to `maxThreads()`
 * sub-threads for parallel per-slave SII reads.
 *
 * ## Querying results
 *
 * `DiscoveredSlave` uses `std::optional` fields. A field that was not requested
 * (or could not be read) is `std::nullopt`. Convenience query methods
 * (`hasVendorId()`, `hasProductCode()`, `hasVendorAndProduct()`,
 * `hasDeviceName()`) make common checks idiomatic.
 *
 * @code
 *   for (const auto& s : slaves) {
 *       if (s.hasVendorAndProduct(0x000022D2, 0x00000001)) {
 *           // Found a specific device
 *       }
 *   }
 * @endcode
 */

#pragma once

#include "tether/ethercat/EtherCATConfig.hpp"

#include <cstdint>
#include <future>
#include <optional>
#include <span>
#include <string>
#include <string_view>
#include <vector>

#if TETHER_ENABLE_SII
#include "tether/sii/SIIParser.hpp"
#endif

namespace EtherCAT {

class Master;

// ============================================================================
// DiscoveryOption bitmask
// ============================================================================

/**
 * @brief Selectable discovery data items.
 *
 * Combine multiple options using `DiscoveryOptions` (which accepts an
 * `initializer_list<DiscoveryOption>`). `All` requests every available item.
 *
 * "Shallow" options only need the first 64 SII words (already prefetched by
 * `initSlaves()`). "Deep" options require a full SII category parse.
 */
enum class DiscoveryOption : uint32_t {
    /// Just count slaves (BRD scan only, no SII reads). Always implicitly done.
    SlaveCount        = 0x00000001,

    // --- Identity (SII words 0x08-0x0F, shallow) ---
    /// Vendor ID (SII word 0x08-0x09)
    VendorId          = 0x00000002,
    /// Product code (SII word 0x0A-0x0B)
    ProductCode       = 0x00000004,
    /// Revision number (SII word 0x0C-0x0D)
    RevisionNumber    = 0x00000008,
    /// Serial number (SII word 0x0E-0x0F)
    SerialNumber      = 0x00000010,

    // --- Strings (SII CAT_STRINGS + CAT_GENERAL, deep) ---
    /// Device name, group name, order code
    DeviceNames       = 0x00000020,

    // --- Mailbox (SII words 0x14-0x1D, shallow) ---
    /// Mailbox offsets/sizes (bootstrap + standard)
    MailboxConfig     = 0x00000040,
    /// Supported mailbox protocols (CoE, FoE, EoE, AoE, SoE, VoE)
    MailboxProtocols  = 0x00000080,

    // --- Process data (SII categories, deep) ---
    /// Sync Manager configuration (CAT_SYNC_MANAGER)
    SyncManagers      = 0x00000100,
    /// FMMU configuration (CAT_FMMU)
    FMMUs             = 0x00000200,
    /// TxPDO descriptions (CAT_TXPDO)
    TxPDOs            = 0x00000400,
    /// RxPDO descriptions (CAT_RXPDO)
    RxPDOs            = 0x00000800,

    // --- Distributed Clocks (SII CAT_DC, deep) ---
    /// DC configuration (cycle times, sync modes)
    DistributedClocks = 0x00001000,

    // --- Device profile (SII CAT_GENERAL, deep) ---
    /// CoE/FoE/EoE details, DS402 channels, system manager class
    DeviceProfile     = 0x00002000,

    // --- General info (SII CAT_GENERAL, deep) ---
    /// General flags, E-Bus current, physical port config
    GeneralInfo       = 0x00004000,

    // --- Physical ports (SII CAT_GENERAL, deep) ---
    /// Physical port descriptor
    PhysicalPorts     = 0x00008000,

    // --- EEPROM size (SII word 0x2E, shallow) ---
    /// EEPROM size in Kbits
    EepromSize        = 0x00010000,

    /// Everything available
    All               = 0xFFFFFFFF,
};

/**
 * @brief Bitmask wrapper around `DiscoveryOption` values.
 *
 * Constructible from a single option or an `initializer_list` of options.
 *
 * @code
 *   DiscoveryOptions opts;  // empty (no SII reads)
 *   DiscoveryOptions opts(DiscoveryOption::All);
 *   DiscoveryOptions opts({DiscoveryOption::VendorId, DiscoveryOption::ProductCode});
 * @endcode
 */
class DiscoveryOptions {
public:
    /// Empty options (only BRD scan / slave count).
    constexpr DiscoveryOptions() = default;

    /// Single option.
    constexpr DiscoveryOptions(DiscoveryOption o)
        : mask_(static_cast<uint32_t>(o)) {}

    /// Combine multiple options from an initializer_list.
    DiscoveryOptions(std::initializer_list<DiscoveryOption> opts) {
        for (auto o : opts) mask_ |= static_cast<uint32_t>(o);
    }

    /// Check whether a specific option is set.
    constexpr bool has(DiscoveryOption o) const {
        return (mask_ & static_cast<uint32_t>(o)) != 0;
    }

    /// True if any option is set (i.e. SII reads are needed).
    constexpr bool any() const { return mask_ != 0; }

    /// True if no options are set (BRD scan only).
    constexpr bool empty() const { return mask_ == 0; }

    /// Raw bitmask value.
    constexpr uint32_t mask() const { return mask_; }

private:
    uint32_t mask_ = 0;
};

// ============================================================================
// DiscoveredSlave
// ============================================================================

/**
 * @brief Information gathered about a single slave during discovery.
 *
 * Fields that were not requested (or could not be read) are `std::nullopt`.
 * The `index` and `configured_address` fields are always populated when the
 * slave exists.
 */
struct DiscoveredSlave {
    /// Zero-based bus position (always set).
    uint16_t index = 0;

    /// Configured station address (SII alias or auto-assigned).
    std::optional<uint16_t> configured_address;

    // --- Identity ---
    std::optional<uint32_t> vendor_id;
    std::optional<uint32_t> product_code;
    std::optional<uint32_t> revision_number;
    std::optional<uint32_t> serial_number;

    // --- Strings ---
    std::optional<std::string> device_name;
    std::optional<std::string> group_name;
    std::optional<std::string> order_code;

    // --- Mailbox ---
#if TETHER_ENABLE_SII
    std::optional<SII::SIIMailboxConfig> mailbox_config;
#endif
    std::optional<uint16_t> mailbox_protocols;

    // --- Process data ---
#if TETHER_ENABLE_SII
    std::optional<std::vector<SII::SIISyncManager>> sync_managers;
    std::optional<std::vector<SII::SIIFMMU>> fmmus;
    std::optional<std::vector<SII::SIIPDO>> tx_pdos;
    std::optional<std::vector<SII::SIIPDO>> rx_pdos;
#endif

    // --- Distributed Clocks ---
#if TETHER_ENABLE_SII
    std::optional<std::vector<SII::SIIDCConfig>> dc_configs;
#endif

    // --- General info ---
#if TETHER_ENABLE_SII
    std::optional<SII::SIIGeneralInfo> general_info;
#endif
    std::optional<uint16_t> physical_ports;

    // --- EEPROM size ---
    std::optional<uint16_t> eeprom_size_kbits;

    // ------------------------------------------------------------------
    // Query helpers
    // ------------------------------------------------------------------

    /// True if this slave's vendor ID matches `vid`.
    bool hasVendorId(uint32_t vid) const {
        return vendor_id.has_value() && *vendor_id == vid;
    }

    /// True if this slave's product code matches `pid`.
    bool hasProductCode(uint32_t pid) const {
        return product_code.has_value() && *product_code == pid;
    }

    /// True if both vendor ID and product code match.
    bool hasVendorAndProduct(uint32_t vid, uint32_t pid) const {
        return hasVendorId(vid) && hasProductCode(pid);
    }

    /// True if the device name exactly matches `name`.
    bool hasDeviceName(std::string_view name) const {
        return device_name.has_value() && *device_name == name;
    }

    /// True if the device name contains `substr` (case-sensitive).
    bool hasDeviceNameContaining(std::string_view substr) const {
        return device_name.has_value() &&
               device_name->find(substr) != std::string::npos;
    }
};

// ============================================================================
// SlaveDiscoveryManager
// ============================================================================

/**
 * @brief Manages slave discovery for an EtherCAT master.
 *
 * Obtained via `master.discovery()`. The manager is owned by the master and
 * is reused across calls. It is thread-safe: concurrent calls to different
 * async methods are safe, though a single discovery operation at a time is
 * recommended to avoid bus contention.
 */
class SlaveDiscoveryManager {
public:
    // ------------------------------------------------------------------
    // Synchronous discovery (blocking, no suffix)
    // ------------------------------------------------------------------

    /**
     * @brief Discover all slaves on the bus.
     * @param opts What to read per slave (default: everything)
     * @return Vector of DiscoveredSlave, one per slave (empty if scan fails)
     */
    std::vector<DiscoveredSlave> discover(DiscoveryOptions opts = DiscoveryOption::All);

    /**
     * @brief Discover a single slave by index.
     * @param index Zero-based slave position
     * @param opts What to read (default: everything)
     * @return DiscoveredSlave with the requested fields (or empty optionals on failure)
     */
    DiscoveredSlave discoverOne(uint16_t index,
                                DiscoveryOptions opts = DiscoveryOption::All);

    /**
     * @brief Discover a specific set of slaves by index.
     * @param indices Slave indices to discover
     * @param opts What to read per slave (default: everything)
     * @return Vector of DiscoveredSlave for the requested indices
     */
    std::vector<DiscoveredSlave> discover(std::span<const uint16_t> indices,
                                         DiscoveryOptions opts = DiscoveryOption::All);

    // ------------------------------------------------------------------
    // Asynchronous discovery (returns std::future, "Async" suffix)
    // ------------------------------------------------------------------

    /// Async version of discover(opts).
    std::future<std::vector<DiscoveredSlave>> discoverAsync(
        DiscoveryOptions opts = DiscoveryOption::All);

    /// Async version of discoverOne(index, opts).
    std::future<DiscoveredSlave> discoverOneAsync(
        uint16_t index,
        DiscoveryOptions opts = DiscoveryOption::All);

    /// Async version of discover(indices, opts).
    std::future<std::vector<DiscoveredSlave>> discoverAsync(
        std::span<const uint16_t> indices,
        DiscoveryOptions opts = DiscoveryOption::All);

    // ------------------------------------------------------------------
    // Configuration
    // ------------------------------------------------------------------

    /// Set the maximum number of worker threads for parallel per-slave SII reads.
    void setMaxThreads(uint16_t max) { max_threads_ = max > 0 ? max : 1; }

    /// Current maximum worker thread count (default: 1).
    uint16_t maxThreads() const { return max_threads_; }

private:
    friend class Master;

    explicit SlaveDiscoveryManager(Master& master);

    /// Core synchronous discovery used by both sync and async public methods.
    std::vector<DiscoveredSlave> discoverSync(DiscoveryOptions opts,
                                              std::span<const uint16_t> indices,
                                              bool all_slaves);

    /// Read SII data for one slave into a DiscoveredSlave.
    void readSlaveSii(DiscoveredSlave& out, DiscoveryOptions opts);

    Master* master_;
    uint16_t max_threads_ = 1;
};

} // namespace EtherCAT

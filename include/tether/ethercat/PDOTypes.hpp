/**
 * @file PDOTypes.hpp
 * @brief PDO value types: sync-manager config, entries, slave config, stats.
 *
 * Split out of PDOManager.hpp.
 */

#pragma once

#include <bit>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <ctime>
#include <functional>
#include <utility>
#include <vector>

#include "tether/ethercat/EtherCATConfig.hpp"
#include "tether/ethercat/PDOModes.hpp"
#include "tether/ethercat/SMRegisters.hpp"
#include "tether/ethercat/Types.hpp"

namespace EtherCAT {

/// Monotonic nanoseconds for inline transport helpers (no Platform dep).
inline uint64_t monoNowNsFallback() {
    timespec ts{};
    ::clock_gettime(CLOCK_MONOTONIC, &ts);
    return static_cast<uint64_t>(ts.tv_sec) * 1'000'000'000ull
         + static_cast<uint64_t>(ts.tv_nsec);
}

struct PDOSliceSpec {
    /// Mapping entry indices (LogicalAddressManager::describeEntries order).
    std::vector<uint16_t> entries;
    /// Explicit image-space ranges, used when `entries` is empty.
    std::vector<std::pair<uint32_t, uint32_t>> ranges;
    /// Exchange every Nth cyclic cycle (1 = every cycle).
    uint32_t every_n = 1;
    /**
     * @brief Optional per-run completion callback — invoked on the
     *        cyclic thread after the run's response validated and
     *        scattered: (run_index, payload, payload_len, wkc).
     *        Keep it real-time safe: no allocation, no blocking.
     */
    std::function<void(uint8_t run, const uint8_t* payload,
                       uint16_t len, uint16_t wkc)> on_exchange;
};

namespace PDO {

// ============================================================================
// Constants and Limits
// ============================================================================

constexpr size_t kMaxPDOEntries = ECAT_PDO_MAX_ENTRIES;
constexpr size_t kMaxPDOSize    = ECAT_PDO_MAX_BUFFER_SIZE;
constexpr size_t kMaxPDOSlaves  = ECAT_PDO_MAX_SLAVES;

// ============================================================================
// Sync Manager Configuration
// ============================================================================

enum class SyncManagerType : uint8_t {
    Unused        = 0,
    MailboxWrite  = 1,
    MailboxRead   = 2,
    ProcessOutput = 3,
    ProcessInput  = 4
};

enum SyncManagerControl : uint8_t {
    SM_CTRL_MODE_MASK     = 0x03,
    SM_CTRL_MODE_BUFFERED = 0x00,
    SM_CTRL_MODE_MAILBOX  = 0x02,
    SM_CTRL_MODE_3PDO     = 0x03,
    SM_CTRL_DIR_READ      = 0x00,
    SM_CTRL_DIR_WRITE     = 0x04,
    SM_CTRL_IRQ_ECAT      = 0x08,
    SM_CTRL_IRQ_PDI       = 0x10,
    SM_CTRL_WATCHDOG      = 0x20,
    SM_CTRL_REPEAT_REQ    = 0x40,
};

struct SyncManagerConfig {
    uint16_t                      phys_start_addr;
    uint16_t                      length;
    EtherCAT::SyncManager::SMControlReg control;
    bool                          enable;
    SyncManagerType               type;

    static SyncManagerConfig mailbox_write(uint16_t addr, uint16_t len) {
        return { addr, len,
                 std::bit_cast<EtherCAT::SyncManager::SMControlReg>(
                     static_cast<uint8_t>(SM_CTRL_MODE_MAILBOX | SM_CTRL_DIR_WRITE | SM_CTRL_WATCHDOG)),
                 true, SyncManagerType::MailboxWrite };
    }
    static SyncManagerConfig mailbox_read(uint16_t addr, uint16_t len) {
        return { addr, len,
                 std::bit_cast<EtherCAT::SyncManager::SMControlReg>(
                     static_cast<uint8_t>(SM_CTRL_MODE_MAILBOX | SM_CTRL_DIR_READ | SM_CTRL_WATCHDOG)),
                 true, SyncManagerType::MailboxRead };
    }
    static SyncManagerConfig process_output(uint16_t addr, uint16_t len) {
        return { addr, len,
                 std::bit_cast<EtherCAT::SyncManager::SMControlReg>(
                     static_cast<uint8_t>(SM_CTRL_MODE_BUFFERED | SM_CTRL_DIR_WRITE | SM_CTRL_REPEAT_REQ)),
                 true, SyncManagerType::ProcessOutput };
    }
    static SyncManagerConfig process_input(uint16_t addr, uint16_t len) {
        return { addr, len,
                 std::bit_cast<EtherCAT::SyncManager::SMControlReg>(
                     static_cast<uint8_t>(SM_CTRL_MODE_BUFFERED | SM_CTRL_DIR_READ | SM_CTRL_REPEAT_REQ)),
                 true, SyncManagerType::ProcessInput };
    }
};

// ============================================================================
// PDO Addressing Modes
// ============================================================================

enum class PDOAddressMode : uint8_t {
    Broadcast         = 0,
    ConfiguredAddress  = 1,
    Position           = 2,
    Logical            = 3
};

// ============================================================================
// PDO Entry Definition
// ============================================================================

enum class PDODirection : uint8_t {
    TxPDO = 0,  ///< Slave→Master
    RxPDO = 1   ///< Master→Slave
};

struct PDOEntry {
    uint16_t       slave_index;
    PDODirection   direction;
    PDOAddressMode address_mode;

    uint16_t configured_address;
    uint32_t logical_address;
    uint16_t physical_offset;

    /**
     * @brief Manager-owned buffered-path storage (Q1: replaces the former
     *        caller-supplied `void* app_buffer`).
     *
     * The exchange gathers/scatters directly in this array.  Applications
     * reach it through PDOMapping::entryData()/entryDataMut()/
     * entryDataAs<T>() or an epoch-checked entryHandle() + resolve().
     * Fixed inline storage keeps PDOEntry trivially copyable and makes the
     * buffered path allocation-free after registration.
     */
    /// `mutable`: the exchange writes wire data here through const
    /// PDOMapping& — storage is I/O state, not mapping metadata (same
    /// const-escape the old app_buffer pointee had).
    alignas(8) mutable uint8_t storage[kMaxPDOSize];
    uint16_t data_size;

    uint16_t pdo_index;

    bool     enabled;
    /**
     * @brief Force this entry onto the buffered (storage) path even when
     *        a ProcessImage mode is active.  Set for FSoE-managed PDOs —
     *        safe frames are staged and CRC'd by the FSoE layer and must
     *        not be written in place by the application.
     */
    bool     image_exclude{false};
    /**
     * @brief A buffered-path accessor (entryData, entryDataAs, resolve)
     *        handed out a pointer into storage[] — the application holds
     *        storage-bound state.  Image-mode exchanges still bridge such
     *        entries: the gather copies storage to the wire and the scatter
     *        copies the wire back to storage, so device-level PDO accessors
     *        stay coherent in every image mode (they just pay a per-entry
     *        copy).  `mutable` like storage[] — marking is a binding
     *        observation, not mapping metadata.
     */
    mutable bool storage_bound{false};
    uint32_t error_count;
    uint32_t success_count;
};

/// Logical placement of one enabled PDO entry within the process image,
/// using the layout of the logical LRW exchange.  Lets a caller compose
/// partial-read slices (e.g. group the FSoE PDOs apart from the RSAP/debug
/// PDOs) without knowing the internal address map.
struct LogicalEntrySlice {
    size_t       entry_index{0};   ///< index into PDOMapping
    uint16_t     slave_index{0};
    uint16_t     pdo_index{0};
    PDODirection direction{PDODirection::RxPDO};
    uint32_t     offset{0};        ///< byte offset from the base logical address
    uint16_t     length{0};
};

// ============================================================================
// Slave Configuration Structure
// ============================================================================

struct SlaveConfig {
    uint16_t slave_index;
    uint16_t configured_address;
    /// True once configured_address has been read from the slave (reg
    /// 0x0010) or set explicitly — finalizeMapping() resolves it lazily.
    bool     configured_address_known = false;
    uint32_t vendor_id;
    uint32_t product_code;

    SyncManagerConfig sm[4];

    uint16_t rxpdo_size;
    uint16_t txpdo_size;
    uint16_t rxpdo_sm;
    uint16_t txpdo_sm;

    uint16_t mbx_write_offset;
    uint16_t mbx_write_size;
    uint16_t mbx_read_offset;
    uint16_t mbx_read_size;
    uint8_t  mbx_protocols;

    bool     configured;
    bool     operational;

    uint32_t pdo_request_count = 0;   // successful RxPDO sends (master -> slave)
    uint32_t pdo_reply_count   = 0;   // successful TxPDO receives (slave -> master)
};

// ============================================================================
// PDO Exchange Statistics
// ============================================================================

struct PDOStats {
    uint64_t total_cycles;
    uint64_t rxpdo_frames_sent;
    uint64_t txpdo_frames_recv;
    uint32_t rxpdo_errors;
    uint32_t txpdo_errors;
    uint32_t wkc_errors;
    uint32_t last_rxpdo_time_us;
    uint32_t last_txpdo_time_us;
    uint32_t max_cycle_time_us;
};

} // namespace PDO

} // namespace EtherCAT

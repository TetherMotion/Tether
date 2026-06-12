/**
 * @file EtherCATPDO.hpp
 * @brief EtherCAT Process Data Object (PDO) mapping and transfer system
 *
 * @details
 * This module provides a flexible system for mapping user-defined data structures
 * to EtherCAT Process Data Objects (PDOs).  PDOs are the primary mechanism for
 * real-time data exchange between the EtherCAT master and slaves.
 *
 * ## Architecture (refactored – no global state)
 *
 * All mutable state lives inside **PDOManager** instances.  Network I/O is
 * abstracted behind the **IPDOTransport** interface so that unit tests can
 * inject a mock without linking the full EtherCAT stack.
 *
 * Backward-compatible free functions in `namespace PDO` are retained but
 * now take a `PDOManager&` as their first parameter.
 */

#pragma once

#include <cstdint>
#include <cstddef>
#include <cstring>
#include <atomic>
#include <functional>

#include "tether/platform/EspCompat.hpp"
#include "tether/ethercat/EtherCATTypes.hpp"

#ifdef ESP_PLATFORM
#include "esp_eth_driver.h"
#endif

namespace EtherCAT {

// Forward declarations
class IPDOTransport;
class PDOManager;
class LogicalAddressManager;

namespace PDO {

// ============================================================================
// Constants and Limits
// ============================================================================

constexpr size_t kMaxPDOEntries = 32;
constexpr size_t kMaxPDOSize    = 256;
constexpr size_t kMaxPDOSlaves  = 16;

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
    SM_CTRL_DIR_READ      = 0x00,
    SM_CTRL_DIR_WRITE     = 0x04,
    SM_CTRL_IRQ_ECAT      = 0x10,
    SM_CTRL_IRQ_PDI       = 0x20,
    SM_CTRL_WATCHDOG      = 0x40,
    SM_CTRL_REPEAT_REQ    = 0x02,
};

struct SyncManagerConfig {
    uint16_t        phys_start_addr;
    uint16_t        length;
    uint8_t         control;
    bool            enable;
    SyncManagerType type;

    static SyncManagerConfig mailbox_write(uint16_t addr, uint16_t len) {
        return { addr, len,
                 static_cast<uint8_t>(SM_CTRL_MODE_MAILBOX | SM_CTRL_DIR_WRITE | SM_CTRL_IRQ_PDI),
                 true, SyncManagerType::MailboxWrite };
    }
    static SyncManagerConfig mailbox_read(uint16_t addr, uint16_t len) {
        return { addr, len,
                 static_cast<uint8_t>(SM_CTRL_MODE_MAILBOX | SM_CTRL_DIR_READ | SM_CTRL_IRQ_PDI),
                 true, SyncManagerType::MailboxRead };
    }
    static SyncManagerConfig process_output(uint16_t addr, uint16_t len) {
        return { addr, len,
                 static_cast<uint8_t>(SM_CTRL_MODE_BUFFERED | SM_CTRL_DIR_WRITE | SM_CTRL_IRQ_PDI | SM_CTRL_WATCHDOG),
                 true, SyncManagerType::ProcessOutput };
    }
    static SyncManagerConfig process_input(uint16_t addr, uint16_t len) {
        return { addr, len,
                 static_cast<uint8_t>(SM_CTRL_MODE_BUFFERED | SM_CTRL_DIR_READ | SM_CTRL_IRQ_PDI),
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

    void*    app_buffer;
    uint16_t data_size;

    uint16_t pdo_index;

    bool     enabled;
    uint32_t error_count;
    uint32_t success_count;
};

// ============================================================================
// PDO Mapping Manager (value type – no transport dependency)
// ============================================================================

class PDOMapping {
public:
    int  add_rxpdo(uint16_t slave_index, void* buffer, uint16_t size,
                   uint16_t pdo_index = 0x1600,
                   PDOAddressMode mode = PDOAddressMode::Position);

    int  add_txpdo(uint16_t slave_index, void* buffer, uint16_t size,
                   uint16_t pdo_index = 0x1A00,
                   PDOAddressMode mode = PDOAddressMode::Position);

    int  add_broadcast_rxpdo(void* buffer, uint16_t size, uint16_t physical_offset);
    int  add_broadcast_txpdo(void* buffer, uint16_t size, uint16_t physical_offset);

    void set_slave_configured_address(uint16_t slave_index, uint16_t configured_addr);

    size_t         entry_count() const { return m_entry_count; }
    const PDOEntry* get_entry(size_t index) const;
    PDOEntry*       get_entry_mut(size_t index);
    void            clear();

    size_t total_rxpdo_bytes() const;
    size_t total_txpdo_bytes() const;

private:
    PDOEntry m_entries[kMaxPDOEntries];
    size_t   m_entry_count = 0;
    uint16_t m_slave_configured_addrs[kMaxPDOSlaves] = {0};
};

// ============================================================================
// Slave Configuration Structure
// ============================================================================

struct SlaveConfig {
    uint16_t slave_index;
    uint16_t configured_address;
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

// ============================================================================
// Backward-compatible free functions (delegate to PDOManager)
// ============================================================================

bool       pdo_init(PDOManager& mgr);
void       pdo_deinit(PDOManager& mgr);
PDOMapping& pdo_get_mapping(PDOManager& mgr);
SlaveConfig* pdo_get_slave_configs(PDOManager& mgr);
bool       pdo_configure_slave_sms(PDOManager& mgr, uint16_t slave_index);
uint16_t   pdo_configure_all_slave_sms(PDOManager& mgr, uint16_t slave_count);
bool       pdo_exchange_all(PDOManager& mgr);
bool       pdo_exchange_lrw(PDOManager& mgr, uint16_t slave_count);
void       pdo_get_lrw_stats(PDOManager& mgr,
                             uint32_t* success, uint32_t* wkc_errors,
                             uint32_t* send_errors, uint32_t* timeout_errors);
void       pdo_set_separate_mode(PDOManager& mgr, bool separate);
bool       pdo_get_separate_mode(PDOManager& mgr);
bool       pdo_exchange_separate(PDOManager& mgr, uint16_t slave_count);
void       pdo_get_separate_stats(PDOManager& mgr,
                                  uint32_t* lwr_success, uint32_t* lwr_wkc_errors,
                                  uint32_t* lrd_success, uint32_t* lrd_wkc_errors);
void       pdo_set_physical_mode(PDOManager& mgr, bool physical);
bool       pdo_get_physical_mode(PDOManager& mgr);
bool       pdo_exchange_physical(PDOManager& mgr, uint16_t slave_count);
void       pdo_get_physical_stats(PDOManager& mgr,
                                  uint32_t* fpwr_success, uint32_t* fpwr_wkc_errors,
                                  uint32_t* fprd_success, uint32_t* fprd_wkc_errors);
bool       pdo_send_rxpdo(PDOManager& mgr, size_t entry_index);
bool       pdo_receive_txpdo(PDOManager& mgr, size_t entry_index);
bool       pdo_finalize_mapping(PDOManager& mgr, uint16_t slave_index);
PDOStats   pdo_get_stats(PDOManager& mgr);
void       pdo_reset_stats(PDOManager& mgr);

} // namespace PDO

// ============================================================================
// IPDOTransport — abstract transport interface for PDO I/O
// ============================================================================

/**
 * @brief Transport interface for PDO operations.
 *
 * Abstracts the low-level EtherCAT I/O primitives needed by PDOManager.
 * Production builds use a concrete implementation that delegates to
 * Raw:: functions; unit tests inject a mock.
 */
class IPDOTransport {
public:
    virtual ~IPDOTransport() = default;

    /// Fire-and-forget index constant
    static constexpr uint8_t kFireAndForgetIdx = 0xFE;

    virtual bool writeRegister(uint16_t adp, uint16_t ado,
                               const void* data, uint16_t len,
                               unsigned int timeout_ms) = 0;

    virtual bool readRegister(uint16_t adp, uint16_t ado,
                              void* data, uint16_t len,
                              unsigned int timeout_ms) = 0;

    virtual bool sendSingleDatagram(Command cmd, uint8_t idx,
                                    uint16_t adp, uint16_t ado,
                                    const void* data, uint16_t datalen,
                                    bool roundtrip) = 0;

    virtual bool waitForResponseIdx(uint8_t idx, unsigned int timeout_ms,
                                    RxDatagram& out) = 0;

    virtual uint8_t  allocIdx() = 0;
    virtual uint16_t adpForSlaveIndex(uint16_t slave_index) = 0;
};

// ============================================================================
// PDOManager — instance-based, owns all PDO/SM state (no globals)
// ============================================================================

/**
 * @brief Instance-based PDO manager.
 *
 * Each PDOManager owns its own SlaveConfig array, PDOMapping, PDOStats,
 * and mode/transfer counters.  Multiple independent instances can co-exist.
 * Network I/O is performed through the injected IPDOTransport.
 */
class PDOManager {
public:
    explicit PDOManager(IPDOTransport& transport);
    ~PDOManager();

    PDOManager(const PDOManager&)            = delete;
    PDOManager& operator=(const PDOManager&) = delete;

    // ----- Lifecycle -----
    bool init();
    void deinit();
    bool isInitialized() const;

    // ----- Configuration Access -----
    PDO::PDOMapping&       mapping();
    const PDO::PDOMapping& mapping() const;

    PDO::SlaveConfig*       slaveConfigs();
    const PDO::SlaveConfig* slaveConfigs() const;

    size_t slaveCount() const;
    void   setSlaveCount(size_t count);

    // ----- Statistics -----
    PDO::PDOStats  getStats() const;
    void           resetStats();
    PDO::PDOStats& statsRef();

    // ----- SM Configuration -----
    bool     configureSlavesSMs(uint16_t slave_index);
    uint16_t configureAllSlaveSMs(uint16_t slave_count);

    // ----- Mapping Finalization -----
    bool finalizeMapping(uint16_t slave_index);

    // ----- PDO Transfer (single entry) -----
    bool sendRxPDO(size_t entry_index);
    bool receiveTxPDO(size_t entry_index);
    bool exchangeAll();

    // ----- PDO Exchange modes -----
    bool exchangeLRW(uint16_t slave_count);
    bool exchangeSeparate(uint16_t slave_count);
    bool exchangePhysical(uint16_t slave_count);

    // ----- Mode settings -----
    void setSeparateMode(bool separate);
    bool getSeparateMode() const;
    void setPhysicalMode(bool physical);
    bool getPhysicalMode() const;

    // ----- Detailed per-mode statistics -----

    struct LRWStats {
        uint32_t lrw_success{0};
        uint32_t lrw_wkc_errors{0};
        uint32_t lrw_send_errors{0};
        uint32_t lrw_timeout_errors{0};
    };
    LRWStats getLRWStats() const;

    struct SeparateStats {
        uint32_t lwr_success{0};
        uint32_t lwr_wkc_errors{0};
        uint32_t lrd_success{0};
        uint32_t lrd_wkc_errors{0};
        uint32_t send_errors{0};
        uint32_t timeout_errors{0};
    };
    SeparateStats getSeparateStats() const;

    struct PhysicalStats {
        uint32_t fpwr_success{0};
        uint32_t fpwr_wkc_errors{0};
        uint32_t fprd_success{0};
        uint32_t fprd_wkc_errors{0};
        uint32_t send_errors{0};
        uint32_t timeout_errors{0};
    };
    PhysicalStats getPhysicalStats() const;

    struct TransferStats {
        uint32_t rxpdo_debug_count{0};
        uint32_t rxpdo_confirmed_ok{0};
        uint32_t rxpdo_confirmed_fail{0};
        uint32_t txpdo_debug_count{0};
    };
    TransferStats getTransferStats() const;

    IPDOTransport& transport();

    void setLogicalAddressManager(LogicalAddressManager* mgr) { logical_addr_mgr_ = mgr; }
    LogicalAddressManager* logicalAddressManager() const { return logical_addr_mgr_; }

private:
    IPDOTransport& transport_;

    // Owned state (formerly globals)
    PDO::SlaveConfig slave_configs_[PDO::kMaxPDOSlaves];
    PDO::PDOMapping  mapping_;
    PDO::PDOStats    stats_{};
    bool             initialized_ = false;
    size_t           slave_count_ = 0;

    // Mode flags
    bool use_separate_commands_ = false;
    bool use_physical_mode_     = false;

    LogicalAddressManager* logical_addr_mgr_ = nullptr;

    // Per-mode stats
    LRWStats      lrw_stats_;
    SeparateStats separate_stats_;
    PhysicalStats physical_stats_;
    TransferStats transfer_stats_;

    // ----- Private helpers -----
    bool writeSMConfig(uint16_t adp, uint8_t sm_index,
                       const PDO::SyncManagerConfig& config);
    bool readSMStatus(uint16_t adp, uint8_t sm_index, uint8_t& status);

    // Transfer helpers
    bool sendRxPDOPosition(const PDO::PDOEntry& entry);
    bool recvTxPDOPosition(PDO::PDOEntry& entry);
    bool sendRxPDOConfigured(const PDO::PDOEntry& entry);
    bool recvTxPDOConfigured(PDO::PDOEntry& entry);
    bool sendRxPDOBroadcast(const PDO::PDOEntry& entry, uint16_t expected_wkc);
    bool recvTxPDOBroadcast(PDO::PDOEntry& entry, uint16_t expected_wkc);
};

} // namespace EtherCAT

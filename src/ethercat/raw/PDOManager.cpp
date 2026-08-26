/**
 * @file PDOManager.cpp
 * @brief PDOManager implementation — replaces the old sync_manager.cpp,
 *        pdo_api.cpp, pdo_logical.cpp and pdo_transfer.cpp files.
 *
 * All mutable state now lives inside PDOManager instances.  There are
 * no file-scoped or namespace-scoped globals.
 */

#include "PDOManager.hpp"
#include "tether/ethercat/LogicalAddressManager.hpp"
#include "tether/ethercat/DebugFlags.hpp"
#include "tether/platform/EspCompat.hpp"
#include "tether/platform/Platform.hpp"

#include <cstring>
#include <cstdio>
#include <bit>

namespace EtherCAT {

static const char* TAG = "ec_pdo_mgr";

// ============================================================================
// SM Register Definitions (internal to this TU)
// ============================================================================

enum SMRegisters : uint16_t {
    EC_REG_SM0_BASE = 0x0800,
    EC_REG_SM1_BASE = 0x0808,
    EC_REG_SM2_BASE = 0x0810,
    EC_REG_SM3_BASE = 0x0818,
};

enum SMOffsets : uint8_t {
    SM_OFF_PHYS_ADDR = 0x00,
    SM_OFF_LENGTH    = 0x02,
    SM_OFF_CONTROL   = 0x04,
    SM_OFF_STATUS    = 0x05,
    SM_OFF_ACTIVATE  = 0x06,
    SM_OFF_PDI_CTRL  = 0x07,
};

enum SMActivateBits : uint8_t {
    SM_ACT_ENABLE      = 0x01,
    SM_ACT_REPEAT_REQ  = 0x02,
    SM_ACT_DC_EVENT0   = 0x04,
    SM_ACT_DC_EVENT1   = 0x08,
    SM_ACT_LATCH_EVENT = 0x10,
};

static inline uint16_t sm_base_address(uint8_t sm_index) {
    return static_cast<uint16_t>(EC_REG_SM0_BASE + (sm_index * 8));
}

// Trivial host↔LE helpers (ESP32 is already little-endian)
static inline uint16_t host_to_le16(uint16_t v) { return v; }

// ============================================================================
// PDOMapping implementation (value type, same as before)
// ============================================================================

namespace PDO {

int PDOMapping::add_rxpdo(uint16_t slave_index, void* buffer, uint16_t size,
                          uint16_t pdo_index, PDOAddressMode mode) {
    if (m_entry_count >= kMaxPDOEntries) {
        TETHER_LOGE(TAG,
            "Tether internal PDO entry limit reached (%zu entries). This is a Tether limit, "
            "not a slave limit. Increase ECAT_PDO_MAX_ENTRIES in TetherConfig.hpp.",
            kMaxPDOEntries);
        return -1;
    }
    if (!buffer || size == 0 || size > kMaxPDOSize) {
        TETHER_LOGE(TAG,
            "Invalid PDO buffer or size (size=%u). %s",
            size,
            (size > kMaxPDOSize)
                ? "This is a Tether limit, not a slave limit. "
                  "Increase ECAT_PDO_MAX_BUFFER_SIZE in TetherConfig.hpp."
                : "Buffer is null or size is zero.");
        return -1;
    }
    PDOEntry& e   = m_entries[m_entry_count];
    e.slave_index  = slave_index;
    e.direction    = PDODirection::RxPDO;
    e.address_mode = mode;
    e.app_buffer   = buffer;
    e.data_size    = size;
    e.pdo_index    = pdo_index;
    e.enabled      = true;
    e.error_count  = 0;
    e.success_count= 0;
    e.physical_offset = 0;
    if (slave_index < kMaxPDOSlaves)
        e.configured_address = m_slave_configured_addrs[slave_index];
    TETHER_LOGI(TAG, "Added RxPDO: slave=%u size=%u pdo=0x%04x mode=%d",
                slave_index, size, pdo_index, static_cast<int>(mode));
    return static_cast<int>(m_entry_count++);
}

int PDOMapping::add_txpdo(uint16_t slave_index, void* buffer, uint16_t size,
                          uint16_t pdo_index, PDOAddressMode mode) {
    if (m_entry_count >= kMaxPDOEntries) {
        TETHER_LOGE(TAG,
            "Tether internal PDO entry limit reached (%zu entries). This is a Tether limit, "
            "not a slave limit. Increase ECAT_PDO_MAX_ENTRIES in TetherConfig.hpp.",
            kMaxPDOEntries);
        return -1;
    }
    if (!buffer || size == 0 || size > kMaxPDOSize) {
        TETHER_LOGE(TAG,
            "Invalid PDO buffer or size (size=%u). %s",
            size,
            (size > kMaxPDOSize)
                ? "This is a Tether limit, not a slave limit. "
                  "Increase ECAT_PDO_MAX_BUFFER_SIZE in TetherConfig.hpp."
                : "Buffer is null or size is zero.");
        return -1;
    }
    PDOEntry& e   = m_entries[m_entry_count];
    e.slave_index  = slave_index;
    e.direction    = PDODirection::TxPDO;
    e.address_mode = mode;
    e.app_buffer   = buffer;
    e.data_size    = size;
    e.pdo_index    = pdo_index;
    e.enabled      = true;
    e.error_count  = 0;
    e.success_count= 0;
    e.physical_offset = 0;
    if (slave_index < kMaxPDOSlaves)
        e.configured_address = m_slave_configured_addrs[slave_index];
    TETHER_LOGI(TAG, "Added TxPDO: slave=%u size=%u pdo=0x%04x mode=%d",
                slave_index, size, pdo_index, static_cast<int>(mode));
    return static_cast<int>(m_entry_count++);
}

int PDOMapping::add_broadcast_rxpdo(void* buffer, uint16_t size, uint16_t physical_offset) {
    if (m_entry_count >= kMaxPDOEntries) return -1;
    PDOEntry& e = m_entries[m_entry_count];
    e.slave_index     = 0xFFFF;
    e.direction       = PDODirection::RxPDO;
    e.address_mode    = PDOAddressMode::Broadcast;
    e.physical_offset = physical_offset;
    e.app_buffer      = buffer;
    e.data_size       = size;
    e.enabled         = true;
    e.error_count     = 0;
    e.success_count   = 0;
    TETHER_LOGI(TAG, "Added broadcast RxPDO: offset=0x%04x size=%u", physical_offset, size);
    return static_cast<int>(m_entry_count++);
}

int PDOMapping::add_broadcast_txpdo(void* buffer, uint16_t size, uint16_t physical_offset) {
    if (m_entry_count >= kMaxPDOEntries) return -1;
    PDOEntry& e = m_entries[m_entry_count];
    e.slave_index     = 0xFFFF;
    e.direction       = PDODirection::TxPDO;
    e.address_mode    = PDOAddressMode::Broadcast;
    e.physical_offset = physical_offset;
    e.app_buffer      = buffer;
    e.data_size       = size;
    e.enabled         = true;
    e.error_count     = 0;
    e.success_count   = 0;
    TETHER_LOGI(TAG, "Added broadcast TxPDO: offset=0x%04x size=%u", physical_offset, size);
    return static_cast<int>(m_entry_count++);
}

void PDOMapping::set_slave_configured_address(uint16_t slave_index, uint16_t configured_addr) {
    if (slave_index < kMaxPDOSlaves) {
        m_slave_configured_addrs[slave_index] = configured_addr;
        for (size_t i = 0; i < m_entry_count; i++) {
            if (m_entries[i].slave_index == slave_index)
                m_entries[i].configured_address = configured_addr;
        }
    }
}

const PDOEntry* PDOMapping::get_entry(size_t index) const {
    return (index < m_entry_count) ? &m_entries[index] : nullptr;
}

PDOEntry* PDOMapping::get_entry_mut(size_t index) {
    return (index < m_entry_count) ? &m_entries[index] : nullptr;
}

void PDOMapping::clear() {
    m_entry_count = 0;
    std::memset(m_entries, 0, sizeof(m_entries));
}

void PDOMapping::remove_entries_for_slave(uint16_t slave_index) {
    size_t write = 0;
    for (size_t read = 0; read < m_entry_count; ++read) {
        if (m_entries[read].slave_index != slave_index) {
            if (write != read) {
                m_entries[write] = m_entries[read];
            }
            ++write;
        }
    }
    // zero out the vacated tail so stale pointers don't dangle
    for (size_t i = write; i < m_entry_count; ++i) {
        std::memset(&m_entries[i], 0, sizeof(PDOEntry));
    }
    m_entry_count = write;
}

size_t PDOMapping::total_rxpdo_bytes() const {
    size_t total = 0;
    for (size_t i = 0; i < m_entry_count; i++)
        if (m_entries[i].direction == PDODirection::RxPDO && m_entries[i].enabled)
            total += m_entries[i].data_size;
    return total;
}

size_t PDOMapping::total_txpdo_bytes() const {
    size_t total = 0;
    for (size_t i = 0; i < m_entry_count; i++)
        if (m_entries[i].direction == PDODirection::TxPDO && m_entries[i].enabled)
            total += m_entries[i].data_size;
    return total;
}

// ============================================================================
// Backward-compatible free functions → delegate to PDOManager
// ============================================================================

bool       pdo_init(PDOManager& m)                        { return m.init(); }
void       pdo_deinit(PDOManager& m)                      { m.deinit(); }
PDOMapping& pdo_get_mapping(PDOManager& m)                { return m.mapping(); }
SlaveConfig* pdo_get_slave_configs(PDOManager& m)         { return m.slaveConfigs(); }
bool       pdo_configure_slave_sms(PDOManager& m, uint16_t s)     { return m.configureSlavesSMs(s); }
uint16_t   pdo_configure_all_slave_sms(PDOManager& m, uint16_t n) { return m.configureAllSlaveSMs(n); }
bool       pdo_exchange_all(PDOManager& m)                { return m.exchangeAll(); }
bool       pdo_exchange_lrw(PDOManager& m, uint16_t n)    { return m.exchangeLRW(n); }
void       pdo_get_lrw_stats(PDOManager& m, uint32_t* s, uint32_t* w, uint32_t* se, uint32_t* t) {
    auto st = m.getLRWStats();
    if (s) *s = st.lrw_success;
    if (w) *w = st.lrw_wkc_errors;
    if (se)*se = st.lrw_send_errors;
    if (t) *t = st.lrw_timeout_errors;
}
void pdo_set_separate_mode(PDOManager& m, bool v) { m.setSeparateMode(v); }
bool pdo_get_separate_mode(PDOManager& m)          { return m.getSeparateMode(); }
bool pdo_exchange_separate(PDOManager& m, uint16_t n) { return m.exchangeSeparate(n); }
void pdo_get_separate_stats(PDOManager& m, uint32_t* a, uint32_t* b, uint32_t* c, uint32_t* d) {
    auto st = m.getSeparateStats();
    if (a) *a = st.lwr_success;
    if (b) *b = st.lwr_wkc_errors;
    if (c) *c = st.lrd_success;
    if (d) *d = st.lrd_wkc_errors;
}
void pdo_set_physical_mode(PDOManager& m, bool v) { m.setPhysicalMode(v); }
bool pdo_get_physical_mode(PDOManager& m)          { return m.getPhysicalMode(); }
bool pdo_exchange_physical(PDOManager& m, uint16_t n) { return m.exchangePhysical(n); }
void pdo_get_physical_stats(PDOManager& m, uint32_t* a, uint32_t* b, uint32_t* c, uint32_t* d) {
    auto st = m.getPhysicalStats();
    if (a) *a = st.fpwr_success;
    if (b) *b = st.fpwr_wkc_errors;
    if (c) *c = st.fprd_success;
    if (d) *d = st.fprd_wkc_errors;
}
bool   pdo_send_rxpdo(PDOManager& m, size_t i)     { return m.sendRxPDO(i); }
bool   pdo_receive_txpdo(PDOManager& m, size_t i)  { return m.receiveTxPDO(i); }
bool   pdo_finalize_mapping(PDOManager& m, uint16_t s) { return m.finalizeMapping(s); }
PDOStats pdo_get_stats(PDOManager& m)              { return m.getStats(); }
void   pdo_reset_stats(PDOManager& m)              { m.resetStats(); }

} // namespace PDO

// ============================================================================
// PDOManager — constructor / destructor / lifecycle
// ============================================================================

PDOManager::PDOManager(IPDOTransport& transport)
    : transport_(transport)
{
    std::memset(slave_configs_, 0, sizeof(slave_configs_));
    std::memset(&stats_, 0, sizeof(stats_));
}

PDOManager::~PDOManager() {
    if (initialized_) deinit();
}

bool PDOManager::init() {
    if (initialized_) return true;
    std::memset(slave_configs_, 0, sizeof(slave_configs_));
    std::memset(&stats_, 0, sizeof(stats_));
    slave_count_ = 0;
    mapping_.clear();
    lrw_stats_      = LRWStats{};
    separate_stats_  = SeparateStats{};
    physical_stats_  = PhysicalStats{};
    transfer_stats_  = TransferStats{};
    use_separate_commands_ = false;
    use_physical_mode_     = false;
    initialized_ = true;
    TETHER_LOGI(TAG, "PDO subsystem initialized");
    return true;
}

void PDOManager::deinit() {
    mapping_.clear();
    initialized_ = false;
    TETHER_LOGI(TAG, "PDO subsystem deinitialized");
}

bool PDOManager::isInitialized() const { return initialized_; }

// ----- Configuration Access -----

PDO::PDOMapping&       PDOManager::mapping()       { return mapping_; }
const PDO::PDOMapping& PDOManager::mapping() const { return mapping_; }

PDO::SlaveConfig*       PDOManager::slaveConfigs()       { return slave_configs_; }
const PDO::SlaveConfig* PDOManager::slaveConfigs() const { return slave_configs_; }

size_t PDOManager::slaveCount() const      { return slave_count_; }
void   PDOManager::setSlaveCount(size_t c) { slave_count_ = c; }

// ----- Statistics -----

PDO::PDOStats  PDOManager::getStats() const  { return stats_; }
void           PDOManager::resetStats()      { std::memset(&stats_, 0, sizeof(stats_)); }
PDO::PDOStats& PDOManager::statsRef()        { return stats_; }

// ----- Per-slave PDO counter accessors -----

bool PDOManager::hasSlavePDOEntries(uint16_t slave_index) const {
    for (size_t i = 0; i < mapping_.entry_count(); i++) {
        const PDO::PDOEntry* e = mapping_.get_entry(i);
        if (e && e->enabled && e->slave_index == slave_index)
            return true;
    }
    return false;
}

uint32_t PDOManager::getSlavePDORequestCount(uint16_t slave_index) const {
    if (slave_index >= PDO::kMaxPDOSlaves) return 0;
    return slave_configs_[slave_index].pdo_request_count;
}

uint32_t PDOManager::getSlavePDOReplyCount(uint16_t slave_index) const {
    if (slave_index >= PDO::kMaxPDOSlaves) return 0;
    return slave_configs_[slave_index].pdo_reply_count;
}

// ----- Mode settings -----

void PDOManager::setSeparateMode(bool v) {
    use_separate_commands_ = v;
    TETHER_LOGI(TAG, "PDO mode set to: %s", v ? "SEPARATE (LRD+LWR)" : "COMBINED (LRW)");
}
bool PDOManager::getSeparateMode()  const { return use_separate_commands_; }
void PDOManager::setPhysicalMode(bool v) {
    use_physical_mode_ = v;
    TETHER_LOGI(TAG, "PDO mode set to: %s", v ? "PHYSICAL (FPWR+FPRD)" : "LOGICAL (FMMU-based)");
}
bool PDOManager::getPhysicalMode()  const { return use_physical_mode_; }

// ----- Detailed per-mode stat accessors -----

PDOManager::LRWStats      PDOManager::getLRWStats()      const { return lrw_stats_; }
PDOManager::SeparateStats PDOManager::getSeparateStats() const { return separate_stats_; }
PDOManager::PhysicalStats PDOManager::getPhysicalStats() const { return physical_stats_; }
PDOManager::TransferStats PDOManager::getTransferStats() const { return transfer_stats_; }

IPDOTransport& PDOManager::transport() { return transport_; }

// ============================================================================
// SM Configuration helpers (formerly in sync_manager.cpp)
// ============================================================================

bool PDOManager::writeSMConfig(uint16_t adp, uint8_t sm_index,
                               const PDO::SyncManagerConfig& config,
                               uint16_t slave_index)
{
    const uint16_t base = sm_base_address(sm_index);

    // Step 1: Disable SM
    uint8_t disable = 0x00;
    if (!transport_.writeRegister(adp, static_cast<uint16_t>(base + SM_OFF_ACTIVATE),
                                  &disable, sizeof(disable), 200)) {
        TETHER_LOGW(TAG, "SM%u: failed to disable", sm_index);
    }

    // Step 2: Physical address
    uint16_t addr_le = host_to_le16(config.phys_start_addr);
    if (!transport_.writeRegister(adp, static_cast<uint16_t>(base + SM_OFF_PHYS_ADDR),
                                  &addr_le, sizeof(addr_le), 200)) {
        TETHER_LOGE(TAG, "SM%u: failed to write phys_addr=0x%04x", sm_index, config.phys_start_addr);
        return false;
    }

    // Step 3: Length
    uint16_t len_le = host_to_le16(config.length);
    if (!transport_.writeRegister(adp, static_cast<uint16_t>(base + SM_OFF_LENGTH),
                                  &len_le, sizeof(len_le), 200)) {
        TETHER_LOGE(TAG, "SM%u: failed to write length=%u", sm_index, config.length);
        return false;
    }

    // Step 4: Control
    uint8_t ctrl_byte = std::bit_cast<uint8_t>(config.control);
    if (!transport_.writeRegister(adp, static_cast<uint16_t>(base + SM_OFF_CONTROL),
                                  &ctrl_byte, sizeof(ctrl_byte), 200)) {
        TETHER_LOGE(TAG, "SM%u: failed to write control=0x%02x", sm_index, ctrl_byte);
        return false;
    }

    // Step 5: Activate
    uint8_t activate = config.enable ? SM_ACT_ENABLE : 0x00;
    if (!transport_.writeRegister(adp, static_cast<uint16_t>(base + SM_OFF_ACTIVATE),
                                  &activate, sizeof(activate), 200)) {
        TETHER_LOGE(TAG, "SM%u: failed to write activate=0x%02x", sm_index, activate);
        return false;
    }

    TETHER_LOGI(TAG, "Slave %u: SM%u: configured addr=0x%04x len=%u ctrl=0x%02x act=0x%02x",
                slave_index, sm_index, config.phys_start_addr, config.length, ctrl_byte, activate);

    if ((rxPDODebug() && config.type == PDO::SyncManagerType::ProcessOutput) ||
        (txPDODebug() && config.type == PDO::SyncManagerType::ProcessInput)) {
        const char* sm_type_str = "unknown";
        switch (config.type) {
            case PDO::SyncManagerType::Unused:        sm_type_str = "unused"; break;
            case PDO::SyncManagerType::MailboxWrite:  sm_type_str = "mailbox-write"; break;
            case PDO::SyncManagerType::MailboxRead:   sm_type_str = "mailbox-read"; break;
            case PDO::SyncManagerType::ProcessOutput: sm_type_str = "process-output (RxPDO)"; break;
            case PDO::SyncManagerType::ProcessInput:  sm_type_str = "process-input (TxPDO)"; break;
        }
        const char* mode_str = (config.control.mode == static_cast<uint8_t>(EtherCAT::SyncManager::SMMode::Mailbox)) ? "mailbox" :
                               (config.control.mode == static_cast<uint8_t>(EtherCAT::SyncManager::SMMode::Buffered)) ? "buffered" : "unknown";
        const char* dir_str  = config.control.direction ? "write (master→slave)" : "read (slave→master)";
        TETHER_LOGI(TAG, "  [PDO-DEBUG] SM%u detail: type=%s mode=%s dir=%s enable=%s",
                    sm_index, sm_type_str, mode_str, dir_str,
                    config.enable ? "yes" : "no");
        if (config.control.ecat_irq) TETHER_LOGI(TAG, "    IRQ eCAT enabled");
        if (config.control.pdi_irq) TETHER_LOGI(TAG, "    IRQ PDI enabled");
        if (config.control.watchdog) TETHER_LOGI(TAG, "    Watchdog enabled");
    }

    return true;
}

bool PDOManager::readSMStatus(uint16_t adp, uint8_t sm_index, uint8_t& status) {
    const uint16_t addr = static_cast<uint16_t>(sm_base_address(sm_index) + SM_OFF_STATUS);
    return transport_.readRegister(adp, addr, &status, sizeof(status), 200);
}

// ============================================================================
// SM Configuration public API
// ============================================================================

bool PDOManager::configureSlavesSMs(uint16_t slave_index) {
    if (slave_index >= PDO::kMaxPDOSlaves) {
        TETHER_LOGE(TAG, "Invalid slave index %u", slave_index);
        return false;
    }
    PDO::SlaveConfig& cfg = slave_configs_[slave_index];
    const uint16_t adp = transport_.adpForSlaveIndex(slave_index);

    TETHER_LOGI(TAG, "Configuring SMs for slave %u (adp=0x%04x)", slave_index, adp);

    if (rxPDODebug(slave_index) || txPDODebug(slave_index)) {
        TETHER_LOGI(TAG, "  [PDO-DEBUG] Slave %u config: vendor=0x%08x product=0x%08x",
                    slave_index, cfg.vendor_id, cfg.product_code);
        for (int sm = 0; sm < 4; sm++) {
            const char* sm_type_str = "unused";
            switch (cfg.sm[sm].type) {
                case PDO::SyncManagerType::MailboxWrite:  sm_type_str = "mailbox-write"; break;
                case PDO::SyncManagerType::MailboxRead:   sm_type_str = "mailbox-read"; break;
                case PDO::SyncManagerType::ProcessOutput: sm_type_str = "process-output (RxPDO)"; break;
                case PDO::SyncManagerType::ProcessInput:  sm_type_str = "process-input (TxPDO)"; break;
                default: break;
            }
            if (cfg.sm[sm].type != PDO::SyncManagerType::Unused) {
                TETHER_LOGI(TAG, "  [PDO-DEBUG]   SM%d: addr=0x%04x len=%u ctrl=0x%02x type=%s enable=%s",
                            sm, cfg.sm[sm].phys_start_addr, cfg.sm[sm].length,
                            cfg.sm[sm].control, sm_type_str,
                            cfg.sm[sm].enable ? "yes" : "no");
            } else {
                TETHER_LOGI(TAG, "  [PDO-DEBUG]   SM%d: unused", sm);
            }
        }
    }

    for (int sm = 0; sm < 4; sm++) {
        if (cfg.sm[sm].type != PDO::SyncManagerType::Unused) {
            if (!writeSMConfig(adp, static_cast<uint8_t>(sm), cfg.sm[sm], slave_index)) {
                TETHER_LOGE(TAG, "Failed to configure SM%d for slave %u", sm, slave_index);
                return false;
            }
        }
    }
    cfg.configured = true;
    return true;
}

uint16_t PDOManager::configureAllSlaveSMs(uint16_t slave_count) {
    uint16_t configured = 0;
    for (uint16_t i = 0; i < slave_count && i < PDO::kMaxPDOSlaves; i++) {
        if (configureSlavesSMs(i)) configured++;
    }
    slave_count_ = slave_count;
    TETHER_LOGI(TAG, "Configured %u/%u slaves", configured, slave_count);
    return configured;
}

// ============================================================================
// Mapping Finalization
// ============================================================================

bool PDOManager::finalizeMapping(uint16_t slave_index) {
    if (slave_index >= PDO::kMaxPDOSlaves) {
        TETHER_LOGE(TAG, "Invalid slave index %u", slave_index);
        return false;
    }
    PDO::SlaveConfig& cfg = slave_configs_[slave_index];
    const uint16_t sm2_addr = cfg.sm[2].phys_start_addr;
    const uint16_t sm3_addr = cfg.sm[3].phys_start_addr;

    TETHER_LOGI(TAG, "Finalizing PDO mapping for slave %u (SM2=0x%04X SM3=0x%04X)",
                slave_index, sm2_addr, sm3_addr);

    uint16_t total_rxpdo_size = 0;
    uint16_t total_txpdo_size = 0;
    size_t rxpdo_count = 0;
    size_t txpdo_count = 0;

    for (size_t i = 0; i < mapping_.entry_count(); i++) {
        PDO::PDOEntry* entry = mapping_.get_entry_mut(i);
        if (!entry || entry->slave_index != slave_index) continue;

        if (entry->direction == PDO::PDODirection::RxPDO) {
            entry->physical_offset = sm2_addr + total_rxpdo_size;
            total_rxpdo_size += entry->data_size;
            rxpdo_count++;
            if (rxPDODebug(slave_index)) {
                TETHER_LOGI(TAG, "  [RxPDO-DEBUG] Entry %zu: slave=%u offset=0x%04x size=%u buf=%p pdo=0x%04x",
                            i, slave_index, entry->physical_offset, entry->data_size,
                            entry->app_buffer, entry->pdo_index);
            }
        } else {
            entry->physical_offset = sm3_addr + total_txpdo_size;
            total_txpdo_size += entry->data_size;
            txpdo_count++;
            if (txPDODebug(slave_index)) {
                TETHER_LOGI(TAG, "  [TxPDO-DEBUG] Entry %zu: slave=%u offset=0x%04x size=%u buf=%p pdo=0x%04x",
                            i, slave_index, entry->physical_offset, entry->data_size,
                            entry->app_buffer, entry->pdo_index);
            }
        }
    }

    if (rxPDODebug(slave_index) || txPDODebug(slave_index)) {
        TETHER_LOGI(TAG, "  [PDO-DEBUG] Summary for slave %u: RxPDO entries=%zu total=%u bytes, TxPDO entries=%zu total=%u bytes",
                    slave_index, rxpdo_count, total_rxpdo_size, txpdo_count, total_txpdo_size);
    }

    if (total_rxpdo_size > 0 && cfg.sm[2].type != PDO::SyncManagerType::Unused) {
        cfg.sm[2].length = total_rxpdo_size;
        cfg.rxpdo_size   = total_rxpdo_size;
    }
    if (total_txpdo_size > 0 && cfg.sm[3].type != PDO::SyncManagerType::Unused) {
        cfg.sm[3].length = total_txpdo_size;
        cfg.txpdo_size   = total_txpdo_size;
    }
    return true;
}

// ============================================================================
// Transfer helpers (formerly in pdo_transfer.cpp)
// ============================================================================

bool PDOManager::sendRxPDOPosition(const PDO::PDOEntry& entry) {
    const uint16_t adp = transport_.adpForSlaveIndex(entry.slave_index);
    bool do_confirmed = ((transfer_stats_.rxpdo_debug_count % 100) == 0);

    if (do_confirmed) {
        const uint8_t idx = transport_.allocIdx();
        bool ok = transport_.sendSingleDatagram(
            Command::APWR, idx, adp, entry.physical_offset,
            entry.app_buffer, entry.data_size, true);
        if (ok) {
            RxDatagram resp;
            bool got = transport_.waitForResponseIdx(idx, 10, resp);
            if (got && resp.wkc > 0) {
                transfer_stats_.rxpdo_confirmed_ok++;
            } else {
                transfer_stats_.rxpdo_confirmed_fail++;
            }
        } else {
            transfer_stats_.rxpdo_confirmed_fail++;
        }
    } else {
        if (!transport_.sendSingleDatagram(
                Command::APWR, IPDOTransport::kFireAndForgetIdx,
                adp, entry.physical_offset,
                entry.app_buffer, entry.data_size, false)) {
            transfer_stats_.rxpdo_debug_count++;
            return false;
        }
    }
    transfer_stats_.rxpdo_debug_count++;
    return true;
}

bool PDOManager::recvTxPDOPosition(PDO::PDOEntry& entry) {
    const uint16_t adp = transport_.adpForSlaveIndex(entry.slave_index);
    transfer_stats_.txpdo_debug_count++;

    // Send APRD and use fire-and-forget — responses are best-effort.
    if (!transport_.sendSingleDatagram(
            Command::APRD, IPDOTransport::kFireAndForgetIdx,
            adp, entry.physical_offset, nullptr, entry.data_size, true)) {
        return false;
    }
    return true;
}

bool PDOManager::sendRxPDOConfigured(const PDO::PDOEntry& entry) {
    const uint8_t idx = transport_.allocIdx();
    if (!transport_.sendSingleDatagram(
            Command::FPWR, idx, entry.configured_address,
            entry.physical_offset, entry.app_buffer, entry.data_size, true)) {
        return false;
    }
    RxDatagram resp;
    return transport_.waitForResponseIdx(idx, 5, resp) && resp.wkc > 0;
}

bool PDOManager::recvTxPDOConfigured(PDO::PDOEntry& entry) {
    const uint8_t idx = transport_.allocIdx();
    if (!transport_.sendSingleDatagram(
            Command::FPRD, idx, entry.configured_address,
            entry.physical_offset, nullptr, entry.data_size, true)) {
        return false;
    }
    RxDatagram resp;
    if (!transport_.waitForResponseIdx(idx, 5, resp)) return false;
    if (resp.wkc == 0 || resp.datalen < entry.data_size) return false;
    std::memcpy(entry.app_buffer, resp.data, entry.data_size);
    return true;
}

bool PDOManager::sendRxPDOBroadcast(const PDO::PDOEntry& entry, uint16_t /*expected_wkc*/) {
    const uint8_t idx = transport_.allocIdx();
    if (!transport_.sendSingleDatagram(
            Command::BWR, idx, 0, entry.physical_offset,
            entry.app_buffer, entry.data_size, true)) {
        return false;
    }
    RxDatagram resp;
    if (!transport_.waitForResponseIdx(idx, 5, resp)) return false;
    return resp.wkc > 0;
}

bool PDOManager::recvTxPDOBroadcast(PDO::PDOEntry& entry, uint16_t /*expected_wkc*/) {
    const uint8_t idx = transport_.allocIdx();
    if (!transport_.sendSingleDatagram(
            Command::BRD, idx, 0, entry.physical_offset,
            nullptr, entry.data_size, true)) {
        return false;
    }
    RxDatagram resp;
    if (!transport_.waitForResponseIdx(idx, 5, resp)) return false;
    if (resp.datalen < entry.data_size) return false;
    std::memcpy(entry.app_buffer, resp.data, entry.data_size);
    return resp.wkc > 0;
}

// ============================================================================
// PDO Transfer public API (formerly pdo_api.cpp)
// ============================================================================

bool PDOManager::sendRxPDO(size_t entry_index) {
    const PDO::PDOEntry* entry = mapping_.get_entry(entry_index);
    if (!entry || !entry->enabled || entry->direction != PDO::PDODirection::RxPDO)
        return false;

    if (rxPDODebug(entry->slave_index)) {
        TETHER_LOGI(TAG, "[RxPDO-DEBUG] Sending entry %zu: slave=%u offset=0x%04x size=%u mode=%s",
                    entry_index, entry->slave_index, entry->physical_offset, entry->data_size,
                    entry->address_mode == PDO::PDOAddressMode::Position ? "position" :
                    entry->address_mode == PDO::PDOAddressMode::ConfiguredAddress ? "configured" :
                    entry->address_mode == PDO::PDOAddressMode::Broadcast ? "broadcast" :
                    entry->address_mode == PDO::PDOAddressMode::Logical ? "logical" : "unknown");
        if (entry->data_size <= 32) {
            char hex[128] = {0};
            size_t pos = 0;
            const uint8_t* buf = static_cast<const uint8_t*>(entry->app_buffer);
            for (uint16_t b = 0; b < entry->data_size && pos + 3 < sizeof(hex); b++) {
                pos += static_cast<size_t>(std::snprintf(hex + pos, sizeof(hex) - pos, "%02X ", buf[b]));
            }
            TETHER_LOGI(TAG, "  [RxPDO-DEBUG] Data: %s", hex);
        }
    }

    bool success = false;
    switch (entry->address_mode) {
        case PDO::PDOAddressMode::Position:
            success = sendRxPDOPosition(*entry);
            break;
        case PDO::PDOAddressMode::ConfiguredAddress:
            success = sendRxPDOConfigured(*entry);
            break;
        case PDO::PDOAddressMode::Broadcast:
            success = sendRxPDOBroadcast(*entry, 1);
            break;
        case PDO::PDOAddressMode::Logical:
            TETHER_LOGW(TAG, "Logical addressing not yet implemented");
            break;
    }

    if (rxPDODebug(entry->slave_index)) {
        TETHER_LOGI(TAG, "  [RxPDO-DEBUG] Result: %s (success_count=%u error_count=%u)",
                    success ? "OK" : "FAIL",
                    static_cast<unsigned>(entry->success_count + (success ? 1 : 0)),
                    static_cast<unsigned>(entry->error_count + (success ? 0 : 1)));
    }

    if (success) {
        const_cast<PDO::PDOEntry*>(entry)->success_count++;
        stats_.rxpdo_frames_sent++;
        if (entry->slave_index < PDO::kMaxPDOSlaves) {
            slave_configs_[entry->slave_index].pdo_request_count++;
        }
    } else {
        const_cast<PDO::PDOEntry*>(entry)->error_count++;
        stats_.rxpdo_errors++;
    }
    return success;
}

bool PDOManager::receiveTxPDO(size_t entry_index) {
    PDO::PDOEntry* entry = mapping_.get_entry_mut(entry_index);
    if (!entry || !entry->enabled || entry->direction != PDO::PDODirection::TxPDO)
        return false;

    if (txPDODebug(entry->slave_index)) {
        TETHER_LOGI(TAG, "[TxPDO-DEBUG] Receiving entry %zu: slave=%u offset=0x%04x size=%u mode=%s",
                    entry_index, entry->slave_index, entry->physical_offset, entry->data_size,
                    entry->address_mode == PDO::PDOAddressMode::Position ? "position" :
                    entry->address_mode == PDO::PDOAddressMode::ConfiguredAddress ? "configured" :
                    entry->address_mode == PDO::PDOAddressMode::Broadcast ? "broadcast" :
                    entry->address_mode == PDO::PDOAddressMode::Logical ? "logical" : "unknown");
    }

    bool success = false;
    switch (entry->address_mode) {
        case PDO::PDOAddressMode::Position:
            success = recvTxPDOPosition(*entry);
            break;
        case PDO::PDOAddressMode::ConfiguredAddress:
            success = recvTxPDOConfigured(*entry);
            break;
        case PDO::PDOAddressMode::Broadcast:
            success = recvTxPDOBroadcast(*entry, 1);
            break;
        case PDO::PDOAddressMode::Logical:
            TETHER_LOGW(TAG, "Logical addressing not yet implemented");
            break;
    }

    if (txPDODebug(entry->slave_index)) {
        TETHER_LOGI(TAG, "  [TxPDO-DEBUG] Result: %s (success_count=%u error_count=%u)",
                    success ? "OK" : "FAIL",
                    static_cast<unsigned>(entry->success_count + (success ? 1 : 0)),
                    static_cast<unsigned>(entry->error_count + (success ? 0 : 1)));
        if (success && entry->data_size <= 32) {
            char hex[128] = {0};
            size_t pos = 0;
            const uint8_t* buf = static_cast<const uint8_t*>(entry->app_buffer);
            for (uint16_t b = 0; b < entry->data_size && pos + 3 < sizeof(hex); b++) {
                pos += static_cast<size_t>(std::snprintf(hex + pos, sizeof(hex) - pos, "%02X ", buf[b]));
            }
            TETHER_LOGI(TAG, "  [TxPDO-DEBUG] Data: %s", hex);
        }
    }

    if (success) {
        entry->success_count++;
        stats_.txpdo_frames_recv++;
        if (entry->slave_index < PDO::kMaxPDOSlaves) {
            slave_configs_[entry->slave_index].pdo_reply_count++;
        }
    } else {
        entry->error_count++;
        stats_.txpdo_errors++;
    }
    return success;
}

// ============================================================================
// Callback mode (Mode 3)
// ============================================================================

void PDOManager::configureCallbackMode(const CallbackModeConfig& config) {
    mode_ = PDOMode::Callback;
    callback_config_ = config;
    callbacks_.resize(PDO::kMaxPDOEntries);
}

void PDOManager::setTxSentCallback(size_t entry_index, PDOTxSentCallback callback) {
    if (entry_index < callbacks_.size()) {
        callbacks_[entry_index].tx_sent = std::move(callback);
    }
}

void PDOManager::setRxReceivedCallback(size_t entry_index, PDORxReceivedCallback callback) {
    if (entry_index < callbacks_.size()) {
        callbacks_[entry_index].rx_received = std::move(callback);
    }
}

// ============================================================================
// Queue mode (Mode 2)
// ============================================================================

void PDOManager::configureQueueMode(const QueueModeConfig& config) {
    mode_ = PDOMode::Queue;
    queue_config_ = config;

    const size_t n = PDO::kMaxPDOEntries;
    tx_queues_.clear();
    rx_queues_.clear();
    tx_queues_.reserve(n);
    rx_queues_.reserve(n);
    for (size_t i = 0; i < n; i++) {
        tx_queues_.push_back(std::make_unique<FrameQueue>(config.tx_queue_capacity));
        rx_queues_.push_back(std::make_unique<FrameQueue>(config.rx_queue_capacity));
    }
    event_queue_ = std::make_unique<EventQueue>(config.event_queue_capacity);
    last_tx_frames_.resize(n);
    underrun_callback_ = nullptr;
}

bool PDOManager::enqueueTx(size_t entry_index, std::shared_ptr<PDOFrame> frame) {
    if (mode_ != PDOMode::Queue || entry_index >= tx_queues_.size() || !tx_queues_[entry_index])
        return false;
    return tx_queues_[entry_index]->try_push(std::move(frame));
}

bool PDOManager::tryDequeueRx(size_t entry_index, std::shared_ptr<PDOFrame>& frame) {
    if (mode_ != PDOMode::Queue || entry_index >= rx_queues_.size() || !rx_queues_[entry_index])
        return false;
    return rx_queues_[entry_index]->try_pop(frame);
}

bool PDOManager::tryPollEvent(std::shared_ptr<PDOEvent>& event) {
    if (mode_ != PDOMode::Queue || !event_queue_)
        return false;
    return event_queue_->try_pop(event);
}

void PDOManager::setUnderrunCallback(UnderrunCallback callback) {
    underrun_callback_ = std::move(callback);
}

bool PDOManager::queueCycle() {
    if (mode_ != PDOMode::Queue) return false;

    const uint64_t cycle_start_ns =
        static_cast<uint64_t>(Tether::Platform::Clock::instance().getMicroseconds()) * 1000ULL;

    // Phase 1: Drain TX queues into app buffers for RxPDO entries
    for (size_t i = 0; i < mapping_.entry_count(); i++) {
        PDO::PDOEntry* e = mapping_.get_entry_mut(i);
        if (!e || !e->enabled || e->direction != PDO::PDODirection::RxPDO)
            continue;

        std::shared_ptr<PDOFrame> frame;
        if (tx_queues_[i] && tx_queues_[i]->try_pop(frame)) {
            // Got new TX data — copy into app buffer
            if (frame->data.size() <= e->data_size) {
                std::memcpy(e->app_buffer, frame->data.data(), frame->data.size());
            }
            last_tx_frames_[i] = frame;
        } else {
            // Underrun — apply policy
            if (underrun_callback_) {
                underrun_callback_(e->slave_index, stats_.total_cycles);
            }
            // Push underrun event
            if (event_queue_) {
                auto ev = std::make_shared<PDOEvent>();
                ev->type = PDOEvent::Type::Underrun;
                ev->slave_index = e->slave_index;
                ev->pdo_entry_index = static_cast<uint16_t>(i);
                ev->timestamp_ns = cycle_start_ns;
                ev->cycle_count = stats_.total_cycles;
                event_queue_->try_push(std::move(ev));  // Drop if full
            }

            switch (queue_config_.underrun_policy) {
                case UnderrunPolicy::RepeatLastFrame:
                    if (last_tx_frames_[i] && last_tx_frames_[i]->data.size() <= e->data_size) {
                        std::memcpy(e->app_buffer, last_tx_frames_[i]->data.data(),
                                    last_tx_frames_[i]->data.size());
                    }
                    break;
                case UnderrunPolicy::SafeState:
                    if (queue_config_.safe_state_buffer.size() <= e->data_size) {
                        std::memcpy(e->app_buffer, queue_config_.safe_state_buffer.data(),
                                    queue_config_.safe_state_buffer.size());
                    }
                    break;
                case UnderrunPolicy::SkipCycle:
                    e->enabled = false;  // Temporarily disable for this cycle
                    break;
                case UnderrunPolicy::Custom:
                    // User callback already invoked above; no automatic action
                    break;
            }
        }
    }

    // Phase 2: Bus exchange
    bool ok = sendAll();
    ok = receiveAll() && ok;

    // Re-enable any entries we skipped for underrun
    for (size_t i = 0; i < mapping_.entry_count(); i++) {
        PDO::PDOEntry* e = mapping_.get_entry_mut(i);
        if (!e) continue;
        // Re-enable entries that were disabled by SkipCycle
        // (only if they were disabled by us, not the user)
        // We can't distinguish, so we re-enable all that have queue data
        if (e->direction == PDO::PDODirection::RxPDO && !e->enabled) {
            // Check if this was a SkipCycle disable by checking if queue has data now
            // Simplest: just re-enable — user disables are done via mapping API
            e->enabled = true;
        }
    }

    // Phase 3: Push received TxPDO data to RX queues and emit events
    for (size_t i = 0; i < mapping_.entry_count(); i++) {
        const PDO::PDOEntry* e = mapping_.get_entry(i);
        if (!e || !e->enabled || e->direction != PDO::PDODirection::TxPDO)
            continue;

        // Create RX frame
        auto frame = std::make_shared<PDOFrame>();
        frame->data.resize(e->data_size);
        std::memcpy(frame->data.data(), e->app_buffer, e->data_size);
        frame->timestamp_ns =
            static_cast<uint64_t>(Tether::Platform::Clock::instance().getMicroseconds()) * 1000ULL;
        frame->cycle_count = stats_.total_cycles;

        // try_push — drop if queue is full (user not consuming fast enough)
        if (rx_queues_[i]) {
            rx_queues_[i]->try_push(std::move(frame));
        }

        // Push RxReceived event
        if (event_queue_ && queue_config_.enable_rx_received_events) {
            auto ev = std::make_shared<PDOEvent>();
            ev->type = PDOEvent::Type::RxReceived;
            ev->slave_index = e->slave_index;
            ev->pdo_entry_index = static_cast<uint16_t>(i);
            ev->timestamp_ns =
                static_cast<uint64_t>(Tether::Platform::Clock::instance().getMicroseconds()) * 1000ULL;
            ev->cycle_count = stats_.total_cycles;
            event_queue_->try_push(std::move(ev));  // Drop if full
        }
    }

    // Push TxSent events for RxPDO entries
    if (queue_config_.enable_tx_sent_events) {
        for (size_t i = 0; i < mapping_.entry_count(); i++) {
            const PDO::PDOEntry* e = mapping_.get_entry(i);
            if (!e || !e->enabled || e->direction != PDO::PDODirection::RxPDO)
                continue;
            if (event_queue_) {
                auto ev = std::make_shared<PDOEvent>();
                ev->type = PDOEvent::Type::TxSent;
                ev->slave_index = e->slave_index;
                ev->pdo_entry_index = static_cast<uint16_t>(i);
                ev->timestamp_ns = cycle_start_ns;
                ev->cycle_count = stats_.total_cycles;
                event_queue_->try_push(std::move(ev));  // Drop if full
            }
        }
    }

    // Push error events if exchange failed
    if (!ok && event_queue_) {
        auto ev = std::make_shared<PDOEvent>();
        ev->type = PDOEvent::Type::Error;
        ev->timestamp_ns =
            static_cast<uint64_t>(Tether::Platform::Clock::instance().getMicroseconds()) * 1000ULL;
        ev->cycle_count = stats_.total_cycles;
        event_queue_->try_push(std::move(ev));
    }

    return ok;
}

bool PDOManager::sendAll() {
    // LRW path: atomic exchange, can't be split
    if (logical_addr_mgr_ && logical_addr_mgr_->isInitialized()) {
        split_state_.lrw_mode = true;
        split_state_.send_phase_ok = logical_addr_mgr_->exchangeAllLRW(mapping_);
        if (split_state_.send_phase_ok) {
            for (size_t i = 0; i < mapping_.entry_count(); i++) {
                const PDO::PDOEntry* e = mapping_.get_entry(i);
                if (!e || !e->enabled || e->slave_index >= PDO::kMaxPDOSlaves) continue;
                if (e->direction == PDO::PDODirection::RxPDO) {
                    slave_configs_[e->slave_index].pdo_request_count++;
                } else {
                    slave_configs_[e->slave_index].pdo_reply_count++;
                }
            }
        }
        return split_state_.send_phase_ok;
    }

    split_state_.lrw_mode = false;
    split_state_.send_phase_ok = true;
    split_state_.rx_confirmed_idxs.clear();
    split_state_.rx_confirmed_entry_idxs.clear();
    split_state_.rx_confirmed_slots.clear();
    split_state_.rx_confirmed_responses.clear();
    stats_.total_cycles++;

    if (rxPDODebug() || txPDODebug()) {
        TETHER_LOGI(TAG, "[PDO-DEBUG] === Cycle %llu ===", static_cast<unsigned long long>(stats_.total_cycles));
    }

    // Phase 1: Batch all RxPDO writes into one frame
    if (rxPDODebug()) {
        TETHER_LOGI(TAG, "  [RxPDO-DEBUG] --- Send phase (batched) ---");
    }

    std::vector<MultiDatagramSpec> rx_specs;
    std::vector<size_t> rx_entry_idx;
    std::vector<size_t> rx_confirmed_spec_map;
    size_t rx_skipped = 0;

    for (size_t i = 0; i < mapping_.entry_count(); i++) {
        const PDO::PDOEntry* e = mapping_.get_entry(i);
        if (!e || !e->enabled || e->direction != PDO::PDODirection::RxPDO)
            continue;

        Command cmd;
        uint16_t adp;
        uint8_t idx;
        bool roundtrip;

        switch (e->address_mode) {
            case PDO::PDOAddressMode::Position:
                cmd = Command::APWR;
                adp = transport_.adpForSlaveIndex(e->slave_index);
                idx = IPDOTransport::kFireAndForgetIdx;
                roundtrip = false;
                break;
            case PDO::PDOAddressMode::ConfiguredAddress:
                cmd = Command::FPWR;
                adp = e->configured_address;
                idx = transport_.allocIdx();
                roundtrip = true;
                break;
            case PDO::PDOAddressMode::Broadcast:
                cmd = Command::BWR;
                adp = 0;
                idx = transport_.allocIdx();
                roundtrip = true;
                break;
            default:
                rx_skipped++;
                continue;
        }

        rx_specs.push_back({cmd, idx, adp, e->physical_offset,
                           e->app_buffer, e->data_size, roundtrip});
        rx_entry_idx.push_back(i);
        if (roundtrip) {
            split_state_.rx_confirmed_idxs.push_back(idx);
            rx_confirmed_spec_map.push_back(rx_specs.size() - 1);
        }
    }

    if (!rx_specs.empty()) {
        // Pre-register response waiter slots for confirmed entries BEFORE
        // sending to avoid the send-then-register race: multiple datagrams
        // share one frame, so all responses arrive in one frame on the RX
        // thread.  If we register slots only after the send returns, later
        // responses arrive with no pending slot and are dropped ("unrouted").
        split_state_.rx_confirmed_responses.resize(
            split_state_.rx_confirmed_idxs.size());
        split_state_.rx_confirmed_slots.resize(
            split_state_.rx_confirmed_idxs.size(),
            IPDOTransport::kPreRegInvalid);
        for (size_t j = 0; j < split_state_.rx_confirmed_idxs.size(); j++) {
            auto& resp_buf = split_state_.rx_confirmed_responses[j];
            split_state_.rx_confirmed_slots[j] = transport_.preRegisterResponseWaiter(
                split_state_.rx_confirmed_idxs[j],
                resp_buf.data, sizeof(resp_buf.data));
        }

        size_t frames_sent = transport_.sendMultiDatagram(rx_specs.data(), rx_specs.size());
        if (frames_sent == 0) {
            for (size_t i : rx_entry_idx) {
                mapping_.get_entry_mut(i)->error_count++;
                stats_.rxpdo_errors++;
            }
            split_state_.send_phase_ok = false;
        } else {
            // Handle fire-and-forget entries (assume success if send succeeded)
            for (size_t j = 0; j < rx_specs.size(); j++) {
                if (rx_specs[j].idx == IPDOTransport::kFireAndForgetIdx) {
                    PDO::PDOEntry* e = mapping_.get_entry_mut(rx_entry_idx[j]);
                    e->success_count++;
                    stats_.rxpdo_frames_sent++;
                    if (e->slave_index < PDO::kMaxPDOSlaves)
                        slave_configs_[e->slave_index].pdo_request_count++;
                    // Callback mode: fire TxSent callback for fire-and-forget entries
                    if (mode_ == PDOMode::Callback && callback_config_.fire_on_tx_sent &&
                        rx_entry_idx[j] < callbacks_.size() && callbacks_[rx_entry_idx[j]].tx_sent) {
                        callbacks_[rx_entry_idx[j]].tx_sent(e->slave_index, stats_.total_cycles,
                            static_cast<uint64_t>(Tether::Platform::Clock::instance().getMicroseconds()) * 1000ULL);
                    }
                }
            }
            // Store confirmed entry indices for receiveAll() to wait on
            for (size_t j = 0; j < split_state_.rx_confirmed_idxs.size(); j++) {
                split_state_.rx_confirmed_entry_idxs.push_back(
                    rx_entry_idx[rx_confirmed_spec_map[j]]);
            }
        }
    }

    // If all enabled RxPDO entries were skipped (e.g. Logical addressing
    // is not implemented), mark the send phase as failed.
    if (rx_specs.empty() && rx_skipped > 0) {
        split_state_.send_phase_ok = false;
        for (size_t i = 0; i < mapping_.entry_count(); i++) {
            PDO::PDOEntry* e = mapping_.get_entry_mut(i);
            if (!e || !e->enabled || e->direction != PDO::PDODirection::RxPDO)
                continue;
            e->error_count++;
            stats_.rxpdo_errors++;
        }
    }

    // Debug gate checkpoint: first successful RxPDO send
    if (debug_gate_ && !first_rxpdo_emitted_ && split_state_.send_phase_ok) {
        first_rxpdo_emitted_ = true;
        debug_gate_->notifyCheckpoint("first-rxpdo");
    }

    return split_state_.send_phase_ok;
}

bool PDOManager::receiveAll() {
    // LRW path: already done in sendAll(), nothing to receive
    if (split_state_.lrw_mode) {
        return split_state_.send_phase_ok;
    }

    bool all_ok = split_state_.send_phase_ok;

    // Wait for confirmed RxPDO responses from sendAll()
    for (size_t j = 0; j < split_state_.rx_confirmed_idxs.size(); j++) {
        size_t entry_i = split_state_.rx_confirmed_entry_idxs[j];
        RxDatagram resp;
        bool got;
        if (j < split_state_.rx_confirmed_slots.size() &&
            split_state_.rx_confirmed_slots[j] != IPDOTransport::kPreRegInvalid) {
            got = transport_.waitForPreRegistered(
                split_state_.rx_confirmed_slots[j], 5, resp);
        } else {
            got = transport_.waitForResponseIdx(
                split_state_.rx_confirmed_idxs[j], 5, resp);
        }
        if (got && resp.wkc > 0) {
            PDO::PDOEntry* e = mapping_.get_entry_mut(entry_i);
            e->success_count++;
            stats_.rxpdo_frames_sent++;
            if (e->slave_index < PDO::kMaxPDOSlaves)
                slave_configs_[e->slave_index].pdo_request_count++;
            // Callback mode: fire TxSent callback for confirmed entries
            if (mode_ == PDOMode::Callback && callback_config_.fire_on_tx_sent &&
                entry_i < callbacks_.size() && callbacks_[entry_i].tx_sent) {
                callbacks_[entry_i].tx_sent(e->slave_index, stats_.total_cycles,
                    static_cast<uint64_t>(Tether::Platform::Clock::instance().getMicroseconds()) * 1000ULL);
            }
        } else {
            PDO::PDOEntry* e = mapping_.get_entry_mut(entry_i);
            e->error_count++;
            stats_.rxpdo_errors++;
            all_ok = false;
        }
    }
    split_state_.rx_confirmed_idxs.clear();
    split_state_.rx_confirmed_entry_idxs.clear();
    split_state_.rx_confirmed_slots.clear();
    split_state_.rx_confirmed_responses.clear();

    // Phase 2: Batch all TxPDO reads into one frame
    if (txPDODebug()) {
        TETHER_LOGI(TAG, "  [TxPDO-DEBUG] --- Receive phase (batched) ---");
    }

    std::vector<MultiDatagramSpec> tx_specs;
    std::vector<size_t> tx_entry_idx;
    std::vector<uint8_t> tx_idxs;
    std::vector<size_t> tx_spec_map;
    size_t tx_skipped = 0;

    for (size_t i = 0; i < mapping_.entry_count(); i++) {
        const PDO::PDOEntry* e = mapping_.get_entry(i);
        if (!e || !e->enabled || e->direction != PDO::PDODirection::TxPDO)
            continue;

        Command cmd;
        uint16_t adp;
        uint8_t idx;
        bool roundtrip;

        switch (e->address_mode) {
            case PDO::PDOAddressMode::Position:
                cmd = Command::APRD;
                adp = transport_.adpForSlaveIndex(e->slave_index);
                idx = IPDOTransport::kFireAndForgetIdx;
                roundtrip = true;  // APRD always roundtrip
                break;
            case PDO::PDOAddressMode::ConfiguredAddress:
                cmd = Command::FPRD;
                adp = e->configured_address;
                idx = transport_.allocIdx();
                roundtrip = true;
                break;
            case PDO::PDOAddressMode::Broadcast:
                cmd = Command::BRD;
                adp = 0;
                idx = transport_.allocIdx();
                roundtrip = true;
                break;
            default:
                tx_skipped++;
                continue;
        }

        tx_specs.push_back({cmd, idx, adp, e->physical_offset,
                           nullptr, e->data_size, roundtrip});
        tx_entry_idx.push_back(i);
        if (idx != IPDOTransport::kFireAndForgetIdx) {
            tx_idxs.push_back(idx);
            tx_spec_map.push_back(tx_specs.size() - 1);
        }
    }

    if (!tx_specs.empty()) {
        // Pre-register response waiter slots for confirmed TxPDO entries
        // BEFORE sending to avoid the send-then-register race.
        std::vector<size_t> tx_slots(tx_idxs.size(), IPDOTransport::kPreRegInvalid);
        std::vector<RxDatagram> tx_responses(tx_idxs.size());
        for (size_t j = 0; j < tx_idxs.size(); j++) {
            tx_slots[j] = transport_.preRegisterResponseWaiter(
                tx_idxs[j], tx_responses[j].data, sizeof(tx_responses[j].data));
        }

        size_t frames_sent = transport_.sendMultiDatagram(tx_specs.data(), tx_specs.size());
        if (frames_sent == 0) {
            // Cancel any pre-registered slots
            for (size_t j = 0; j < tx_slots.size(); j++) {
                if (tx_slots[j] != IPDOTransport::kPreRegInvalid)
                    transport_.waitForPreRegistered(tx_slots[j], 0, tx_responses[j]);
            }
            for (size_t i : tx_entry_idx) {
                mapping_.get_entry_mut(i)->error_count++;
                stats_.txpdo_errors++;
            }
            all_ok = false;
        } else {
            // Handle fire-and-forget entries (position mode - assume success)
            for (size_t j = 0; j < tx_specs.size(); j++) {
                if (tx_specs[j].idx == IPDOTransport::kFireAndForgetIdx) {
                    PDO::PDOEntry* e = mapping_.get_entry_mut(tx_entry_idx[j]);
                    e->success_count++;
                    stats_.txpdo_frames_recv++;
                    if (e->slave_index < PDO::kMaxPDOSlaves)
                        slave_configs_[e->slave_index].pdo_reply_count++;
                    // Callback mode: fire RxReceived callback for fire-and-forget entries
                    // (data not actually received for fire-and-forget, but callback signals cycle completion)
                    if (mode_ == PDOMode::Callback && callback_config_.fire_on_rx_received &&
                        tx_entry_idx[j] < callbacks_.size() && callbacks_[tx_entry_idx[j]].rx_received) {
                        callbacks_[tx_entry_idx[j]].rx_received(e->slave_index, static_cast<const uint8_t*>(e->app_buffer), e->data_size,
                            stats_.total_cycles,
                            static_cast<uint64_t>(Tether::Platform::Clock::instance().getMicroseconds()) * 1000ULL);
                    }
                }
            }
            // Wait for confirmed responses and copy data
            for (size_t j = 0; j < tx_idxs.size(); j++) {
                size_t entry_i = tx_entry_idx[tx_spec_map[j]];
                PDO::PDOEntry* e = mapping_.get_entry_mut(entry_i);
                RxDatagram resp;
                bool got;
                if (tx_slots[j] != IPDOTransport::kPreRegInvalid) {
                    got = transport_.waitForPreRegistered(tx_slots[j], 5, resp);
                } else {
                    got = transport_.waitForResponseIdx(tx_idxs[j], 5, resp);
                }
                if (got &&
                    resp.wkc > 0 && resp.datalen >= e->data_size) {
                    std::memcpy(e->app_buffer, resp.data, e->data_size);
                    e->success_count++;
                    stats_.txpdo_frames_recv++;
                    if (e->slave_index < PDO::kMaxPDOSlaves)
                        slave_configs_[e->slave_index].pdo_reply_count++;
                    // Callback mode: fire RxReceived callback for confirmed entries
                    if (mode_ == PDOMode::Callback && callback_config_.fire_on_rx_received &&
                        entry_i < callbacks_.size() && callbacks_[entry_i].rx_received) {
                        callbacks_[entry_i].rx_received(e->slave_index, static_cast<const uint8_t*>(e->app_buffer), e->data_size,
                            stats_.total_cycles,
                            static_cast<uint64_t>(Tether::Platform::Clock::instance().getMicroseconds()) * 1000ULL);
                    }
                } else {
                    e->error_count++;
                    stats_.txpdo_errors++;
                    all_ok = false;
                }
            }
        }
    }

    // If all enabled TxPDO entries were skipped (e.g. Logical addressing
    // is not implemented), mark the receive phase as failed.
    if (tx_specs.empty() && tx_skipped > 0) {
        all_ok = false;
        for (size_t i = 0; i < mapping_.entry_count(); i++) {
            PDO::PDOEntry* e = mapping_.get_entry_mut(i);
            if (!e || !e->enabled || e->direction != PDO::PDODirection::TxPDO)
                continue;
            e->error_count++;
            stats_.txpdo_errors++;
        }
    }

    if (rxPDODebug() || txPDODebug()) {
        TETHER_LOGI(TAG, "  [PDO-DEBUG] Cycle result: %s", all_ok ? "OK" : "ERRORS");
    }

    // Debug gate checkpoint: first successful TxPDO receive
    if (debug_gate_ && !first_txpdo_emitted_ && all_ok) {
        first_txpdo_emitted_ = true;
        debug_gate_->notifyCheckpoint("first-txpdo");
    }

    return all_ok;
}

bool PDOManager::exchangeAll() {
    if (!sendAll()) {
        // Even if send phase fails, still try to receive
        // (receiveAll handles the split_state_ correctly)
    }
    return receiveAll();
}

// ============================================================================
// Exchange modes (formerly pdo_logical.cpp)
// ============================================================================

bool PDOManager::exchangeLRW(uint16_t slave_count) {
    if (logical_addr_mgr_ && logical_addr_mgr_->isInitialized()) {
        return logical_addr_mgr_->exchangeAllLRW(mapping_);
    }
    if (slave_count == 0) return true;
    lrw_stats_.lrw_send_errors++;
    TETHER_LOGW(TAG, "LRW exchange requires FMMU; use master-level API");
    return false;
}

bool PDOManager::exchangeSeparate(uint16_t slave_count) {
    if (slave_count == 0) return true;
    separate_stats_.send_errors++;
    TETHER_LOGW(TAG, "Separate exchange requires FMMU; use master-level API");
    return false;
}

bool PDOManager::exchangePhysical(uint16_t slave_count) {
    if (slave_count == 0) return true;
    PDO::SlaveConfig* cfg = &slave_configs_[0];
    if (!cfg || !cfg->configured) return false;

    const auto& sm2 = cfg->sm[2];
    const auto& sm3 = cfg->sm[3];
    bool fpwr_ok = true, fprd_ok = true;

    // Build the RxPDO output buffer
    uint8_t out_buf[PDO::kMaxPDOSize] = {0};
    bool have_write = false;
    if (sm2.type == PDO::SyncManagerType::ProcessOutput && sm2.length > 0) {
        have_write = true;
        for (size_t i = 0; i < mapping_.entry_count(); i++) {
            const PDO::PDOEntry* e = mapping_.get_entry(i);
            if (e && e->enabled && e->direction == PDO::PDODirection::RxPDO
                && e->app_buffer && e->data_size <= sm2.length) {
                std::memcpy(out_buf, e->app_buffer, e->data_size);
            }
        }
        // Periodic hex dump: log RxPDO bytes every 1000 cycles
        // (skip cycle 0 — fpwr_success==0 makes 0%1000==0 which would
        //  fire every cycle until the first successful write)
        if ((physical_stats_.fpwr_success > 0 &&
             (physical_stats_.fpwr_success % 1000) == 0) || rxPDODebug()) {
            char hex[128];
            size_t pos = 0;
            size_t dump_len = sm2.length < 32 ? sm2.length : 32;
            for (size_t b = 0; b < dump_len && pos + 3 < sizeof(hex); b++) {
                pos += static_cast<size_t>(std::snprintf(hex + pos, sizeof(hex) - pos, "%02X ", out_buf[b]));
            }
            if (rxPDODebug()) {
                TETHER_LOGI(TAG, "[RxPDO-DEBUG] Physical write SM2: addr=0x%04x len=%u data=%s",
                            sm2.phys_start_addr, sm2.length, hex);
            } else {
                // Decode CW and TargetPos from the buffer for clarity
                uint16_t cw_val = static_cast<uint16_t>(out_buf[0] | (out_buf[1] << 8));
                int32_t tp_val = static_cast<int32_t>(out_buf[2] | (out_buf[3] << 8) | (out_buf[4] << 16) | (out_buf[5] << 24));
                TETHER_LOGI(TAG, "[RxPDO-WIRE] Cycle %u: CW=0x%04X TP=%ld | %s",
                         static_cast<unsigned>(physical_stats_.fpwr_success), cw_val, (long)tp_val, hex);
            }
        }
    }

    bool have_read = (sm3.type == PDO::SyncManagerType::ProcessInput && sm3.length > 0);

    // Build multi-datagram specs: pack FPWR + FPRD into one frame
    if (have_write && have_read) {
        // Batch both write and read into a single frame.
        // Use the slave's configured station address for FPWR/FPRD (not the
        // auto-increment position).  Fall back to adpForSlaveIndex if no
        // configured address was set.
        const uint16_t adp = (cfg->configured_address != 0)
                                 ? cfg->configured_address
                                 : transport_.adpForSlaveIndex(0);
        const uint8_t write_idx = transport_.allocIdx();
        const uint8_t read_idx = transport_.allocIdx();

        // Pre-register response waiter slots BEFORE sending to avoid the
        // send-then-register race: both datagrams share one frame, so both
        // responses arrive in one frame on the RX thread.  If we register
        // slots only after the send returns, the second response arrives
        // with no pending slot and is dropped ("unrouted"), causing a 50 ms
        // timeout per cycle.
        RxDatagram write_resp{};
        RxDatagram read_resp{};
        size_t write_slot = transport_.preRegisterResponseWaiter(
            write_idx, write_resp.data, sizeof(write_resp.data));
        size_t read_slot = transport_.preRegisterResponseWaiter(
            read_idx, read_resp.data, sizeof(read_resp.data));

        MultiDatagramSpec specs[2] = {
            {Command::FPWR, write_idx, adp, sm2.phys_start_addr, out_buf, sm2.length, true},
            {Command::FPRD, read_idx, adp, sm3.phys_start_addr, nullptr, sm3.length, true},
        };

        size_t frames_sent = transport_.sendMultiDatagram(specs, 2);
        if (frames_sent == 0) {
            if (write_slot < IPDOTransport::kPreRegInvalid)
                transport_.waitForPreRegistered(write_slot, 0, write_resp); // cancel
            if (read_slot < IPDOTransport::kPreRegInvalid)
                transport_.waitForPreRegistered(read_slot, 0, read_resp);  // cancel
            physical_stats_.fpwr_wkc_errors++;
            physical_stats_.fprd_wkc_errors++;
            return false;
        }

        // Wait for write response
        bool write_ok;
        if (write_slot < IPDOTransport::kPreRegInvalid) {
            write_ok = transport_.waitForPreRegistered(write_slot, 50, write_resp);
        } else {
            write_ok = transport_.waitForResponseIdx(write_idx, 50, write_resp);
        }
        if (write_ok && write_resp.wkc > 0) {
            physical_stats_.fpwr_success++;
        } else {
            physical_stats_.fpwr_wkc_errors++;
            fpwr_ok = false;
        }

        // Wait for read response
        bool read_ok;
        if (read_slot < IPDOTransport::kPreRegInvalid) {
            read_ok = transport_.waitForPreRegistered(read_slot, 50, read_resp);
        } else {
            read_ok = transport_.waitForResponseIdx(read_idx, 50, read_resp);
        }
        if (read_ok && read_resp.wkc > 0) {
            physical_stats_.fprd_success++;
            if (txPDODebug()) {
                char hex[128];
                size_t pos = 0;
                size_t dump_len = sm3.length < 32 ? sm3.length : 32;
                for (size_t b = 0; b < dump_len && pos + 3 < sizeof(hex); b++) {
                    pos += static_cast<size_t>(std::snprintf(hex + pos, sizeof(hex) - pos, "%02X ", read_resp.data[b]));
                }
                TETHER_LOGI(TAG, "[TxPDO-DEBUG] Physical read SM3: addr=0x%04x len=%u data=%s",
                            sm3.phys_start_addr, sm3.length, hex);
            }
            for (size_t i = 0; i < mapping_.entry_count(); i++) {
                PDO::PDOEntry* e = mapping_.get_entry_mut(i);
                if (e && e->enabled && e->direction == PDO::PDODirection::TxPDO
                    && e->app_buffer && e->data_size <= sm3.length) {
                    std::memcpy(e->app_buffer, read_resp.data, e->data_size);
                    e->success_count++;
                    if (txPDODebug(e->slave_index)) {
                        TETHER_LOGI(TAG, "  [TxPDO-DEBUG] Copied %u bytes to entry %zu buf=%p",
                                    e->data_size, i, e->app_buffer);
                    }
                }
            }
        } else {
            physical_stats_.fprd_wkc_errors++;
            fprd_ok = false;
            if (txPDODebug()) {
                TETHER_LOGI(TAG, "[TxPDO-DEBUG] Physical read SM3 FAILED: addr=0x%04x len=%u",
                            sm3.phys_start_addr, sm3.length);
            }
        }
    } else if (have_write) {
        // Write only — uses APWR via writeRegister (position-based addressing)
        if (transport_.writeRegister(transport_.adpForSlaveIndex(0),
                         sm2.phys_start_addr, out_buf, sm2.length, 50)) {
            physical_stats_.fpwr_success++;
        } else {
            physical_stats_.fpwr_wkc_errors++;
            fpwr_ok = false;
        }
    } else if (have_read) {
        // Read only — uses APRD via readRegister (position-based addressing)
        uint8_t in_buf[PDO::kMaxPDOSize] = {0};
        if (transport_.readRegister(transport_.adpForSlaveIndex(0),
                        sm3.phys_start_addr, in_buf, sm3.length, 50)) {
            physical_stats_.fprd_success++;
            if (txPDODebug()) {
                char hex[128];
                size_t pos = 0;
                size_t dump_len = sm3.length < 32 ? sm3.length : 32;
                for (size_t b = 0; b < dump_len && pos + 3 < sizeof(hex); b++) {
                    pos += static_cast<size_t>(std::snprintf(hex + pos, sizeof(hex) - pos, "%02X ", in_buf[b]));
                }
                TETHER_LOGI(TAG, "[TxPDO-DEBUG] Physical read SM3: addr=0x%04x len=%u data=%s",
                            sm3.phys_start_addr, sm3.length, hex);
            }
            for (size_t i = 0; i < mapping_.entry_count(); i++) {
                PDO::PDOEntry* e = mapping_.get_entry_mut(i);
                if (e && e->enabled && e->direction == PDO::PDODirection::TxPDO
                    && e->app_buffer && e->data_size <= sm3.length) {
                    std::memcpy(e->app_buffer, in_buf, e->data_size);
                    e->success_count++;
                    if (txPDODebug(e->slave_index)) {
                        TETHER_LOGI(TAG, "  [TxPDO-DEBUG] Copied %u bytes to entry %zu buf=%p",
                                    e->data_size, i, e->app_buffer);
                    }
                }
            }
        } else {
            physical_stats_.fprd_wkc_errors++;
            fprd_ok = false;
            if (txPDODebug()) {
                TETHER_LOGI(TAG, "[TxPDO-DEBUG] Physical read SM3 FAILED: addr=0x%04x len=%u",
                            sm3.phys_start_addr, sm3.length);
            }
        }
    }
    stats_.total_cycles++;
    return fpwr_ok && fprd_ok;
}

} // namespace EtherCAT

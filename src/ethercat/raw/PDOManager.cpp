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

#include <cstring>
#include <cstdio>

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
        TETHER_LOGE(TAG, "PDO entry limit reached");
        return -1;
    }
    if (!buffer || size == 0 || size > kMaxPDOSize) {
        TETHER_LOGE(TAG, "Invalid PDO buffer or size");
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
    if (debug::rxPDO()) {
        TETHER_LOGI(TAG, "  [RxPDO-DEBUG] Entry %zu: slave=%u buf=%p size=%u addr_mode=%s (%d)",
                    m_entry_count, slave_index, buffer, size,
                    mode == PDOAddressMode::Broadcast ? "broadcast" :
                    mode == PDOAddressMode::ConfiguredAddress ? "configured" :
                    mode == PDOAddressMode::Position ? "position" :
                    mode == PDOAddressMode::Logical ? "logical" : "unknown",
                    static_cast<int>(mode));
    }
    return static_cast<int>(m_entry_count++);
}

int PDOMapping::add_txpdo(uint16_t slave_index, void* buffer, uint16_t size,
                          uint16_t pdo_index, PDOAddressMode mode) {
    if (m_entry_count >= kMaxPDOEntries) {
        TETHER_LOGE(TAG, "PDO entry limit reached");
        return -1;
    }
    if (!buffer || size == 0 || size > kMaxPDOSize) {
        TETHER_LOGE(TAG, "Invalid PDO buffer or size");
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
    if (debug::txPDO()) {
        TETHER_LOGI(TAG, "  [TxPDO-DEBUG] Entry %zu: slave=%u buf=%p size=%u addr_mode=%s (%d)",
                    m_entry_count, slave_index, buffer, size,
                    mode == PDOAddressMode::Broadcast ? "broadcast" :
                    mode == PDOAddressMode::ConfiguredAddress ? "configured" :
                    mode == PDOAddressMode::Position ? "position" :
                    mode == PDOAddressMode::Logical ? "logical" : "unknown",
                    static_cast<int>(mode));
    }
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
                               const PDO::SyncManagerConfig& config)
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
    if (!transport_.writeRegister(adp, static_cast<uint16_t>(base + SM_OFF_CONTROL),
                                  &config.control, sizeof(config.control), 200)) {
        TETHER_LOGE(TAG, "SM%u: failed to write control=0x%02x", sm_index, config.control);
        return false;
    }

    // Step 5: Activate
    uint8_t activate = config.enable ? SM_ACT_ENABLE : 0x00;
    if (!transport_.writeRegister(adp, static_cast<uint16_t>(base + SM_OFF_ACTIVATE),
                                  &activate, sizeof(activate), 200)) {
        TETHER_LOGE(TAG, "SM%u: failed to write activate=0x%02x", sm_index, activate);
        return false;
    }

    TETHER_LOGI(TAG, "SM%u: configured addr=0x%04x len=%u ctrl=0x%02x act=0x%02x",
                sm_index, config.phys_start_addr, config.length, config.control, activate);

    if ((debug::rxPDO() && config.type == PDO::SyncManagerType::ProcessOutput) ||
        (debug::txPDO() && config.type == PDO::SyncManagerType::ProcessInput)) {
        const char* sm_type_str = "unknown";
        switch (config.type) {
            case PDO::SyncManagerType::Unused:        sm_type_str = "unused"; break;
            case PDO::SyncManagerType::MailboxWrite:  sm_type_str = "mailbox-write"; break;
            case PDO::SyncManagerType::MailboxRead:   sm_type_str = "mailbox-read"; break;
            case PDO::SyncManagerType::ProcessOutput: sm_type_str = "process-output (RxPDO)"; break;
            case PDO::SyncManagerType::ProcessInput:  sm_type_str = "process-input (TxPDO)"; break;
        }
        const char* mode_str = (config.control & 0x02) ? "mailbox" :
                               (config.control & 0x01) ? "buffered" : "unknown";
        const char* dir_str  = (config.control & 0x04) ? "write (master→slave)" : "read (slave→master)";
        TETHER_LOGI(TAG, "  [PDO-DEBUG] SM%u detail: type=%s mode=%s dir=%s enable=%s",
                    sm_index, sm_type_str, mode_str, dir_str,
                    config.enable ? "yes" : "no");
        if (config.control & 0x10) TETHER_LOGI(TAG, "    IRQ eCAT enabled");
        if (config.control & 0x20) TETHER_LOGI(TAG, "    IRQ PDI enabled");
        if (config.control & 0x40) TETHER_LOGI(TAG, "    Watchdog enabled");
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

    if (debug::rxPDO() || debug::txPDO()) {
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
            if (!writeSMConfig(adp, static_cast<uint8_t>(sm), cfg.sm[sm])) {
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
            if (debug::rxPDO()) {
                TETHER_LOGI(TAG, "  [RxPDO-DEBUG] Entry %zu: slave=%u offset=0x%04x size=%u buf=%p pdo=0x%04x",
                            i, slave_index, entry->physical_offset, entry->data_size,
                            entry->app_buffer, entry->pdo_index);
            }
        } else {
            entry->physical_offset = sm3_addr + total_txpdo_size;
            total_txpdo_size += entry->data_size;
            txpdo_count++;
            if (debug::txPDO()) {
                TETHER_LOGI(TAG, "  [TxPDO-DEBUG] Entry %zu: slave=%u offset=0x%04x size=%u buf=%p pdo=0x%04x",
                            i, slave_index, entry->physical_offset, entry->data_size,
                            entry->app_buffer, entry->pdo_index);
            }
        }
    }

    if (debug::rxPDO() || debug::txPDO()) {
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

    if (debug::rxPDO()) {
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

    if (debug::rxPDO()) {
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

    if (debug::txPDO()) {
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

    if (debug::txPDO()) {
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

bool PDOManager::exchangeAll() {
    if (logical_addr_mgr_ && logical_addr_mgr_->isInitialized()) {
        bool ok = logical_addr_mgr_->exchangeAllLRW(mapping_);
        if (ok) {
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
        return ok;
    }

    bool all_ok = true;
    stats_.total_cycles++;

    if (debug::rxPDO() || debug::txPDO()) {
        TETHER_LOGI(TAG, "[PDO-DEBUG] === Cycle %llu ===", static_cast<unsigned long long>(stats_.total_cycles));
    }

    // Phase 1: Send all RxPDO
    if (debug::rxPDO()) {
        TETHER_LOGI(TAG, "  [RxPDO-DEBUG] --- Send phase ---");
    }
    for (size_t i = 0; i < mapping_.entry_count(); i++) {
        const PDO::PDOEntry* e = mapping_.get_entry(i);
        if (e && e->enabled && e->direction == PDO::PDODirection::RxPDO) {
            if (!sendRxPDO(i)) all_ok = false;
        }
    }
    // Phase 2: Receive all TxPDO
    if (debug::txPDO()) {
        TETHER_LOGI(TAG, "  [TxPDO-DEBUG] --- Receive phase ---");
    }
    for (size_t i = 0; i < mapping_.entry_count(); i++) {
        const PDO::PDOEntry* e = mapping_.get_entry(i);
        if (e && e->enabled && e->direction == PDO::PDODirection::TxPDO) {
            if (!receiveTxPDO(i)) all_ok = false;
        }
    }

    if (debug::rxPDO() || debug::txPDO()) {
        TETHER_LOGI(TAG, "  [PDO-DEBUG] Cycle result: %s", all_ok ? "OK" : "ERRORS");
    }

    return all_ok;
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

    if (sm2.type == PDO::SyncManagerType::ProcessOutput && sm2.length > 0) {
        uint8_t out_buf[PDO::kMaxPDOSize] = {0};
        for (size_t i = 0; i < mapping_.entry_count(); i++) {
            const PDO::PDOEntry* e = mapping_.get_entry(i);
            if (e && e->enabled && e->direction == PDO::PDODirection::RxPDO
                && e->app_buffer && e->data_size <= sm2.length) {
                std::memcpy(out_buf, e->app_buffer, e->data_size);
            }
        }
        // Periodic hex dump: log RxPDO bytes every 1000 cycles
        if ((physical_stats_.fpwr_success % 1000) == 0 || debug::rxPDO()) {
            char hex[128];
            size_t pos = 0;
            size_t dump_len = sm2.length < 32 ? sm2.length : 32;
            for (size_t b = 0; b < dump_len && pos + 3 < sizeof(hex); b++) {
                pos += static_cast<size_t>(std::snprintf(hex + pos, sizeof(hex) - pos, "%02X ", out_buf[b]));
            }
            if (debug::rxPDO()) {
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
        if (transport_.writeRegister(transport_.adpForSlaveIndex(0),
                         sm2.phys_start_addr, out_buf, sm2.length, 50)) {
            physical_stats_.fpwr_success++;
        } else {
            physical_stats_.fpwr_wkc_errors++;
            fpwr_ok = false;
        }
    }

    if (sm3.type == PDO::SyncManagerType::ProcessInput && sm3.length > 0) {
        uint8_t in_buf[PDO::kMaxPDOSize] = {0};
        if (transport_.readRegister(transport_.adpForSlaveIndex(0),
                        sm3.phys_start_addr, in_buf, sm3.length, 50)) {
            physical_stats_.fprd_success++;
            if (debug::txPDO()) {
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
                    if (debug::txPDO()) {
                        TETHER_LOGI(TAG, "  [TxPDO-DEBUG] Copied %u bytes to entry %zu buf=%p",
                                    e->data_size, i, e->app_buffer);
                    }
                }
            }
        } else {
            physical_stats_.fprd_wkc_errors++;
            fprd_ok = false;
            if (debug::txPDO()) {
                TETHER_LOGI(TAG, "[TxPDO-DEBUG] Physical read SM3 FAILED: addr=0x%04x len=%u",
                            sm3.phys_start_addr, sm3.length);
            }
        }
    }
    stats_.total_cycles++;
    return fpwr_ok && fprd_ok;
}

} // namespace EtherCAT

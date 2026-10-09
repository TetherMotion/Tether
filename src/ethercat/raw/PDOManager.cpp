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
#include "tether/ethercat/ProcessImage.hpp"
#include "tether/ethercat/DebugFlags.hpp"
#include "tether/utils/ColoredBitsetFormatter.hpp"
#include "tether/platform/EspCompat.hpp"
#include "tether/platform/Platform.hpp"

#include <cstring>
#include <cstdio>
#include <bit>
#include <format>

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
// Backward-compatible free functions → delegate to PDOManager
// ============================================================================

namespace PDO {


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
    const auto* df = debug_flags_.load(std::memory_order_relaxed);
    if (df && df->shutdown) {
        TETHER_LOGI(TAG, "PDO subsystem deinitialized");
    }
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

void PDOManager::dumpStats(const char* tag) const {
    TETHER_LOGI(tag, "=== PDO Transfer Statistics ===");
    TETHER_LOGI(tag, "  [FMMU] total_cycles={} rx_sent={} tx_recv={}",
        static_cast<unsigned long long>(stats_.total_cycles),
        static_cast<unsigned long long>(stats_.rxpdo_frames_sent),
        static_cast<unsigned long long>(stats_.txpdo_frames_recv));
    TETHER_LOGI(tag, "  [FMMU] rx_errors={} tx_errors={} wkc_errors={}",
        stats_.rxpdo_errors, stats_.txpdo_errors, stats_.wkc_errors);
    TETHER_LOGI(tag,
        "  [PHYS] fpwr_ok={} fpwr_wkc_err={} fprd_ok={} fprd_wkc_err={}",
        physical_stats_.fpwr_success, physical_stats_.fpwr_wkc_errors,
        physical_stats_.fprd_success, physical_stats_.fprd_wkc_errors);
    TETHER_LOGI(tag, "  [PHYS] send_err={} timeout_err={}",
        physical_stats_.send_errors, physical_stats_.timeout_errors);
}

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
    TETHER_LOGI(TAG, "PDO mode set to: {}", v ? "SEPARATE (LRD+LWR)" : "COMBINED (LRW)");
}
bool PDOManager::getSeparateMode()  const { return use_separate_commands_; }
void PDOManager::setPhysicalMode(bool v) {
    use_physical_mode_ = v;
    TETHER_LOGI(TAG, "PDO mode set to: {}", v ? "PHYSICAL (FPWR+FPRD)" : "LOGICAL (FMMU-based)");
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

    // Control — keep the ESI watchdog-enable bit verbatim: firmware-driven
    // ESCs validate the SM control byte at the PRE_OP->SAFE_OP transition
    // and reject a cleared watchdog bit with AL 0x0017 "Invalid Sync
    // Manager configuration".  A latched SM watchdog at OP is handled by
    // the disarm fallback in Slave::transitionToOp().
    uint8_t ctrl_byte = std::bit_cast<uint8_t>(config.control);

    // Disable the SM first — the ESC rejects phys-addr/length writes on
    // an active channel (mailbox SMs may already be running from the
    // EEPROM bootstrap).
    uint8_t disable = 0x00;
    if (!transport_.writeRegister(adp, static_cast<uint16_t>(base + SM_OFF_ACTIVATE),
                                  &disable, sizeof(disable), 200)) {
        TETHER_LOGW(TAG, "SM{}: failed to disable", sm_index);
    }

    // Then write the configurable SM registers (phys addr + length +
    // control, bytes 0-4) in a single datagram — replaces the previous
    // per-field round-trips.  Byte 5 (status) is read-only and byte 7
    // (PDI control) must NOT be written: zeroing it breaks the SM
    // programming on firmware-driven ESCs (AL 0x0017 "Invalid Sync
    // Manager configuration" on the SAFE_OP request).  Activation
    // happens separately below so the channel comes up with a
    // fully-programmed block.
    uint8_t block[5] = {};
    const uint16_t addr_le = host_to_le16(config.phys_start_addr);
    const uint16_t len_le  = host_to_le16(config.length);
    std::memcpy(block + 0, &addr_le, sizeof(addr_le));
    std::memcpy(block + 2, &len_le, sizeof(len_le));
    block[4] = ctrl_byte;
    if (!transport_.writeRegister(adp, base, block, sizeof(block), 200)) {
        TETHER_LOGE(TAG, "SM{}: failed to write register block "
                         "(addr=0x{:04x} len={} ctrl=0x{:02x})",
                    sm_index, config.phys_start_addr, config.length, ctrl_byte);
        return false;
    }

    TETHER_LOGI(TAG, "{}: SM{}: configured addr=0x{:04x} len={} ctrl=0x{:02x} act=0x{:02x}",
                slavePrefix(slave_index).c_str(), sm_index, config.phys_start_addr, config.length, ctrl_byte,
                config.enable ? SM_ACT_ENABLE : 0x00);

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
        TETHER_LOGI(TAG, "  [PDO-DEBUG] SM{} detail: type={} mode={} dir={} enable={}",
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
        TETHER_LOGE(TAG, "Invalid slave index {}", slave_index);
        return false;
    }
    PDO::SlaveConfig& cfg = slave_configs_[slave_index];
    const uint16_t adp = transport_.adpForSlaveIndex(slave_index);

    TETHER_LOGI(TAG, "Configuring SMs for {} (adp=0x{:04x})", slavePrefix(slave_index).c_str(), adp);

    if (rxPDODebug(slave_index) || txPDODebug(slave_index)) {
        TETHER_LOGI(TAG, "  [PDO-DEBUG] {} config: vendor=0x{:08x} product=0x{:08x}",
                    slavePrefix(slave_index).c_str(), cfg.vendor_id, cfg.product_code);
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
                TETHER_LOGI(TAG, "  [PDO-DEBUG]   SM{}: addr=0x{:04x} len={} ctrl=0x{:02x} type={} enable={}",
                            sm, cfg.sm[sm].phys_start_addr, cfg.sm[sm].length,
                            std::bit_cast<uint8_t>(cfg.sm[sm].control), sm_type_str,
                            cfg.sm[sm].enable ? "yes" : "no");
            } else {
                TETHER_LOGI(TAG, "  [PDO-DEBUG]   SM{}: unused", sm);
            }
        }
    }

    for (int sm = 0; sm < 4; sm++) {
        if (cfg.sm[sm].type != PDO::SyncManagerType::Unused) {
            if (!writeSMConfig(adp, static_cast<uint8_t>(sm), cfg.sm[sm], slave_index)) {
                TETHER_LOGE(TAG, "Failed to configure SM{} for {}", sm, slavePrefix(slave_index).c_str());
                return false;
            }
        }
    }

    // Activate all enabled SMs in a single frame — one 1-byte APWR per
    // SM with pre-registered waiters, instead of one round-trip each.
    {
        uint8_t act_vals[4];
        MultiDatagramSpec specs[4];
        RxDatagram resps[4];
        size_t slots[4];
        size_t n = 0;
        bool prereg_ok = true;
        for (int sm = 0; sm < 4; ++sm) {
            if (cfg.sm[sm].type == PDO::SyncManagerType::Unused) continue;
            act_vals[sm] = cfg.sm[sm].enable ? SM_ACT_ENABLE : 0x00;
            const uint8_t idx = transport_.allocIdx();
            const size_t slot = transport_.preRegisterResponseWaiter(
                idx, resps[n].data, sizeof(resps[n].data));
            if (slot == IPDOTransport::kPreRegInvalid) { prereg_ok = false; break; }
            slots[n] = slot;
            specs[n] = {Command::APWR, idx, adp,
                        static_cast<uint16_t>(
                            sm_base_address(static_cast<uint8_t>(sm)) +
                            SM_OFF_ACTIVATE),
                        &act_vals[sm], 1, true};
            ++n;
        }
        if (prereg_ok && n > 0) {
            if (transport_.sendMultiDatagram(specs, n) == 0) {
                TETHER_LOGE(TAG, "SM activate frame send failed for {}",
                            slavePrefix(slave_index).c_str());
                return false;
            }
            for (size_t i = 0; i < n; ++i) {
                if (!transport_.waitForPreRegistered(slots[i], 200, resps[i])) {
                    TETHER_LOGE(TAG, "SM{}: activate not confirmed for {}",
                                i, slavePrefix(slave_index).c_str());
                    return false;
                }
            }
        } else if (!prereg_ok) {
            // Fallback: sequential activate writes.
            for (int sm = 0; sm < 4; ++sm) {
                if (cfg.sm[sm].type == PDO::SyncManagerType::Unused) continue;
                if (!transport_.writeRegister(
                        adp,
                        static_cast<uint16_t>(
                            sm_base_address(static_cast<uint8_t>(sm)) +
                            SM_OFF_ACTIVATE),
                        &act_vals[sm], 1, 200)) {
                    TETHER_LOGE(TAG, "SM{}: failed to activate for {}",
                                sm, slavePrefix(slave_index).c_str());
                    return false;
                }
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
    TETHER_LOGI(TAG, "Configured {}/{} slaves", configured, slave_count);
    return configured;
}

// ============================================================================
// Mapping Finalization
// ============================================================================

bool PDOManager::ensureConfiguredAddress(uint16_t slave_index) {
    if (slave_index >= PDO::kMaxPDOSlaves) return false;
    PDO::SlaveConfig& cfg = slave_configs_[slave_index];
    if (cfg.configured_address_known) return true;

    // Single APRD of the Configured Station Address register (0x0010).
    // The slave's configured station address is assigned during INIT (from
    // SII/EEPROM or by the master).  Without it, FPWR/FPRD transfers would
    // fall back to the auto-increment position address, which addressed
    // commands don't respond to after INIT.
    constexpr uint16_t kRegConfiguredStationAddress = 0x0010;
    const uint16_t adp = transport_.adpForSlaveIndex(slave_index);
    const uint8_t idx = transport_.allocIdx();
    if (!transport_.sendSingleDatagram(
            Command::APRD, idx, adp, kRegConfiguredStationAddress,
            nullptr, 2, /*roundtrip=*/true)) {
        TETHER_LOGW(TAG,
            "{}: failed to read configured station address (reg 0x0010) — "
            "FPWR/FPRD will use auto-increment fallback",
            slavePrefix(slave_index).c_str());
        return false;
    }
    RxDatagram resp;
    if (!transport_.waitForResponseIdx(idx, 200, resp) ||
        resp.wkc == 0 || resp.datalen < 2) {
        TETHER_LOGW(TAG,
            "{}: no response reading configured station address "
            "(reg 0x0010) — FPWR/FPRD will use auto-increment fallback",
            slavePrefix(slave_index).c_str());
        return false;
    }
    uint16_t cfg_addr;
    std::memcpy(&cfg_addr, resp.data, 2);
    if (cfg_addr == 0) {
        // Station address 0 is the unassigned default — with several slaves
        // sharing it, every FPWR/FPRD to that address is answered by all of
        // them (outputs collide, inputs mirror).  Assign a unique station
        // address via APWR (position addressing always works, even in INIT).
        const uint16_t assigned =
            static_cast<uint16_t>(0x1000 + slave_index);
        const uint8_t widx = transport_.allocIdx();
        if (transport_.sendSingleDatagram(
                Command::APWR, widx, adp, kRegConfiguredStationAddress,
                reinterpret_cast<const uint8_t*>(&assigned), 2,
                /*roundtrip=*/true)) {
            RxDatagram wresp;
            if (transport_.waitForResponseIdx(widx, 200, wresp) &&
                wresp.wkc > 0) {
                cfg_addr = assigned;
                TETHER_LOGI(TAG,
                    "{}: station address was 0 — assigned 0x{:04X}",
                    slavePrefix(slave_index).c_str(), cfg_addr);
            }
        }
    }
    cfg.configured_address       = cfg_addr;
    cfg.configured_address_known = true;
    mapping_.set_slave_configured_address(slave_index, cfg_addr);
    TETHER_LOGI(TAG, "{}: configured station address (reg 0x0010) = 0x{:04X}",
                slavePrefix(slave_index).c_str(), cfg_addr);
    return true;
}

bool PDOManager::finalizeMapping(uint16_t slave_index) {
    if (slave_index >= PDO::kMaxPDOSlaves) {
        TETHER_LOGE(TAG, "Invalid slave index {}", slave_index);
        return false;
    }
    // Make sure FPWR/FPRD transfers address the slave by its configured
    // station address — resolve it from reg 0x0010 when nobody set it.
    ensureConfiguredAddress(slave_index);

    PDO::SlaveConfig& cfg = slave_configs_[slave_index];
    const uint16_t sm2_addr = cfg.sm[2].phys_start_addr;
    const uint16_t sm3_addr = cfg.sm[3].phys_start_addr;

    TETHER_LOGI(TAG, "Finalizing PDO mapping for {} (SM2=0x{:04X} SM3=0x{:04X})",
                slavePrefix(slave_index).c_str(), sm2_addr, sm3_addr);

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
                TETHER_LOGI(TAG, "  [RxPDO-DEBUG] Entry {}: slave={} offset=0x{:04x} size={} buf={:p} pdo=0x{:04x}",
                            i, slave_index, entry->physical_offset, entry->data_size,
                            static_cast<const void*>(entry->storage), entry->pdo_index);
            }
        } else {
            entry->physical_offset = sm3_addr + total_txpdo_size;
            total_txpdo_size += entry->data_size;
            txpdo_count++;
            if (txPDODebug(slave_index)) {
                TETHER_LOGI(TAG, "  [TxPDO-DEBUG] Entry {}: slave={} offset=0x{:04x} size={} buf={:p} pdo=0x{:04x}",
                            i, slave_index, entry->physical_offset, entry->data_size,
                            static_cast<const void*>(entry->storage), entry->pdo_index);
            }
        }
    }

    if (rxPDODebug(slave_index) || txPDODebug(slave_index)) {
        TETHER_LOGI(TAG, "  [PDO-DEBUG] Summary for {}: RxPDO entries={} total={} bytes, TxPDO entries={} total={} bytes",
                    slavePrefix(slave_index).c_str(), rxpdo_count, total_rxpdo_size, txpdo_count, total_txpdo_size);
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
            entry.storage, entry.data_size, true);
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
                entry.storage, entry.data_size, false)) {
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
            entry.physical_offset, entry.storage, entry.data_size, true)) {
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
    std::memcpy(entry.storage, resp.data, entry.data_size);
    return true;
}

bool PDOManager::sendRxPDOBroadcast(const PDO::PDOEntry& entry, uint16_t /*expected_wkc*/) {
    const uint8_t idx = transport_.allocIdx();
    if (!transport_.sendSingleDatagram(
            Command::BWR, idx, 0, entry.physical_offset,
            entry.storage, entry.data_size, true)) {
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
    std::memcpy(entry.storage, resp.data, entry.data_size);
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
        TETHER_LOGI(TAG, "[RxPDO-DEBUG] Sending entry {}: slave={} offset=0x{:04x} size={} mode={}",
                    entry_index, entry->slave_index, entry->physical_offset, entry->data_size,
                    entry->address_mode == PDO::PDOAddressMode::Position ? "position" :
                    entry->address_mode == PDO::PDOAddressMode::ConfiguredAddress ? "configured" :
                    entry->address_mode == PDO::PDOAddressMode::Broadcast ? "broadcast" :
                    entry->address_mode == PDO::PDOAddressMode::Logical ? "logical" : "unknown");
        if (entry->data_size <= 32) {
            char hex[128] = {0};
            size_t pos = 0;
            const uint8_t* buf = entry->storage;
            for (uint16_t b = 0; b < entry->data_size && pos + 3 < sizeof(hex); b++) {
                pos += static_cast<size_t>(std::snprintf(hex + pos, sizeof(hex) - pos, "%02X ", buf[b]));
            }
            TETHER_LOGI(TAG, "  [RxPDO-DEBUG] Data: {}", hex);
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
        TETHER_LOGI(TAG, "  [RxPDO-DEBUG] Result: {} (success_count={} error_count={})",
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
        TETHER_LOGI(TAG, "[TxPDO-DEBUG] Receiving entry {}: slave={} offset=0x{:04x} size={} mode={}",
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
        TETHER_LOGI(TAG, "  [TxPDO-DEBUG] Result: {} (success_count={} error_count={})",
                    success ? "OK" : "FAIL",
                    static_cast<unsigned>(entry->success_count + (success ? 1 : 0)),
                    static_cast<unsigned>(entry->error_count + (success ? 0 : 1)));
        if (success && entry->data_size <= 32) {
            char hex[128] = {0};
            size_t pos = 0;
            const uint8_t* buf = entry->storage;
            for (uint16_t b = 0; b < entry->data_size && pos + 3 < sizeof(hex); b++) {
                pos += static_cast<size_t>(std::snprintf(hex + pos, sizeof(hex) - pos, "%02X ", buf[b]));
            }
            TETHER_LOGI(TAG, "  [TxPDO-DEBUG] Data: {}", hex);
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
                std::memcpy(e->storage, frame->data.data(), frame->data.size());
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
                        std::memcpy(e->storage, last_tx_frames_[i]->data.data(),
                                    last_tx_frames_[i]->data.size());
                    }
                    break;
                case UnderrunPolicy::SafeState:
                    if (queue_config_.safe_state_buffer.size() <= e->data_size) {
                        std::memcpy(e->storage, queue_config_.safe_state_buffer.data(),
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
        std::memcpy(frame->data.data(), e->storage, e->data_size);
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

        // A process image larger than one Ethernet frame cannot be sent
        // as a single LRW datagram.  Exchange it in contiguous logical-
        // address slices, split at the RxPDO/TxPDO boundary (partial
        // reads — see LogicalAddressManager::exchangeLRWSlice()).
        const uint32_t max_len = logical_addr_mgr_->maxSliceLength();
        const uint32_t total   = logical_addr_mgr_->totalLogicalSize();
        if (max_len != 0 && total > max_len) {
            const uint32_t rx_total = logical_addr_mgr_->totalRxPDOBytes();
            bool ok = true;
            for (uint32_t off = 0; off < total; ) {
                uint32_t len = std::min(max_len, total - off);
                if (off < rx_total && off + len > rx_total) len = rx_total - off;
                // exchangeLRWSlice() mirrors the per-slave exchange
                // counters for intersected entries.
                ok = exchangeLRWSlice(off, len) && ok;
                off += len;
            }
            split_state_.send_phase_ok = ok;
            return ok;
        }

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
        TETHER_LOGI(TAG, "[PDO-DEBUG] === Cycle {} ===", static_cast<unsigned long long>(stats_.total_cycles));
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
                           e->storage, e->data_size, roundtrip});
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
                        callbacks_[tx_entry_idx[j]].rx_received(e->slave_index, e->storage, e->data_size,
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
                    std::memcpy(e->storage, resp.data, e->data_size);
                    e->success_count++;
                    stats_.txpdo_frames_recv++;
                    if (e->slave_index < PDO::kMaxPDOSlaves)
                        slave_configs_[e->slave_index].pdo_reply_count++;
                    // Callback mode: fire RxReceived callback for confirmed entries
                    if (mode_ == PDOMode::Callback && callback_config_.fire_on_rx_received &&
                        entry_i < callbacks_.size() && callbacks_[entry_i].rx_received) {
                        callbacks_[entry_i].rx_received(e->slave_index, e->storage, e->data_size,
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
        TETHER_LOGI(TAG, "  [PDO-DEBUG] Cycle result: {}", all_ok ? "OK" : "ERRORS");
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

bool PDOManager::exchangeConfiguredSlaveWindows() {
    if (!logical_addr_mgr_ || !logical_addr_mgr_->isInitialized()) {
        return exchangeAll();
    }
    bool ok = true;
    for (uint16_t s = 0; s < PDO::kMaxPDOSlaves; ++s) {
        uint32_t offset = 0, length = 0;
        if (!logical_addr_mgr_->getSlaveLogicalWindow(s, offset, length)) {
            continue;
        }
        ok = exchangeLRWSlice(offset, length) && ok;
    }
    return ok;
}

bool PDOManager::exchangeLRWSlice(uint32_t offset, uint32_t length) {
    if (!logical_addr_mgr_ || !logical_addr_mgr_->isInitialized()) {
        TETHER_LOGW(TAG, "exchangeLRWSlice: no logical address manager");
        return false;
    }
    const bool ok = logical_addr_mgr_->exchangeLRWSlice(mapping_, offset, length);
    if (ok) {
        // Mirror sendAll()'s per-slave exchange counters so the OP-transition
        // "PDO exchange happened" check sees the traffic (req/reply > 0).
        const uint32_t end = offset + length;
        for (const auto& s : logical_addr_mgr_->describeEntries(mapping_)) {
            if (s.slave_index >= PDO::kMaxPDOSlaves) continue;
            if (s.offset >= end || s.offset + s.length <= offset) continue;
            if (s.direction == PDO::PDODirection::RxPDO)
                slave_configs_[s.slave_index].pdo_request_count++;
            else
                slave_configs_[s.slave_index].pdo_reply_count++;
        }
    }
    return ok;
}

bool PDOManager::exchangeAllLRWCyclic(uint32_t rx_timeout_ns,
                                      ProcessImage* image) {
    if (!logical_addr_mgr_ || !logical_addr_mgr_->isInitialized()) {
        return exchangeAll();
    }
    const bool ok = logical_addr_mgr_->exchangeAllLRWCyclic(
        mapping_, rx_timeout_ns, image);
    if (ok) {
        // Mirror the per-slave counters like exchangeLRWSlice() does.
        for (const auto& s : logical_addr_mgr_->describeEntries(mapping_)) {
            if (s.slave_index >= PDO::kMaxPDOSlaves) continue;
            if (s.direction == PDO::PDODirection::RxPDO)
                slave_configs_[s.slave_index].pdo_request_count++;
            else
                slave_configs_[s.slave_index].pdo_reply_count++;
        }
    }
    return ok;
}

bool PDOManager::cyclicSend(ProcessImage* image, uint32_t rx_timeout_ns) {
    if (!logical_addr_mgr_ || !logical_addr_mgr_->isInitialized()) {
        return exchangeAll();
    }
    return logical_addr_mgr_->cyclicSend(mapping_, image, rx_timeout_ns);
}

bool PDOManager::cyclicCollect(ProcessImage* image) {
    if (!logical_addr_mgr_ || !logical_addr_mgr_->isInitialized()) {
        return true;
    }
    const bool ok = logical_addr_mgr_->cyclicCollect(mapping_, image);
    if (ok) {
        for (const auto& s : logical_addr_mgr_->describeEntries(mapping_)) {
            if (s.slave_index >= PDO::kMaxPDOSlaves) continue;
            if (s.direction == PDO::PDODirection::RxPDO)
                slave_configs_[s.slave_index].pdo_request_count++;
            else
                slave_configs_[s.slave_index].pdo_reply_count++;
        }
    }
    return ok;
}

bool PDOManager::cyclicExchangePending() const {
    return logical_addr_mgr_ && logical_addr_mgr_->cyclicExchangePending();
}

uint32_t PDOManager::definePDOSlice(const PDOSliceSpec& spec) {
    if (!logical_addr_mgr_ || !logical_addr_mgr_->isInitialized())
        return 0xFFFFFFFFu;
    return logical_addr_mgr_->definePDOSlice(mapping_, spec);
}

bool PDOManager::clearPDOSlices() {
    return logical_addr_mgr_ && logical_addr_mgr_->clearPDOSlices();
}

size_t PDOManager::pdoSliceCount() const {
    return logical_addr_mgr_ ? logical_addr_mgr_->pdoSliceCount() : 0;
}

bool PDOManager::exchangePDOSlice(const PDOSliceSpec& spec) {
    if (!logical_addr_mgr_ || !logical_addr_mgr_->isInitialized())
        return false;
    return logical_addr_mgr_->exchangePDOSlice(mapping_, spec);
}

void PDOManager::setImageExchangeDecimation(uint32_t every_n) {
    if (logical_addr_mgr_)
        logical_addr_mgr_->setImageExchangeDecimation(every_n);
}

uint8_t PDOManager::cyclicSliceCount() const {
    return logical_addr_mgr_ ? logical_addr_mgr_->cyclicSliceCount() : 0;
}

void PDOManager::setCyclicStrictWkc(bool strict) {
    if (logical_addr_mgr_) logical_addr_mgr_->setStrictWkc(strict);
}

bool PDOManager::configureProcessImage(ProcessImage& image,
                                       ImageMode mode,
                                       const char* shm_name) {
    if (!logical_addr_mgr_ || !logical_addr_mgr_->isInitialized()) {
        return false;
    }
    int32_t offsets[ProcessImage::kMaxEntries];
    const size_t n = logical_addr_mgr_->computeImageOffsets(
        mapping_, offsets, ProcessImage::kMaxEntries);
    ProcessImage::Config cfg;
    cfg.mode          = mode;
    cfg.rx_bytes      = logical_addr_mgr_->totalRxPDOBytes();
    cfg.tx_bytes      = logical_addr_mgr_->totalTxPDOBytes();
    cfg.image_bytes   = logical_addr_mgr_->totalLogicalSize();
    cfg.entry_offsets = offsets;
    cfg.entry_count   = n;
    cfg.shm_name      = shm_name;
    return image.configure(cfg);
}

uint32_t PDOManager::maxLogicalSliceLength() const {
    if (!logical_addr_mgr_ || !logical_addr_mgr_->isInitialized()) return 0;
    return logical_addr_mgr_->maxSliceLength();
}

std::vector<PDO::LogicalEntrySlice> PDOManager::describeLogicalEntries() const {
    if (!logical_addr_mgr_ || !logical_addr_mgr_->isInitialized()) return {};
    return logical_addr_mgr_->describeEntries(mapping_);
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
    bool fpwr_ok = true, fprd_ok = true;

    // Iterate over all configured slaves (not just index 0).
    // slave_count limits the range of indices to scan.
    const uint16_t max_scan = (slave_count < PDO::kMaxPDOSlaves)
                                  ? slave_count : PDO::kMaxPDOSlaves;

    for (uint16_t si = 0; si < max_scan; si++) {
        PDO::SlaveConfig* cfg = &slave_configs_[si];
        if (!cfg || !cfg->configured) continue;

        const auto& sm2 = cfg->sm[2];
        const auto& sm3 = cfg->sm[3];

    // Build the RxPDO output buffer
    uint8_t out_buf[PDO::kMaxPDOSize] = {0};
    bool have_write = false;
    bool should_log_wire = false;
    uint32_t wire_cycle = 0;
    if (sm2.type == PDO::SyncManagerType::ProcessOutput && sm2.length > 0) {
        have_write = true;
        for (size_t i = 0; i < mapping_.entry_count(); i++) {
            const PDO::PDOEntry* e = mapping_.get_entry(i);
            if (e && e->enabled && e->slave_index == si
                && e->direction == PDO::PDODirection::RxPDO
                && e->data_size > 0 && e->data_size <= sm2.length
                && e->physical_offset >= sm2.phys_start_addr
                && e->physical_offset - sm2.phys_start_addr + e->data_size
                       <= sm2.length) {
                std::memcpy(out_buf + (e->physical_offset - sm2.phys_start_addr),
                            e->storage, e->data_size);
            }
        }
        // Periodic wire log: log interpreted RxPDO output every 1000 cycles
        // (skip cycle 0 — fpwr_success==0 makes 0%1000==0 which would
        //  fire every cycle until the first successful write)
        if (physical_stats_.fpwr_success > 0 &&
            (physical_stats_.fpwr_success % 1000) == 0) {
            should_log_wire = true;
            wire_cycle = physical_stats_.fpwr_success;
        }
        if (rxPDODebug()) {
            char hex[128];
            size_t pos = 0;
            size_t dump_len = sm2.length < 32 ? sm2.length : 32;
            for (size_t b = 0; b < dump_len && pos + 3 < sizeof(hex); b++) {
                pos += static_cast<size_t>(std::snprintf(hex + pos, sizeof(hex) - pos, "%02X ", out_buf[b]));
            }
            TETHER_LOGI(TAG, "[RxPDO-DEBUG] Physical write SM2 ({}): addr=0x{:04x} len={} data={}",
                        slavePrefix(si).c_str(), sm2.phys_start_addr, sm2.length, hex);
        }
    }

    bool have_read = (sm3.type == PDO::SyncManagerType::ProcessInput && sm3.length > 0);

    // Build multi-datagram specs: pack FPWR + FPRD into one frame
    if (have_write && have_read) {
        // Batch both write and read into a single frame.
        // Use the slave's configured station address for FPWR/FPRD (not the
        // auto-increment position).  The configured station address is read
        // from register 0x0010 during configureMultiPDOs().
        // Note: 0x0000 is a valid configured station address — only fall back
        // to auto-increment if the slave was not configured at all.
        const uint16_t adp = cfg->configured
                                 ? cfg->configured_address
                                 : transport_.adpForSlaveIndex(si);
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
            fpwr_ok = false;
            fprd_ok = false;
            continue;
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
                TETHER_LOGI(TAG, "[TxPDO-DEBUG] Physical read SM3 ({}): addr=0x{:04x} len={} data={}",
                            slavePrefix(si).c_str(), sm3.phys_start_addr, sm3.length, hex);
            }
            for (size_t i = 0; i < mapping_.entry_count(); i++) {
                PDO::PDOEntry* e = mapping_.get_entry_mut(i);
                if (e && e->enabled && e->slave_index == si
                    && e->direction == PDO::PDODirection::TxPDO
                    && e->data_size > 0 && e->data_size <= sm3.length
                    && e->physical_offset >= sm3.phys_start_addr
                    && e->physical_offset - sm3.phys_start_addr + e->data_size
                           <= sm3.length) {
                    std::memcpy(e->storage,
                                read_resp.data + (e->physical_offset - sm3.phys_start_addr),
                                e->data_size);
                    e->success_count++;
                    if (txPDODebug(e->slave_index)) {
                        TETHER_LOGI(TAG, "  [TxPDO-DEBUG] Copied {} bytes to entry {} buf={:p}",
                                    e->data_size, i, static_cast<const void*>(e->storage));
                    }
                }
            }
        } else {
            physical_stats_.fprd_wkc_errors++;
            fprd_ok = false;
            if (txPDODebug()) {
                TETHER_LOGI(TAG, "[TxPDO-DEBUG] Physical read SM3 FAILED ({}): addr=0x{:04x} len={}",
                            slavePrefix(si).c_str(), sm3.phys_start_addr, sm3.length);
            }
        }
        // Periodic interpreted RxPDO/TxPDO wire log
        if (should_log_wire && write_ok && (read_ok && read_resp.wkc > 0)) {
            static const Utils::BitLabel kCwLabels[] = {
                Utils::BitLabel::bit("SwOn",   0x0001),
                Utils::BitLabel::bit("EnV",    0x0002),
                Utils::BitLabel::bit("NoQS",   0x0004),
                Utils::BitLabel::bit("EnOp",   0x0008),
                Utils::BitLabel::bit("NewSP",  0x0010),
                Utils::BitLabel::bit("ChgSI",  0x0020),
                Utils::BitLabel::bit("AbsRel", 0x0040),
                Utils::BitLabel::bit("FltR",   0x0080),
                Utils::BitLabel::bit("Halt",   0x0100),
            };
            const std::span<const Utils::BitLabel> kCwSpan(kCwLabels);
            Utils::ColoredBitsetFormatter cw_fmt(kCwSpan);
            const uint32_t known_cw = Utils::ColoredBitsetFormatter::labelCoverage(kCwSpan);

            // Resolve the PDO indices actually assigned to this slave so the
            // decode matches the configured mapping (AS715N slave-defined
            // set: Rx 0x1701-0x1705, Tx 0x1B01-0x1B04).
            uint16_t rxpdo_index = 0;
            uint16_t txpdo_index = 0;
            for (size_t i = 0; i < mapping_.entry_count(); i++) {
                const PDO::PDOEntry* e = mapping_.get_entry(i);
                if (!e || !e->enabled || e->slave_index != si) continue;
                if (e->direction == PDO::PDODirection::RxPDO) rxpdo_index = e->pdo_index;
                else                                        txpdo_index = e->pdo_index;
            }

            // AS715N RxPDO layouts: 0x6071 TargetTorque and 0x6060
            // ModesOfOperation sit at different offsets (or are absent)
            // depending on the assigned PDO.  -1 = field not present.
            int tq_off = -1, opmode_off = -1;
            bool has_tv = true;
            switch (rxpdo_index) {
                case 0x1701: has_tv = false; break;                // CW,TP,TPF,DO
                case 0x1702:
                case 0x1704: tq_off = 10; opmode_off = 12; break;  // +TT,Mode,TPF,...
                case 0x1703:
                case 0x1705: opmode_off = 10; break;               // Mode,TPF,+/-TqLim,...
                default: break;
            }

            // Bounds-checked little-endian field readers.
            auto rd8 = [](const uint8_t* b, uint16_t len, int off) -> int8_t {
                return (off >= 0 && static_cast<uint16_t>(off) < len)
                    ? static_cast<int8_t>(b[off]) : static_cast<int8_t>(0);
            };
            auto rd16 = [](const uint8_t* b, uint16_t len, int off) -> uint16_t {
                return (off >= 0 && static_cast<uint16_t>(off) + 2 <= len)
                    ? static_cast<uint16_t>(b[off] | (b[off + 1] << 8))
                    : static_cast<uint16_t>(0);
            };
            auto rd32 = [](const uint8_t* b, uint16_t len, int off) -> int32_t {
                return (off >= 0 && static_cast<uint16_t>(off) + 4 <= len)
                    ? static_cast<int32_t>(b[off] | (b[off + 1] << 8) |
                                           (b[off + 2] << 16) | (b[off + 3] << 24))
                    : static_cast<int32_t>(0);
            };

            const uint16_t cw = rd16(out_buf, sm2.length, 0);
            const std::string cw_state = cw_fmt.format(cw, " ", known_cw);

            const int32_t tp = rd32(out_buf, sm2.length, 2);
            const int32_t tv = has_tv ? rd32(out_buf, sm2.length, 6) : 0;
            const int16_t tq = static_cast<int16_t>(rd16(out_buf, sm2.length, tq_off));
            const int8_t opmode = rd8(out_buf, sm2.length, opmode_off);

            TETHER_LOGI(TAG, "[RxPDO 0x{:04X}] {} Cycle {}:", rxpdo_index, slavePrefix(si).c_str(), wire_cycle);
            TETHER_LOGI(TAG, "  Controlword: {} (0x{:04X})", cw_state, cw);

            // Print the demand value matching the active operating mode plus
            // the raw setpoints.  PDOs without a mode field (e.g. 0x1701)
            // print the raw targets only.
            if (opmode_off >= 0 && opmode == 8) {
                TETHER_LOGI(TAG, "  Mode=CSP(8) DemandPosition={:>10} | TargetPosition={:>10} TargetVelocity={:>10} TargetTorque={:>6}",
                            tp, tp, tv, tq);
            } else if (opmode_off >= 0 && opmode == 9) {
                TETHER_LOGI(TAG, "  Mode=CSV(9) DemandVelocity={:>10} | TargetPosition={:>10} TargetVelocity={:>10} TargetTorque={:>6}",
                            tv, tp, tv, tq);
            } else if (opmode_off >= 0 && opmode == 10) {
                TETHER_LOGI(TAG, "  Mode=CST(10) DemandTorque={:>6} | TargetPosition={:>10} TargetVelocity={:>10} TargetTorque={:>6}",
                            tq, tp, tv, tq);
            } else if (opmode_off >= 0) {
                TETHER_LOGI(TAG, "  Mode={} | TargetPosition={:>10} TargetVelocity={:>10} TargetTorque={:>6}",
                            opmode, tp, tv, tq);
            } else {
                TETHER_LOGI(TAG, "  TargetPosition={:>10} TargetVelocity={:>10} TargetTorque={:>6}",
                            tp, tv, tq);
            }

            static const Utils::BitLabel kSwLabels[] = {
                Utils::BitLabel::bit("Rdy",     0x0001),
                Utils::BitLabel::bit("SwOn",    0x0002),
                Utils::BitLabel::bit("EnOp",    0x0004),
                Utils::BitLabel::bit("Flt",     0x0008),
                Utils::BitLabel::bit("EnV",     0x0010),
                Utils::BitLabel::bit("NoQS",    0x0020),
                Utils::BitLabel::bit("SwOnDsbl",0x0040),
                Utils::BitLabel::bit("Wrn",     0x0080),
                Utils::BitLabel::bit("Rem",     0x0200),
                Utils::BitLabel::bit("TgtRec",  0x0400),
                Utils::BitLabel::bit("IntLim",  0x0800),
                Utils::BitLabel::bit("SetAck",  0x1000),
                Utils::BitLabel::bit("FolErr",  0x2000),
            };
            const std::span<const Utils::BitLabel> kSwSpan(kSwLabels);
            Utils::ColoredBitsetFormatter sw_fmt(kSwSpan);
            const uint32_t known_sw = Utils::ColoredBitsetFormatter::labelCoverage(kSwSpan);
            const uint16_t sw = rd16(read_resp.data, sm3.length, 2);
            const std::string sw_state = sw_fmt.format(sw, " ", known_sw);

            const int32_t ap = rd32(read_resp.data, sm3.length, 4);
            const int16_t at = static_cast<int16_t>(rd16(read_resp.data, sm3.length, 8));
            // The trailing 4-byte slot is 0x606C speed feedback on 0x1B04
            // and 0x60FD digital inputs on 0x1B01/0x1B02/0x1B03.
            const bool last_is_di = (txpdo_index == 0x1B01 ||
                                     txpdo_index == 0x1B02 ||
                                     txpdo_index == 0x1B03);
            const int32_t last_field = rd32(read_resp.data, sm3.length,
                                            static_cast<int>(sm3.length) - 4);

            TETHER_LOGI(TAG, "[TxPDO 0x{:04X}] {} Cycle {}:", txpdo_index, slavePrefix(si).c_str(), wire_cycle);
            TETHER_LOGI(TAG, "  Statusword: {} (0x{:04X})", sw_state, sw);
            if (last_is_di) {
                TETHER_LOGI(TAG, "  ActualPosition={:>10} DigitalInputs=0x{:08X} ActualTorque={:>6}",
                            ap, static_cast<uint32_t>(last_field), at);
            } else {
                TETHER_LOGI(TAG, "  ActualPosition={:>10} ActualVelocity={:>10} ActualTorque={:>6}",
                            ap, last_field, at);
            }
        }
    } else if (have_write) {
        // Write only — uses APWR via writeRegister (position-based addressing)
        if (transport_.writeRegister(transport_.adpForSlaveIndex(si),
                         sm2.phys_start_addr, out_buf, sm2.length, 50)) {
            physical_stats_.fpwr_success++;
        } else {
            physical_stats_.fpwr_wkc_errors++;
            fpwr_ok = false;
        }
    } else if (have_read) {
        // Read only — uses APRD via readRegister (position-based addressing)
        uint8_t in_buf[PDO::kMaxPDOSize] = {0};
        if (transport_.readRegister(transport_.adpForSlaveIndex(si),
                        sm3.phys_start_addr, in_buf, sm3.length, 50)) {
            physical_stats_.fprd_success++;
            if (txPDODebug()) {
                char hex[128];
                size_t pos = 0;
                size_t dump_len = sm3.length < 32 ? sm3.length : 32;
                for (size_t b = 0; b < dump_len && pos + 3 < sizeof(hex); b++) {
                    pos += static_cast<size_t>(std::snprintf(hex + pos, sizeof(hex) - pos, "%02X ", in_buf[b]));
                }
                TETHER_LOGI(TAG, "[TxPDO-DEBUG] Physical read SM3 ({}): addr=0x{:04x} len={} data={}",
                            slavePrefix(si).c_str(), sm3.phys_start_addr, sm3.length, hex);
            }
            for (size_t i = 0; i < mapping_.entry_count(); i++) {
                PDO::PDOEntry* e = mapping_.get_entry_mut(i);
                if (e && e->enabled && e->slave_index == si
                    && e->direction == PDO::PDODirection::TxPDO
                    && e->data_size > 0 && e->data_size <= sm3.length
                    && e->physical_offset >= sm3.phys_start_addr
                    && e->physical_offset - sm3.phys_start_addr + e->data_size
                           <= sm3.length) {
                    std::memcpy(e->storage,
                                in_buf + (e->physical_offset - sm3.phys_start_addr),
                                e->data_size);
                    e->success_count++;
                    if (txPDODebug(e->slave_index)) {
                        TETHER_LOGI(TAG, "  [TxPDO-DEBUG] Copied {} bytes to entry {} buf={:p}",
                                    e->data_size, i, static_cast<const void*>(e->storage));
                    }
                }
            }
        } else {
            physical_stats_.fprd_wkc_errors++;
            fprd_ok = false;
            if (txPDODebug()) {
                TETHER_LOGI(TAG, "[TxPDO-DEBUG] Physical read SM3 FAILED ({}): addr=0x{:04x} len={}",
                            slavePrefix(si).c_str(), sm3.phys_start_addr, sm3.length);
            }
        }
    }
    }  // end for each slave
    stats_.total_cycles++;
    return fpwr_ok && fprd_ok;
}

} // namespace EtherCAT

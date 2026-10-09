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
#include "raw/PDOManagerInternal.hpp"

#include <cstring>
#include <cstdio>
#include <bit>
#include <format>

namespace EtherCAT {

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

} // namespace EtherCAT

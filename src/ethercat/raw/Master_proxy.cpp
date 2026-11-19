/**
 * @file Master_proxy.cpp
 * @brief Master — external-source PDO forwarding ("EtherCAT proxy").
 *
 * Owns the runtime plumbing for ProxyForwardConfig:
 *   - config validation (band inside the async range, interface name
 *     present, no UDP encapsulation),
 *   - the allocIdx() band exclusion (packed into proxy_idx_band_),
 *   - rewriting the wire socket's BPF so the band stays disjoint from
 *     the async traffic set (SO_ATTACH_FILTER replaces whole programs —
 *     the fastpath-exclusion and band-deny clauses are re-composed
 *     together), and
 *   - the ProxyForwarder lifecycle (sockets + pump thread).
 */

#include "tether/ethercat/Master.hpp"
#include "raw/ProxyForwarder.hpp"
#include "raw/WireBpfSpec.hpp"
#include "raw/CyclicDatapath.hpp"
#include "tether/ethercat/CBPFProgramFactory.hpp"
#include "tether/ethercat/LogicalAddressManager.hpp"
#include "logging/Logger.hpp"

#include <algorithm>
#include <cerrno>
#include <cstdint>
#include <cstring>
#include <format>
#include <vector>

#if defined(__linux__)
#include <linux/if_packet.h>
#include <net/if.h>
#include <sys/socket.h>
#endif

namespace EtherCAT {

static const char* TAG = "ecat_proxy";

namespace {

bool validateProxyConfig(const ProxyForwardConfig& c, std::string& err)
{
    if (c.ifname.empty()) {
        err = "empty proxy interface name";
        return false;
    }
    if (c.idx_count == 0) {
        err = "idx_count must be >= 1";
        return false;
    }
    const unsigned end =
        static_cast<unsigned>(c.idx_base) + c.idx_count - 1u;
    if (end >= kFastSlotBaseIdx) {
        err = std::format(
            "band [0x{:02X},0x{:02X}] overlaps the cyclic fastpath band "
            "[0x{:02X},0xFF]", c.idx_base, end,
            static_cast<unsigned>(kFastSlotBaseIdx));
        return false;
    }
    if (c.idx_count >= kFastSlotBaseIdx) {
        err = "band would consume the entire async idx space";
        return false;
    }
    if (c.filter && !c.filter->validate(err)) {
        err = "filter: " + err;
        return false;
    }
    return true;
}

constexpr uint16_t packProxyBand(const ProxyForwardConfig& c)
{
    return static_cast<uint16_t>(
        (static_cast<uint16_t>(c.end()) << 8) | c.idx_base);
}

/// Membership test for the small selection vectors.
bool selected(const std::vector<uint16_t>& v, uint16_t x)
{
    return std::find(v.begin(), v.end(), x) != v.end();
}

/// Append one window for PDO entry `e` honouring the selection.
void harvestPdoEntry(const LogicalAddressManager::PDOLogicalAddrEntry& e,
                     const ProxyPdoSelection& sel, ProxyPdoFilter& out)
{
    if (e.length == 0) return;
    const bool out_dir = e.is_output;
    if ( out_dir && !sel.include_outputs) return;
    if (!out_dir && !sel.include_inputs)  return;
    if (!sel.pdo_indexes.empty() &&
        !selected(sel.pdo_indexes, e.pdo_index)) return;
    out.access.push_back(ProxyPdoAccessRule{
        e.logical_addr, e.length,
        sel.bidirectional || !out_dir,     // read
        sel.bidirectional || out_dir});    // write
}

/// Copy every selected PDO window of one LAM into the filter.
void harvestLam(const LogicalAddressManager& lam,
                const ProxyPdoSelection& sel, ProxyPdoFilter& out)
{
    for (uint16_t s = 0; s < PDO::kMaxPDOSlaves; ++s) {
        if (!sel.slaves.empty() && !selected(sel.slaves, s)) continue;
        auto entries = lam.getSlavePDOLogicalAddrs(s);
        if (entries.empty() && lam.hasSlavePDOs(s)) {
            // Legacy single-PDO path: no per-PDO entries — synthesize
            // windows from the aggregate Rx/Tx regions.  pdo_index 0
            // means index selections never match these.
            LogicalAddressManager::PDOLogicalAddrEntry rx, tx;
            rx.is_output    = true;
            rx.logical_addr = lam.getRxPDOLogicalAddr(s);
            rx.length       = lam.getRxPDOLength(s);
            tx.is_output    = false;
            tx.logical_addr = lam.getTxPDOLogicalAddr(s);
            tx.length       = lam.getTxPDOLength(s);
            entries.push_back(rx);
            entries.push_back(tx);
        }
        for (const auto& e : entries) harvestPdoEntry(e, sel, out);
    }
}

} // namespace

// ============================================================================
// Runtime API
// ============================================================================

bool Master::setProxyForwarding(const ProxyForwardConfig& cfg)
{
    if (cfg.enabled) {
        std::string err;
        if (!validateProxyConfig(cfg, err)) {
            TETHER_LOGE(TAG, "proxy forwarding rejected: {}", err);
            return false;
        }
    }
    std::lock_guard<std::mutex> lk(proxy_mtx_);
    config_.proxy = cfg;
    applyProxyConfigLocked();
    return true;
}

bool Master::isProxyForwardingEnabled() const
{
    std::lock_guard<std::mutex> lk(proxy_mtx_);
    return proxy_forwarder_ && proxy_forwarder_->running();
}

Master::ProxyForwardStats Master::proxyForwardingStats() const
{
    std::lock_guard<std::mutex> lk(proxy_mtx_);
    return proxy_forwarder_ ? proxy_forwarder_->stats()
                            : ProxyForwardStats{};
}

// ============================================================================
// Auto-generated access whitelist
// ============================================================================

ProxyPdoFilter Master::buildProxyPdoFilter(const ProxyPdoSelection& sel)
{
    ProxyPdoFilter out;
    harvestLam(logicalAddressManager(), sel, out);
    for (const auto& g : pdo_groups_) {
        if (g.lam) harvestLam(*g.lam, sel, out);
    }
    return out;
}

// ============================================================================
// Lifecycle plumbing
// ============================================================================

void Master::applyProxyConfig()
{
    std::lock_guard<std::mutex> lk(proxy_mtx_);
    applyProxyConfigLocked();
}

/**
 * Bring the running state in line with config_.proxy:
 * disabled → stop the forwarder, restore the wire filter, free the band.
 * enabled  → reserve the band for allocIdx, carve it out of the wire
 *            socket's accept set, then start the forwarder.
 */
void Master::applyProxyConfigLocked()
{
    const auto& p = config_.proxy;

    if (!p.enabled) {
        proxy_idx_band_.store(kProxyBandEmpty, std::memory_order_relaxed);
        if (proxy_forwarder_) {
            proxy_forwarder_->stop();
            proxy_forwarder_.reset();
            TETHER_LOGI(TAG, "proxy forwarding disabled");
        }
        if (running_.load(std::memory_order_acquire))
            attachProxyWireFilter(false);   // restore pre-proxy program
        return;
    }

    // The allocator exclusion applies even before start() — allocIdx
    // reads the packed band on every call.
    proxy_idx_band_.store(packProxyBand(p), std::memory_order_relaxed);

    if (!running_.load(std::memory_order_acquire)) {
        // Deferred: sockets + filter rewrites happen in start().
        return;
    }

#if defined(__linux__)
    if (isUdpEncapsulationEnabled()) {
        TETHER_LOGE(TAG, "proxy forwarding requires direct EtherCAT "
                    "(EtherCAT-over-UDP encapsulation is active)");
        proxy_idx_band_.store(kProxyBandEmpty, std::memory_order_relaxed);
        return;
    }

    const int wfd = proxyWireFd();
    if (wfd < 0) {
        TETHER_LOGE(TAG, "proxy forwarding needs an AF_PACKET wire socket "
                    "— interface has no native handle and no wire_fd");
        proxy_idx_band_.store(kProxyBandEmpty, std::memory_order_relaxed);
        return;
    }
    sockaddr_ll sll{};
    socklen_t sll_len = sizeof(sll);
    int ifindex = 0;
    if (::getsockname(wfd, reinterpret_cast<sockaddr*>(&sll),
                      &sll_len) == 0) {
        ifindex = sll.sll_ifindex;
    }
    if (ifindex <= 0) {
        TETHER_LOGE(TAG, "proxy forwarding: cannot resolve wire ifindex");
        proxy_idx_band_.store(kProxyBandEmpty, std::memory_order_relaxed);
        return;
    }

    // Runtime re-enable: drop the old forwarder before the filter rewrite.
    if (proxy_forwarder_) {
        proxy_forwarder_->stop();
        proxy_forwarder_.reset();
    }

    // Resolve the armed whitelist: an explicit filter wins; otherwise
    // auto-generate from the PDO map — deny-by-default when the map
    // has no PDO regions yet.
    const ProxyPdoFilter* eff = nullptr;
    ProxyPdoFilter autogen;
    if (p.filter) {
        eff = &*p.filter;
    } else {
        autogen = buildProxyPdoFilter(p.pdo_selection);
        eff = &autogen;
        TETHER_LOGI(TAG, "proxy access whitelist auto-generated: {} PDO "
                    "windows{}", autogen.access.size(),
                    autogen.access.empty()
                        ? " — no PDOs mapped, ALL injected frames will "
                          "be dropped (PDO map not configured yet?)"
                        : "");
    }

    // Carve the band out of the async socket BEFORE the forwarder comes
    // up — band frames arriving in the gap are dropped rather than
    // misrouted into the master's parser.
    attachProxyWireFilter(true);

    auto fwd = std::make_unique<ProxyForwarder>();
    if (!fwd->start(p, ifindex, config_.wire_encap, eff)) {
        TETHER_LOGE(TAG, "proxy forwarding failed to start: {}",
                    fwd->lastError());
        proxy_idx_band_.store(kProxyBandEmpty, std::memory_order_relaxed);
        attachProxyWireFilter(false);   // restore — no forwarder running
        return;
    }
    proxy_forwarder_ = std::move(fwd);
#else
    TETHER_LOGE(TAG, "proxy forwarding requires Linux AF_PACKET sockets");
    proxy_idx_band_.store(kProxyBandEmpty, std::memory_order_relaxed);
#endif
}

void Master::stopProxyForwarder()
{
    std::lock_guard<std::mutex> lk(proxy_mtx_);
    proxy_idx_band_.store(kProxyBandEmpty, std::memory_order_relaxed);
    if (proxy_forwarder_) {
        proxy_forwarder_->stop();
        proxy_forwarder_.reset();
    }
}

int Master::proxyWireFd() const
{
    if (config_.wire_fd >= 0) return config_.wire_fd;
    if (iface_.native_handle)
        return static_cast<int>(
            reinterpret_cast<intptr_t>(iface_.native_handle));
    return -1;
}

/**
 * Rewrite the wire socket's BPF for the current proxy state.  The four
 * combinations of (cyclic-mirror attached?, proxy active?) map to:
 *
 *   mirror + proxy : encap ∧ idx∉fastpath ∧ idx∉band
 *   mirror only    : encap ∧ idx∉fastpath            (unchanged mirror)
 *   proxy only     : encap ∧ idx∉band
 *   neither        : encap                           (startup program)
 */
void Master::attachProxyWireFilter(bool active)
{
#if defined(__linux__)
    const int fd = proxyWireFd();
    if (fd < 0) return;

    const auto& p = config_.proxy;
    CBPFSpec spec = wireEncapBpfSpec(config_.wire_encap);

    if (datapath_->asyncMirrorExpected()) {
        spec.first_idx_range   = fastPathIdxRange();
        spec.first_idx_exclude = true;
    }
    if (active) {
        const CBPFIdxRange band{p.idx_base, p.end()};
        if (spec.first_idx_range) {
            spec.first_idx_deny = band;
        } else {
            // No cyclic mirror — the band is the only idx clause.
            spec.first_idx_range   = band;
            spec.first_idx_exclude = true;
        }
    }

    const auto prog = CBPFProgramFactory::build(spec);
    if (prog.empty() || !CBPFProgramFactory::attach(fd, prog)) {
        // Soft failure: the forwarder still works — band frames may
        // additionally reach the async parser as unrouted strays.
        TETHER_LOGW(TAG, "wire-socket BPF rewrite failed ({}) — "
                    "proxy-band frames may reach the async parser",
                    std::strerror(errno));
    }
#endif
}

} // namespace EtherCAT

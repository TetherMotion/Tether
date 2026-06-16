/**
 * @file VLANRouter.cpp
 * @brief Implementation of the 802.1Q VLAN router for EtherCAT masters.
 */

#include "tether/ethercat/VLANRouter.hpp"
#include "tether/ethercat/Master.hpp"
#include "tether/ethercat/Types.hpp"
#include "tether/platform/Platform.hpp"

#include <algorithm>
#include <cstring>

namespace EtherCAT {

// ============================================================================
// Constants
// ============================================================================

static constexpr uint16_t kVlanEtherType = 0x8100u;
static constexpr uint16_t kEtherCATEtherType = 0x88A4u;
static constexpr size_t  kEthernetHeaderSize = 14u;  // dst(6) + src(6) + ethertype(2)
static constexpr size_t  kVlanTagSize = 4u;           // TPID(2) + TCI(2)

// ============================================================================
// Helpers
// ============================================================================

static inline uint16_t be16(uint16_t host)
{
    return __builtin_bswap16(host);
}

static inline uint16_t be16_from_raw(const uint8_t* p)
{
    return static_cast<uint16_t>((p[0] << 8) | p[1]);
}

static inline void be16_to_raw(uint16_t v, uint8_t* p)
{
    p[0] = static_cast<uint8_t>(v >> 8);
    p[1] = static_cast<uint8_t>(v & 0xFFu);
}

// ============================================================================
// Construction / Destruction
// ============================================================================

VLANRouter::VLANRouter()
    : deliver_([](Master* m, const uint8_t* d, size_t l) {
          m->handleRxFrame(d, l);
      })
{
}

VLANRouter::~VLANRouter() = default;

// ============================================================================
// Backend
// ============================================================================

void VLANRouter::setBackend(NetworkInterface* backend)
{
    std::lock_guard<std::mutex> lock(mutex_);
    backend_ = backend;
    rebuildInterfacesLocked();
}

// ============================================================================
// Master registry
// ============================================================================

void VLANRouter::addMaster(std::shared_ptr<Master> master,
                           VLANRange rx_range,
                           std::optional<uint16_t> tx_vlan)
{
    if (!master) return;

    std::lock_guard<std::mutex> lock(mutex_);

    // Reject the undefined sentinel in the regular registry
    if (rx_range.start == kUndefinedVlanId || rx_range.end == kUndefinedVlanId) {
        TETHER_LOGW("VLANRouter", "addMaster: kUndefinedVlanId (%u) is not allowed in regular entries. Use setUndefinedTarget() instead.",
                    kUndefinedVlanId);
        return;
    }

    // Reject inverted ranges
    if (rx_range.start > rx_range.end) {
        TETHER_LOGW("VLANRouter", "addMaster: inverted range (%u > %u) rejected",
                    rx_range.start, rx_range.end);
        return;
    }

    // Prevent duplicate registration of the same master pointer
    auto it = std::find_if(entries_.begin(), entries_.end(),
                           [&master](const InternalEntry& e) {
                               return e.master.get() == master.get();
                           });
    if (it != entries_.end()) {
        it->rx_vlan_range = rx_range;
        it->tx_vlan_id = tx_vlan;
        rebuildInterfacesLocked();
        return;
    }

    InternalEntry entry;
    entry.master = std::move(master);
    entry.rx_vlan_range = rx_range;
    entry.tx_vlan_id = tx_vlan;
    entries_.push_back(std::move(entry));
    rebuildInterfacesLocked();
}

void VLANRouter::addMaster(std::shared_ptr<Master> master,
                           std::optional<uint16_t> rx_vlan,
                           std::optional<uint16_t> tx_vlan)
{
    if (rx_vlan.has_value()) {
        addMaster(std::move(master), VLANRange{rx_vlan.value(), rx_vlan.value()}, tx_vlan);
    } else {
        addMaster(std::move(master), std::nullopt, tx_vlan);
    }
}

void VLANRouter::addMaster(std::shared_ptr<Master> master,
                             std::nullopt_t,
                             std::optional<uint16_t> tx_vlan)
{
    if (!master) return;

    std::lock_guard<std::mutex> lock(mutex_);

    auto it = std::find_if(entries_.begin(), entries_.end(),
                           [&master](const InternalEntry& e) {
                               return e.master.get() == master.get();
                           });
    if (it != entries_.end()) {
        it->rx_vlan_range = std::nullopt;
        it->tx_vlan_id = tx_vlan;
        rebuildInterfacesLocked();
        return;
    }

    InternalEntry entry;
    entry.master = std::move(master);
    entry.rx_vlan_range = std::nullopt;
    entry.tx_vlan_id = tx_vlan;
    entries_.push_back(std::move(entry));
    rebuildInterfacesLocked();
}

void VLANRouter::removeMaster(const Master* master)
{
    if (!master) return;

    std::lock_guard<std::mutex> lock(mutex_);

    // Also check undefined target
    if (undefined_target_.has_value() && undefined_target_->master.get() == master) {
        undefined_target_.reset();
        return;
    }

    auto it = std::find_if(entries_.begin(), entries_.end(),
                           [master](const InternalEntry& e) {
                               return e.master.get() == master;
                           });
    if (it != entries_.end()) {
        entries_.erase(it);
        rebuildInterfacesLocked();
    }
}

void VLANRouter::clearMasters()
{
    std::lock_guard<std::mutex> lock(mutex_);
    entries_.clear();
    undefined_target_.reset();
    backend_ = nullptr;
}

// ============================================================================
// Interface rebuild
// ============================================================================

void VLANRouter::rebuildInterfaceSend(NetworkInterface* iface, bool has_tx_vlan, uint16_t tx_vid)
{
    iface->send = [this, has_tx_vlan, tx_vid](const uint8_t* data, size_t len) -> bool {
        std::lock_guard<std::mutex> lk(mutex_);
        if (!backend_ || !backend_->send) return false;

        if (!has_tx_vlan) {
            return backend_->send(data, len);
        }

        // Encapsulate with 802.1Q tag
        if (len < kEthernetHeaderSize) {
            return false;
        }

        constexpr size_t kMaxFrame = 1518;
        uint8_t buf[kMaxFrame];
        if (len + kVlanTagSize > sizeof(buf)) {
            return false;
        }

        std::memcpy(buf, data, 12);
        be16_to_raw(kVlanEtherType, buf + 12);
        be16_to_raw(tx_vid & 0x0FFFu, buf + 14);
        std::memcpy(buf + 16, data + 12, len - 12);

        return backend_->send(buf, len + kVlanTagSize);
    };

    iface->receive = [](uint8_t*, size_t, size_t* out_len) -> bool {
        if (out_len) *out_len = 0;
        return false;
    };
}

void VLANRouter::rebuildInterfacesLocked()
{
    for (auto& entry : entries_) {
        const bool has_tx_vlan = entry.tx_vlan_id.has_value();
        const uint16_t tx_vid = has_tx_vlan ? entry.tx_vlan_id.value() : 0u;
        rebuildInterfaceSend(&entry.iface, has_tx_vlan, tx_vid);
    }

    if (undefined_target_.has_value()) {
        const bool has_tx_vlan = undefined_target_->tx_vlan_id.has_value();
        const uint16_t tx_vid = has_tx_vlan ? undefined_target_->tx_vlan_id.value() : 0u;
        rebuildInterfaceSend(&undefined_target_->iface, has_tx_vlan, tx_vid);
    }
}

// ============================================================================
// Per-master interface lookup
// ============================================================================

NetworkInterface* VLANRouter::networkInterfaceFor(const Master* master)
{
    if (!master) return nullptr;

    std::lock_guard<std::mutex> lock(mutex_);
    for (auto& entry : entries_) {
        if (entry.master.get() == master) {
            return &entry.iface;
        }
    }
    return nullptr;
}

NetworkInterface* VLANRouter::undefinedNetworkInterface() const
{
    std::lock_guard<std::mutex> lock(mutex_);
    if (undefined_target_.has_value()) {
        return const_cast<NetworkInterface*>(&undefined_target_->iface);
    }
    return nullptr;
}

// ============================================================================
// RX processing
// ============================================================================

void VLANRouter::processRxFrame(const uint8_t* data, size_t len)
{
    if (!data || len < kEthernetHeaderSize) return;

    const uint16_t ether_type = be16_from_raw(data + 12);

    std::vector<InternalEntry> targets;
    std::shared_ptr<Master> undefined_master;
    bool has_undefined = false;

    {
        std::lock_guard<std::mutex> lock(mutex_);
        targets.reserve(entries_.size());

        if (ether_type == kVlanEtherType) {
            // 802.1Q tagged frame: need at least TPID + TCI
            if (len < kEthernetHeaderSize + kVlanTagSize) return;

            const uint16_t tci = be16_from_raw(data + 14);
            const uint16_t vid = tci & 0x0FFFu;

            // Collect all masters whose range contains this VID
            for (const auto& entry : entries_) {
                if (entry.rx_vlan_range.has_value() &&
                    entry.rx_vlan_range->contains(vid)) {
                    targets.push_back(entry);
                }
            }

            // If no range matches, check the dedicated undefined target
            if (targets.empty() && undefined_target_.has_value()) {
                undefined_master = undefined_target_->master;
                has_undefined = true;
            }
        } else {
            // Untagged frame
            for (const auto& entry : entries_) {
                if (!entry.rx_vlan_range.has_value()) {
                    targets.push_back(entry);
                }
            }
        }
    }

    // Decapsulate if needed and deliver outside the lock
    if (ether_type == kVlanEtherType) {
        if (len < kEthernetHeaderSize + kVlanTagSize) return;

        constexpr size_t kMaxFrame = 1518;
        uint8_t decap[kMaxFrame];
        if (len - kVlanTagSize > sizeof(decap)) return;

        // Reconstruct original frame: copy MACs, then original EtherType + payload
        std::memcpy(decap, data, 12);
        std::memcpy(decap + 12, data + 16, len - 16);
        const size_t decap_len = len - kVlanTagSize;

        if (has_undefined && undefined_master && deliver_) {
            deliver_(undefined_master.get(), decap, decap_len);
            return;
        }

        if (targets.empty()) {
            // Log warning for unhandled tagged frame
            const uint16_t inner_et = be16_from_raw(data + 16);
            const char* et_name = etherTypeName(inner_et);
            TETHER_LOGW("VLANRouter",
                        "Received tagged frame with VID %u, inner EtherType 0x%04X (%s) — no matching master or undefined target",
                        be16_from_raw(data + 14) & 0x0FFFu,
                        inner_et,
                        et_name ? et_name : "unknown");
            return;
        }

        for (const auto& entry : targets) {
            if (entry.master && deliver_) {
                deliver_(entry.master.get(), decap, decap_len);
            }
        }
    } else {
        for (const auto& entry : targets) {
            if (entry.master && deliver_) {
                deliver_(entry.master.get(), data, len);
            }
        }
    }
}

// ============================================================================
// Queries
// ============================================================================

std::vector<std::shared_ptr<Master>> VLANRouter::mastersForVlanId(uint16_t vlan_id) const
{
    std::vector<std::shared_ptr<Master>> result;
    std::lock_guard<std::mutex> lock(mutex_);
    for (const auto& entry : entries_) {
        if (entry.rx_vlan_range.has_value() && entry.rx_vlan_range->contains(vlan_id)) {
            result.push_back(entry.master);
        }
    }
    return result;
}

std::vector<VLANRouter::Entry> VLANRouter::entries() const
{
    std::vector<Entry> result;
    std::lock_guard<std::mutex> lock(mutex_);
    result.reserve(entries_.size());
    for (const auto& e : entries_) {
        result.push_back({e.master, e.rx_vlan_range, e.tx_vlan_id});
    }
    return result;
}

size_t VLANRouter::masterCount() const
{
    std::lock_guard<std::mutex> lock(mutex_);
    return entries_.size();
}

// ============================================================================
// EtherType name helper
// ============================================================================

bool VLANRouter::setUndefinedTarget(std::shared_ptr<Master> master,
                                     std::optional<uint16_t> tx_vlan,
                                     bool replace)
{
    if (!master) return false;

    std::lock_guard<std::mutex> lock(mutex_);
    if (undefined_target_.has_value() && !replace) {
        return false;
    }

    UndefinedTarget ut;
    ut.master = std::move(master);
    ut.tx_vlan_id = tx_vlan;
    undefined_target_ = std::move(ut);
    rebuildInterfacesLocked();
    return true;
}

void VLANRouter::clearUndefinedTarget()
{
    std::lock_guard<std::mutex> lock(mutex_);
    undefined_target_.reset();
}

std::shared_ptr<Master> VLANRouter::undefinedTarget() const
{
    std::lock_guard<std::mutex> lock(mutex_);
    if (undefined_target_.has_value()) {
        return undefined_target_->master;
    }
    return nullptr;
}

void VLANRouter::setDeliverFunction(std::function<void(Master*, const uint8_t*, size_t)> fn)
{
    std::lock_guard<std::mutex> lock(mutex_);
    deliver_ = std::move(fn);
}

const char* VLANRouter::etherTypeName(uint16_t ether_type)
{
    switch (ether_type) {
        case 0x0800: return "IPv4";
        case 0x0806: return "ARP";
        case 0x0842: return "WoL";
        case 0x22F3: return "IETF TRILL";
        case 0x22EA: return "Stream Reservation";
        case 0x6003: return "DECnet Phase IV";
        case 0x8035: return "RARP";
        case 0x809B: return "AppleTalk";
        case 0x80F3: return "AARP";
        case 0x8100: return "VLAN (802.1Q)";
        case 0x8204: return "QNX Qnet";
        case 0x86DD: return "IPv6";
        case 0x8808: return "Ethernet Flow Control";
        case 0x8809: return "Ethernet Slow Protocols (LACP)";
        case 0x8819: return "CobraNet";
        case 0x8847: return "MPLS unicast";
        case 0x8848: return "MPLS multicast";
        case 0x8863: return "PPPoE Discovery";
        case 0x8864: return "PPPoE Session";
        case 0x887B: return "HomePlug 1.0 MME";
        case 0x888E: return "EAPoL (802.1X)";
        case 0x8892: return "PROFINET";
        case 0x889A: return "HyperSCSI";
        case 0x88A2: return "ATAoE";
        case 0x88A4: return "EtherCAT";
        case 0x88A8: return "Provider Bridging (802.1ad)";
        case 0x88AB: return "EtherCAT Automation Protocol";
        case 0x88B8: return "GOOSE (IEC 61850)";
        case 0x88B9: return "GSE Management";
        case 0x88BA: return "SV (IEC 61850)";
        case 0x88BF: return "MikroTik RoMON";
        case 0x88CC: return "LLDP";
        case 0x88CD: return "SERCOS III";
        case 0x88E1: return "HomePlug AV MME";
        case 0x88E3: return "MRP (IEC 62439-2)";
        case 0x88E5: return "MACsec (802.1AE)";
        case 0x88E7: return "PBB (802.1ah)";
        case 0x88F7: return "PTP (IEEE 1588)";
        case 0x88F8: return "NC-SI";
        case 0x88FB: return "PRP (IEC 62439-3)";
        case 0x8902: return "IEEE 802.1ag CFM";
        case 0x8906: return "FCoE";
        case 0x8914: return "FCoE Initialization";
        case 0x8915: return "RoCE";
        case 0x891D: return "TTE";
        case 0x892F: return "HSR (IEC 62439-3)";
        case 0x8932: return "802.1Qbj MVRP";
        case 0x9000: return "Loopback";
        case 0x9100: return "Q-in-Q";
        default: return nullptr;
    }
}

} // namespace EtherCAT

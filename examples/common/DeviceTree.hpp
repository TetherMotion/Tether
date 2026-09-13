/**
 * @file DeviceTree.hpp
 * @brief Build a navigable TUI tree from the discovered EtherCAT topology
 *
 * Produces a TUI::TreeNode hierarchy for the beckhoff_* examples:
 * level 1 = coupler devices (EK1100, EK1101, ... — everything matching
 * the Beckhoff coupler family), expanded by default; level 2 = the
 * terminals downstream of each coupler, in bus order.  Slaves appearing
 * before the first coupler (or when no coupler is present at all) hang
 * under a synthetic "EtherCAT bus" root.
 *
 * Every node carries its slave index in `tag`, so the detail-pane hook
 * can map the selection back to driver state.
 *
 * Only available when the tether_terminal_ui component was built
 * (TETHER_HAS_TERMINAL_UI).
 */

#pragma once

#ifdef TETHER_HAS_TERMINAL_UI

#include <cstdint>
#include <functional>
#include <span>
#include <string>

#include "tether/ethercat/SlaveDiscoveryManager.hpp"
#include "tether/terminal_ui/TreeView.hpp"

namespace Tether {
namespace Examples {

/// Beckhoff vendor id.
inline constexpr uint32_t kBeckhoffVendor = 0x00000002;

/// Coupler detection: Beckhoff vendor, and either a name starting with "EK"
/// ("EK1100 EtherCAT Coupler") or the EK-family product-code marker
/// (low word 0x2C5x — e.g. EK1100 = 0x044C2C52).
inline bool isCouplerDevice(const EtherCAT::DiscoveredSlave& s) {
    const uint32_t vendor  = s.vendor_id ? *s.vendor_id : 0;
    const uint32_t product = s.product_code ? *s.product_code : 0;
    if (vendor != kBeckhoffVendor) return false;
    if (s.device_name) {
        const std::string& n = *s.device_name;
        if (n.size() >= 2 && n[0] == 'E' && n[1] == 'K') return true;
        if (n.find("Coupler") != std::string::npos) return true;
    }
    const uint16_t low = static_cast<uint16_t>(product & 0xFFFF);
    return low == 0x2C52 || low == 0x2C50;
}

/// One-line label for a slave: "s3 EL2004 4K. Dig. Ausgang 24V, 0.5A".
inline std::string slaveLabel(const EtherCAT::DiscoveredSlave& s) {
    std::string label = "s" + std::to_string(s.index) + " ";
    label += s.device_name ? *s.device_name : "?";
    return label;
}

/// Build the coupler/terminal tree.  `managed(slave_index)` decides the
/// node color: managed terminals get PalValue, couplers PalHeader.
inline TUI::TreeNode buildDeviceTree(
    std::span<const EtherCAT::DiscoveredSlave> slaves,
    const std::function<bool(uint16_t)>& managed) {
    TUI::TreeNode root;   // invisible root — children are level 1
    root.label = "bus";

    TUI::TreeNode* coupler = nullptr;
    for (const auto& s : slaves) {
        TUI::TreeNode node;
        node.label = slaveLabel(s);
        node.tag   = static_cast<int>(s.index);

        if (isCouplerDevice(s)) {
            node.color = TUI::PalHeader;
            root.children.push_back(std::move(node));
            coupler = &root.children.back();
        } else {
            node.color = managed(s.index) ? TUI::PalValue : TUI::PalNone;
            if (!coupler) {
                // Slaves before the first coupler get a synthetic root.
                TUI::TreeNode bus;
                bus.label = "EtherCAT bus";
                bus.tag   = -1;
                bus.color = TUI::PalHeader;
                root.children.push_back(std::move(bus));
                coupler = &root.children.back();
            }
            coupler->children.push_back(std::move(node));
        }
    }
    return root;
}

/// Find a slave by its bus index (for detail-pane identity info).
inline const EtherCAT::DiscoveredSlave* slaveByIndex(
    std::span<const EtherCAT::DiscoveredSlave> slaves, int index) {
    for (const auto& s : slaves) {
        if (s.index == static_cast<uint16_t>(index)) return &s;
    }
    return nullptr;
}

} // namespace Examples
} // namespace Tether

#endif // TETHER_HAS_TERMINAL_UI

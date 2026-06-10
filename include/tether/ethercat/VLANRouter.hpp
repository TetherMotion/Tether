#pragma once

/**
 * @file VLANRouter.hpp
 * @brief 802.1Q VLAN router for multiplexing multiple EtherCAT masters over a single physical interface.
 *
 * ## Purpose
 *
 * The VLANRouter sits between a raw Ethernet backend and one or more
 * `EtherCATMaster` instances.  It transparently inserts and strips 802.1Q
 * VLAN tags so that each master can operate on its own logical EtherCAT
 * segment without needing dedicated hardware.
 *
 * ## 802.1Q encapsulation format
 *
 * An untagged EtherCAT frame on the wire has the layout:
 *   [dst MAC (6)] [src MAC (6)] [EtherType 0x88A4 (2)] [ECAT payload ...]
 *
 * After VLAN encapsulation the layout becomes:
 *   [dst MAC (6)] [src MAC (6)] [TPID 0x8100 (2)] [TCI (2)] [EtherType 0x88A4 (2)] [ECAT payload ...]
 *
 * The Tag Control Information (TCI) word is:
 *   bits 15..13 : PCP (priority, currently fixed to 0)
 *   bit  12     : DEI (drop eligible, currently fixed to 0)
 *   bits 11..0  : VID (VLAN identifier, 0–4094)
 *
 * ## Usage example
 *
 * @code
 *   EtherCAT::VLANRouter router;
 *   router.setBackend(raw_ethernet_iface);
 *
 *   auto master_a = std::make_shared<EtherCAT::EtherCATMaster>();
 *   router.addMaster(master_a, 100, 100);   // VLAN 100 for both RX and TX
 *   master_a->start(*router.networkInterfaceFor(master_a.get()), src_mac);
 *
 *   auto master_b = std::make_shared<EtherCAT::EtherCATMaster>();
 *   router.addMaster(master_b, std::nullopt, std::nullopt); // No VLAN
 *   master_b->start(*router.networkInterfaceFor(master_b.get()), src_mac);
 *
 *   // On every received Ethernet frame:
 *   raw_eth->setRxCallback([&router](const uint8_t* f, size_t l, ...) {
 *       router.processRxFrame(f, l);
 *   }, nullptr);
 * @endcode
 *
 * ## VLAN ID sharing
 *
 * Multiple masters may share the same VLAN ID.  In that case every
 * matching master receives a copy of the decapsulated frame.  This is
 * fully supported but not demonstrated in the example above.
 *
 * ## Thread safety
 *
 * All mutating operations (`addMaster`, `removeMaster`) are protected by
 * an internal mutex and are safe to call concurrently with TX/RX traffic.
 * Read-only lookups (`networkInterfaceFor`, `mastersForVlanId`, `entries`)
 * are also mutex-protected and return snapshots where required.
 */

#include "tether/ethercat/EtherCATTypes.hpp"

#include <cstddef>
#include <cstdint>
#include <functional>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <vector>

namespace EtherCAT {

// Forward declarations
class EtherCATMaster;

/**
 * @brief 802.1Q VLAN router for multiplexing EtherCAT masters.
 *
 * The router owns a backend NetworkInterface (raw Ethernet) and exposes
 * per-master NetworkInterface views.  TX frames are optionally
 * encapsulated with an 802.1Q tag; RX frames are inspected, decapsulated,
 * and routed to every master whose rx_vlan_id matches the extracted VID.
 */
class VLANRouter {
public:
    /**
     * @brief Inclusive VLAN range used for RX matching.
     *
     * A single VLAN is represented as {vid, vid}.
     */
    struct VLANRange {
        uint16_t start;  ///< First VLAN ID in the range (inclusive)
        uint16_t end;    ///< Last VLAN ID in the range (inclusive)

        VLANRange(uint16_t s, uint16_t e) : start(s), end(e) {}

        bool contains(uint16_t vid) const {
            return vid >= start && vid <= end;
        }
    };

    /**
     * @brief Public snapshot of a registered master entry.
     */
    struct Entry {
        std::shared_ptr<EtherCATMaster> master;       ///< The registered master
        std::optional<VLANRange> rx_vlan_range;       ///< Expected RX VLAN range (nullopt = untagged)
        std::optional<uint16_t> tx_vlan_id;           ///< TX VLAN ID to insert (nullopt = pass-through)
    };

    /**
     * @brief Sentinel VLAN ID for the dedicated undefined target.
     *
     * Normal VLAN IDs are 12-bit (0–4095).  4096 is outside that range
     * and is reserved for the router's internal catch-all mechanism.
     * It is **not** accepted by addMaster() as an RX value; callers
     * must use setUndefinedTarget() instead.
     *
     * Untagged (non-802.1Q) frames are still routed to masters whose
     * rx_vlan_range is std::nullopt, regardless of this sentinel.
     */
    static constexpr uint16_t kUndefinedVlanId = 4096;

    /**
     * @brief Default constructor.  No backend is set.
     */
    VLANRouter();

    /**
     * @brief Destructor.  Clears all registered masters.
     */
    ~VLANRouter();

    // Non-copyable, movable
    VLANRouter(const VLANRouter&) = delete;
    VLANRouter& operator=(const VLANRouter&) = delete;
    VLANRouter(VLANRouter&&) = default;
    VLANRouter& operator=(VLANRouter&&) = default;

    /**
     * @brief Set the raw Ethernet backend used for all TX operations.
     *
     * The backend is stored as a non-owning pointer.  The caller must
     * ensure the backend outlives the router.
     *
     * @param backend Pointer to the raw NetworkInterface.
     */
    void setBackend(NetworkInterface* backend);

    /**
     * @brief Register an EtherCAT master with an optional RX VLAN range.
     *
     * @param master    Shared pointer to the master (must not be null).
     * @param rx_range  VLAN range this master expects on incoming frames.
     * @param tx_vlan   VLAN ID inserted into every frame this master sends.
     *                  std::nullopt means frames are forwarded unchanged.
     *
     * It is legal for multiple masters to have overlapping ranges.
     */
    void addMaster(std::shared_ptr<EtherCATMaster> master,
                   VLANRange rx_range,
                   std::optional<uint16_t> tx_vlan);

    /**
     * @brief Convenience overload: register a master with a single RX VLAN.
     */
    void addMaster(std::shared_ptr<EtherCATMaster> master,
                   std::optional<uint16_t> rx_vlan,
                   std::optional<uint16_t> tx_vlan);

    /**
     * @brief Register an EtherCAT master with no RX VLAN (untagged only).
     */
    void addMaster(std::shared_ptr<EtherCATMaster> master,
                   std::nullopt_t,
                   std::optional<uint16_t> tx_vlan);

    /**
     * @brief Unregister a master.
     *
     * Removes the first entry whose raw master pointer equals @p master.
     *
     * @param master Raw pointer to the master to remove.
     */
    void removeMaster(const EtherCATMaster* master);

    /**
     * @brief Clear all registered masters.
     */
    void clearMasters();

    /**
     * @brief Obtain the per-master NetworkInterface for TX.
     *
     * The returned interface's `send` lambda encapsulates frames with the
     * master's tx_vlan_id (if set) before forwarding them to the backend.
     * The `receive` lambda always returns false (RX is callback-driven).
     *
     * @param master Raw pointer to a previously added master.
     * @return Pointer to the master's NetworkInterface, or nullptr if
     *         the master is not registered.
     */
    NetworkInterface* networkInterfaceFor(const EtherCATMaster* master);

    /**
     * @brief Process a received Ethernet frame and route it to matching masters.
     *
     * This is the main RX entry point.  It must be called by the
     * application (e.g. from the HAL RX callback) for every incoming
     * frame.
     *
     * The function:
     * 1. Inspects the EtherType after the Ethernet header.
     * 2. If the EtherType is 0x8100 (802.1Q), extracts the 12-bit VID
     *    from the TCI, strips the 4-byte tag, and routes the
     *    decapsulated frame to every master whose rx_vlan_range
     *    contains the extracted VID.
     * 3. If no range matches and a dedicated undefined target has been
     *    set, the frame is delivered to that target.
     * 4. If the EtherType is not 0x8100, routes the raw frame to every
     *    master whose rx_vlan_range is std::nullopt.
     * 5. If no master matches and no undefined target exists, the frame
     *    is dropped after logging a warning with the VID and inner
     *    EtherType.
     *
     * @param data Pointer to the raw Ethernet frame.
     * @param len  Length of the frame in bytes.
     */
    void processRxFrame(const uint8_t* data, size_t len);

    /**
     * @brief Return all masters whose rx_vlan_range contains @p vlan_id.
     *
     * A snapshot is returned under the internal mutex.
     *
     * @param vlan_id The VLAN ID to look up.
     * @return Vector of shared_ptr to matching masters (may be empty).
     */
    std::vector<std::shared_ptr<EtherCATMaster>> mastersForVlanId(uint16_t vlan_id) const;

    /**
     * @brief Return a snapshot of all registered entries.
     *
     * @return Vector of Entry snapshots.
     */
    std::vector<Entry> entries() const;

    /**
     * @brief Return the number of registered masters.
     */
    size_t masterCount() const;

    /**
     * @brief Return the name of a well-known EtherType, or nullptr.
     *
     * This is a helper used internally for debug logging.  It covers
     * common industrial and networking EtherTypes.
     *
     * @param ether_type 16-bit EtherType in host byte order.
     * @return Human-readable name, or nullptr if unknown.
     */
    static const char* etherTypeName(uint16_t ether_type);

    /**
     * @brief Set the dedicated catch-all (undefined VLAN) target.
     *
     * The undefined target receives all tagged frames whose VID is not
     * matched by any other master's rx_vlan_range.
     *
     * @param master   Shared pointer to the master.
     * @param tx_vlan  Optional TX VLAN ID for this target.
     * @param replace  If true, overwrite an existing undefined target.
     *                 If false, return false when one already exists.
     * @return true on success, false if a target already exists and
     *         replace is false.
     */
    bool setUndefinedTarget(std::shared_ptr<EtherCATMaster> master,
                            std::optional<uint16_t> tx_vlan = std::nullopt,
                            bool replace = false);

    /**
     * @brief Remove the dedicated undefined target.
     */
    void clearUndefinedTarget();

    /**
     * @brief Return the current undefined target master, or nullptr.
     */
    std::shared_ptr<EtherCATMaster> undefinedTarget() const;

    /**
     * @brief Return the NetworkInterface for the undefined target.
     *
     * @return Pointer to the interface, or nullptr if no undefined
     *         target is registered.
     */
    NetworkInterface* undefinedNetworkInterface() const;

    /**
     * @brief Override the frame delivery callback (test hook).
     *
     * By default the router calls master->handleRxFrame().  Unit tests
     * can substitute a capture lambda to record or inspect delivered
     * frames without requiring full master lifecycle.
     *
     * @param fn Callback with signature (master, data, len).
     */
    void setDeliverFunction(std::function<void(EtherCATMaster*, const uint8_t*, size_t)> fn);

private:
    /**
     * @brief Internal entry stored in the vector for memory locality.
     */
    struct InternalEntry {
        std::shared_ptr<EtherCATMaster> master;
        std::optional<VLANRange> rx_vlan_range;
        std::optional<uint16_t> tx_vlan_id;
        NetworkInterface iface;  ///< Per-master view (send encapsulates)
    };

    /**
     * @brief Dedicated catch-all target (separate from the entries vector).
     */
    struct UndefinedTarget {
        std::shared_ptr<EtherCATMaster> master;
        std::optional<uint16_t> tx_vlan_id;
        NetworkInterface iface;
    };

    mutable std::mutex mutex_;
    std::vector<InternalEntry> entries_;
    std::optional<UndefinedTarget> undefined_target_;
    NetworkInterface* backend_ = nullptr;

    /**
     * @brief Rebuild all NetworkInterface send lambdas.
     *
     * Must be called with mutex_ held.
     */
    void rebuildInterfacesLocked();

    /**
     * @brief Configure a NetworkInterface's send/receive lambdas.
     *
     * Must be called with mutex_ held.
     */
    void rebuildInterfaceSend(NetworkInterface* iface, bool has_tx_vlan, uint16_t tx_vid);

    /**
     * @brief Deliver a frame to a specific master.
     *
     * Default behaviour invokes master->handleRxFrame().  Overridable
     * for unit testing.
     */
    std::function<void(EtherCATMaster*, const uint8_t*, size_t)> deliver_;
};

} // namespace EtherCAT

/**
 * @file ProxyForwardConfig.hpp
 * @brief Runtime configuration + counters for the optional external-source
 *        PDO forwarding path ("EtherCAT proxy").
 *
 * When enabled, Tether reserves a contiguous band of EtherCAT datagram
 * indexes for a foreign EtherCAT master (e.g. a SOEM instance) reachable
 * over a dedicated proxy network interface:
 *
 *   - packet sockets bound to the proxy interface carry the external
 *     source's frames — a kernel BPF accepts only frames whose FIRST
 *     datagram index lies inside the band;
 *   - validated frames (every datagram = logical command, every index
 *     inside the band) are injected verbatim onto the EtherCAT bus
 *     interface; slave responses are forwarded back the same way;
 *   - a second BPF on the proxy interface monitors EtherCAT frames
 *     outside the band — they are ignored and reported as errors;
 *   - the master's async index allocator and wire-socket BPF stay
 *     disjoint from the band, so Tether's own traffic can never collide
 *     with proxy traffic.
 *
 * Disabled (the default), nothing changes: no sockets, no thread, no
 * filter changes, no allocator difference.
 */

#pragma once

#include <cstdint>
#include <optional>
#include <string>
#include <vector>

namespace EtherCAT {

/**
 * @brief One capability window over the FMMU logical address space.
 *
 * Every logical (PDO) datagram carries a 32-bit logical address — the
 * external source may only touch addresses covered by rules granting
 * the matching capability:  LRD needs `read`, LWR needs `write`, LRW
 * needs a window whose OR'd flags cover both.  Windows may overlap;
 * capabilities of all covering rules OR together.
 */
struct ProxyPdoAccessRule {
    uint32_t addr  = 0;   /**< First logical address of the window. */
    uint32_t len   = 0;   /**< Window length in bytes (>= 1). */
    bool     read  = false;
    bool     write = false;
};

/**
 * @brief Value constraint on a write — applies to the `width`-byte cell
 *        at `addr` whenever a write datagram fully covers it.
 *
 * The little-endian value extracted from the write payload must satisfy
 * `(value & mask) == expected`.  Several rules on the SAME (addr, width)
 * cell are alternatives: the write is allowed when ANY of them matches
 * (enumerated-value whitelist).  Constraints on different cells are all
 * enforced (AND).  A write that covers only part of a constrained cell
 * is rejected — the value cannot be evaluated from a fragment.
 *
 * mask = all-ones pins exact values; sparser masks leave bit-fields
 * free (e.g. mask=0xFF00, expected=0x0600 accepts every low byte).
 */
struct ProxyPdoValueRule {
    uint32_t addr     = 0;  /**< First byte of the constrained cell. */
    uint8_t  width    = 0;  /**< Cell width in bytes: 1..8. */
    uint64_t mask     = 0;  /**< Bits under comparison. */
    uint64_t expected = 0;  /**< Required value of the masked bits. */
};

/**
 * @brief Address/value whitelist applied to injected datagrams.
 *
 * Evaluated per datagram AFTER the structural PDO/band check:
 *   - every byte of the datagram's logical span must be covered by an
 *     access rule granting the required capability (read / write /
 *     both for LRW), and
 *   - for write datagrams, every constrained value cell overlapping
 *     the span must be fully covered and satisfied.
 *
 * An empty filter object (present but no rules) denies everything —
 * absence of the filter (std::nullopt on the config) keeps the
 * PDO-only validation.
 */
struct ProxyPdoFilter {
    std::vector<ProxyPdoAccessRule> access;
    std::vector<ProxyPdoValueRule>  values;

    /// Structural sanity — called by Master::setProxyForwarding().
    bool validate(std::string& err) const;
    /**
     * Per-datagram verdict.  `data`/`dlen` are the payload bytes of a
     * logical datagram at logical address `adr`.
     */
    bool allows(bool need_read, bool need_write, uint32_t adr,
                const uint8_t* data, uint16_t dlen) const;
};

/**
 * @brief Which mapped PDOs the auto-generated access whitelist covers —
 *        see Master::buildProxyPdoFilter().
 *
 * Used only when ProxyForwardConfig::filter is unset.  An all-default
 * selection covers every mapped PDO region of every slave (all LAMs,
 * including PDO groups).
 */
struct ProxyPdoSelection {
    /**Restrict to these slave indexes (empty = all slaves). */
    std::vector<uint16_t> slaves;
    /**
     * Restrict to these PDO mapping-object indexes (e.g. 0x1600, 0x1A01 —
     * empty = all PDOs).  Only meaningful for slaves mapped via the
     * multi-PDO path (per-PDO logical addresses); slaves on the legacy
     * single-PDO path carry pdo_index 0 and never match a non-empty
     * index selection.
     */
    std::vector<uint16_t> pdo_indexes;
    /**Include RxPDO regions (master→slave outputs — the write side). */
    bool include_outputs = true;
    /**Include TxPDO regions (slave→master inputs — the read side). */
    bool include_inputs  = true;
    /**
     * Capabilities of the generated windows:
     *   true  → every window grants read AND write — required when the
     *           external source exchanges via LRW datagrams spanning
     *           output and input regions (the common cyclic pattern),
     *           or writes outputs with read-back in one datagram.
     *   false → output windows grant write, input windows grant read —
     *           strict direction split for sources using separate
     *           LRD/LWR datagrams.
     */
    bool bidirectional = true;
};

/**
 * @brief Runtime switch + band description for the proxy forwarding path.
 *
 * Pass to Master::setProxyForwarding().  `idx_base`/`idx_count` select the
 * reserved band [idx_base, idx_base+idx_count-1], which must lie entirely
 * inside the async range (below the cyclic fastpath band at 0x9C) and
 * leave at least one async index for the master's own traffic.
 */
struct ProxyForwardConfig {
    bool enabled = false; /**< Master switch — everything off when false. */

    /**
     * Name of the proxy network interface the external source is
     * reachable on (e.g. "eth1").  Required when enabled; must differ
     * from the EtherCAT bus interface.
     */
    std::string ifname;

    uint8_t idx_base  = 0; /**< First reserved datagram index. */
    uint8_t idx_count = 0; /**< Band size; >= 1 when enabled. */

    /**
     * Address/value whitelist for injected datagrams.
     *
     *   - Set   → used verbatim (manual whitelist — build or inspect the
     *             auto-generated one via Master::buildProxyPdoFilter()).
     *   - Unset → AUTO-GENERATED from the master's PDO map at forwarder
     *             start: one access window per mapped PDO region,
     *             restricted by `pdo_selection`.  A not-yet-configured
     *             map yields zero windows → everything is denied.
     */
    std::optional<ProxyPdoFilter> filter;

    /**
     * Selection for the auto-generated whitelist — used only when
     * `filter` is unset.  Default = every mapped PDO on every slave,
     * bidirectional caps.
     */
    ProxyPdoSelection pdo_selection;

    /**
     * Optional UDP input listener on the proxy interface
     * (EtherCAT-over-UDP, ETG.1000.3).  0 disables.
     *
     * When non-zero the forwarder also accepts UDP datagrams on this
     * port whose payload is a bare EtherCAT frame ([EtherCAT frame
     * header][datagrams], no Ethernet header).  The payload is
     * validated like any in-band frame and injected onto the bus
     * inside a synthesised Ethernet header (broadcast dst, bus-iface
     * src MAC, VLAN tag when the wire runs tagged).  Slave responses
     * are re-encapsulated and returned to the last-seen UDP peer —
     * one UDP source at a time is supported.
     */
    uint16_t udp_input_port = 0;

    /**
     * Inclusive band end.  Only meaningful when idx_count >= 1 — config
     * validation enforces that before the band is ever used.
     */
    uint8_t end() const {
        return static_cast<uint8_t>(
            (static_cast<unsigned>(idx_base) + idx_count - 1u) & 0xFFu);
    }
};

/**
 * @brief Forwarder counters — a point-in-time snapshot, all counters
 *        monotonically increasing while the forwarder runs.
 */
struct ProxyForwardStats {
    uint64_t rx_forwarded{0}; /**< in-band bus→proxy frames forwarded */
    uint64_t tx_forwarded{0}; /**< valid proxy→bus frames injected */
    /**
     * Proxy-interface frames that passed the in-band BPF but failed
     * userspace validation (non-logical datagram command, out-of-band
     * secondary index, malformed chain) — dropped, never forwarded.
     */
    uint64_t tx_rejected{0};
    /**
     * Structurally valid PDO frames rejected by the configured
     * ProxyPdoFilter (address window or value constraint) — dropped,
     * never forwarded.
     */
    uint64_t filtered{0};
    /**
     * EtherCAT frames seen on the proxy interface whose first index lies
     * OUTSIDE the configured band — ignored + rate-limited error log.
     */
    uint64_t out_of_band{0};
    uint64_t send_errors{0};  /**< sendto() failures in either direction */
};

} // namespace EtherCAT

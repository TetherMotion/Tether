/**
 * @file ProxyPdoFilter.cpp
 * @brief Address/value whitelist for the external-source PDO proxy.
 *
 * Semantics (see ProxyForwardConfig.hpp for the type docs):
 *
 *   Access — a datagram's logical span [adr, adr+dlen) must be fully
 *   covered by rule windows granting the needed capability.  LRD needs
 *   `read`, LWR needs `write`, LRW needs both (the OR of flags across
 *   covering rules).  Empty rule set → deny everything.
 *
 *   Values — enforced on writes only.  Each constrained cell that
 *   overlaps the span must be fully covered (partial → reject) and the
 *   extracted little-endian value must match ≥1 of the rules defined
 *   for that (addr, width) cell:  (value & mask) == expected.
 */

#include "tether/ethercat/ProxyForwardConfig.hpp"

#include <algorithm>
#include <format>

namespace EtherCAT {

namespace {

constexpr unsigned kCapRead  = 1;
constexpr unsigned kCapWrite = 2;

constexpr unsigned ruleCaps(const ProxyPdoAccessRule& r) {
    return (r.read ? kCapRead : 0u) | (r.write ? kCapWrite : 0u);
}

/// Little-endian integer over `width` bytes (width ∈ [1,8]).
uint64_t readLe(const uint8_t* p, unsigned width) {
    uint64_t v = 0;
    for (unsigned i = 0; i < width; ++i)
        v |= static_cast<uint64_t>(p[i]) << (8 * i);
    return v;
}

} // namespace

bool ProxyPdoFilter::validate(std::string& err) const
{
    for (const auto& r : access) {
        if (r.len == 0) {
            err = "access window has zero length";
            return false;
        }
        if (!r.read && !r.write) {
            err = std::format("access window [0x{:X},+{}) grants "
                              "neither read nor write", r.addr, r.len);
            return false;
        }
        if (static_cast<uint64_t>(r.addr) + r.len > (1ull << 32)) {
            err = std::format("access window [0x{:X},+{}) overflows the "
                              "32-bit logical address space",
                              r.addr, r.len);
            return false;
        }
    }
    for (const auto& v : values) {
        if (v.width < 1 || v.width > 8) {
            err = std::format("value rule at 0x{:X}: width {} not in 1..8",
                              v.addr, v.width);
            return false;
        }
        if (static_cast<uint64_t>(v.addr) + v.width > (1ull << 32)) {
            err = std::format("value rule cell [0x{:X},+{}) overflows",
                              v.addr, v.width);
            return false;
        }
        const uint64_t wmask = v.width >= 8
            ? ~0ull : ((1ull << (v.width * 8)) - 1);
        if (v.mask & ~wmask) {
            err = std::format("value rule at 0x{:X}: mask 0x{:X} wider "
                              "than {} bytes", v.addr, v.mask, v.width);
            return false;
        }
        if (v.expected & ~v.mask) {
            err = std::format("value rule at 0x{:X}: expected 0x{:X} has "
                              "bits outside mask 0x{:X}",
                              v.addr, v.expected, v.mask);
            return false;
        }
    }
    return true;
}

bool ProxyPdoFilter::allows(bool need_read, bool need_write, uint32_t adr,
                            const uint8_t* data, uint16_t dlen) const
{
    if (dlen == 0 || !data) return false;
    const uint64_t end = static_cast<uint64_t>(adr) + dlen;
    if (end > (1ull << 32)) return false;

    // Coverage per capability: every byte of the span must lie inside
    // ≥1 rule granting that capability — read and write grants are
    // independent (a read rule + a write rule at the same address
    // together cover an LRW).
    auto covers = [&](unsigned cap) {
        uint64_t p = adr;
        while (p < end) {
            uint64_t next = 0;
            for (const auto& r : access) {
                if (!(ruleCaps(r) & cap)) continue;
                const uint64_t lo = r.addr, hi = lo + r.len;
                if (p >= lo && p < hi) next = std::max(next, hi);
            }
            if (next <= p) return false;
            p = next;
        }
        return true;
    };
    if (need_read  && !covers(kCapRead))  return false;
    if (need_write && !covers(kCapWrite)) return false;
    if (!need_write) return true;

    // Value constraints — each distinct constrained cell overlapped by
    // the span must be fully covered and satisfied by ≥1 same-cell rule.
    for (size_t i = 0; i < values.size(); ++i) {
        const auto& v = values[i];
        const uint64_t clo = v.addr, chi = clo + v.width;
        if (chi <= adr || clo >= end) continue;    // no overlap
        if (clo < adr || chi > end) return false;  // partial → reject

        // Skip cells already evaluated by an earlier same-cell rule.
        bool dup = false;
        for (size_t j = 0; j < i; ++j) {
            if (values[j].addr == v.addr && values[j].width == v.width) {
                dup = true;
                break;
            }
        }
        if (dup) continue;

        const uint64_t got =
            readLe(data + (clo - adr), v.width);
        bool pass = false;
        for (const auto& alt : values) {
            if (alt.addr == v.addr && alt.width == v.width &&
                (got & alt.mask) == alt.expected) {
                pass = true;
                break;
            }
        }
        if (!pass) return false;
    }
    return true;
}

} // namespace EtherCAT

/**
 * @file SlaveRegistry.hpp
 * @brief The master's slave table: Slave objects, display names, the
 *        invalid-index sentinel, and the discovered-slave count.
 *
 * @internal Internal header — not installed, not part of the public API.
 *
 * Master owns a `std::unique_ptr<SlaveRegistry>`; the public accessors
 * (slave(), slaveName(), sii(), getDiscoveredSlaveCount()) forward here.
 */

#pragma once

#include "tether/TetherConfig.hpp"

#include <atomic>
#include <cstdint>
#include <memory>
#include <string>
#include <string_view>
#include <vector>

namespace EtherCAT {

class Master;
class Slave;
class NonExistingSlave;
namespace SII { class SIIManager; }

class SlaveRegistry {
public:
    explicit SlaveRegistry(Master& master) : master_(master) {}

    /// Clear and resize for @p count slaves (names reset to empty).
    void reset(uint16_t count);
    /// Install a constructed Slave at position @p i.
    void set(uint16_t i, std::unique_ptr<Slave> slave);

    /// Element access — nullptr entries possible during partial teardown.
    std::unique_ptr<Slave>&       operator[](size_t i)       { return entries_[i]; }
    const std::unique_ptr<Slave>& operator[](size_t i) const { return entries_[i]; }
    size_t size()  const { return entries_.size(); }
    bool   empty() const { return entries_.empty(); }

    /// Slave at @p i — falls back to the NonExistingSlave sentinel for an
    /// out-of-range index or null entry (never returns a dangling ref).
    Slave& get(uint16_t i);

#if TETHER_ENABLE_SII
    /// Per-slave SII manager — sentinel-backed fallback out of range.
    SII::SIIManager& sii(uint16_t i);
#endif

    void setName(uint16_t i, std::string name);
    std::string_view name(uint16_t i) const;
    /// "Slave <name> (#<idx>)" — falls back to "Slave <idx>".
    std::string logPrefix(uint16_t i) const;

    /// Discovered-slave count published by the discovery scan.
    std::atomic<uint16_t> discovered_count{0};

private:
    NonExistingSlave& sentinel(uint16_t i);

    Master& master_;
    std::vector<std::unique_ptr<Slave>> entries_;
    std::vector<std::string>            names_;
    std::unique_ptr<NonExistingSlave>   sentinel_;
};

} // namespace EtherCAT

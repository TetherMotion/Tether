/**
 * @file OidAllocator.hpp
 * @brief Object ID (OID) allocator for Klipper peripheral objects.
 *
 * @details
 * Each peripheral instance on the device (a GPIO, PWM, stepper, etc.) is
 * identified by an 8-bit OID assigned during the config phase. The host
 * allocates OIDs by sending `allocate_oids` commands; the device assigns
 * contiguous OIDs starting from 0. This allocator tracks the next free OID
 * and the count per object type.
 */

#pragma once

#include <cstdint>
#include <optional>
#include <string>
#include <unordered_map>

namespace tether::klipper::objects {

/**
 * @brief Allocates 8-bit OIDs for peripheral objects.
 */
class OidAllocator {
public:
    /// @brief Allocate the next OID (0..254).
    /// @return The allocated OID, or std::nullopt if exhausted.
    std::optional<uint8_t> allocate() {
        if (nextOid_ > 254) { full_ = true; return std::nullopt; }
        return nextOid_++;
    }

    /// @brief Allocate a contiguous block of @p count OIDs; returns the first.
    /// @return The first OID in the block, or std::nullopt if not enough remain.
    std::optional<uint8_t> allocateBlock(uint8_t count) {
        if (nextOid_ + count > 255) { full_ = true; return std::nullopt; }
        uint8_t first = nextOid_;
        nextOid_ += count;
        return first;
    }

    /// @brief Record an OID -> type mapping (for bookkeeping).
    void assign(uint8_t oid, std::string type) {
        types_[oid] = std::move(type);
    }

    /// @return The type string for an OID, or empty if unknown.
    std::string typeOf(uint8_t oid) const {
        auto it = types_.find(oid);
        return it == types_.end() ? std::string{} : it->second;
    }

    /// @return The next OID that will be allocated.
    uint8_t nextOid() const { return nextOid_; }

    /// @return True if all 255 OIDs are exhausted.
    bool isFull() const { return full_; }

    /// @brief Reset the allocator.
    void reset() { nextOid_ = 0; full_ = false; types_.clear(); }

private:
    uint8_t nextOid_ = 0;
    bool full_ = false;
    std::unordered_map<uint8_t, std::string> types_;
};

} // namespace tether::klipper::objects

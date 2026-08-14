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
#include <string>
#include <unordered_map>

namespace tether::klipper::objects {

/**
 * @brief Allocates 8-bit OIDs for peripheral objects.
 */
class OidAllocator {
public:
    /// @brief Allocate the next OID (0..254). Returns 255 and sets full_ if exhausted.
    /// Callers should check isFull() after allocation to detect exhaustion.
    uint8_t allocate() {
        if (nextOid_ > 254) { full_ = true; return 255; }
        return nextOid_++;
    }

    /// @brief Allocate a contiguous block of @p count OIDs; returns the first.
    ///        Returns 255 if not enough OIDs remain. Callers should check isFull().
    uint8_t allocateBlock(uint8_t count) {
        if (nextOid_ + count > 255) { full_ = true; return 255; }
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

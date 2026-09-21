/**
 * @file SlaveRegistry.cpp
 * @brief SlaveRegistry — slave table, names, sentinel, discovered count.
 */

#include "raw/SlaveRegistry.hpp"

#include "tether/ethercat/Master.hpp"
#include "tether/ethercat/Slave.hpp"
#include "tether/platform/Platform.hpp"

#include <format>
#include <utility>

namespace EtherCAT {

void SlaveRegistry::reset(uint16_t count)
{
    entries_.clear();
    entries_.reserve(count);
    names_.clear();
    names_.resize(count);
    sentinel_.reset();
}

void SlaveRegistry::set(uint16_t i, std::unique_ptr<Slave> slave)
{
    if (i < entries_.size()) {
        entries_[i] = std::move(slave);
    } else {
        entries_.resize(i + 1);
        entries_[i] = std::move(slave);
    }
}

NonExistingSlave& SlaveRegistry::sentinel(uint16_t i)
{
    // Recreate so the embedded index (and its error messages) match.
    sentinel_ = std::make_unique<NonExistingSlave>(master_, i);
    return *sentinel_;
}

Slave& SlaveRegistry::get(uint16_t i)
{
    if (i < entries_.size() && entries_[i]) {
        return *entries_[i];
    }
    return sentinel(i);
}

#if TETHER_ENABLE_SII

SII::SIIManager& SlaveRegistry::sii(uint16_t i)
{
    if (i < entries_.size() && entries_[i]) {
        return entries_[i]->sii();
    }
    return sentinel(i).sii();
}

#endif // TETHER_ENABLE_SII

void SlaveRegistry::setName(uint16_t i, std::string name)
{
    if (i >= names_.size()) {
        TETHER_LOGW("master", "setSlaveName: index {} out of range (slaves={})",
                    i, names_.size());
        return;
    }
    names_[i] = std::move(name);
}

std::string_view SlaveRegistry::name(uint16_t i) const
{
    if (i >= names_.size()) return {};
    return names_[i];
}

std::string SlaveRegistry::logPrefix(uint16_t i) const
{
    if (i >= names_.size()) return std::format("Slave {}", i);
    const auto& n = names_[i];
    if (n.empty()) return std::format("Slave {}", i);
    return std::format("Slave {} (#{})", n, i);
}

} // namespace EtherCAT

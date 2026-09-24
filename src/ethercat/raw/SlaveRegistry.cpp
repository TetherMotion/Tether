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

SlaveRegistry::SlaveRegistry(Master& master) : master_(master) {}
SlaveRegistry::~SlaveRegistry() = default;

void SlaveRegistry::reset(uint16_t count)
{
    std::lock_guard<std::mutex> lock(mutex_);
    entries_.clear();
    entries_.reserve(count);
    names_.clear();
    names_.resize(count);
    sentinel_.reset();
}

void SlaveRegistry::set(uint16_t i, std::unique_ptr<Slave> slave)
{
    std::lock_guard<std::mutex> lock(mutex_);
    if (i < entries_.size()) {
        entries_[i] = std::move(slave);
    } else {
        entries_.resize(i + 1);
        entries_[i] = std::move(slave);
    }
}

Slave* SlaveRegistry::slaveAt(size_t i)
{
    std::lock_guard<std::mutex> lock(mutex_);
    return i < entries_.size() ? entries_[i].get() : nullptr;
}

size_t SlaveRegistry::size() const
{
    std::lock_guard<std::mutex> lock(mutex_);
    return entries_.size();
}

NonExistingSlave& SlaveRegistry::sentinel(uint16_t i)
{
    // mutex_ is held by the caller.  Recreate so the embedded index (and
    // its error messages) match.
    sentinel_ = std::make_unique<NonExistingSlave>(master_, i);
    return *sentinel_;
}

Slave& SlaveRegistry::get(uint16_t i)
{
    std::lock_guard<std::mutex> lock(mutex_);
    if (i < entries_.size() && entries_[i]) {
        return *entries_[i];
    }
    return sentinel(i);
}

#if TETHER_ENABLE_SII

SII::SIIManager& SlaveRegistry::sii(uint16_t i)
{
    std::lock_guard<std::mutex> lock(mutex_);
    if (i < entries_.size() && entries_[i]) {
        return entries_[i]->sii();
    }
    return sentinel(i).sii();
}

#endif // TETHER_ENABLE_SII

void SlaveRegistry::setName(uint16_t i, std::string name)
{
    std::lock_guard<std::mutex> lock(mutex_);
    if (i >= names_.size()) {
        TETHER_LOGW("master", "setSlaveName: index {} out of range (slaves={})",
                    i, names_.size());
        return;
    }
    names_[i] = std::move(name);
}

std::string_view SlaveRegistry::name(uint16_t i) const
{
    std::lock_guard<std::mutex> lock(mutex_);
    if (i >= names_.size()) return {};
    return names_[i];
}

std::string SlaveRegistry::logPrefix(uint16_t i) const
{
    std::lock_guard<std::mutex> lock(mutex_);
    if (i >= names_.size()) return std::format("Slave {}", i);
    const auto& n = names_[i];
    if (n.empty()) return std::format("Slave {}", i);
    return std::format("Slave {} (#{})", n, i);
}

} // namespace EtherCAT

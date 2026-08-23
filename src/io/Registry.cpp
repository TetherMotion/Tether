/**
 * @file Registry.cpp
 * @brief Implementation of the parameter/signal registry.
 * @copyright Copyright (C) 2025-2026 Tether Authors
 */
#include "tether/io/Registry.hpp"
#include <algorithm>

namespace tether { namespace io {

bool Registry::addParam(ParamEntry entry) {
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (idMap_.count(entry.id)) return false;
        idMap_[entry.id] = EntryKind::Parameter;
        params_.push_back(std::move(entry));
        ++revision_;
    }
    notifyChange();
    return true;
}

bool Registry::addSignal(SignalEntry entry) {
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (idMap_.count(entry.id)) return false;
        idMap_[entry.id] = EntryKind::Signal;
        signals_.push_back(std::move(entry));
        ++revision_;
    }
    notifyChange();
    return true;
}

bool Registry::addFunction(FunctionEntry entry) {
    if (!entry.validSignature()) return false;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (idMap_.count(entry.id)) return false;
        idMap_[entry.id] = EntryKind::Function;
        functions_.push_back(std::move(entry));
        ++revision_;
    }
    notifyChange();
    return true;
}

uint32_t Registry::paramCount() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return static_cast<uint32_t>(params_.size());
}

uint32_t Registry::signalCount() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return static_cast<uint32_t>(signals_.size());
}

uint32_t Registry::functionCount() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return static_cast<uint32_t>(functions_.size());
}

std::vector<FunctionView> Registry::functionPage(uint32_t offset, uint32_t maxCount) const {
    std::lock_guard<std::mutex> lock(mutex_);
    std::vector<FunctionView> result;
    if (offset >= functions_.size()) return result;
    const uint32_t available = static_cast<uint32_t>(functions_.size()) - offset;
    const uint32_t end = offset + std::min(maxCount, available);
    result.reserve(end - offset);
    for (uint32_t i = offset; i < end; ++i) result.emplace_back(&functions_[i]);
    return result;
}

FunctionView Registry::findFunction(uint64_t id) const {
    std::lock_guard<std::mutex> lock(mutex_);
    auto it = idMap_.find(id);
    if (it == idMap_.end() || it->second != EntryKind::Function) return FunctionView();
    for (const auto& function : functions_) {
        if (function.id == id) return FunctionView(&function);
    }
    return FunctionView();
}

uint32_t Registry::totalCount() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return static_cast<uint32_t>(params_.size() + signals_.size());
}

void Registry::defineStreamFilterProperty(FilterPropertyDef definition) {
    std::lock_guard<std::mutex> lock(mutex_);
    filterSchema_.defineProperty(std::move(definition));
}

StreamFilterSchema::Result Registry::validateStreamFilter(const FilterProperty& property) const {
    std::lock_guard<std::mutex> lock(mutex_);
    return filterSchema_.validate(property);
}

bool Registry::supportsStreamFilters() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return !filterSchema_.empty();
}

std::vector<EntryView> Registry::paramPage(uint32_t offset, uint32_t maxCount) const {
    std::lock_guard<std::mutex> lock(mutex_);
    std::vector<EntryView> result;
    if (offset >= params_.size()) return result;
    const uint32_t available = static_cast<uint32_t>(params_.size()) - offset;
    const uint32_t end = offset + std::min(maxCount, available);
    result.reserve(end - offset);
    for (uint32_t i = offset; i < end; ++i) {
        result.emplace_back(&params_[i]);
    }
    return result;
}

std::vector<EntryView> Registry::signalPage(uint32_t offset, uint32_t maxCount) const {
    std::lock_guard<std::mutex> lock(mutex_);
    std::vector<EntryView> result;
    if (offset >= signals_.size()) return result;
    const uint32_t available = static_cast<uint32_t>(signals_.size()) - offset;
    const uint32_t end = offset + std::min(maxCount, available);
    result.reserve(end - offset);
    for (uint32_t i = offset; i < end; ++i) {
        result.emplace_back(&signals_[i]);
    }
    return result;
}

EntryView Registry::find(uint64_t id) const {
    std::lock_guard<std::mutex> lock(mutex_);
    auto it = idMap_.find(id);
    if (it == idMap_.end()) return EntryView();
    if (it->second == EntryKind::Parameter) {
        for (const auto& p : params_) {
            if (p.id == id) return EntryView(&p);
        }
    } else if (it->second == EntryKind::Signal) {
        for (const auto& s : signals_) {
            if (s.id == id) return EntryView(&s);
        }
    }
    return EntryView();
}

EntryView Registry::findParam(uint64_t id) const {
    std::lock_guard<std::mutex> lock(mutex_);
    auto it = idMap_.find(id);
    if (it == idMap_.end() || it->second != EntryKind::Parameter) return EntryView();
    for (const auto& p : params_) {
        if (p.id == id) return EntryView(&p);
    }
    return EntryView();
}

EntryView Registry::findSignal(uint64_t id) const {
    std::lock_guard<std::mutex> lock(mutex_);
    auto it = idMap_.find(id);
    if (it == idMap_.end() || it->second != EntryKind::Signal) return EntryView();
    for (const auto& s : signals_) {
        if (s.id == id) return EntryView(&s);
    }
    return EntryView();
}

size_t Registry::addChangeListener(CatalogChangeListener listener) {
    std::lock_guard<std::mutex> lock(mutex_);
    size_t handle = nextListenerHandle_++;
    listeners_[handle] = std::move(listener);
    return handle;
}

void Registry::removeChangeListener(size_t handle) {
    std::unique_lock<std::mutex> lock(mutex_);
    listeners_.erase(handle);
    // Wait for any in-flight notifyChange() calls to finish before returning.
    // This guarantees that by the time removeChangeListener() returns, no
    // callback that was copied out of `listeners_` is still running — which
    // is essential when the listener's owner (e.g. Session) is about to be
    // destroyed and the lambda captures `this`.
    notifyCv_.wait(lock, [this] { return notifyInFlight_ == 0; });
}

void Registry::notifyChange() {
    // Copy listeners under the lock, then invoke them outside the lock.
    // This prevents deadlock if a listener calls back into Registry (e.g.
    // addParam). The `notifyInFlight_` counter + condition variable ensures
    // removeChangeListener() can quiesce in-flight callbacks before the
    // caller proceeds to destroy the listener's owner.
    std::vector<CatalogChangeListener> listeners_copy;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        for (const auto& [handle, listener] : listeners_) {
            listeners_copy.push_back(listener);
        }
        ++notifyInFlight_;
    }
    for (const auto& listener : listeners_copy) {
        listener();
    }
    {
        std::lock_guard<std::mutex> lock(mutex_);
        --notifyInFlight_;
    }
    notifyCv_.notify_all();
}

}} // namespace tether::io

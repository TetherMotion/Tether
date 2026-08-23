/**
 * @file Registry.hpp
 * @brief Thread-safe registry of parameters and signals for the IO protocol.
 *
 * The Registry holds the catalog of all exposed parameters and signals.
 * Entries are registered during initialization, then the registry is used
 * read-only by sessions.  The registry supports both heap-allocated entries
 * (ParamEntry / SignalEntry) and static (ROM-friendly) entries.
 *
 * ## Parameters vs Signals
 *
 * - **Parameter**: A named value that can be read and (optionally) written.
 *   Examples: PID gains, motion limits, configuration flags.
 * - **Signal**: A named value that can only be read (observed).
 *   Examples: actual position, measured force, error integrator state.
 *
 * Both share the same numeric ID space (uint64_t).  The registry enforces
 * uniqueness across both parameters and signals.
 *
 * @copyright Copyright (C) 2025-2026 Tether Authors
 */
#pragma once

#include "tether/io/Protocol.hpp"
#include "tether/io/BinaryStruct.hpp"
#include "tether/io/Function.hpp"
#include <cstdint>
#include <cstddef>
#include <string>
#include <string_view>
#include <vector>
#include <deque>
#include <map>
#include <functional>
#include <mutex>
#include <condition_variable>

namespace tether { namespace io {

// ---------------------------------------------------------------------------
// Callback types
// ---------------------------------------------------------------------------

/// Reads a fixed-size value into dest (exactly valueTypeSize bytes).
using ReadFn  = std::function<void(void* dest)>;
/// Writes a fixed-size value from src (exactly valueTypeSize bytes).
using WriteFn = std::function<void(const void* src)>;
/// Reads a variable-length value. Returns actual bytes written (≤ maxLen).
using VarReadFn  = std::function<size_t(void* dest, size_t maxLen)>;
/// Writes a variable-length value.
using VarWriteFn = std::function<void(const void* src, size_t len)>;

// ---------------------------------------------------------------------------
// Parameter entry
// ---------------------------------------------------------------------------

/// A single parameter definition.
struct ParamEntry {
    uint64_t    id;
    std::string name;
    std::string description;
    std::string group;              ///< Logical group (e.g. "pid.axis0", "ethercat.slave1")
    ValueType   valueType;
    ReadFn      readFn;
    WriteFn     writeFn;            ///< nullptr = read-only parameter
    VarReadFn   varReadFn;          ///< For String/Binary
    VarWriteFn  varWriteFn;         ///< For String/Binary
    uint16_t    maxValueSize = 0;   ///< 0 for fixed-size; max bytes for variable-length
    std::map<std::string, std::string> metadata;
    const StructDescriptor* structDesc = nullptr;  ///< Optional struct layout

    uint8_t valueSize() const { return valueTypeSize(valueType); }
    bool isVariableLength() const { return tether::io::isVariableLength(valueType) || valueType == ValueType::Struct; }
    bool writable() const { return static_cast<bool>(writeFn) || static_cast<bool>(varWriteFn); }

    uint8_t flags() const {
        uint8_t f = EntryFlags::Readable;
        if (writable()) f |= EntryFlags::Writable;
        if (isVariableLength()) f |= EntryFlags::VariableLen;
        if (structDesc) f |= EntryFlags::HasStruct;
        return f;
    }
};

// ---------------------------------------------------------------------------
// Signal entry
// ---------------------------------------------------------------------------

/// A single signal definition (read-only).
struct SignalEntry {
    uint64_t    id;
    std::string name;
    std::string description;
    std::string group;
    ValueType   valueType;
    ReadFn      readFn;
    VarReadFn   varReadFn;
    uint16_t    maxValueSize = 0;
    std::map<std::string, std::string> metadata;
    const StructDescriptor* structDesc = nullptr;

    uint8_t valueSize() const { return valueTypeSize(valueType); }
    bool isVariableLength() const { return tether::io::isVariableLength(valueType) || valueType == ValueType::Struct; }

    uint8_t flags() const {
        uint8_t f = EntryFlags::Readable;
        if (isVariableLength()) f |= EntryFlags::VariableLen;
        if (structDesc) f |= EntryFlags::HasStruct;
        return f;
    }
};

// ---------------------------------------------------------------------------
// Unified entry view (can reference either a ParamEntry or SignalEntry)
// ---------------------------------------------------------------------------

class EntryView {
public:
    EntryView() = default;
    explicit EntryView(const ParamEntry* p) : param_(p) {}
    explicit EntryView(const SignalEntry* s) : signal_(s) {}

    explicit operator bool() const { return param_ || signal_; }

    EntryKind kind() const { return param_ ? EntryKind::Parameter : EntryKind::Signal; }
    uint64_t id() const { return param_ ? param_->id : signal_->id; }
    ValueType valueType() const { return param_ ? param_->valueType : signal_->valueType; }
    uint8_t valueSize() const { return param_ ? param_->valueSize() : signal_->valueSize(); }
    bool writable() const { return param_ ? param_->writable() : false; }
    uint8_t flags() const { return param_ ? param_->flags() : signal_->flags(); }
    bool isVariableLength() const { return param_ ? param_->isVariableLength() : signal_->isVariableLength(); }
    uint16_t maxValueSize() const { return param_ ? param_->maxValueSize : signal_->maxValueSize; }
    const StructDescriptor* structDesc() const { return param_ ? param_->structDesc : signal_->structDesc; }

    std::string_view name() const {
        return param_ ? std::string_view(param_->name) : std::string_view(signal_->name);
    }
    std::string_view description() const {
        return param_ ? std::string_view(param_->description) : std::string_view(signal_->description);
    }
    std::string_view group() const {
        return param_ ? std::string_view(param_->group) : std::string_view(signal_->group);
    }

    void read(void* dest) const {
        if (param_) param_->readFn(dest);
        else signal_->readFn(dest);
    }

    void write(const void* src) const {
        if (param_ && param_->writeFn) param_->writeFn(src);
    }

    size_t readVar(void* dest, size_t maxLen) const {
        if (param_ && param_->varReadFn) return param_->varReadFn(dest, maxLen);
        if (signal_ && signal_->varReadFn) return signal_->varReadFn(dest, maxLen);
        return 0;
    }

    void writeVar(const void* src, size_t len) const {
        if (param_ && param_->varWriteFn) param_->varWriteFn(src, len);
    }

    size_t metadataCount() const {
        return param_ ? param_->metadata.size() : signal_->metadata.size();
    }

    template<typename Fn>
    void forEachMetadata(Fn&& fn) const {
        const auto& md = param_ ? param_->metadata : signal_->metadata;
        for (const auto& [k, v] : md) {
            fn(std::string_view(k), std::string_view(v));
        }
    }

    const ParamEntry* asParam() const { return param_; }
    const SignalEntry* asSignal() const { return signal_; }

private:
    const ParamEntry*  param_  = nullptr;
    const SignalEntry* signal_ = nullptr;
};

// ---------------------------------------------------------------------------
// Catalog change listener
// ---------------------------------------------------------------------------

/// Callback invoked when the catalog changes (entries added/removed).
using CatalogChangeListener = std::function<void()>;

// ---------------------------------------------------------------------------
// Registry
// ---------------------------------------------------------------------------

/**
 * @class Registry
 * @brief Holds the catalog of parameters and signals.
 *
 * Thread safety:
 *  - addParam() / addSignal() are mutex-protected and can be called
 *    from any thread.  They also notify any registered catalog change listeners.
 *  - All read methods are safe for concurrent use after initialization.
 *  - If entries are added dynamically (rare), reads use a shared lock
 *    internally.
 */
class Registry {
public:
    /// Register a parameter. Returns true on success, false if ID is duplicate.
    bool addParam(ParamEntry entry);

    /// Register a signal. Returns true on success, false if ID is duplicate.
    bool addSignal(SignalEntry entry);

    /// Total parameter count.
    uint32_t paramCount() const;

    /// Total signal count.
    uint32_t signalCount() const;

    /// Total entry count (params + signals).
    uint32_t totalCount() const;

    /// Get a page of parameter views.
    std::vector<EntryView> paramPage(uint32_t offset, uint32_t maxCount) const;

    /// Get a page of signal views.
    std::vector<EntryView> signalPage(uint32_t offset, uint32_t maxCount) const;

    /// Find any entry (param or signal) by ID.
    EntryView find(uint64_t id) const;

    /// Find a parameter by ID.
    EntryView findParam(uint64_t id) const;

    /// Find a signal by ID.
    EntryView findSignal(uint64_t id) const;

    /// Register a fully annotated function. IDs are unique across all catalogs.
    bool addFunction(FunctionEntry entry);

    /// Total registered function count.
    uint32_t functionCount() const;

    /// Get a page of function descriptors.
    std::vector<FunctionView> functionPage(uint32_t offset, uint32_t maxCount) const;

    /// Find a function by ID.
    FunctionView findFunction(uint64_t id) const;

    /// Register a catalog change listener. Returns a handle for removal.
    size_t addChangeListener(CatalogChangeListener listener);

    /// Remove a catalog change listener by handle.
    void removeChangeListener(size_t handle);

    /// Get a monotonically increasing revision counter (incremented on each add).
    uint32_t revision() const { return revision_; }

    /// Configure the declarative stream-filter properties supported by this registry.
    void defineStreamFilterProperty(FilterPropertyDef definition);
    StreamFilterSchema::Result validateStreamFilter(const FilterProperty& property) const;
    bool supportsStreamFilters() const;

private:
    void notifyChange();

    mutable std::mutex mutex_;
    // EntryView intentionally contains non-owning pointers. Deque storage
    // keeps those pointers valid when entries are appended dynamically.
    std::deque<ParamEntry>  params_;
    std::deque<SignalEntry> signals_;
    std::deque<FunctionEntry> functions_;
    std::map<uint64_t, EntryKind> idMap_;  ///< id → kind for fast lookup
    StreamFilterSchema filterSchema_;
    uint32_t revision_ = 0;

    std::map<size_t, CatalogChangeListener> listeners_;
    size_t nextListenerHandle_ = 1;

    /// In-flight callback counter. `notifyChange` increments this under
    /// `mutex_` before invoking listeners outside the lock; `removeChangeListener`
    /// waits for it to reach zero before returning, guaranteeing that no
    /// in-flight callback can outlive a removed (and potentially destroyed)
    /// listener. This prevents use-after-free when a listener captures `this`
    /// and the owner is destroyed during `removeChangeListener`.
    mutable std::condition_variable notifyCv_;
    uint32_t notifyInFlight_ = 0;
};

}} // namespace tether::io

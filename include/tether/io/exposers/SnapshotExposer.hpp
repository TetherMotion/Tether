/**
 * @file SnapshotExposer.hpp
 * @brief Generic exposer for fixed-layout POD "snapshot" structs.
 *
 * Many subsystems keep their observable state in a plain POD stats struct
 * (cyclic-loop counters, supervisor state, axis state, ...).  Registering
 * each member as an individual hand-written signal is repetitive and the
 * resulting signals cannot be read atomically as a group.  SnapshotExposer
 * solves both with a single table-driven idiom:
 *
 * For every registered snapshot it emits:
 *  - "<name>"         — a Binary signal of `totalSize` bytes carrying a
 *                       StructDescriptor, so clients can DescribeStruct()
 *                       and decode every field without out-of-band schema.
 *  - "<name>.<field>" — one scalar signal per field, suitable for
 *                       individual polling and for ring-backed streaming.
 *
 * The snapshot read function is called on IO session threads and must fill
 * `totalSize` bytes at the destination; whatever locking or lifetime
 * guarantees the source struct needs belong inside that callback.
 *
 * Enum-valued fields may carry a `labels` string ("0=Init,1=PreOp,...")
 * which is copied into the scalar signal's metadata as `enum.<v>` keys so
 * clients can render names without a shared enum table.
 *
 * @copyright Copyright (C) 2025-2026 Tether Authors
 */
#pragma once

#include "tether/io/ParameterExposer.hpp"

#include <cstdint>
#include <cstdio>
#include <cstring>
#include <deque>
#include <string>
#include <vector>

namespace tether { namespace io { namespace exposers {

/// One scalar field inside a snapshot POD.
struct SnapshotField {
    const char* name;              ///< Field name: "<snap>.<name>" + struct member
    ValueType   type;              ///< Scalar ValueType of the member
    uint16_t    offset;            ///< Byte offset inside the POD (offsetof)
    uint16_t    size;              ///< Byte size of the member
    const char* unit   = "";       ///< Optional unit metadata ("us", "ns", ...)
    const char* labels = nullptr;  ///< Optional "v=name,..." enum labels
};

/**
 * @class SnapshotExposer
 * @brief Registers POD snapshots as self-describing Binary signals plus
 *        per-field scalar signals.
 *
 * Descriptor storage is owned by the exposer (std::deque — stable
 * addresses), so StructDescriptor pointers handed to the registry stay
 * valid for the exposer's lifetime.  As with all exposers, callers must
 * ensure the registry is cleared (and the server stopped) before the
 * exposer is destroyed.
 */
class SnapshotExposer : public IParameterExposer {
public:
    /// Largest snapshot POD the per-field scalar reads will copy through.
    static constexpr uint16_t kMaxSnapshotSize = 512;

    explicit SnapshotExposer(const char* moduleName) : moduleName_(moduleName) {}

    const char* moduleName() const override { return moduleName_.c_str(); }

    /**
     * Queue a snapshot for the next expose() call.
     *
     * @param localId            Local ID of the Binary snapshot signal;
     *                           scalar field signals take localId+1 …
     *                           localId+fields.size() — keep per-snapshot
     *                           ranges disjoint.
     * @param name               Signal name, e.g. "cyclic.stats".
     * @param description        Human-readable description.
     * @param group              Registry group, e.g. "ethercat.cyclic".
     * @param fields             Field table covering the POD layout.
     * @param totalSize          sizeof() the POD.
     * @param snapshot           Fills `totalSize` bytes at dest; runs on IO
     *                           session threads — must not block
     *                           indefinitely or touch invalid objects.
     * @param exposeScalarFields Also register one scalar signal per field.
     * @return false if the spec is invalid (oversized, no fields, no
     *         snapshot) and was not queued.
     */
    bool addSnapshot(uint32_t localId, std::string name,
                     std::string description, std::string group,
                     std::vector<SnapshotField> fields,
                     uint16_t totalSize, ReadFn snapshot,
                     bool exposeScalarFields = true) {
        if (!snapshot || fields.empty() || totalSize == 0 ||
            totalSize > kMaxSnapshotSize)
            return false;
        specs_.push_back(Spec{localId, std::move(name), std::move(description),
                              std::move(group), std::move(fields), totalSize,
                              std::move(snapshot), exposeScalarFields});
        return true;
    }

    void expose(Registry& registry, uint64_t idBase) override {
        for (const auto& spec : specs_) {
            const uint64_t snapId = makeId(idBase, spec.localId);
            descriptors_.push_back(buildDescriptor(spec, snapId));
            const StructDescriptor* sd = &descriptors_.back();
            ReadFn snap = spec.snapshot;
            const uint16_t ts = spec.totalSize;

            SignalEntry s;
            s.id          = snapId;
            s.name        = spec.name;
            s.description = spec.description;
            s.group       = spec.group;
            s.valueType   = ValueType::Binary;
            s.maxValueSize = ts;
            s.structDesc  = sd;
            s.varReadFn   = [snap, ts](void* d, size_t maxLen) -> size_t {
                if (maxLen < ts) return 0;
                snap(d);
                return ts;
            };
            registry.addSignal(std::move(s));

            if (!spec.exposeScalarFields) continue;
            uint32_t local = spec.localId + 1;
            for (const auto& f : spec.fields) {
                SignalEntry fs;
                fs.id          = makeId(idBase, local++);
                fs.name        = spec.name + "." + f.name;
                fs.description = spec.description + " — " + f.name;
                fs.group       = spec.group;
                fs.valueType   = f.type;
                if (f.unit && *f.unit) fs.metadata["unit"] = f.unit;
                addEnumLabels(f.labels, fs.metadata);
                const uint16_t off = f.offset;
                const uint16_t sz  = f.size;
                fs.readFn = [snap, ts, off, sz](void* d) {
                    uint8_t buf[kMaxSnapshotSize];
                    snap(buf);
                    std::memcpy(d, buf + off, sz);
                };
                registry.addSignal(std::move(fs));
            }
        }
    }

private:
    struct Spec {
        uint32_t                  localId;
        std::string               name;
        std::string               description;
        std::string               group;
        std::vector<SnapshotField> fields;
        uint16_t                  totalSize;
        ReadFn                    snapshot;
        bool                      exposeScalarFields;
    };

    static StructDescriptor buildDescriptor(const Spec& spec, uint64_t id) {
        StructDescriptor sd;
        sd.entryId   = id;
        sd.name      = spec.name;
        sd.totalSize = spec.totalSize;
        sd.fields.reserve(spec.fields.size());
        for (const auto& f : spec.fields) {
            sd.fields.push_back({f.name, f.type, f.offset, f.size,
                                 f.unit ? f.unit : ""});
        }
        return sd;
    }

    /// "0=Init,1=PreOp,4=SafeOp" -> metadata["enum.0"]="Init", ...
    static void addEnumLabels(const char* labels,
                              std::map<std::string, std::string>& md) {
        if (!labels) return;
        std::string work = labels;
        size_t pos = 0;
        while (pos <= work.size()) {
            const size_t comma = work.find(',', pos);
            const std::string item = work.substr(
                pos, comma == std::string::npos ? comma : comma - pos);
            const size_t eq = item.find('=');
            if (eq != std::string::npos && eq > 0) {
                md["enum." + item.substr(0, eq)] = item.substr(eq + 1);
            }
            if (comma == std::string::npos) break;
            pos = comma + 1;
        }
    }

    std::string                  moduleName_;
    std::vector<Spec>            specs_;
    std::deque<StructDescriptor> descriptors_;
};

}}} // namespace tether::io::exposers

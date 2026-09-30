#pragma once

#include "tether/io/Schema.hpp"
#include "tether/io/SchemaDigest.hpp"

#include <algorithm>
#include <cstdint>
#include <optional>
#include <vector>

namespace tether::io {

using SchemaSlot = uint32_t;
using SchemaEpoch = uint32_t;

struct SchemaManifestEntry {
    SchemaRef ref;
    uint32_t revision = 0;

    friend bool operator==(const SchemaManifestEntry&, const SchemaManifestEntry&) = default;
};

struct SchemaSlotEntry {
    SchemaSlot slot = 0;
    SchemaManifestEntry manifest;
    const SchemaNode* node = nullptr;
};

class SchemaCatalog {
public:
    explicit SchemaCatalog(SchemaLimits limits = {}) : limits_(limits) {}

    bool install(const SchemaGraph& graph, const std::vector<SchemaManifestEntry>& manifest) {
        if (!validateSchemaGraph(graph, limits_)) return false;
        std::vector<SchemaSlotEntry> next;
        next.reserve(manifest.size());
        for (const auto& entry : manifest) {
            const auto* node = graph.find(entry.ref.key);
            if (!node || node->revision != entry.revision) return false;
            if (computeSchemaDigest(*node) != entry.ref.digest) return false;
            if (std::any_of(next.begin(), next.end(), [&](const auto& existing) {
                    return existing.manifest.ref == entry.ref;
                })) return false;
            next.push_back({static_cast<SchemaSlot>(next.size()), entry, node});
        }
        std::sort(next.begin(), next.end(), [](const auto& left, const auto& right) {
            return left.manifest.ref.key < right.manifest.ref.key;
        });
        for (SchemaSlot slot = 0; slot < next.size(); ++slot) next[slot].slot = slot;
        graph_ = &graph;
        slots_ = std::move(next);
        ++epoch_;
        if (epoch_ == 0) ++epoch_;
        return true;
    }

    SchemaEpoch epoch() const { return epoch_; }
    size_t size() const { return slots_.size(); }
    const SchemaGraph* graph() const { return graph_; }

    std::vector<SchemaManifestEntry> manifest() const {
        std::vector<SchemaManifestEntry> result;
        result.reserve(slots_.size());
        for (const auto& entry : slots_) result.push_back(entry.manifest);
        return result;
    }

    std::optional<SchemaSlot> slotFor(const SchemaRef& ref) const {
        for (const auto& entry : slots_) {
            if (entry.manifest.ref == ref) return entry.slot;
        }
        return std::nullopt;
    }

    const SchemaNode* resolve(SchemaEpoch epoch, SchemaSlot slot) const {
        if (epoch != epoch_ || slot >= slots_.size()) return nullptr;
        return slots_[slot].node;
    }

    const SchemaManifestEntry* describe(SchemaSlot slot) const {
        if (slot >= slots_.size()) return nullptr;
        return &slots_[slot].manifest;
    }

private:
    SchemaLimits limits_;
    const SchemaGraph* graph_ = nullptr;
    std::vector<SchemaSlotEntry> slots_;
    SchemaEpoch epoch_ = 0;
};

} // namespace tether::io

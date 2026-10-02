#pragma once

#include "tether/io/Schema.hpp"

namespace tether::io {

SchemaDigest computeSchemaDigest(const SchemaNode& node);

/// Backfills every schema ref inside the graph with the digest of its target
/// node, iterating to a fixpoint (child digests feed into parent digests).
/// Returns false when the graph contains a ref to an unknown node or the
/// pass limit is exceeded (a dependency cycle would never converge).
inline bool resolveSchemaDigests(SchemaGraph& graph) {
    const auto resolveRef = [&graph](SchemaRef& ref) -> bool {
        const auto* target = graph.find(ref.key);
        if (!target) return false;
        ref.digest = computeSchemaDigest(*target);
        return true;
    };
    for (size_t pass = 0; pass <= graph.nodes.size(); ++pass) {
        bool changed = false;
        for (auto& node : graph.nodes) {
            const auto before = computeSchemaDigest(node);
            if (node.element && !resolveRef(*node.element)) return false;
            if (node.mapKey && !resolveRef(*node.mapKey)) return false;
            if (node.mapValue && !resolveRef(*node.mapValue)) return false;
            if (node.target && !resolveRef(*node.target)) return false;
            for (auto& field : node.fields) {
                if (!resolveRef(field.schema)) return false;
            }
            for (auto& [key, member] : node.oneOfMembers) {
                (void)key;
                if (!resolveRef(member)) return false;
            }
            changed = changed || computeSchemaDigest(node) != before;
        }
        if (!changed) return true;
    }
    return false;
}

} // namespace tether::io

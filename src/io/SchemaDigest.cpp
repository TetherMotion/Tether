#include "tether/io/SchemaDigest.hpp"

#include "blake3.h"

namespace tether::io {

SchemaDigest computeSchemaDigest(const SchemaNode& node) {
    const auto descriptor = canonicalDescriptor(node);
    blake3_hasher hasher;
    blake3_hasher_init(&hasher);
    blake3_hasher_update(&hasher, descriptor.data(), descriptor.size());

    SchemaDigest digest{};
    blake3_hasher_finalize(&hasher, digest.data(), digest.size());
    return digest;
}

} // namespace tether::io

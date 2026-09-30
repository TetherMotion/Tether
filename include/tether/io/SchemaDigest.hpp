#pragma once

#include "tether/io/Schema.hpp"

namespace tether::io {

SchemaDigest computeSchemaDigest(const SchemaNode& node);

} // namespace tether::io

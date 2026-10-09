/**
 * @file CoEErrorStrings.hpp
 * @brief CoE error-string helpers shared across CoEManager translation units.
 * @internal Internal header — not installed, not part of the public API.
 */

#pragma once

#include "tether/ethercat/CoETypes.hpp"
#include "tether/ethercat/SDOAbortCodes.hpp"

namespace EtherCAT {
namespace CoE {

/// Map a CoEError to the closest SDO abort code (preserving the slave's own
/// abort code when it carries one).
SDOAbortCode coeErrorToAbortCode(CoEError err);

} // namespace CoE
} // namespace EtherCAT

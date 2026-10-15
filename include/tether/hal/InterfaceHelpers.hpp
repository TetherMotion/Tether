#pragma once

#include <string>

namespace EtherCAT {
namespace HAL {

/**
 * @brief Resolve a requested interface name to a usable one.
 *
 * If @p requested is non-empty, it is returned as-is (caller explicitly
 * specified it).
 *
 * If empty, the system is queried for physical Ethernet interfaces via
 * getPhysicalEthernetInterfaces():
 *   - Exactly one found: auto-selected, logged, and returned.
 *   - Zero found: error logged, process exits.
 *   - More than one: error logged with the list, process exits.
 *
 * @param requested  The user-supplied interface name, or empty
 * @param tag        Log tag for diagnostics
 * @return           Interface name; exits the process on failure.
 */
std::string resolveInterface(const std::string& requested, const char* tag);

/// Enumerate and log all physical network interfaces (name, type, state,
/// MAC) to help the user pick one.
void listPhysicalInterfaces(const char* tag);

/// Log a consistent "permission denied" error message advising the user to
/// run via `runec` or `sudo`.  Call this when the Ethernet HAL returns
/// Error::PermissionDenied.
void logPermissionDeniedError(const char* tag);

} // namespace HAL
} // namespace EtherCAT

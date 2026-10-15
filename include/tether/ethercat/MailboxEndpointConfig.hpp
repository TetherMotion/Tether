#pragma once

#include <cstdint>
#include <string>

namespace EtherCAT {

// ============================================================================
// Mailbox endpoint configuration
//
// Parses and logs the mailbox in/out sizing and base addresses used when
// configuring a slave's SyncManagers for mailbox communication.
// ============================================================================

struct MailboxSizeConfig {
    uint16_t inSize = 256;
    uint16_t outSize = 256;
};

struct MailboxAddressConfig {
    uint16_t inAddress = 0x1000;
    uint16_t outAddress = 0x1200;
};

/// Parse a mailbox-size spec.  Single number sets both; `in:X,out:Y` sets
/// independently (values are decimal or 0x-hex, range 1-65535).
/// Returns false and prints to stderr on invalid input.
bool parseMailboxSize(const std::string& str, MailboxSizeConfig& out);

/// Parse a mailbox-address spec.  Expected format: `in:<hex>,out:<hex>`.
/// Validates that in-address != out-address.
/// Returns false and prints to stderr on invalid input.
bool parseMailboxAddress(const std::string& str, MailboxAddressConfig& out);

/// Log the mailbox configuration via TETHER_LOGI.
void logMailboxConfig(const MailboxSizeConfig& size,
                      const MailboxAddressConfig& addr,
                      const char* tag);

} // namespace EtherCAT

#pragma once

#include <cstdint>

namespace EtherCAT {

/**
 * @brief Per-Sync-Manager mailbox configuration entry.
 *
 * Used with Slave::configureMailbox() for explicit mailbox override.
 */
struct MailboxSyncManagerConfig {
    uint16_t address = 0;  ///< Start address in ESC RAM
    uint16_t length  = 0;  ///< Size in bytes
};

} // namespace EtherCAT

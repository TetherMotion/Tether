/**
 * @file MailboxRecovery.hpp
 * @brief Mailbox drain + SM activate-cycle recovery for stuck mailbox-full.
 *
 * @internal Internal header — not installed, not part of the public API.
 *
 * When a slave's mailbox-full flag latches at startup (stale data from a
 * previous session, or an ESC that only clears WRITE_BUF_FULL after a full
 * mailbox read), the master drains the SM1 buffer and, if that fails, cycles
 * the SM0/SM1 activate registers to clear the stuck condition.  Stateless
 * beyond the Master reference it borrows for register access.
 */

#pragma once

#include <cstdint>

namespace EtherCAT {

class Master;

class MailboxRecovery {
public:
    explicit MailboxRecovery(Master& master) : master_(master) {}

    /// Drain stale mailbox data from SM1 (slave→master).  Returns false when
    /// the mailbox is unconfigured or stays full after max_drain reads.
    bool drain(uint16_t slave_index, unsigned int max_drain);

    /// Cycle the SM1 activate register to clear a stuck WRITE_BUF_FULL.
    bool resetSM1(uint16_t slave_index);

    /// Cycle the SM0 activate register to clear a stuck mailbox-full.
    bool resetSM0(uint16_t slave_index);

private:
    /// Write the SMx activate register with a few retries — on a lossy link a
    /// single dropped frame must not abort the whole mailbox reset.
    bool writeActivateWithRetry(uint16_t slave_index, uint16_t activate_addr,
                                uint8_t value, const char* what,
                                const char* local_tag);

    Master& master_;
};

} // namespace EtherCAT

/**
 * @file MailboxRecovery.cpp
 * @brief Mailbox drain + SM activate-cycle recovery.
 */

#include "raw/MailboxRecovery.hpp"

#include "tether/ethercat/Master.hpp"
#include "tether/ethercat/CoEManager.hpp"
#include "tether/sii/SIIParser.hpp"
#include "raw/internal.hpp"
#include "sii/SIIReader.hpp"

#include <chrono>
#include <thread>
#include <vector>

namespace EtherCAT {

namespace {

// Determine whether SM0 is configured as write in SII (DEPRECATED/UNUSED).
// EtherCAT standard mandates: SM0=MbxIn(master→slave/write),
// SM1=MbxOut(slave→master/read).  Some device SII EEPROMs incorrectly
// specify reversed directions, but the master must ignore those errors and
// always configure according to the standard.  Retained for diagnostics only.
[[maybe_unused]] bool siiMailboxSM0IsWrite(Master& master, uint16_t slave_index)
{
    EtherCAT::SII::SIIData sii;
    if (!EtherCAT::SII::readSII(master, slave_index, sii) || sii.sm_count < 2)
        return false;
    return sii.sync_managers[0].control_register.direction;
}

} // namespace

bool MailboxRecovery::drain(uint16_t slave_index, unsigned int max_drain)
{
    const char* local_tag = "mbox_drain";

    uint16_t mbx_wr_addr = 0, mbx_wr_len = 0;
    uint16_t mbx_rd_addr = 0, mbx_rd_len = 0;
    if (!master_.sdoManager(slave_index).getMailbox(&mbx_wr_addr, &mbx_wr_len,
                                                    &mbx_rd_addr, &mbx_rd_len)) {
        TETHER_LOGW(local_tag, "{}: cannot drain mailbox, no mailbox configured", master_.slaveLogPrefix(slave_index).c_str());
        return false;
    }

    if (mbx_rd_len == 0) {
        TETHER_LOGW(local_tag, "{}: cannot drain mailbox, read length is zero", master_.slaveLogPrefix(slave_index).c_str());
        return false;
    }

    // Some ESCs only clear WRITE_BUF_FULL when the master reads the entire
    // configured mailbox length in one datagram. Allocate a buffer that matches
    // the SM length instead of capping at 256 bytes.
    std::vector<uint8_t> drain_buf(mbx_rd_len, 0);

    bool drained_any = false;
    for (unsigned int i = 0; i < max_drain; ++i) {
        uint8_t sm1_status = 0;
        if (!master_.readRegister(SlaveAddress(slave_index), Raw::sm_status_address(1), sm1_status, 100)) {
            TETHER_LOGW(local_tag, "{}: failed to read SM1 status while draining", master_.slaveLogPrefix(slave_index).c_str());
            return false;
        }

        // SM1 (slave→master mailbox) signals unread data via the mailbox-full
        // flag (bit 3, 0x08) per ETG.1000.4. Bit 7 (WRITE_BUFFER_FULL) is only
        // meaningful for 3-buffer PDO SMs and must not be treated as "mailbox
        // full" — doing so causes premature reads (WKC=0) on slaves that
        // transiently set bit 7 before writing the response.
        if ((sm1_status & Raw::EC_SM_STATUS_MBXFULL) == 0) {
            if (drained_any) {
                TETHER_LOGI(local_tag, "{}: SM1 drained successfully", master_.slaveLogPrefix(slave_index).c_str());
            }
            break;
        }

        if (!drained_any) {
            TETHER_LOGW(local_tag, "{}: SM1 full at startup (status=0x{:02X}). Draining stale mailbox data...",
                        master_.slaveLogPrefix(slave_index).c_str(), sm1_status);
        }

        if (!master_.readRegister(SlaveAddress(slave_index), mbx_rd_addr,
                          drain_buf.data(), static_cast<uint16_t>(drain_buf.size()), 200)) {
            TETHER_LOGW(local_tag, "{}: SM1 drain read failed, attempting SM1 activate reset", master_.slaveLogPrefix(slave_index).c_str());
            if (resetSM1(slave_index)) {
                // After a successful reset the buffer should be empty; re-check
                // before continuing so we don't loop on stale status.
                uint8_t sm1_status = 0;
                if (master_.readRegister(SlaveAddress(slave_index), Raw::sm_status_address(1), sm1_status, 100) &&
                    (sm1_status & Raw::EC_SM_STATUS_MBXFULL) == 0) {
                    TETHER_LOGI(local_tag, "{}: SM1 empty after reset", master_.slaveLogPrefix(slave_index).c_str());
                    return true;
                }
            }
            return false;
        }
        TETHER_LOGW(local_tag, "{}: drained stale mailbox data #{} (len={})",
                    master_.slaveLogPrefix(slave_index).c_str(), i + 1, static_cast<unsigned>(drain_buf.size()));
        drained_any = true;
        // Do NOT extract/sync the mailbox counter from drained stale responses.
        // Stale responses are from a previous session; the slave's counter has
        // been reset on INIT transition.  Syncing to the stale counter causes
        // all subsequent requests to fail with counter mismatch errors.
    }

    if (!drained_any) {
        // SM1 was never full — nothing to drain.
        return true;
    }

    // After max_drain reads, SM1 is still reporting full. The slave may be
    // continuously refilling the buffer or the ESC is not acknowledging the
    // reads; do not pretend the drain succeeded.
    TETHER_LOGE(local_tag, "{}: SM1 still full after {} drain attempts", master_.slaveLogPrefix(slave_index).c_str(), max_drain);
    return false;
}

bool MailboxRecovery::resetSM1(uint16_t slave_index)
{
    const char* local_tag = "mbox_reset";
    const uint16_t sm1_activate_addr = static_cast<uint16_t>(Raw::EC_REG_SM1 + 0x06u);
    const uint16_t sm1_status_addr = Raw::sm_status_address(1);

    TETHER_LOGW(local_tag, "{}: cycling SM1 activate register (0x{:04X}) to clear stuck WRITE_BUF_FULL",
                master_.slaveLogPrefix(slave_index).c_str(), sm1_activate_addr);

    uint8_t disable = 0x00;
    uint8_t enable = 0x01;
    if (!master_.writeRegister(SlaveAddress(slave_index), RegisterAddress(sm1_activate_addr), &disable, sizeof(disable), 200)) {
        TETHER_LOGE(local_tag, "{}: failed to disable SM1", master_.slaveLogPrefix(slave_index).c_str());
        return false;
    }
    if (!master_.writeRegister(SlaveAddress(slave_index), RegisterAddress(sm1_activate_addr), &enable, sizeof(enable), 200)) {
        TETHER_LOGE(local_tag, "{}: failed to re-enable SM1", master_.slaveLogPrefix(slave_index).c_str());
        return false;
    }

    uint8_t sm1_status = 0;
    if (!master_.readRegister(SlaveAddress(slave_index), RegisterAddress(sm1_status_addr), sm1_status, 100)) {
        TETHER_LOGE(local_tag, "{}: failed to read SM1 status after reset", master_.slaveLogPrefix(slave_index).c_str());
        return false;
    }

    if ((sm1_status & Raw::EC_SM_STATUS_MBXFULL) != 0) {
        TETHER_LOGE(local_tag, "{}: SM1 still full after activate reset (status=0x{:02X})",
                    master_.slaveLogPrefix(slave_index).c_str(), sm1_status);
        return false;
    }

    TETHER_LOGI(local_tag, "{}: SM1 reset successful", master_.slaveLogPrefix(slave_index).c_str());
    return true;
}

bool MailboxRecovery::writeActivateWithRetry(uint16_t slave_index,
                                             uint16_t activate_addr, uint8_t value,
                                             const char* what, const char* local_tag)
{
    for (int i = 0; i < 5; ++i) {
        if (master_.isCancelRequested()) {
            return false;
        }
        if (master_.writeRegister(SlaveAddress(slave_index),
                                 RegisterAddress(activate_addr),
                                 &value, sizeof(value), 200)) {
            return true;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
    TETHER_LOGE(local_tag, "{}: failed to {}", master_.slaveLogPrefix(slave_index).c_str(), what);
    return false;
}

bool MailboxRecovery::resetSM0(uint16_t slave_index)
{
    const char* local_tag = "mbox_reset";
    const uint16_t sm0_activate_addr = static_cast<uint16_t>(Raw::EC_REG_SM0 + 0x06u);
    const uint16_t sm0_status_addr = Raw::sm_status_address(0);

    TETHER_LOGW(local_tag, "{}: cycling SM0 activate register (0x{:04X}) to clear stuck mailbox-full",
                master_.slaveLogPrefix(slave_index).c_str(), sm0_activate_addr);

    if (!writeActivateWithRetry(slave_index, sm0_activate_addr, 0x00,
                                "disable SM0", local_tag)) {
        return false;
    }
    if (!writeActivateWithRetry(slave_index, sm0_activate_addr, 0x01,
                                "re-enable SM0", local_tag)) {
        return false;
    }

    uint8_t sm0_status = 0;
    if (!master_.readRegister(SlaveAddress(slave_index), RegisterAddress(sm0_status_addr), sm0_status, 100)) {
        TETHER_LOGE(local_tag, "{}: failed to read SM0 status after reset", master_.slaveLogPrefix(slave_index).c_str());
        return false;
    }

    if ((sm0_status & Raw::EC_SM_STATUS_MBXFULL) != 0) {
        TETHER_LOGE(local_tag, "{}: SM0 still full after activate reset (status=0x{:02X})",
                    master_.slaveLogPrefix(slave_index).c_str(), sm0_status);
        return false;
    }

    TETHER_LOGI(local_tag, "{}: SM0 reset successful", master_.slaveLogPrefix(slave_index).c_str());
    return true;
}

} // namespace EtherCAT

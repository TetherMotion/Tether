#include "tether/ethercat/Mailbox/Utils.hpp"

#include "logging/Logger.hpp"              // for TETHER_LOGI/W
#include "tether/ethercat/Master.hpp" // for Master
#include "tether/ethercat/Slave.hpp"  // full slave type for status dumps
#include "tether/ethercat/FaultDetection.hpp" // AL status helpers
#include "tether/ethercat/SyncManager/Utils.hpp" // for SM dumps

// NOTE: dumpSlaveSyncAndMailboxInfo(const CiA402Drive&, ...) is declared in
// this header (forward-declared CiA402Drive) but implemented in
// tether_cia_profiles (src/profiles/cia402/MailboxDiagnostics.cpp) to avoid a
// circular link dependency between the master core and the profile library.

namespace EtherCAT {
namespace Mailbox {
namespace Utils {

void logStatuswordDiagnostics(uint16_t statusword,
                              bool& warning_active,
                              uint64_t& warning_first_cycle,
                              uint64_t cycle,
                              const char* tag)
{
    bool warning_now = (statusword & (1u << 7)) != 0;
    if (warning_now && !warning_active) {
        warning_active = true;
        warning_first_cycle = cycle;
        TETHER_LOGW(tag, "[WARN] StatusWord Warning active (bit 7) at cycle %lu, StatusWord=0x%04X",
                 (unsigned long)cycle, statusword);
    } else if (!warning_now && warning_active) {
        warning_active = false;
        TETHER_LOGI(tag, "[WARN] StatusWord Warning cleared after %lu cycles",
                 (unsigned long)(cycle - warning_first_cycle));
    }

    if (statusword & (1u << 13)) {
        if (cycle % 500 == 0) {
            TETHER_LOGW(tag, "[ERR] Following error active (StatusWord bit 13), StatusWord=0x%04X", statusword);
        }
    }

    if (statusword & (1u << 11)) {
        if (cycle % 1000 == 0) {
            TETHER_LOGW(tag, "[ERR] Internal limit active (StatusWord bit 11), StatusWord=0x%04X", statusword);
        }
    }
}

void dumpHeaderAndStatus(Master& master,
                         uint16_t slave_idx,
                         const char* tag)
{
    // Dump SM0/SM1 status registers (including watchdog) via SyncManagerAccessor
    master.slave(slave_idx).sm(0).dumpMailboxStatus(tag);
    master.slave(slave_idx).sm(1).dumpMailboxStatus(tag);

    // Peek at mailbox header content (mailbox-buffer-specific, not SM-register-specific)
    uint16_t mbx_wr_addr = 0, mbx_wr_len = 0, mbx_rd_addr = 0, mbx_rd_len = 0;
    bool mbx_cfg = master.sdoManager(slave_idx).getMailbox(
                                                      &mbx_wr_addr, &mbx_wr_len,
                                                      &mbx_rd_addr, &mbx_rd_len);
    if (mbx_cfg) {
        uint8_t mbx_header[6] = {0};
        if (master.readRegister(SlaveAddress(slave_idx), mbx_wr_addr, mbx_header, sizeof(mbx_header), 200)) {
            uint16_t length   = mbx_header[0] | (mbx_header[1] << 8);
            uint16_t address  = mbx_header[2] | (mbx_header[3] << 8);
            uint8_t  channel  = mbx_header[4];
            uint8_t  priority = mbx_header[5] >> 6;
            uint8_t  type     = mbx_header[5] & 0x0F;
            TETHER_LOGI(tag, "[DEBUG] Mailbox WR (0x%04X) header: len=%u addr=0x%04X ch=%u prio=%u type=0x%X",
                     mbx_wr_addr, length, address, channel, priority, type);
        } else {
            TETHER_LOGW(tag, "[DEBUG] Mailbox WR read FAILED (addr=0x%04X)", mbx_wr_addr);
        }

        if (master.readRegister(SlaveAddress(slave_idx), mbx_rd_addr, mbx_header, sizeof(mbx_header), 200)) {
            uint16_t length   = mbx_header[0] | (mbx_header[1] << 8);
            uint16_t address  = mbx_header[2] | (mbx_header[3] << 8);
            uint8_t  channel  = mbx_header[4];
            uint8_t  priority = mbx_header[5] >> 6;
            uint8_t  type     = mbx_header[5] & 0x0F;
            TETHER_LOGI(tag, "[DEBUG] Mailbox RD (0x%04X) header: len=%u addr=0x%04X ch=%u prio=%u type=0x%X",
                     mbx_rd_addr, length, address, channel, priority, type);
        } else {
            TETHER_LOGW(tag, "[DEBUG] Mailbox RD read FAILED (addr=0x%04X)", mbx_rd_addr);
        }
    } else {
        TETHER_LOGW(tag, "SDO mailbox configuration unavailable for slave %u", slave_idx);
    }
}

} // namespace Utils
} // namespace Mailbox
} // namespace EtherCAT

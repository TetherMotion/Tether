#include "tether/ethercat/SDOMailboxIO.hpp"
#include "tether/ethercat/SDODiagnostics.hpp"
#include "tether/ethercat/Master.hpp"
#include "tether/platform/Platform.hpp"
#include "raw/internal.hpp"
#include <chrono>
#include <thread>
#include <vector>

namespace EtherCAT {
namespace Raw {

static const char* TAG = "ethercat";

SDOMailboxIO::SDOMailboxIO(SDODiagnostics& diag)
    : diag_(diag) {}

void SDOMailboxIO::drainStale(Master& master, uint16_t adp,
                               uint16_t mbxReadAddr, uint16_t mbxReadLen,
                               unsigned int maxDrain) {
    if (mbxReadLen == 0) {
        return;
    }

    std::vector<uint8_t> drain_buf(mbxReadLen, 0);

    for (unsigned int i = 0; i < maxDrain; ++i) {
        uint8_t sm1_status = 0;
        if (!master.readRegister(Master::slaveAddressFromADP(adp), sm_status_address(1), sm1_status, 100)) {
            break;
        }
        if ((sm1_status & EC_SM_STATUS_MBXFULL) == 0) {
            if (i > 0) {
                TETHER_LOGI(TAG, "SM1 drained successfully (adp=0x%04X)", adp);
            }
            break;
        }
        if (i == 0) {
            TETHER_LOGW(TAG, "Stale mailbox data detected (SM1 full, adp=0x%04X). Draining before new SDO request.",
                        adp);
        }
        if (!master.readRegister(Master::slaveAddressFromADP(adp), mbxReadAddr,
                                 drain_buf.data(), static_cast<uint16_t>(drain_buf.size()), 200)) {
            TETHER_LOGW(TAG, "SM1 drain read failed (adp=0x%04X), attempting SM1 activate reset", adp);
            const uint16_t slave_index = Master::slaveAddressFromADP(adp).slavePosition();
            if (master.resetSlaveMailboxSM1(slave_index)) {
                uint8_t sm1_status = 0;
                if (master.readRegister(Master::slaveAddressFromADP(adp), sm_status_address(1), sm1_status, 100) &&
                    (sm1_status & EC_SM_STATUS_MBXFULL) == 0) {
                    TETHER_LOGI(TAG, "SM1 empty after reset (adp=0x%04X)", adp);
                    break;
                }
            }
            break;
        }
        TETHER_LOGW(TAG, "Drained stale mailbox data #%u (adp=0x%04X, len=%u)",
                    i + 1, adp, static_cast<unsigned>(drain_buf.size()));
    }
}

bool SDOMailboxIO::waitSm0NotFull(Master& master, uint16_t adp,
                                   unsigned int timeoutMs,
                                   unsigned int pollIntervalMs) {
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(timeoutMs);
    while (std::chrono::steady_clock::now() < deadline) {
        if (master.isCancelRequested()) {
            return false;
        }
        uint8_t sm0_status = 0;
        if (master.readRegister(Master::slaveAddressFromADP(adp), sm_status_address(0), sm0_status, 100)) {
            if ((sm0_status & EC_SM_STATUS_MBXFULL) == 0) {
                return true;
            }
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(pollIntervalMs));
    }
    TETHER_LOGE(TAG, "SM0 mailbox stayed full (adp=0x%04X) after %ums timeout — slave PDI not draining mailbox",
                adp, timeoutMs);

    // Last-resort recovery: cycle SM0 activate register to flush the stuck
    // write buffer.  This clears the mailbox-full flag so the next SDO attempt
    // can write its request.  The slave never read the previous request, so
    // no valid response is lost.
    const uint16_t slave_index = Master::slaveAddressFromADP(adp).slavePosition();
    if (master.resetSlaveMailboxSM0(slave_index)) {
        TETHER_LOGI(TAG, "SM0 reset succeeded (adp=0x%04X) — mailbox ready for next write", adp);
        return true;
    }
    return false;
}

bool SDOMailboxIO::apwrWithWkcProbe(Master& master, uint16_t adp,
                                     uint16_t primaryAddr, uint16_t altAddr,
                                     const uint8_t* payload, uint16_t payloadLen,
                                     unsigned int timeoutMs, bool* outUsedAlt) {
    if (outUsedAlt) *outUsedAlt = false;

    if (!waitSm0NotFull(master, adp, timeoutMs)) {
        TETHER_LOGE(TAG, "mailbox write aborted: SM0 still full (adp=0x%04X addr=0x%04X)", adp, primaryAddr);
        return false;
    }

    if (master.writeRegister(Master::slaveAddressFromADP(adp), primaryAddr, payload, payloadLen, timeoutMs)) {
        return true;
    }

    if (master.lastWkc() == 0) {
        TETHER_LOGE(TAG, "mailbox transaction failed: Working counter is 0 (adp=0x%04X addr=0x%04X)", adp, primaryAddr);
        return false;
    }

    TETHER_LOGW(TAG, "SDO mailbox APWR not acknowledged for adp=0x%04X addr=0x%04X (len=%u) after retries. Probing alt addr=0x%04X...",
                adp, primaryAddr, (unsigned)payloadLen, altAddr);

    if (master.writeRegister(Master::slaveAddressFromADP(adp), altAddr, payload, payloadLen, timeoutMs)) {
        if (outUsedAlt) *outUsedAlt = true;
        TETHER_LOGW(TAG, "SDO mailbox APWR acknowledged on alt addr=0x%04X. Treating mailbox wr/rd as swapped for this SDO op.",
                    altAddr);
        return true;
    }

    TETHER_LOGE(TAG, "SDO mailbox APWR not acknowledged on both addr=0x%04X and alt=0x%04X (adp=0x%04X)",
                primaryAddr, altAddr, adp);
    diag_.dumpSlaveState(master, adp, primaryAddr, altAddr);
    return false;
}

bool SDOMailboxIO::pollSm1Full(Master& master, uint16_t adp,
                                unsigned int timeoutMs,
                                unsigned int pollIntervalMs) {
    if (pollIntervalMs == 0) {
        pollIntervalMs = 1;
    }
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(timeoutMs);
    while (std::chrono::steady_clock::now() < deadline) {
        if (master.isCancelRequested()) {
            return false;
        }
        uint8_t sm1_status = 0;
        if (master.readRegister(Master::slaveAddressFromADP(adp), sm_status_address(1), sm1_status, 100)) {
            if ((sm1_status & EC_SM_STATUS_MBXFULL) != 0) {
                return true;
            }
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(pollIntervalMs));
    }
    return false;
}

} // namespace Raw
} // namespace EtherCAT

#include "tether/ethercat/SDOTransactionBase.hpp"
#include "tether/ethercat/SDOErrorDecoder.hpp"
#include "tether/ethercat/SDOMailboxIO.hpp"
#include "tether/ethercat/SDODiagnostics.hpp"
#include "tether/ethercat/Master.hpp"
#include "tether/platform/Platform.hpp"
#include "raw/internal.hpp"
#include <chrono>
#include <thread>
#include <cstring>
#include <cstdio>

namespace EtherCAT {
namespace Raw {

static const char* TAG = "ethercat";

static uint16_t slaveIndexFromADP(uint16_t adp) {
    return Master::slaveAddressFromADP(adp).slavePosition();
}

SDOTransactionBase::SDOTransactionBase(SDOErrorDecoder& errorDecoder,
                                       SDOMailboxIO& mailboxIO,
                                       SDODiagnostics& diagnostics)
    : errorDecoder_(errorDecoder)
    , mailboxIO_(mailboxIO)
    , diagnostics_(diagnostics) {}

bool SDOTransactionBase::sendAndWait(Master& master, uint16_t adp,
                                     uint16_t mbxWriteAddr, uint16_t mbxWriteLen,
                                     uint16_t mbxReadAddr, uint16_t mbxReadLen,
                                     uint8_t* mbxbuf, unsigned int timeoutMs,
                                     unsigned int pollIntervalMs,
                                     unsigned int transactionTimeoutMs,
                                     const char* phaseLabel) {
    if (!mailboxIO_.apwrWithWkcProbe(master, adp,
                                     mbxWriteAddr, mbxReadAddr,
                                     mbxbuf,
                                     static_cast<uint16_t>(mbxWriteLen),
                                     timeoutMs, nullptr)) {
        TETHER_LOGE(TAG, "SDO %s: re-send failed after stale response (adp=0x%04X)",
                    phaseLabel, adp);
        return false;
    }
    if (!mailboxIO_.pollSm1Full(master, adp, transactionTimeoutMs, pollIntervalMs)) {
        TETHER_LOGE(TAG, "SDO %s: SM1 never full after re-send (adp=0x%04X timeout=%ums)",
                    phaseLabel, adp, transactionTimeoutMs);
        diagnostics_.dumpSlaveState(master, adp, mbxWriteAddr, mbxReadAddr);
        return false;
    }
    return true;
}

MbxPollOutcome SDOTransactionBase::pollSm1AndRead(Master& master, uint16_t adp,
                                                   uint16_t mbxReadAddr, uint16_t mbxReadLen,
                                                   uint8_t* mbxbuf, unsigned int pollIntervalMs,
                                                   MbxResponseHeader& hdr) {
    if (master.isCancelRequested()) {
        return MbxPollOutcome::Cancelled;
    }

    uint8_t sm1_status = 0;
    if (!master.readRegister(Master::slaveAddressFromADP(adp), sm_status_address(1), sm1_status, 100)) {
        std::this_thread::sleep_for(std::chrono::milliseconds(pollIntervalMs));
        return MbxPollOutcome::Sm1Empty;
    }
    if ((sm1_status & EC_SM_STATUS_MBXFULL) == 0) {
        std::this_thread::sleep_for(std::chrono::milliseconds(pollIntervalMs));
        return MbxPollOutcome::Sm1Empty;
    }

    if (!master.readRegister(Master::slaveAddressFromADP(adp), mbxReadAddr, mbxbuf,
                             static_cast<uint16_t>(mbxReadLen), 200)) {
        return MbxPollOutcome::ReadFailed;
    }

    MbxHeader r_mbx;
    std::memcpy(&r_mbx, mbxbuf, sizeof(r_mbx));
    hdr.len = le16_to_host(r_mbx.length_le);
    hdr.type = static_cast<uint8_t>(r_mbx.mbxtype & 0x0Fu);
    hdr.cnt = static_cast<uint8_t>((r_mbx.mbxtype >> 4) & 0x0Fu);
    hdr.priority = r_mbx.priority;
    hdr.rawMbxType = r_mbx.mbxtype;

    return MbxPollOutcome::GotData;
}

bool SDOTransactionBase::handleMailboxError(const uint8_t* mbxbuf, const MbxResponseHeader& hdr,
                                            uint16_t adp, uint16_t index, uint8_t sub) {
    if (hdr.len >= 4) {
        const uint16_t err = le16_to_host(*reinterpret_cast<const uint16_t*>(mbxbuf + sizeof(MbxHeader) + 0));
        const uint16_t detail = le16_to_host(*reinterpret_cast<const uint16_t*>(mbxbuf + sizeof(MbxHeader) + 2));
        TETHER_LOGE(TAG, "Slave %u: Mailbox error response: cnt=%u err=0x%04X (%s) detail=0x%04X (%s) (index=0x%04X:%u)",
                    slaveIndexFromADP(adp), hdr.cnt, err, errorDecoder_.mbxErrorCodeStr(err),
                    detail, errorDecoder_.mbxErrorDetailStr(err, detail), index, sub);
    } else {
        TETHER_LOGE(TAG, "Slave %u: Mailbox error response (truncated): cnt=%u len=%u (index=0x%04X:%u)",
                    slaveIndexFromADP(adp), hdr.cnt, hdr.len, index, sub);
    }
    return false;
}

bool SDOTransactionBase::isCounterMismatchError(const uint8_t* mbxbuf, const MbxResponseHeader& hdr) {
    if (hdr.type != EC_MBXT_ERR || hdr.len < 4) {
        return false;
    }
    const uint16_t err = le16_to_host(*reinterpret_cast<const uint16_t*>(mbxbuf + sizeof(MbxHeader) + 0));
    const uint16_t detail = le16_to_host(*reinterpret_cast<const uint16_t*>(mbxbuf + sizeof(MbxHeader) + 2));
    // ETG.1000.6 error 0x0001 = "Syntax error in mailbox message".
    // detail 0x0004 = offset of priority byte, 0x0005 = offset of type/counter
    // byte in the mailbox header.  Either indicates the slave rejected the
    // mailbox header — most commonly due to a counter mismatch.
    return (err == 0x0001 && (detail == 0x0004 || detail == 0x0005));
}

bool SDOTransactionBase::checkStaleCounter(Master& master, uint16_t adp,
                                           uint16_t mbxWriteAddr, uint16_t mbxWriteLen,
                                           uint16_t mbxReadAddr, uint16_t mbxReadLen,
                                           uint8_t* mbxbuf, unsigned int pollIntervalMs,
                                           unsigned int transactionTimeoutMs,
                                           const MbxResponseHeader& hdr,
                                           uint8_t* inoutMbxCnt, uint8_t& inOutExpectedCnt,
                                           int& staleRetryCount,
                                           uint16_t index_, uint8_t sub_,
                                           const char* phaseLabel) {
    TETHER_LOGW(TAG, "Slave %u: Stale mailbox response (%s): cnt=%u expected=%u (index=0x%04X:%u) — clearing and re-sending",
                slaveIndexFromADP(adp), phaseLabel, hdr.cnt, inOutExpectedCnt, index_, sub_);
    // Do NOT sync the counter here.  A stale response (wrong cnt or wrong
    // index/sub) is typically a leftover from a previous session that was
    // not drained.  The slave's counter has been reset on INIT transition,
    // so the master's counter is correct — we just need to drain the stale
    // response and retry with the SAME counter.
    // Counter syncing on stale responses causes regressions on slaves that
    // DO reset their counter (e.g. Synapticon ESC211): the master syncs to
    // the stale counter, which is wrong, and all subsequent requests fail.
    (void)inoutMbxCnt;  // Unused — no counter sync
    if (++staleRetryCount <= MAX_STALE_RETRIES) {
        if (!sendAndWait(master, adp, mbxWriteAddr, mbxWriteLen,
                         mbxReadAddr, mbxReadLen, mbxbuf, 500,
                         pollIntervalMs, transactionTimeoutMs, phaseLabel)) {
            return false;
        }
    } else {
        std::this_thread::sleep_for(std::chrono::milliseconds(pollIntervalMs));
    }
    return true;
}

} // namespace Raw
} // namespace EtherCAT

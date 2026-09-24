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
        if (!master.isCancelRequested()) {
            TETHER_LOGE(TAG, "SDO {}: re-send failed after stale response (adp=0x{:04X})",
                        phaseLabel, adp);
        }
        return false;
    }
    if (!mailboxIO_.pollSm1Full(master, adp, transactionTimeoutMs, pollIntervalMs)) {
        if (!master.isCancelRequested()) {
            TETHER_LOGE(TAG, "SDO {}: SM1 never full after re-send (adp=0x{:04X} timeout={}ms)",
                        phaseLabel, adp, transactionTimeoutMs);
            diagnostics_.dumpSlaveState(master, adp, mbxWriteAddr, mbxReadAddr);
        }
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
        TETHER_LOGE(TAG, "Slave {}: Mailbox error response: cnt={} err=0x{:04X} ({}) detail=0x{:04X} ({}) (index=0x{:04X}:{})",
                    slaveIndexFromADP(adp), hdr.cnt, err, errorDecoder_.mbxErrorCodeStr(err),
                    detail, errorDecoder_.mbxErrorDetailStr(err, detail), index, sub);
    } else {
        TETHER_LOGE(TAG, "Slave {}: Mailbox error response (truncated): cnt={} len={} (index=0x{:04X}:{})",
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

bool SDOTransactionBase::adoptCounterOnEcho(uint16_t adp,
                                           const uint8_t* reqbuf,
                                           const uint8_t* rspbuf,
                                           uint16_t mbxReadLen,
                                           const MbxResponseHeader& hdr,
                                           uint8_t* inoutMbxCnt,
                                           uint8_t& curCnt,
                                           uint8_t& expectedCnt,
                                           uint16_t index, uint8_t sub,
                                           const char* phaseLabel) {
    if (hdr.type != EC_MBXT_COE) {
        return false;
    }
    const size_t sdo_off = sizeof(MbxHeader) + sizeof(CoeHeader);
    if (static_cast<size_t>(hdr.len) < sizeof(CoeHeader) + 4 ||
        sdo_off + 4 > mbxReadLen) {
        return false;
    }
    // Only response-type commands carry a meaningful index echo: the init
    // download response (0x60), the init upload response (ccs=2), and the
    // SDO abort (0x80) all place index_le/sub at sdo_off+1/+3.
    const uint8_t sdo_cmd = rspbuf[sdo_off];
    const bool is_echoing =
        (sdo_cmd & 0xE0u) == 0x60u ||
        ((sdo_cmd >> 5) & 0x07u) == 2u ||
        (sdo_cmd & 0xE0u) == EC_SDO_ABORT;
    if (!is_echoing) {
        return false;
    }
    const uint16_t r_index =
        le16_to_host(*reinterpret_cast<const uint16_t*>(rspbuf + sdo_off + 1));
    const uint8_t r_sub = rspbuf[sdo_off + 3];
    if (r_index != index || r_sub != sub) {
        return false;
    }

    // The response answers our request — the slave simply runs its own
    // persistent response-counter sequence (e.g. SOMANET keeps counting
    // across master restarts and INIT transitions).  Adopt its numbering:
    // this response is valid (expectedCnt = hdr.cnt) and the next request
    // continues from nextMbxCnt(hdr.cnt), restoring master/slave counter
    // lockstep for subsequent transactions.
    const uint8_t last_req = static_cast<uint8_t>((reqbuf[5] >> 4) & 0x07u);
    uint8_t next_req = SDOMailboxIO::nextMbxCnt(hdr.cnt);
    if (next_req == last_req) {
        // Never reuse the counter of the request we just sent — slaves with
        // duplicate detection would drop the next request as a retransmit.
        next_req = SDOMailboxIO::nextMbxCnt(next_req);
    }
    TETHER_LOGI(TAG,
        "Slave {}: adopting slave mailbox counter: response cnt={} "
        "(expected {}) echoes index=0x{:04X}:{} ({}) — synced, next req cnt={}",
        slaveIndexFromADP(adp), hdr.cnt, expectedCnt, index, sub,
        phaseLabel, next_req);
    expectedCnt = hdr.cnt;
    curCnt = next_req;
    if (inoutMbxCnt != nullptr) {
        *inoutMbxCnt = next_req;
    }
    return true;
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
    TETHER_LOGW(TAG, "{}: Stale mailbox response ({}): cnt={} expected={} (index=0x{:04X}:{}) — clearing and re-sending",
                master.slaveLogPrefix(slaveIndexFromADP(adp)).c_str(), phaseLabel, hdr.cnt, inOutExpectedCnt, index_, sub_);
    // This response is genuinely stale — the caller already checked
    // adoptCounterOnEcho() so it neither echoes our index nor matches the
    // expected counter.  Drain it and re-send the request.
    //
    // The re-send MUST use a fresh counter: slaves that use the mailbox
    // counter for duplicate detection (e.g. Synapticon SOMANET) silently
    // drop a request whose counter repeats the last one seen.  The bumped
    // counter is written back to the request buffer and becomes the new
    // expected response counter for slaves that echo the request counter
    // (e.g. ESC211); slaves with an independent response counter are still
    // handled by adoptCounterOnEcho() when the response arrives.
    if (++staleRetryCount <= MAX_STALE_RETRIES) {
        const uint8_t prev_req = static_cast<uint8_t>((mbxbuf[5] >> 4) & 0x07u);
        const uint8_t new_req = SDOMailboxIO::nextMbxCnt(prev_req);
        mbxbuf[5] = mbx_type_with_cnt(mbxbuf[5] & 0x0Fu, new_req);
        inOutExpectedCnt = new_req;
        if (inoutMbxCnt != nullptr) {
            *inoutMbxCnt = SDOMailboxIO::nextMbxCnt(new_req);
        }
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

MbxResyncResult SDOTransactionBase::resyncMailboxCounter(
    Master& master, uint16_t adp,
    uint16_t mbxWriteAddr, uint16_t mbxWriteLen,
    uint16_t mbxReadAddr, uint16_t mbxReadLen,
    uint8_t* reqbuf, uint8_t* rspbuf,
    uint8_t* inoutMbxCnt, uint8_t& curCnt, uint8_t& expectedCnt,
    uint16_t index, uint8_t sub,
    bool verifyIndexEcho,
    unsigned int pollIntervalMs,
    const char* phaseLabel,
    MbxResponseHeader& outHdr) {
    if (pollIntervalMs == 0) pollIntervalMs = 1;

    TETHER_LOGW(TAG,
        "{}: mailbox counter mismatch ({}): slave retained its counter "
        "across restart — probing counter space to resync (index=0x{:04X}:{})",
        master.slaveLogPrefix(slaveIndexFromADP(adp)).c_str(), phaseLabel, index, sub);

    const size_t sdo_offset = sizeof(MbxHeader) + sizeof(CoeHeader);
    uint8_t cand = static_cast<uint8_t>((reqbuf[5] >> 4) & 0x07u);
    if (cand == 0) cand = 1;

    // Probe the remaining counter values.  Each probe is a fresh counter so
    // duplicate-detecting slaves never see a repeat; the probe that lands
    // on the slave's expected counter is processed and answered.
    for (int probe = 0; probe < 6; ++probe) {
        if (master.isCancelRequested()) {
            return MbxResyncResult::Cancelled;
        }
        cand = SDOMailboxIO::nextMbxCnt(cand);
        reqbuf[5] = mbx_type_with_cnt(reqbuf[5] & 0x0Fu, cand);

        mailboxIO_.drainStale(master, adp, mbxReadAddr, mbxReadLen, 4);
        if (master.isCancelRequested()) {
            return MbxResyncResult::Cancelled;
        }
        if (!mailboxIO_.apwrWithWkcProbe(master, adp, mbxWriteAddr, mbxReadAddr,
                                       reqbuf, static_cast<uint16_t>(mbxWriteLen),
                                       500, nullptr)) {
            if (master.isCancelRequested()) {
                return MbxResyncResult::Cancelled;
            }
            return MbxResyncResult::Failed;
        }

        // Wait for the response.  Counter-mismatch errors arrive within a
        // few ms; a valid response takes as long as the slave needs.  Stale
        // or unrelated responses are skipped while the probe stays live.
        MbxResponseHeader hdr{};
        for (int wait = 0; wait < 50; ++wait) {
            const auto outcome = pollSm1AndRead(master, adp, mbxReadAddr,
                                                mbxReadLen, rspbuf,
                                                pollIntervalMs, hdr);
            if (outcome == MbxPollOutcome::Cancelled) {
                return MbxResyncResult::Cancelled;
            }
            if (outcome != MbxPollOutcome::GotData) {
                continue;
            }

            if (hdr.type == EC_MBXT_ERR) {
                if (isCounterMismatchError(rspbuf, hdr)) {
                    break;   // rejected counter — try next candidate
                }
                return MbxResyncResult::Failed;   // genuine mailbox error
            }
            if (hdr.type != EC_MBXT_COE) {
                continue;   // FoE/EoE or other noise — keep waiting
            }
            if (verifyIndexEcho) {
                // Init-phase SDO responses (incl. aborts) echo index:sub at
                // a fixed offset; a non-echoing CoE frame is stale.
                if (static_cast<size_t>(hdr.len) < sizeof(CoeHeader) + 4 ||
                    sizeof(MbxHeader) + hdr.len > mbxReadLen ||
                    le16_to_host(*reinterpret_cast<const uint16_t*>(
                                     rspbuf + sdo_offset + 1)) != index ||
                    rspbuf[sdo_offset + 3] != sub) {
                    continue;
                }
            }

            // Accepted — the slave processed our probe.  Adopt the slave's
            // response counter (its own sequence) as the new lockstep base.
            uint8_t next_req = SDOMailboxIO::nextMbxCnt(hdr.cnt);
            if (next_req == cand) {
                next_req = SDOMailboxIO::nextMbxCnt(next_req);
            }
            if (inoutMbxCnt != nullptr) {
                *inoutMbxCnt = next_req;
            }
            curCnt = next_req;
            expectedCnt = hdr.cnt;
            outHdr = hdr;
            TETHER_LOGI(TAG,
                "{}: mailbox counter resynced (request cnt={}, "
                "response cnt={}, {})",
                master.slaveLogPrefix(slaveIndexFromADP(adp)).c_str(), cand, hdr.cnt, phaseLabel);
            return MbxResyncResult::Recovered;
        }
        // Probe produced no usable response — try the next counter.
    }

    TETHER_LOGE(TAG,
        "{}: mailbox counter resync failed — no counter accepted ({})",
        master.slaveLogPrefix(slaveIndexFromADP(adp)).c_str(), phaseLabel);
    return MbxResyncResult::Failed;
}

} // namespace Raw
} // namespace EtherCAT

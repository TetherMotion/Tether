#pragma once

#include <cstdint>
#include "tether/ethercat/EtherCATConfig.hpp"

namespace EtherCAT { class Master; }

namespace EtherCAT {
namespace Raw {

class SDOErrorDecoder;
class SDOMailboxIO;
class SDODiagnostics;

struct MbxResponseHeader {
    uint16_t len;
    uint8_t type;
    uint8_t cnt;
    uint8_t priority;
    uint8_t rawMbxType;
};

enum class MbxPollOutcome {
    GotData,         // SM1 full, mailbox data read successfully
    Sm1Empty,        // SM1 not full, should retry
    ReadFailed,      // WKC=0 on data read despite SM1 full
    Cancelled,       // Cancel requested
};

enum class MbxResyncResult : uint8_t {
    Recovered,       // Counter resynced; accepted response left in rspbuf/outHdr
    Failed,          // No counter accepted (or unrecoverable transport error)
    Cancelled,       // Cancel requested mid-resync
};

class SDOTransactionBase {
public:
    virtual ~SDOTransactionBase() = default;

    SDOTransactionBase(SDOErrorDecoder& errorDecoder,
                       SDOMailboxIO& mailboxIO,
                       SDODiagnostics& diagnostics);

    bool sendAndWait(Master& master, uint16_t adp,
                     uint16_t mbxWriteAddr, uint16_t mbxWriteLen,
                     uint16_t mbxReadAddr, uint16_t mbxReadLen,
                     uint8_t* mbxbuf, unsigned int timeoutMs,
                     unsigned int pollIntervalMs,
                     unsigned int transactionTimeoutMs,
                     const char* phaseLabel);

    MbxPollOutcome pollSm1AndRead(Master& master, uint16_t adp,
                                  uint16_t mbxReadAddr, uint16_t mbxReadLen,
                                  uint8_t* mbxbuf, unsigned int pollIntervalMs,
                                  MbxResponseHeader& hdr);

    bool handleMailboxError(const uint8_t* mbxbuf, const MbxResponseHeader& hdr,
                            uint16_t adp, uint16_t index, uint8_t sub);

    // Check if a mailbox error response indicates a counter mismatch —
    // specifically, ETG.1000.6 error code 0x0001 (Syntax error in mailbox
    // message) with detail 0x0004 or 0x0005 (offset of the priority or
    // type/counter byte in the mailbox header).  Slaves that validate the
    // incoming mailbox counter (e.g. ESC211) return this error when the
    // master's counter doesn't match the slave's expected value.
    static bool isCounterMismatchError(const uint8_t* mbxbuf, const MbxResponseHeader& hdr);

    // Check for a stale mailbox response (counter mismatch) and drain it.
    // The response was already consumed by the caller's poll; this helper
    // only logs, throttles, and returns — it deliberately does NOT re-send
    // the request.  Slaves like the ESC211 queue SM0 writes internally and
    // answer FIFO, ~25-30 ms per response; re-sending on every stale
    // response feeds that queue faster than it drains and the device
    // eventually faults (AL 0x0016, INIT+ERR).  The caller's poll loop keeps
    // draining until our response surfaces or the quiet deadline expires.
    //
    // @param inoutMbxCnt  Pointer to the master's persistent mailbox counter
    //                     (unused — no re-send occurs).
    // @param inOutExpectedCnt  Expected response counter (unchanged).
    // @return always true — the caller should continue polling.
    bool checkStaleCounter(Master& master, uint16_t adp,
                           uint16_t mbxWriteAddr, uint16_t mbxWriteLen,
                           uint16_t mbxReadAddr, uint16_t mbxReadLen,
                           uint8_t* mbxbuf, unsigned int pollIntervalMs,
                           unsigned int transactionTimeoutMs,
                           const MbxResponseHeader& hdr,
                           uint8_t* inoutMbxCnt, uint8_t& inOutExpectedCnt,
                           int& staleRetryCount,
                           uint16_t index_, uint8_t sub_,
                           const char* phaseLabel);

    // Decide whether a counter-mismatched response actually answers our
    // request, and if so adopt the slave's counter sequence.
    //
    // Two different slave behaviours produce "hdr.cnt != expected":
    //   a) A stale leftover response from a previous session — must be
    //      drained and the request re-sent (checkStaleCounter()).
    //   b) The slave runs its own persistent response-counter sequence
    //      (e.g. Synapticon SOMANET, which keeps counting across master
    //      restarts and even INIT transitions).  The response is valid;
    //      re-sending the request is actively harmful because such slaves
    //      also drop requests whose counter repeats the last one seen.
    //
    // Init-phase SDO responses (and SDO aborts) echo the request's
    // index:sub at a fixed offset.  If the echo matches, the response is
    // ours and we adopt the slave's numbering: expectedCnt = hdr.cnt and
    // *inoutMbxCnt/curCnt = next counter AFTER hdr.cnt (skipping the
    // request counter we just sent so duplicate-detecting slaves do not
    // drop the next request).
    //
    // @return true if the response was adopted — the caller continues
    //         processing it normally; false if it is stale (caller should
    //         fall back to checkStaleCounter()).
    bool adoptCounterOnEcho(Master& master, uint16_t adp,
                            const uint8_t* reqbuf, const uint8_t* rspbuf,
                            uint16_t mbxReadLen, const MbxResponseHeader& hdr,
                            uint8_t* inoutMbxCnt, uint8_t& curCnt,
                            uint8_t& expectedCnt,
                            uint16_t index, uint8_t sub,
                            const char* phaseLabel);

    /// Rate limit for drained-foreign-traffic WARN logs: a slave that
    /// keeps feeding stale/foreign responses would otherwise emit one
    /// WARN per poll iteration (line-rate log spam).  WARNs the first 3
    /// drains of a transaction, then one per 256.
    static bool drainWarnVisible(uint32_t drain_count) {
        return drain_count <= 3 || (drain_count & 0xFFu) == 0;
    }

    // Recover from a mailbox counter desync detected via an explicit
    // counter-mismatch mailbox error (isCounterMismatchError() == true) or
    // a silent drop (request sent, no response).  Probes the 3-bit counter
    // space: re-sends the request with each successive counter value
    // (cycling 1-7) until the slave returns a valid CoE response for this
    // request.
    //
    // On Recovered:
    //   - rspbuf/outHdr hold the accepted response (caller parses normally)
    //   - counters are adopted as in adoptCounterOnEcho()
    //
    // @param reqbuf  The request datagram buffer (mailbox write length
    //                bytes).  Its counter nibble (byte 5) is patched per
    //                probe and left at the accepted counter.
    // @param rspbuf  Response buffer (mbxReadLen bytes).
    // @param verifyIndexEcho  If true, a candidate response must echo the
    //                request's index:sub (init-phase requests).  If false
    //                (segment-phase requests, which carry no index echo),
    //                any CoE response is accepted.
    MbxResyncResult resyncMailboxCounter(
        Master& master, uint16_t adp,
        uint16_t mbxWriteAddr, uint16_t mbxWriteLen,
        uint16_t mbxReadAddr, uint16_t mbxReadLen,
        uint8_t* reqbuf, uint8_t* rspbuf,
        uint8_t* inoutMbxCnt, uint8_t& curCnt, uint8_t& expectedCnt,
        uint16_t index, uint8_t sub,
        bool verifyIndexEcho,
        unsigned int pollIntervalMs,
        const char* phaseLabel,
        MbxResponseHeader& outHdr);

    static constexpr int MAX_STALE_RETRIES = ECAT_SDO_MAX_STALE_RETRIES;
    static constexpr int MAX_POLL_ATTEMPTS = ECAT_SDO_MAX_POLL_ATTEMPTS;

protected:
    SDOErrorDecoder& errorDecoder_;
    SDOMailboxIO& mailboxIO_;
    SDODiagnostics& diagnostics_;

    // Per-instance mailbox write counter used to throttle periodic SDO
    // request logging. Formerly a function-local static shared across all
    // transactions (process-global state); now instance-scoped.
    uint32_t mbx_write_count_ = 0;
};

} // namespace Raw
} // namespace EtherCAT

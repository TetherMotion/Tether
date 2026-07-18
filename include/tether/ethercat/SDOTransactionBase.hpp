#pragma once

#include <cstdint>

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

class SDOTransactionBase {
public:
    virtual ~SDOTransactionBase() = default;

    SDOTransactionBase(SDOErrorDecoder& errorDecoder,
                       SDOMailboxIO& mailboxIO,
                       SDODiagnostics& diagnostics);

    bool sendAndWait(Master& master, uint16_t adp,
                     uint16_t mbxWriteAddr, uint16_t mbxWriteLen,
                     uint16_t mbxReadAddr, uint16_t mbxReadLen,
                     const uint8_t* mbxbuf, unsigned int timeoutMs,
                     unsigned int pollIntervalMs,
                     unsigned int transactionTimeoutMs,
                     const char* phaseLabel);

    MbxPollOutcome pollSm1AndRead(Master& master, uint16_t adp,
                                  uint16_t mbxReadAddr, uint16_t mbxReadLen,
                                  uint8_t* mbxbuf, unsigned int pollIntervalMs,
                                  MbxResponseHeader& hdr);

    bool handleMailboxError(const uint8_t* mbxbuf, const MbxResponseHeader& hdr,
                            uint16_t adp, uint16_t index, uint8_t sub);

    bool checkStaleCounter(Master& master, uint16_t adp,
                           uint16_t mbxWriteAddr, uint16_t mbxWriteLen,
                           uint16_t mbxReadAddr, uint16_t mbxReadLen,
                           const uint8_t* mbxbuf, unsigned int pollIntervalMs,
                           unsigned int transactionTimeoutMs,
                           const MbxResponseHeader& hdr,
                           uint8_t expectedMbxCnt, int& staleRetryCount,
                           uint16_t index_, uint8_t sub_,
                           const char* phaseLabel);

    static constexpr int MAX_STALE_RETRIES = 8;
    static constexpr int MAX_POLL_ATTEMPTS = 50;

protected:
    SDOErrorDecoder& errorDecoder_;
    SDOMailboxIO& mailboxIO_;
    SDODiagnostics& diagnostics_;
};

} // namespace Raw
} // namespace EtherCAT

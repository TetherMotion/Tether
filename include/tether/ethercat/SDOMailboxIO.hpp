#pragma once

#include <cstdint>

namespace EtherCAT { class Master; }

namespace EtherCAT {
namespace Raw {

class SDODiagnostics;

class SDOMailboxIO {
public:
    SDOMailboxIO(SDODiagnostics& diag);
    virtual ~SDOMailboxIO() = default;

    virtual void drainStale(Master& master, uint16_t adp,
                            uint16_t mbxReadAddr, uint16_t mbxReadLen,
                            unsigned int maxDrain = 16);

    virtual bool waitSm0NotFull(Master& master, uint16_t adp,
                                unsigned int timeoutMs,
                                unsigned int pollIntervalMs = 5);

    virtual bool apwrWithWkcProbe(Master& master, uint16_t adp,
                                 uint16_t primaryAddr, uint16_t altAddr,
                                 const uint8_t* payload, uint16_t payloadLen,
                                 unsigned int timeoutMs, bool* outUsedAlt);

    virtual bool pollSm1Full(Master& master, uint16_t adp,
                             unsigned int timeoutMs,
                             unsigned int pollIntervalMs = 5);

    // Sync the master's mailbox counter to match the slave's counter.
    //
    // When the slave persists its mailbox counter across master restarts,
    // the master (which resets to 1) and the slave are permanently out of
    // sync.  Every response carries a cnt that doesn't match the master's
    // expected value, and slaves that validate the incoming counter (e.g.
    // ESC211) reject requests with a "syntax error in mailbox header"
    // (err=0x0001, detail=0x0004).
    //
    // This helper updates the counter embedded in an already-built mailbox
    // buffer (mbxbuf, byte offset 5 = MbxHeader.mbxtype) and the master's
    // persistent counter (*inoutMbxCnt) to the value the slave expects next.
    //
    // @param slaveResponseCnt  The cnt field from the slave's last response
    //                          (stale CoE response or mailbox error).
    // @param inoutMbxCnt       Pointer to the master's persistent counter.
    // @param mbxbuf            The already-built mailbox request buffer.
    static void syncMbxCounter(uint8_t slaveResponseCnt,
                               uint8_t* inoutMbxCnt,
                               uint8_t* mbxbuf);

    // Compute the counter value the slave will use in its NEXT response,
    // given the cnt it just sent.  EtherCAT mailbox counters cycle 1..7.
    static constexpr uint8_t nextMbxCnt(uint8_t cnt) {
        return static_cast<uint8_t>((cnt >= 7) ? 1 : (cnt + 1));
    }

private:
    SDODiagnostics& diag_;
};

} // namespace Raw
} // namespace EtherCAT

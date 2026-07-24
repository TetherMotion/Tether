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

private:
    SDODiagnostics& diag_;
};

} // namespace Raw
} // namespace EtherCAT

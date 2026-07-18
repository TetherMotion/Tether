#pragma once

#include <cstdint>
#include <cstddef>

namespace EtherCAT { class Master; }

namespace EtherCAT {
namespace Raw {

class SDODiagnostics {
public:
    virtual ~SDODiagnostics() = default;

    virtual void dumpSlaveState(Master& master, uint16_t adp,
                                uint16_t mbxWrAddr, uint16_t mbxRdAddr);

    virtual void logCoeMbxPacket(const char* dir, uint16_t adp,
                                 uint16_t index, uint8_t sub,
                                 const uint8_t* data, size_t len, bool enabled);

    virtual void smControlStr(uint8_t ctrlByte, char* buf, size_t bufLen) const;
    virtual void smStatusStr(uint8_t statByte, char* buf, size_t bufLen) const;
    virtual void smActivateStr(uint8_t actByte, char* buf, size_t bufLen) const;

    virtual bool isPdoMappingIndex(uint16_t idx) const;

    using UploadFn = bool(*)(Master&, uint16_t, uint8_t*,
                             uint16_t, uint16_t, uint16_t, uint16_t,
                             uint16_t, uint8_t,
                             uint8_t*, size_t, size_t*,
                             bool, unsigned int, unsigned int);

    virtual void logPdoMappingSubindexDiagnostic(Master& master, uint16_t adp,
                                                 uint8_t* inoutMbxCnt,
                                                 uint16_t mbxWriteAddr, uint16_t mbxWriteLen,
                                                 uint16_t mbxReadAddr, uint16_t mbxReadLen,
                                                 uint16_t index, uint8_t sub,
                                                 bool diagEnabled,
                                                 unsigned int pollIntervalMs,
                                                 unsigned int transactionTimeoutMs,
                                                 UploadFn uploadFn);

#ifdef TETHER_DIAG_SDO_IO
    virtual void diagHexdump(const uint8_t* data, size_t len, size_t maxPrint = 64) const;
#endif
};

} // namespace Raw
} // namespace EtherCAT

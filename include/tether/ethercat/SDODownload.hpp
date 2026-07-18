#pragma once

#include <cstdint>
#include <cstddef>
#include "tether/ethercat/SDOTransactionBase.hpp"

namespace EtherCAT {
namespace Raw {

class SDODownload : public SDOTransactionBase {
public:
    SDODownload(SDOErrorDecoder& ed, SDOMailboxIO& mio, SDODiagnostics& diag)
        : SDOTransactionBase(ed, mio, diag) {}

    virtual bool execute(Master& master, uint16_t adp,
                         uint8_t* inoutMbxCnt,
                         uint16_t mbxWriteAddr, uint16_t mbxWriteLen,
                         uint16_t mbxReadAddr, uint16_t mbxReadLen,
                         uint16_t index, uint8_t sub,
                         const uint8_t* data, size_t dataLen,
                         bool diagEnabled = false,
                         unsigned int pollIntervalMs = 5,
                         unsigned int transactionTimeoutMs = 1000);

protected:
    virtual bool executeExpedited(Master& master, uint16_t adp,
                                  uint8_t* inoutMbxCnt,
                                  uint16_t mbxWriteAddr, uint16_t mbxWriteLen,
                                  uint16_t mbxReadAddr, uint16_t mbxReadLen,
                                  uint16_t index, uint8_t sub,
                                  const uint8_t* data, size_t dataLen,
                                  bool diagEnabled,
                                  unsigned int pollIntervalMs,
                                  unsigned int transactionTimeoutMs);

    virtual bool executeNormal(Master& master, uint16_t adp,
                               uint8_t* inoutMbxCnt,
                               uint16_t mbxWriteAddr, uint16_t mbxWriteLen,
                               uint16_t mbxReadAddr, uint16_t mbxReadLen,
                               uint16_t index, uint8_t sub,
                               const uint8_t* data, size_t dataLen,
                               bool diagEnabled,
                               unsigned int pollIntervalMs,
                               unsigned int transactionTimeoutMs);

    virtual bool executeSegmented(Master& master, uint16_t adp,
                                  uint8_t* inoutMbxCnt,
                                  uint16_t mbxWriteAddr, uint16_t mbxWriteLen,
                                  uint16_t mbxReadAddr, uint16_t mbxReadLen,
                                  uint16_t index, uint8_t sub,
                                  const uint8_t* data, size_t dataLen,
                                  bool diagEnabled,
                                  unsigned int pollIntervalMs,
                                  unsigned int transactionTimeoutMs);
};

} // namespace Raw
} // namespace EtherCAT

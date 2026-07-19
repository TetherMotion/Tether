#pragma once

#include <cstdint>
#include <cstddef>
#include "tether/ethercat/SDOTransactionBase.hpp"

namespace EtherCAT {
namespace Raw {

class SDOUpload : public SDOTransactionBase {
public:
    SDOUpload(SDOErrorDecoder& ed, SDOMailboxIO& mio, SDODiagnostics& diag)
        : SDOTransactionBase(ed, mio, diag) {}

    /// @param outAbortCode  If non-null, set to the CoE SDO abort code (0 on
    ///                      success or non-abort failure) reported by the slave.
    virtual bool execute(Master& master, uint16_t adp,
                         uint8_t* inoutMbxCnt,
                         uint16_t mbxWriteAddr, uint16_t mbxWriteLen,
                         uint16_t mbxReadAddr, uint16_t mbxReadLen,
                         uint16_t index, uint8_t sub,
                         uint8_t* out, size_t outCap, size_t* outLen,
                         bool diagEnabled = false,
                         unsigned int pollIntervalMs = 5,
                         unsigned int transactionTimeoutMs = 1000,
                         uint32_t* outAbortCode = nullptr);
};

} // namespace Raw
} // namespace EtherCAT

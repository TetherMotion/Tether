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

    /**
     * @brief Perform a CoE SDO upload (slave → master read).
     * @param master      Master to issue the mailbox transaction on.
     * @param adp         Auto-increment/slave address of the target.
     * @param inoutMbxCnt Mailbox toggle counter; updated in place.
     * @param mbxWriteAddr  Slave mailbox write (master→slave) address.
     * @param mbxWriteLen   Slave mailbox write area length in bytes.
     * @param mbxReadAddr   Slave mailbox read (slave→master) address.
     * @param mbxReadLen    Slave mailbox read area length in bytes.
     * @param index       CoE object index to read.
     * @param sub         CoE object subindex.
     * @param out         Destination buffer for the object data.
     * @param outCap      Capacity of `out` in bytes.
     * @param outLen      If non-null, set to the number of bytes written.
     * @param diagEnabled Enable diagnostic logging of mailbox traffic.
     * @param pollIntervalMs      Milliseconds between mailbox polls.
     * @param transactionTimeoutMs  Overall transaction timeout (ms).
     * @param outAbortCode  If non-null, set to the CoE SDO abort code (0 on
     *                      success or non-abort failure) reported by the slave.
     * @return true when the upload completed; false on abort/timeout.
     */
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

#pragma once

#include <cstdint>
#include <cstddef>
#include "tether/ethercat/SDOTransactionBase.hpp"

namespace EtherCAT {
namespace Raw {

class SDOUpload; // forward declaration (used for PDO-mapping diagnostics)

class SDODownload : public SDOTransactionBase {
public:
    SDODownload(SDOErrorDecoder& ed, SDOMailboxIO& mio, SDODiagnostics& diag)
        : SDOTransactionBase(ed, mio, diag) {}

    /**
     * @brief Perform a CoE SDO download (master → slave write).
     * @param master      Master to issue the mailbox transaction on.
     * @param adp         Auto-increment/slave address of the target.
     * @param inoutMbxCnt Mailbox toggle counter; updated in place.
     * @param mbxWriteAddr  Slave mailbox write (master→slave) address.
     * @param mbxWriteLen   Slave mailbox write area length in bytes.
     * @param mbxReadAddr   Slave mailbox read (slave→master) address.
     * @param mbxReadLen    Slave mailbox read area length in bytes.
     * @param index       CoE object index to write.
     * @param sub         CoE object subindex.
     * @param data        Object data to download.
     * @param dataLen     Length of `data` in bytes.
     * @param diagEnabled Enable diagnostic logging of mailbox traffic.
     * @param pollIntervalMs      Milliseconds between mailbox polls.
     * @param transactionTimeoutMs  Overall transaction timeout (ms).
     * @param outAbortCode  If non-null, set to the CoE SDO abort code (0 on
     *                      success or non-abort failure) reported by the slave.
     *                      Callers can use this to distinguish a definitive
     *                      slave rejection (e.g. 0x06070010 length mismatch)
     *                      from a transport/timeout failure.
     * @param uploadForDiag  Optional SDOUpload instance used to perform a
     *                       follow-up read when a PDO-mapping subindex abort
     *                       is observed, so a richer diagnostic can be logged.
     *                       May be nullptr to skip the diagnostic read.
     * @return true when the download completed; false on abort/timeout.
     */
    virtual bool execute(Master& master, uint16_t adp,
                         uint8_t* inoutMbxCnt,
                         uint16_t mbxWriteAddr, uint16_t mbxWriteLen,
                         uint16_t mbxReadAddr, uint16_t mbxReadLen,
                         uint16_t index, uint8_t sub,
                         const uint8_t* data, size_t dataLen,
                         bool diagEnabled = false,
                         unsigned int pollIntervalMs = 5,
                         unsigned int transactionTimeoutMs = 1000,
                         uint32_t* outAbortCode = nullptr,
                         SDOUpload* uploadForDiag = nullptr);

protected:
    virtual bool executeExpedited(Master& master, uint16_t adp,
                                  uint8_t* inoutMbxCnt,
                                  uint16_t mbxWriteAddr, uint16_t mbxWriteLen,
                                  uint16_t mbxReadAddr, uint16_t mbxReadLen,
                                  uint16_t index, uint8_t sub,
                                  const uint8_t* data, size_t dataLen,
                                  bool diagEnabled,
                                  unsigned int pollIntervalMs,
                                  unsigned int transactionTimeoutMs,
                                  uint32_t* outAbortCode,
                                  SDOUpload* uploadForDiag);

    virtual bool executeNormal(Master& master, uint16_t adp,
                               uint8_t* inoutMbxCnt,
                               uint16_t mbxWriteAddr, uint16_t mbxWriteLen,
                               uint16_t mbxReadAddr, uint16_t mbxReadLen,
                               uint16_t index, uint8_t sub,
                               const uint8_t* data, size_t dataLen,
                               bool diagEnabled,
                               unsigned int pollIntervalMs,
                               unsigned int transactionTimeoutMs,
                               uint32_t* outAbortCode);

    virtual bool executeSegmented(Master& master, uint16_t adp,
                                  uint8_t* inoutMbxCnt,
                                  uint16_t mbxWriteAddr, uint16_t mbxWriteLen,
                                  uint16_t mbxReadAddr, uint16_t mbxReadLen,
                                  uint16_t index, uint8_t sub,
                                  const uint8_t* data, size_t dataLen,
                                  bool diagEnabled,
                                  unsigned int pollIntervalMs,
                                  unsigned int transactionTimeoutMs,
                                  uint32_t* outAbortCode,
                                  SDOUpload* uploadForDiag);
};

} // namespace Raw
} // namespace EtherCAT

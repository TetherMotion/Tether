#pragma once

#include <atomic>
#include <cstdint>
#include <cstddef>
#include "tether/ethercat/SDOErrorDecoder.hpp"
#include "tether/ethercat/SDOMailboxIO.hpp"
#include "tether/ethercat/SDODiagnostics.hpp"
#include "tether/ethercat/SDOUpload.hpp"
#include "tether/ethercat/SDODownload.hpp"

namespace EtherCAT { class Master; }

namespace EtherCAT {
namespace Raw {

class CoeSDOChannel {
public:
    CoeSDOChannel();

    virtual bool upload(Master& master, uint16_t adp, uint8_t* inoutMbxCnt,
                        uint16_t mbxWriteAddr, uint16_t mbxWriteLen,
                        uint16_t mbxReadAddr, uint16_t mbxReadLen,
                        uint16_t index, uint8_t sub,
                        uint8_t* out, size_t outCap, size_t* outLen,
                        bool diagEnabled = false,
                        unsigned int pollIntervalMs = 5,
                        unsigned int transactionTimeoutMs = 1000);

    virtual bool download(Master& master, uint16_t adp, uint8_t* inoutMbxCnt,
                          uint16_t mbxWriteAddr, uint16_t mbxWriteLen,
                          uint16_t mbxReadAddr, uint16_t mbxReadLen,
                          uint16_t index, uint8_t sub,
                          const uint8_t* data, size_t dataLen,
                          bool diagEnabled = false,
                          unsigned int pollIntervalMs = 5,
                          unsigned int transactionTimeoutMs = 1000);

    SDOErrorDecoder& errorDecoder() { return errorDecoder_; }
    SDOMailboxIO& mailboxIO() { return mailboxIO_; }
    SDODiagnostics& diagnostics() { return diagnostics_; }
    SDOUpload& uploadOp() { return upload_; }
    SDODownload& downloadOp() { return download_; }

    /// @brief Return the CoE SDO abort code reported by the slave on the most
    /// recent upload/download call. 0 means no abort (success or non-abort
    /// failure such as a transport/timeout error). Safe to read immediately
    /// after a call returns false; SDO operations on a master are serialized.
    uint32_t lastAbortCode() const { return last_abort_code_.load(std::memory_order_relaxed); }

private:
    SDOErrorDecoder errorDecoder_;
    SDODiagnostics diagnostics_;
    SDOMailboxIO mailboxIO_;
    SDOUpload upload_;
    SDODownload download_;
    std::atomic<uint32_t> last_abort_code_{0};
};

} // namespace Raw
} // namespace EtherCAT

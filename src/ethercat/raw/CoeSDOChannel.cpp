#include "tether/ethercat/CoeSDOChannel.hpp"
#include "tether/ethercat/Master.hpp"

namespace EtherCAT {
namespace Raw {

// Forward declaration of the global thunk pointer defined in SDODownload.cpp
extern SDOUpload* g_sdo_upload_for_thunk;

CoeSDOChannel::CoeSDOChannel()
    : errorDecoder_()
    , diagnostics_()
    , mailboxIO_(diagnostics_)
    , upload_(errorDecoder_, mailboxIO_, diagnostics_)
    , download_(errorDecoder_, mailboxIO_, diagnostics_) {}

bool CoeSDOChannel::upload(Master& master, uint16_t adp, uint8_t* inoutMbxCnt,
                           uint16_t mbxWriteAddr, uint16_t mbxWriteLen,
                           uint16_t mbxReadAddr, uint16_t mbxReadLen,
                           uint16_t index, uint8_t sub,
                           uint8_t* out, size_t outCap, size_t* outLen,
                           bool diagEnabled,
                           unsigned int pollIntervalMs,
                           unsigned int transactionTimeoutMs) {
    return upload_.execute(master, adp, inoutMbxCnt,
                           mbxWriteAddr, mbxWriteLen,
                           mbxReadAddr, mbxReadLen,
                           index, sub, out, outCap, outLen,
                           diagEnabled, pollIntervalMs, transactionTimeoutMs);
}

bool CoeSDOChannel::download(Master& master, uint16_t adp, uint8_t* inoutMbxCnt,
                             uint16_t mbxWriteAddr, uint16_t mbxWriteLen,
                             uint16_t mbxReadAddr, uint16_t mbxReadLen,
                             uint16_t index, uint8_t sub,
                             const uint8_t* data, size_t dataLen,
                             bool diagEnabled,
                             unsigned int pollIntervalMs,
                             unsigned int transactionTimeoutMs) {
    // Set the global thunk pointer so SDODownload can call SDOUpload for
    // PDO mapping diagnostics. This is safe because SDO operations are
    // single-threaded on the EtherCAT master.
    g_sdo_upload_for_thunk = &upload_;
    return download_.execute(master, adp, inoutMbxCnt,
                             mbxWriteAddr, mbxWriteLen,
                             mbxReadAddr, mbxReadLen,
                             index, sub, data, dataLen,
                             diagEnabled, pollIntervalMs, transactionTimeoutMs);
}

} // namespace Raw
} // namespace EtherCAT

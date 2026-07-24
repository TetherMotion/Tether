#include "tether/ethercat/CoeSDOChannel.hpp"
#include "tether/ethercat/Master.hpp"

namespace EtherCAT {
namespace Raw {

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
    last_abort_code_.store(0, std::memory_order_relaxed);
    uint32_t abort_code = 0;
    bool ok = upload_.execute(master, adp, inoutMbxCnt,
                              mbxWriteAddr, mbxWriteLen,
                              mbxReadAddr, mbxReadLen,
                              index, sub, out, outCap, outLen,
                              diagEnabled, pollIntervalMs, transactionTimeoutMs,
                              &abort_code);
    if (!ok && abort_code != 0) {
        last_abort_code_.store(abort_code, std::memory_order_relaxed);
    }
    return ok;
}

bool CoeSDOChannel::download(Master& master, uint16_t adp, uint8_t* inoutMbxCnt,
                             uint16_t mbxWriteAddr, uint16_t mbxWriteLen,
                             uint16_t mbxReadAddr, uint16_t mbxReadLen,
                             uint16_t index, uint8_t sub,
                             const uint8_t* data, size_t dataLen,
                             bool diagEnabled,
                             unsigned int pollIntervalMs,
                             unsigned int transactionTimeoutMs) {
    last_abort_code_.store(0, std::memory_order_relaxed);
    uint32_t abort_code = 0;
    bool ok = download_.execute(master, adp, inoutMbxCnt,
                                mbxWriteAddr, mbxWriteLen,
                                mbxReadAddr, mbxReadLen,
                                index, sub, data, dataLen,
                                diagEnabled, pollIntervalMs, transactionTimeoutMs,
                                &abort_code, &upload_);
    if (!ok && abort_code != 0) {
        last_abort_code_.store(abort_code, std::memory_order_relaxed);
    }
    return ok;
}

} // namespace Raw
} // namespace EtherCAT

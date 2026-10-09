#include "tether/ethercat/SDODownload.hpp"
#include "tether/ethercat/SDOErrorDecoder.hpp"
#include "tether/ethercat/SDOMailboxIO.hpp"
#include "tether/ethercat/SDODiagnostics.hpp"
#include "tether/ethercat/SDOUpload.hpp"
#include "tether/ethercat/Master.hpp"
#include "tether/ethercat/DebugFlags.hpp"
#include "tether/platform/Platform.hpp"
#include "raw/internal.hpp"
#include "raw/SDODownloadInternal.hpp"
#include <cstring>
#include <cstdio>
#include <chrono>
#include <thread>
#include <cinttypes>
#include <utility>
#include <vector>
#include <algorithm>

namespace EtherCAT {
namespace Raw {

static const char* TAG = "ethercat";

bool SDODownload::execute(Master& master, uint16_t adp,
                          uint8_t* inoutMbxCnt,
                          uint16_t mbxWriteAddr, uint16_t mbxWriteLen,
                          uint16_t mbxReadAddr, uint16_t mbxReadLen,
                          uint16_t index, uint8_t sub,
                          const uint8_t* data, size_t dataLen,
                          bool diagEnabled,
                          unsigned int pollIntervalMs,
                          unsigned int transactionTimeoutMs,
                          uint32_t* outAbortCode,
                          SDOUpload* uploadForDiag) {
    if (outAbortCode) *outAbortCode = 0;
    if (master.isCancelRequested()) {
        return false;
    }
    if (data == nullptr || dataLen == 0) {
        TETHER_LOGE(TAG, "Invalid SDO download parameters (len={})", static_cast<unsigned>(dataLen));
        return false;
    }

    const size_t sdo_header_size = sizeof(MbxHeader) + sizeof(CoeHeader) + sizeof(SdoInitDownloadReq);
    const size_t max_inline = (mbxWriteLen > sdo_header_size)
        ? (mbxWriteLen - sdo_header_size) : 0;

    if (dataLen > 4 && dataLen <= max_inline) {
        return executeNormal(master, adp, inoutMbxCnt,
                             mbxWriteAddr, mbxWriteLen,
                             mbxReadAddr, mbxReadLen,
                             index, sub, data, dataLen,
                             diagEnabled, pollIntervalMs, transactionTimeoutMs,
                             outAbortCode);
    }

    if (dataLen > max_inline) {
        return executeSegmented(master, adp, inoutMbxCnt,
                                mbxWriteAddr, mbxWriteLen,
                                mbxReadAddr, mbxReadLen,
                                index, sub, data, dataLen,
                                diagEnabled, pollIntervalMs, transactionTimeoutMs,
                                outAbortCode, uploadForDiag);
    }

    return executeExpedited(master, adp, inoutMbxCnt,
                            mbxWriteAddr, mbxWriteLen,
                            mbxReadAddr, mbxReadLen,
                            index, sub, data, dataLen,
                            diagEnabled, pollIntervalMs, transactionTimeoutMs,
                            outAbortCode, uploadForDiag);
}


} // namespace Raw
} // namespace EtherCAT

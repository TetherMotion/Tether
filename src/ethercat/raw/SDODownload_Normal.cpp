/**
 * @file SDODownload_Normal.cpp
 * @brief SDODownload — normal (<= mailbox) transfer.
 *
 * TU split out of SDODownload.cpp.
 */

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

bool SDODownload::executeNormal(Master& master, uint16_t adp,
                                uint8_t* inoutMbxCnt,
                                uint16_t mbxWriteAddr, uint16_t mbxWriteLen,
                                uint16_t mbxReadAddr, uint16_t mbxReadLen,
                                uint16_t index, uint8_t sub,
                                const uint8_t* data, size_t dataLen,
                                bool diagEnabled,
                                unsigned int pollIntervalMs,
                                unsigned int transactionTimeoutMs,
                                uint32_t* outAbortCode) {
    const size_t buf_size = std::max(static_cast<size_t>(mbxWriteLen),
                                      static_cast<size_t>(mbxReadLen));
    if (buf_size > kRawSDOMbxBufferSize) {
        TETHER_LOGE(TAG,
            "Slave mailbox size exceeds Tether safety ceiling "
            "(wr={} rd={}, needed={}, ceiling={} bytes). "
            "Increase ECAT_RAW_SDO_MBX_BUFFER_SIZE in EtherCATConfig.hpp to >= {}.",
            mbxWriteLen, mbxReadLen, buf_size,
            static_cast<unsigned>(kRawSDOMbxBufferSize), buf_size);
        return false;
    }
    // mbxbuf holds the REQUEST (re-sent intact on stale responses);
    // rspbuf holds the RESPONSE.
    std::vector<uint8_t> mbxbuf_storage(buf_size, 0);
    std::vector<uint8_t> rspbuf_storage(buf_size, 0);
    uint8_t* mbxbuf = mbxbuf_storage.data();
    uint8_t* rspbuf = rspbuf_storage.data();

    const size_t sdo_header_size = sizeof(MbxHeader) + sizeof(CoeHeader) + sizeof(SdoInitDownloadReq);
    const size_t msg_len = sdo_header_size + dataLen;
    if (msg_len > mbxWriteLen) {
        TETHER_LOGE(TAG, "Normal download: data_len={} exceeds slave mailbox write capacity "
                    "(msg={} > slave mailbox wr={}). The slave mailbox is too small for this data; "
                    "use segmented download or increase the slave's mailbox size.",
                    dataLen, msg_len, mbxWriteLen);
        return false;
    }

    // Translate internal CA signal (bit 7 in subindex) to the ETG.1000.6
    // Complete-Access bit (0x10) in the SDO command byte.  See SDOUpload.cpp
    // for the full rationale.
    const bool complete_access = (sub & 0x80u) != 0;
    sub = static_cast<uint8_t>(sub & 0x7Fu);

    uint8_t mbx_cnt = 0;
    if (inoutMbxCnt != nullptr) {
        mbx_cnt = *inoutMbxCnt;
    }
    uint8_t expected_mbx_cnt = mbx_cnt;

    MbxHeader mbx{};
    mbx.length_le = host_to_le16(static_cast<uint16_t>(sizeof(CoeHeader) + sizeof(SdoInitDownloadReq) + dataLen));
    mbx.address_le = host_to_le16(0);
    mbx.priority = 0;
    mbx.mbxtype = mbx_type_with_cnt(EC_MBXT_COE, mbx_cnt);

    CoeHeader coe{};
    coe.raw_le = host_to_le16(coe_make_raw(0, EC_COES_SDOREQ));

    SdoInitDownloadReq sdo{};
    sdo.cmd = static_cast<uint8_t>(EC_SDO_DOWN_REQ | 0x01u | (complete_access ? 0x10u : 0x00u));
    sdo.index_le = host_to_le16(index);
    sdo.sub = sub;
    sdo.data_le = host_to_le32(static_cast<uint32_t>(dataLen));

    std::memcpy(mbxbuf, &mbx, sizeof(mbx));
    std::memcpy(mbxbuf + sizeof(mbx), &coe, sizeof(coe));
    std::memcpy(mbxbuf + sizeof(mbx) + sizeof(coe), &sdo, sizeof(sdo));
    std::memcpy(mbxbuf + sdo_header_size, data, dataLen);

    diagnostics_.logCoeMbxPacket("TX", adp, index, sub, mbxbuf, msg_len,
                                 master.debugFlags().coeTxPackets && master.debugFlags().coeTxPacketsFilt.allows(slaveIndexFromADP(adp)));

    mbx_cnt = static_cast<uint8_t>((mbx_cnt >= 7) ? 1 : (mbx_cnt + 1));
    if (inoutMbxCnt != nullptr) {
        *inoutMbxCnt = mbx_cnt;
    }

    mailboxIO_.drainStale(master, adp, mbxReadAddr, mbxReadLen);

    bool used_alt = false;
    if (!mailboxIO_.apwrWithWkcProbe(master, adp,
                                     mbxWriteAddr, mbxReadAddr,
                                     mbxbuf, static_cast<uint16_t>(mbxWriteLen),
                                     500, &used_alt)) {
        if (!master.isCancelRequested()) {
            TETHER_LOGE(TAG, "{}: SDO normal download: Mailbox write failed (wr=0x{:04X} rd=0x{:04X})",
                        master.slaveLogPrefix(slaveIndexFromADP(adp)).c_str(), mbxWriteAddr, mbxReadAddr);
        }
        return false;
    }
    if (used_alt) {
        std::swap(mbxWriteAddr, mbxReadAddr);
        std::swap(mbxWriteLen, mbxReadLen);
    }

    if (!mailboxIO_.pollSm1Full(master, adp, transactionTimeoutMs, pollIntervalMs)) {
        if (!master.isCancelRequested()) {
            TETHER_LOGE(TAG, "{}: SDO normal download: SM1 never became full (index=0x{:04X}:{} timeout={}ms)",
                        master.slaveLogPrefix(slaveIndexFromADP(adp)).c_str(), index, sub, transactionTimeoutMs);
            diagnostics_.dumpSlaveState(master, adp, mbxWriteAddr, mbxReadAddr);
        }
        return false;
    }

    int stale_retry_count = 0;

    // Slaves like the ESC211 drain SM0 into an internal FIFO and answer each
    // request ~25-30 ms late; responses queued ahead of ours must be drained,
    // never re-sent.  Keep polling while the mailbox yields data and give up
    // after transactionTimeoutMs of silence (hard cap guards a pathological
    // slave).
    const auto quiet_timeout = std::chrono::milliseconds(transactionTimeoutMs);
    auto quiet_deadline = std::chrono::steady_clock::now() + quiet_timeout;
    const auto hard_deadline = quiet_deadline + std::chrono::milliseconds(60000);
    for (int attempt = 0; attempt < MAX_POLL_ATTEMPTS * 100; attempt++) {
        const auto poll_now = std::chrono::steady_clock::now();
        if (poll_now > quiet_deadline || poll_now > hard_deadline) {
            break;
        }
        MbxResponseHeader hdr;
        auto outcome = pollSm1AndRead(master, adp, mbxReadAddr, mbxReadLen,
                                      rspbuf, pollIntervalMs, hdr);
        if (outcome == MbxPollOutcome::GotData) {
            quiet_deadline = std::chrono::steady_clock::now() + quiet_timeout;
        }

        if (outcome == MbxPollOutcome::Cancelled) {
            TETHER_LOGW(TAG, "SDO normal download cancelled");
            return false;
        }
        if (outcome == MbxPollOutcome::Sm1Empty || outcome == MbxPollOutcome::ReadFailed) {
            if (outcome == MbxPollOutcome::ReadFailed) {
                TETHER_LOGW(TAG, "SDO normal download: mailbox data read WKC=0 despite SM1 full (adp=0x{:04X})", adp);
            }
            continue;
        }

        if (hdr.type == EC_MBXT_ERR) {
            if (master.mailboxCounterResyncEnabled() &&
                isCounterMismatchError(rspbuf, hdr)) {
                const auto resync = resyncMailboxCounter(
                    master, adp, mbxWriteAddr, mbxWriteLen,
                    mbxReadAddr, mbxReadLen, mbxbuf, rspbuf,
                    inoutMbxCnt, mbx_cnt, expected_mbx_cnt,
                    index, sub, true, pollIntervalMs,
                    "normal download", hdr);
                if (resync == MbxResyncResult::Cancelled) {
                    return false;
                }
                if (resync == MbxResyncResult::Failed) {
                    handleMailboxError(rspbuf, hdr, adp, index, sub);
                    return false;
                }
                // Recovered — rspbuf/hdr hold the accepted response.
            } else {
                handleMailboxError(rspbuf, hdr, adp, index, sub);
                return false;
            }
        }
        if (hdr.type != EC_MBXT_COE) {
            TETHER_LOGW(TAG, "Non-CoE mailbox response (normal download): type={} cnt={} (adp=0x{:04X} index=0x{:04X}:{}) — draining",
                        hdr.type, hdr.cnt, adp, index, sub);
            continue;  // foreign mailbox traffic — drain, keep polling
        }
        if (hdr.cnt != expected_mbx_cnt) {
            if (!adoptCounterOnEcho(master, adp, mbxbuf, rspbuf, mbxReadLen, hdr,
                                  inoutMbxCnt, mbx_cnt, expected_mbx_cnt,
                                  index, sub, "normal download") &&
                !checkStaleCounter(master, adp, mbxWriteAddr, mbxWriteLen,
                                   mbxReadAddr, mbxReadLen, mbxbuf,
                                   pollIntervalMs, transactionTimeoutMs, hdr,
                                   inoutMbxCnt, expected_mbx_cnt,
                                   stale_retry_count, index, sub, "normal download")) {
                return false;
            }
            if (hdr.cnt != expected_mbx_cnt) {
                continue;
            }
        }
        if (hdr.len < sizeof(CoeHeader) + 1) {
            std::this_thread::sleep_for(std::chrono::milliseconds(pollIntervalMs));
            continue;
        }

        const size_t sdo_offset = sizeof(MbxHeader) + sizeof(CoeHeader);
        const uint8_t sdo_cmd = rspbuf[sdo_offset];

        // Check for SDO abort before the CoE service field — some slaves
        // (e.g. ESC211) emit abort responses with a buggy CoE service field
        // (0x2/SDO-REQ instead of 0x3/SDO-RES). Surface the abort code
        // regardless of the CoE service so callers see the real rejection
        // instead of a misleading timeout.
        if ((sdo_cmd & 0xE0u) == EC_SDO_ABORT) {
            if (mbxReadLen >= sdo_offset + sizeof(SdoAbort)) {
                SdoAbort abort{};
                std::memcpy(&abort, rspbuf + sdo_offset, sizeof(abort));
                const uint32_t abort_code = le32_to_host(abort.abortCode_le);
                const uint16_t ab_index = le16_to_host(abort.index_le);
                if (ab_index != index || abort.sub != sub) {
                    // Stale abort answering an earlier queued request —
                    // drain it and keep waiting for ours.
                    ++stale_retry_count;
                    TETHER_LOGD(TAG, "Stale SDO abort (normal download): idx=0x{:04x}:{} expected=0x{:04x}:{} — draining",
                             ab_index, abort.sub, index, sub);
                    continue;
                }
                TETHER_LOGD(TAG, "SDO normal download abort: index=0x{:04x}:{:02x} code=0x{:08x} ({})",
                         index, sub, abort_code, errorDecoder_.sdoAbortCodeStr(abort_code));
                if (outAbortCode) *outAbortCode = abort_code;
            } else {
                TETHER_LOGE(TAG, "SDO normal download abort (malformed response)");
            }
            return false;
        }

        CoeHeader resp_coe{};
        std::memcpy(&resp_coe, rspbuf + sizeof(MbxHeader), sizeof(resp_coe));
        const uint8_t resp_service = (le16_to_host(resp_coe.raw_le) >> 12) & 0x0Fu;
        if (resp_service != EC_COES_SDORES) {
            TETHER_LOGW(TAG, "Unexpected CoE service (normal download): 0x{:X} (expected 0x3) (adp=0x{:04X} index=0x{:04X}:{}) — draining",
                        resp_service, adp, index, sub);
            continue;  // foreign CoE service — drain, keep polling
        }

        if ((sdo_cmd & 0xE0u) == 0x60u) {
            if (mbxReadLen >= sdo_offset + sizeof(SdoInitDownloadRes)) {
                SdoInitDownloadRes res{};
                std::memcpy(&res, rspbuf + sdo_offset, sizeof(res));
                const uint16_t res_index = le16_to_host(res.index_le);
                if (res_index == index && res.sub == sub) {
                    diagnostics_.logCoeMbxPacket("RX", adp, index, sub, rspbuf, mbxReadLen,
                            master.debugFlags().coeRxPackets && master.debugFlags().coeRxPacketsFilt.allows(slaveIndexFromADP(adp)));
                    return true;
                }
                ++stale_retry_count;
                if (stale_retry_count <= 4 || (stale_retry_count % 32) == 0) {
                    TETHER_LOGW(TAG, "Stale SDO normal download response: idx=0x{:04X}:{} expected=0x{:04X}:{} (adp=0x{:04X}) — draining (#{}), NOT re-sending",
                                res_index, res.sub, index, sub, adp, stale_retry_count);
                }
                // Stale answer to an earlier queued request — drain and keep
                // waiting for ours; re-sending only grows the slave's queue.
                std::this_thread::sleep_for(std::chrono::milliseconds(pollIntervalMs));
                continue;
            }
        }

        // Foreign SDO response — drain and keep waiting for ours.
        TETHER_LOGW(TAG, "Unexpected SDO command (normal download): cmd=0x{:02X} (adp=0x{:04X} index=0x{:04X}:{}) — draining",
                    sdo_cmd, adp, index, sub);
        std::this_thread::sleep_for(std::chrono::milliseconds(pollIntervalMs));
        continue;
    }

    TETHER_LOGE(TAG, "{}: SDO normal download timeout: index=0x{:04x}:{:02x} (wr=0x{:04X} rd=0x{:04X})",
                master.slaveLogPrefix(slaveIndexFromADP(adp)).c_str(), index, sub, mbxWriteAddr, mbxReadAddr);
    diagnostics_.dumpSlaveState(master, adp, mbxWriteAddr, mbxReadAddr);
    return false;
}

} // namespace Raw
} // namespace EtherCAT


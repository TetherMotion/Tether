/**
 * @file SDODownload_Expedited.cpp
 * @brief SDODownload — expedited (<=4 byte) transfer.
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

bool SDODownload::executeExpedited(Master& master, uint16_t adp,
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
    // mbxbuf holds the REQUEST (so it can be re-sent intact); rspbuf holds
    // the RESPONSE — keeping them separate prevents a stale-response re-send
    // from transmitting the previously received response as a request.
    std::vector<uint8_t> mbxbuf_storage(buf_size, 0);
    std::vector<uint8_t> rspbuf_storage(buf_size, 0);
    uint8_t* mbxbuf = mbxbuf_storage.data();
    uint8_t* rspbuf = rspbuf_storage.data();

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
    mbx.length_le = host_to_le16(static_cast<uint16_t>(sizeof(CoeHeader) + sizeof(SdoInitDownloadReq)));
    mbx.address_le = host_to_le16(0);
    mbx.priority = 0;
    mbx.mbxtype = mbx_type_with_cnt(EC_MBXT_COE, mbx_cnt);

    CoeHeader coe{};
    coe.raw_le = host_to_le16(coe_make_raw(0, EC_COES_SDOREQ));

    const uint8_t n = static_cast<uint8_t>((4u - dataLen) & 0x03u);
    SdoInitDownloadReq sdo{};
    sdo.cmd = static_cast<uint8_t>(EC_SDO_DOWN_REQ | 0x02u | 0x01u | (n << 2) | (complete_access ? 0x10u : 0x00u));
    sdo.index_le = host_to_le16(index);
    sdo.sub = sub;

    uint32_t data_u32 = 0;
    std::memcpy(&data_u32, data, dataLen);
    sdo.data_le = host_to_le32(data_u32);

    const size_t msg_len = sizeof(mbx) + sizeof(coe) + sizeof(sdo);
    if (msg_len > mbxWriteLen) {
        TETHER_LOGE(TAG, "Mailbox write len too small ({} < {})", mbxWriteLen, static_cast<unsigned>(msg_len));
        return false;
    }

    std::memcpy(mbxbuf, &mbx, sizeof(mbx));
    std::memcpy(mbxbuf + sizeof(mbx), &coe, sizeof(coe));
    std::memcpy(mbxbuf + sizeof(mbx) + sizeof(coe), &sdo, sizeof(sdo));

    diagnostics_.logCoeMbxPacket("TX", adp, index, sub, mbxbuf, msg_len,
                                 master.debugFlags().coeTxPackets && master.debugFlags().coeTxPacketsFilt.allows(slaveIndexFromADP(adp)));

    mbx_cnt = static_cast<uint8_t>((mbx_cnt >= 7) ? 1 : (mbx_cnt + 1));
    if (inoutMbxCnt != nullptr) {
        *inoutMbxCnt = mbx_cnt;
    }

    {
        uint8_t sm0_status = 0;
        (void)master.readRegister(Master::slaveAddressFromADP(adp), 0x0805, sm0_status, 100);
        uint16_t al_status = 0;
        (void)master.readRegister(Master::slaveAddressFromADP(adp), 0x0130, al_status, 100);

        mbx_write_count_++;
        if ((mbx_write_count_ % 1000) == 1) {
            TETHER_LOGI(TAG, "{}: SDO download (write) request: index=0x{:04X}:{} [mailbox #{} -> 0x{:04X}, len={}, SM0=0x{:02X}, AL=0x{:04X}]",
                     master.slaveLogPrefix(slaveIndexFromADP(adp)).c_str(), index, sub, (unsigned long)mbx_write_count_, mbxWriteAddr, mbxWriteLen, sm0_status, al_status);
        }

#ifdef TETHER_DIAG_SDO_IO
        if (diagEnabled) {
            TETHER_LOGI(TAG, "SDO Download INIT: adp=0x{:04x} index=0x{:04x} sub={} mbx_wr=0x{:04x}/{} mbx_rd=0x{:04x}/{}",
                     adp, index, sub, mbxWriteAddr, mbxWriteLen, mbxReadAddr, mbxReadLen);
            diagnostics_.diagHexdump(mbxbuf, msg_len, 64);
        }
#endif
        mailboxIO_.drainStale(master, adp, mbxReadAddr, mbxReadLen);

        bool used_alt = false;
        if (!mailboxIO_.apwrWithWkcProbe(master, adp,
                                          mbxWriteAddr, mbxReadAddr,
                                          mbxbuf, static_cast<uint16_t>(mbxWriteLen),
                                          500, &used_alt)) {
            if (!master.isCancelRequested()) {
                TETHER_LOGE(TAG, "{}: SDO download: Mailbox write failed (wr=0x{:04X} rd=0x{:04X})",
                            master.slaveLogPrefix(slaveIndexFromADP(adp)).c_str(), mbxWriteAddr, mbxReadAddr);
            }
            return false;
        }
        if (used_alt) {
            std::swap(mbxWriteAddr, mbxReadAddr);
            std::swap(mbxWriteLen, mbxReadLen);
        }
    }

    if (!mailboxIO_.pollSm1Full(master, adp, transactionTimeoutMs, pollIntervalMs)) {
        if (!master.isCancelRequested()) {
            TETHER_LOGE(TAG, "{}: SDO download: SM1 mailbox never became full (wr=0x{:04X} rd=0x{:04X} index=0x{:04X}:{:02x} timeout={}ms)",
                        master.slaveLogPrefix(slaveIndexFromADP(adp)).c_str(), mbxWriteAddr, mbxReadAddr, index, sub, transactionTimeoutMs);
            diagnostics_.dumpSlaveState(master, adp, mbxWriteAddr, mbxReadAddr);
        }
        return false;
    }

    int stale_retry_count = 0;

    for (int attempt = 0; attempt < MAX_POLL_ATTEMPTS; attempt++) {
        MbxResponseHeader hdr;
        auto outcome = pollSm1AndRead(master, adp, mbxReadAddr, mbxReadLen,
                                      rspbuf, pollIntervalMs, hdr);

        if (outcome == MbxPollOutcome::Cancelled) {
            TETHER_LOGW(TAG, "SDO download cancelled");
            return false;
        }
        if (outcome == MbxPollOutcome::Sm1Empty || outcome == MbxPollOutcome::ReadFailed) {
            if (outcome == MbxPollOutcome::ReadFailed) {
                TETHER_LOGW(TAG, "SDO download: mailbox data read WKC=0 despite SM1 full — backing off (adp=0x{:04X})", adp);
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
                    index, sub, true, pollIntervalMs, "download", hdr);
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
            TETHER_LOGW(TAG, "Non-CoE mailbox response (download): type={} cnt={} (adp=0x{:04X} index=0x{:04X}:{}) — aborting",
                        hdr.type, hdr.cnt, adp, index, sub);
            break;
        }
        if (hdr.cnt != expected_mbx_cnt) {
            if (!adoptCounterOnEcho(adp, mbxbuf, rspbuf, mbxReadLen, hdr,
                                  inoutMbxCnt, mbx_cnt, expected_mbx_cnt,
                                  index, sub, "download") &&
                !checkStaleCounter(master, adp, mbxWriteAddr, mbxWriteLen,
                                   mbxReadAddr, mbxReadLen, mbxbuf,
                                   pollIntervalMs, transactionTimeoutMs, hdr,
                                   inoutMbxCnt, expected_mbx_cnt,
                                   stale_retry_count, index, sub, "download")) {
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
                TETHER_LOGD(TAG, "SDO download abort: index=0x{:04x}:{:02x} code=0x{:08x} ({})",
                         index, sub, abort_code, errorDecoder_.sdoAbortCodeStr(abort_code));
                if (outAbortCode) *outAbortCode = abort_code;
                if (abort_code == 0x06090011 && diagnostics_.isPdoMappingIndex(index)) {
                    diagnostics_.logPdoMappingSubindexDiagnostic(
                        master, adp, inoutMbxCnt,
                        mbxWriteAddr, mbxWriteLen,
                        mbxReadAddr, mbxReadLen,
                        index, sub,
                        diagEnabled, pollIntervalMs, transactionTimeoutMs,
                        makeUploadDiagFn(uploadForDiag));
                }
            } else {
                TETHER_LOGE(TAG, "SDO download abort (malformed response)");
            }
#ifdef TETHER_DIAG_SDO_IO
            if (diagEnabled) {
                TETHER_LOGI(TAG, "SDO download abort raw response (mbx_read_len={})", (unsigned)mbxReadLen);
                diagnostics_.diagHexdump(rspbuf + sdo_offset, mbxReadLen - sdo_offset, 256);
            }
#endif
            return false;
        }

        CoeHeader resp_coe{};
        std::memcpy(&resp_coe, rspbuf + sizeof(MbxHeader), sizeof(resp_coe));
        const uint8_t resp_service = (le16_to_host(resp_coe.raw_le) >> 12) & 0x0Fu;
        if (resp_service != EC_COES_SDORES) {
            TETHER_LOGW(TAG, "Unexpected CoE service (download): 0x{:X} (expected 0x3) (adp=0x{:04X} index=0x{:04X}:{}) — aborting",
                        resp_service, adp, index, sub);
            break;
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
                TETHER_LOGW(TAG, "Stale SDO download response: idx=0x{:04X}:{} expected=0x{:04X}:{} (adp=0x{:04X}) — clearing and re-sending",
                            res_index, res.sub, index, sub, adp);
                // Stale response from a previous operation — just drain and
                // retry with the same counter.  Do NOT sync the counter.
                if (++stale_retry_count <= MAX_STALE_RETRIES) {
                    if (!sendAndWait(master, adp, mbxWriteAddr, mbxWriteLen,
                                     mbxReadAddr, mbxReadLen, mbxbuf, 500,
                                     pollIntervalMs, transactionTimeoutMs, "download")) {
                        return false;
                    }
                } else {
                    std::this_thread::sleep_for(std::chrono::milliseconds(pollIntervalMs));
                }
                continue;
            }
        }

        TETHER_LOGW(TAG, "Unexpected SDO command (download): cmd=0x{:02X} (adp=0x{:04X} index=0x{:04X}:{}) — aborting",
                    sdo_cmd, adp, index, sub);
        break;
    }

    TETHER_LOGE(TAG, "{}: SDO download timeout: index=0x{:04x}:{:02x} (wr=0x{:04X} rd=0x{:04X})",
                master.slaveLogPrefix(slaveIndexFromADP(adp)).c_str(), index, sub, mbxWriteAddr, mbxReadAddr);
    diagnostics_.dumpSlaveState(master, adp, mbxWriteAddr, mbxReadAddr);
    return false;
}

} // namespace Raw
} // namespace EtherCAT


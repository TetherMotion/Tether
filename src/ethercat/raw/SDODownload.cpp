#include "tether/ethercat/SDODownload.hpp"
#include "tether/ethercat/SDOErrorDecoder.hpp"
#include "tether/ethercat/SDOMailboxIO.hpp"
#include "tether/ethercat/SDODiagnostics.hpp"
#include "tether/ethercat/SDOUpload.hpp"
#include "tether/ethercat/Master.hpp"
#include "tether/ethercat/DebugFlags.hpp"
#include "tether/platform/Platform.hpp"
#include "raw/internal.hpp"
#include <cstring>
#include <cstdio>
#include <chrono>
#include <thread>
#include <cinttypes>
#include <utility>

namespace EtherCAT {
namespace Raw {

static const char* TAG = "ethercat";

static uint16_t slaveIndexFromADP(uint16_t adp) {
    return Master::slaveAddressFromADP(adp).slavePosition();
}

// Build a diagnostic upload functor bound to a specific SDOUpload instance.
// Returns a no-op functor (returns false) when @p upload is null, so callers
// can unconditionally pass the result to SDODiagnostics without null checks.
static SDODiagnostics::UploadFn makeUploadDiagFn(SDOUpload* upload) {
    return [upload](Master& m, uint16_t a, uint8_t* c,
                    uint16_t wwa, uint16_t wwl, uint16_t rwa, uint16_t rwl,
                    uint16_t idx, uint8_t s,
                    uint8_t* out, size_t out_cap, size_t* out_len,
                    bool de, unsigned int pim, unsigned int ttm) -> bool {
        if (!upload) return false;
        return upload->execute(m, a, c, wwa, wwl, rwa, rwl, idx, s,
                               out, out_cap, out_len, de, pim, ttm);
    };
}

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
    if (data == nullptr || dataLen == 0) {
        TETHER_LOGE(TAG, "Invalid SDO download parameters (len=%u)", static_cast<unsigned>(dataLen));
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
    uint8_t mbxbuf[kRawSDOMbxBufferSize] = {0};
    if (mbxWriteLen > sizeof(mbxbuf) || mbxReadLen > sizeof(mbxbuf)) {
        TETHER_LOGE(TAG,
            "Tether internal SDO mailbox buffer too small for slave mailbox size "
            "(wr=%u rd=%u, Tether max=%zu bytes). This is a Tether limit, not a slave limit. "
            "Increase ECAT_RAW_SDO_MBX_BUFFER_SIZE in TetherConfig.hpp to >= %u.",
            mbxWriteLen, mbxReadLen, sizeof(mbxbuf),
            (mbxWriteLen > mbxReadLen) ? mbxWriteLen : mbxReadLen);
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
    const uint8_t expected_mbx_cnt = mbx_cnt;

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
        TETHER_LOGE(TAG, "Mailbox write len too small (%u < %u)", mbxWriteLen, static_cast<unsigned>(msg_len));
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
            TETHER_LOGI(TAG, "SDO download (write) request to adp=0x%04X: index=0x%04X:%u [mailbox #%lu -> 0x%04X, len=%u, SM0=0x%02X, AL=0x%04X]",
                     adp, index, sub, (unsigned long)mbx_write_count_, mbxWriteAddr, mbxWriteLen, sm0_status, al_status);
        }

#ifdef TETHER_DIAG_SDO_IO
        if (diagEnabled) {
            TETHER_LOGI(TAG, "SDO Download INIT: adp=0x%04x index=0x%04x sub=%u mbx_wr=0x%04x/%u mbx_rd=0x%04x/%u",
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
            TETHER_LOGE(TAG, "SDO download: Mailbox write failed (adp=0x%04X wr=0x%04X rd=0x%04X)",
                        adp, mbxWriteAddr, mbxReadAddr);
            return false;
        }
        if (used_alt) {
            std::swap(mbxWriteAddr, mbxReadAddr);
            std::swap(mbxWriteLen, mbxReadLen);
        }
    }

    if (!mailboxIO_.pollSm1Full(master, adp, transactionTimeoutMs, pollIntervalMs)) {
        TETHER_LOGE(TAG, "SDO download: SM1 mailbox never became full (adp=0x%04X wr=0x%04X rd=0x%04X index=0x%04X:%02x timeout=%ums)",
                    adp, mbxWriteAddr, mbxReadAddr, index, sub, transactionTimeoutMs);
        diagnostics_.dumpSlaveState(master, adp, mbxWriteAddr, mbxReadAddr);
        return false;
    }

    int stale_retry_count = 0;

    for (int attempt = 0; attempt < MAX_POLL_ATTEMPTS; attempt++) {
        MbxResponseHeader hdr;
        auto outcome = pollSm1AndRead(master, adp, mbxReadAddr, mbxReadLen,
                                      mbxbuf, pollIntervalMs, hdr);

        if (outcome == MbxPollOutcome::Cancelled) {
            TETHER_LOGW(TAG, "SDO download cancelled");
            return false;
        }
        if (outcome == MbxPollOutcome::Sm1Empty || outcome == MbxPollOutcome::ReadFailed) {
            if (outcome == MbxPollOutcome::ReadFailed) {
                TETHER_LOGW(TAG, "SDO download: mailbox data read WKC=0 despite SM1 full — backing off (adp=0x%04X)", adp);
            }
            continue;
        }

        if (hdr.type == EC_MBXT_ERR) {
            handleMailboxError(mbxbuf, hdr, adp, index, sub);
            return false;
        }
        if (hdr.type != EC_MBXT_COE) {
            TETHER_LOGW(TAG, "Non-CoE mailbox response (download): type=%u cnt=%u (adp=0x%04X index=0x%04X:%u) — aborting",
                        hdr.type, hdr.cnt, adp, index, sub);
            break;
        }
        if (hdr.cnt != expected_mbx_cnt) {
            TETHER_LOGW(TAG, "Stale mailbox response (download): cnt=%u expected=%u (adp=0x%04X index=0x%04X:%u) — clearing and re-sending",
                        hdr.cnt, expected_mbx_cnt, adp, index, sub);
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
        if (hdr.len < sizeof(CoeHeader) + 1) {
            std::this_thread::sleep_for(std::chrono::milliseconds(pollIntervalMs));
            continue;
        }

        const size_t sdo_offset = sizeof(MbxHeader) + sizeof(CoeHeader);
        const uint8_t sdo_cmd = mbxbuf[sdo_offset];

        // Check for SDO abort before the CoE service field — some slaves
        // (e.g. ESC211) emit abort responses with a buggy CoE service field
        // (0x2/SDO-REQ instead of 0x3/SDO-RES). Surface the abort code
        // regardless of the CoE service so callers see the real rejection
        // instead of a misleading timeout.
        if ((sdo_cmd & 0xE0u) == EC_SDO_ABORT) {
            if (mbxReadLen >= sdo_offset + sizeof(SdoAbort)) {
                SdoAbort abort{};
                std::memcpy(&abort, mbxbuf + sdo_offset, sizeof(abort));
                const uint32_t abort_code = le32_to_host(abort.abortCode_le);
                TETHER_LOGE(TAG, "SDO download abort: index=0x%04x:%02x code=0x%08" PRIx32 " (%s)",
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
                TETHER_LOGI(TAG, "SDO download abort raw response (mbx_read_len=%u)", (unsigned)mbxReadLen);
                diagnostics_.diagHexdump(mbxbuf + sdo_offset, mbxReadLen - sdo_offset, 256);
            }
#endif
            return false;
        }

        CoeHeader resp_coe{};
        std::memcpy(&resp_coe, mbxbuf + sizeof(MbxHeader), sizeof(resp_coe));
        const uint8_t resp_service = (le16_to_host(resp_coe.raw_le) >> 12) & 0x0Fu;
        if (resp_service != EC_COES_SDORES) {
            TETHER_LOGW(TAG, "Unexpected CoE service (download): 0x%X (expected 0x3) (adp=0x%04X index=0x%04X:%u) — aborting",
                        resp_service, adp, index, sub);
            break;
        }

        if ((sdo_cmd & 0xE0u) == 0x60u) {
            if (mbxReadLen >= sdo_offset + sizeof(SdoInitDownloadRes)) {
                SdoInitDownloadRes res{};
                std::memcpy(&res, mbxbuf + sdo_offset, sizeof(res));
                const uint16_t res_index = le16_to_host(res.index_le);
                if (res_index == index && res.sub == sub) {
                    diagnostics_.logCoeMbxPacket("RX", adp, index, sub, mbxbuf, mbxReadLen,
                            master.debugFlags().coeRxPackets && master.debugFlags().coeRxPacketsFilt.allows(slaveIndexFromADP(adp)));
                    return true;
                }
                TETHER_LOGW(TAG, "Stale SDO download response: idx=0x%04X:%u expected=0x%04X:%u (adp=0x%04X) — clearing and re-sending",
                            res_index, res.sub, index, sub, adp);
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

        TETHER_LOGW(TAG, "Unexpected SDO command (download): cmd=0x%02X (adp=0x%04X index=0x%04X:%u) — aborting",
                    sdo_cmd, adp, index, sub);
        break;
    }

    TETHER_LOGE(TAG, "SDO download timeout: index=0x%04x:%02x (adp=0x%04X wr=0x%04X rd=0x%04X)",
                index, sub, adp, mbxWriteAddr, mbxReadAddr);
    diagnostics_.dumpSlaveState(master, adp, mbxWriteAddr, mbxReadAddr);
    return false;
}

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
    uint8_t mbxbuf[kRawSDOMbxBufferSize] = {0};
    if (mbxWriteLen > sizeof(mbxbuf) || mbxReadLen > sizeof(mbxbuf)) {
        TETHER_LOGE(TAG,
            "Tether internal SDO mailbox buffer too small for slave mailbox size "
            "(wr=%u rd=%u, Tether max=%zu bytes). This is a Tether limit, not a slave limit. "
            "Increase ECAT_RAW_SDO_MBX_BUFFER_SIZE in TetherConfig.hpp to >= %u.",
            mbxWriteLen, mbxReadLen, sizeof(mbxbuf),
            (mbxWriteLen > mbxReadLen) ? mbxWriteLen : mbxReadLen);
        return false;
    }

    const size_t sdo_header_size = sizeof(MbxHeader) + sizeof(CoeHeader) + sizeof(SdoInitDownloadReq);
    const size_t msg_len = sdo_header_size + dataLen;
    if (msg_len > mbxWriteLen) {
        TETHER_LOGE(TAG, "Normal download: data_len=%zu exceeds slave mailbox write capacity "
                    "(msg=%zu > slave mailbox wr=%u). The slave mailbox is too small for this data; "
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
    const uint8_t expected_mbx_cnt = mbx_cnt;

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
        TETHER_LOGE(TAG, "SDO normal download: Mailbox write failed (adp=0x%04X wr=0x%04X rd=0x%04X)",
                    adp, mbxWriteAddr, mbxReadAddr);
        return false;
    }
    if (used_alt) {
        std::swap(mbxWriteAddr, mbxReadAddr);
        std::swap(mbxWriteLen, mbxReadLen);
    }

    if (!mailboxIO_.pollSm1Full(master, adp, transactionTimeoutMs, pollIntervalMs)) {
        TETHER_LOGE(TAG, "SDO normal download: SM1 never became full (adp=0x%04X index=0x%04X:%u timeout=%ums)",
                    adp, index, sub, transactionTimeoutMs);
        diagnostics_.dumpSlaveState(master, adp, mbxWriteAddr, mbxReadAddr);
        return false;
    }

    int stale_retry_count = 0;

    for (int attempt = 0; attempt < MAX_POLL_ATTEMPTS; attempt++) {
        MbxResponseHeader hdr;
        auto outcome = pollSm1AndRead(master, adp, mbxReadAddr, mbxReadLen,
                                      mbxbuf, pollIntervalMs, hdr);

        if (outcome == MbxPollOutcome::Cancelled) {
            TETHER_LOGW(TAG, "SDO normal download cancelled");
            return false;
        }
        if (outcome == MbxPollOutcome::Sm1Empty || outcome == MbxPollOutcome::ReadFailed) {
            if (outcome == MbxPollOutcome::ReadFailed) {
                TETHER_LOGW(TAG, "SDO normal download: mailbox data read WKC=0 despite SM1 full (adp=0x%04X)", adp);
            }
            continue;
        }

        if (hdr.type == EC_MBXT_ERR) {
            handleMailboxError(mbxbuf, hdr, adp, index, sub);
            return false;
        }
        if (hdr.type != EC_MBXT_COE) {
            TETHER_LOGW(TAG, "Non-CoE mailbox response (normal download): type=%u cnt=%u (adp=0x%04X index=0x%04X:%u) — aborting",
                        hdr.type, hdr.cnt, adp, index, sub);
            break;
        }
        if (hdr.cnt != expected_mbx_cnt) {
            TETHER_LOGW(TAG, "Stale mailbox response (normal download): cnt=%u expected=%u (adp=0x%04X index=0x%04X:%u) — re-sending",
                        hdr.cnt, expected_mbx_cnt, adp, index, sub);
            if (++stale_retry_count <= MAX_STALE_RETRIES) {
                if (!sendAndWait(master, adp, mbxWriteAddr, mbxWriteLen,
                                 mbxReadAddr, mbxReadLen, mbxbuf, 500,
                                 pollIntervalMs, transactionTimeoutMs, "normal download")) {
                    return false;
                }
            } else {
                std::this_thread::sleep_for(std::chrono::milliseconds(pollIntervalMs));
            }
            continue;
        }
        if (hdr.len < sizeof(CoeHeader) + 1) {
            std::this_thread::sleep_for(std::chrono::milliseconds(pollIntervalMs));
            continue;
        }

        const size_t sdo_offset = sizeof(MbxHeader) + sizeof(CoeHeader);
        const uint8_t sdo_cmd = mbxbuf[sdo_offset];

        // Check for SDO abort before the CoE service field — some slaves
        // (e.g. ESC211) emit abort responses with a buggy CoE service field
        // (0x2/SDO-REQ instead of 0x3/SDO-RES). Surface the abort code
        // regardless of the CoE service so callers see the real rejection
        // instead of a misleading timeout.
        if ((sdo_cmd & 0xE0u) == EC_SDO_ABORT) {
            if (mbxReadLen >= sdo_offset + sizeof(SdoAbort)) {
                SdoAbort abort{};
                std::memcpy(&abort, mbxbuf + sdo_offset, sizeof(abort));
                const uint32_t abort_code = le32_to_host(abort.abortCode_le);
                TETHER_LOGE(TAG, "SDO normal download abort: index=0x%04x:%02x code=0x%08" PRIx32 " (%s)",
                         index, sub, abort_code, errorDecoder_.sdoAbortCodeStr(abort_code));
                if (outAbortCode) *outAbortCode = abort_code;
            } else {
                TETHER_LOGE(TAG, "SDO normal download abort (malformed response)");
            }
            return false;
        }

        CoeHeader resp_coe{};
        std::memcpy(&resp_coe, mbxbuf + sizeof(MbxHeader), sizeof(resp_coe));
        const uint8_t resp_service = (le16_to_host(resp_coe.raw_le) >> 12) & 0x0Fu;
        if (resp_service != EC_COES_SDORES) {
            TETHER_LOGW(TAG, "Unexpected CoE service (normal download): 0x%X (expected 0x3) (adp=0x%04X index=0x%04X:%u) — aborting",
                        resp_service, adp, index, sub);
            break;
        }

        if ((sdo_cmd & 0xE0u) == 0x60u) {
            if (mbxReadLen >= sdo_offset + sizeof(SdoInitDownloadRes)) {
                SdoInitDownloadRes res{};
                std::memcpy(&res, mbxbuf + sdo_offset, sizeof(res));
                const uint16_t res_index = le16_to_host(res.index_le);
                if (res_index == index && res.sub == sub) {
                    diagnostics_.logCoeMbxPacket("RX", adp, index, sub, mbxbuf, mbxReadLen,
                            master.debugFlags().coeRxPackets && master.debugFlags().coeRxPacketsFilt.allows(slaveIndexFromADP(adp)));
                    return true;
                }
                TETHER_LOGW(TAG, "Stale SDO normal download response: idx=0x%04X:%u expected=0x%04X:%u (adp=0x%04X) — re-sending",
                            res_index, res.sub, index, sub, adp);
                if (++stale_retry_count <= MAX_STALE_RETRIES) {
                    if (!sendAndWait(master, adp, mbxWriteAddr, mbxWriteLen,
                                     mbxReadAddr, mbxReadLen, mbxbuf, 500,
                                     pollIntervalMs, transactionTimeoutMs, "normal download")) {
                        return false;
                    }
                } else {
                    std::this_thread::sleep_for(std::chrono::milliseconds(pollIntervalMs));
                }
                continue;
            }
        }

        TETHER_LOGW(TAG, "Unexpected SDO command (normal download): cmd=0x%02X (adp=0x%04X index=0x%04X:%u) — aborting",
                    sdo_cmd, adp, index, sub);
        break;
    }

    TETHER_LOGE(TAG, "SDO normal download timeout: index=0x%04x:%02x (adp=0x%04X wr=0x%04X rd=0x%04X)",
                index, sub, adp, mbxWriteAddr, mbxReadAddr);
    diagnostics_.dumpSlaveState(master, adp, mbxWriteAddr, mbxReadAddr);
    return false;
}

bool SDODownload::executeSegmented(Master& master, uint16_t adp,
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
    uint8_t mbxbuf[kRawSDOMbxBufferSize] = {0};
    if (mbxWriteLen > sizeof(mbxbuf) || mbxReadLen > sizeof(mbxbuf)) {
        TETHER_LOGE(TAG,
            "Tether internal SDO mailbox buffer too small for slave mailbox size "
            "(wr=%u rd=%u, Tether max=%zu bytes). This is a Tether limit, not a slave limit. "
            "Increase ECAT_RAW_SDO_MBX_BUFFER_SIZE in TetherConfig.hpp to >= %u.",
            mbxWriteLen, mbxReadLen, sizeof(mbxbuf),
            (mbxWriteLen > mbxReadLen) ? mbxWriteLen : mbxReadLen);
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

    const size_t sdo_hdr_size = sizeof(MbxHeader) + sizeof(CoeHeader) + sizeof(SdoInitDownloadReq);
    const size_t seg_max_inline = (mbxWriteLen > sdo_hdr_size)
        ? (mbxWriteLen - sdo_hdr_size) : 0;
    const size_t inline_bytes = (dataLen < seg_max_inline) ? dataLen : seg_max_inline;

    {
        const uint8_t expected_mbx_cnt = mbx_cnt;

        MbxHeader mbx{};
        mbx.length_le = host_to_le16(static_cast<uint16_t>(sizeof(CoeHeader) + sizeof(SdoInitDownloadReq) + inline_bytes));
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
        if (inline_bytes > 0) {
            std::memcpy(mbxbuf + sdo_hdr_size, data, inline_bytes);
        }

        const size_t init_msg_len = sdo_hdr_size + inline_bytes;
        if (init_msg_len > mbxWriteLen) {
            TETHER_LOGE(TAG, "Mailbox write len too small for segmented init (%u < %u)",
                        mbxWriteLen, static_cast<unsigned>(init_msg_len));
            return false;
        }

        diagnostics_.logCoeMbxPacket("TX", adp, index, sub, mbxbuf, init_msg_len,
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
            TETHER_LOGE(TAG, "SDO segmented download init: Mailbox write failed (adp=0x%04X wr=0x%04X rd=0x%04X)",
                        adp, mbxWriteAddr, mbxReadAddr);
            return false;
        }
        if (used_alt) {
            std::swap(mbxWriteAddr, mbxReadAddr);
            std::swap(mbxWriteLen, mbxReadLen);
        }

        if (!mailboxIO_.pollSm1Full(master, adp, transactionTimeoutMs, pollIntervalMs)) {
            TETHER_LOGE(TAG, "SDO segmented download init: SM1 never became full (adp=0x%04X wr=0x%04X rd=0x%04X index=0x%04X:%02x timeout=%ums)",
                        adp, mbxWriteAddr, mbxReadAddr, index, sub, transactionTimeoutMs);
            diagnostics_.dumpSlaveState(master, adp, mbxWriteAddr, mbxReadAddr);
            return false;
        }

        bool got_init = false;
        int stale_retry_count = 0;
        for (int attempt = 0; attempt < MAX_POLL_ATTEMPTS; attempt++) {
            MbxResponseHeader hdr;
            auto outcome = pollSm1AndRead(master, adp, mbxReadAddr, mbxReadLen,
                                          mbxbuf, pollIntervalMs, hdr);

            if (outcome == MbxPollOutcome::Cancelled) {
                TETHER_LOGW(TAG, "SDO segmented download cancelled");
                return false;
            }
            if (outcome == MbxPollOutcome::Sm1Empty || outcome == MbxPollOutcome::ReadFailed) {
                if (outcome == MbxPollOutcome::ReadFailed) {
                    TETHER_LOGW(TAG, "SDO segmented download init: mailbox data read WKC=0 despite SM1 full — backing off (adp=0x%04X)", adp);
                }
                continue;
            }

            if (hdr.type == EC_MBXT_ERR) {
                handleMailboxError(mbxbuf, hdr, adp, index, sub);
                return false;
            }
            if (hdr.type != EC_MBXT_COE) {
                TETHER_LOGW(TAG, "Non-CoE mailbox response (seg download init): type=%u cnt=%u (adp=0x%04X index=0x%04X:%u) — aborting",
                            hdr.type, hdr.cnt, adp, index, sub);
                break;
            }
            if (hdr.cnt != expected_mbx_cnt) {
                TETHER_LOGW(TAG, "Stale mailbox response (segmented download init): cnt=%u expected=%u (adp=0x%04X index=0x%04X:%u) — clearing and re-sending",
                            hdr.cnt, expected_mbx_cnt, adp, index, sub);
                if (++stale_retry_count <= MAX_STALE_RETRIES) {
                    if (!sendAndWait(master, adp, mbxWriteAddr, mbxWriteLen,
                                     mbxReadAddr, mbxReadLen, mbxbuf, 500,
                                     pollIntervalMs, transactionTimeoutMs, "seg download init")) {
                        return false;
                    }
                } else {
                    std::this_thread::sleep_for(std::chrono::milliseconds(pollIntervalMs));
                }
                continue;
            }
            if (hdr.len < sizeof(CoeHeader) + 1) {
                std::this_thread::sleep_for(std::chrono::milliseconds(pollIntervalMs));
                continue;
            }

            const size_t sdo_offset = sizeof(MbxHeader) + sizeof(CoeHeader);
            const uint8_t sdo_cmd = mbxbuf[sdo_offset];

            // Check for SDO abort before the CoE service field — some slaves
            // (e.g. ESC211) emit abort responses with a buggy CoE service field
            // (0x2/SDO-REQ instead of 0x3/SDO-RES). Surface the abort code
            // regardless of the CoE service so callers see the real rejection
            // instead of a misleading timeout.
            if ((sdo_cmd & 0xE0u) == EC_SDO_ABORT) {
                if (mbxReadLen >= sdo_offset + sizeof(SdoAbort)) {
                    SdoAbort abort{};
                    std::memcpy(&abort, mbxbuf + sdo_offset, sizeof(abort));
                    const uint32_t abort_code = le32_to_host(abort.abortCode_le);
                    TETHER_LOGE(TAG, "SDO segmented download abort: index=0x%04x:%02x code=0x%08" PRIx32 " (%s)",
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
                    TETHER_LOGE(TAG, "SDO segmented download abort (malformed response)");
                }
#ifdef TETHER_DIAG_SDO_IO
                if (diagEnabled) {
                    TETHER_LOGI(TAG, "SDO segmented download abort raw response (mbx_read_len=%u)", (unsigned)mbxReadLen);
                    diagnostics_.diagHexdump(mbxbuf + sdo_offset, mbxReadLen - sdo_offset, 256);
                }
#endif
                return false;
            }

            if (hdr.len < sizeof(CoeHeader) + sizeof(SdoInitDownloadRes)) {
                std::this_thread::sleep_for(std::chrono::milliseconds(pollIntervalMs));
                continue;
            }

            CoeHeader resp_coe{};
            std::memcpy(&resp_coe, mbxbuf + sizeof(MbxHeader), sizeof(resp_coe));
            const uint8_t resp_service = (le16_to_host(resp_coe.raw_le) >> 12) & 0x0Fu;
            if (resp_service != EC_COES_SDORES) {
                TETHER_LOGW(TAG, "Unexpected CoE service (seg download init): 0x%X (expected 0x3) (adp=0x%04X index=0x%04X:%u) — aborting",
                            resp_service, adp, index, sub);
                break;
            }

            if ((sdo_cmd & 0xE0u) == 0x60u) {
                SdoInitDownloadRes res{};
                std::memcpy(&res, mbxbuf + sdo_offset, sizeof(res));
                const uint16_t res_index = le16_to_host(res.index_le);

                if (res_index == index && res.sub == sub) {
                    got_init = true;
                    break;
                }

                TETHER_LOGW(TAG, "Stale SDO segmented download init response: idx=0x%04X:%u expected=0x%04X:%u (adp=0x%04X) — clearing and re-sending",
                            res_index, res.sub, index, sub, adp);
                if (++stale_retry_count <= MAX_STALE_RETRIES) {
                    if (!sendAndWait(master, adp, mbxWriteAddr, mbxWriteLen,
                                     mbxReadAddr, mbxReadLen, mbxbuf, 500,
                                     pollIntervalMs, transactionTimeoutMs, "seg download init")) {
                        return false;
                    }
                } else {
                    std::this_thread::sleep_for(std::chrono::milliseconds(pollIntervalMs));
                }
                continue;
            }

            TETHER_LOGW(TAG, "Unexpected SDO command (seg download init): cmd=0x%02X (adp=0x%04X index=0x%04X:%u) — aborting",
                        sdo_cmd, adp, index, sub);
            break;
        }

        if (!got_init) {
            TETHER_LOGE(TAG, "SDO segmented download init timeout: index=0x%04x:%02x (adp=0x%04X wr=0x%04X rd=0x%04X)",
                        index, sub, adp, mbxWriteAddr, mbxReadAddr);
            diagnostics_.dumpSlaveState(master, adp, mbxWriteAddr, mbxReadAddr);
            return false;
        }
    }

    bool toggle = false;
    const uint8_t* seg_ptr = data + inline_bytes;
    size_t remaining = dataLen - inline_bytes;

    if (remaining == 0) {
        TETHER_LOGI(TAG, "SDO segmented download: all %zu bytes sent inline in init packet (no segments needed)",
                    dataLen);
        return true;
    }

    for (int seg = 0; seg < ECAT_SDO_DOWNLOAD_MAX_SEGMENTS && remaining > 0; seg++) {
        const size_t seg_bytes = (remaining < 7) ? remaining : 7;
        const uint8_t n = static_cast<uint8_t>(7 - seg_bytes);
        const bool last = (remaining == seg_bytes);
        const uint8_t seg_cmd = static_cast<uint8_t>((toggle ? 0x10u : 0x00u) | (n << 1) | (last ? 0x01u : 0x00u));

        const uint8_t expected_mbx_cnt = mbx_cnt;

        MbxHeader seg_mbx{};
        seg_mbx.length_le = host_to_le16(static_cast<uint16_t>(sizeof(CoeHeader) + sizeof(SdoDownloadSegReq)));
        seg_mbx.address_le = host_to_le16(0);
        seg_mbx.priority = 0;
        seg_mbx.mbxtype = mbx_type_with_cnt(EC_MBXT_COE, mbx_cnt);

        CoeHeader seg_coe{};
        seg_coe.raw_le = host_to_le16(coe_make_raw(0, EC_COES_SDOREQ));

        SdoDownloadSegReq seg_req{};
        seg_req.cmd = seg_cmd;
        std::memcpy(seg_req.data, seg_ptr, seg_bytes);

        std::memset(mbxbuf, 0, sizeof(mbxbuf));
        std::memcpy(mbxbuf, &seg_mbx, sizeof(seg_mbx));
        std::memcpy(mbxbuf + sizeof(seg_mbx), &seg_coe, sizeof(seg_coe));
        std::memcpy(mbxbuf + sizeof(seg_mbx) + sizeof(seg_coe), &seg_req, sizeof(seg_req));

        mbx_cnt = static_cast<uint8_t>((mbx_cnt >= 7) ? 1 : (mbx_cnt + 1));
        if (inoutMbxCnt != nullptr) {
            *inoutMbxCnt = mbx_cnt;
        }

        bool used_alt = false;
        if (!mailboxIO_.apwrWithWkcProbe(master, adp,
                                          mbxWriteAddr, mbxReadAddr,
                                          mbxbuf, static_cast<uint16_t>(mbxWriteLen),
                                          500, &used_alt)) {
            TETHER_LOGE(TAG, "SDO segmented download segment %d: Mailbox write failed (adp=0x%04X wr=0x%04X rd=0x%04X)",
                        seg, adp, mbxWriteAddr, mbxReadAddr);
            return false;
        }
        if (used_alt) {
            std::swap(mbxWriteAddr, mbxReadAddr);
            std::swap(mbxWriteLen, mbxReadLen);
        }

        if (!mailboxIO_.pollSm1Full(master, adp, transactionTimeoutMs, pollIntervalMs)) {
            TETHER_LOGE(TAG, "SDO segmented download segment %d: SM1 never became full (adp=0x%04X wr=0x%04X rd=0x%04X index=0x%04X:%02x timeout=%ums)",
                        seg, adp, mbxWriteAddr, mbxReadAddr, index, sub, transactionTimeoutMs);
            diagnostics_.dumpSlaveState(master, adp, mbxWriteAddr, mbxReadAddr);
            return false;
        }

        bool got_seg = false;
        int stale_retry_count = 0;
        for (int attempt2 = 0; attempt2 < MAX_POLL_ATTEMPTS; attempt2++) {
            MbxResponseHeader hdr;
            auto outcome = pollSm1AndRead(master, adp, mbxReadAddr, mbxReadLen,
                                          mbxbuf, pollIntervalMs, hdr);

            if (outcome == MbxPollOutcome::Cancelled) {
                TETHER_LOGW(TAG, "SDO segmented download cancelled");
                return false;
            }
            if (outcome == MbxPollOutcome::Sm1Empty || outcome == MbxPollOutcome::ReadFailed) {
                if (outcome == MbxPollOutcome::ReadFailed) {
                    TETHER_LOGW(TAG, "SDO segmented download segment %d: mailbox data read WKC=0 despite SM1 full — backing off (adp=0x%04X)", seg, adp);
                }
                continue;
            }

            if (hdr.type == EC_MBXT_ERR) {
                handleMailboxError(mbxbuf, hdr, adp, index, sub);
                return false;
            }
            if (hdr.type != EC_MBXT_COE) {
                TETHER_LOGW(TAG, "Non-CoE mailbox response (seg download seg %d): type=%u cnt=%u (adp=0x%04X index=0x%04X:%u) — aborting",
                            seg, hdr.type, hdr.cnt, adp, index, sub);
                break;
            }
            if (hdr.len < sizeof(CoeHeader) + 1) {
                std::this_thread::sleep_for(std::chrono::milliseconds(pollIntervalMs));
                continue;
            }

            const uint8_t* seg_res = mbxbuf + sizeof(MbxHeader) + sizeof(CoeHeader);
            const uint8_t seg_res_cmd = seg_res[0];

            // Check for SDO abort before the CoE service field — some slaves
            // (e.g. ESC211) emit abort responses with a buggy CoE service field
            // (0x2/SDO-REQ instead of 0x3/SDO-RES). Surface the abort code
            // regardless of the CoE service so callers see the real rejection
            // instead of a misleading timeout.
            if ((seg_res_cmd & 0xE0u) == EC_SDO_ABORT) {
                if (mbxReadLen >= sizeof(MbxHeader) + sizeof(CoeHeader) + sizeof(SdoAbort)) {
                    SdoAbort abort{};
                    std::memcpy(&abort, seg_res, sizeof(abort));
                    const uint32_t abort_code = le32_to_host(abort.abortCode_le);
                    TETHER_LOGE(TAG, "SDO segmented download segment %d abort: index=0x%04x:%02x code=0x%08" PRIx32 " (%s)",
                                seg, index, sub, abort_code, errorDecoder_.sdoAbortCodeStr(abort_code));
                    if (outAbortCode) *outAbortCode = abort_code;
                } else {
                    TETHER_LOGE(TAG, "SDO segmented download segment %d abort (malformed response)", seg);
                }
                return false;
            }

            if (hdr.len < sizeof(CoeHeader) + sizeof(SdoDownloadSegRes)) {
                std::this_thread::sleep_for(std::chrono::milliseconds(pollIntervalMs));
                continue;
            }

            const auto* r2_coe = reinterpret_cast<const CoeHeader*>(mbxbuf + sizeof(MbxHeader));
            const uint16_t r2_coe_raw = le16_to_host(r2_coe->raw_le);
            const uint8_t r2_service = (r2_coe_raw >> 12) & 0x0Fu;
            if (r2_service != EC_COES_SDORES) {
                TETHER_LOGW(TAG, "Unexpected CoE service (seg download seg %d): 0x%X (expected 0x3) (adp=0x%04X index=0x%04X:%u) — aborting",
                            seg, r2_service, adp, index, sub);
                break;
            }

            const uint8_t seg_ccs = (seg_res_cmd >> 5) & 0x07u;
            if (seg_ccs != 1) {
                TETHER_LOGW(TAG, "Unexpected SDO command (seg download seg %d): cmd=0x%02X ccs=%u (adp=0x%04X index=0x%04X:%u) — aborting",
                            seg, seg_res_cmd, seg_ccs, adp, index, sub);
                break;
            }

            const bool seg_toggle = (seg_res_cmd & 0x10u) != 0;
            if (seg_toggle != toggle) {
                TETHER_LOGW(TAG, "SDO segmented download segment %d toggle mismatch: got=%u expected=%u (adp=0x%04X index=0x%04X:%u) — re-sending",
                            seg, seg_toggle, toggle, adp, index, sub);
                if (++stale_retry_count <= MAX_STALE_RETRIES) {
                    if (!sendAndWait(master, adp, mbxWriteAddr, mbxWriteLen,
                                     mbxReadAddr, mbxReadLen, mbxbuf, 500,
                                     pollIntervalMs, transactionTimeoutMs, "seg download segment")) {
                        return false;
                    }
                } else {
                    std::this_thread::sleep_for(std::chrono::milliseconds(pollIntervalMs));
                }
                continue;
            }

            if (hdr.cnt != expected_mbx_cnt) {
                TETHER_LOGW(TAG, "Stale mailbox response (segmented download segment %d): cnt=%u expected=%u (adp=0x%04X index=0x%04X:%u) — re-sending",
                            seg, hdr.cnt, expected_mbx_cnt, adp, index, sub);
                if (++stale_retry_count <= MAX_STALE_RETRIES) {
                    if (!sendAndWait(master, adp, mbxWriteAddr, mbxWriteLen,
                                     mbxReadAddr, mbxReadLen, mbxbuf, 500,
                                     pollIntervalMs, transactionTimeoutMs, "seg download segment")) {
                        return false;
                    }
                } else {
                    std::this_thread::sleep_for(std::chrono::milliseconds(pollIntervalMs));
                }
                continue;
            }

            if (last) {
                diagnostics_.logCoeMbxPacket("RX", adp, index, sub, mbxbuf, mbxReadLen,
                                master.debugFlags().coeRxPackets && master.debugFlags().coeRxPacketsFilt.allows(slaveIndexFromADP(adp)));
                return true;
            }

            toggle = !toggle;
            seg_ptr += seg_bytes;
            remaining -= seg_bytes;
            got_seg = true;
            break;
        }

        if (!got_seg) {
            TETHER_LOGE(TAG, "SDO segmented download segment %d timeout (adp=0x%04X wr=0x%04X rd=0x%04X index=0x%04X:%u)",
                        seg, adp, mbxWriteAddr, mbxReadAddr, index, sub);
            diagnostics_.dumpSlaveState(master, adp, mbxWriteAddr, mbxReadAddr);
            return false;
        }
    }

    TETHER_LOGE(TAG, "SDO segmented download exceeded max segments (%d) for index=0x%04X:%u (len=%zu)",
                ECAT_SDO_DOWNLOAD_MAX_SEGMENTS, index, sub, dataLen);
    return false;
}

} // namespace Raw
} // namespace EtherCAT

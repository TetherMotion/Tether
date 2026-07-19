#include "tether/ethercat/SDOUpload.hpp"
#include "tether/ethercat/SDOErrorDecoder.hpp"
#include "tether/ethercat/SDOMailboxIO.hpp"
#include "tether/ethercat/SDODiagnostics.hpp"
#include "tether/ethercat/Master.hpp"
#include "tether/ethercat/DebugFlags.hpp"
#include "tether/platform/Platform.hpp"
#include "raw/internal.hpp"
#include <cstring>
#include <cstdio>
#include <chrono>
#include <thread>
#include <cinttypes>

namespace EtherCAT {
namespace Raw {

static const char* TAG = "ethercat";

static uint16_t slaveIndexFromADP(uint16_t adp) {
    return Master::slaveAddressFromADP(adp).slavePosition();
}

bool SDOUpload::execute(Master& master, uint16_t adp,
                        uint8_t* inoutMbxCnt,
                        uint16_t mbxWriteAddr, uint16_t mbxWriteLen,
                        uint16_t mbxReadAddr, uint16_t mbxReadLen,
                        uint16_t index, uint8_t sub,
                        uint8_t* out, size_t outCap, size_t* outLen,
                        bool diagEnabled,
                        unsigned int pollIntervalMs,
                        unsigned int transactionTimeoutMs,
                        uint32_t* outAbortCode) {
    if (outAbortCode) *outAbortCode = 0;
    if (outLen) {
        *outLen = 0;
    }

    // Translate the internal Complete-Access signal (bit 7 set in the
    // subindex, as set by CoEManager when options.complete_access is true)
    // into the spec-compliant ETG.1000.6 Complete-Access bit (bit 4 / 0x10
    // in the SDO command byte).  On the wire the subindex must carry the
    // real entry number without the 0x80 marker, and the slave's response
    // will echo the clean subindex — so we translate `sub` in-place here
    // and all downstream code (request, response matching, logging,
    // diagnostics) consistently uses the clean value.
    const bool complete_access = (sub & 0x80u) != 0;
    sub = static_cast<uint8_t>(sub & 0x7Fu);

    uint8_t mbxbuf[kRawSDOMbxBufferSize] = {0};
    if (mbxWriteLen > sizeof(mbxbuf) || mbxReadLen > sizeof(mbxbuf)) {
        TETHER_LOGE(TAG, "Mailbox size too large (wr=%u rd=%u, max=%zu)", mbxWriteLen, mbxReadLen, sizeof(mbxbuf));
        return false;
    }

    uint16_t coe_number = static_cast<uint16_t>(master.allocIdx());

    uint8_t mbx_cnt = 0;
    if (inoutMbxCnt != nullptr) {
        mbx_cnt = *inoutMbxCnt;
    }

    const uint8_t expected_mbx_cnt = mbx_cnt;

    MbxHeader mbx{};
    mbx.length_le = host_to_le16(static_cast<uint16_t>(sizeof(CoeHeader) + sizeof(SdoInitUploadReq)));
    mbx.address_le = host_to_le16(0);
    mbx.priority = 0;
    mbx.mbxtype = mbx_type_with_cnt(EC_MBXT_COE, mbx_cnt);

    CoeHeader coe{};
    coe.raw_le = host_to_le16(coe_make_raw(0, EC_COES_SDOREQ));

    SdoInitUploadReq sdo{};
    sdo.cmd = static_cast<uint8_t>(EC_SDO_UP_REQ | (complete_access ? 0x10u : 0x00u));
    sdo.index_le = host_to_le16(index);
    sdo.sub = sub;
    std::memset(sdo.reserved, 0, sizeof(sdo.reserved));

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
            TETHER_LOGI(TAG, "SDO upload (read) request to adp=0x%04X: index=0x%04X:%u [mailbox #%lu -> 0x%04X, len=%u, SM0=0x%02X, AL=0x%04X]",
                     adp, index, sub, (unsigned long)mbx_write_count_, mbxWriteAddr, mbxWriteLen, sm0_status, al_status);
        }

#ifdef TETHER_DIAG_SDO_IO
        if (diagEnabled) {
            TETHER_LOGI(TAG, "SDO Upload INIT: adp=0x%04x index=0x%04x sub=%u coe_num=%u mbx_wr=0x%04x/%u mbx_rd=0x%04x/%u SM0=0x%02x AL=0x%04x",
                     adp, index, sub, coe_number, mbxWriteAddr, mbxWriteLen, mbxReadAddr, mbxReadLen, sm0_status, al_status);
            diagnostics_.diagHexdump(mbxbuf, msg_len, 64);
        }
#endif

        mailboxIO_.drainStale(master, adp, mbxReadAddr, mbxReadLen);

        {
            bool used_alt = false;
            if (!mailboxIO_.apwrWithWkcProbe(master, adp,
                                              mbxWriteAddr,
                                              mbxReadAddr,
                                              mbxbuf,
                                              static_cast<uint16_t>(mbxWriteLen),
                                              500,
                                              &used_alt)) {
                return false;
            }

            if (used_alt) {
                std::swap(mbxWriteAddr, mbxReadAddr);
                std::swap(mbxWriteLen, mbxReadLen);
            }
        }

        if (!mailboxIO_.pollSm1Full(master, adp, transactionTimeoutMs, pollIntervalMs)) {
            TETHER_LOGE(TAG, "SDO upload: SM1 mailbox never became full (adp=0x%04X wr=0x%04X rd=0x%04X index=0x%04X:%u timeout=%ums)",
                        adp, mbxWriteAddr, mbxReadAddr, index, sub, transactionTimeoutMs);
            diagnostics_.dumpSlaveState(master, adp, mbxWriteAddr, mbxReadAddr);
            return false;
        }

        bool logged_any_mbx = false;
        bool logged_mbx_mismatch = false;
        bool got_init_response = false;
        int stale_retry_count = 0;
        const uint8_t* r_sdo_bytes = nullptr;
        uint8_t sdo_cmd = 0;
        uint16_t r_len = 0;

        for (int attempt = 0; attempt < MAX_POLL_ATTEMPTS; attempt++) {
            MbxResponseHeader hdr;
            auto outcome = pollSm1AndRead(master, adp, mbxReadAddr, mbxReadLen,
                                          mbxbuf, pollIntervalMs, hdr);

            if (outcome == MbxPollOutcome::Cancelled) {
                TETHER_LOGW(TAG, "SDO upload cancelled");
                return false;
            }
            if (outcome == MbxPollOutcome::Sm1Empty || outcome == MbxPollOutcome::ReadFailed) {
                if (outcome == MbxPollOutcome::ReadFailed) {
                    TETHER_LOGW(TAG, "SDO upload: mailbox data read WKC=0 despite SM1 full — backing off (adp=0x%04X)", adp);
                }
                continue;
            }

            r_len = hdr.len;
            if (!logged_any_mbx) {
                logged_any_mbx = true;
#ifdef TETHER_DIAG_SDO_IO
                if (diagEnabled) {
                    const uint8_t *p = mbxbuf;
                    TETHER_LOGI(TAG, "MBX poll: len=%u type=0x%02x cnt=%u rawType=0x%02x bytes=%02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x",
                             r_len, hdr.type, hdr.cnt, hdr.rawMbxType, p[0], p[1], p[2], p[3], p[4], p[5], p[6],
                             p[7], p[8], p[9], p[10], p[11], p[12], p[13], p[14], p[15]);
                }
#endif
            }
            if (hdr.type == EC_MBXT_ERR) {
                handleMailboxError(mbxbuf, hdr, adp, index, sub);
                return false;
            }
            if (r_len == 0 || hdr.type != EC_MBXT_COE) {
                if (!logged_mbx_mismatch) {
                    logged_mbx_mismatch = true;
                    char tmp[96];
                    snprintf(tmp, sizeof(tmp), "MBX mismatch: len=%u type=0x%02x cnt=%u prio=0x%02x rawType=0x%02x",
                             r_len, hdr.type, hdr.cnt, hdr.priority, hdr.rawMbxType);
                    TETHER_LOGI(TAG, "%s", tmp);
                }
                break;
            }
            if (sizeof(MbxHeader) + r_len > mbxReadLen) {
                std::this_thread::sleep_for(std::chrono::milliseconds(pollIntervalMs));
                continue;
            }
            if (r_len < sizeof(CoeHeader) + 1) {
                std::this_thread::sleep_for(std::chrono::milliseconds(pollIntervalMs));
                continue;
            }

            r_sdo_bytes = mbxbuf + sizeof(MbxHeader) + sizeof(CoeHeader);
            sdo_cmd = r_sdo_bytes[0];

            // Check for SDO abort before the CoE service field — some slaves
            // (e.g. ESC211) emit abort responses with a buggy CoE service field
            // (0x2/SDO-REQ instead of 0x3/SDO-RES). Surface the abort code
            // regardless of the CoE service so callers see the real rejection
            // instead of a misleading timeout.
            if (sdo_cmd == EC_SDO_ABORT) {
                if (r_len >= sizeof(CoeHeader) + sizeof(SdoAbort)) {
                    const auto *ab = reinterpret_cast<const SdoAbort *>(r_sdo_bytes);
                    const uint32_t abort_code = le32_to_host(ab->abortCode_le);
                    TETHER_LOGE(TAG, "SDO abort 0x%04x:%u code=0x%08" PRIx32 " (%s)",
                             le16_to_host(ab->index_le), ab->sub,
                             abort_code, errorDecoder_.sdoAbortCodeStr(abort_code));
                    if (outAbortCode) *outAbortCode = abort_code;
                }
#ifdef TETHER_DIAG_SDO_IO
                if (diagEnabled) {
                    TETHER_LOGI(TAG, "SDO abort raw response (len=%u)", r_len);
                    diagnostics_.diagHexdump(mbxbuf, r_len, 256);
                }
#endif
                return false;
            }

            CoeHeader r_coe;
            std::memcpy(&r_coe, mbxbuf + sizeof(MbxHeader), sizeof(r_coe));
            const uint16_t r_coe_raw = le16_to_host(r_coe.raw_le);
            const uint16_t r_number = r_coe_raw & 0x01FFu;
            const uint8_t r_service = (r_coe_raw >> 12) & 0x0Fu;
            if (r_service != EC_COES_SDORES) {
                if (!logged_mbx_mismatch) {
                    logged_mbx_mismatch = true;
                    const char* svc_name = "Unknown";
                    switch (r_service) {
                        case 0x00: svc_name = "Emergency (0x0)"; break;
                        case 0x01: svc_name = "EMail (0x1)"; break;
                        case 0x02: svc_name = "SDO Request (0x2)"; break;
                        case 0x03: svc_name = "SDO Response (0x3)"; break;
                        case 0x04: svc_name = "TxPDO (0x4)"; break;
                        case 0x05: svc_name = "RxPDO (0x5)"; break;
                        case 0x06: svc_name = "TxPDO RR (0x6)"; break;
                        case 0x07: svc_name = "RxPDO RR (0x7)"; break;
                        default:   break;
                    }
                    char tmp[160];
                    snprintf(tmp, sizeof(tmp),
                             "Expected SDO Response (CoE service 0x3) but received %s "
                             "[ CoE header=0x%04X service=0x%X number=%u ]",
                             svc_name, r_coe_raw, r_service, r_number);
                    TETHER_LOGI(TAG, "%s", tmp);
#ifdef TETHER_DIAG_SDO_IO
                    if (diagEnabled) {
                        TETHER_LOGI(TAG, "CoE mismatch raw mbx (len=%u)", r_len);
                        diagnostics_.diagHexdump(mbxbuf, r_len, 256);
                    }
#endif
                }
                break;
            }
            if (hdr.cnt != expected_mbx_cnt) {
                if (!checkStaleCounter(master, adp, mbxWriteAddr, mbxWriteLen,
                                       mbxReadAddr, mbxReadLen, mbxbuf, pollIntervalMs,
                                       transactionTimeoutMs, hdr, expected_mbx_cnt,
                                       stale_retry_count, index, sub, "upload")) {
                    return false;
                }
                continue;
            }
            r_len = le16_to_host(reinterpret_cast<const MbxHeader*>(mbxbuf)->length_le);
            got_init_response = true;
            if (r_len < (sizeof(CoeHeader) + sizeof(SdoInitUploadRes))) {
                return false;
            }
            if (r_len >= sizeof(CoeHeader) + sizeof(SdoInitUploadRes)) {
                const auto *res = reinterpret_cast<const SdoInitUploadRes *>(r_sdo_bytes);
                const uint16_t r_index = le16_to_host(res->index_le);
                const uint8_t r_sub = res->sub;
                if (r_index != index || r_sub != sub) {
                    TETHER_LOGW(TAG, "Stale SDO response: idx=0x%04X:%u expected=0x%04X:%u (adp=0x%04X) — clearing and re-sending",
                                r_index, r_sub, index, sub, adp);
                    if (++stale_retry_count <= MAX_STALE_RETRIES) {
                        if (!sendAndWait(master, adp, mbxWriteAddr, mbxWriteLen,
                                         mbxReadAddr, mbxReadLen, mbxbuf, 500,
                                         pollIntervalMs, transactionTimeoutMs, "upload")) {
                            return false;
                        }
                    } else {
                        std::this_thread::sleep_for(std::chrono::milliseconds(10));
                    }
                    continue;
                }
            }
            break;
        }

        if (!got_init_response) {
            TETHER_LOGE(TAG, "SDO upload timeout: no mailbox response (adp=0x%04X wr=0x%04X rd=0x%04X index=0x%04X:%u)",
                        adp, mbxWriteAddr, mbxReadAddr, index, sub);
            diagnostics_.dumpSlaveState(master, adp, mbxWriteAddr, mbxReadAddr);
            return false;
        }

        const uint8_t ccs = (sdo_cmd >> 5) & 0x07u;
        const bool expedited = (sdo_cmd & 0x02u) != 0;
        const bool size_indicated = (sdo_cmd & 0x01u) != 0;

        if (ccs != 2) {
            return false;
        }

        if (expedited) {
            const auto *res = reinterpret_cast<const SdoInitUploadRes *>(r_sdo_bytes);
            const uint8_t n = (sdo_cmd >> 2) & 0x03u;
            const size_t data_bytes = 4u - n;
            const uint32_t v = le32_to_host(res->data_or_size_le);
#ifdef TETHER_DIAG_SDO_IO
            if (diagEnabled) {
                TETHER_LOGI(TAG, "SDO Upload MATCH: idx=0x%04x:%u cmd=0x%02x n=%u data_bytes=%zu v=0x%08x raw=%02x %02x %02x %02x",
                         le16_to_host(res->index_le), res->sub, sdo_cmd, n, data_bytes, v,
                         r_sdo_bytes[4], r_sdo_bytes[5], r_sdo_bytes[6], r_sdo_bytes[7]);
            }
#endif
            if (out && outCap > 0) {
                const size_t copy_n = (data_bytes < outCap) ? data_bytes : outCap;
                std::memcpy(out, &v, copy_n);
                if (outLen) {
                    *outLen = copy_n;
                }
            }
            diagnostics_.logCoeMbxPacket("RX", adp, index, sub, mbxbuf, mbxReadLen,
                                         master.debugFlags().coeRxPackets && master.debugFlags().coeRxPacketsFilt.allows(slaveIndexFromADP(adp)));
            if (master.debugGate().hasAnyConditions()) {
                master.debugGate().onCoERead(slaveIndexFromADP(adp), index, sub,
                                             out, outLen ? *outLen : 0);
            }
            return true;
        }

        uint32_t total_size = 0;
        if (size_indicated && r_len >= sizeof(CoeHeader) + sizeof(SdoInitUploadRes)) {
            const auto *res = reinterpret_cast<const SdoInitUploadRes *>(r_sdo_bytes);
            total_size = le32_to_host(res->data_or_size_le);
        }

        bool toggle = false;
        size_t produced = 0;
        for (int seg = 0; seg < ECAT_SDO_UPLOAD_MAX_SEGMENTS; seg++) {
            MbxHeader seg_mbx{};
            seg_mbx.length_le = host_to_le16(static_cast<uint16_t>(sizeof(CoeHeader) + 8));
            seg_mbx.address_le = host_to_le16(0);
            seg_mbx.priority = 0;
            seg_mbx.mbxtype = mbx_type_with_cnt(EC_MBXT_COE, mbx_cnt);

            CoeHeader seg_coe{};
            seg_coe.raw_le = host_to_le16(coe_make_raw(0, EC_COES_SDOREQ));

            const uint8_t seg_req_cmd = static_cast<uint8_t>(EC_SDO_SEG_UP_REQ | (toggle ? 0x10u : 0x00u));

            std::memset(mbxbuf, 0, sizeof(mbxbuf));
            std::memcpy(mbxbuf, &seg_mbx, sizeof(seg_mbx));
            std::memcpy(mbxbuf + sizeof(seg_mbx), &seg_coe, sizeof(seg_coe));
            mbxbuf[sizeof(seg_mbx) + sizeof(seg_coe) + 0] = seg_req_cmd;

            mbx_cnt = static_cast<uint8_t>((mbx_cnt >= 7) ? 1 : (mbx_cnt + 1));
            if (inoutMbxCnt != nullptr) {
                *inoutMbxCnt = mbx_cnt;
            }

            {
                bool used_alt = false;
                if (!mailboxIO_.apwrWithWkcProbe(master, adp,
                                                  mbxWriteAddr,
                                                  mbxReadAddr,
                                                  mbxbuf,
                                                  static_cast<uint16_t>(mbxWriteLen),
                                                  500,
                                                  &used_alt)) {
                    return false;
                }
                if (used_alt) {
                    std::swap(mbxWriteAddr, mbxReadAddr);
                    std::swap(mbxWriteLen, mbxReadLen);
                }
            }

            if (!mailboxIO_.pollSm1Full(master, adp, transactionTimeoutMs, pollIntervalMs)) {
                TETHER_LOGE(TAG, "SDO upload segment: SM1 mailbox never became full (adp=0x%04X wr=0x%04X rd=0x%04X index=0x%04X:%u timeout=%ums)",
                            adp, mbxWriteAddr, mbxReadAddr, index, sub, transactionTimeoutMs);
                diagnostics_.dumpSlaveState(master, adp, mbxWriteAddr, mbxReadAddr);
                return false;
            }

            bool got = false;
            for (int attempt2 = 0; attempt2 < 50; attempt2++) {
                if (master.isCancelRequested()) {
                    TETHER_LOGW(TAG, "SDO upload segment cancelled");
                    return false;
                }
                if (!master.readRegister(Master::slaveAddressFromADP(adp), mbxReadAddr, mbxbuf, static_cast<uint16_t>(mbxReadLen), 200)) {
                    std::this_thread::sleep_for(std::chrono::milliseconds(5));
                    continue;
                }
                MbxHeader r2_mbx;
                std::memcpy(&r2_mbx, mbxbuf, sizeof(r2_mbx));
                const uint16_t r2_len = le16_to_host(r2_mbx.length_le);
                const uint8_t r2_type = static_cast<uint8_t>(r2_mbx.mbxtype & 0x0Fu);
                if (r2_len < sizeof(CoeHeader) + 8 || r2_type != EC_MBXT_COE) {
                    std::this_thread::sleep_for(std::chrono::milliseconds(5));
                    continue;
                }
                const uint8_t *seg_res = mbxbuf + sizeof(MbxHeader) + sizeof(CoeHeader);
                const uint8_t seg_cmd = seg_res[0];

                // Check for SDO abort before the CoE service field — some slaves
                // (e.g. ESC211) emit abort responses with a buggy CoE service
                // field (0x2/SDO-REQ instead of 0x3/SDO-RES). Surface the abort
                // code regardless of the CoE service so callers see the real
                // rejection instead of a misleading timeout.
                if ((seg_cmd & 0xE0u) == EC_SDO_ABORT) {
                    if (mbxReadLen >= sizeof(MbxHeader) + sizeof(CoeHeader) + sizeof(SdoAbort)) {
                        SdoAbort abort{};
                        std::memcpy(&abort, seg_res, sizeof(abort));
                        const uint32_t abort_code = le32_to_host(abort.abortCode_le);
                        TETHER_LOGE(TAG, "SDO upload segment abort: index=0x%04x:%02x code=0x%08" PRIx32 " (%s)",
                                 index, sub, abort_code, errorDecoder_.sdoAbortCodeStr(abort_code));
                        if (outAbortCode) *outAbortCode = abort_code;
                    } else {
                        TETHER_LOGE(TAG, "SDO upload segment abort (malformed response)");
                    }
                    return false;
                }

                const auto *r2_coe = reinterpret_cast<const CoeHeader *>(mbxbuf + sizeof(MbxHeader));
                const uint16_t r2_coe_raw = le16_to_host(r2_coe->raw_le);
                const uint8_t r2_service = (r2_coe_raw >> 12) & 0x0Fu;
                if (r2_service != EC_COES_SDORES) {
                    std::this_thread::sleep_for(std::chrono::milliseconds(5));
                    continue;
                }
                const uint8_t seg_ccs = (seg_cmd >> 5) & 0x07u;
                if (seg_ccs != 0) {
                    std::this_thread::sleep_for(std::chrono::milliseconds(5));
                    continue;
                }
                const bool seg_toggle = (seg_cmd & 0x10u) != 0;
                if (seg_toggle != toggle) {
                    std::this_thread::sleep_for(std::chrono::milliseconds(5));
                    continue;
                }
                const uint8_t n = (seg_cmd >> 1) & 0x07u;
                const bool last = (seg_cmd & 0x01u) != 0;
                const size_t seg_data_bytes = 7u - n;
                if (out && outCap > produced) {
                    const size_t copy_n = (seg_data_bytes < (outCap - produced)) ? seg_data_bytes : (outCap - produced);
                    std::memcpy(out + produced, seg_res + 1, copy_n);
                }
                produced += seg_data_bytes;
                if (last) {
                    if (outLen) {
                        *outLen = produced;
                    }
                    if (total_size != 0 && produced > total_size && outLen) {
                        *outLen = total_size;
                    }
                    diagnostics_.logCoeMbxPacket("RX", adp, index, sub, mbxbuf, mbxReadLen,
                            master.debugFlags().coeRxPackets && master.debugFlags().coeRxPacketsFilt.allows(slaveIndexFromADP(adp)));
                    if (master.debugGate().hasAnyConditions()) {
                        master.debugGate().onCoERead(slaveIndexFromADP(adp), index, sub,
                                                     out, outLen ? *outLen : 0);
                    }
                    return true;
                }
                toggle = !toggle;
                got = true;
                break;
            }
            if (!got) {
                TETHER_LOGE(TAG, "SDO upload segment timeout (adp=0x%04X wr=0x%04X rd=0x%04X index=0x%04X:%u)",
                            adp, mbxWriteAddr, mbxReadAddr, index, sub);
                diagnostics_.dumpSlaveState(master, adp, mbxWriteAddr, mbxReadAddr);
                return false;
            }
        }
        return false;
    }

    return false;
}

} // namespace Raw
} // namespace EtherCAT

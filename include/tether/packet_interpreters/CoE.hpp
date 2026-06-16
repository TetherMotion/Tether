/**
 * @file CoE.hpp
 * @brief CoE (CANopen over EtherCAT) packet interpreter
 *
 * Thin diagnostic wrapper that turns raw EtherCAT datagram payloads
 * into human-readable strings.  All parsing is done with inline
 * packed structs that mirror the definitions in
 * src/ethercat/raw/internal.hpp so that this public header stays
 * self-contained.
 */

#pragma once

#include <cstdint>
#include <cstdio>
#include <cstring>
#include <string>

namespace EtherCAT {
namespace PacketInterpreters {

// ---------------------------------------------------------------------------
// Minimal inline wire-format definitions (mirrors internal.hpp)
// ---------------------------------------------------------------------------

namespace {

// Mailbox types
constexpr uint8_t EC_MBXT_COE = 0x03;

// CoE service types
constexpr uint8_t EC_COES_SDOREQ = 0x02;
constexpr uint8_t EC_COES_SDORES = 0x03;

// SDO command specifiers
constexpr uint8_t EC_SDO_DOWN_REQ     = 0x20;
constexpr uint8_t EC_SDO_DOWN_SEG_REQ = 0x00;
constexpr uint8_t EC_SDO_UP_REQ     = 0x40;
constexpr uint8_t EC_SDO_SEG_UP_REQ = 0x60;
constexpr uint8_t EC_SDO_ABORT      = 0x80;

struct __attribute__((packed)) MbxHeader {
    uint16_t length_le;
    uint16_t address_le;
    uint8_t  priority;
    uint8_t  mbxtype;
};

struct __attribute__((packed)) CoeHeader {
    uint16_t raw_le;
};

struct __attribute__((packed)) SdoInitUploadReq {
    uint8_t  cmd;
    uint16_t index_le;
    uint8_t  sub;
    uint8_t  reserved[4];
};

struct __attribute__((packed)) SdoInitUploadRes {
    uint8_t  cmd;
    uint16_t index_le;
    uint8_t  sub;
    uint32_t data_or_size_le;
};

struct __attribute__((packed)) SdoInitDownloadReq {
    uint8_t  cmd;
    uint16_t index_le;
    uint8_t  sub;
    uint32_t data_le;
};

struct __attribute__((packed)) SdoInitDownloadRes {
    uint8_t  cmd;
    uint16_t index_le;
    uint8_t  sub;
    uint8_t  reserved[4];
};

struct __attribute__((packed)) SdoAbort {
    uint8_t  cmd;
    uint16_t index_le;
    uint8_t  sub;
    uint32_t abortCode_le;
};

// Host is little-endian (ESP32 / typical Linux x86_64)
static inline uint16_t le16_to_host(uint16_t le) { return le; }
static inline uint32_t le32_to_host(uint32_t le) { return le; }

} // anonymous namespace

// ---------------------------------------------------------------------------
// CoEPacketInterpreter
// ---------------------------------------------------------------------------

class CoEPacketInterpreter {
public:
    explicit CoEPacketInterpreter(const uint8_t* payload, size_t len)
        : data_(payload), len_(len) {}

    std::string toString() const;

private:
    const uint8_t* data_;
    size_t len_;

    static const char* mbxTypeName(uint8_t type);
    static const char* coeServiceName(uint8_t service);
    static const char* sdoCmdName(uint8_t cmd, uint8_t coeService);
};

// ---------------------------------------------------------------------------
// String helpers
// ---------------------------------------------------------------------------

inline const char* CoEPacketInterpreter::mbxTypeName(uint8_t type) {
    switch (type) {
        case 0x00: return "ERR";
        case 0x01: return "AOE";
        case 0x02: return "EOE";
        case 0x03: return "CoE";
        case 0x04: return "FoE";
        case 0x05: return "SoE";
        case 0x0F: return "VoE";
        default:   return "UNKNOWN";
    }
}

inline const char* CoEPacketInterpreter::coeServiceName(uint8_t service) {
    switch (service) {
        case 0x01: return "EMERGENCY";
        case 0x02: return "SDO-REQ";
        case 0x03: return "SDO-RES";
        case 0x04: return "TX-PDO";
        case 0x05: return "RX-PDO";
        case 0x06: return "TX-PDO-REMOTE";
        case 0x07: return "RX-PDO-REMOTE";
        case 0x08: return "SDO-INFO";
        default:   return "UNKNOWN";
    }
}

inline const char* CoEPacketInterpreter::sdoCmdName(uint8_t cmd, uint8_t coeService) {
    uint8_t ccs = (cmd >> 5) & 0x07;
    if (coeService == EC_COES_SDOREQ) {
        switch (ccs) {
            case 0: return "DOWNLOAD-SEG-REQ";
            case 1: return "DOWNLOAD-INIT-REQ";
            case 2: return "UPLOAD-INIT-REQ";
            case 3: return "UPLOAD-SEG-REQ";
            case 4: return "ABORT";
        }
    } else if (coeService == EC_COES_SDORES) {
        switch (ccs) {
            case 0: return "UPLOAD-SEG-RES";
            case 1: return "DOWNLOAD-SEG-RES";
            case 2: return "UPLOAD-INIT-RES";
            case 3: return "DOWNLOAD-INIT-RES";
            case 4: return "ABORT";
        }
    }
    return "UNKNOWN";
}

// ---------------------------------------------------------------------------
// toString implementation
// ---------------------------------------------------------------------------

inline std::string CoEPacketInterpreter::toString() const {
    if (len_ < sizeof(MbxHeader)) {
        char buf[64];
        std::snprintf(buf, sizeof(buf), "Truncated packet (len=%zu, need >=%zu)", len_, sizeof(MbxHeader));
        return std::string(buf);
    }

    MbxHeader mbx;
    std::memcpy(&mbx, data_, sizeof(mbx));
    const uint16_t mbx_len   = le16_to_host(mbx.length_le);
    const uint16_t mbx_addr  = le16_to_host(mbx.address_le);
    const uint8_t  mbx_ch    = mbx.priority & 0x3F;        // byte 4: channel[6] | priority[2]
    const uint8_t  mbx_prio  = (mbx.priority >> 6) & 0x03;
    const uint8_t  mbx_type  = mbx.mbxtype & 0x0F;         // byte 5: type[4] | counter[4]
    const uint8_t  mbx_cnt   = (mbx.mbxtype >> 4) & 0x0F;

    std::string result;
    char line[256];

    std::snprintf(line, sizeof(line),
                  "Mailbox: len=%u type=%s(0x%02X) cnt=0x%02X addr=0x%04X ch=%u prio=%u\n",
                  mbx_len, mbxTypeName(mbx_type), mbx_type, mbx_cnt, mbx_addr, mbx_ch, mbx_prio);
    result += line;

    if (mbx_type != EC_MBXT_COE) {
        std::snprintf(line, sizeof(line), "(Not a CoE packet)\n");
        result += line;
        return result;
    }

    if (len_ < sizeof(MbxHeader) + sizeof(CoeHeader)) {
        std::snprintf(line, sizeof(line), "Truncated CoE packet (missing CoE header)\n");
        result += line;
        return result;
    }

    CoeHeader coe;
    std::memcpy(&coe, data_ + sizeof(MbxHeader), sizeof(coe));
    const uint16_t coe_raw = le16_to_host(coe.raw_le);
    const uint16_t coe_num = coe_raw & 0x01FFu;
    const uint8_t  coe_svc = (coe_raw >> 12) & 0x0Fu;

    std::snprintf(line, sizeof(line),
                  "CoE:     service=%s(0x%02X) number=0x%03X\n",
                  coeServiceName(coe_svc), coe_svc, coe_num);
    result += line;

    const size_t sdo_offset = sizeof(MbxHeader) + sizeof(CoeHeader);
    const size_t sdo_avail  = (len_ > sdo_offset) ? (len_ - sdo_offset) : 0;

    if (sdo_avail < 1) {
        std::snprintf(line, sizeof(line), "(No SDO data)\n");
        result += line;
        return result;
    }

    const uint8_t sdo_cmd = data_[sdo_offset];
    const uint8_t ccs     = (sdo_cmd >> 5) & 0x07;
    const bool    e       = (sdo_cmd & 0x02) != 0;
    const bool    s       = (sdo_cmd & 0x01) != 0;
    const uint8_t n       = (sdo_cmd >> 2) & 0x03;

    if (coe_svc == EC_COES_SDOREQ || coe_svc == EC_COES_SDORES) {
        std::snprintf(line, sizeof(line),
                      "SDO:     cmd=%s(0x%02X) ccs=%u", sdoCmdName(sdo_cmd, coe_svc), sdo_cmd, ccs);
        result += line;

        // Abort has no e/s/n, just index/sub/code
        if (ccs == 4) {
            if (sdo_avail >= sizeof(SdoAbort)) {
                SdoAbort ab;
                std::memcpy(&ab, data_ + sdo_offset, sizeof(ab));
                std::snprintf(line, sizeof(line),
                              " index=0x%04X sub=0x%02X code=0x%08X\n",
                              le16_to_host(ab.index_le), ab.sub, le32_to_host(ab.abortCode_le));
                result += line;
            } else {
                result += " (truncated abort)\n";
            }
            return result;
        }

        std::snprintf(line, sizeof(line), " e=%d s=%d n=%u", e ? 1 : 0, s ? 1 : 0, n);
        result += line;

        // Initiate transfers carry index + subindex
        if ((coe_svc == EC_COES_SDOREQ && (ccs == 1 || ccs == 2)) ||
            (coe_svc == EC_COES_SDORES && (ccs == 2 || ccs == 3))) {
            if (sdo_avail >= 4) {
                uint16_t idx = le16_to_host(*reinterpret_cast<const uint16_t*>(data_ + sdo_offset + 1));
                uint8_t  sub = data_[sdo_offset + 3];
                std::snprintf(line, sizeof(line), " index=0x%04X sub=0x%02X", idx, sub);
                result += line;
            }

            // Download init request: show data if expedited
            if (coe_svc == EC_COES_SDOREQ && ccs == 1 && e && sdo_avail >= sizeof(SdoInitDownloadReq)) {
                SdoInitDownloadReq req;
                std::memcpy(&req, data_ + sdo_offset, sizeof(req));
                std::snprintf(line, sizeof(line), " data=0x%08X", le32_to_host(req.data_le));
                result += line;
            }

            // Upload init response: show data if expedited, else show size
            if (coe_svc == EC_COES_SDORES && ccs == 2 && sdo_avail >= sizeof(SdoInitUploadRes)) {
                SdoInitUploadRes res;
                std::memcpy(&res, data_ + sdo_offset, sizeof(res));
                if (e) {
                    const size_t data_bytes = 4 - n;
                    std::snprintf(line, sizeof(line), " data=0x%08X (%zu bytes)",
                                  le32_to_host(res.data_or_size_le), data_bytes);
                    result += line;
                } else if (s) {
                    std::snprintf(line, sizeof(line), " size=%u", le32_to_host(res.data_or_size_le));
                    result += line;
                }
            }
        }

        // Segmented transfers
        if ((coe_svc == EC_COES_SDOREQ && (ccs == 0 || ccs == 3)) ||
            (coe_svc == EC_COES_SDORES && (ccs == 0 || ccs == 1))) {
            const bool toggle = (sdo_cmd & 0x10) != 0;
            const bool last   = (sdo_cmd & 0x01) != 0;
            const uint8_t seg_n = (sdo_cmd >> 1) & 0x07;
            const size_t seg_bytes = 7 - seg_n;
            std::snprintf(line, sizeof(line), " toggle=%d last=%d seg_bytes=%zu",
                          toggle ? 1 : 0, last ? 1 : 0, seg_bytes);
            result += line;
        }

        result += "\n";
    } else {
        std::snprintf(line, sizeof(line), "(Non-SDO CoE service, no further decoder)\n");
        result += line;
    }

    return result;
}

} // namespace PacketInterpreters
} // namespace EtherCAT

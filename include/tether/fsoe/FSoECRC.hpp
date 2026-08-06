#pragma once

#include <cstdint>
#include <cstddef>

namespace FSoE::CRC {

// ETG.5100 §8.1.3.2 specifies the safety CRC polynomial as 0x139B7
// (normal/MSB-first form; the implicit leading 1 represents x^16).
// The 16-bit polynomial value is 0x39B7.
// The CRC is non-reflected (refIn=false, refOut=false), init=0x0000, no final XOR.
inline constexpr uint16_t kPolynomial = 0x39B7;
inline constexpr uint16_t kInitValue  = 0x0000;

inline constexpr uint16_t crcTable[256] = {
    0x0000, 0x39B7, 0x736E, 0x4AD9, 0xE6DC, 0xDF6B, 0x95B2, 0xAC05,
    0xF40F, 0xCDB8, 0x8761, 0xBED6, 0x12D3, 0x2B64, 0x61BD, 0x580A,
    0xD1A9, 0xE81E, 0xA2C7, 0x9B70, 0x3775, 0x0EC2, 0x441B, 0x7DAC,
    0x25A6, 0x1C11, 0x56C8, 0x6F7F, 0xC37A, 0xFACD, 0xB014, 0x89A3,
    0x9AE5, 0xA352, 0xE98B, 0xD03C, 0x7C39, 0x458E, 0x0F57, 0x36E0,
    0x6EEA, 0x575D, 0x1D84, 0x2433, 0x8836, 0xB181, 0xFB58, 0xC2EF,
    0x4B4C, 0x72FB, 0x3822, 0x0195, 0xAD90, 0x9427, 0xDEFE, 0xE749,
    0xBF43, 0x86F4, 0xCC2D, 0xF59A, 0x599F, 0x6028, 0x2AF1, 0x1346,
    0x0C7D, 0x35CA, 0x7F13, 0x46A4, 0xEAA1, 0xD316, 0x99CF, 0xA078,
    0xF872, 0xC1C5, 0x8B1C, 0xB2AB, 0x1EAE, 0x2719, 0x6DC0, 0x5477,
    0xDDD4, 0xE463, 0xAEBA, 0x970D, 0x3B08, 0x02BF, 0x4866, 0x71D1,
    0x29DB, 0x106C, 0x5AB5, 0x6302, 0xCF07, 0xF6B0, 0xBC69, 0x85DE,
    0x9698, 0xAF2F, 0xE5F6, 0xDC41, 0x7044, 0x49F3, 0x032A, 0x3A9D,
    0x6297, 0x5B20, 0x11F9, 0x284E, 0x844B, 0xBDFC, 0xF725, 0xCE92,
    0x4731, 0x7E86, 0x345F, 0x0DE8, 0xA1ED, 0x985A, 0xD283, 0xEB34,
    0xB33E, 0x8A89, 0xC050, 0xF9E7, 0x55E2, 0x6C55, 0x268C, 0x1F3B,
    0x18FA, 0x214D, 0x6B94, 0x5223, 0xFE26, 0xC791, 0x8D48, 0xB4FF,
    0xECF5, 0xD542, 0x9F9B, 0xA62C, 0x0A29, 0x339E, 0x7947, 0x40F0,
    0xC953, 0xF0E4, 0xBA3D, 0x838A, 0x2F8F, 0x1638, 0x5CE1, 0x6556,
    0x3D5C, 0x04EB, 0x4E32, 0x7785, 0xDB80, 0xE237, 0xA8EE, 0x9159,
    0x821F, 0xBBA8, 0xF171, 0xC8C6, 0x64C3, 0x5D74, 0x17AD, 0x2E1A,
    0x7610, 0x4FA7, 0x057E, 0x3CC9, 0x90CC, 0xA97B, 0xE3A2, 0xDA15,
    0x53B6, 0x6A01, 0x20D8, 0x196F, 0xB56A, 0x8CDD, 0xC604, 0xFFB3,
    0xA7B9, 0x9E0E, 0xD4D7, 0xED60, 0x4165, 0x78D2, 0x320B, 0x0BBC,
    0x1487, 0x2D30, 0x67E9, 0x5E5E, 0xF25B, 0xCBEC, 0x8135, 0xB882,
    0xE088, 0xD93F, 0x93E6, 0xAA51, 0x0654, 0x3FE3, 0x753A, 0x4C8D,
    0xC52E, 0xFC99, 0xB640, 0x8FF7, 0x23F2, 0x1A45, 0x509C, 0x692B,
    0x3121, 0x0896, 0x424F, 0x7BF8, 0xD7FD, 0xEE4A, 0xA493, 0x9D24,
    0x8E62, 0xB7D5, 0xFD0C, 0xC4BB, 0x68BE, 0x5109, 0x1BD0, 0x2267,
    0x7A6D, 0x43DA, 0x0903, 0x30B4, 0x9CB1, 0xA506, 0xEFDF, 0xD668,
    0x5FCB, 0x667C, 0x2CA5, 0x1512, 0xB917, 0x80A0, 0xCA79, 0xF3CE,
    0xABC4, 0x9273, 0xD8AA, 0xE11D, 0x4D18, 0x74AF, 0x3E76, 0x07C1
};

inline uint16_t calculate(const uint8_t* data, size_t len, uint16_t init_crc = kInitValue) {
    if (!data || len == 0) return init_crc;
    uint16_t crc = init_crc;
    for (size_t i = 0; i < len; i++) {
        crc = (crc << 8) ^ crcTable[(crc >> 8) ^ data[i]];
    }
    return crc;
}

inline bool verify(const uint8_t* data, size_t len) {
    if (len < 2) return false;
    uint16_t calculated = calculate(data, len - 2);
    uint16_t stored = static_cast<uint16_t>(data[len - 2]) |
                      (static_cast<uint16_t>(data[len - 1]) << 8);
    return calculated == stored;
}

inline bool verifyFSoECRC(const uint8_t* data, size_t len, uint16_t expected_crc) {
    return calculate(data, len) == expected_crc;
}

inline constexpr size_t MIN_FSOE_FRAME_SIZE = 3;

inline constexpr size_t fsoeFrameSize(size_t data_len) {
    size_t chunks = (data_len + 1) / 2;
    return 1 + chunks * 4 + 2;
}

inline constexpr size_t fsoeDataLen(size_t frame_size) {
    if (frame_size < MIN_FSOE_FRAME_SIZE) return 0;
    size_t remaining = frame_size - 1 - 2;
    size_t chunks = remaining / 4;
    return chunks * 2;
}

inline size_t buildFSoEFrame(uint8_t* out, uint8_t cmd,
                              const uint8_t* data, size_t data_len,
                              uint16_t conn_id) {
    size_t chunks = (data_len + 1) / 2;
    size_t frame_size = fsoeFrameSize(data_len);

    out[0] = cmd;

    size_t offset = 1;
    for (size_t i = 0; i < chunks; i++) {
        uint8_t chunk[2] = {0, 0};
        size_t chunk_start = i * 2;
        chunk[0] = (chunk_start < data_len) ? data[chunk_start] : 0;
        chunk[1] = (chunk_start + 1 < data_len) ? data[chunk_start + 1] : 0;

        out[offset] = chunk[0];
        out[offset + 1] = chunk[1];
        uint16_t crc = calculate(chunk, 2);
        out[offset + 2] = crc & 0xFF;
        out[offset + 3] = (crc >> 8) & 0xFF;
        offset += 4;
    }

    out[offset] = conn_id & 0xFF;
    out[offset + 1] = (conn_id >> 8) & 0xFF;

    return frame_size;
}

inline bool parseFSoEFrame(const uint8_t* frame, size_t frame_len,
                            uint8_t& out_cmd,
                            uint8_t* out_data, size_t& out_data_len,
                            uint16_t& out_conn_id) {
    if (frame_len < MIN_FSOE_FRAME_SIZE) return false;

    out_cmd = frame[0];

    size_t remaining = frame_len - 1 - 2;
    size_t chunks = remaining / 4;
    out_data_len = chunks * 2;

    size_t offset = 1;
    for (size_t i = 0; i < chunks; i++) {
        uint8_t chunk[2] = {frame[offset], frame[offset + 1]};
        uint16_t stored_crc = static_cast<uint16_t>(frame[offset + 2]) |
                              (static_cast<uint16_t>(frame[offset + 3]) << 8);
        uint16_t calc_crc = calculate(chunk, 2);
        if (stored_crc != calc_crc) {
            return false;
        }
        if (out_data) {
            out_data[i * 2] = chunk[0];
            out_data[i * 2 + 1] = chunk[1];
        }
        offset += 4;
    }

    out_conn_id = static_cast<uint16_t>(frame[offset]) |
                  (static_cast<uint16_t>(frame[offset + 1]) << 8);
    return true;
}

} // namespace FSoE::CRC

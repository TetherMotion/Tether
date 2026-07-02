#pragma once

#include <cstdint>
#include <cstddef>

namespace FSoE::CRC {

inline constexpr uint16_t kPolynomial = 0x39B7;
inline constexpr uint16_t kInitValue  = 0x0000;

inline constexpr uint16_t crcTable[256] = {
    0x0000, 0x22CF, 0x36F1, 0x143E, 0x1E8D, 0x3C42, 0x287C, 0x0AB3,
    0x3D1A, 0x1FD5, 0x0BEB, 0x2924, 0x2397, 0x0158, 0x1566, 0x37A9,
    0x095B, 0x2B94, 0x3FAA, 0x1D65, 0x17D6, 0x3519, 0x2127, 0x03E8,
    0x3441, 0x168E, 0x02B0, 0x207F, 0x2ACC, 0x0803, 0x1C3D, 0x3EF2,
    0x12B6, 0x3079, 0x2447, 0x0688, 0x0C3B, 0x2EF4, 0x3ACA, 0x1805,
    0x2FAC, 0x0D63, 0x195D, 0x3B92, 0x3121, 0x13EE, 0x07D0, 0x251F,
    0x1BED, 0x3922, 0x2D1C, 0x0FD3, 0x0560, 0x27AF, 0x3391, 0x115E,
    0x26F7, 0x0438, 0x1006, 0x32C9, 0x387A, 0x1AB5, 0x0E8B, 0x2C44,
    0x256C, 0x07A3, 0x139D, 0x3152, 0x3BE1, 0x192E, 0x0D10, 0x2FDF,
    0x1876, 0x3AB9, 0x2E87, 0x0C48, 0x06FB, 0x2434, 0x300A, 0x12C5,
    0x2C37, 0x0EF8, 0x1AC6, 0x3809, 0x32BA, 0x1075, 0x044B, 0x2684,
    0x112D, 0x33E2, 0x27DC, 0x0513, 0x0FA0, 0x2D6F, 0x3951, 0x1B9E,
    0x37DA, 0x1515, 0x012B, 0x23E4, 0x2957, 0x0B98, 0x1FA6, 0x3D69,
    0x0AC0, 0x280F, 0x3C31, 0x1EFE, 0x144D, 0x3682, 0x22BC, 0x0073,
    0x3E81, 0x1C4E, 0x0870, 0x2ABF, 0x200C, 0x02C3, 0x16FD, 0x3432,
    0x039B, 0x2154, 0x356A, 0x17A5, 0x1D16, 0x3FD9, 0x2BE7, 0x0928,
    0x39B7, 0x1B78, 0x0F46, 0x2D89, 0x273A, 0x05F5, 0x11CB, 0x3304,
    0x04AD, 0x2662, 0x325C, 0x1093, 0x1A20, 0x38EF, 0x2CD1, 0x0E1E,
    0x30EC, 0x1223, 0x061D, 0x24D2, 0x2E61, 0x0CAE, 0x1890, 0x3A5F,
    0x0DF6, 0x2F39, 0x3B07, 0x19C8, 0x137B, 0x31B4, 0x258A, 0x0745,
    0x2B01, 0x09CE, 0x1DF0, 0x3F3F, 0x358C, 0x1743, 0x037D, 0x21B2,
    0x161B, 0x34D4, 0x20EA, 0x0225, 0x0896, 0x2A59, 0x3E67, 0x1CA8,
    0x225A, 0x0095, 0x14AB, 0x3664, 0x3CD7, 0x1E18, 0x0A26, 0x28E9,
    0x1F40, 0x3D8F, 0x29B1, 0x0B7E, 0x01CD, 0x2302, 0x373C, 0x15F3,
    0x1CDB, 0x3E14, 0x2A2A, 0x08E5, 0x0256, 0x2099, 0x34A7, 0x1668,
    0x21C1, 0x030E, 0x1730, 0x35FF, 0x3F4C, 0x1D83, 0x09BD, 0x2B72,
    0x1580, 0x374F, 0x2371, 0x01BE, 0x0B0D, 0x29C2, 0x3DFC, 0x1F33,
    0x289A, 0x0A55, 0x1E6B, 0x3CA4, 0x3617, 0x14D8, 0x00E6, 0x2229,
    0x0E6D, 0x2CA2, 0x389C, 0x1A53, 0x10E0, 0x322F, 0x2611, 0x04DE,
    0x3377, 0x11B8, 0x0586, 0x2749, 0x2DFA, 0x0F35, 0x1B0B, 0x39C4,
    0x0736, 0x25F9, 0x31C7, 0x1308, 0x19BB, 0x3B74, 0x2F4A, 0x0D85,
    0x3A2C, 0x18E3, 0x0CDD, 0x2E12, 0x24A1, 0x066E, 0x1250, 0x309F
};

inline uint16_t calculate(const uint8_t* data, size_t len, uint16_t init_crc = kInitValue) {
    if (!data || len == 0) return init_crc;
    uint16_t crc = init_crc;
    for (size_t i = 0; i < len; i++) {
        uint8_t index = (crc ^ data[i]) & 0xFF;
        crc = (crc >> 8) ^ crcTable[index];
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

#pragma once

#include <cstddef>
#include <cstdint>
#include <vector>

namespace EtherCAT {
namespace Utils {

/// Reflected CRC-16, polynomial 0x8005 (reflected: 0xA001), no final XOR.
/// (CRC-16/BUYPASS family — e.g. the NexSPGR.dll per-word checksum.)
///
/// Incremental: feed the returned value back as `crc` to continue over
/// more data.
inline uint16_t crc16Update(uint16_t crc, const uint8_t* data, size_t length) {
    constexpr uint16_t kPoly = 0xA001;  // reflected polynomial of 0x8005
    for (size_t i = 0; i < length; ++i) {
        crc ^= data[i];
        for (int bit = 0; bit < 8; ++bit) {
            crc = (crc & 0x0001u) ? static_cast<uint16_t>((crc >> 1) ^ kPoly)
                                  : static_cast<uint16_t>(crc >> 1);
        }
    }
    return crc;
}

/// Reflected CRC-16 over a byte range (init = 0x0000 unless given).
inline uint16_t crc16(const uint8_t* data, size_t length,
                      uint16_t init = 0x0000) {
    return crc16Update(init, data, length);
}

inline uint16_t crc16(const std::vector<uint8_t>& data,
                      uint16_t init = 0x0000) {
    return crc16(data.data(), data.size(), init);
}

/// IEEE 802.3 CRC-32 (reflected, poly 0x04C11DB7, init 0xFFFFFFFF,
/// final XOR 0xFFFFFFFF) — compatible with zlib.crc32().
inline uint32_t crc32(const uint8_t* data, size_t length) {
    constexpr uint32_t kPoly = 0xEDB88320u;
    uint32_t crc = 0xFFFFFFFFu;
    for (size_t i = 0; i < length; ++i) {
        crc ^= data[i];
        for (int bit = 0; bit < 8; ++bit) {
            crc = (crc & 1u) ? (crc >> 1) ^ kPoly : (crc >> 1);
        }
    }
    return ~crc;
}

/// CRC-32 over `data`, extended with zero padding up to `bufSize` bytes.
/// When `length` exceeds `bufSize` the extra bytes are ignored (the CRC
/// covers exactly bufSize bytes).
inline uint32_t crc32Padded(const uint8_t* data, size_t length,
                            size_t bufSize) {
    constexpr uint32_t kPoly = 0xEDB88320u;
    uint32_t crc = 0xFFFFFFFFu;
    const size_t effectiveLen = (length < bufSize) ? length : bufSize;
    for (size_t i = 0; i < bufSize; ++i) {
        crc ^= (i < effectiveLen) ? data[i] : 0;
        for (int bit = 0; bit < 8; ++bit) {
            crc = (crc & 1u) ? (crc >> 1) ^ kPoly : (crc >> 1);
        }
    }
    return ~crc;
}

inline uint32_t crc32(const std::vector<uint8_t>& data) {
    return crc32(data.data(), data.size());
}

inline uint32_t crc32Padded(const std::vector<uint8_t>& data,
                            size_t bufSize) {
    return crc32Padded(data.data(), data.size(), bufSize);
}

} // namespace Utils
} // namespace EtherCAT

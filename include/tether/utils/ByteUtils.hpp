#pragma once

#include <cstdint>
#include <cstdio>
#include <string>

namespace EtherCAT {
namespace Utils {

/// Convert a byte buffer to an uppercase hex string with a space separator.
inline std::string bytesToHex(const uint8_t* data, size_t len) {
    std::string s;
    s.reserve(len * 3);
    for (size_t i = 0; i < len; ++i) {
        char buf[4];
        std::snprintf(buf, sizeof(buf), "%02X ", data[i]);
        s += buf;
    }
    if (!s.empty()) s.pop_back();  // trailing space
    return s;
}

/// Return true if all bytes in [data, data + len) are zero.
inline bool isAllZero(const uint8_t* data, size_t len) {
    for (size_t i = 0; i < len; ++i) {
        if (data[i] != 0) return false;
    }
    return true;
}

} // namespace Utils
} // namespace EtherCAT

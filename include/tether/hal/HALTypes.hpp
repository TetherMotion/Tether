/**
 * @file HALTypes.hpp
 * @brief Common type definitions for the Hardware Abstraction Layer
 *
 * This header defines types and constants used across all HAL components.
 */

#pragma once

#include <cstdint>
#include <cstddef>
#include <cstring>
#include <array>
#include <magic_enum/magic_enum.hpp>

namespace EtherCAT {
namespace HAL {

// ============================================================================
// Platform Detection
// ============================================================================

#if defined(ESP_PLATFORM) || defined(ESP32)
    #define HAL_PLATFORM_ESP32 1
#elif defined(__linux__)
    #define HAL_PLATFORM_LINUX 1
#elif defined(STM32F4) || defined(STM32F7) || defined(STM32H7) || defined(STM32_HAL)
    #define HAL_PLATFORM_STM32 1
#endif

#if defined(UNIT_TEST_HOST)
    #define HAL_PLATFORM_TEST 1
#endif

// ============================================================================
// Ethernet Constants
// ============================================================================

/// Maximum Ethernet frame size (without FCS)
constexpr size_t kMaxFrameSize = 1514;

/// Minimum Ethernet frame size (without FCS)
constexpr size_t kMinFrameSize = 60;

/// Ethernet header size
constexpr size_t kEthernetHeaderSize = 14;

/// 802.1Q VLAN tag size
constexpr size_t kVlanTagSize = 4;

/// Maximum frame size with VLAN tag
constexpr size_t kMaxFrameSizeVlan = kMaxFrameSize + kVlanTagSize;

/// EtherType for EtherCAT
constexpr uint16_t kEtherTypeEtherCAT = 0x88A4;

/// EtherType for IPv4
constexpr uint16_t kEtherTypeIPv4 = 0x0800;

/// EtherType for IPv6
constexpr uint16_t kEtherTypeIPv6 = 0x86DD;

/// EtherType for ARP
constexpr uint16_t kEtherTypeARP = 0x0806;

/// EtherType for 802.1Q VLAN
constexpr uint16_t kEtherType8021Q = 0x8100;

/// MAC address size
constexpr size_t kMacAddressSize = 6;

// ============================================================================
// MAC Address Type
// ============================================================================

/**
 * @brief MAC address wrapper with utility methods
 */
struct MacAddress {
    uint8_t bytes[kMacAddressSize];
    
    MacAddress() { std::memset(bytes, 0, kMacAddressSize); }
    
    MacAddress(uint8_t b0, uint8_t b1, uint8_t b2, uint8_t b3, uint8_t b4, uint8_t b5) {
        bytes[0] = b0; bytes[1] = b1; bytes[2] = b2;
        bytes[3] = b3; bytes[4] = b4; bytes[5] = b5;
    }
    
    MacAddress(const uint8_t* mac) {
        std::memcpy(bytes, mac, kMacAddressSize);
    }
    
    bool operator==(const MacAddress& other) const {
        return std::memcmp(bytes, other.bytes, kMacAddressSize) == 0;
    }
    
    bool operator!=(const MacAddress& other) const {
        return !(*this == other);
    }
    
    bool isZero() const {
        for (size_t i = 0; i < kMacAddressSize; i++) {
            if (bytes[i] != 0) return false;
        }
        return true;
    }
    
    bool isBroadcast() const {
        for (size_t i = 0; i < kMacAddressSize; i++) {
            if (bytes[i] != 0xFF) return false;
        }
        return true;
    }
    
    bool isMulticast() const {
        return (bytes[0] & 0x01) != 0;
    }
    
    bool isLocallyAdministered() const {
        return (bytes[0] & 0x02) != 0;
    }
    
    /// EtherCAT broadcast MAC
    static MacAddress etherCATBroadcast() {
        return MacAddress(0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF);
    }
    
    /// EtherCAT multicast MAC (used by some implementations)
    static MacAddress etherCATMulticast() {
        return MacAddress(0x01, 0x01, 0x05, 0x00, 0x00, 0x00);
    }
};

// ============================================================================
// Time Types
// ============================================================================

/// Time duration in microseconds
using Microseconds = int64_t;

/// Time duration in milliseconds
using Milliseconds = int64_t;

/// Time duration in nanoseconds
using Nanoseconds = int64_t;

/// Timestamp (microseconds since epoch or system start)
using Timestamp = uint64_t;

// ============================================================================
// Error Codes
// ============================================================================

/**
 * @brief HAL error codes
 */
enum class Error {
    OK = 0,
    InvalidArgument,
    NotInitialized,
    AlreadyInitialized,
    Timeout,
    WouldBlock,
    NoMemory,
    BufferTooSmall,
    BufferFull,
    Empty,
    NotSupported,
    PermissionDenied,
    InterfaceNotFound,
    LinkDown,
    TransmitFailed,
    ReceiveFailed,
    ConfigurationFailed,
    InternalError,
    Cancelled,
};

// ============================================================================
// Result Type
// ============================================================================

/**
 * @brief Result type combining success/error with optional value
 */
template<typename T>
struct Result {
    T value;
    Error error;
    
    Result() : value{}, error{Error::OK} {}
    Result(const T& v) : value{v}, error{Error::OK} {}
    Result(Error e) : value{}, error{e} {}
    Result(const T& v, Error e) : value{v}, error{e} {}
    
    bool ok() const { return error == Error::OK; }
    operator bool() const { return ok(); }
};

/**
 * @brief Specialization for void result
 */
template<>
struct Result<void> {
    Error error;
    
    Result() : error{Error::OK} {}
    Result(Error e) : error{e} {}
    
    bool ok() const { return error == Error::OK; }
    operator bool() const { return ok(); }
};

using VoidResult = Result<void>;

// ============================================================================
// Byte Order Utilities
// ============================================================================

/**
 * @brief Swap bytes of a 16-bit value
 */
inline uint16_t bswap16(uint16_t v) {
#if defined(__GNUC__) || defined(__clang__)
    return __builtin_bswap16(v);
#else
    return (v >> 8) | (v << 8);
#endif
}

/**
 * @brief Swap bytes of a 32-bit value
 */
inline uint32_t bswap32(uint32_t v) {
#if defined(__GNUC__) || defined(__clang__)
    return __builtin_bswap32(v);
#else
    return ((v >> 24) & 0x000000FF) |
           ((v >>  8) & 0x0000FF00) |
           ((v <<  8) & 0x00FF0000) |
           ((v << 24) & 0xFF000000);
#endif
}

/**
 * @brief Host to big-endian (network byte order)
 */
inline uint16_t htobe16(uint16_t host) {
#if __BYTE_ORDER__ == __ORDER_LITTLE_ENDIAN__
    return bswap16(host);
#else
    return host;
#endif
}

inline uint32_t htobe32(uint32_t host) {
#if __BYTE_ORDER__ == __ORDER_LITTLE_ENDIAN__
    return bswap32(host);
#else
    return host;
#endif
}

/**
 * @brief Big-endian to host
 */
inline uint16_t betoh16(uint16_t be) {
    return htobe16(be);  // Same operation
}

inline uint32_t betoh32(uint32_t be) {
    return htobe32(be);  // Same operation
}

/**
 * @brief Host to little-endian
 */
inline uint16_t htole16(uint16_t host) {
#if __BYTE_ORDER__ == __ORDER_LITTLE_ENDIAN__
    return host;
#else
    return bswap16(host);
#endif
}

inline uint32_t htole32(uint32_t host) {
#if __BYTE_ORDER__ == __ORDER_LITTLE_ENDIAN__
    return host;
#else
    return bswap32(host);
#endif
}

/**
 * @brief Little-endian to host
 */
inline uint16_t letoh16(uint16_t le) {
    return htole16(le);  // Same operation
}

inline uint32_t letoh32(uint32_t le) {
    return htole32(le);  // Same operation
}

} // namespace HAL
} // namespace EtherCAT

/**
 * @file NetworkAbstraction.hpp
 * @brief EtherCAT types: Network Abstraction
 *
 * Split out of Types.hpp.
 */

#pragma once

#include <array>
#include <cstdint>
#include <cstring>
#include <functional>

namespace EtherCAT {

// ============================================================================
// Network Abstraction
// ============================================================================

/**
 * @brief Abstract network interface for EtherCAT communication
 */
struct NetworkInterface {
    /**
     * @brief Send a packet
     * @param data Pointer to packet data
     * @param len Length of packet
     * @return true on success
     */
    std::function<bool(const uint8_t* data, size_t len)> send;

    /**
     * @brief Receive a packet (optional/polling)
     * @param buffer Output buffer
     * @param max_len Maximum length to read
     * @param[out] out_len Actual length read
     * @return true on success
     */
    std::function<bool(uint8_t* buffer, size_t max_len, size_t* out_len)> receive;

    /**
     * @brief Optional opaque native handle owned by the platform adapter.
     */
    void* native_handle = nullptr;

#ifdef ESP_PLATFORM
    static NetworkInterface fromEspHandle(esp_eth_handle_t eth_handle) {
        NetworkInterface iface;
        iface.native_handle = eth_handle;
        return iface;
    }
#endif
};

} // namespace EtherCAT

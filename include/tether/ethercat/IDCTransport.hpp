#pragma once

/**
 * @file IDCTransport.hpp
 * @brief Abstract transport interface for DC register I/O
 *
 * Implementations provide the actual register read/write operations
 * for DC synchronization. A concrete implementation backed by Raw
 * EtherCAT frames wraps ec_aprd / ec_apwr. A mock implementation
 * is used for unit tests.
 */

#include <cstdint>
#include <cstddef>

namespace EtherCAT {

/**
 * @brief Abstract transport interface for DC register I/O
 *
 * All slave addressing is by zero-based index; concrete implementations
 * translate to the appropriate auto-increment or configured address.
 */
class IDCTransport {
public:
    virtual ~IDCTransport() = default;

    /**
     * @brief Read a DC register from a slave (position-addressed)
     *
     * @param slave_index Zero-based slave index
     * @param reg_addr    Register address (e.g. 0x0910 for DC_SYSTIME)
     * @param data        Destination buffer
     * @param size        Number of bytes to read
     * @param timeout_ms  Timeout for the operation
     * @return true on success
     */
    virtual bool readRegister(uint16_t slave_index, uint16_t reg_addr,
                              void* data, uint16_t size,
                              unsigned int timeout_ms = 200) = 0;

    /**
     * @brief Write a DC register on a slave (position-addressed)
     *
     * @param slave_index Zero-based slave index
     * @param reg_addr    Register address (e.g. 0x0920 for DC_SYSOFFSET)
     * @param data        Source buffer
     * @param size        Number of bytes to write
     * @param timeout_ms  Timeout for the operation
     * @return true on success
     */
    virtual bool writeRegister(uint16_t slave_index, uint16_t reg_addr,
                               const void* data, uint16_t size,
                               unsigned int timeout_ms = 200) = 0;

    /**
     * @brief Send a DC synchronization datagram (ARMW/FRMW)
     *
     * Sends a sync datagram to the reference slave. The transport
     * handles frame construction, index allocation, and transmission.
     *
     * @param slave_index Zero-based slave index (reference clock slave)
     * @param reg_addr    Register address (typically 0x0910 DC_SYSTIME)
     * @param data        Time data to write
     * @param size        Number of bytes
     * @return true on success
     */
    virtual bool sendSyncDatagram(uint16_t slave_index, uint16_t reg_addr,
                                  const void* data, uint16_t size) = 0;

    /**
     * @brief Get current master time in nanoseconds
     *
     * @return Current monotonic time in nanoseconds
     */
    virtual uint64_t getMasterTimeNs() = 0;

    /**
     * @brief Blocking delay
     *
     * @param ms Milliseconds to wait
     */
    virtual void delayMs(uint32_t ms) = 0;
};

} // namespace EtherCAT

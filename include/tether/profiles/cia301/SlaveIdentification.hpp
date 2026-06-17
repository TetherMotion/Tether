/**
 * @file SlaveIdentification.hpp
 * @brief EtherCAT Slave Identification Module
 * 
 * @details
 * This module provides comprehensive slave identification by reading CiA 301
 * predefined objects from EtherCAT slaves. All string data is efficiently stored
 * in a single contiguous memory buffer with null-terminated strings, allowing
 * direct const char* access without individual allocations.
 * 
 * ## Memory Layout
 * 
 * The SlaveIdentity structure stores all strings in a single buffer:
 * ```
 * m_string_buffer: "DeviceName\0HWVersion\0SWVersion\0Manufacturer\0..."
 *                   ^          ^          ^          ^
 *                   |          |          |          |
 * Pointers:    m_device_name  m_hw_ver   m_sw_ver   m_manufacturer
 * ```
 * 
 * This approach:
 * - Avoids heap fragmentation from many small string allocations
 * - Provides cache-friendly sequential memory access
 * - Allows zero-copy string views via const char*
 * - Makes the structure self-contained and easily copyable
 * 
 * ## Usage Example
 * 
 * ```cpp
 * #include "SlaveIdentification.hpp"
 * 
 * EtherCAT::SDO::SDOManager& sdo = master.sdoManager();
 * SlaveIdentifier identifier(sdo);
 * 
 * // Identify slave 0
 * SlaveIdentity identity;
 * if (identifier.identify(0, identity)) {
 *     printf("Device: %s\n", identity.deviceName());
 *     printf("Vendor: 0x%08X\n", identity.vendorId());
 *     printf("HW: %s\n", identity.hardwareVersion());
 * }
 * ```
 * 
 * @see CiA301Defs.hpp for object dictionary definitions
 */

#pragma once

#include <cstdint>
#include <cstddef>
#include <cstring>
#include "profiles/cia301/CiA301Defs.hpp"

namespace EtherCAT {
class Master;  // forward declaration

// ============================================================================
// Configuration
// ============================================================================

/**
 * @brief Maximum size of the string buffer per slave identity
 * 
 * This should be large enough to hold all string fields concatenated:
 * - Device name (~64 chars)
 * - Hardware version (~32 chars)
 * - Software version (~32 chars)
 * - Manufacturer name (~64 chars)
 * - Additional strings (~128 chars)
 */
constexpr size_t kMaxIdentityStringBuffer = 512;

/**
 * @brief Maximum number of supported slaves for batch identification
 */
constexpr size_t kMaxIdentifiedSlaves = 16;

// ============================================================================
// Identity Record (CiA 301 0x1018)
// ============================================================================

/**
 * @brief CiA 301 Identity Record structure
 * 
 * Contains the standard identity information from object 0x1018.
 */
struct IdentityRecord {
    uint32_t vendor_id{0};       ///< Vendor ID (ETG assigned)
    uint32_t product_code{0};    ///< Vendor-specific product code
    uint32_t revision_number{0}; ///< Revision (major.minor in high.low words)
    uint32_t serial_number{0};   ///< Device serial number
    
    /**
     * @brief Get major revision (high word)
     */
    uint16_t revisionMajor() const { 
        return static_cast<uint16_t>((revision_number >> 16) & 0xFFFF); 
    }
    
    /**
     * @brief Get minor revision (low word)
     */
    uint16_t revisionMinor() const { 
        return static_cast<uint16_t>(revision_number & 0xFFFF); 
    }
    
    /**
     * @brief Check if identity has been read successfully
     */
    bool isValid() const { return vendor_id != 0; }
};

// ============================================================================
// Device Type Information
// ============================================================================

/**
 * @brief CiA 301 Device Type structure (0x1000)
 */
struct DeviceTypeInfo {
    uint32_t raw_value{0};       ///< Raw device type value
    
    /**
     * @brief Get device profile number (e.g., 402 for drives)
     */
    uint16_t profileNumber() const {
        return static_cast<uint16_t>(raw_value & 0xFFFF);
    }
    
    /**
     * @brief Get additional information (upper 16 bits)
     */
    uint16_t additionalInfo() const {
        return static_cast<uint16_t>((raw_value >> 16) & 0xFFFF);
    }
    
    /**
     * @brief Check if this is a CiA 402 drive
     */
    bool isCiA402Drive() const { return profileNumber() == 402; }
};

// ============================================================================
// Slave Identity Structure
// ============================================================================

/**
 * @brief Complete slave identity with all readable information
 * 
 * This structure stores all identification data for a slave, using a single
 * contiguous buffer for string storage. String accessors return const char*
 * pointers into this buffer.
 * 
 * ## String Fields
 * 
 * The following string fields are supported:
 * - Device name (0x1008)
 * - Hardware version (0x1009)
 * - Software version (0x100A)
 * - Order code / catalog number (if available)
 * - Additional vendor-specific strings
 * 
 * ## Copying
 * 
 * The structure is safely copyable. When copied, string pointers are
 * automatically adjusted to point to the copy's buffer.
 */
class SlaveIdentity {
public:
    /**
     * @brief Default constructor - creates empty identity
     */
    SlaveIdentity();
    
    /**
     * @brief Copy constructor - properly copies string buffer
     */
    SlaveIdentity(const SlaveIdentity& other);
    
    /**
     * @brief Copy assignment - properly copies string buffer
     */
    SlaveIdentity& operator=(const SlaveIdentity& other);
    
    /**
     * @brief Move constructor
     */
    SlaveIdentity(SlaveIdentity&& other) noexcept;
    
    /**
     * @brief Move assignment
     */
    SlaveIdentity& operator=(SlaveIdentity&& other) noexcept;
    
    // ========================================================================
    // Identity Record Access
    // ========================================================================
    
    /**
     * @brief Get identity record (0x1018)
     */
    const IdentityRecord& identityRecord() const { return m_identity; }
    
    /**
     * @brief Get vendor ID
     */
    uint32_t vendorId() const { return m_identity.vendor_id; }
    
    /**
     * @brief Get product code
     */
    uint32_t productCode() const { return m_identity.product_code; }
    
    /**
     * @brief Get revision number
     */
    uint32_t revisionNumber() const { return m_identity.revision_number; }
    
    /**
     * @brief Get serial number
     */
    uint32_t serialNumber() const { return m_identity.serial_number; }
    
    // ========================================================================
    // Device Type Access
    // ========================================================================
    
    /**
     * @brief Get device type info (0x1000)
     */
    const DeviceTypeInfo& deviceType() const { return m_device_type; }
    
    /**
     * @brief Check if this is a CiA 402 drive
     */
    bool isCiA402Drive() const { return m_device_type.isCiA402Drive(); }
    
    // ========================================================================
    // String Access
    // ========================================================================
    
    /**
     * @brief Get device name (0x1008)
     * @return Pointer to null-terminated string, or empty string if not set
     */
    const char* deviceName() const { return m_device_name ? m_device_name : ""; }
    
    /**
     * @brief Get hardware version (0x1009)
     * @return Pointer to null-terminated string, or empty string if not set
     */
    const char* hardwareVersion() const { return m_hw_version ? m_hw_version : ""; }
    
    /**
     * @brief Get software version (0x100A)
     * @return Pointer to null-terminated string, or empty string if not set
     */
    const char* softwareVersion() const { return m_sw_version ? m_sw_version : ""; }
    
    /**
     * @brief Get order code / catalog number (if available)
     * @return Pointer to null-terminated string, or empty string if not set
     */
    const char* orderCode() const { return m_order_code ? m_order_code : ""; }
    
    /**
     * @brief Check if device name was read
     */
    bool hasDeviceName() const { return m_device_name != nullptr && m_device_name[0] != '\0'; }
    
    /**
     * @brief Check if hardware version was read
     */
    bool hasHardwareVersion() const { return m_hw_version != nullptr && m_hw_version[0] != '\0'; }
    
    /**
     * @brief Check if software version was read
     */
    bool hasSoftwareVersion() const { return m_sw_version != nullptr && m_sw_version[0] != '\0'; }
    
    // ========================================================================
    // Additional Information
    // ========================================================================
    
    /**
     * @brief Get slave index (position in chain)
     */
    uint16_t slaveIndex() const { return m_slave_index; }
    
    /**
     * @brief Get error register value (0x1001)
     */
    uint8_t errorRegister() const { return m_error_register; }
    
    /**
     * @brief Get supported drive modes (0x6502, CiA 402)
     */
    uint32_t supportedDriveModes() const { return m_supported_modes; }
    
    /**
     * @brief Check if identification was successful
     */
    bool isValid() const { return m_valid && m_identity.isValid(); }
    
    // ========================================================================
    // String Buffer Management (for internal use)
    // ========================================================================
    
    /**
     * @brief Add a string to the buffer
     * @param str String to add
     * @param len Length of string (excluding null terminator)
     * @return Pointer to the string in the buffer, or nullptr if no space
     */
    const char* addString(const char* str, size_t len);
    
    /**
     * @brief Set identity record
     */
    void setIdentityRecord(const IdentityRecord& record) { m_identity = record; }
    
    /**
     * @brief Set device type
     */
    void setDeviceType(const DeviceTypeInfo& type) { m_device_type = type; }
    
    /**
     * @brief Set device name pointer (must point to buffer)
     */
    void setDeviceNamePtr(const char* ptr) { m_device_name = ptr; }
    
    /**
     * @brief Set hardware version pointer (must point to buffer)
     */
    void setHardwareVersionPtr(const char* ptr) { m_hw_version = ptr; }
    
    /**
     * @brief Set software version pointer (must point to buffer)
     */
    void setSoftwareVersionPtr(const char* ptr) { m_sw_version = ptr; }
    
    /**
     * @brief Set order code pointer (must point to buffer)
     */
    void setOrderCodePtr(const char* ptr) { m_order_code = ptr; }
    
    /**
     * @brief Set slave index
     */
    void setSlaveIndex(uint16_t index) { m_slave_index = index; }
    
    /**
     * @brief Set error register
     */
    void setErrorRegister(uint8_t reg) { m_error_register = reg; }
    
    /**
     * @brief Set supported drive modes
     */
    void setSupportedDriveModes(uint32_t modes) { m_supported_modes = modes; }
    
    /**
     * @brief Mark identity as valid
     */
    void setValid(bool valid) { m_valid = valid; }
    
    /**
     * @brief Clear all data
     */
    void clear();
    
    /**
     * @brief Get remaining buffer space
     */
    size_t remainingBufferSpace() const { 
        return kMaxIdentityStringBuffer - m_buffer_offset; 
    }

private:
    /**
     * @brief Recalculate string pointers after copy
     * 
     * Called after copying buffer to update pointers to new buffer location.
     */
    void recalculatePointers(const SlaveIdentity& source);
    
    // Numeric identity data
    IdentityRecord m_identity;
    DeviceTypeInfo m_device_type;
    uint16_t m_slave_index{0};
    uint8_t m_error_register{0};
    uint32_t m_supported_modes{0};
    bool m_valid{false};
    
    // String buffer and pointers
    char m_string_buffer[kMaxIdentityStringBuffer];
    size_t m_buffer_offset{0};
    
    // Pointers into string buffer (null if not set)
    const char* m_device_name{nullptr};
    const char* m_hw_version{nullptr};
    const char* m_sw_version{nullptr};
    const char* m_order_code{nullptr};
};

// ============================================================================
// Slave Identifier Class
// ============================================================================

/**
 * @brief Reads identification data from EtherCAT slaves
 * 
 * This class provides methods to read identification information from
 * EtherCAT slaves using CoE SDO transfers. It reads:
 * 
 * 1. Identity record (0x1018) - Vendor ID, product code, revision, serial
 * 2. Device type (0x1000) - Profile number
 * 3. Device name (0x1008) - Human-readable name
 * 4. Hardware version (0x1009)
 * 5. Software version (0x100A)
 * 6. Error register (0x1001)
 * 7. CiA 402 specific: Supported drive modes (0x6502)
 * 
 * ## Usage
 * 
 * ```cpp
 * EtherCAT::Master& master = ...;
 * SlaveIdentifier identifier(master);
 * identifier.setSDOTimeout(2000);  // 2 second timeout
 * 
 * // Identify single slave
 * SlaveIdentity id;
 * if (identifier.identify(0, id)) {
 *     printSlaveInfo(id);
 * }
 * 
 * // Identify all slaves
 * std::vector<SlaveIdentity> identities;
 * int count = identifier.identifyAll(identities, 4);  // 4 slaves
 * ```
 */
class SlaveIdentifier {
public:
    /**
     * @brief Construct with a Master reference
     * @param master  Master instance (must outlive this object)
     */
    explicit SlaveIdentifier(Master& master);
    
    /**
     * @brief Set SDO timeout in milliseconds
     */
    void setSDOTimeout(uint32_t timeout_ms) { m_sdo_timeout_ms = timeout_ms; }
    
    /**
     * @brief Get SDO timeout
     */
    uint32_t getSDOTimeout() const { return m_sdo_timeout_ms; }
    
    /**
     * @brief Identify a single slave
     * 
     * Reads all available identification data from the specified slave.
     * The slave must be in at least PRE_OP state for SDO access.
     * 
     * @param slave_index Slave index (0-based)
     * @param[out] identity Filled with identification data
     * @return true if identification was successful (at least identity record read)
     */
    bool identify(uint16_t slave_index, SlaveIdentity& identity);
    
    /**
     * @brief Identify multiple slaves
     * 
     * @param[out] identities Vector to fill with identities
     * @param slave_count Number of slaves to identify
     * @return Number of successfully identified slaves
     */
    size_t identifyAll(SlaveIdentity* identities, size_t max_count, size_t slave_count);
    
    /**
     * @brief Read identity record (0x1018)
     * 
     * @param slave_index Slave index
     * @param[out] record Filled with identity data
     * @return true on success
     */
    bool readIdentityRecord(uint16_t slave_index, IdentityRecord& record);
    
    /**
     * @brief Read device type (0x1000)
     * 
     * @param slave_index Slave index
     * @param[out] type Filled with device type
     * @return true on success
     */
    bool readDeviceType(uint16_t slave_index, DeviceTypeInfo& type);
    
    /**
     * @brief Read a string object
     * 
     * @param slave_index Slave index
     * @param index Object dictionary index
     * @param[out] buffer Buffer to store string
     * @param buffer_size Size of buffer
     * @param[out] actual_len Actual string length (optional)
     * @return true on success
     */
    bool readString(uint16_t slave_index, uint16_t index, 
                    char* buffer, size_t buffer_size,
                    size_t* actual_len = nullptr);
    
    /**
     * @brief Read error register (0x1001)
     * 
     * @param slave_index Slave index
     * @param[out] error_register Error register value
     * @return true on success
     */
    bool readErrorRegister(uint16_t slave_index, uint8_t& error_register);
    
    /**
     * @brief Read supported drive modes (0x6502, CiA 402)
     * 
     * @param slave_index Slave index
     * @param[out] modes Supported modes bitmask
     * @return true on success
     */
    bool readSupportedDriveModes(uint16_t slave_index, uint32_t& modes);

private:
    Master& m_master;
    uint32_t m_sdo_timeout_ms{1000};
};

// ============================================================================
// Known Vendor IDs
// ============================================================================

namespace VendorID {
    constexpr uint32_t Beckhoff           = 0x00000002;
    constexpr uint32_t Lenze              = 0x0000003B;
    constexpr uint32_t Bosch              = 0x00000048;
    constexpr uint32_t Kollmorgen         = 0x0000006A;
    constexpr uint32_t Delta              = 0x000001DD;
    constexpr uint32_t Omron              = 0x00000083;
    constexpr uint32_t Panasonic          = 0x0000006D;
    constexpr uint32_t Yaskawa            = 0x00000539;
    constexpr uint32_t Mitsubishi         = 0x00000070;
    constexpr uint32_t Siemens            = 0x0000004D;
    constexpr uint32_t ABB                = 0x00000006;
    constexpr uint32_t Schneider          = 0x0000005A;
    constexpr uint32_t Festo              = 0x0000001F;
    constexpr uint32_t Copley             = 0x000000AB;
    constexpr uint32_t ElmoMotion         = 0x0000009A;
    constexpr uint32_t TechnosoftMotion   = 0x000000C4;
    constexpr uint32_t Nanotec            = 0x0000026C;
    constexpr uint32_t Trinamic           = 0x00000286;
    constexpr uint32_t INOVANCE           = 0x00100000;
    constexpr uint32_t Leadshine          = 0x000004D8;
}

/**
 * @brief Get vendor name from vendor ID
 * 
 * @param vendor_id ETG-assigned vendor ID
 * @return Vendor name string, or "Unknown" if not recognized
 */
const char* getVendorName(uint32_t vendor_id);

/**
 * @brief Get human-readable product name from product code (fallback)
 *
 * Returns a static string for well-known product codes or a formatted
 * "Product 0x%08X" fallback when unknown.
 */
const char* getProductName(uint32_t product_code);

// ============================================================================
// Utility Functions
// ============================================================================

/**
 * @brief Print slave identity to log
 * 
 * @param identity Slave identity to print
 * @param tag Log tag
 */
void logSlaveIdentity(const SlaveIdentity& identity, const char* tag = "SlaveID");

/**
 * @brief Print all slave identities in a formatted table
 * 
 * @param identities Array of identities
 * @param count Number of identities
 * @param tag Log tag
 */
void logSlaveIdentityTable(const SlaveIdentity* identities, size_t count, 
                           const char* tag = "SlaveID");

} // namespace EtherCAT

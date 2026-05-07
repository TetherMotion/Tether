/**
 * @file SlaveIdentification.cpp
 * @brief EtherCAT Slave Identification Implementation
 */

#include "profiles/cia301/SlaveIdentification.hpp"
#include "tether/platform/EspCompat.hpp"
#include "EtherCATSDO.hpp"

using ::EtherCAT::SDO::SDOManager;

static const char* TAG = "SlaveID";

namespace EtherCAT {

// ============================================================================
// SlaveIdentity Implementation
// ============================================================================

SlaveIdentity::SlaveIdentity() {
    std::memset(m_string_buffer, 0, sizeof(m_string_buffer));
}

SlaveIdentity::SlaveIdentity(const SlaveIdentity& other)
    : m_identity(other.m_identity)
    , m_device_type(other.m_device_type)
    , m_slave_index(other.m_slave_index)
    , m_error_register(other.m_error_register)
    , m_supported_modes(other.m_supported_modes)
    , m_valid(other.m_valid)
    , m_buffer_offset(other.m_buffer_offset)
{
    // Copy the string buffer
    std::memcpy(m_string_buffer, other.m_string_buffer, sizeof(m_string_buffer));
    
    // Recalculate pointers to point to our buffer
    recalculatePointers(other);
}

SlaveIdentity& SlaveIdentity::operator=(const SlaveIdentity& other) {
    if (this != &other) {
        m_identity = other.m_identity;
        m_device_type = other.m_device_type;
        m_slave_index = other.m_slave_index;
        m_error_register = other.m_error_register;
        m_supported_modes = other.m_supported_modes;
        m_valid = other.m_valid;
        m_buffer_offset = other.m_buffer_offset;
        
        // Copy the string buffer
        std::memcpy(m_string_buffer, other.m_string_buffer, sizeof(m_string_buffer));
        
        // Recalculate pointers
        recalculatePointers(other);
    }
    return *this;
}

SlaveIdentity::SlaveIdentity(SlaveIdentity&& other) noexcept
    : m_identity(other.m_identity)
    , m_device_type(other.m_device_type)
    , m_slave_index(other.m_slave_index)
    , m_error_register(other.m_error_register)
    , m_supported_modes(other.m_supported_modes)
    , m_valid(other.m_valid)
    , m_buffer_offset(other.m_buffer_offset)
{
    // Move the buffer contents
    std::memcpy(m_string_buffer, other.m_string_buffer, sizeof(m_string_buffer));
    recalculatePointers(other);
    
    // Clear source
    other.clear();
}

SlaveIdentity& SlaveIdentity::operator=(SlaveIdentity&& other) noexcept {
    if (this != &other) {
        m_identity = other.m_identity;
        m_device_type = other.m_device_type;
        m_slave_index = other.m_slave_index;
        m_error_register = other.m_error_register;
        m_supported_modes = other.m_supported_modes;
        m_valid = other.m_valid;
        m_buffer_offset = other.m_buffer_offset;
        
        std::memcpy(m_string_buffer, other.m_string_buffer, sizeof(m_string_buffer));
        recalculatePointers(other);
        
        other.clear();
    }
    return *this;
}

void SlaveIdentity::recalculatePointers(const SlaveIdentity& source) {
    // Calculate pointer offsets and translate to our buffer
    auto translatePtr = [this, &source](const char* src_ptr) -> const char* {
        if (src_ptr == nullptr) return nullptr;
        
        // Calculate offset in source buffer
        ptrdiff_t offset = src_ptr - source.m_string_buffer;
        if (offset >= 0 && static_cast<size_t>(offset) < sizeof(m_string_buffer)) {
            return m_string_buffer + offset;
        }
        return nullptr;
    };
    
    m_device_name = translatePtr(source.m_device_name);
    m_hw_version = translatePtr(source.m_hw_version);
    m_sw_version = translatePtr(source.m_sw_version);
    m_order_code = translatePtr(source.m_order_code);
}

const char* SlaveIdentity::addString(const char* str, size_t len) {
    if (str == nullptr || len == 0) {
        return nullptr;
    }
    
    // Need space for string + null terminator
    if (m_buffer_offset + len + 1 > kMaxIdentityStringBuffer) {
        return nullptr;  // No space
    }
    
    // Copy string to buffer
    char* dest = m_string_buffer + m_buffer_offset;
    std::memcpy(dest, str, len);
    dest[len] = '\0';
    
    m_buffer_offset += len + 1;
    
    return dest;
}

void SlaveIdentity::clear() {
    m_identity = IdentityRecord{};
    m_device_type = DeviceTypeInfo{};
    m_slave_index = 0;
    m_error_register = 0;
    m_supported_modes = 0;
    m_valid = false;
    m_buffer_offset = 0;
    m_device_name = nullptr;
    m_hw_version = nullptr;
    m_sw_version = nullptr;
    m_order_code = nullptr;
    std::memset(m_string_buffer, 0, sizeof(m_string_buffer));
}

// ============================================================================
// SlaveIdentifier Implementation
// ============================================================================

SlaveIdentifier::SlaveIdentifier(SDO::SDOManager& sdo)
    : m_sdo(sdo) {}

bool SlaveIdentifier::identify(uint16_t slave_index, SlaveIdentity& identity) {
    identity.clear();
    identity.setSlaveIndex(slave_index);
    
    bool any_success = false;
    
    // Read identity record (0x1018) - this is the most important
    IdentityRecord record;
    if (readIdentityRecord(slave_index, record)) {
        identity.setIdentityRecord(record);
        any_success = true;
    } else {
        TETHER_LOGW(TAG, "Slave %u: Failed to read identity record", slave_index);
    }
    
    // Read device type (0x1000)
    DeviceTypeInfo device_type;
    if (readDeviceType(slave_index, device_type)) {
        identity.setDeviceType(device_type);
    }
    
    // Read device name (0x1008)
    char temp_buffer[256];
    size_t actual_len = 0;
    
    if (readString(slave_index, CiA301::ManufacturerDeviceName, 
                   temp_buffer, sizeof(temp_buffer), &actual_len)) {
        const char* ptr = identity.addString(temp_buffer, actual_len);
        identity.setDeviceNamePtr(ptr);
    }
    
    // Read hardware version (0x1009)
    if (readString(slave_index, CiA301::ManufacturerHWVersion,
                   temp_buffer, sizeof(temp_buffer), &actual_len)) {
        const char* ptr = identity.addString(temp_buffer, actual_len);
        identity.setHardwareVersionPtr(ptr);
    }
    
    // Read software version (0x100A)
    if (readString(slave_index, CiA301::ManufacturerSWVersion,
                   temp_buffer, sizeof(temp_buffer), &actual_len)) {
        const char* ptr = identity.addString(temp_buffer, actual_len);
        identity.setSoftwareVersionPtr(ptr);
    }
    
    // Read error register (0x1001)
    uint8_t error_reg = 0;
    if (readErrorRegister(slave_index, error_reg)) {
        identity.setErrorRegister(error_reg);
    }
    
    // If it's a CiA 402 drive, read supported modes
    if (identity.deviceType().isCiA402Drive()) {
        uint32_t modes = 0;
        if (readSupportedDriveModes(slave_index, modes)) {
            identity.setSupportedDriveModes(modes);
        }
    }
    
    identity.setValid(any_success);
    return any_success;
}

size_t SlaveIdentifier::identifyAll(SlaveIdentity* identities, size_t max_count, 
                                    size_t slave_count) {
    if (identities == nullptr || max_count == 0) {
        return 0;
    }
    
    size_t count_to_identify = (slave_count < max_count) ? slave_count : max_count;
    size_t success_count = 0;
    
    for (size_t i = 0; i < count_to_identify; i++) {
        if (identify(static_cast<uint16_t>(i), identities[i])) {
            success_count++;
        }
    }
    
    return success_count;
}

bool SlaveIdentifier::readIdentityRecord(uint16_t slave_index, IdentityRecord& record) {
    record = IdentityRecord{};
    
    // Read Vendor ID (0x1018:01)
    if (!m_sdo.readU32(slave_index, CiA301::Identity, CiA301::IdentitySub::VendorID,
                       record.vendor_id, m_sdo_timeout_ms)) {
        return false;
    }
    
    // Read Product Code (0x1018:02)
    m_sdo.readU32(slave_index, CiA301::Identity, CiA301::IdentitySub::ProductCode,
                  record.product_code, m_sdo_timeout_ms);
    
    // Read Revision Number (0x1018:03)
    m_sdo.readU32(slave_index, CiA301::Identity, CiA301::IdentitySub::RevisionNumber,
                  record.revision_number, m_sdo_timeout_ms);
    
    // Read Serial Number (0x1018:04)
    m_sdo.readU32(slave_index, CiA301::Identity, CiA301::IdentitySub::SerialNumber,
                  record.serial_number, m_sdo_timeout_ms);
    
    return true;
}

bool SlaveIdentifier::readDeviceType(uint16_t slave_index, DeviceTypeInfo& type) {
    type = DeviceTypeInfo{};
    
    return m_sdo.readU32(slave_index, CiA301::DeviceType, 0,
                         type.raw_value, m_sdo_timeout_ms);
}

bool SlaveIdentifier::readString(uint16_t slave_index, uint16_t index,
                                 char* buffer, size_t buffer_size,
                                 size_t* actual_len) {
    if (buffer == nullptr || buffer_size == 0) {
        return false;
    }
    
    buffer[0] = '\0';
    
    size_t read_len = 0;
    // Try subindex 0 first (common for visible strings)
    bool ok = m_sdo.readSync(slave_index, index, 0, buffer, buffer_size - 1,
                             m_sdo_timeout_ms, &read_len);

    // Some devices present the string at subindex 1 - try that as a fallback
    if ((!ok || read_len == 0) && m_sdo.readSync(slave_index, index, 1, buffer, buffer_size - 1,
                                                  m_sdo_timeout_ms, &read_len)) {
        ok = true;
    }

    if (!ok) {
        return false;
    }

    // Ensure null termination (read_len may be larger than buffer-1)
    if (read_len >= buffer_size) {
        read_len = buffer_size - 1;
    }
    buffer[read_len] = '\0';

    // Trim trailing whitespace and non-printable chars (some devices pad strings)
    size_t str_len = read_len;
    while (str_len > 0 && (buffer[str_len - 1] == '\0' || buffer[str_len - 1] == ' ' ||
                           buffer[str_len - 1] == '\r' || buffer[str_len - 1] == '\n' ||
                           static_cast<unsigned char>(buffer[str_len - 1]) < 32)) {
        str_len--;
    }

    buffer[str_len] = '\0';

    if (actual_len) {
        *actual_len = str_len;
    }

    return true;
}

bool SlaveIdentifier::readErrorRegister(uint16_t slave_index, uint8_t& error_register) {
    return m_sdo.readU8(slave_index, CiA301::ErrorRegister, 0,
                        error_register, m_sdo_timeout_ms);
}

bool SlaveIdentifier::readSupportedDriveModes(uint16_t slave_index, uint32_t& modes) {
    return m_sdo.readU32(slave_index, 0x6502, 0, modes, m_sdo_timeout_ms);
}

// ============================================================================
// Vendor Name Lookup
// ============================================================================

const char* getVendorName(uint32_t vendor_id) {
    switch (vendor_id) {
        case VendorID::Beckhoff:          return "Beckhoff";
        case VendorID::Lenze:             return "Lenze";
        case VendorID::Bosch:             return "Bosch Rexroth";
        case VendorID::Kollmorgen:        return "Kollmorgen";
        case VendorID::Delta:             return "Delta Electronics";
        case VendorID::Omron:             return "Omron";
        case VendorID::Panasonic:         return "Panasonic";
        case VendorID::Yaskawa:           return "Yaskawa";
        case VendorID::Mitsubishi:        return "Mitsubishi";
        case VendorID::Siemens:           return "Siemens";
        case VendorID::ABB:               return "ABB";
        case VendorID::Schneider:         return "Schneider Electric";
        case VendorID::Festo:             return "Festo";
        case VendorID::Copley:            return "Copley Controls";
        case VendorID::ElmoMotion:        return "Elmo Motion";
        case VendorID::TechnosoftMotion:  return "Technosoft Motion";
        case VendorID::Nanotec:           return "Nanotec";
        case VendorID::Trinamic:          return "Trinamic";
        case VendorID::INOVANCE:          return "INOVANCE";
        case VendorID::Leadshine:         return "Leadshine";
        default: {
            // Unknown vendor: return a formatted fallback like "Vendor 0x%08X"
            // Use a thread-local buffer so successive calls are safe across threads
            static thread_local char buf[32];
            snprintf(buf, sizeof(buf), "Vendor 0x%08lX", (unsigned long)vendor_id);
            return buf;
        }
    }
}

// Product name lookup/fallback
const char* getProductName(uint32_t product_code) {
    switch (product_code) {
        // Add known product codes here if desired, e.g.:
        // case 0x00000715: return "ACME Drive Model 715";
        default: {
            static thread_local char buf[32];
            snprintf(buf, sizeof(buf), "Product 0x%08lX", (unsigned long)product_code);
            return buf;
        }
    }
}

// ============================================================================
// Logging Functions
// ============================================================================

void logSlaveIdentity(const SlaveIdentity& identity, const char* tag) {
    if (!identity.isValid()) {
        TETHER_LOGW(tag, "Slave %u: Invalid/not identified", identity.slaveIndex());
        return;
    }
    
    TETHER_LOGI(tag, "========================================\nSlave %u Identity:\n----------------------------------------", identity.slaveIndex());
    
    // Identity record
    TETHER_LOGI(tag, "  Vendor ID:    0x%08lX (%s)\n  Product Code: 0x%08lX\n  Revision:     %u.%u\n  Serial:       %lu",
             (unsigned long)identity.vendorId(), getVendorName(identity.vendorId()),
             (unsigned long)identity.productCode(),
             identity.identityRecord().revisionMajor(),
             identity.identityRecord().revisionMinor(),
             (unsigned long)identity.serialNumber());
    
    // Device type
    TETHER_LOGI(tag, "  Profile:      %u (CiA %u)",
             identity.deviceType().profileNumber(),
             identity.deviceType().profileNumber());
    
    // Strings
    if (identity.hasDeviceName()) {
        TETHER_LOGI(tag, "  Device Name:  %s", identity.deviceName());
    }
    if (identity.hasHardwareVersion()) {
        TETHER_LOGI(tag, "  HW Version:   %s", identity.hardwareVersion());
    }
    if (identity.hasSoftwareVersion()) {
        TETHER_LOGI(tag, "  SW Version:   %s", identity.softwareVersion());
    }
    
    // CiA 402 specific
    if (identity.isCiA402Drive()) {
        TETHER_LOGI(tag, "  Drive Type:   CiA 402 Servo/Stepper");
        uint32_t modes = identity.supportedDriveModes();
        if (modes != 0) {
            TETHER_LOGI(tag, "  Modes:        0x%08lX", (unsigned long)modes);
            if (modes & (1 << 0)) TETHER_LOGI(tag, "                - Profile Position (PP)");
            if (modes & (1 << 2)) TETHER_LOGI(tag, "                - Profile Velocity (PV)");
            if (modes & (1 << 3)) TETHER_LOGI(tag, "                - Profile Torque (TQ)");
            if (modes & (1 << 5)) TETHER_LOGI(tag, "                - Homing (HM)");
            if (modes & (1 << 6)) TETHER_LOGI(tag, "                - Interpolated Position (IP)");
            if (modes & (1 << 7)) TETHER_LOGI(tag, "                - Cyclic Sync Position (CSP)");
            if (modes & (1 << 8)) TETHER_LOGI(tag, "                - Cyclic Sync Velocity (CSV)");
            if (modes & (1 << 9)) TETHER_LOGI(tag, "                - Cyclic Sync Torque (CST)");
        }
    }
    
    // Error register
    if (identity.errorRegister() != 0) {
        TETHER_LOGW(tag, "  Error Reg:    0x%02X", identity.errorRegister());
    }
    
    TETHER_LOGI(tag, "========================================");
}

void logSlaveIdentityTable(const SlaveIdentity* identities, size_t count, 
                           const char* tag) {
    TETHER_LOGI(tag, "╔════╦══════════════════════════════════╦══════════════╦══════════╗\n║ #  ║ Device Name                      ║ Vendor       ║ Type     ║\n╠════╬══════════════════════════════════╬══════════════╬══════════╣");
    
    for (size_t i = 0; i < count; i++) {
        const auto& id = identities[i];
        
        const char* name = nullptr;
        if (id.hasDeviceName()) {
            name = id.deviceName();
        } else if (id.isValid()) {
            // Use product code fallback when device name is not provided
            name = getProductName(id.productCode());
        } else {
            name = "(unknown)";
        }

        const char* vendor = getVendorName(id.vendorId());
        const char* type = id.isCiA402Drive() ? "CiA402" : "Other";
        
        // Truncate strings if too long
        char name_buf[35] = {0};
        char vendor_buf[32] = {0};
        
        std::strncpy(name_buf, name, sizeof(name_buf) - 1);
        std::strncpy(vendor_buf, vendor, sizeof(vendor_buf) - 1);
        
        if (id.isValid()) {
            TETHER_LOGI(tag, "║ %2zu ║ %-32s ║ %-12s ║ %-8s ║",
                     i, name_buf, vendor_buf, type);
        } else {
            TETHER_LOGI(tag, "║ %2zu ║ %-32s ║ %-12s ║ %-8s ║",
                     i, "(not found)", "-", "-");
        }
    }
    
    TETHER_LOGI(tag, "╚════╩══════════════════════════════════╩══════════════╩══════════╝");
}

} // namespace EtherCAT

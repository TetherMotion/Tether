/**
 * @file mock_ethercat_sdo.cpp
 * @brief Mock implementations of EtherCAT SDO functions for host testing
 */

#include <cstdint>
#include <cstddef>
#include <cstring>
#include <map>
#include <string>
#include <vector>
#include "MockSDOHelper.hpp"
#include "PDOManager.hpp"

// Global mock state
static std::map<uint16_t, std::string> g_mock_strings;

namespace MockSDO {
    void setString(uint16_t index, const std::string& value) {
        g_mock_strings[index] = value;
    }
    
    void clear() {
        g_mock_strings.clear();
    }
}

namespace EtherCAT {
namespace SDO {

/**
 * @brief Mock SDO read (synchronous)
 */
bool sdo_read_sync(uint16_t slave, uint16_t index, uint8_t subindex,
                  void* data, size_t size, uint32_t timeout_ms,
                  size_t* actual_size) {
    // Check if we have a mocked string for this index
    auto it = g_mock_strings.find(index);
    if (it != g_mock_strings.end()) {
        const std::string& str = it->second;
        size_t len = str.length(); 
        
        size_t copy_len = (len < size) ? len : size;
        if (data && copy_len > 0) {
            std::memcpy(data, str.c_str(), copy_len);
        }
        if (actual_size) {
             *actual_size = copy_len; // Only report what we read? 
             // Or full length?
             // Usually read returns amount of data read.
             // If loop in readString uses actual_len to check strictness...
        }
        return true; // Success
    }

    // Default behavior: Fill with zeros
    if (data) {
        std::memset(data, 0, size);
    }
    
    if (actual_size) {
        *actual_size = size;
    }
    
    return true;  // Success
}

/**
 * @brief Mock SDO write (synchronous)
 */
bool sdo_write_sync(uint16_t slave, uint16_t index, uint8_t subindex,
                   const void* data, size_t size, uint32_t timeout_ms) {
    (void)slave;
    (void)index;
    (void)subindex;
    (void)data;
    (void)size;
    (void)timeout_ms;
    
    return true;  // Success
}

} // namespace SDO

/**
 * @brief Mock RequestSlaveState
 */
bool RequestSlaveState(uint16_t slave_index, uint8_t state_code) {
    (void)slave_index;
    (void)state_code;
    return true;  // Success
}

/**
 * @brief Mock GetSlaveState
 */
bool GetSlaveState(uint16_t slave_index, uint8_t& state_code) {
    (void)slave_index;
    state_code = 0x08;  // OP state
    return true;  // Success
}

// ============================================================================
// PDO Mock Functions - implementations for the real PDOMapping class
// ============================================================================

namespace PDO {

// Include the actual header to get the class definition
// Then provide implementations for the methods

static PDOMapping g_pdo_mapping;

PDOMapping* pdo_get_mapping() {
    return &g_pdo_mapping;
}

// Implement the class methods
int PDOMapping::add_rxpdo(uint16_t slave_index, void* buffer, uint16_t size, 
                          uint16_t pdo_index, PDOAddressMode mode) {
    (void)slave_index; (void)buffer; (void)size; (void)pdo_index; (void)mode;
    return 0;  // Return entry index 0
}

int PDOMapping::add_txpdo(uint16_t slave_index, void* buffer, uint16_t size,
                          uint16_t pdo_index, PDOAddressMode mode) {
    (void)slave_index; (void)buffer; (void)size; (void)pdo_index; (void)mode;
    return 0;  // Return entry index 0
}

int PDOMapping::add_broadcast_rxpdo(void* buffer, uint16_t size, uint16_t physical_offset) {
    (void)buffer; (void)size; (void)physical_offset;
    return 0;
}

int PDOMapping::add_broadcast_txpdo(void* buffer, uint16_t size, uint16_t physical_offset) {
    (void)buffer; (void)size; (void)physical_offset;
    return 0;
}

void PDOMapping::set_slave_configured_address(uint16_t slave_index, uint16_t configured_addr) {
    (void)slave_index; (void)configured_addr;
}

const PDOEntry* PDOMapping::get_entry(size_t index) const {
    (void)index;
    return nullptr;
}

PDOEntry* PDOMapping::get_entry_mut(size_t index) {
    (void)index;
    return nullptr;
}

void PDOMapping::clear() {
    m_entry_count = 0;
}

size_t PDOMapping::total_rxpdo_bytes() const {
    return 0;
}

size_t PDOMapping::total_txpdo_bytes() const {
    return 0;
}

} // namespace pdo

} // namespace EtherCAT

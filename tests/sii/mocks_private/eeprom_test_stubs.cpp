#include <cstdint>
#include <map>
#include <mutex>
#include "ethercat/raw/internal.hpp"

namespace EtherCAT {
namespace Raw {

static std::map<uint16_t, uint32_t> s_eeprom_data;
static std::mutex s_eeprom_mutex;
static uint16_t s_last_requested_word = 0;

void test_set_eeprom_u32(uint16_t word_addr, uint32_t val) {
    std::lock_guard<std::mutex> lock(s_eeprom_mutex);
    s_eeprom_data[word_addr] = val;
}

bool ec_aprd(void* eth_handle, const uint8_t* src_mac, uint16_t adp, uint16_t reg, void* data, uint16_t len, unsigned int timeout) {
    (void)eth_handle; (void)src_mac; (void)adp; (void)timeout;
    // EEPSTAT read (16 bits)
    if (reg == EC_REG_EEPSTAT || reg == EC_REG_EEPCTL) {
        if (len >= 2) {
            uint16_t estat_le = 0; // not busy, no errors
            std::memcpy(data, &estat_le, 2);
            return true;
        }
        return false;
    }
    // EEPDAT read (32 bits)
    if (reg == EC_REG_EEPDAT) {
        uint32_t val = 0;
        std::lock_guard<std::mutex> lock(s_eeprom_mutex);
        auto it = s_eeprom_data.find(s_last_requested_word);
        if (it != s_eeprom_data.end()) val = it->second;
        uint32_t le = val; // host is little-endian in tests
        if (len >= 4) {
            std::memcpy(data, &le, 4);
            return true;
        }
        return false;
    }
    // Default: no data
    return false;
}

bool ec_apwr(void* eth_handle, const uint8_t* src_mac, uint16_t adp, uint16_t reg, const void* data, uint16_t len, unsigned int timeout) {
    (void)eth_handle; (void)src_mac; (void)adp; (void)timeout;
    // Writing EEPCTL with EepromCmd or NOP - capture requested addr
    if (reg == EC_REG_EEPCTL) {
        if (len >= sizeof(uint16_t)) {
            // EepromCmd layout: uint16_t comm_le; uint16_t addr_le; uint16_t d2_le;
            if (len >= 6) {
                uint16_t addr_le = 0;
                std::memcpy(&addr_le, reinterpret_cast<const uint8_t*>(data) + sizeof(uint16_t), sizeof(uint16_t));
                s_last_requested_word = addr_le; // little-endian
                return true;
            }
            return true;
        }
        return false;
    }

    // Accept writes to SM0/SM1
    if (reg == EC_REG_SM0 || reg == EC_REG_SM1) {
        return true;
    }

    return false;
}

} // namespace Raw
} // namespace EtherCAT

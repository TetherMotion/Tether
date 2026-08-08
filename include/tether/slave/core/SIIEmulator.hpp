// SPDX-License-Identifier: MIT
#pragma once

/**
 * @file SIIEmulator.hpp
 * @brief SII (EEPROM) emulation extracted from SlaveCore
 *
 * @details
 * Encapsulates the SII/EEPROM sub-responsibility of SlaveCore:
 *  - EEPROM data storage and word-level read/write
 *  - SII control register command processing (read/write operations)
 *  - Register write dispatch for SIIControl / SIIAddress
 *  - Default EEPROM initialization from slave identity
 */

#include <array>
#include <cstdint>
#include <bit>
#include <vector>

#include "tether/slave/core/SIIStateSlave.hpp"
#include "tether/slave/core/SlaveIdentity.hpp"
#include "tether/ethercat/ESCRegisterMap.hpp"
#include "tether/ethercat/SIIRegisters.hpp"

namespace EtherCAT { namespace slave {

class SIIEmulator {
public:
    /// @param registers  Pointer to the ESC register bank (SlaveCore::registers_).
    explicit SIIEmulator(std::array<uint8_t, 4096>* registers)
        : registers_(registers) {}

    // ---- Data access -------------------------------------------------------

    void setData(const std::vector<uint8_t>& data) {
        siiState_.eepromData = data;
    }

    const std::vector<uint8_t>& getData() const {
        return siiState_.eepromData;
    }

    uint16_t readWord(uint16_t wordAddr) const {
        size_t byteAddr = static_cast<size_t>(wordAddr) * 2;
        if (byteAddr + 1 > siiState_.eepromData.size()) {
            return 0xFFFF;
        }
        return siiState_.eepromData[byteAddr] |
               (siiState_.eepromData[byteAddr + 1] << 8);
    }

    bool writeWord(uint16_t wordAddr, uint16_t data) {
        size_t byteAddr = static_cast<size_t>(wordAddr) * 2;
        if (byteAddr + 1 > siiState_.eepromData.size()) {
            return false;
        }
        siiState_.eepromData[byteAddr] = data & 0xFF;
        siiState_.eepromData[byteAddr + 1] = (data >> 8) & 0xFF;
        return true;
    }

    // ---- Command processing -----------------------------------------------

    /// @brief Process the current SII control command (read or write op).
    void processCommand();

    /// @brief Handle a register write that targets SIIControl or SIIAddress.
    /// Called from SlaveCore::writeRegister for SII-related register addresses.
    void handleRegisterWrite(uint16_t addr, const uint8_t* data, uint16_t len);

    // ---- Initialization ----------------------------------------------------

    /// @brief Create minimal SII/EEPROM data from a slave identity.
    void initializeFromIdentity(const SlaveIdentity& identity);

    /// @brief Direct access to the SII state (for SlaveCore compatibility).
    SIIState& state() { return siiState_; }
    const SIIState& state() const { return siiState_; }

private:
    std::array<uint8_t, 4096>* registers_;
    SIIState siiState_;
};

}} // namespace EtherCAT::slave

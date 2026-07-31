/**
 * @file TmcUart.hpp
 * @brief TMC stepper driver UART interface.
 *
 * Provides:
 *   - TmcUart: UART interface for TMC2209, TMC2208, TMC5160, etc.
 *   - TmcField: register field accessor
 *   - TmcRegister: register read/write via UART
 */

#pragma once

#include <cstdint>
#include <functional>
#include <string>
#include <vector>

namespace tether::klipper::objects {

/// @brief TMC stepper driver UART interface.
class TmcUart {
public:
    using UartTransferFunc = std::function<std::vector<uint8_t>(std::span<const uint8_t>)>;

    TmcUart(uint8_t oid, uint8_t slaveAddr, UartTransferFunc transfer)
        : oid_(oid)
        , slaveAddr_(slaveAddr)
        , transfer_(std::move(transfer)) {}

    uint8_t oid() const { return oid_; }
    uint8_t slaveAddress() const { return slaveAddr_; }

    /// @brief Read a register from the TMC driver.
    /// @param reg Register address (7 bits).
    /// @return 32-bit register value, or -1 on error.
    int64_t readRegister(uint8_t reg) {
        // TMC UART format: sync byte, slave addr, register+R/W, CRC, data
        // Read request: 0x05, slave, 0x00 | reg, CRC
        std::vector<uint8_t> req = {
            0x05,                                    // Sync
            static_cast<uint8_t>(slaveAddr_),        // Slave address
            static_cast<uint8_t>(reg & 0x7F),        // Register (read = MSB=0)
        };
        uint8_t crc = computeCrc(req);
        req.push_back(crc);

        auto resp = transfer_(req);
        if (resp.size() < 8) return -1;
        // Response: sync(0x05), slave, addr|0x80, CRC, 4 data bytes, CRC
        if (resp[0] != 0x05) return -1;
        uint32_t value = (static_cast<uint32_t>(resp[4]) << 24) |
                         (static_cast<uint32_t>(resp[5]) << 16) |
                         (static_cast<uint32_t>(resp[6]) << 8) |
                         static_cast<uint32_t>(resp[7]);
        return value;
    }

    /// @brief Write a register to the TMC driver.
    /// @param reg Register address (7 bits).
    /// @param value 32-bit value to write.
    /// @return True on success.
    bool writeRegister(uint8_t reg, uint32_t value) {
        // Write request: 0x05, slave, 0x80 | reg, CRC, 4 data bytes, CRC
        std::vector<uint8_t> req = {
            0x05,                                    // Sync
            static_cast<uint8_t>(slaveAddr_),        // Slave address
            static_cast<uint8_t>(0x80 | (reg & 0x7F)), // Register (write = MSB=1)
        };
        uint8_t crc1 = computeCrc(req);
        req.push_back(crc1);
        req.push_back(static_cast<uint8_t>((value >> 24) & 0xFF));
        req.push_back(static_cast<uint8_t>((value >> 16) & 0xFF));
        req.push_back(static_cast<uint8_t>((value >> 8) & 0xFF));
        req.push_back(static_cast<uint8_t>(value & 0xFF));

        // CRC over all bytes except the first CRC
        std::vector<uint8_t> crcInput(req.begin() + 4, req.end());
        uint8_t crc2 = computeCrc(crcInput);
        req.push_back(crc2);

        auto resp = transfer_(req);
        return resp.size() >= 4; // Simple ACK check
    }

    /// @brief Set a field within a register.
    bool setField(uint8_t reg, uint8_t offset, uint8_t width, uint32_t value) {
        int64_t current = readRegister(reg);
        if (current < 0) return false;
        uint32_t mask = (1u << width) - 1;
        uint32_t newVal = (current & ~(mask << offset)) | ((value & mask) << offset);
        return writeRegister(reg, newVal);
    }

    /// @brief Get a field within a register.
    int32_t getField(uint8_t reg, uint8_t offset, uint8_t width) {
        int64_t current = readRegister(reg);
        if (current < 0) return -1;
        uint32_t mask = (1u << width) - 1;
        return (current >> offset) & mask;
    }

private:
    /// @brief Compute TMC UART CRC (XOR of all bytes).
    uint8_t computeCrc(const std::vector<uint8_t>& data) {
        uint8_t crc = 0;
        for (uint8_t b : data) crc ^= b;
        return crc;
    }

    uint8_t oid_;
    uint8_t slaveAddr_;
    UartTransferFunc transfer_;
};

} // namespace tether::klipper::objects

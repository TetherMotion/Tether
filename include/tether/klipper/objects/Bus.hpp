/**
 * @file Bus.hpp
 * @brief SPI and I2C bus peripherals and proxies.
 */

#pragma once

#include <cstdint>
#include <functional>
#include <span>
#include <vector>

namespace tether::klipper::objects {

// ============================================================================
// SPI
// ============================================================================

/// @brief SPI peripheral.
class Spi {
public:
    using TransferFunc = std::function<std::vector<uint8_t>(std::span<const uint8_t>)>;

    Spi(uint8_t oid) : oid_(oid) {}
    Spi(uint8_t oid, TransferFunc transfer)
        : oid_(oid), transfer_(std::move(transfer)) {}

    uint8_t oid() const { return oid_; }

    /// @brief Transfer data over SPI.
    std::vector<uint8_t> transfer(std::span<const uint8_t> data) {
        if (transfer_) return transfer_(data);
        // Default: return same size vector of zeros
        return std::vector<uint8_t>(data.size(), 0);
    }

    /// @brief Transfer data over SPI (vector overload).
    std::vector<uint8_t> transfer(const std::vector<uint8_t>& data) {
        return transfer(std::span<const uint8_t>(data));
    }

    /// @brief Set the transfer function.
    void setTransferFunc(TransferFunc func) { transfer_ = std::move(func); }

private:
    uint8_t oid_;
    TransferFunc transfer_;
};

/// @brief Proxy for SPI that echoes or stores data.
class SpiProxy {
public:
    explicit SpiProxy(uint8_t oid) : oid_(oid) {}

    uint8_t oid() const { return oid_; }

    std::vector<uint8_t> transfer(std::span<const uint8_t> data) {
        lastData_.assign(data.begin(), data.end());
        return std::vector<uint8_t>(data.size(), 0);
    }

    const std::vector<uint8_t>& lastData() const { return lastData_; }

private:
    uint8_t oid_;
    std::vector<uint8_t> lastData_;
};

// ============================================================================
// I2C
// ============================================================================

/// @brief I2C peripheral.
class I2c {
public:
    using ReadFunc = std::function<std::vector<uint8_t>(uint8_t addr, uint8_t reg, size_t len)>;
    using WriteFunc = std::function<void(uint8_t addr, uint8_t reg, std::span<const uint8_t> data)>;
    using ReadNoRegFunc = std::function<std::vector<uint8_t>(uint8_t addr, size_t len)>;
    using WriteNoRegFunc = std::function<void(uint8_t addr, std::span<const uint8_t> data)>;
    using Read16Func = std::function<std::vector<uint8_t>(uint8_t addr, uint16_t reg, size_t len)>;
    using Write16Func = std::function<void(uint8_t addr, uint16_t reg, std::span<const uint8_t> data)>;

    I2c(uint8_t oid) : oid_(oid) {}
    I2c(uint8_t oid, ReadFunc readFunc, WriteFunc writeFunc)
        : oid_(oid), readFunc_(std::move(readFunc)), writeFunc_(std::move(writeFunc)) {}

    uint8_t oid() const { return oid_; }

    /// @brief Read from an I2C device with an 8-bit register address.
    std::vector<uint8_t> read(uint8_t addr, uint8_t reg, size_t len) {
        if (readFunc_) return readFunc_(addr, reg, len);
        return std::vector<uint8_t>(len, 0);
    }

    /// @brief Read from an I2C device without register addressing.
    /// Some devices (e.g. MLX90614) don't use register addresses.
    std::vector<uint8_t> readNoRegister(uint8_t addr, size_t len) {
        if (readNoRegFunc_) return readNoRegFunc_(addr, len);
        if (readFunc_) return readFunc_(addr, 0, len); // Fallback
        return std::vector<uint8_t>(len, 0);
    }

    /// @brief Read from an I2C device with a 16-bit register address.
    /// Used by devices with many registers (e.g. EEPROM, large sensors).
    std::vector<uint8_t> read16(uint8_t addr, uint16_t reg, size_t len) {
        if (read16Func_) return read16Func_(addr, reg, len);
        // Fallback: split 16-bit reg into two 8-bit writes
        if (readFunc_) return readFunc_(addr, static_cast<uint8_t>(reg >> 8), len);
        return std::vector<uint8_t>(len, 0);
    }

    /// @brief Write to an I2C device with an 8-bit register address.
    bool write(uint8_t addr, uint8_t reg, std::span<const uint8_t> data) {
        if (writeFunc_) { writeFunc_(addr, reg, data); return true; }
        return true; // Return true even without writeFunc (no-op success)
        }

    /// @brief Write to an I2C device without register addressing.
    bool writeNoRegister(uint8_t addr, std::span<const uint8_t> data) {
        if (writeNoRegFunc_) { writeNoRegFunc_(addr, data); return true; }
        if (writeFunc_) { writeFunc_(addr, 0, data); return true; }
        return true;
    }

    /// @brief Write to an I2C device with a 16-bit register address.
    bool write16(uint8_t addr, uint16_t reg, std::span<const uint8_t> data) {
        if (write16Func_) { write16Func_(addr, reg, data); return true; }
        if (writeFunc_) { writeFunc_(addr, static_cast<uint8_t>(reg >> 8), data); return true; }
        return true;
    }

    // Convenience overloads
    std::vector<uint8_t> read(uint8_t addr, size_t len) {
        return readNoRegister(addr, len);
    }
    bool write(uint8_t addr, std::span<const uint8_t> data) {
        return writeNoRegister(addr, data);
    }
    bool write(uint8_t addr, const std::vector<uint8_t>& data) {
        return writeNoRegister(addr, std::span<const uint8_t>(data));
    }

    void setReadFunc(ReadFunc func) { readFunc_ = std::move(func); }
    void setWriteFunc(WriteFunc func) { writeFunc_ = std::move(func); }
    void setReadNoRegFunc(ReadNoRegFunc func) { readNoRegFunc_ = std::move(func); }
    void setWriteNoRegFunc(WriteNoRegFunc func) { writeNoRegFunc_ = std::move(func); }
    void setRead16Func(Read16Func func) { read16Func_ = std::move(func); }
    void setWrite16Func(Write16Func func) { write16Func_ = std::move(func); }

private:
    uint8_t oid_;
    ReadFunc readFunc_;
    WriteFunc writeFunc_;
    ReadNoRegFunc readNoRegFunc_;
    WriteNoRegFunc writeNoRegFunc_;
    Read16Func read16Func_;
    Write16Func write16Func_;
};

/// @brief Proxy for I2C that stores data.
class I2cProxy {
public:
    explicit I2cProxy(uint8_t oid) : oid_(oid) {}

    uint8_t oid() const { return oid_; }

    std::vector<uint8_t> read(uint8_t addr, uint8_t reg, size_t len) {
        return std::vector<uint8_t>(len, 0);
    }

    void write(uint8_t addr, uint8_t reg, std::span<const uint8_t> data) {
        lastAddr_ = addr;
        lastReg_ = reg;
        lastData_.assign(data.begin(), data.end());
    }

    uint8_t lastAddr() const { return lastAddr_; }
    uint8_t lastReg() const { return lastReg_; }
    const std::vector<uint8_t>& lastData() const { return lastData_; }

private:
    uint8_t oid_;
    uint8_t lastAddr_ = 0;
    uint8_t lastReg_ = 0;
    std::vector<uint8_t> lastData_;
};

} // namespace tether::klipper::objects

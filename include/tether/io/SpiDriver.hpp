/**
 * @file SpiDriver.hpp
 * @brief Abstract SPI driver interface and POSIX implementation.
 *
 * @details
 * Provides a simple SPI driver abstraction for use by Klipper peripherals
 * (ADXL345, thermocouples, etc.). The POSIX implementation opens /dev/spidevX.Y
 * devices and performs full-duplex transfers via ioctl(SPIOC_MESSAGE).
 *
 * On embedded targets (ESP-IDF), users can provide their own ISpiDriver
 * implementation backed by the SPI HAL.
 *
 * @copyright Copyright (C) 2025-2026 Tether Authors
 */

#pragma once

#include <cstdint>
#include <cstddef>
#include <vector>
#include <span>
#include <string>

namespace tether { namespace io {

/**
 * @class ISpiDriver
 * @brief Abstract SPI bus driver.
 *
 * Implementations should provide full-duplex SPI transfers.
 */
class ISpiDriver {
public:
    virtual ~ISpiDriver() = default;

    /// Open the SPI device. Returns true on success.
    /// @param device Device path (e.g. "/dev/spidev0.0").
    /// @param mode SPI mode (0-3).
    /// @param speedHz Clock speed in Hz.
    /// @param bitsPerWord Bits per word (usually 8).
    virtual bool open(const char* device, uint8_t mode, uint32_t speedHz,
                      uint8_t bitsPerWord = 8) = 0;

    /// Close the SPI device.
    virtual void close() = 0;

    /// Full-duplex transfer: writes txLen bytes and reads back the same count.
    /// @param tx Data to send.
    /// @param rx Buffer for received data (must be at least tx.size() bytes).
    /// @return Number of bytes transferred, or 0 on error.
    virtual size_t transfer(std::span<const uint8_t> tx, uint8_t* rx) = 0;

    /// Convenience: full-duplex transfer returning a vector.
    std::vector<uint8_t> transfer(std::span<const uint8_t> tx) {
        std::vector<uint8_t> rx(tx.size());
        size_t n = transfer(tx, rx.data());
        rx.resize(n);
        return rx;
    }

    /// Returns true if the device is open.
    virtual bool isOpen() const = 0;
};

#if !defined(ESP_PLATFORM)
/**
 * @class PosixSpiDriver
 * @brief POSIX SPI driver implementation for Linux.
 *
 * Uses /dev/spidevX.Y devices with ioctl(SPIOC_MESSAGE) for transfers.
 */
class PosixSpiDriver : public ISpiDriver {
public:
    PosixSpiDriver() = default;
    ~PosixSpiDriver() override;

    bool open(const char* device, uint8_t mode, uint32_t speedHz,
              uint8_t bitsPerWord = 8) override;
    void close() override;
    size_t transfer(std::span<const uint8_t> tx, uint8_t* rx) override;
    bool isOpen() const override;

    // Bring in the convenience overload from the base class
    using ISpiDriver::transfer;

private:
    int fd_ = -1;
};
#endif // !ESP_PLATFORM

}} // namespace tether::io

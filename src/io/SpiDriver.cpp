/**
 * @file SpiDriver.cpp
 * @brief POSIX implementation of SPI driver using /dev/spidevX.Y.
 */

#include "tether/io/SpiDriver.hpp"

#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/spi/spidev.h>
#include <cstring>
#include <stdexcept>

namespace tether { namespace io {

#if !defined(ESP_PLATFORM)

PosixSpiDriver::~PosixSpiDriver() {
    close();
}

bool PosixSpiDriver::open(const char* device, uint8_t mode, uint32_t speedHz,
                          uint8_t bitsPerWord) {
    if (fd_ >= 0) return true;

    fd_ = ::open(device, O_RDWR);
    if (fd_ < 0) return false;

    // Set SPI mode
    uint8_t spiMode = mode;
    if (::ioctl(fd_, SPI_IOC_WR_MODE, &spiMode) < 0) {
        ::close(fd_);
        fd_ = -1;
        return false;
    }

    // Set bits per word
    uint8_t bpw = bitsPerWord;
    if (::ioctl(fd_, SPI_IOC_WR_BITS_PER_WORD, &bpw) < 0) {
        ::close(fd_);
        fd_ = -1;
        return false;
    }

    // Set max speed
    uint32_t speed = speedHz;
    if (::ioctl(fd_, SPI_IOC_WR_MAX_SPEED_HZ, &speed) < 0) {
        ::close(fd_);
        fd_ = -1;
        return false;
    }

    return true;
}

void PosixSpiDriver::close() {
    if (fd_ >= 0) {
        ::close(fd_);
        fd_ = -1;
    }
}

size_t PosixSpiDriver::transfer(std::span<const uint8_t> tx, uint8_t* rx) {
    if (fd_ < 0 || tx.empty()) return 0;

    struct spi_ioc_transfer tr;
    std::memset(&tr, 0, sizeof(tr));
    tr.tx_buf = reinterpret_cast<uint64_t>(tx.data());
    tr.rx_buf = reinterpret_cast<uint64_t>(rx);
    tr.len = tx.size();
    tr.speed_hz = 0;        // Use previously set speed
    tr.delay_usecs = 0;
    tr.bits_per_word = 0;   // Use previously set bits
    tr.cs_change = 0;

    int ret = ::ioctl(fd_, SPI_IOC_MESSAGE(1), &tr);
    if (ret < 1) return 0;
    return tx.size();
}

bool PosixSpiDriver::isOpen() const {
    return fd_ >= 0;
}

#endif // !ESP_PLATFORM

}} // namespace tether::io

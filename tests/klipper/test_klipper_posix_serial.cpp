/**
 * @file test_klipper_posix_serial.cpp
 * @brief Tests for PosixSerialByteStream adapter using a mock ISerialDriver.
 */

#include <gtest/gtest.h>
#include "tether/klipper/transport/PosixSerialByteStream.hpp"
#include "tether/io/SerialTransport.hpp"

#include <vector>
#include <cstring>

using namespace tether::klipper::transport;
using tether::io::ISerialDriver;

/// @brief Mock serial driver for testing the adapter.
class MockSerialDriver : public ISerialDriver {
public:
    bool open(const char* port, uint32_t baudRate) override {
        port_ = port;
        baudRate_ = baudRate;
        opened_ = true;
        return true;
    }

    void close() override { opened_ = false; }

    size_t write(const uint8_t* data, size_t len) override {
        if (!opened_) return 0;
        writeBuf_.insert(writeBuf_.end(), data, data + len);
        return len;
    }

    size_t read(uint8_t* buf, size_t maxLen, uint32_t timeoutMs) override {
        (void)timeoutMs;
        if (!opened_ || readBuf_.empty()) return 0;
        size_t n = std::min(maxLen, readBuf_.size());
        std::memcpy(buf, readBuf_.data(), n);
        readBuf_.erase(readBuf_.begin(), readBuf_.begin() + n);
        return n;
    }

    bool isOpen() const override { return opened_; }

    // Test helpers
    void injectReadData(const std::vector<uint8_t>& data) {
        readBuf_.insert(readBuf_.end(), data.begin(), data.end());
    }

    std::vector<uint8_t> writtenData() const { return writeBuf_; }
    const std::string& port() const { return port_; }
    uint32_t baudRate() const { return baudRate_; }

private:
    bool opened_ = false;
    std::string port_;
    uint32_t baudRate_ = 0;
    std::vector<uint8_t> writeBuf_;
    std::vector<uint8_t> readBuf_;
};

TEST(PosixSerialByteStreamTest, OpenClose) {
    auto mock = std::make_unique<MockSerialDriver>();
    auto* raw = mock.get();
    PosixSerialByteStream transport(std::move(mock), "/dev/ttyUSB0", 250000);

    EXPECT_FALSE(transport.isOpen());
    EXPECT_TRUE(transport.open());
    EXPECT_TRUE(transport.isOpen());
    EXPECT_EQ(raw->port(), "/dev/ttyUSB0");
    EXPECT_EQ(raw->baudRate(), 250000);

    transport.close();
    EXPECT_FALSE(transport.isOpen());
}

TEST(PosixSerialByteStreamTest, WriteData) {
    auto mock = std::make_unique<MockSerialDriver>();
    auto* raw = mock.get();
    PosixSerialByteStream transport(std::move(mock), "/dev/ttyUSB0", 115200);
    ASSERT_TRUE(transport.open());

    std::vector<uint8_t> data = {0x01, 0x02, 0x03, 0x04, 0x05};
    size_t written = transport.write(data);
    EXPECT_EQ(written, 5u);
    EXPECT_EQ(raw->writtenData(), data);
}

TEST(PosixSerialByteStreamTest, ReadData) {
    auto mock = std::make_unique<MockSerialDriver>();
    auto* raw = mock.get();
    PosixSerialByteStream transport(std::move(mock), "/dev/ttyUSB0", 115200);
    ASSERT_TRUE(transport.open());

    // Inject data to read
    std::vector<uint8_t> injected = {0xAA, 0xBB, 0xCC};
    raw->injectReadData(injected);

    uint8_t buf[16] = {};
    size_t n = transport.read(buf, sizeof(buf), false);
    EXPECT_EQ(n, 3u);
    EXPECT_EQ(buf[0], 0xAA);
    EXPECT_EQ(buf[1], 0xBB);
    EXPECT_EQ(buf[2], 0xCC);
}

TEST(PosixSerialByteStreamTest, ReadEmpty) {
    auto mock = std::make_unique<MockSerialDriver>();
    PosixSerialByteStream transport(std::move(mock), "/dev/ttyUSB0", 115200);
    ASSERT_TRUE(transport.open());

    uint8_t buf[16] = {};
    size_t n = transport.read(buf, sizeof(buf), false);
    EXPECT_EQ(n, 0u);
}

TEST(PosixSerialByteStreamTest, WriteBeforeOpen) {
    auto mock = std::make_unique<MockSerialDriver>();
    PosixSerialByteStream transport(std::move(mock), "/dev/ttyUSB0", 115200);

    std::vector<uint8_t> data = {0x01, 0x02};
    EXPECT_EQ(transport.write(data), 0u);
}

TEST(PosixSerialByteStreamTest, PortAndBaudAccessors) {
    PosixSerialByteStream transport("/dev/ttyACM0", 500000);
    EXPECT_EQ(transport.port(), "/dev/ttyACM0");
    EXPECT_EQ(transport.baudRate(), 500000u);
}

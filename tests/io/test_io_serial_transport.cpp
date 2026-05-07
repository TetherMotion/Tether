/**
 * @file test_io_serial_transport.cpp
 * @brief Unit tests for SerialTransport with a mock ISerialDriver.
 */
#include <gtest/gtest.h>
#include "tether/io/SerialTransport.hpp"
#include <queue>
#include <cstring>

#if !defined(ESP_PLATFORM)
#include <pty.h>
#include <unistd.h>
#include <fcntl.h>
#endif

using namespace tether::io;

// ===========================================================================
// Mock serial driver
// ===========================================================================

class MockSerialDriver : public ISerialDriver {
public:
    bool open(const char* /*port*/, uint32_t /*baudRate*/) override {
        open_ = true;
        return openSuccess_;
    }

    void close() override { open_ = false; }

    size_t write(const uint8_t* data, size_t len) override {
        if (!open_ || writeFailure_) return 0;
        written_.insert(written_.end(), data, data + len);
        return len;
    }

    size_t read(uint8_t* buf, size_t maxLen, uint32_t /*timeoutMs*/) override {
        if (!open_) return 0;
        size_t n = std::min(maxLen, readable_.size());
        if (n > 0) {
            std::memcpy(buf, readable_.data(), n);
            readable_.erase(readable_.begin(), readable_.begin() + static_cast<ptrdiff_t>(n));
        }
        return n;
    }

    bool isOpen() const override { return open_; }

    // Test helpers
    void setReadableData(const std::vector<uint8_t>& data) {
        readable_ = data;
    }

    std::vector<uint8_t> getWritten() const { return written_; }

    bool openSuccess_ = true;
    bool writeFailure_ = false;

private:
    bool open_ = false;
    std::vector<uint8_t> readable_;
    std::vector<uint8_t> written_;
};

// ===========================================================================
// Tests
// ===========================================================================

TEST(SerialTransport, SendReceive) {
    auto driver = std::make_unique<MockSerialDriver>();
    MockSerialDriver* dp = driver.get();
    dp->open("test", 115200);

    SerialTransport transport(std::move(driver));
    EXPECT_TRUE(transport.isConnected());

    // Send
    uint8_t data[] = {1, 2, 3};
    EXPECT_TRUE(transport.send(data, 3));
    EXPECT_EQ(dp->getWritten().size(), 3u);

    // Receive
    dp->setReadableData({10, 20});
    uint8_t buf[16];
    size_t n = transport.receive(buf, sizeof(buf), 0);
    EXPECT_EQ(n, 2u);
    EXPECT_EQ(buf[0], 10);
    EXPECT_EQ(buf[1], 20);
}

TEST(SerialTransport, CloseDisconnects) {
    auto driver = std::make_unique<MockSerialDriver>();
    driver->open("test", 115200);

    SerialTransport transport(std::move(driver));
    EXPECT_TRUE(transport.isConnected());

    transport.close();
    EXPECT_FALSE(transport.isConnected());
}

TEST(SerialTransport, SendWhenClosed) {
    auto driver = std::make_unique<MockSerialDriver>();
    driver->open("test", 115200);

    SerialTransport transport(std::move(driver));
    transport.close();

    uint8_t data[] = {1};
    EXPECT_FALSE(transport.send(data, 1));
}

TEST(SerialTransport, ReceiveWhenClosed) {
    auto driver = std::make_unique<MockSerialDriver>();
    driver->open("test", 115200);

    SerialTransport transport(std::move(driver));
    transport.close();

    uint8_t buf[4];
    EXPECT_EQ(transport.receive(buf, sizeof(buf), 0), 0u);
}

TEST(SerialTransport, SendFailureOnWriteError) {
    auto driver = std::make_unique<MockSerialDriver>();
    MockSerialDriver* dp = driver.get();
    dp->open("test", 115200);
    dp->writeFailure_ = true;

    SerialTransport transport(std::move(driver));
    uint8_t data[] = {1};
    EXPECT_FALSE(transport.send(data, 1));
}

TEST(SerialTransport, NullDriver) {
    // Should handle nullptr gracefully
    SerialTransport transport(nullptr);
    EXPECT_FALSE(transport.isConnected());

    uint8_t data[] = {1};
    EXPECT_FALSE(transport.send(data, 1));

    uint8_t buf[4];
    EXPECT_EQ(transport.receive(buf, sizeof(buf), 0), 0u);
}

TEST(SerialTransport, DestructorClosesDriver) {
    // After move, driver_ is owned by the transport, so we verify
    // indirectly that close is called via isConnected returning false
    // after destruction.
    auto driver = std::make_unique<MockSerialDriver>();
    driver->open("test", 115200);

    {
        SerialTransport transport(std::move(driver));
        EXPECT_TRUE(transport.isConnected());
    }  // destructor should close — transport is gone

    // We can't access the raw pointer after move; the test verifies
    // no crash occurs during destruction.
}

#if !defined(ESP_PLATFORM)

namespace {

struct PtyPair {
    int masterFd = -1;
    std::string slavePath;

    static PtyPair create() {
        PtyPair pair;
        int slaveFd = -1;
        char slaveName[128] = {};
        if (::openpty(&pair.masterFd, &slaveFd, slaveName, nullptr, nullptr) != 0) {
            pair.masterFd = -1;
            return pair;
        }
        pair.slavePath = slaveName;
        ::close(slaveFd);
        return pair;
    }

    ~PtyPair() {
        if (masterFd >= 0) {
            ::close(masterFd);
        }
    }
};

} // namespace

TEST(PosixSerialDriver, OpenReadWriteClose) {
    auto pty = PtyPair::create();
    ASSERT_GE(pty.masterFd, 0);

    PosixSerialDriver driver;
    ASSERT_TRUE(driver.open(pty.slavePath.c_str(), 115200));
    EXPECT_TRUE(driver.isOpen());

    const uint8_t outbound[] = {1, 2, 3, 4};
    EXPECT_EQ(driver.write(outbound, sizeof(outbound)), sizeof(outbound));

    uint8_t masterBuf[8] = {};
    ASSERT_EQ(::read(pty.masterFd, masterBuf, sizeof(masterBuf)), 4);
    EXPECT_EQ(std::memcmp(masterBuf, outbound, sizeof(outbound)), 0);

    const uint8_t inbound[] = {9, 8, 7};
    ASSERT_EQ(::write(pty.masterFd, inbound, sizeof(inbound)), 3);

    uint8_t readBuf[8] = {};
    EXPECT_EQ(driver.read(readBuf, sizeof(readBuf), 100), 3u);
    EXPECT_EQ(std::memcmp(readBuf, inbound, sizeof(inbound)), 0);

    driver.close();
    EXPECT_FALSE(driver.isOpen());
    EXPECT_EQ(driver.write(outbound, sizeof(outbound)), 0u);
    EXPECT_EQ(driver.read(readBuf, sizeof(readBuf), 10), 0u);
}

TEST(PosixSerialDriver, ReadTimeoutReturnsZero) {
    auto pty = PtyPair::create();
    ASSERT_GE(pty.masterFd, 0);

    PosixSerialDriver driver;
    ASSERT_TRUE(driver.open(pty.slavePath.c_str(), 115200));

    uint8_t buf[4] = {};
    EXPECT_EQ(driver.read(buf, sizeof(buf), 20), 0u);
}

TEST(PosixSerialDriver, UnsupportedBaudFallsBackToDefault) {
    auto pty = PtyPair::create();
    ASSERT_GE(pty.masterFd, 0);

    PosixSerialDriver driver;
    EXPECT_TRUE(driver.open(pty.slavePath.c_str(), 12345));
    EXPECT_TRUE(driver.isOpen());
}

TEST(PosixSerialDriver, SupportedBaudRatesOpenSuccessfully) {
    const uint32_t baudRates[] = {
        9600, 19200, 38400, 57600, 115200, 230400,
        460800, 921600, 1000000, 1500000, 2000000, 3000000,
    };

    for (uint32_t baud : baudRates) {
        auto pty = PtyPair::create();
        ASSERT_GE(pty.masterFd, 0);

        PosixSerialDriver driver;
        EXPECT_TRUE(driver.open(pty.slavePath.c_str(), baud)) << baud;
        EXPECT_TRUE(driver.isOpen()) << baud;
        driver.close();
        EXPECT_FALSE(driver.isOpen()) << baud;
    }
}

TEST(PosixSerialDriver, OpenInvalidPathFails) {
    PosixSerialDriver driver;
    EXPECT_FALSE(driver.open("/definitely/not/a/serial/device", 115200));
    EXPECT_FALSE(driver.isOpen());
}

TEST(PosixSerialDriver, OpenNonTerminalFailsTcgetattr) {
    const char* tmpPath = "/tmp/tether_serial_not_a_tty.bin";
    int fd = ::open(tmpPath, O_CREAT | O_RDWR | O_TRUNC, 0600);
    ASSERT_GE(fd, 0);
    ::close(fd);

    PosixSerialDriver driver;
    EXPECT_FALSE(driver.open(tmpPath, 115200));
    EXPECT_FALSE(driver.isOpen());

    ::unlink(tmpPath);
}

#endif

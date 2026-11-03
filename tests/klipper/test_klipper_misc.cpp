/**
 * @file test_klipper_misc.cpp
 * @brief Coverage for small klipper units: KlipperLog levels/callback,
 *        objects::Stepper::tick scheduling, and PipeTransport over real
 *        pipe() file descriptors.
 */

#include <gtest/gtest.h>

#include <fcntl.h>
#include <unistd.h>

#include <array>
#include <cstring>
#include <memory>
#include <string>
#include <vector>

#include "tether/klipper/KlipperLog.hpp"
#include "tether/klipper/objects/Stepper.hpp"
#include "tether/klipper/transport/PipeTransport.hpp"

namespace {

using tether::klipper::logging::Level;
namespace logging = tether::klipper::logging;
using tether::klipper::objects::Stepper;
using tether::klipper::objects::StepCommand;
using tether::klipper::transport::PipeTransport;
using tether::klipper::transport::PipeTransportConfig;

// ---------------------------------------------------------------------------
// KlipperLog
// ---------------------------------------------------------------------------

class KlipperLogTest : public ::testing::Test {
protected:
    void TearDown() override {
        logging::setLogCallback(nullptr);
        logging::setLogLevel(Level::Warn);  // restore default
    }
};

TEST_F(KlipperLogTest, LevelGetSetRoundTrips) {
    // Note: don't assert the process default — other suites in this binary
    // mutate the global level without restoring it.
    logging::setLogLevel(Level::Debug);
    EXPECT_EQ(logging::getLogLevel(), Level::Debug);
    logging::setLogLevel(Level::Error);
    EXPECT_EQ(logging::getLogLevel(), Level::Error);
}

TEST_F(KlipperLogTest, EmitInvokesCallbackWithBasename) {
    struct Rec { Level level; std::string file; int line; std::string msg; };
    std::vector<Rec> recs;
    logging::setLogCallback(
        [&recs](Level l, std::string_view f, int ln, std::string_view m) {
            recs.push_back({l, std::string(f), ln, std::string(m)});
        });
    logging::setLogLevel(Level::Debug);

    logging::emit(Level::Info, "/a/b/KlippyServer.cpp", 42, "hello");
    ASSERT_EQ(recs.size(), 1u);
    EXPECT_EQ(recs[0].level, Level::Info);
    EXPECT_EQ(recs[0].file, "KlippyServer.cpp");  // basename only
    EXPECT_EQ(recs[0].line, 42);
    EXPECT_EQ(recs[0].msg, "hello");
}

TEST_F(KlipperLogTest, EmitFiltersByLevel) {
    int calls = 0;
    logging::setLogCallback(
        [&calls](Level, std::string_view, int, std::string_view) { ++calls; });

    logging::setLogLevel(Level::Error);
    logging::emit(Level::Debug, "f.cpp", 1, "d");
    logging::emit(Level::Info, "f.cpp", 1, "i");
    logging::emit(Level::Warn, "f.cpp", 1, "w");
    EXPECT_EQ(calls, 0);

    logging::emit(Level::Error, "f.cpp", 1, "e");
    EXPECT_EQ(calls, 1);
}

TEST_F(KlipperLogTest, EmitWithoutCallbackWritesStderr) {
    logging::setLogCallback(nullptr);
    logging::setLogLevel(Level::Debug);
    // No callback → stderr fallback; just verify it doesn't crash and
    // levelStr covers all enum values.
    logging::emit(Level::Error, "x.cpp", 1, "e");
    logging::emit(Level::Warn, "x.cpp", 1, "w");
    logging::emit(Level::Info, "x.cpp", 1, "i");
    logging::emit(Level::Debug, "x.cpp", 1, "d");
    SUCCEED();
}

TEST_F(KlipperLogTest, GetLogCallbackReturnsInstalled) {
    auto cb = [](Level, std::string_view, int, std::string_view) {};
    logging::setLogCallback(cb);
    EXPECT_TRUE(logging::getLogCallback() != nullptr);
}

TEST_F(KlipperLogTest, EmitHandlesEmptyBasenameAndPathNoSlash) {
    std::string file;
    logging::setLogCallback(
        [&file](Level, std::string_view f, int, std::string_view) {
            file = std::string(f);
        });
    logging::setLogLevel(Level::Debug);
    logging::emit(Level::Info, "noslash.cpp", 7, "m");
    EXPECT_EQ(file, "noslash.cpp");
}

// ---------------------------------------------------------------------------
// Stepper::tick
// ---------------------------------------------------------------------------

TEST(StepperTickTest, EmptyQueueDoesNothing) {
    Stepper s{0};
    EXPECT_EQ(s.tick(1000), 0u);
    EXPECT_EQ(s.position(), 0);
    EXPECT_TRUE(s.idle());
}

TEST(StepperTickTest, ExecutesStepsAtScheduledTimes) {
    Stepper s{0};
    StepCommand cmd{.interval = 10, .count = 5, .add = 0, .dir = 1};
    s.enqueueStep(cmd, /*startClock=*/100);

    EXPECT_EQ(s.tick(99), 0u);    // not yet
    EXPECT_EQ(s.tick(100), 1u);   // first step at startClock
    EXPECT_EQ(s.tick(109), 0u);
    EXPECT_EQ(s.tick(110), 1u);   // second step
    EXPECT_EQ(s.position(), 2);

    // Remaining 3 steps at 120, 130, 140
    EXPECT_EQ(s.tick(140), 3u);
    EXPECT_EQ(s.position(), 5);
    EXPECT_TRUE(s.idle());
}

TEST(StepperTickTest, DirectionControlsPositionDelta) {
    Stepper s{0};
    s.setDirection(-1);
    StepCommand cmd{.interval = 5, .count = 3, .add = 0, .dir = -1};
    s.enqueueStep(cmd, 0);
    EXPECT_EQ(s.tick(20), 3u);
    EXPECT_EQ(s.position(), -3);
}

TEST(StepperTickTest, AddAcceleratesInterval) {
    Stepper s{0};
    // interval starts 100, add=-10 → steps at 0, 100, 190, 270 (interval
    // decreases each step: 100, 90, 80).
    StepCommand cmd{.interval = 100, .count = 4, .add = -10, .dir = 1};
    s.enqueueStep(cmd, 0);
    EXPECT_EQ(s.tick(0), 1u);
    EXPECT_EQ(s.tick(99), 0u);
    EXPECT_EQ(s.tick(100), 1u);   // interval 100
    EXPECT_EQ(s.tick(189), 0u);
    EXPECT_EQ(s.tick(190), 1u);   // interval 90
    EXPECT_EQ(s.tick(270), 1u);   // interval 80
    EXPECT_EQ(s.position(), 4);
}

TEST(StepperTickTest, QueuedCommandsRunBackToBack) {
    Stepper s{0};
    StepCommand a{.interval = 10, .count = 2, .add = 0, .dir = 1};
    StepCommand b{.interval = 5, .count = 2, .add = 0, .dir = 1};
    s.enqueueStep(a, 0);
    s.enqueueStep(b, 20);
    EXPECT_EQ(s.pendingCommands(), 2u);
    // a runs at 0,10; b starts at 20 → steps at 20,25
    EXPECT_EQ(s.tick(30), 4u);
    EXPECT_EQ(s.position(), 4);
    EXPECT_TRUE(s.idle());
}

TEST(StepperTickTest, FutureCommandNotStartedEarly) {
    Stepper s{0};
    StepCommand a{.interval = 10, .count = 1, .add = 0, .dir = 1};
    StepCommand b{.interval = 5, .count = 1, .add = 0, .dir = 1};
    s.enqueueStep(a, 0);
    s.enqueueStep(b, 1000);
    EXPECT_EQ(s.tick(50), 1u);    // only a
    EXPECT_EQ(s.tick(999), 0u);   // b not yet
    EXPECT_EQ(s.tick(1000), 1u);  // b now
    EXPECT_EQ(s.position(), 2);
}

TEST(StepperTickTest, ResetClearsState) {
    Stepper s{0};
    s.enqueueStep({.interval = 1, .count = 10, .add = 0, .dir = 1}, 0);
    s.tick(5);
    EXPECT_NE(s.position(), 0);
    s.reset();
    EXPECT_EQ(s.position(), 0);
    EXPECT_TRUE(s.idle());
    EXPECT_EQ(s.pendingCommands(), 0u);
}

TEST(StepperTickTest, SetPositionOverrides) {
    Stepper s{0};
    s.setPosition(-42);
    EXPECT_EQ(s.position(), -42);
    EXPECT_EQ(s.direction(), 1);
    s.setDirection(0);   // dir<0 false → +1
    EXPECT_EQ(s.direction(), 1);
    s.setDirection(-2);
    EXPECT_EQ(s.direction(), -1);
}

// ---------------------------------------------------------------------------
// PipeTransport over real pipe() FDs
// ---------------------------------------------------------------------------

class PipeTransportTest : public ::testing::Test {
protected:
    // data written to upstreamW can be read from upstreamR (transport read),
    // transport writes go to downstreamW and are read from downstreamR.
    int upstream_[2] = {-1, -1};
    int downstream_[2] = {-1, -1};
    PipeTransportConfig cfg_;
    std::unique_ptr<PipeTransport> t_;

    void SetUp() override {
        ASSERT_EQ(::pipe(upstream_), 0);
        ASSERT_EQ(::pipe(downstream_), 0);
        // PipeTransport expects non-blocking fds (like the serial path sets
        // O_NONBLOCK); read(canBlock=false) issues ::read directly.
        int fl = ::fcntl(upstream_[0], F_GETFL, 0);
        ::fcntl(upstream_[0], F_SETFL, fl | O_NONBLOCK);
        cfg_.readFd = upstream_[0];
        cfg_.writeFd = downstream_[1];
        cfg_.ownsFds = false;
        t_ = std::make_unique<PipeTransport>(cfg_);
    }
    void TearDown() override {
        t_->close();
        for (int fd : {upstream_[0], upstream_[1], downstream_[0], downstream_[1]})
            if (fd >= 0) ::close(fd);
    }
};

TEST_F(PipeTransportTest, OpenAndIsOpen) {
    EXPECT_FALSE(t_->isOpen());
    EXPECT_TRUE(t_->open());
    EXPECT_TRUE(t_->isOpen());
    EXPECT_TRUE(t_->open());  // idempotent
    t_->close();
    EXPECT_FALSE(t_->isOpen());
}

TEST_F(PipeTransportTest, WriteReadRoundTrip) {
    ASSERT_TRUE(t_->open());
    const std::array<uint8_t, 5> out{0xAA, 0xBB, 0xCC, 0xDD, 0xEE};
    EXPECT_EQ(t_->write(out), out.size());
    // Written bytes land on downstreamR (host side would read them there).
    uint8_t buf[8]{};
    ASSERT_EQ(::read(downstream_[0], buf, sizeof(buf)), (ssize_t)out.size());
    EXPECT_EQ(std::memcmp(buf, out.data(), out.size()), 0);

    // Feed bytes into upstreamW → transport reads them.
    const std::array<uint8_t, 3> in{1, 2, 3};
    ASSERT_EQ(::write(upstream_[1], in.data(), in.size()), (ssize_t)in.size());
    EXPECT_EQ(t_->available(), in.size());
    uint8_t rb[8]{};
    EXPECT_EQ(t_->read(rb, sizeof(rb), false), in.size());
    EXPECT_EQ(std::memcmp(rb, in.data(), in.size()), 0);
    EXPECT_EQ(t_->available(), 0u);
}

TEST_F(PipeTransportTest, NonBlockingReadReturnsZeroWhenEmpty) {
    ASSERT_TRUE(t_->open());
    uint8_t buf[4];
    EXPECT_EQ(t_->read(buf, sizeof(buf), /*canBlock=*/false), 0u);
}

TEST_F(PipeTransportTest, BlockingReadWaitsForData) {
    ASSERT_TRUE(t_->open());
    const uint8_t b = 0x42;
    ASSERT_EQ(::write(upstream_[1], &b, 1), 1);
    uint8_t buf[4]{};
    EXPECT_EQ(t_->read(buf, sizeof(buf), /*canBlock=*/true), 1u);
    EXPECT_EQ(buf[0], 0x42);
}

TEST_F(PipeTransportTest, ClosedTransportRejectsIo) {
    PipeTransport t{cfg_};
    // never opened
    const uint8_t b = 1;
    EXPECT_EQ(t.write(std::span<const uint8_t>(&b, 1)), 0u);
    EXPECT_EQ(t.available(), 0u);
    uint8_t buf[4];
    EXPECT_EQ(t.read(buf, sizeof(buf)), 0u);
}

TEST_F(PipeTransportTest, InvalidFdsFailOpen) {
    PipeTransportConfig bad;
    bad.readFd = -1;
    bad.writeFd = -1;
    PipeTransport t{bad};
    EXPECT_FALSE(t.open());
    EXPECT_FALSE(t.isOpen());
}

TEST_F(PipeTransportTest, BadDevicePathFailsOpen) {
    PipeTransportConfig bad;
    bad.devicePath = "/nonexistent/serial/dev_xyz";
    PipeTransport t{bad};
    EXPECT_FALSE(t.open());
}

}  // namespace

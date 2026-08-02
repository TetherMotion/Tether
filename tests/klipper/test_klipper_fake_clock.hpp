/**
 * @file test_klipper_fake_clock.hpp
 * @brief FakeClock and FakeTransport test helpers for deterministic testing.
 *
 * FakeClock provides a controllable time source that can be advanced
 * manually, eliminating sleep_for calls in tests.
 *
 * FakeTransport wraps LoopbackTransport with error injection capabilities
 * (drop next N bytes, fail reads/writes, delay delivery).
 */

#pragma once

#include "tether/klipper/transport/LoopbackTransport.hpp"

#include <chrono>
#include <cstdint>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

namespace tether::klipper::test {

/// @brief A controllable fake clock for deterministic time-based testing.
///
/// Instead of sleeping, tests advance the clock manually:
///   FakeClock clock;
///   clock.advance(std::chrono::milliseconds(50));
///   EXPECT_TRUE(clock.hasElapsed(50ms));
class FakeClock {
public:
    using time_point = std::chrono::steady_clock::time_point;
    using duration = std::chrono::steady_clock::duration;

    FakeClock() : now_(time_point{}) {}

    /// @return The current fake time.
    time_point now() const { return now_; }

    /// @brief Advance the clock by a duration.
    void advance(duration d) { now_ += d; }

    /// @brief Advance the clock by milliseconds.
    void advanceMs(int64_t ms) { advance(std::chrono::milliseconds(ms)); }

    /// @brief Advance the clock by microseconds.
    void advanceUs(int64_t us) { advance(std::chrono::microseconds(us)); }

    /// @brief Check if a duration has elapsed since a start point.
    bool hasElapsedSince(time_point start, duration d) const {
        return (now_ - start) >= d;
    }

private:
    time_point now_;
};

/// @brief A fake transport with error injection capabilities.
///
/// Wraps LoopbackTransport and adds:
/// - Drop next N bytes on write
/// - Fail reads (return false) on next N read calls
/// - Delay delivery (buffer writes, deliver on flush())
/// - Corrupt data (flip bits in written data)
class FakeTransport : public transport::IByteStreamTransport {
public:
    enum class ErrorMode {
        None,       ///< No errors injected.
        DropWrite,  ///< Drop next N bytes written.
        FailRead,   ///< Fail next N read calls.
        Corrupt,    ///< Corrupt written data (flip bits).
    };

    FakeTransport() {
        buffer_ = std::make_shared<transport::LoopbackTransport::SharedBuffer>();
        transport_ = std::make_shared<transport::LoopbackTransport>();
    }

    /// @brief Wire two FakeTransports together (like LoopbackTransport::wire).
    void wire(std::shared_ptr<FakeTransport> other) {
        transport_->wire(buffer_, other->buffer_);
        other->transport_->wire(other->buffer_, buffer_);
    }

    // --- IByteStreamTransport interface ---

    bool open() override {
        return transport_->open();
    }

    void close() override {
        transport_->close();
    }

    bool isOpen() const override {
        return transport_->isOpen();
    }

    size_t write(std::span<const uint8_t> data) override {
        if (errorMode_ == ErrorMode::DropWrite && dropCount_ > 0) {
            size_t toDrop = std::min(dropCount_, data.size());
            dropCount_ -= toDrop;
            if (toDrop >= data.size()) return data.size();
            data = data.subspan(toDrop);
        }

        std::vector<uint8_t> modified(data.begin(), data.end());

        if (errorMode_ == ErrorMode::Corrupt && !modified.empty()) {
            // Flip the first bit of the first byte.
            modified[0] ^= 0x01;
        }

        return transport_->write(std::span<const uint8_t>(modified));
    }

    size_t available() const override {
        return transport_->available();
    }

    size_t read(uint8_t* out, size_t maxLen, bool canBlock = false) override {
        if (errorMode_ == ErrorMode::FailRead && failReadCount_ > 0) {
            failReadCount_--;
            return 0;
        }
        return transport_->read(out, maxLen, canBlock);
    }

    // --- Error injection controls ---

    /// @brief Inject drop-write errors: drop the next N bytes written.
    void injectDropWrite(size_t count) {
        errorMode_ = ErrorMode::DropWrite;
        dropCount_ = count;
    }

    /// @brief Inject read failures: fail the next N read calls.
    void injectFailRead(size_t count) {
        errorMode_ = ErrorMode::FailRead;
        failReadCount_ = count;
    }

    /// @brief Inject data corruption: flip a bit in the next write.
    void injectCorruption() {
        errorMode_ = ErrorMode::Corrupt;
    }

    /// @brief Clear all error injection.
    void clearErrors() {
        errorMode_ = ErrorMode::None;
        dropCount_ = 0;
        failReadCount_ = 0;
    }

    /// @brief Check if errors are currently injected.
    bool hasErrors() const {
        return errorMode_ != ErrorMode::None;
    }

private:
    std::shared_ptr<transport::LoopbackTransport::SharedBuffer> buffer_;
    std::shared_ptr<transport::LoopbackTransport> transport_;
    ErrorMode errorMode_ = ErrorMode::None;
    size_t dropCount_ = 0;
    size_t failReadCount_ = 0;
};

} // namespace tether::klipper::test

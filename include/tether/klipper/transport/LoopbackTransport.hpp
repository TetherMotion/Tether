/**
 * @file LoopbackTransport.hpp
 * @brief In-process bidirectional byte-stream transport pair for tests/examples.
 *
 * @details
 * A LoopbackTransportPair creates two connected endpoints (hostEnd and
 * deviceEnd). Bytes written to one endpoint are buffered and become available
 * for reading on the other endpoint. This is purely in-memory and synchronous:
 * no threads or I/O are involved, making it ideal for unit tests and examples.
 *
 * Usage:
 *   LoopbackTransportPair pair;
 *   auto& host = pair.hostEnd();
 *   auto& dev  = pair.deviceEnd();
 *   host.write(...);
 *   dev.read(...);
 */

#pragma once

#include "tether/klipper/transport/IByteStreamTransport.hpp"

#include <mutex>
#include <vector>
#include <deque>
#include <span>
#include <memory>
#include <cstddef>
#include <cstdint>

namespace tether::klipper::transport {

/**
 * @brief One end of an in-process loopback transport pair.
 */
class LoopbackTransport : public IByteStreamTransport {
public:
    /// @brief Internal shared buffer type.
    struct SharedBuffer {
        std::mutex mtx;
        std::deque<uint8_t> buf;
    };

    /// @brief Construct an unconnected endpoint.
    LoopbackTransport() = default;

    bool open() override { open_ = true; return true; }
    bool isOpen() const override { return open_; }
    void close() override { open_ = false; }

    size_t write(std::span<const uint8_t> data) override {
        if (!open_ || !writeTarget_) return 0;
        std::lock_guard<std::mutex> lk(writeTarget_->mtx);
        writeTarget_->buf.insert(writeTarget_->buf.end(), data.begin(), data.end());
        return data.size();
    }

    size_t available() const override {
        if (!open_ || !readSource_) return 0;
        std::lock_guard<std::mutex> lk(readSource_->mtx);
        return readSource_->buf.size();
    }

    size_t read(uint8_t* out, size_t maxLen, bool /*canBlock*/ = false) override {
        if (!open_ || !readSource_) return 0;
        std::lock_guard<std::mutex> lk(readSource_->mtx);
        size_t n = std::min(maxLen, readSource_->buf.size());
        for (size_t i = 0; i < n; ++i) out[i] = readSource_->buf[i];
        readSource_->buf.erase(readSource_->buf.begin(), readSource_->buf.begin() + n);
        return n;
    }

    /// @brief Wire this endpoint: writes go to @p writeTarget, reads come from @p readSource.
    void wire(std::shared_ptr<SharedBuffer> writeTarget,
              std::shared_ptr<SharedBuffer> readSource) {
        writeTarget_ = std::move(writeTarget);
        readSource_ = std::move(readSource);
    }

private:
    std::shared_ptr<SharedBuffer> writeTarget_; // peer's read buffer (we write here)
    std::shared_ptr<SharedBuffer> readSource_;  // our read buffer (peer writes here)
    bool open_ = false;
};

/**
 * @brief A connected pair of LoopbackTransport endpoints.
 *
 * Bytes written to hostEnd() are readable on deviceEnd() and vice versa.
 */
class LoopbackTransportPair {
public:
    LoopbackTransportPair() {
        // host->dev buffer: host writes here, dev reads from here.
        hostToDev_ = std::make_shared<LoopbackTransport::SharedBuffer>();
        // dev->host buffer: dev writes here, host reads from here.
        devToHost_ = std::make_shared<LoopbackTransport::SharedBuffer>();
        // host writes to hostToDev, reads from devToHost.
        host_.wire(hostToDev_, devToHost_);
        // dev writes to devToHost, reads from hostToDev.
        dev_.wire(devToHost_, hostToDev_);
        host_.open();
        dev_.open();
    }

    LoopbackTransport& hostEnd() { return host_; }
    LoopbackTransport& deviceEnd() { return dev_; }

private:
    std::shared_ptr<LoopbackTransport::SharedBuffer> hostToDev_;
    std::shared_ptr<LoopbackTransport::SharedBuffer> devToHost_;
    LoopbackTransport host_;
    LoopbackTransport dev_;
};

} // namespace tether::klipper::transport

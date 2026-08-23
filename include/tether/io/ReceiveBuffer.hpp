/**
 * @file ReceiveBuffer.hpp
 * @brief Bounded receive-buffer abstractions for framed IO transports.
 *
 * A session uses one buffer for the encoded SLIP accumulator and one for the
 * decoded protocol message.  The interface keeps the session independent of
 * whether storage is static or heap-backed while retaining a hard upper bound
 * for every allocation.
 *
 * @copyright Copyright (C) 2025-2026 Tether Authors
 */
#pragma once

#include "tether/io/Protocol.hpp"

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <memory>
#include <limits>
#include <vector>

namespace tether::io {

/**
 * @class IReceiveBuffer
 * @brief Bounded byte storage used by a Session receive path.
 *
 * `resize()` changes the logical byte count and may grow dynamic storage.  It
 * never permits the logical size or capacity to exceed `maxCapacity()`.
 */
class IReceiveBuffer {
public:
    virtual ~IReceiveBuffer() = default;

    virtual void clear() noexcept = 0;
    virtual bool append(const uint8_t* data, size_t len) = 0;
    virtual bool resize(size_t size) = 0;
    virtual uint8_t* data() noexcept = 0;
    virtual const uint8_t* data() const noexcept = 0;
    virtual size_t size() const noexcept = 0;
    virtual size_t capacity() const noexcept = 0;
    virtual size_t maxCapacity() const noexcept = 0;
};

/**
 * @class StaticReceiveBuffer
 * @brief Fixed-capacity receive buffer with no dynamic allocation.
 */
template <size_t Capacity>
class StaticReceiveBuffer final : public IReceiveBuffer {
    static_assert(Capacity > 0, "A receive buffer must have non-zero capacity");

public:
    void clear() noexcept override { size_ = 0; }

    bool append(const uint8_t* source, size_t len) override {
        if (len > Capacity - size_) return false;
        if (len != 0) std::copy_n(source, len, storage_ + size_);
        size_ += len;
        return true;
    }

    bool resize(size_t requested) override {
        if (requested > Capacity) return false;
        size_ = requested;
        return true;
    }

    uint8_t* data() noexcept override { return storage_; }
    const uint8_t* data() const noexcept override { return storage_; }
    size_t size() const noexcept override { return size_; }
    size_t capacity() const noexcept override { return Capacity; }
    size_t maxCapacity() const noexcept override { return Capacity; }

private:
    uint8_t storage_[Capacity]{};
    size_t size_ = 0;
};

/**
 * @class DynamicReceiveBuffer
 * @brief Heap-backed buffer that grows geometrically up to a hard maximum.
 */
class DynamicReceiveBuffer final : public IReceiveBuffer {
public:
    explicit DynamicReceiveBuffer(size_t initialCapacity,
                                  size_t maximumCapacity)
        : maximumCapacity_(maximumCapacity) {
        if (maximumCapacity_ != 0) {
            const size_t initial = std::min(initialCapacity, maximumCapacity_);
            storage_.resize(initial);
        }
    }

    void clear() noexcept override { size_ = 0; }

    bool append(const uint8_t* source, size_t len) override {
        if (size_ > maximumCapacity_ || len > maximumCapacity_ - size_) {
            return false;
        }
        if (len == 0) return true;
        const size_t oldSize = size_;
        if (!resize(size_ + len)) return false;
        std::copy_n(source, len, storage_.data() + oldSize);
        return true;
    }

    bool resize(size_t requested) override {
        if (requested > maximumCapacity_) return false;
        if (requested > storage_.size()) {
            size_t nextCapacity = storage_.size() == 0 ? 1 : storage_.size();
            while (nextCapacity < requested) {
                if (nextCapacity > maximumCapacity_ / 2) {
                    nextCapacity = maximumCapacity_;
                    break;
                }
                nextCapacity *= 2;
            }
            if (nextCapacity < requested) return false;
            try {
                storage_.resize(nextCapacity);
            } catch (...) {
                return false;
            }
        }
        size_ = requested;
        return true;
    }

    uint8_t* data() noexcept override { return storage_.data(); }
    const uint8_t* data() const noexcept override { return storage_.data(); }
    size_t size() const noexcept override { return size_; }
    size_t capacity() const noexcept override { return storage_.size(); }
    size_t maxCapacity() const noexcept override { return maximumCapacity_; }

private:
    std::vector<uint8_t> storage_;
    size_t maximumCapacity_ = 0;
    size_t size_ = 0;
};

/// Maximum encoded size of a SLIP frame carrying MAX_MESSAGE_SIZE bytes.
inline constexpr size_t MAX_ENCODED_MESSAGE_SIZE =
    MAX_MESSAGE_SIZE > (std::numeric_limits<size_t>::max() - 1) / 2
        ? std::numeric_limits<size_t>::max()
        : MAX_MESSAGE_SIZE * 2 + 1;

/// Default initial encoded receive capacity for a Session.
inline constexpr size_t DEFAULT_RECEIVE_BUFFER_CAPACITY = 8192;

/// Factory for an independent receive buffer per Session.
using ReceiveBufferFactory = std::function<std::unique_ptr<IReceiveBuffer>()>;

} // namespace tether::io

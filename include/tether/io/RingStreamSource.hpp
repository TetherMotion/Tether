#pragma once

#include <atomic>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <span>
#include <type_traits>
#include <utility>
#include <vector>

#include "tether/utils/SPSCRing.hpp"

namespace tether {
namespace io {

/**
 * A ring-buffered, producer-driven stream source for the IO protocol.
 *
 * A ring stream source owns a fixed-schema, single-producer/single-consumer
 * ring of raw row images.  Each row is a packed byte image:
 *
 *     [ u64 producer timestamp (us) ][ field0 ][ field1 ] ...
 *
 * with the field byte sizes given by schemaFieldSizes() in schema order.
 * This is the same on-the-wire row layout Session emits for polled streams,
 * so a session can project any subset of the schema fields into a client's
 * configured stream.
 *
 * Purpose: the normal streaming path is pull-based — the session thread
 * calls each entry's read function once per interval and therefore samples
 * at consumer cadence.  A ring source decouples production from the session:
 * a producer (e.g. a cyclic/PDO task) pushes rows whenever data is ready and
 * the session drains them in bulk.  A capacity-1 ring degenerates to
 * "latest sample, best effort"; larger rings buffer bursts.
 *
 * Threading:
 * - start()/stop()/drainRows() are called from the session thread.
 * - The producer calls produce() on SpscRingStreamSource from its own
 *   (possibly realtime) thread.  produce() is wait-free and never blocks.
 * - A source can be bound to at most one session at a time (tryAcquire()).
 */
class IRingStreamSource {
public:
    virtual ~IRingStreamSource() = default;

    /// Entry IDs covered by this source's rows, in row-field order.
    virtual std::span<const uint64_t> schemaEntryIds() const = 0;
    /// Byte size of each field, parallel to schemaEntryIds().  Fixed-size only.
    virtual std::span<const uint16_t> schemaFieldSizes() const = 0;
    /// Total bytes per row image, including the leading 8-byte timestamp.
    virtual size_t rowSize() const = 0;

    /**
     * Copy up to maxRows oldest pending row images into dst.
     * dst must have room for at least maxRows * rowSize() bytes.
     * @return number of rows written.
     */
    virtual size_t drainRows(uint8_t* dst, size_t maxRows) = 0;

    /// Enable production.  Clears pending rows and the drop counter.
    virtual void start() = 0;
    /// Disable production.
    virtual void stop() = 0;
    virtual bool active() const = 0;

    /// Producer-side count of rows dropped because the ring was full.
    virtual uint64_t dropped() const = 0;

    /**
     * Exclusive binding: returns false if another session already holds
     * this source.  Pair with release() when the stream ends.
     */
    bool tryAcquire() {
        bool expected = false;
        return inUse_.compare_exchange_strong(expected, true,
                                              std::memory_order_acquire);
    }
    void release() { inUse_.store(false, std::memory_order_release); }
    bool acquired() const { return inUse_.load(std::memory_order_acquire); }

private:
    std::atomic<bool> inUse_{false};
};

/**
 * SPSCRing-backed ring stream source.
 *
 * Row must be a trivially-copyable packed struct whose first 8 bytes are
 * the producer timestamp; the remaining fields must be laid out packed in
 * the same order as the schema field list passed to the constructor.
 *
 * The single producer calls produce(fill) from its own thread; when the
 * source is inactive (no bound session streaming) produce() short-circuits
 * on one atomic load.  When the ring is full the row is dropped and counted
 * — the producer is never stalled.
 */
template <typename Row, size_t Capacity>
class SpscRingStreamSource final : public IRingStreamSource {
    static_assert(std::is_trivially_copyable_v<Row>,
                  "SpscRingStreamSource requires trivially copyable Row");
    static_assert(sizeof(Row) >= 8,
                  "Row must begin with an 8-byte producer timestamp");

public:
    SpscRingStreamSource() = default;
    SpscRingStreamSource(std::vector<uint64_t> entryIds,
                         std::vector<uint16_t> fieldSizes)
        : entryIds_(std::move(entryIds))
        , fieldSizes_(std::move(fieldSizes)) {}

    /// (Re)install the schema.  Not thread-safe: call during setup, before
    /// any session can bind this source.
    void setSchema(std::vector<uint64_t> entryIds,
                   std::vector<uint16_t> fieldSizes) {
        entryIds_   = std::move(entryIds);
        fieldSizes_ = std::move(fieldSizes);
    }

    std::span<const uint64_t> schemaEntryIds() const override { return entryIds_; }
    std::span<const uint16_t> schemaFieldSizes() const override { return fieldSizes_; }
    size_t rowSize() const override { return sizeof(Row); }

    size_t drainRows(uint8_t* dst, size_t maxRows) override {
        size_t n = 0;
        ring_.drain([&](const Row& row) {
            std::memcpy(dst + n * sizeof(Row), &row, sizeof(Row));
            ++n;
        }, maxRows);
        return n;
    }

    void start() override {
        ring_.reset();
        active_.store(true, std::memory_order_release);
    }
    void stop() override { active_.store(false, std::memory_order_release); }
    bool active() const override { return active_.load(std::memory_order_acquire); }
    uint64_t dropped() const override { return ring_.dropped(); }

    /// Producer side (single producer thread): fill a row in place.
    /// Returns false if the source is inactive or the ring is full.
    template <typename F>
    bool produce(F&& fill) {
        if (!active_.load(std::memory_order_acquire)) return false;
        return ring_.produce(std::forward<F>(fill));
    }

    /// Underlying ring (e.g. for reset during shutdown).
    Tether::Utils::SPSCRing<Row, Capacity>& ring() { return ring_; }

private:
    std::vector<uint64_t> entryIds_;
    std::vector<uint16_t> fieldSizes_;
    Tether::Utils::SPSCRing<Row, Capacity> ring_;
    std::atomic<bool> active_{false};
};

} // namespace io
} // namespace tether

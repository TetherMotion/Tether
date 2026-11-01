/**
 * @file LatestSignal.hpp
 * @brief Latest-value + streamed-ring signal sink for Tether IO producers.
 *
 * A telemetry signal on a Tether IO server is two reader-facing views of
 * the same produced values:
 *   - a streamed history: an SPSC row ring drained in bulk by a bound
 *     session (SpscRingStreamSource), and
 *   - a latest-value snapshot: the most recent row, served to polled
 *     GetSignal reads through the SignalEntry::varReadFn.
 *
 * LatestSignal owns both halves plus the produce() entry point. One
 * produce() call fills a ring row (best effort — dropped+counted when a
 * bound consumer lags, short-circuited when no session streams) and
 * updates the latest snapshot. The producer is never stalled by readers.
 *
 * Threading: produce() is called from the single producer thread (e.g.
 * the control loop). readLatest()/varReadFn() may run on session threads.
 * setSchema()/latestSize() are setup-time calls, not thread-safe.
 *
 * Lifetime: the VarReadFn returned by varReadFn() captures `this` —
 * the LatestSignal must outlive the SignalEntry it is wired into.
 */
#pragma once

#include <algorithm>
#include <array>
#include <cstdint>
#include <cstring>
#include <mutex>
#include <span>
#include <vector>

#include "tether/io/Registry.hpp"
#include "tether/io/RingStreamSource.hpp"

namespace tether::io {

/**
 * Streamed+latest signal for fixed-capacity row payloads.
 * @tparam MaxPayload largest row payload in bytes (excluding the
 *         8-byte producer timestamp the ring prepends).
 * @tparam Capacity ring depth in rows.
 */
template <size_t MaxPayload, size_t Capacity = 256>
class LatestSignal {
public:
    /**Ring row image: producer timestamp + opaque payload. */
    struct Row {
        uint64_t tsUs = 0;
        std::array<uint8_t, MaxPayload> payload{};
    };
    using Ring = SpscRingStreamSource<Row, Capacity>;

    /**
     * @param initialLatestSize pre-sizes the latest blob (declared
     *        maxValueSize for a fixed-size frame; readers see this many
     *        bytes — zero-filled — until the first produce()).
     */
    explicit LatestSignal(size_t initialLatestSize = 0)
        : latest_(initialLatestSize) {}

    /**
     * Producer side: emit one row. Updates the latest snapshot even when
     * the ring is inactive or full.
     * @param tsUs producer timestamp in µs (same clock as the server's
     *        TimestampFn).
     * @param data row payload bytes.
     * @param len payload size, must be <= MaxPayload.
     * @return false when len exceeds MaxPayload (nothing is emitted).
     */
    bool produce(uint64_t tsUs, const uint8_t* data, size_t len) {
        if (len > MaxPayload) return false;
        ring_.produce([&](Row& row) {
            row.tsUs = tsUs;
            std::memcpy(row.payload.data(), data, len);
        });
        {
            std::lock_guard lk(mtx_);
            latest_.assign(data, data + len);
        }
        return true;
    }
    /**@copydoc produce(uint64_t,const uint8_t*,size_t) */
    bool produce(uint64_t tsUs, std::span<const uint8_t> data) {
        return produce(tsUs, data.data(), data.size());
    }

    /**
     * Polled-read side: copy the latest snapshot into `dest`.
     * @return bytes written (<= maxLen).
     */
    size_t readLatest(uint8_t* dest, size_t maxLen) const {
        std::lock_guard lk(mtx_);
        const size_t n = std::min(maxLen, latest_.size());
        std::memcpy(dest, latest_.data(), n);
        return n;
    }
    /**Current latest snapshot size in bytes. */
    size_t latestSize() const {
        std::lock_guard lk(mtx_);
        return latest_.size();
    }
    /**
     * A VarReadFn bound to this signal — assign to SignalEntry::varReadFn.
     * The function object captures `this` (see file-level lifetime rule).
     */
    VarReadFn varReadFn() {
        return [this](void* dest, size_t maxLen) {
            return readLatest(static_cast<uint8_t*>(dest), maxLen);
        };
    }

    /**The ring source handed to ServerConfig::ringSources. */
    Ring& source() { return ring_; }
    /**Producer-side count of rows dropped because the ring was full. */
    uint64_t dropped() const { return ring_.dropped(); }
    /**(Re)install the row-field schema. Setup-time only. */
    void setSchema(std::vector<uint64_t> entryIds,
                   std::vector<uint16_t> fieldSizes) {
        ring_.setSchema(std::move(entryIds), std::move(fieldSizes));
    }

private:
    Ring ring_;
    mutable std::mutex mtx_;
    std::vector<uint8_t> latest_;
};

}  // namespace tether::io

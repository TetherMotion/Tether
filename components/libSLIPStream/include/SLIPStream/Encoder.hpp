/**
 * @file SLIPStreamEncoder.hpp
 * @author Uli Köhler <github@techoverflow.net>
 * @version 1.0
 * @date 2025-08-19
 * 
 * @copyright Copyright (C) 2022..2025 Uli Köhler
 */
#pragma once
#include <cstdint>
#include <cstddef>
#include <functional>
#include <vector>
#include "SLIPStream/SLIP.hpp"

namespace SLIPStream {

/**
 * Non-blocking write status for the encoder's single-byte output callback
 */
enum class WriteStatus: uint8_t {
    Ok = 0,       // Byte accepted and written
    RetryLater = 1, // Output would block; try again later without data loss
    Error = 2     // Non-recoverable error from output
};

using OutputFn = std::function<WriteStatus(uint8_t)>;

/**
 * A stateful, non-blocking SLIP encoder with internal buffering.
 */
class Encoder {
public:
    Encoder(OutputFn outputFn, size_t txBufferSize, size_t maxSendChunk = 64);

    // Attempt to flush up to maxSendChunk queued encoded bytes via outputFn.
    WriteStatus flush();

    // Encode and queue a complete SLIP packet (payload escaped, END appended).
    // Returns pair of (status, consumedBytes).
    std::pair<WriteStatus, size_t> pushPacket(const uint8_t* data, size_t size);

    void setMaxSendChunk(size_t n) { maxSendChunk = n; }

    size_t queued() const { return txSize; }
    size_t capacity() const { return txBuf.size(); }
    size_t free() const { return txBuf.size() - txSize; }

private:
    // Internal helpers for queue management
    bool queueByte(uint8_t b);
    bool dequeueByte(uint8_t& b);

    // Try to ensure at least 'n' bytes free in queue by flushing as needed
    WriteStatus ensureFree(size_t n);

    // Encode one payload byte into queue (may be 1 or 2 bytes)
    WriteStatus encodeOne(const uint8_t* data, size_t size, size_t& consumed);

    OutputFn outputFn;
    std::vector<uint8_t> txBuf;
    size_t txHead; // pop index
    size_t txTail; // push index
    size_t txSize; // number of bytes currently queued
    size_t maxSendChunk;

    // Packet state: whether we still need to append a trailing END for the current packet
    bool endPending;
};

} // namespace SLIPStream
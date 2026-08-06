/**
 * @file BlockReader.hpp
 * @brief Streaming message-block reader: reads blocks from a transport on demand.
 *
 * @details
 * BlockReader wraps an IByteStreamTransport and provides an incremental
 * readNext() API for parsing message blocks one at a time. It maintains an
 * internal accumulation buffer so that blocks spanning multiple transport
 * reads are handled transparently.
 *
 * This mirrors the streaming readNext() API of the pcapng reader
 * (PCAPNGReader::readNext / reset), adapted to the Klipper message-block
 * framing. Like the pcapng reader, BlockReader also supports error-recovery
 * mode (via the embedded BlockParser) and a reset() method that clears
 * internal state without re-opening the transport.
 *
 * Usage:
 * @code
 *   BlockReader reader(transport);
 *   reader.setRecoveryMode(true, [](size_t off, auto status, auto msg) {
 *       LOG_WARN("skipped block at " << off << ": " << msg);
 *   });
 *   MessageBlock block;
 *   while (reader.readNext(block)) {
 *       // process block
 *   }
 *   std::cout << reader.stats().blocksParsed << " blocks";
 * @endcode
 *
 * @see BlockParser.hpp for the stateful parser with recovery mode.
 * @see MessageBlock.hpp for the block frame format.
 */

#pragma once

#include "tether/klipper/protocol/BlockParser.hpp"
#include "tether/klipper/protocol/MessageBlock.hpp"
#include "tether/klipper/transport/IByteStreamTransport.hpp"

#include <cstdint>
#include <cstddef>
#include <vector>
#include <span>

namespace tether::klipper::protocol {

/**
 * @brief Streaming message-block reader with internal accumulation buffer.
 *
 * Reads bytes from a transport on demand and parses complete message blocks.
 * Partial data is buffered internally until a complete block arrives. Supports
 * error-recovery mode (skip corrupt blocks) and statistics tracking via the
 * embedded BlockParser.
 */
class BlockReader {
public:
    /// @brief Callback invoked for each rejected/skipped block in recovery mode.
    using ErrorCallback = BlockParser::ErrorCallback;

    /**
     * @brief Construct a reader bound to a transport.
     * @param transport The byte-stream transport to read from.
     * @param readChunkSize Maximum bytes to read per transport call (default 256).
     */
    explicit BlockReader(transport::IByteStreamTransport& transport,
                          size_t readChunkSize = 256)
        : transport_(transport), readChunkSize_(readChunkSize) {}

    /**
     * @brief Read and parse the next complete message block from the transport.
     *
     * Reads bytes from the transport as needed, accumulating them in an
     * internal buffer until a complete block is available. In recovery mode,
     * corrupt blocks are skipped (with the error callback invoked) and the
     * reader continues scanning for the next valid block.
     *
     * @param out Output: the parsed message block (valid when returning true).
     * @return true if a block was produced; false at EOF or on unrecoverable
     *         error (when recovery mode is disabled).
     */
    bool readNext(MessageBlock& out) {
        while (true) {
            auto pb = parser_.parse(std::span<const uint8_t>(buffer_));
            switch (pb.status) {
                case BlockParseStatus::Ok:
                    // Consume the parsed bytes from the buffer.
                    consumeFromBuffer(pb.consumedBytes);
                    out = std::move(pb.block);
                    return true;
                case BlockParseStatus::NeedMoreData: {
                    // Discard any leading garbage that the parser skipped.
                    if (pb.consumedBytes > 0 && pb.consumedBytes < buffer_.size()) {
                        consumeFromBuffer(pb.consumedBytes);
                    } else if (pb.consumedBytes >= buffer_.size()) {
                        buffer_.clear();
                    }
                    // Try to read more data from the transport.
                    if (!fillFromTransport()) {
                        // No more data available; return false (EOF or empty).
                        // If there's leftover data in the buffer, it's a
                        // partial block that will never complete.
                        return false;
                    }
                    continue;
                }
                case BlockParseStatus::BadCrc:
                case BlockParseStatus::BadSync:
                case BlockParseStatus::BadLength:
                    if (parser_.recoveryMode()) {
                        // Recovery mode: the parser already skipped the bad
                        // block and updated stats. Consume the bytes it
                        // reported and continue.
                        consumeFromBuffer(pb.consumedBytes);
                        continue;
                    }
                    // Non-recovery mode: return false on error.
                    consumeFromBuffer(pb.consumedBytes);
                    return false;
            }
            return false;
        }
    }

    /**
     * @brief Enable or disable recovery mode (delegates to the BlockParser).
     *
     * When enabled, corrupt blocks are skipped instead of causing readNext()
     * to return false. The error callback is invoked for each skipped block.
     *
     * @param enabled True to skip corrupt blocks; false to stop on error.
     * @param cb      Optional callback invoked for each skipped block.
     */
    void setRecoveryMode(bool enabled, ErrorCallback cb = nullptr) {
        parser_.setRecoveryMode(enabled, std::move(cb));
    }

    /// @return True if recovery mode is enabled.
    bool recoveryMode() const { return parser_.recoveryMode(); }

    /// @return Number of blocks skipped due to errors in recovery mode.
    size_t skippedBlockCount() const { return parser_.skippedBlockCount(); }

    /// @return Cumulative parse statistics (from the embedded BlockParser).
    const BlockParseStats& stats() const { return parser_.stats(); }

    /**
     * @brief Reset the reader's internal state.
     *
     * Clears the accumulation buffer and resets parse statistics. The
     * transport is not touched. Recovery-mode setting and error callback
     * are preserved. This is useful for re-iterating from the current
     * transport position after a protocol reset.
     */
    void reset() {
        buffer_.clear();
        parser_.resetStats();
    }

    /// @brief Clear the internal accumulation buffer (without resetting stats).
    void clearBuffer() { buffer_.clear(); }

    /// @return Number of bytes currently buffered (not yet parsed).
    size_t bufferedBytes() const { return buffer_.size(); }

private:
    bool fillFromTransport() {
        size_t avail = transport_.available();
        if (avail == 0) {
            // Try a blocking read of at least one byte.
            uint8_t tmp[1];
            size_t n = transport_.read(tmp, 1, false);
            if (n == 0) return false;
            buffer_.push_back(tmp[0]);
            // Also drain whatever else is now available.
            avail = transport_.available();
        }
        size_t toRead = std::min(avail, readChunkSize_);
        if (toRead == 0) return true; // got one byte above
        size_t oldSize = buffer_.size();
        buffer_.resize(oldSize + toRead);
        size_t got = transport_.read(buffer_.data() + oldSize, toRead, false);
        buffer_.resize(oldSize + got);
        return got > 0;
    }

    void consumeFromBuffer(size_t n) {
        if (n == 0) return;
        if (n >= buffer_.size()) {
            buffer_.clear();
        } else {
            buffer_.erase(buffer_.begin(), buffer_.begin() + n);
        }
    }

    transport::IByteStreamTransport& transport_;
    size_t readChunkSize_;
    std::vector<uint8_t> buffer_;
    BlockParser parser_;
};

} // namespace tether::klipper::protocol

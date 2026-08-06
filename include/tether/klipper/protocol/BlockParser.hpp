/**
 * @file BlockParser.hpp
 * @brief Stateful message-block parser with error recovery and statistics.
 *
 * @details
 * This header provides a @ref BlockParser class that wraps the stateless
 * parseBlock() function with:
 *   - **Error-recovery mode**: when enabled, corrupt blocks (bad CRC, bad
 *     sync, bad length) are skipped instead of being returned as errors.
 *     An optional error callback is invoked for each skipped block, and a
 *     skipped-block counter is maintained.
 *   - **Statistics tracking**: @ref BlockParseStats records how many blocks
 *     were parsed successfully, how many had bad CRC/sync/length, and how
 *     many bytes were skipped during resynchronisation.
 *   - **Reset**: clears accumulated state and statistics without
 *     reconstructing the parser.
 *
 * This mirrors the error-recovery and statistics features of the pcapng
 * reader (PCAPNGReader::setRecoveryMode / skippedBlockCount), adapted to
 * the Klipper message-block framing.
 *
 * The stateless free function parseBlock() remains available for callers
 * that do not need recovery mode or statistics; it is implemented in terms
 * of a default-constructed BlockParser.
 *
 * @see MessageBlock.hpp for the block frame format and parseBlock().
 */

#pragma once

#include "tether/klipper/protocol/MessageBlock.hpp"
#include "tether/klipper/protocol/Constants.hpp"

#include <cstdint>
#include <cstddef>
#include <span>
#include <functional>
#include <string>
#include <string_view>

namespace tether::klipper::protocol {

/**
 * @brief Statistics accumulated by a BlockParser over its lifetime.
 *
 * All counters are monotonic and only reset by an explicit call to
 * BlockParser::reset() or BlockParser::resetStats().
 */
struct BlockParseStats {
    size_t blocksParsed = 0;     ///< Blocks successfully parsed (status == Ok)
    size_t badCrcCount = 0;      ///< Blocks rejected due to CRC mismatch
    size_t badSyncCount = 0;     ///< Blocks rejected due to sync-byte mismatch
    size_t badLengthCount = 0;   ///< Blocks rejected due to out-of-range length
    size_t bytesSkipped = 0;     ///< Total garbage bytes skipped during resync
    size_t blocksSkipped = 0;    ///< Total blocks skipped in recovery mode

    /// @return Total number of blocks encountered (parsed + rejected).
    size_t totalBlocks() const {
        return blocksParsed + badCrcCount + badSyncCount + badLengthCount;
    }

    /// @return Total number of error blocks (all non-Ok statuses).
    size_t totalErrors() const {
        return badCrcCount + badSyncCount + badLengthCount;
    }
};

/**
 * @brief Stateful message-block parser with error recovery and statistics.
 *
 * This class wraps the stateless parseBlock() function with:
 *   - Recovery mode (skip corrupt blocks instead of returning errors)
 *   - An error callback invoked for each skipped/rejected block
 *   - Cumulative statistics (@ref BlockParseStats)
 *
 * Usage without recovery mode (behaves like parseBlock() but tracks stats):
 * @code
 *   BlockParser parser;
 *   auto pb = parser.parse(buffer);
 *   if (pb.status == BlockParseStatus::Ok) { ... }
 *   std::cout << parser.stats().blocksParsed << " blocks parsed";
 * @endcode
 *
 * Usage with recovery mode (skip corrupt blocks, invoke callback):
 * @code
 *   BlockParser parser;
 *   parser.setRecoveryMode(true, [](size_t offset, BlockParseStatus s,
 *                                    std::string_view msg) {
 *       LOG_WARN("skipped block at offset " << offset << ": " << msg);
 *   });
 *   auto pb = parser.parse(buffer);
 *   // In recovery mode, pb.status is Ok or NeedMoreData (never BadCrc etc.)
 * @endcode
 */
class BlockParser {
public:
    /// @brief Callback invoked for each rejected/skipped block in recovery mode.
    /// @param offset   Byte offset within the input where the error was found.
    /// @param status   The parse status that would have been returned.
    /// @param msg      Human-readable description of the error.
    using ErrorCallback = std::function<void(size_t offset,
                                              BlockParseStatus status,
                                              std::string_view msg)>;

    BlockParser() = default;

    /**
     * @brief Enable or disable recovery mode.
     *
     * When enabled, corrupt blocks (BadCrc, BadSync, BadLength) are skipped
     * (their consumed bytes are discarded) and the parser continues scanning
     * for the next valid block. The error callback is invoked for each
     * skipped block. When disabled, corrupt blocks are returned as-is
     * (matching the behaviour of the stateless parseBlock()).
     *
     * @param enabled True to skip corrupt blocks; false to return errors.
     * @param cb      Optional callback invoked for each skipped block.
     */
    void setRecoveryMode(bool enabled, ErrorCallback cb = nullptr) {
        recoveryMode_ = enabled;
        errorCallback_ = std::move(cb);
    }

    /// @return True if recovery mode is enabled.
    bool recoveryMode() const { return recoveryMode_; }

    /// @return Number of blocks skipped due to errors in recovery mode.
    size_t skippedBlockCount() const { return stats_.blocksSkipped; }

    /// @return Cumulative parse statistics.
    const BlockParseStats& stats() const { return stats_; }

    /**
     * @brief Parse one block from the front of a byte buffer.
     *
     * Behaves like the stateless parseBlock(), but:
     *   - In recovery mode, BadCrc/BadSync/BadLength results are skipped
     *     (the parser re-invokes itself on the remaining buffer) and the
     *     error callback is invoked. The returned ParsedBlock will have
     *     status Ok or NeedMoreData (never an error status).
     *   - Statistics are updated for each parse attempt.
     *
     * @param buffer Incoming bytes (may contain partial/garbage data).
     * @return ParsedBlock; see parseBlock() for status semantics.
     */
    ParsedBlock parse(std::span<const uint8_t> buffer) {
        size_t offset = 0;
        while (true) {
            auto pb = parseBlock(
                std::span<const uint8_t>(buffer.data() + offset,
                                          buffer.size() - offset));
            // Update statistics based on the raw result.
            updateStats(pb);
            switch (pb.status) {
                case BlockParseStatus::Ok:
                    // Adjust consumedBytes and skippedBytes for the offset.
                    pb.consumedBytes += offset;
                    pb.skippedBytes += offset;
                    return pb;
                case BlockParseStatus::NeedMoreData:
                    // Not enough data; caller should wait for more.
                    pb.consumedBytes += offset;
                    pb.skippedBytes += offset;
                    return pb;
                case BlockParseStatus::BadCrc:
                case BlockParseStatus::BadSync:
                case BlockParseStatus::BadLength:
                    if (recoveryMode_) {
                        // Invoke the error callback if set.
                        if (errorCallback_) {
                            errorCallback_(offset + pb.skippedBytes,
                                           pb.status,
                                           statusToString(pb.status));
                        }
                        ++stats_.blocksSkipped;
                        // Skip the consumed bytes and try again.
                        offset += pb.consumedBytes;
                        if (offset >= buffer.size()) {
                            // Exhausted the buffer after skipping.
                            ParsedBlock exhausted;
                            exhausted.status = BlockParseStatus::NeedMoreData;
                            exhausted.consumedBytes = offset;
                            exhausted.skippedBytes = offset;
                            return exhausted;
                        }
                        continue; // try parsing the next block
                    }
                    // Not in recovery mode: return the error as-is.
                    pb.consumedBytes += offset;
                    pb.skippedBytes += offset;
                    return pb;
            }
            // Unreachable (all enum cases handled), but keep the compiler happy.
            pb.consumedBytes += offset;
            pb.skippedBytes += offset;
            return pb;
        }
    }

    /// @brief Reset statistics and recovery state (keeps recovery-mode setting).
    void resetStats() {
        stats_ = BlockParseStats{};
    }

    /// @brief Reset all parser state: statistics, recovery mode, and callback.
    void reset() {
        stats_ = BlockParseStats{};
        recoveryMode_ = false;
        errorCallback_ = nullptr;
    }

private:
    void updateStats(const ParsedBlock& pb) {
        switch (pb.status) {
            case BlockParseStatus::Ok:
                ++stats_.blocksParsed;
                break;
            case BlockParseStatus::BadCrc:
                ++stats_.badCrcCount;
                break;
            case BlockParseStatus::BadSync:
                ++stats_.badSyncCount;
                break;
            case BlockParseStatus::BadLength:
                ++stats_.badLengthCount;
                break;
            case BlockParseStatus::NeedMoreData:
                break;
        }
        // Track bytes skipped during resync (garbage before a block).
        if (pb.status == BlockParseStatus::Ok ||
            pb.status == BlockParseStatus::BadCrc) {
            stats_.bytesSkipped += pb.skippedBytes;
        }
    }

    static std::string_view statusToString(BlockParseStatus s) {
        switch (s) {
            case BlockParseStatus::Ok:           return "ok";
            case BlockParseStatus::NeedMoreData: return "need more data";
            case BlockParseStatus::BadSync:      return "bad sync byte";
            case BlockParseStatus::BadCrc:       return "CRC mismatch";
            case BlockParseStatus::BadLength:    return "length out of range";
        }
        return "unknown";
    }

    BlockParseStats stats_;
    bool recoveryMode_ = false;
    ErrorCallback errorCallback_;
};

} // namespace tether::klipper::protocol

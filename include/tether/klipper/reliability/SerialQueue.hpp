/**
 * @file SerialQueue.hpp
 * @brief Host-side serial queue: send blocks, track acks, retransmit on RTO.
 *
 * @details
 * The SerialQueue is the reliability layer on the host (klippy) side. It:
 *   - Assigns sequence numbers to outgoing message blocks.
 *   - Sends blocks via the transport and tracks them as "pending".
 *   - Processes incoming ack/nak blocks from the device:
 *       * An ack with seq=S acknowledges all blocks up to and including S.
 *       * A nak with seq=S requests retransmission of block S and all
 *         subsequent pending blocks.
 *   - Retransmits blocks whose RTO expires before being acked.
 *   - Enforces a maximum number of pending (in-flight) blocks (sliding window).
 *
 * The device side does not maintain a serial queue; it simply sends ack/nak
 * blocks in response to received blocks (see KlipperDevice).
 */

#pragma once

#include "tether/klipper/protocol/MessageBlock.hpp"
#include "tether/klipper/protocol/Constants.hpp"
#include "tether/klipper/reliability/SequenceCounter.hpp"
#include "tether/klipper/reliability/RtoEstimator.hpp"
#include "tether/klipper/transport/IByteStreamTransport.hpp"

#include <cstdint>
#include <vector>
#include <deque>
#include <chrono>
#include <functional>
#include <mutex>
#include <optional>

namespace tether::klipper::reliability {

using Clock = std::chrono::steady_clock;
using TimePoint = Clock::time_point;

/// @brief A pending block awaiting acknowledgement.
struct PendingBlock {
    uint8_t sequence = 0;
    std::vector<uint8_t> wireBytes;     ///< Complete block bytes for retransmit
    TimePoint sendTime;                 ///< When the block was last sent
    uint8_t retransmitCount = 0;         ///< Number of retransmits so far
};

/**
 * @brief Host-side serial queue with sliding window, ack/nak, and RTO retransmit.
 */
class SerialQueue {
public:
    /// @brief Callback invoked when a block's ack is received (for RTT measurement).
    using AckCallback = std::function<void(double rttSample)>;

    explicit SerialQueue(transport::IByteStreamTransport& transport,
                         uint8_t maxPending = protocol::kDefaultMaxPendingBlocks)
        : transport_(transport), maxPending_(maxPending) {}

    /// @return The next sequence number that will be assigned.
    uint8_t nextSequence() const { std::lock_guard<std::mutex> lk(mutex_); return sendSeq_.value(); }

    /// @return Number of blocks currently pending acknowledgement.
    size_t pendingCount() const { std::lock_guard<std::mutex> lk(mutex_); return pending_.size(); }

    /// @return True if the window allows sending another block.
    bool canSend() const { std::lock_guard<std::mutex> lk(mutex_); return pending_.size() < maxPending_; }

    /// @brief Set the ack callback (for RTT measurement).
    void setAckCallback(AckCallback cb) { std::lock_guard<std::mutex> lk(mutex_); ackCb_ = std::move(cb); }

    /**
     * @brief Send a content payload as a new message block.
     *
     * Assigns the next sequence number, builds the block, writes it to the
     * transport, and records it as pending.
     *
     * @param content Content bytes for the block.
     * @return The assigned sequence number, or std::nullopt if the window is full.
     */
    std::optional<uint8_t> send(std::span<const uint8_t> content);

    /**
     * @brief Process an incoming ack/nak block from the device.
     *
     * An ack block has empty content; a nak block has content whose first byte
     * is the nak'd sequence (or is also empty, treated as nak of the last
     * received sequence). This implementation treats any received block from
     * the device as an ack of the last in-order received block, encoded in the
     * block's sequence field.
     *
     * @param block The ack/nak block received from the device.
     */
    void processAck(const protocol::MessageBlock& block);

    /**
     * @brief Check for RTO expiry and retransmit timed-out blocks.
     *
     * Should be called periodically by the host's event loop.
     * @param now Current time (defaults to steady_clock::now()).
     */
    void checkTimeouts(TimePoint now = Clock::now());

    /// @brief Reset the queue (e.g. on connection re-establishment).
    void reset();

    /// @return The RTO estimator (for inspection/tuning). Not thread-safe; call from same thread as checkTimeouts.
    const RtoEstimator& rtoEstimator() const { return rto_; }

private:
    transport::IByteStreamTransport& transport_;
    uint8_t maxPending_;
    static constexpr uint8_t kMaxRetransmits = 5; ///< Max retransmits before dropping
    SequenceCounter sendSeq_;
    SequenceCounter ackSeq_; // last acked sequence (starts behind sendSeq)
    std::deque<PendingBlock> pending_;
    RtoEstimator rto_;
    AckCallback ackCb_;
    mutable std::mutex mutex_;
};

} // namespace tether::klipper::reliability

/**
 * @file SerialQueue.cpp
 * @brief SerialQueue implementation.
 */

#include "tether/klipper/reliability/SerialQueue.hpp"
#include "tether/klipper/KlipperLog.hpp"

#include <algorithm>
#include <format>

namespace tether::klipper::reliability {

std::optional<uint8_t> SerialQueue::send(std::span<const uint8_t> content) {
    if (!canSend()) {
        KLIPPER_LOG_WARN("SerialQueue::send() rejected - window full");
        return std::nullopt;
    }
    uint8_t seq = sendSeq_.value();
    auto wire = protocol::buildBlockVec(seq, content);
    if (wire.empty()) {
        KLIPPER_LOG_ERROR("SerialQueue::send() failed to build message block");
        return std::nullopt;
    }
    size_t written = transport_.write(wire);
    if (written != wire.size()) {
        KLIPPER_LOG_ERROR(std::format("SerialQueue::send() partial write: {}/{} bytes", written, wire.size()));
        return std::nullopt;
    }
    PendingBlock pb;
    pb.sequence = seq;
    pb.wireBytes = std::move(wire);
    pb.sendTime = Clock::now();
    pending_.push_back(std::move(pb));
    sendSeq_.advance();
    return seq;
}

void SerialQueue::processAck(const protocol::MessageBlock& block) {
    // The device sends ack blocks with the sequence number of the last
    // in-order received block. Acknowledge all pending blocks up to and
    // including that sequence.
    uint8_t ackedSeq = block.sequence;
    // Measure RTT for the acked block (the oldest pending one).
    if (!pending_.empty() && ackCb_) {
        auto now = Clock::now();
        // Find the pending block with this sequence.
        for (const auto& pb : pending_) {
            if (pb.sequence == ackedSeq) {
                double rtt = std::chrono::duration<double>(now - pb.sendTime).count();
                rto_.update(rtt);
                ackCb_(rtt);
                break;
            }
        }
    }
    // Remove all pending blocks with sequence <= ackedSeq (mod 16).
    while (!pending_.empty()) {
        uint8_t frontSeq = pending_.front().sequence;
        // Mod-16 "less than or equal": frontSeq is acked if it equals ackedSeq
        // or is "before" it in the window. We remove from the front until we
        // pass the acked sequence.
        uint8_t distance = (ackedSeq - frontSeq) & 0x0F;
        if (distance < 0x10 && frontSeq != ackedSeq && distance > 0) {
            // frontSeq is ahead of ackedSeq — shouldn't happen for in-order acks.
            break;
        }
        pending_.pop_front();
        if (frontSeq == ackedSeq) break;
    }
    ackSeq_.set(ackedSeq);
}

void SerialQueue::checkTimeouts(TimePoint now) {
    double rto = rto_.rto();
    auto rtoDur = std::chrono::duration_cast<Clock::duration>(
        std::chrono::duration<double>(rto));
    for (auto& pb : pending_) {
        if (now - pb.sendTime >= rtoDur) {
            // Retransmit.
            transport_.write(pb.wireBytes);
            pb.sendTime = now;
            pb.retransmitCount++;
        }
    }
}

void SerialQueue::reset() {
    pending_.clear();
    sendSeq_.set(0);
    ackSeq_.set(0);
    rto_.reset();
    firstAck_ = true;
}

} // namespace tether::klipper::reliability

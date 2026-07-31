/**
 * @file AckMessage.hpp
 * @brief Device-side ack/nak message block construction.
 *
 * @details
 * The device acknowledges received blocks by sending back an empty message
 * block whose sequence number is the last in-order received sequence. If a
 * block arrives out of order or fails CRC, the device sends a nak (a block
 * with a nak message in its content) or simply re-acks the last good
 * sequence.
 *
 * In the Klipper protocol, the device sends ack blocks with empty content and
 * the sequence set to the last received in-order block. The host interprets
 * this as acknowledging all blocks up to and including that sequence.
 */

#pragma once

#include "tether/klipper/protocol/MessageBlock.hpp"

#include <cstdint>
#include <vector>

namespace tether::klipper::reliability {

/**
 * @brief Build an ack block for the given sequence number.
 * @param sequence The last in-order received sequence to acknowledge.
 * @return Wire bytes for the ack block (empty content).
 */
inline std::vector<uint8_t> buildAckBlock(uint8_t sequence) {
    return protocol::buildAckBlock(sequence);
}

} // namespace tether::klipper::reliability

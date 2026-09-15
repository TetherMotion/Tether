#pragma once

// ============================================================================
// FSoE frame rewriter — active blackchannel bridging
// ============================================================================
//
// The rewriter sits between two FSoE peers whose CRC chaining models differ
// (e.g. an ESC211 master that resets its CRC chain at state transitions and
// a spec-conformant slave).  For each direction it extracts (command, safe
// data, Conn_Id) from the incoming frame and re-serializes them with a fresh
// CRC chain that matches what the RECEIVING peer expects — the payload passes
// through untouched, only the envelope (CRCs / implicit sequence number) is
// rewritten.
//
// Chaining model used (cross-direction inheritance, ETG.5100 §8.1.3.2):
//
//   - A Reset frame resets the chain: it is built with (startCrc=0, seq=1)
//     and the receiver resets its expected sequence accordingly.
//   - Every other frame chains from the CRC0 of the last frame the RECEIVER
//     transmitted, with a sequence number that the receiver increments once
//     per accepted frame.
//   - Slave responses reuse the sequence number of the master frame they
//     answer (the master validates a response against its own last TX).
//
// Because the peers do not transmit their sequence numbers, the slave-side
// rewriter learns the master's actual (startCrc, seq) by CRC-verifying each
// new master frame against a small set of candidate chain states — this makes
// it robust to whichever chaining variant the master firmware implements.
//
// Threading: each rewriter is a plain stateful object; use one instance per
// direction and call it from a single thread.  The peer's latest frame is
// fed via note*Frame() so the rewriter can pick up its stored CRC0.
// ============================================================================

#include <array>
#include <cstdint>
#include <cstddef>
#include <cstring>

#include "tether/fsoe/FSoECRC.hpp"
#include "tether/fsoe/FSoEDefs.hpp"

namespace FSoE {

/// Initial sequence number (ETG.5100: range 1..65535, 0 is never used).
inline constexpr uint16_t kInitialSeqNo = 1;

/// Extract the CRC0 field stored inside a serialized FSoE frame.
///
/// Frame layouts:
///   len == 3 : [CMD][ConnID(2)]            — bare Reset, no CRC → returns 0
///   len 4..6 : [CMD][D0][CRC0(2)][ConnID(2)]        — CRC0 at offset 2
///   len >= 7 : [CMD][D0][D1][CRC0(2)]...            — CRC0 at offset 3
inline uint16_t storedCrc0(const uint8_t* frame, size_t len) {
    if (!frame || len <= CRC::MIN_FSOE_FRAME_SIZE) return 0;
    const size_t off = (len <= 6) ? 2 : 3;
    if (off + 2 > len) return 0;
    return static_cast<uint16_t>(frame[off]) |
           static_cast<uint16_t>(frame[off + 1]) << 8;
}

/// Whether a byte is a defined FSoE command (guards against forwarding
/// garbage from an all-zero / uninitialized PDO buffer).
inline bool isFSoECommand(uint8_t cmd) {
    switch (cmd) {
        case Command::Reset:
        case Command::Session:
        case Command::Connection:
        case Command::Parameter:
        case Command::ProcessData:
        case Command::FailSafeData:
            return true;
        default:
            return false;
    }
}

// ---------------------------------------------------------------------------
// MasterFrameRewriter — rebuilds master→slave frames for the slave
// ---------------------------------------------------------------------------
//
// Fed the master peer's outgoing frames plus the slave's latest TX frame
// (for its CRC0).  Emits a spec-conformant master frame:
//
//   Reset      → (startCrc = 0, seq = 1); resets next_seq_ to 2
//   non-Reset  → (startCrc = slave's last TX CRC0, seq = next_seq_++)
//
// The first non-Reset frame after a Reset is a protocol boundary: some
// slaves expect it strictly chained from their Reset CRC0, others accept a
// chain-reset (0, seq=1).  Since the slave's expectation cannot be known a
// priori, the rewriter starts with the strict model and — while the slave
// stays in Reset — alternates the two models every kStallCycles emitted
// frames until the slave advances (detected via noteSlaveFrame seeing a
// non-Reset command, which locks the model in place).
class MasterFrameRewriter {
public:
    /// Cycles to wait for slave progress before toggling the first-frame
    /// chaining model (~0.3 s at a 1 kHz processing cycle).
    static constexpr uint32_t kStallCycles = 300;

    void reset() {
        slave_crc0_ = 0;
        next_seq_ = kInitialSeqNo;
        first_after_reset_ = true;
        lenient_first_frame_ = false;
        model_locked_ = false;
        stall_cycles_ = 0;
    }

    /// Observe the slave's latest TX frame — captures its stored CRC0 and
    /// locks the chaining model once the slave advances past Reset.
    void noteSlaveFrame(const uint8_t* frame, size_t len) {
        if (!frame || len == 0 || !isFSoECommand(frame[0])) return;
        slave_crc0_ = storedCrc0(frame, len);
        if (frame[0] != Command::Reset) model_locked_ = true;
    }

    /// Rewrite one incoming master frame for delivery to the slave.
    /// Returns the output frame length, or 0 if the input is not a
    /// recognizable FSoE frame (caller should emit nothing / zeros).
    size_t rewrite(const uint8_t* in, size_t in_len,
                   uint8_t* out, size_t out_cap) {
        uint8_t cmd = 0;
        size_t data_len = 0;
        uint16_t conn_id = 0;
        uint8_t data[CRC::MAX_PARSE_DATA_SIZE];
        if (!CRC::extractFSoEFrame(in, in_len, cmd, data, data_len, conn_id) ||
            !isFSoECommand(cmd)) {
            return 0;
        }
        if (CRC::fsoeFrameSize(data_len) > out_cap) return 0;

        if (cmd == Command::Reset) {
            // Reset resets the CRC chain on both sides; the slave expects
            // (startCrc=0, seq=initial) and will expect seq=2 next.
            first_after_reset_ = true;
            stall_cycles_ = 0;
            next_seq_ = CRC::incrementSeqNo(kInitialSeqNo);
            return CRC::buildFSoEFrame(out, cmd, data, data_len, conn_id,
                                       /*start_crc=*/0, kInitialSeqNo);
        }

        const bool chain_reset = first_after_reset_ && lenient_first_frame_;
        const uint16_t start_crc = chain_reset ? 0 : slave_crc0_;
        const uint16_t seq = chain_reset ? kInitialSeqNo : next_seq_;

        uint16_t seq_used = 0;
        const size_t n = CRC::buildFSoEFrameWithCollisionAvoidance(
            out, cmd, data, data_len, conn_id, start_crc, seq,
            nullptr, &seq_used);
        if (n == 0) return 0;
        first_after_reset_ = false;
        next_seq_ = CRC::incrementSeqNo(seq_used);

        // If the slave stays in Reset the chaining model may be wrong —
        // alternate strict/lenient for the next first-frame until it moves.
        if (!model_locked_ && ++stall_cycles_ >= kStallCycles) {
            lenient_first_frame_ = !lenient_first_frame_;
            first_after_reset_ = true;
            next_seq_ = kInitialSeqNo;
            stall_cycles_ = 0;
        }
        return n;
    }

private:
    uint16_t slave_crc0_ = 0;
    uint16_t next_seq_ = kInitialSeqNo;
    bool first_after_reset_ = true;
    bool lenient_first_frame_ = false;
    bool model_locked_ = false;
    uint32_t stall_cycles_ = 0;
};

// ---------------------------------------------------------------------------
// SlaveFrameRewriter — rebuilds slave→master frames for the master
// ---------------------------------------------------------------------------
//
// Fed the slave peer's outgoing frames plus the master's latest TX frame.
// Emits a spec-conformant slave frame:
//
//   Reset      → (startCrc = 0, seq = 1)
//   non-Reset  → (startCrc = master's last TX CRC0, seq = that frame's seq)
//
// The master's last-TX CRC0 is read straight out of its frame bytes; its
// sequence number is not transmitted, so noteMasterFrame() identifies it by
// CRC-verifying each *new* master frame against candidate chain states:
//   (0, 1)                       — chain reset (Reset / first post-Reset frame)
//   (latest slave CRC0, seq+1)   — normal cross-direction progression
//   (previous slave CRC0, seq+1) — same, one exchange cycle of skew
//   (own last CRC0, seq+1)       — self-chaining variant
//   (…, seq)                     — same seq re-used with a new chain
// If none verify, the frame's CRC0 is still adopted and the last seq kept —
// the resulting responses will fail the master's CRC check until the master
// resets the connection, at which point tracking resynchronizes on the
// chain-reset frames.
class SlaveFrameRewriter {
public:
    void reset() {
        master_crc0_ = 0;
        master_seq_ = kInitialSeqNo;
        curr_slave_crc0_ = 0;
        prev_slave_crc0_ = 0;
        last_len_ = 0;
        desynced_ = false;
    }

    /// Observe the slave's latest TX frame — maintains the current and
    /// previous stored CRC0 used to identify the master's chain base.
    void noteSlaveFrame(const uint8_t* frame, size_t len) {
        if (!frame || len == 0 || !isFSoECommand(frame[0])) return;
        const uint16_t crc0 = storedCrc0(frame, len);
        if (crc0 != curr_slave_crc0_) {
            prev_slave_crc0_ = curr_slave_crc0_;
            curr_slave_crc0_ = crc0;
        }
    }

    /// Observe the master's latest TX frame — tracks its CRC0 and sequence
    /// number so responses can be built with the values the master will
    /// validate against.
    void noteMasterFrame(const uint8_t* frame, size_t len) {
        if (!frame || len > last_frame_.size()) return;
        if (len == last_len_ &&
            std::memcmp(frame, last_frame_.data(), len) == 0) {
            return;  // identical re-send — the master did not advance seq
        }

        uint8_t cmd = 0;
        size_t data_len = 0;
        uint16_t conn_id = 0;
        uint8_t data[CRC::MAX_PARSE_DATA_SIZE];
        if (!CRC::extractFSoEFrame(frame, len, cmd, data, data_len, conn_id) ||
            !isFSoECommand(cmd)) {
            return;
        }

        if (len <= CRC::MIN_FSOE_FRAME_SIZE) {
            // Bare Reset carries no CRC — the chain resets.
            master_crc0_ = 0;
            master_seq_ = kInitialSeqNo;
            desynced_ = false;
            std::memcpy(last_frame_.data(), frame, len);
            last_len_ = len;
            return;
        }

        const uint16_t stored = storedCrc0(frame, len);
        const int first_data = (len > 6) ? 2 : 1;
        const uint16_t next_seq = CRC::incrementSeqNo(master_seq_);
        const struct { uint16_t start_crc, seq; } candidates[] = {
            {0, kInitialSeqNo},              // chain reset (Reset, 1st frame)
            {curr_slave_crc0_, next_seq},    // chained from latest slave TX
            {prev_slave_crc0_, next_seq},    //   one cycle of skew
            {master_crc0_, next_seq},        // self-chaining
            {curr_slave_crc0_, master_seq_}, // same seq, refreshed chain
            {prev_slave_crc0_, master_seq_},
            {0, next_seq},
        };
        desynced_ = true;
        for (const auto& c : candidates) {
            if (CRC::computeCrc0(c.start_crc, conn_id, c.seq, cmd,
                                 data, first_data) == stored) {
                master_seq_ = c.seq;
                desynced_ = false;
                break;
            }
        }
        master_crc0_ = stored;
        std::memcpy(last_frame_.data(), frame, len);
        last_len_ = len;
    }

    /// Rewrite one incoming slave frame for delivery to the master.
    /// Returns the output frame length, or 0 if the input is not a
    /// recognizable FSoE frame.
    size_t rewrite(const uint8_t* in, size_t in_len,
                   uint8_t* out, size_t out_cap) {
        uint8_t cmd = 0;
        size_t data_len = 0;
        uint16_t conn_id = 0;
        uint8_t data[CRC::MAX_PARSE_DATA_SIZE];
        if (!CRC::extractFSoEFrame(in, in_len, cmd, data, data_len, conn_id) ||
            !isFSoECommand(cmd)) {
            return 0;
        }
        if (CRC::fsoeFrameSize(data_len) > out_cap) return 0;

        if (cmd == Command::Reset) {
            return CRC::buildFSoEFrame(out, cmd, data, data_len, conn_id,
                                       /*start_crc=*/0, kInitialSeqNo);
        }
        return CRC::buildFSoEFrameWithCollisionAvoidance(
            out, cmd, data, data_len, conn_id,
            master_crc0_, master_seq_, nullptr, nullptr);
    }

    /// True when the last master frame's (startCrc, seq) could not be
    /// identified — responses built in this state are best-effort.
    bool desynced() const { return desynced_; }

private:
    uint16_t master_crc0_ = 0;
    uint16_t master_seq_ = kInitialSeqNo;
    uint16_t curr_slave_crc0_ = 0;
    uint16_t prev_slave_crc0_ = 0;
    std::array<uint8_t, 64> last_frame_{};
    size_t last_len_ = 0;
    bool desynced_ = false;
};

} // namespace FSoE

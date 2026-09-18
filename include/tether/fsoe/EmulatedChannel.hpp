#pragma once

// One FSoE channel whose wire frames live at byte offsets inside a
// larger shared PDO region (e.g. the ESC211's 496-byte SafetyPDU maps).
//
// Two roles share the channel through a small mutex-protected mailbox:
//
//   PDO-exchange side    — slices the channel's master→slave frame out
//                          of the shared RX window (publishMasterFrame)
//                          and copies the emulated slave→master response
//                          back into the shared TX window
//                          (readSlaveResponse).  When the cmd offset is
//                          not yet known it is auto-detected from live
//                          traffic via detectCmdOffset().
//
//   Emulator side        — free-running tick: consume the newest master
//                          frame through FSoESlave::processRxFrame(),
//                          advance the watchdog timer, run the profile
//                          emulator's command/status sync hook,
//                          prepareTxFrame() and publish the response.
//
// The FSoESlave object is not owned — pass it to tick() so the channel
// stays agnostic of the profile-specific emulator wrapper
// (FSoESlaveEmulator<Codec,...>).

#include <atomic>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <functional>
#include <mutex>

#include "tether/fsoe/FSoEDefs.hpp"
#include "tether/fsoe/FSoESlave.hpp"

namespace FSoE {

/// Returns true when `cmd` is a recognised FSoE command byte.
inline bool isValidCommandByte(uint8_t cmd) {
    return cmd == Command::ProcessData ||
           cmd == Command::Reset ||
           cmd == Command::Session ||
           cmd == Command::Connection ||
           cmd == Command::Parameter ||
           cmd == Command::FailSafeData;
}

/// Offset returned by findFrameOffset() when no frame matched.
inline constexpr size_t kFrameOffsetNotFound = ~size_t{0};

/// Scan `region` for a wire frame of `frame_len` whose first byte is a
/// valid FSoE command and whose trailing two bytes (little-endian) equal
/// `conn_id`.  Returns the first matching offset or kFrameOffsetNotFound.
/// conn_id == 0 never matches.
inline size_t findFrameOffset(const uint8_t* region, size_t region_len,
                              size_t frame_len, uint16_t conn_id) {
    if (conn_id == 0 || frame_len < 3 || frame_len > region_len) {
        return kFrameOffsetNotFound;
    }
    for (size_t p = 0; p + frame_len <= region_len; ++p) {
        if (!isValidCommandByte(region[p])) continue;
        const uint16_t cid =
            static_cast<uint16_t>(region[p + frame_len - 2]) |
            (static_cast<uint16_t>(region[p + frame_len - 1]) << 8);
        if (cid == conn_id) return p;
    }
    return kFrameOffsetNotFound;
}

/// Mailbox + wire-window state for one channel.  MaxFrame bounds the
/// wire frames (FSoE spec max is well below 64 bytes).
template <size_t MaxFrame = 64>
class EmulatedChannel {
public:
    EmulatedChannel() = default;

    // --- PDO-exchange side -------------------------------------------------

    /// Publish the newest master→slave frame for the emulator thread.
    void publishMasterFrame(const uint8_t* data, size_t len) {
        std::lock_guard<std::mutex> g(rx_mtx_);
        const size_t n = len < MaxFrame ? len : MaxFrame;
        std::memcpy(master_frame_, data, n);
        master_frame_len_ = n;
        master_frame_valid_ = true;
        ++master_frame_seq_;
    }

    /// Clear the published master frame (e.g. after a session reset).
    /// Resets both sequence counters so a republished frame can never
    /// collide with an already-consumed sequence number.
    void clearMasterFrame() {
        std::lock_guard<std::mutex> g(rx_mtx_);
        master_frame_valid_ = false;
        master_frame_seq_ = 0;
        last_consumed_seq_ = 0;
    }

    /// Copy the latest slave→master response into `out`.
    /// Returns false when nothing has been published yet.
    bool readSlaveResponse(uint8_t* out, size_t cap, size_t& len) const {
        std::lock_guard<std::mutex> g(tx_mtx_);
        if (!slave_response_valid_) return false;
        len = slave_response_len_ < cap ? slave_response_len_ : cap;
        std::memcpy(out, slave_response_, len);
        return true;
    }

    /// Cmd-offset auto-detect: call each cycle while !cmd_locked.load().
    /// Locks cmd_offset after the same candidate matches on two
    /// consecutive calls (single PDO-side caller — no atomics needed for
    /// the bookkeeping).  Returns true on the call that locks.
    bool detectCmdOffset(const uint8_t* region, size_t region_len) {
        const size_t p =
            findFrameOffset(region, region_len, cmd_len, conn_id);
        if (p == kFrameOffsetNotFound) {
            detect_hits_ = 0;
            return false;
        }
        if (detect_candidate_ == p && ++detect_hits_ >= 2) {
            cmd_offset.store(static_cast<uint16_t>(p),
                             std::memory_order_relaxed);
            cmd_locked.store(true, std::memory_order_relaxed);
            return true;
        }
        if (detect_candidate_ != p) {
            detect_candidate_ = p;
            detect_hits_ = 1;
        }
        return false;
    }

    // --- Emulator side -----------------------------------------------------

    /// Fetch the newest master→slave frame if one arrived since the last
    /// call.  Returns true (frame in `out`, wire length in `len`).
    bool takeMasterFrame(uint8_t* out, size_t cap, size_t& len) {
        std::lock_guard<std::mutex> g(rx_mtx_);
        if (!master_frame_valid_ || master_frame_seq_ == last_consumed_seq_) {
            return false;
        }
        len = master_frame_len_ < cap ? master_frame_len_ : cap;
        std::memcpy(out, master_frame_, len);
        last_consumed_seq_ = master_frame_seq_;
        return true;
    }

    /// Publish a new slave→master response (from prepareTxFrame()).
    void publishSlaveResponse(const uint8_t* data, size_t len) {
        std::lock_guard<std::mutex> g(tx_mtx_);
        const size_t n = len < MaxFrame ? len : MaxFrame;
        std::memcpy(slave_response_, data, n);
        slave_response_len_ = n;
        slave_response_valid_ = true;
    }

    /// Result of one tick().
    struct TickResult {
        bool new_frame = false;    ///< a fresh master frame was processed
        uint8_t master_cmd = 0;    ///< command byte of that frame
        bool process_ok = true;    ///< processRxFrame() result
        size_t frame_len = 0;      ///< processed wire length (0 on invalid cmd)
        size_t wire_len = 0;       ///< raw received wire length
        uint8_t master_frame[MaxFrame] = {};  ///< copy of that frame
    };

    /// One emulator tick: consume the newest master frame (if any) through
    /// slave.processRxFrame(), advance slave.update(time_ms), run `sync`
    /// (e.g. emulator->synchronizeCommandAndStatus()), then prepareTxFrame()
    /// and publish the slave→master response.
    ///
    /// On a new frame with an invalid command byte the frame is NOT fed to
    /// the slave (process_ok=false, frame_len=0) — matching the behaviour
    /// the callers implemented by hand.
    TickResult tick(FSoESlave& slave, uint64_t time_ms,
                    const std::function<void()>& sync = {}) {
        TickResult r;
        uint8_t frame[MaxFrame] = {};
        size_t len = 0;
        if (takeMasterFrame(frame, sizeof(frame), len)) {
            r.new_frame = true;
            r.master_cmd = frame[0];
            r.wire_len = len;
            std::memcpy(r.master_frame, frame, len);
            if (isValidCommandByte(r.master_cmd)) {
                r.frame_len = len;
                r.process_ok = slave.processRxFrame(frame, len);
            } else {
                r.process_ok = false;
            }
        }
        slave.update(time_ms);
        if (sync) sync();

        uint8_t tx[MaxFrame] = {};
        const size_t tx_len = slave.prepareTxFrame(tx, sizeof(tx));
        publishSlaveResponse(tx, tx_len);
        return r;
    }

    // --- Wire-window state ---------------------------------------------------
    // Atomics: written by the PDO-exchange side / FNI setup, read by both.

    std::atomic<uint16_t> cmd_offset{0};   ///< cmd frame offset in RX window
    std::atomic<bool>     cmd_locked{false};
    std::atomic<uint16_t> resp_offset{0};  ///< response offset in TX window
    std::atomic<bool>     resp_locked{false};

    size_t cmd_len = 0;      ///< master→slave wire length
    size_t resp_len = 0;     ///< slave→master wire length
    uint16_t conn_id = 0;    ///< FSoE connection ID (drives auto-detect)
    size_t index = 0;        ///< channel index (for logging)

private:
    // master→slave mailbox (PDO thread writes, emulator thread reads)
    mutable std::mutex rx_mtx_;
    uint8_t master_frame_[MaxFrame] = {};
    size_t master_frame_len_ = 0;
    bool master_frame_valid_ = false;
    uint64_t master_frame_seq_ = 0;
    uint64_t last_consumed_seq_ = 0;  // emulator side only

    // slave→master mailbox (emulator thread writes, PDO thread reads)
    mutable std::mutex tx_mtx_;
    uint8_t slave_response_[MaxFrame] = {};
    size_t slave_response_len_ = 0;
    bool slave_response_valid_ = false;

    // Auto-detect bookkeeping (PDO side only).
    size_t detect_candidate_ = 0;
    int detect_hits_ = 0;
};

} // namespace FSoE

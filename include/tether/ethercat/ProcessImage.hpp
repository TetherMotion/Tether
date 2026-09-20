#pragma once

/**
 * @file ProcessImage.hpp
 * @brief Zero-copy-capable process image for the cyclic LRW exchange.
 *
 * The EtherCAT LRW datagram payload *is* a fixed-layout process image:
 * RxPDO region [0, rx_bytes) written by the master, TxPDO region
 * [rx_bytes, rx_bytes + tx_bytes) written by slaves.  Instead of the legacy
 * per-entry app_buffer gather/scatter, an application can read/write the
 * image in place through this API.
 *
 * Modes (hierarchy safe → unsafe):
 *
 *   Buffered       — image API inactive; entries use app_buffer memcpy.
 *   DoubleBuffered — writer owns a *persistent* write buffer; the wire owns
 *                    a separate send snapshot.  Untouched fields keep their
 *                    last values forever — partial writes are inherently
 *                    safe.  outputWrite() is a stable pointer.  The writer
 *                    calls commitOutputs() at the end of a write burst;
 *                    acquireSendImage() then refreshes the snapshot under a
 *                    seqlock (bounded retries — naturally-aligned ≤4-byte
 *                    fields are single-copy atomic and never tear; wider
 *                    bursts should be wrapped in beginWrite()/
 *                    commitOutputs() to be detected).  Without a commit the
 *                    previous snapshot is resent.
 *   Direct         — one staging image; outputWrite() points into it.
 *                    Safe when the only writer runs in-loop (MotionControl
 *                    phase) or when all written fields are naturally aligned
 *                    ≤4-byte values (single-copy atomic on x86/ARMv7+).
 *                    An asynchronous writer tearing a wider field is the
 *                    documented hazard.
 *   TripleBuffered — three output buffers rotating through write/ready/send
 *                    roles via a lock-free index pack.  The writer calls
 *                    commitOutputs() to publish; tearing is impossible for
 *                    any field size.  Contract: the writer must fully
 *                    refresh the output region per commit — untouched
 *                    fields keep each buffer's own history (no carry-
 *                    forward across commits).  Recommended for external
 *                    motion sources doing full-refresh writes.
 *   Rotating       — outputWrite() points into the channel's *next TX frame*
 *                    payload region (zero copies even on the ring backend).
 *                    Contract: the writer must fully refresh the output
 *                    region every cycle and re-query the pointer per cycle;
 *                    stale bytes would otherwise be sent verbatim.  The
 *                    unsafe-but-fast option.
 *
 * Input (TxPDO) side is always view-based: publishInputView() points
 * inputRead() at the received payload inside the channel's ring/bank memory
 * (held via the channel's cookie mechanism), publishInputCopy() stages it
 * into an owned bank when no channel exists.  inputSeq() bumps per publish —
 * readers detect new data or staleness.
 *
 * Entry addressing: entryOffset(i) is the byte offset of mapping entry i
 * inside the image, computed at configure() time with the same layout walk
 * as LogicalAddressManager::describeEntries().  Entries sharing a byte with
 * a neighbour (bit-packed PDOs) or flagged PDOEntry::image_exclude are
 * marked -1 → they stay on the app_buffer path even in image modes.
 */

#include <atomic>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <memory>

namespace EtherCAT {

class ICyclicChannel;   // fwd — rxRelease hook for held input views

enum class ImageMode : uint8_t {
    Buffered = 0,
    DoubleBuffered,
    Direct,
    TripleBuffered,
    Rotating,
};

class ProcessImage {
public:
    static constexpr size_t kMaxEntries = 128;

    struct Config {
        ImageMode mode = ImageMode::Buffered;
        uint32_t  rx_bytes = 0;          ///< output region bytes (RxPDO)
        uint32_t  tx_bytes = 0;          ///< input region bytes (TxPDO)
        /// Byte offset of each mapping entry in the image; -1 = not image
        /// mapped (forced-buffered).  May be null → all entries buffered.
        const int32_t* entry_offsets = nullptr;
        size_t         entry_count   = 0;
    };

    ProcessImage() = default;
    ~ProcessImage() { clearInputHold(); }

    ProcessImage(const ProcessImage&) = delete;
    ProcessImage& operator=(const ProcessImage&) = delete;

    // ====================================================================
    // Configuration (non-RT — call before the cyclic loop starts)
    // ====================================================================

    bool configure(const Config& cfg);

    bool      configured() const { return size_ > 0; }
    ImageMode mode()       const { return mode_; }
    uint32_t  epoch()      const { return epoch_.load(std::memory_order_acquire); }
    uint32_t  outputBytes() const { return rx_bytes_; }
    uint32_t  inputBytes()  const { return tx_bytes_; }
    uint32_t  imageBytes()  const { return size_; }

    // ====================================================================
    // Application-facing API (any thread)
    // ====================================================================

    /**
     * @brief Output region to write this cycle's setpoints into.
     *
     * Direct: the staging image.  TripleBuffered: the current write buffer.
     * Rotating: the channel's pending TX payload region — nullptr after the
     * frame commits until the next cycle attaches a fresh one (re-query per
     * cycle; writes to nullptr are dropped).
     * Buffered: nullptr.
     */
    uint8_t* outputWrite() {
        switch (mode_) {
            case ImageMode::Direct:  return img_.get();
            case ImageMode::DoubleBuffered:
                return dbl_[0].get();      // persistent writer buffer
            case ImageMode::TripleBuffered:
                return tri_[triState_.load(std::memory_order_acquire) & 3].get();
            case ImageMode::Rotating:
                return rotating_payload_.load(std::memory_order_acquire);
            default: return nullptr;
        }
    }

    /**
     * @brief Latest published input image (full LRW payload: outputs echo
     *        at [0, rx_bytes), inputs at [rx_bytes, size)).
     *        nullptr until the first publish; may point into channel-owned
     *        ring memory — valid until inputSeq() bumps again.
     */
    const uint8_t* inputRead() const {
        return in_ptr_.load(std::memory_order_acquire);
    }

    /// Monotonic publish counter — bumps once per published input frame.
    uint64_t inputSeq() const { return in_seq_.load(std::memory_order_acquire); }

    /// Byte offset of mapping entry i in the image; -1 = buffered-only.
    int32_t entryOffset(size_t i) const {
        return i < entry_count_ ? entry_off_[i] : -1;
    }

    /// Typed field accessors (nullptr for unmapped/buffered entries).
    template<typename T>
    T* outputPtr(size_t entry_idx) {
        const int32_t off = entryOffset(entry_idx);
        uint8_t* base = outputWrite();
        if (off < 0 || !base ||
            static_cast<uint32_t>(off) + sizeof(T) > rx_bytes_) return nullptr;
        return reinterpret_cast<T*>(base + off);
    }
    template<typename T>
    const T* inputPtr(size_t entry_idx) const {
        const int32_t off = entryOffset(entry_idx);
        const uint8_t* base = inputRead();
        if (off < 0 || !base ||
            static_cast<uint32_t>(off) + sizeof(T) > size_) return nullptr;
        return reinterpret_cast<const T*>(base + off);
    }

    /**
     * @brief DoubleBuffered: mark the start of a write burst so the
     *        seqlock copy in acquireSendImage() can detect mid-burst
     *        snapshots.  Optional — unnecessary when every written field
     *        is naturally aligned and ≤4 bytes (single-copy atomic).
     */
    void beginWrite() {
        if (mode_ == ImageMode::DoubleBuffered)
            dblSeq_.fetch_or(0x01, std::memory_order_acq_rel);
    }

    /**
     * @brief Publish the completed write burst.
     *
     * TripleBuffered: write buffer becomes the ready image (lock-free
     * role swap).  DoubleBuffered: bumps the seqlock, clears
     * writer-active and sets dirty — the next acquireSendImage()
     * refreshes the send snapshot.  Other modes: no-op.
     */
    void commitOutputs() {
        if (mode_ == ImageMode::DoubleBuffered) {
            uint32_t s = dblSeq_.load(std::memory_order_acquire);
            // bump seq (bits 31:2), set dirty (bit1), clear writing (bit0)
            while (!dblSeq_.compare_exchange_weak(
                       s, ((s & ~0x03u) + 4u) | 0x02u,
                       std::memory_order_acq_rel,
                       std::memory_order_acquire)) {}
            return;
        }
        if (mode_ != ImageMode::TripleBuffered) return;
        // state pack: bits [1:0]=write, [3:2]=ready, [5:4]=send, bit6=fresh
        uint8_t s = triState_.load(std::memory_order_acquire);
        while (!triState_.compare_exchange_weak(
                   s, static_cast<uint8_t>(swapRoles(s, 0, 2) | 0x40),
                   std::memory_order_acq_rel, std::memory_order_acquire)) {}
    }

    // ====================================================================
    // Cyclic-side API (the executive thread only)
    // ====================================================================

    /**
     * @brief Image whose output region goes onto the wire this cycle.
     *
     * Direct/TripleBuffered → an owned image buffer (used as txSendParts
     * payload — zero-copy on the socket backend).
     * Rotating → nullptr (the payload already lives inside the acquired
     * frame; use rotatingFrameBase() + sendCyclicFrame instead).
     * Buffered → nullptr (caller uses its own staging buffer).
     */
    const uint8_t* acquireSendImage() {
        switch (mode_) {
            case ImageMode::Direct: return img_.get();
            case ImageMode::DoubleBuffered: {
                // dbl_[0] = persistent write buf, dbl_[1] = send snapshot.
                uint32_t s = dblSeq_.load(std::memory_order_acquire);
                if (!(s & 0x02)) return dbl_[1].get();    // clean → resend
                // Claim the dirty work first — a commit landing during the
                // copy sets dirty again and triggers another refresh.
                while (!dblSeq_.compare_exchange_weak(
                           s, s & ~0x02u, std::memory_order_acq_rel,
                           std::memory_order_acquire)) {}
                // Seqlock copy W → S; a concurrent begin/commit bumps the
                // word and invalidates the snapshot.
                for (int i = 0; i < 8; ++i) {
                    const uint32_t s0 = dblSeq_.load(std::memory_order_acquire);
                    if (s0 & 0x01) continue;              // writer mid-burst
                    std::memcpy(dbl_[1].get(), dbl_[0].get(), rx_bytes_);
                    if (dblSeq_.load(std::memory_order_acquire) == s0)
                        return dbl_[1].get();
                }
                // Pathological writer churn — send the best-effort copy;
                // aligned ≤4-byte fields are still coherent values.
                std::memcpy(dbl_[1].get(), dbl_[0].get(), rx_bytes_);
                return dbl_[1].get();
            }
            case ImageMode::TripleBuffered: {
                // bit6 = ready holds an unpublished commit.
                uint8_t s = triState_.load(std::memory_order_acquire);
                for (;;) {
                    if (!(s & 0x40))
                        return tri_[(s >> 4) & 3].get();  // resend last
                    const uint8_t next = static_cast<uint8_t>(
                        swapRoles(s, 2, 4) & ~0x40);   // ready <-> send
                    if (triState_.compare_exchange_weak(
                            s, next, std::memory_order_acq_rel,
                            std::memory_order_acquire))
                        return tri_[(s >> 2) & 3].get(); // old ready → send
                }
            }
            default: return nullptr;
        }
    }

    /**
     * @brief Rotating: attach the channel's next TX frame base so
     *        outputWrite() exposes its payload region.  Called by the
     *        executive at frame-build time each cycle.  The frame header
     *        region [base, base+26) is written by the exchange; the payload
     *        region [base+26, +26+size) is what outputWrite() returns.
     */
    void attachTxFrame(uint8_t* frame_base, uint32_t frame_payload_off = 26) {
        rotating_frame_ = frame_base;
        rotating_payload_.store(frame_base ? frame_base + frame_payload_off
                                           : nullptr,
                                std::memory_order_release);
    }
    uint8_t* rotatingFrameBase() const { return rotating_frame_; }
    void     detachTxFrame() {
        rotating_frame_ = nullptr;
        rotating_payload_.store(nullptr, std::memory_order_release);
    }

    /**
     * @brief Publish a received payload as the input image — zero-copy view
     *        into channel memory.  Holds the cookie until the next publish.
     */
    void publishInputView(const uint8_t* payload, uint32_t len,
                          uint32_t cookie, ICyclicChannel* ch);

    /**
     * @brief Publish by copy into an owned bank — used when no channel
     *        exists (software deposit path).
     */
    void publishInputCopy(const uint8_t* payload, uint32_t len);

private:
    static uint8_t swapRoles(uint8_t s, int a, int b) {
        const uint8_t va = (s >> a) & 3, vb = (s >> b) & 3;
        return static_cast<uint8_t>((s & ~(3 << a) & ~(3 << b)) |
                                    (vb << a) | (va << b));
    }
    void clearInputHold();

    ImageMode mode_ = ImageMode::Buffered;
    uint32_t  rx_bytes_ = 0, tx_bytes_ = 0, size_ = 0;

    std::atomic<uint32_t> epoch_{0};

    // Direct: one image.  DoubleBuffered: write buf + send snapshot.
    // TripleBuffered: three.
    std::unique_ptr<uint8_t[]> img_;
    std::unique_ptr<uint8_t[]> dbl_[2];
    std::atomic<uint32_t>      dblSeq_{0};   // bits31:2 seq, bit1 dirty, bit0 writing
    std::unique_ptr<uint8_t[]> tri_[3];
    std::atomic<uint8_t>       triState_{0b100100};  // w=0,r=1,s=2 (+bit6 fresh)

    // Rotating: payload region inside the channel's pending TX frame.
    std::atomic<uint8_t*> rotating_payload_{nullptr};
    uint8_t*               rotating_frame_ = nullptr;

    // Input publish: view or owned bank copy.
    std::atomic<const uint8_t*> in_ptr_{nullptr};
    uint32_t                    in_len_ = 0;
    std::atomic<uint64_t>       in_seq_{0};
    // Held input cookie (channel view mode)
    ICyclicChannel* in_channel_ = nullptr;
    int64_t         in_cookie_  = -1;
    // Owned bank (copy mode)
    std::unique_ptr<uint8_t[]>  in_bank_;

    int32_t entry_off_[kMaxEntries] = {};
    size_t  entry_count_ = 0;
};

} // namespace EtherCAT

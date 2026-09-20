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

/**
 * @brief Epoch-checked handle to one mapping entry's image location.
 *
 * Acquired via entryHandle() / PDOManager::entryHandle(); carries the
 * image epoch at acquisition time.  resolve()/ptr accessors return
 * nullptr when the image was re-configured since — the raw-pointer
 * lifetime bug entryOffset()+arithmetic cannot catch.  offset < 0 marks
 * a forced-buffered entry (use its app_buffer path).
 */
struct EntryHandle {
    uint32_t index  = UINT32_MAX;   ///< mapping entry index
    int32_t  offset = -1;           ///< byte offset in the image (-1 = buffered)
    uint32_t epoch  = 0;            ///< image epoch when acquired

    bool valid()       const { return index != UINT32_MAX; }
    bool imageMapped() const { return offset >= 0; }
};

/// shm header — layout shared between the exporting master and any
/// process-external motion source attaching via attachShared().
/// Keep POD + naturally aligned atomics; the struct is mmap'd verbatim.
struct ShmImageHeader {
    uint32_t magic;           ///< kShmMagic
    uint32_t version;         ///< kShmVersion
    uint32_t rx_bytes;        ///< output region bytes
    uint32_t tx_bytes;        ///< input region bytes
    uint32_t header_bytes;    ///< = sizeof(ShmImageHeader) — fwd compat
    uint32_t epoch;
    std::atomic<uint32_t> in_seq;      ///< input publish counter (futex word)
    std::atomic<uint32_t> in_waiters;  ///< registered input waiters
    std::atomic<uint32_t> out_seq;     ///< output commit counter
    std::atomic<uint32_t> out_waiters; ///< registered output waiters
    std::atomic<uint32_t> send_seq;    ///< async-loop send-request counter
    std::atomic<uint32_t> send_waiters;///< registered send waiters
    uint32_t reserved[12];
};
inline constexpr uint32_t kShmMagic   = 0x54494D47;  ///< 'TIMG'
inline constexpr uint32_t kShmVersion = 1;
/// Payload layout: [ShmImageHeader][output rx_bytes][input tx_bytes],
/// regions 64-byte aligned after the header.
struct ShmImageLayout {
    static constexpr uint32_t kHeaderPadded = 256;
    static uint32_t outputOff()                { return kHeaderPadded; }
    static uint32_t inputOff(uint32_t rx)      { return kHeaderPadded + rx; }
    static uint32_t totalBytes(uint32_t rx, uint32_t tx) {
        return kHeaderPadded + rx + tx;
    }
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
        /// Optional POSIX shm export name (e.g. "tether-img" — the leading
        /// '/' is added internally).  When set, the image's output and
        /// input regions live in a shared-memory segment so a
        /// process-external motion source can attachShared() to it.
        /// Forces Direct-style semantics (zero copy both ways).
        const char* shm_name = nullptr;
    };

    ProcessImage() = default;
    ~ProcessImage() { clearInputHold(); releaseShm(); }

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
            case ImageMode::Direct:  return shm_out_ ? shm_out_ : img_.get();
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
    /// shm-attached readers see the exporting process's publishes.
    uint64_t inputSeq() const {
        return ext_in_seq_
            ? ext_in_seq_->load(std::memory_order_acquire)
            : in_seq_.load(std::memory_order_acquire);
    }

    /// Byte offset of mapping entry i in the image; -1 = buffered-only.
    /// Legacy raw access — prefer entryHandle() for epoch-checked use.
    int32_t entryOffset(size_t i) const {
        return i < entry_count_ ? entry_off_[i] : -1;
    }

    /// Epoch-checked handle for entry i.  invalid() when i is out of range.
    EntryHandle entryHandle(size_t i) const {
        if (i >= entry_count_) return {};
        return { static_cast<uint32_t>(i), entry_off_[i],
                 epoch_.load(std::memory_order_acquire) };
    }

    /// Resolve a handle to the current output image base + offset.
    /// nullptr when the epoch went stale or the entry is buffered-only.
    uint8_t* outputPtrRaw(const EntryHandle& h) {
        if (!h.valid() || h.offset < 0 ||
            h.epoch != epoch_.load(std::memory_order_acquire)) return nullptr;
        uint8_t* base = outputWrite();
        if (!base || static_cast<uint32_t>(h.offset) >= rx_bytes_)
            return nullptr;
        return base + h.offset;
    }
    const uint8_t* inputPtrRaw(const EntryHandle& h) const {
        if (!h.valid() || h.offset < 0 ||
            h.epoch != epoch_.load(std::memory_order_acquire)) return nullptr;
        const uint8_t* base = inputRead();
        if (!base || static_cast<uint32_t>(h.offset) >= size_)
            return nullptr;
        return base + h.offset;
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
    /// Handle variants — stale-epoch safe.
    template<typename T> T* outputPtr(const EntryHandle& h) {
        return reinterpret_cast<T*>(outputPtrRaw(h));
    }
    template<typename T> const T* inputPtr(const EntryHandle& h) const {
        return reinterpret_cast<const T*>(inputPtrRaw(h));
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
        if (shm_attached_ && shm_out_seq_) {
            shm_out_seq_->fetch_add(1, std::memory_order_release);
        }
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
            case ImageMode::Direct:
                return shm_out_ ? shm_out_ : img_.get();
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

    // ====================================================================
    // Multi-part publish + cross-thread/process wait
    // ====================================================================

    /**
     * @brief Writable staging bank for multi-part input publishes
     *        (multi-slice images).  Copy each slice's payload at its
     *        image offset, then call commitInput() once — readers see a
     *        single coherent publish.  In shm mode this returns the shm
     *        input region directly.  nullptr when not configured.
     */
    uint8_t* inputWriteBank();

    /**
     * @brief Publish the staged input bank as one input update.
     *        Bumps inputSeq() and wakes any waitInput() sleepers
     *        (futex — works across processes in shm mode).
     */
    void commitInput();

    /**
     * @brief Block until inputSeq() differs from @p last_seq or the
     *        timeout elapses — the cycle-tick for an external motion
     *        thread/process.  Linux: futex on the publish counter
     *        (cross-process when shm mode is active).  Elsewhere: bounded
     *        spin+yield.  Returns true when new input was published.
     */
    bool waitInput(uint64_t last_seq, uint32_t timeout_ns);

    // ====================================================================
    // Shared-memory export/attach (process-external motion source)
    // ====================================================================

    /**
     * @brief Attach to an image exported by another process's
     *        configure(shm_name=...).  After a successful attach this
     *        instance behaves like Direct mode over the shared regions:
     *        outputWrite() → shm output, inputRead() → shm input,
     *        waitInput() blocks on the shm futex word, commitOutputs()
     *        bumps the shm output counter.  Entry offsets are left
     *        unmapped (-1) — address fields by raw byte offset.
     * @param name  shm name as passed to the creator's Config::shm_name
     *              (without or with leading '/')
     */
    bool attachShared(const char* name);

    /// True when this instance's regions live in shared memory.
    bool shmBacked() const { return shm_map_ != nullptr; }

    /// Output commit counter — bumps when the external writer calls
    /// commitOutputs() on an attached image (master-side observability).
    uint32_t outputSeq() const {
        return shm_out_seq_
            ? shm_out_seq_->load(std::memory_order_acquire)
            : out_seq_.load(std::memory_order_acquire);
    }

    // ====================================================================
    // Async-loop send trigger (producer side: any thread/process)
    // ====================================================================

    /**
     * @brief Request an asynchronous RxPDO send — the "on change" edge for
     *        the AsyncCyclicLoop.
     *
     * The producer commits its new output data (commitOutputs() first),
     * then calls triggerSend() to fire the wire send.  Deliberately
     * separate from commitOutputs(): committing means "data ready" in
     * *every* loop mode and must not imply a send — the cyclic loop
     * transmits on its own deadline regardless.  Bumps sendSeq() and
     * wakes any waitSend() sleeper (cross-process when shm-backed).
     */
    void triggerSend() {
        auto* w = sendSeqWord();
        w->fetch_add(1, std::memory_order_release);
        // Only pay the wake syscall when a waiter is registered — the
        // waiter registers before re-checking the word, so a trigger
        // landing between the check and the registration is still seen.
        if (sendWaitersWord()->load(std::memory_order_acquire) > 0)
            sendWakeAll();
    }

    /// Send-request counter — bumped once per triggerSend().
    uint32_t sendSeq() const {
        return sendSeqWord()->load(std::memory_order_acquire);
    }

    /**
     * @brief Block until sendSeq() differs from @p last_seq or
     *        @p timeout_ns elapses — a *relative* timeout, so callers in
     *        any clock domain can pass their remaining budget directly.
     *
     * timeout_ns == UINT64_MAX waits indefinitely (C++20 atomic::wait
     *        in-process; shared FUTEX_WAIT when shm-backed).  Returns true
     *        when the send-request counter changed — callers re-check
     *        their own stop flag before acting on the wake.
     */
    bool waitSend(uint32_t last_seq, uint64_t timeout_ns);

private:
    static uint8_t swapRoles(uint8_t s, int a, int b) {
        const uint8_t va = (s >> a) & 3, vb = (s >> b) & 3;
        return static_cast<uint8_t>((s & ~(3 << a) & ~(3 << b)) |
                                    (vb << a) | (va << b));
    }
    void clearInputHold();
    void publishNotify();          // seq bump + futex wake
    void futexWakeAll();
    bool futexWait(uint32_t expected, uint32_t timeout_ns);
    // Word-addressable variants — send trigger + shm paths.
    void futexWakeAllOn(std::atomic<uint32_t>* word);
    bool futexWaitOn(std::atomic<uint32_t>* word, uint32_t expected,
                     int64_t timeout_ns);   // <0 → untimed
    void sendWakeAll();
    /// Active send-request word: shm header field when backed, else the
    /// in-process member.  Non-const — producers bump it.
    std::atomic<uint32_t>* sendSeqWord() {
        return shm_send_seq_ ? shm_send_seq_ : &send_seq_;
    }
    const std::atomic<uint32_t>* sendSeqWord() const {
        return shm_send_seq_ ? shm_send_seq_ : &send_seq_;
    }
    std::atomic<uint32_t>* sendWaitersWord() {
        return shm_send_waiters_ ? shm_send_waiters_ : &send_waiters_;
    }
    void releaseShm();
    bool mapShm(const char* name, bool create, uint32_t rx, uint32_t tx);

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

    // Publish notification — futex word + waiter count.  In shm mode these
    // alias the shm header's fields so they work across processes.
    std::atomic<uint32_t>  in_pub_ctr_{0};
    std::atomic<uint32_t>  in_waiters_{0};
    std::atomic<uint32_t>* pub_ctr_     = nullptr;  // set by configure/attach
    std::atomic<uint32_t>* pub_waiters_ = nullptr;
    /// External input sequence source — points at the shm header's in_seq
    /// when shm-backed so inputSeq() sees the exporting process's count.
    const std::atomic<uint32_t>* ext_in_seq_ = nullptr;
    std::atomic<uint32_t>  out_seq_{0};
    // Async send trigger — in-process word + waiters; shm aliases set by
    // mapShm so a process-external producer's triggerSend() wakes the
    // master's loop via shared futex.
    std::atomic<uint32_t>  send_seq_{0};
    std::atomic<uint32_t>  send_waiters_{0};
    bool futex_shared_ = false;

    // shm export/attach state (all null/-1 when not backed)
    void*    shm_map_     = nullptr;
    size_t   shm_map_len_ = 0;
    int      shm_fd_      = -1;
    bool     shm_owner_   = false;    ///< this instance created the segment
    bool     shm_attached_ = false;   ///< this instance attached to one
    char     shm_name_[64] = {};
    uint8_t* shm_out_ = nullptr;      ///< shm output region
    uint8_t* shm_in_  = nullptr;      ///< shm input region
    std::atomic<uint32_t>* shm_out_seq_ = nullptr;
    std::atomic<uint32_t>* shm_send_seq_ = nullptr;
    std::atomic<uint32_t>* shm_send_waiters_ = nullptr;

    int32_t entry_off_[kMaxEntries] = {};
    size_t  entry_count_ = 0;
};

} // namespace EtherCAT

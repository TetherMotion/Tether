/**
 * @file ProcessImage.cpp
 * @brief Process image configuration, input publish, futex wait, shm
 *        export/attach implementation.
 */

#include "tether/ethercat/ProcessImage.hpp"
#include "tether/ethercat/CyclicChannel.hpp"
#include "logging/Logger.hpp"

#include <algorithm>
#include <cerrno>
#include <cstring>
#include <ctime>

#ifdef __linux__
#include <fcntl.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <sys/syscall.h>
#include <unistd.h>
#include <linux/futex.h>
#elif defined(_WIN32)
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <windows.h>
#include <synchapi.h>
#pragma comment(lib, "synchronization.lib")
#else
#include <thread>
#endif

namespace EtherCAT {

static const char* TAG = "process_image";

static uint64_t monoNowNs() {
#if defined(_WIN32)
    LARGE_INTEGER f, c;
    ::QueryPerformanceFrequency(&f);
    ::QueryPerformanceCounter(&c);
    return static_cast<uint64_t>(c.QuadPart) * 1'000'000'000ull /
           static_cast<uint64_t>(f.QuadPart);
#else
    timespec ts{};
    ::clock_gettime(CLOCK_MONOTONIC, &ts);
    return static_cast<uint64_t>(ts.tv_sec) * 1'000'000'000ull
         + static_cast<uint64_t>(ts.tv_nsec);
#endif
}

// ============================================================================
// Configuration
// ============================================================================

bool ProcessImage::configure(const Config& cfg) {
    releaseShm();
    rx_bytes_ = cfg.rx_bytes;
    tx_bytes_ = cfg.tx_bytes;
    size_     = std::max<uint32_t>(cfg.rx_bytes + cfg.tx_bytes,
                                   cfg.image_bytes);
    mode_     = cfg.mode;
    entry_count_ = cfg.entry_count < kMaxEntries ? cfg.entry_count
                                                 : kMaxEntries;
    if (cfg.entry_offsets) {
        std::memcpy(entry_off_, cfg.entry_offsets,
                    entry_count_ * sizeof(int32_t));
    } else {
        for (size_t i = 0; i < entry_count_; ++i) entry_off_[i] = -1;
    }

    clearInputHold();
    img_.reset();
    for (auto& b : dbl_) b.reset();
    for (auto& b : tri_) b.reset();
    in_bank_.reset();
    in_ptr_.store(nullptr, std::memory_order_release);
    in_seq_.store(0, std::memory_order_release);
    rotating_frame_ = nullptr;
    rotating_payload_.store(nullptr, std::memory_order_release);
    pub_ctr_     = &in_pub_ctr_;
    pub_waiters_ = &in_waiters_;
    futex_shared_ = false;
    shm_out_seq_ = nullptr;
    shm_send_seq_ = nullptr;
    shm_send_waiters_ = nullptr;
    shm_attached_ = false;
    shm_entries_ = nullptr;
    shm_entry_count_ = 0;
    shm_entry_capacity_ = 0;
    shm_flags_ = 0;
    shm_ready_ = false;
    in_pub_ctr_.store(0, std::memory_order_release);
    in_waiters_.store(0, std::memory_order_release);
    send_seq_.store(0, std::memory_order_release);
    send_waiters_.store(0, std::memory_order_release);
    send_stamp_ns_.store(0, std::memory_order_release);
    send_epoch_.fetch_add(1, std::memory_order_acq_rel);   // reclaim stale
    for (auto& p : prod_triggers_) p.store(0, std::memory_order_release);
    prod_claims_.store(1, std::memory_order_release);  // slot0 = anon bucket
    send_consumer_.store(0, std::memory_order_release);
    cyclic_trigger_warned_.store(false, std::memory_order_release);

    if (size_ == 0 || mode_ == ImageMode::Buffered) {
        mode_ = ImageMode::Buffered;
        epoch_.fetch_add(1, std::memory_order_acq_rel);
        return size_ > 0;
    }

    // shm export forces Direct semantics — the regions are the segment.
    // The segment layout ([output rx][input tx][entries]) cannot express
    // dead space left by reallocated slave windows (extent > rx+tx), so
    // export is skipped for non-compact images rather than exporting a
    // truncated view.
    if (cfg.shm_name && cfg.shm_name[0] != '\0' &&
        size_ == rx_bytes_ + tx_bytes_) {
        const uint32_t rows = cfg.shm_entry_capacity
            ? cfg.shm_entry_capacity
            : static_cast<uint32_t>(entry_count_);
        if (!mapShm(cfg.shm_name, /*create=*/true, rx_bytes_, tx_bytes_,
                    rows, cfg.shm_seqlock)) {
            TETHER_LOGW(TAG, "shm export '{}' failed — falling back to "
                             "in-process image", cfg.shm_name);
        } else {
            mode_ = ImageMode::Direct;
        }
    }

    switch (mode_) {
        case ImageMode::Direct:
            if (!shm_out_) {   // shm mode owns its regions instead
                img_ = std::make_unique<uint8_t[]>(size_);
                std::memset(img_.get(), 0, size_);
            }
            break;
        case ImageMode::DoubleBuffered:
            for (auto& b : dbl_) {
                b = std::make_unique<uint8_t[]>(size_);
                std::memset(b.get(), 0, size_);
            }
            dblSeq_.store(0, std::memory_order_release);  // clean, idle
            break;
        case ImageMode::TripleBuffered:
            for (auto& b : tri_) {
                b = std::make_unique<uint8_t[]>(size_);
                std::memset(b.get(), 0, size_);
            }
            triState_.store(0b100100, std::memory_order_release); // w0 r1 s2
            break;
        case ImageMode::Rotating:
            break;   // payload lives in the channel frame — nothing owned
        default: break;
    }
    // Owned input bank for the no-channel fallback (publishInputCopy) and
    // for multi-part publishes — allocated here so the RT publish path
    // never allocates.  shm mode uses the segment's input region instead.
    if (!shm_in_) {
        in_bank_ = std::make_unique<uint8_t[]>(size_);
    }
    epoch_.fetch_add(1, std::memory_order_acq_rel);
    return true;
}

// ============================================================================
// Publish / wait
// ============================================================================

void ProcessImage::clearInputHold() {
    if (in_channel_ && in_cookie_ >= 0) {
        in_channel_->rxRelease(static_cast<uint32_t>(in_cookie_));
    }
    in_channel_ = nullptr;
    in_cookie_  = -1;
}

void ProcessImage::publishNotify() {
    in_seq_.fetch_add(1, std::memory_order_acq_rel);
    if (pub_ctr_) {
        pub_ctr_->fetch_add(1, std::memory_order_release);
        if (pub_waiters_ &&
            pub_waiters_->load(std::memory_order_acquire) > 0) {
            futexWakeAll();
        }
    }
}

void ProcessImage::futexWakeAllOn(std::atomic<uint32_t>* word) {
#ifdef __linux__
    // Shared word (shm) needs the non-private futex op so a *different*
    // process's waiters wake — FUTEX_WAKE_PRIVATE keys on the caller's mm.
    const int op = futex_shared_ ? FUTEX_WAKE : FUTEX_WAKE_PRIVATE;
    ::syscall(SYS_futex,
              reinterpret_cast<uint32_t*>(word), op,
              INT32_MAX, nullptr, nullptr, 0);
#elif defined(_WIN32)
    // WaitOnAddress works on any process-local or mapped address, so
    // futex_shared_ needs no separate op — same wake reaches waiters in
    // other processes on a mapped view.
    ::WakeByAddressAll(word);
#else
    (void)word;
#endif
}

void ProcessImage::futexWakeAll() {
    futexWakeAllOn(pub_ctr_);
}

bool ProcessImage::futexWaitOn(std::atomic<uint32_t>* word,
                               uint32_t expected, int64_t timeout_ns) {
#ifdef __linux__
    timespec ts{};
    timespec* tsp = nullptr;
    if (timeout_ns >= 0) {
        ts.tv_sec  = timeout_ns / 1'000'000'000;
        ts.tv_nsec = timeout_ns % 1'000'000'000;
        tsp = &ts;
    }
    const int op = futex_shared_ ? FUTEX_WAIT : FUTEX_WAIT_PRIVATE;
    const long r = ::syscall(SYS_futex,
                             reinterpret_cast<uint32_t*>(word), op,
                             expected, tsp, nullptr, 0);
    return r == 0;
#elif defined(_WIN32)
    // WaitOnAddress semantics match FUTEX_WAIT: it sleeps only while the
    // word still equals `expected` (re-checked under the same memory
    // ordering as the caller's load).  Timeout granularity is ms — round
    // up so a sub-ms budget still waits once.
    DWORD ms = INFINITE;
    if (timeout_ns >= 0) {
        ms = static_cast<DWORD>((timeout_ns + 999'999) / 1'000'000);
    }
    const uint32_t cmp = expected;
    return ::WaitOnAddress(word, const_cast<uint32_t*>(&cmp),
                           sizeof(uint32_t), ms) != FALSE;
#else
    (void)word; (void)expected; (void)timeout_ns;
    return false;
#endif
}

bool ProcessImage::futexWait(uint32_t expected, uint32_t timeout_ns) {
    return futexWaitOn(pub_ctr_, expected, timeout_ns);
}

// ============================================================================
// Async send trigger — wait/wake, producer accounting, consumer warn
// ============================================================================

void ProcessImage::warnCyclicTrigger() {
    bool expected = false;
    if (cyclic_trigger_warned_.compare_exchange_strong(
            expected, true, std::memory_order_acq_rel)) {
        TETHER_LOGW(TAG, "triggerSend() while a cyclic loop owns the wire "
                    "— the send-request counter bumps but nobody consumes "
                    "it; use startAsyncLoop() for send-on-change");
    }
}

void ProcessImage::triggerSend(uint32_t producer_id) {
    if (send_consumer_.load(std::memory_order_acquire) ==
        static_cast<uint8_t>(SendConsumer::Cyclic)) {
        warnCyclicTrigger();
    }
    // Stamp BEFORE the seq bump (release) — a waiter observing the new
    // seq is guaranteed to read a stamp ≥ this trigger's (Q24 latency).
    sendStampWord()->store(monoNowNs(), std::memory_order_release);
    if (producer_id < kMaxShmProducers) {
        producerTriggers()[producer_id]
            .fetch_add(1, std::memory_order_relaxed);
    }
    auto* w = sendSeqWord();
    w->fetch_add(1, std::memory_order_release);
    // Only pay the wake syscall when a waiter is registered — the
    // waiter registers before re-checking the word, so a trigger
    // landing between the check and the registration is still seen.
    if (sendWaitersWord()->load(std::memory_order_acquire) & 0x00FFFFFFu)
        sendWakeAll();
}

int ProcessImage::claimProducerSlot() {
    // Bits 1..kMaxShmProducers-1 are claimable; bit 0 is the anonymous
    // bucket and stays claimed forever.
    auto* claims = producerClaims();
    uint32_t c = claims->load(std::memory_order_acquire);
    for (;;) {
        uint32_t free_mask = ~c & 0xFEu;   // claimable bits 1..7
        if (!free_mask) return -1;
        const uint32_t bit = free_mask & (~free_mask + 1);   // lowest free
        if (claims->compare_exchange_weak(c, c | bit,
                                        std::memory_order_acq_rel,
                                        std::memory_order_acquire)) {
            int slot = 0;
            while (!(bit & (1u << slot))) ++slot;
            return slot;
        }
        // c refreshed by the failed CAS — rescan
    }
}

uint32_t ProcessImage::producerTriggerCount(uint32_t id) const {
    if (id >= kMaxShmProducers) return 0;
    return producerTriggers()[id].load(std::memory_order_acquire);
}

void ProcessImage::sendWakeAll() {
    // Raw futex wake on the word — symmetric with futexWaitOn() below.
    // NOTE: std::atomic::notify_all() is NOT usable here: libstdc++ keeps a
    // separate waiter-pool count, so a wake only reaches waiters that went
    // through atomic::wait — never a raw futex_wait() on the same address.
    // Timed waits have no atomic::wait equivalent until C++26 anyway, so
    // all send-seq blocking uses raw futex ops on the word (private for
    // in-process, shared for shm so a waitSend() in the *exporting* process
    // wakes across the boundary).
    futexWakeAllOn(sendSeqWord());
}

bool ProcessImage::waitSend(uint32_t last_seq, uint64_t timeout_ns) {
    auto* w   = sendSeqWord();
    auto* wt  = sendWaitersWord();
    if (w->load(std::memory_order_acquire) != last_seq) return true;
    // Register in the packed {epoch:8|count:24} waiters word — CAS so a
    // reclaim (attach/configure bumps the epoch, wipes the count) can
    // never orphan our decrement (Q25).
    uint32_t packed = wt->load(std::memory_order_acquire);
    uint32_t reg_word;
    for (;;) {
        if ((packed & 0x00FFFFFFu) == 0x00FFFFFFu) {
            reg_word = packed;   // count saturated — wait uncounted
            break;
        }
        if (wt->compare_exchange_weak(packed, packed + 1,
                                      std::memory_order_acq_rel,
                                      std::memory_order_acquire)) {
            reg_word = packed;
            break;
        }
    }
    uint32_t reg_epoch = reg_word >> 24;
    // Relative timeout → internal absolute deadline (CLOCK_MONOTONIC).
    // Overflow-safe: a timeout that would wrap means "effectively never".
    const uint64_t deadline =
        (timeout_ns == UINT64_MAX) ? UINT64_MAX
        : (timeout_ns >= UINT64_MAX - monoNowNs() ? UINT64_MAX
                                                  : monoNowNs() + timeout_ns);
    // shm waits cap each futex slice at 1s: a reclaimed registration is
    // re-established on the next slice so a live waiter can't be orphaned
    // by an attach-time epoch bump.
    constexpr int64_t kShmSliceNs = 1'000'000'000;
    bool got = false;
    bool counted = (reg_word & 0x00FFFFFFu) != 0x00FFFFFFu;
    for (;;) {
        const uint32_t cur = w->load(std::memory_order_acquire);
        if (cur != last_seq) { got = true; break; }
        if (futex_shared_ && counted &&
            (wt->load(std::memory_order_acquire) >> 24) != reg_epoch) {
            // Reclaimed mid-wait — re-register under the new epoch.
            counted = false;
            packed = wt->load(std::memory_order_acquire);
            for (;;) {
                if ((packed & 0x00FFFFFFu) == 0x00FFFFFFu) break;
                if (wt->compare_exchange_weak(packed, packed + 1,
                                              std::memory_order_acq_rel,
                                              std::memory_order_acquire)) {
                    counted = true;
                    reg_epoch = packed >> 24;
                    break;
                }
            }
            continue;
        }
        int64_t slice;
        if (deadline == UINT64_MAX) {
            slice = futex_shared_ ? kShmSliceNs : -1;
        } else {
            const uint64_t now = monoNowNs();
            if (now >= deadline) break;
            const uint64_t delta = deadline - now;
            slice = delta > static_cast<uint64_t>(INT64_MAX)
                        ? -1 : static_cast<int64_t>(delta);
            if (futex_shared_ &&
                (slice < 0 || slice > kShmSliceNs)) slice = kShmSliceNs;
        }
        if (slice < 0) {
#ifdef __linux__
            // Unbounded in-process or shm slice boundary: raw futex wait on
            // the word — atomic::wait would be invisible to sendWakeAll's
            // raw futex wake (libstdc++ waiter pool, see sendWakeAll).
            futexWaitOn(w, cur, -1);
#else
            std::this_thread::yield();
#endif
        } else {
#ifdef __linux__
            futexWaitOn(w, cur, slice);
#else
            std::this_thread::yield();
#endif
        }
    }
    // Deregister only when our epoch still owns the count — after a
    // reclaim the count was already zeroed, decrementing would corrupt
    // a new generation's count.
    if (counted) {
        uint32_t cur = wt->load(std::memory_order_acquire);
        while ((cur >> 24) == reg_epoch && (cur & 0x00FFFFFFu) > 0) {
            if (wt->compare_exchange_weak(cur, cur - 1,
                                          std::memory_order_acq_rel,
                                          std::memory_order_acquire))
                break;
        }
    }
    return got;
}

bool ProcessImage::waitInput(uint64_t last_seq, uint32_t timeout_ns) {
    const uint64_t deadline = monoNowNs() + timeout_ns;
    if (in_seq_.load(std::memory_order_acquire) != last_seq) return true;
    if (!pub_ctr_ || !pub_waiters_) return false;

    // Register before re-checking — a publish landing between the check
    // and the futex_wait bumps pub_ctr_, making the wait return EAGAIN.
    pub_waiters_->fetch_add(1, std::memory_order_acq_rel);
    bool got = false;
    for (;;) {
        if (in_seq_.load(std::memory_order_acquire) != last_seq) {
            got = true;
            break;
        }
        const uint64_t now = monoNowNs();
        if (now >= deadline) break;
        const uint32_t ctr0 = pub_ctr_->load(std::memory_order_acquire);
#ifdef __linux__
        futexWait(ctr0, static_cast<uint32_t>(deadline - now));
#else
        std::this_thread::yield();
#endif
    }
    pub_waiters_->fetch_sub(1, std::memory_order_acq_rel);
    return got;
}

uint8_t* ProcessImage::inputWriteBank() {
    if (shm_in_)  return shm_in_;
    return in_bank_.get();
}

void ProcessImage::commitInput() {
    uint8_t* bank = inputWriteBank();
    if (!bank) return;
    clearInputHold();
    in_ptr_.store(bank, std::memory_order_release);
    in_len_ = size_;
    publishNotify();
}

void ProcessImage::publishInputView(const uint8_t* payload, uint32_t len,
                                    uint32_t cookie, ICyclicChannel* ch) {
    // Release the previous held view, then hold the new one — the pointer
    // handed to readers stays valid until the next publish.
    clearInputHold();
    if (ch) {
        ch->rxHold(cookie);
        in_channel_ = ch;
        in_cookie_  = static_cast<int64_t>(cookie);
    }
    in_ptr_.store(payload, std::memory_order_release);
    in_len_ = len;
    publishNotify();
}

void ProcessImage::publishInputCopy(const uint8_t* payload, uint32_t len) {
    uint8_t* bank = inputWriteBank();
    if (!bank) return;   // not configured
    const uint32_t n = len < size_ ? len : size_;
    std::memcpy(bank, payload, n);
    clearInputHold();
    in_ptr_.store(bank, std::memory_order_release);
    in_len_ = n;
    publishNotify();
}

} // namespace EtherCAT

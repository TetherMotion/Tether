/**
 * @file ProcessImage_shm.cpp
 * @brief ProcessImage — POSIX shared-memory export/attach, seqlock and entry table.
 *
 * TU split out of ProcessImage.cpp.
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

// ============================================================================
// shm export / attach (Linux; stubs elsewhere)
// ============================================================================

/// FNV-1a over the fixed sizing/offset fields the handshake guards —
/// catches a half-written export before the attacher trusts any offset.
/// entry_count is deliberately excluded: exportEntryTable publishes it
/// after ready=1, so it must not invalidate the static geometry check.
uint32_t ProcessImage::headerChecksum(const ShmImageHeader& h) {
    const uint32_t fields[] = {
        h.magic, h.version, h.rx_bytes, h.tx_bytes, h.header_bytes,
        h.epoch, h.flags.load(std::memory_order_relaxed),
        h.entry_table_off,
    };
    uint32_t hash = 2166136261u;
    for (uint32_t f : fields) {
        for (int b = 0; b < 4; ++b) {
            hash ^= (f >> (b * 8)) & 0xFF;
            hash *= 16777619u;
        }
    }
    return hash;
}

bool ProcessImage::mapShm(const char* name, bool create,
                          uint32_t rx, uint32_t tx,
                          uint32_t entry_rows, bool seqlock) {
#ifdef __linux__
    char path[80];
    if (name[0] == '/') {
        std::snprintf(path, sizeof(path), "%s", name);
    } else {
        std::snprintf(path, sizeof(path), "/%s", name);
    }

    int fd;
    if (create) {
        fd = ::shm_open(path, O_RDWR | O_CREAT | O_TRUNC, 0600);
        if (fd < 0) {
            TETHER_LOGW(TAG, "shm_open('{}') create failed: {}",
                        path, strerror(errno));
            return false;
        }
        const uint32_t total =
            ShmImageLayout::totalBytes(rx, tx, entry_rows);
        if (::ftruncate(fd, total) != 0) {
            TETHER_LOGW(TAG, "ftruncate({}) failed: {}", path,
                        strerror(errno));
            ::close(fd);
            ::shm_unlink(path);
            return false;
        }
        void* map = ::mmap(nullptr, total, PROT_READ | PROT_WRITE,
                           MAP_SHARED, fd, 0);
        if (map == MAP_FAILED) {
            TETHER_LOGW(TAG, "mmap({}) failed: {}", path, strerror(errno));
            ::close(fd);
            ::shm_unlink(path);
            return false;
        }
        auto* hdr = static_cast<ShmImageHeader*>(map);
        std::memset(map, 0, total);
        // Fill every field BEFORE publishing ready — attachers validate
        // the checksum only after observing ready=1 (Q20).
        hdr->magic        = kShmMagic;
        hdr->version      = kShmVersion;
        hdr->rx_bytes     = rx;
        hdr->tx_bytes     = tx;
        hdr->header_bytes = sizeof(ShmImageHeader);
        hdr->epoch        = epoch_.load(std::memory_order_acquire) + 1;
        hdr->in_seq.store(0);  hdr->in_waiters.store(0);
        hdr->out_seq.store(0); hdr->out_waiters.store(0);
        hdr->send_seq.store(0); hdr->send_waiters.store(0);
        hdr->send_stamp_ns.store(0);
        hdr->send_epoch.store(1);          // generation 1
        hdr->rx_lock_seq.store(0); hdr->tx_lock_seq.store(0);
        hdr->producer_claims.store(1);     // slot 0 = anonymous bucket
        // flags is checksummed — must be final before ready=1.  EntryTable
        // means "table space reserved"; entry_count publishes the rows.
        const uint32_t flags =
            (seqlock ? kShmFlagSeqlock : 0u) |
            (entry_rows ? kShmFlagEntryTable : 0u);
        hdr->flags.store(flags, std::memory_order_relaxed);
        hdr->entry_table_off =
            ShmImageLayout::entryTableOff(rx, tx);
        hdr->entry_count.store(0, std::memory_order_relaxed);  // exportEntryTable fills this

        shm_map_      = map;
        shm_map_len_  = total;
        shm_fd_       = fd;
        shm_owner_    = true;
        std::strncpy(shm_name_, path, sizeof(shm_name_) - 1);
        shm_out_ = static_cast<uint8_t*>(map) + ShmImageLayout::outputOff();
        shm_in_  = static_cast<uint8_t*>(map) + ShmImageLayout::inputOff(rx);
        shm_entry_capacity_ = entry_rows;
        shm_entries_ = reinterpret_cast<ShmEntryDesc*>(
            static_cast<uint8_t*>(map) + hdr->entry_table_off);
        shm_flags_ = hdr->flags.load(std::memory_order_relaxed);
        pub_ctr_     = &hdr->in_seq;
        pub_waiters_ = &hdr->in_waiters;
        shm_out_seq_ = &hdr->out_seq;
        shm_send_seq_ = &hdr->send_seq;
        shm_send_waiters_ = &hdr->send_waiters;
        shm_send_stamp_ = &hdr->send_stamp_ns;
        shm_send_epoch_ = &hdr->send_epoch;
        shm_rx_lock_ = &hdr->rx_lock_seq;
        shm_tx_lock_ = &hdr->tx_lock_seq;
        shm_prod_triggers_ = hdr->producer_triggers;
        shm_prod_claims_ = &hdr->producer_claims;
        ext_in_seq_  = &hdr->in_seq;
        futex_shared_ = true;
        // Handshake: checksum over the fixed fields, then release-publish
        // ready LAST so no attacher can observe a half-written export.
        hdr->checksum.store(headerChecksum(*hdr),
                            std::memory_order_relaxed);
        hdr->ready.store(1, std::memory_order_release);
        shm_ready_ = true;
        TETHER_LOGI(TAG, "process image exported via shm '{}' "
                    "(rx={}B tx={}B entries={} seqlock={} total={}B)",
                    path, rx, tx, entry_rows, seqlock, total);
        return true;
    }

    // Attach path: map the header first, then validate the handshake
    // before trusting any size/offset field (Q20).
    fd = ::shm_open(path, O_RDWR, 0600);
    if (fd < 0) {
        TETHER_LOGW(TAG, "shm_open('{}') attach failed: {}",
                    path, strerror(errno));
        return false;
    }
    void* hmap = ::mmap(nullptr, ShmImageLayout::kHeaderPadded,
                        PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);
    if (hmap == MAP_FAILED) {
        ::close(fd);
        return false;
    }
    auto* hdr = static_cast<ShmImageHeader*>(hmap);
    if (hdr->magic != kShmMagic || hdr->version != kShmVersion) {
        TETHER_LOGW(TAG, "shm '{}': bad magic/version — not a Tether image",
                    path);
        ::munmap(hmap, ShmImageLayout::kHeaderPadded);
        ::close(fd);
        return false;
    }
    // Poll briefly for ready — the exporter may be mid-export.  ~1s bound
    // keeps a crashed-exporter segment from hanging the attach forever.
    bool ready = false;
    for (int i = 0; i < 1000; ++i) {
        if (hdr->ready.load(std::memory_order_acquire) == 1 &&
            hdr->checksum.load(std::memory_order_acquire) ==
                headerChecksum(*hdr)) {
            ready = true;
            break;
        }
        timespec ts{0, 1'000'000};
        ::nanosleep(&ts, nullptr);
    }
    if (!ready) {
        TETHER_LOGW(TAG, "shm '{}': export handshake incomplete — "
                    "refusing to attach", path);
        ::munmap(hmap, ShmImageLayout::kHeaderPadded);
        ::close(fd);
        return false;
    }
    // Map the *whole* segment — the entry table may follow the input
    // region; fstat gives the true exported size.
    struct stat st{};
    if (::fstat(fd, &st) != 0 ||
        st.st_size < static_cast<off_t>(hdr->entry_table_off)) {
        ::munmap(hmap, ShmImageLayout::kHeaderPadded);
        ::close(fd);
        return false;
    }
    const uint32_t total = static_cast<uint32_t>(st.st_size);
    void* map = ::mmap(nullptr, total, PROT_READ | PROT_WRITE,
                       MAP_SHARED, fd, 0);
    if (map == MAP_FAILED) {
        ::munmap(hmap, ShmImageLayout::kHeaderPadded);
        ::close(fd);
        return false;
    }
    ::munmap(hmap, ShmImageLayout::kHeaderPadded);
    hdr = static_cast<ShmImageHeader*>(map);
    shm_map_     = map;
    shm_map_len_ = total;
    shm_fd_      = fd;
    shm_owner_   = false;
    std::strncpy(shm_name_, path, sizeof(shm_name_) - 1);
    shm_out_ = static_cast<uint8_t*>(map) + ShmImageLayout::outputOff();
    shm_in_  = static_cast<uint8_t*>(map)
             + ShmImageLayout::inputOff(hdr->rx_bytes);
    shm_flags_ = hdr->flags.load(std::memory_order_acquire);
    shm_entry_capacity_ =
        (total - hdr->entry_table_off) / sizeof(ShmEntryDesc);
    shm_entry_count_ = hdr->entry_count.load(std::memory_order_acquire);
    if (shm_entry_count_ > shm_entry_capacity_)
        shm_entry_count_ = shm_entry_capacity_;   // clamp torn/poisoned
    shm_entries_ = shm_entry_capacity_
        ? reinterpret_cast<ShmEntryDesc*>(
              static_cast<uint8_t*>(map) + hdr->entry_table_off)
        : nullptr;
    shm_ready_ = true;
    pub_ctr_     = &hdr->in_seq;
    pub_waiters_ = &hdr->in_waiters;
    shm_out_seq_ = &hdr->out_seq;
    shm_send_seq_ = &hdr->send_seq;
    shm_send_waiters_ = &hdr->send_waiters;
    shm_send_stamp_ = &hdr->send_stamp_ns;
    shm_send_epoch_ = &hdr->send_epoch;
    shm_rx_lock_ = &hdr->rx_lock_seq;
    shm_tx_lock_ = &hdr->tx_lock_seq;
    shm_prod_triggers_ = hdr->producer_triggers;
    shm_prod_claims_ = &hdr->producer_claims;
    ext_in_seq_  = &hdr->in_seq;
    futex_shared_ = true;
    // Reclaim (Q25): CAS the packed {epoch|count} waiters word to a new
    // generation with count 0 — orphans a count left by a crashed waiter;
    // live waiters re-register on their next ≤1s slice.  send_epoch
    // tracks the reclaim generation for diagnostics.
    shm_send_epoch_->fetch_add(1, std::memory_order_acq_rel);
    {
        uint32_t pw = shm_send_waiters_->load(std::memory_order_acquire);
        while (!shm_send_waiters_->compare_exchange_weak(
                   pw, ((pw >> 24) + 1u) << 24,
                   std::memory_order_acq_rel, std::memory_order_acquire)) {}
    }
    return true;
#else
    (void)name; (void)create; (void)rx; (void)tx;
    (void)entry_rows; (void)seqlock;
    TETHER_LOGW(TAG, "shm process images are Linux-only on this build");
    return false;
#endif
}

void ProcessImage::releaseShm() {
#ifdef __linux__
    if (shm_map_ && shm_owner_) {
        // Withdraw the handshake before unmapping so a racing attacher
        // can't validate a segment that's about to disappear (Q20).
        auto* hdr = static_cast<ShmImageHeader*>(shm_map_);
        hdr->ready.store(0, std::memory_order_release);
    }
    if (shm_map_) {
        ::munmap(shm_map_, shm_map_len_);
        shm_map_ = nullptr;
    }
    if (shm_fd_ >= 0) {
        ::close(shm_fd_);
        shm_fd_ = -1;
    }
    if (shm_owner_ && shm_name_[0] != '\0') {
        ::shm_unlink(shm_name_);
    }
#else
    shm_map_ = nullptr;
    shm_fd_  = -1;
#endif
    shm_map_len_  = 0;
    shm_owner_    = false;
    shm_attached_ = false;
    shm_ready_    = false;
    shm_flags_    = 0;
    shm_name_[0]  = '\0';
    shm_out_ = nullptr;
    shm_in_  = nullptr;
    shm_out_seq_ = nullptr;
    shm_send_seq_ = nullptr;
    shm_send_waiters_ = nullptr;
    shm_send_stamp_ = nullptr;
    shm_send_epoch_ = nullptr;
    shm_rx_lock_ = nullptr;
    shm_tx_lock_ = nullptr;
    shm_prod_triggers_ = nullptr;
    shm_prod_claims_ = nullptr;
    shm_entries_ = nullptr;
    shm_entry_count_ = 0;
    shm_entry_capacity_ = 0;
    ext_in_seq_  = nullptr;
    pub_ctr_     = &in_pub_ctr_;
    pub_waiters_ = &in_waiters_;
    futex_shared_ = false;
}

bool ProcessImage::attachShared(const char* name) {
    configure({});   // reset everything
    if (!mapShm(name, /*create=*/false, 0, 0, 0, false)) return false;
    auto* hdr = static_cast<ShmImageHeader*>(shm_map_);
    rx_bytes_ = hdr->rx_bytes;
    tx_bytes_ = hdr->tx_bytes;
    size_     = rx_bytes_ + tx_bytes_;
    mode_     = ImageMode::Direct;
    shm_attached_ = true;
    // Populate entry offsets from the exported table when present (Q4) —
    // attachers get entryHandle()/typed access instead of raw offsets.
    for (size_t i = 0; i < kMaxEntries; ++i) entry_off_[i] = -1;
    entry_count_ = 0;
    for (uint32_t i = 0; i < shm_entry_count_; ++i) {
        const ShmEntryDesc& d = shm_entries_[i];
        if (d.index < kMaxEntries) {
            entry_off_[d.index] = d.offset;
            if (d.index + 1 > entry_count_) entry_count_ = d.index + 1;
        }
    }
    in_ptr_.store(shm_in_, std::memory_order_release);
    in_len_ = size_;
    epoch_.fetch_add(1, std::memory_order_acq_rel);
    TETHER_LOGI(TAG, "attached to shared process image '{}' "
                "(rx={}B tx={}B entries={} flags=0x{:x})",
                shm_name_, rx_bytes_, tx_bytes_, shm_entry_count_,
                shm_flags_);
    return true;
}

// ============================================================================
// shm seqlock + entry table (Q4/Q5)
// ============================================================================

std::atomic<uint32_t>* ProcessImage::lockSeq(ShmRegion r) {
    if (!shm_map_ || !(shm_flags_ & kShmFlagSeqlock)) return nullptr;
    return r == ShmRegion::Output ? shm_rx_lock_ : shm_tx_lock_;
}
const std::atomic<uint32_t>* ProcessImage::lockSeq(ShmRegion r) const {
    if (!shm_map_ || !(shm_flags_ & kShmFlagSeqlock)) return nullptr;
    return r == ShmRegion::Output ? shm_rx_lock_ : shm_tx_lock_;
}

void ProcessImage::shmWriteBegin(ShmRegion r) {
    auto* s = lockSeq(r);
    if (s) s->fetch_add(1, std::memory_order_acq_rel);   // → odd
}

void ProcessImage::shmWriteEnd(ShmRegion r) {
    auto* s = lockSeq(r);
    if (s) s->fetch_add(1, std::memory_order_acq_rel);   // → even
}

uint32_t ProcessImage::shmReadBegin(ShmRegion r) const {
    const auto* s = lockSeq(r);
    if (!s) return 0;
    // Spin while a writer is mid-burst (odd) — bounded: a crashed writer
    // leaves it odd forever; return the odd stamp so shmReadEnd fails
    // and the caller retries rather than hanging here.
    uint32_t v = s->load(std::memory_order_acquire);
    for (int i = 0; (v & 1u) && i < 100'000; ++i)
        v = s->load(std::memory_order_acquire);
    return v;
}

bool ProcessImage::shmReadEnd(ShmRegion r, uint32_t stamp) const {
    const auto* s = lockSeq(r);
    if (!s) return true;
    std::atomic_thread_fence(std::memory_order_acquire);
    return s->load(std::memory_order_acquire) == stamp;
}

void ProcessImage::exportEntryTable(const ShmEntryDesc* rows,
                                    uint32_t count) {
    if (!shm_map_ || !shm_entries_ || !shm_owner_) return;
    const uint32_t n = count < shm_entry_capacity_ ? count
                                                   : shm_entry_capacity_;
    if (rows && n) std::memcpy(shm_entries_, rows, n * sizeof(*rows));
    auto* hdr = static_cast<ShmImageHeader*>(shm_map_);
    // Publish count last with release — attachers reading entry_count
    // with acquire see a fully-written table.  The EntryTable flag was
    // already set at export (it marks reserved capacity) — mutating it
    // here would invalidate the handshake checksum.
    std::atomic_thread_fence(std::memory_order_release);
    hdr->entry_count.store(n, std::memory_order_release);
    shm_entry_count_ = n;
}

} // namespace EtherCAT

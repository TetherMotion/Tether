/**
 * @file ProcessImage.cpp
 * @brief Process image configuration, input publish, futex wait, shm
 *        export/attach implementation.
 */

#include "tether/ethercat/ProcessImage.hpp"
#include "tether/ethercat/CyclicChannel.hpp"
#include "logging/Logger.hpp"

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
#else
#include <thread>
#endif

namespace EtherCAT {

static const char* TAG = "process_image";

static uint64_t monoNowNs() {
    timespec ts{};
    ::clock_gettime(CLOCK_MONOTONIC, &ts);
    return static_cast<uint64_t>(ts.tv_sec) * 1'000'000'000ull
         + static_cast<uint64_t>(ts.tv_nsec);
}

// ============================================================================
// Configuration
// ============================================================================

bool ProcessImage::configure(const Config& cfg) {
    releaseShm();
    rx_bytes_ = cfg.rx_bytes;
    tx_bytes_ = cfg.tx_bytes;
    size_     = cfg.rx_bytes + cfg.tx_bytes;
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
    shm_attached_ = false;
    in_pub_ctr_.store(0, std::memory_order_release);
    in_waiters_.store(0, std::memory_order_release);

    if (size_ == 0 || mode_ == ImageMode::Buffered) {
        mode_ = ImageMode::Buffered;
        epoch_.fetch_add(1, std::memory_order_acq_rel);
        return size_ > 0;
    }

    // shm export forces Direct semantics — the regions are the segment.
    if (cfg.shm_name && cfg.shm_name[0] != '\0') {
        if (!mapShm(cfg.shm_name, /*create=*/true, rx_bytes_, tx_bytes_)) {
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
// shm export / attach (Linux; stubs elsewhere)
// ============================================================================

bool ProcessImage::mapShm(const char* name, bool create,
                          uint32_t rx, uint32_t tx) {
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
        const uint32_t total = ShmImageLayout::totalBytes(rx, tx);
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
        hdr->magic        = kShmMagic;
        hdr->version      = kShmVersion;
        hdr->rx_bytes     = rx;
        hdr->tx_bytes     = tx;
        hdr->header_bytes = sizeof(ShmImageHeader);
        hdr->epoch        = epoch_.load(std::memory_order_acquire) + 1;
        hdr->in_seq.store(0);  hdr->in_waiters.store(0);
        hdr->out_seq.store(0); hdr->out_waiters.store(0);

        shm_map_      = map;
        shm_map_len_  = total;
        shm_fd_       = fd;
        shm_owner_    = true;
        std::strncpy(shm_name_, path, sizeof(shm_name_) - 1);
        shm_out_ = static_cast<uint8_t*>(map) + ShmImageLayout::outputOff();
        shm_in_  = static_cast<uint8_t*>(map) + ShmImageLayout::inputOff(rx);
        pub_ctr_     = &hdr->in_seq;
        pub_waiters_ = &hdr->in_waiters;
        shm_out_seq_ = &hdr->out_seq;
        ext_in_seq_  = &hdr->in_seq;
        futex_shared_ = true;
        TETHER_LOGI(TAG, "process image exported via shm '{}' "
                    "(rx={}B tx={}B total={}B)", path, rx, tx, total);
        return true;
    }

    // Attach path: map the header first to learn the sizes.
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
    const uint32_t total = ShmImageLayout::totalBytes(hdr->rx_bytes,
                                                    hdr->tx_bytes);
    void* map = ::mmap(nullptr, total, PROT_READ | PROT_WRITE,
                       MAP_SHARED, fd, 0);
    ::munmap(hmap, ShmImageLayout::kHeaderPadded);
    if (map == MAP_FAILED) {
        ::close(fd);
        return false;
    }
    hdr = static_cast<ShmImageHeader*>(map);
    shm_map_     = map;
    shm_map_len_ = total;
    shm_fd_      = fd;
    shm_owner_   = false;
    std::strncpy(shm_name_, path, sizeof(shm_name_) - 1);
    shm_out_ = static_cast<uint8_t*>(map) + ShmImageLayout::outputOff();
    shm_in_  = static_cast<uint8_t*>(map)
             + ShmImageLayout::inputOff(hdr->rx_bytes);
    pub_ctr_     = &hdr->in_seq;
    pub_waiters_ = &hdr->in_waiters;
    shm_out_seq_ = &hdr->out_seq;
    ext_in_seq_  = &hdr->in_seq;
    futex_shared_ = true;
    return true;
#else
    (void)name; (void)create; (void)rx; (void)tx;
    TETHER_LOGW(TAG, "shm process images are Linux-only on this build");
    return false;
#endif
}

void ProcessImage::releaseShm() {
#ifdef __linux__
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
    shm_name_[0]  = '\0';
    shm_out_ = nullptr;
    shm_in_  = nullptr;
    shm_out_seq_ = nullptr;
    ext_in_seq_  = nullptr;
    pub_ctr_     = &in_pub_ctr_;
    pub_waiters_ = &in_waiters_;
    futex_shared_ = false;
}

bool ProcessImage::attachShared(const char* name) {
    configure({});   // reset everything
    if (!mapShm(name, /*create=*/false, 0, 0)) return false;
    auto* hdr = static_cast<ShmImageHeader*>(shm_map_);
    rx_bytes_ = hdr->rx_bytes;
    tx_bytes_ = hdr->tx_bytes;
    size_     = rx_bytes_ + tx_bytes_;
    mode_     = ImageMode::Direct;
    shm_attached_ = true;
    entry_count_ = 0;   // no mapping info on the client — raw offsets only
    in_ptr_.store(shm_in_, std::memory_order_release);
    in_len_ = size_;
    epoch_.fetch_add(1, std::memory_order_acq_rel);
    TETHER_LOGI(TAG, "attached to shared process image '{}' "
                "(rx={}B tx={}B)", shm_name_, rx_bytes_, tx_bytes_);
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

void ProcessImage::futexWakeAll() {
#ifdef __linux__
    const int op = futex_shared_ ? FUTEX_WAKE : FUTEX_WAKE_PRIVATE;
    ::syscall(SYS_futex,
              reinterpret_cast<uint32_t*>(pub_ctr_), op,
              INT32_MAX, nullptr, nullptr, 0);
#endif
}

bool ProcessImage::futexWait(uint32_t expected, uint32_t timeout_ns) {
#ifdef __linux__
    timespec ts{};
    ts.tv_sec  = timeout_ns / 1'000'000'000u;
    ts.tv_nsec = timeout_ns % 1'000'000'000u;
    const int op = futex_shared_ ? FUTEX_WAIT : FUTEX_WAIT_PRIVATE;
    const long r = ::syscall(SYS_futex,
                             reinterpret_cast<uint32_t*>(pub_ctr_), op,
                             expected, &ts, nullptr, 0);
    return r == 0;
#else
    (void)expected; (void)timeout_ns;
    return false;
#endif
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

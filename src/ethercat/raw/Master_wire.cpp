/**
 * @file Master_wire.cpp
 * @brief Master — RX frame handling and wire drain.
 *
 * TU split out of Master_slave.cpp.
 */

#include "tether/ethercat/Master.hpp"
#include "tether/ethercat/Slave.hpp"
#include "tether/ethercat/PDOManager.hpp"
#include "tether/ethercat/LogicalAddressManager.hpp"
#include "tether/ethercat/CoEManager.hpp"
#include "tether/ethercat/DebugFlags.hpp"
#include "tether/sii/SIIParser.hpp"
#include "tether/fmmu/FMMUConfiguration.hpp"
#include "raw/internal.hpp"
#include "raw/SlaveRegistry.hpp"
#include "tether/platform/Platform.hpp"
#include <cstring>
#include <format>
#ifdef __linux__
#include <sys/socket.h>
#include <unistd.h>
#include <cerrno>
#endif

namespace EtherCAT {

static const char* TAG = "ethercat";

// ============================================================================
// Frame handling
// ============================================================================

void Master::handleRxFrame(const uint8_t* frame, size_t length)
{
    parseEtherCATFrame(frame, length);
}

int Master::drainWire(int max_frames)
{
    if (!iface_.receive || max_frames <= 0) return 0;
#ifdef __linux__
    // Fast path: the native handle is the wire socket fd — dequeue a
    // batch per recvmmsg() call instead of a recv per frame.  A stall
    // backlog of hundreds of frames otherwise costs a syscall each on
    // the cyclic thread's deadline budget.
    if (iface_.native_handle) {
        const int fd = static_cast<int>(
            reinterpret_cast<intptr_t>(iface_.native_handle));
        const int n = drainWireBatch(fd, max_frames);
        if (n >= 0) return n;
        // Not a drainable socket — fall through to the per-frame path.
    }
#endif
    int drained = 0;
    uint8_t buf[kMaxJumboFrameSize];
    while (drained < max_frames) {
        size_t n = 0;
        if (!iface_.receive(buf, sizeof(buf), &n) || n == 0) break;
        handleRxFrame(buf, n);
        ++drained;
    }
    return drained;
}

#ifdef __linux__
int Master::drainWireBatch(int fd, int max_frames)
{
    constexpr int kBatch = 32;
    if (drain_buf_.size() <
        static_cast<size_t>(kBatch) * kMaxJumboFrameSize) {
        try {
            drain_buf_.resize(static_cast<size_t>(kBatch) *
                              kMaxJumboFrameSize);
        } catch (...) {
            return -1;   // allocation failed — per-frame path still works
        }
    }
    int drained = 0;
    while (drained < max_frames) {
        const int want = std::min(kBatch, max_frames - drained);
        mmsghdr msgs[kBatch]{};
        iovec   iov[kBatch]{};
        for (int i = 0; i < want; ++i) {
            iov[i].iov_base = drain_buf_.data() +
                              static_cast<size_t>(i) * kMaxJumboFrameSize;
            iov[i].iov_len  = kMaxJumboFrameSize;
            msgs[i].msg_hdr.msg_iov    = &iov[i];
            msgs[i].msg_hdr.msg_iovlen = 1;
        }
        const int n = ::recvmmsg(fd, msgs, want, MSG_DONTWAIT, nullptr);
        if (n < 0) {
            if (errno == EAGAIN || errno == EWOULDBLOCK ||
                errno == EINTR) {
                break;   // queue empty (or interrupted) — done
            }
            return drained > 0 ? drained : -1;   // unusable fd → fallback
        }
        for (int i = 0; i < n; ++i) {
            handleRxFrame(static_cast<const uint8_t*>(iov[i].iov_base),
                          msgs[i].msg_len);
        }
        drained += n;
        if (n < want) break;   // queue emptied mid-batch
    }
    return drained;
}
#endif

void Master::purgePendingResponses()
{
    packet_router_.purgeAllPending();
}

} // namespace EtherCAT


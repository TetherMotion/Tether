/**
 * @file CyclicChannel_linux.cpp
 * @brief Linux cyclic-channel factories (AF_PACKET socket / PACKET_MMAP ring)
 *        plus the test/embedding seams.
 *
 * The channel class bodies live in LinuxSocketChannel.hpp and
 * LinuxRingChannel.hpp; the shared helpers and BPF program builder live in
 * CyclicChannelSupport.{hpp,cpp}.
 */

#include "tether/ethercat/CyclicChannel.hpp"
#include "tether/ethercat/Types.hpp"

#if defined(__linux__)

#include "raw/CyclicChannelSupport.hpp"
#include "raw/LinuxSocketChannel.hpp"
#include "raw/LinuxRingChannel.hpp"
#include "tether/ethercat/CBPFProgramFactory.hpp"
#include "logging/Logger.hpp"

#include <cerrno>
#include <cstring>
#include <memory>

namespace EtherCAT {

// TAG is provided by raw/LinuxSocketChannel.hpp (included above).

// ============================================================================
// Factory
// ============================================================================

std::unique_ptr<ICyclicChannel> createCyclicChannel(
    const CyclicChannelConfig& cfg)
{
    if (cfg.ifindex <= 0) {
        TETHER_LOGE(TAG, "createCyclicChannel: invalid ifindex {}", cfg.ifindex);
        return nullptr;
    }

    // Mirror filter on the async socket — optional and independent of the
    // backend we end up running.  A caller-composed program (encap ∧
    // idx∉fastpath) replaces the built-in mirror so the encapsulation
    // clause survives — SO_ATTACH_FILTER swaps the whole program.
    if (cfg.async_fd >= 0) {
        bool attached;
        if (cfg.async_prog && cfg.async_prog_len) {
            attached = CBPFProgramFactory::attach(
                cfg.async_fd, cfg.async_prog, cfg.async_prog_len);
        } else {
            attached = cyclicChannelAttachAsyncFilter(cfg.async_fd);
        }
        if (attached) {
            TETHER_LOGI(TAG, "async socket: cyclic-idx exclusion BPF attached");
        } else {
            TETHER_LOGW(TAG,
                "async socket: exclusion BPF attach failed ({}) — async "
                "traffic may still see cyclic frames (correctness unaffected)",
                strerror(errno));
        }
    }

    if (cfg.wire_mode != CyclicWireMode::SocketIO) {
        const int fd = openCyclicSocket(cfg.ifindex, cfg.accept_prog,
                                        cfg.accept_prog_len);
        if (fd >= 0) {
            LinuxRingChannel::Config rcfg{cfg.rx_ring_blocks,
                                          cfg.tx_ring_blocks,
                                          cfg.rx_spin_ns,
                                          cfg.rx_tpacket_v3,
                                          cfg.rx_v3_retire_us,
                                          cfg.frame_size};
            auto ring = std::make_unique<LinuxRingChannel>(fd, cfg.ifindex,
                                                           rcfg);
            if (ring->init()) {
                TETHER_LOGI(TAG,
                    "cyclic channel: PACKET_MMAP rings active "
                    "(rx x{} blocks{}, tx x{} blocks, ifindex {})",
                    cfg.rx_ring_blocks,
                    cfg.rx_tpacket_v3 ? " [TPACKET_V3]" : "",
                    cfg.tx_ring_blocks, cfg.ifindex);
                return ring;
            }
            // ring dtor closes fd
        }
        if (cfg.wire_mode == CyclicWireMode::PacketRing) {
            TETHER_LOGE(TAG, "cyclic channel: PACKET_RING requested but "
                        "setup failed ({})", strerror(errno));
            return nullptr;
        }
        TETHER_LOGW(TAG,
            "cyclic channel: ring setup failed ({}) — "
            "falling back to socket mode", strerror(errno));
    }

    const int fd = openCyclicSocket(cfg.ifindex, cfg.accept_prog,
                                    cfg.accept_prog_len);
    if (fd < 0) {
        TETHER_LOGW(TAG, "createCyclicChannel: cannot open cyclic socket "
                    "on ifindex {} ({})", cfg.ifindex, strerror(errno));
        return nullptr;
    }
    TETHER_LOGI(TAG, "cyclic channel: socket mode (ifindex {})", cfg.ifindex);
    return std::make_unique<LinuxSocketChannel>(fd, cfg.ifindex);
}

// ---- Test/embedding seams -------------------------------------------------

std::unique_ptr<ICyclicChannel> createCyclicSocketChannelForFd(
    int fd, int ifindex)
{
    if (fd < 0) return nullptr;
    return std::make_unique<LinuxSocketChannel>(fd, ifindex);
}

std::unique_ptr<ICyclicChannel> createCyclicRingChannelForFd(
    int fd, int ifindex, uint32_t rx_ring_blocks, uint32_t tx_ring_blocks,
    uint32_t rx_spin_ns)
{
    if (fd < 0) return nullptr;
    LinuxRingChannel::Config rcfg{rx_ring_blocks, tx_ring_blocks, rx_spin_ns};
    auto ring = std::make_unique<LinuxRingChannel>(fd, ifindex, rcfg);
    if (!ring->init()) return nullptr;   // dtor closes fd + unmaps
    return ring;
}

std::unique_ptr<ICyclicChannel> createCyclicRingChannelForMemory(
    int fd, int ifindex,
    void* rx_ring, uint32_t rx_frame_size, uint32_t rx_frames,
    void* tx_ring, uint32_t tx_frame_size, uint32_t tx_frames,
    uint32_t rx_spin_ns,
    uint32_t rx_block_size, uint32_t rx_fpb,
    uint32_t tx_block_size, uint32_t tx_fpb)
{
    if (fd < 0 || !rx_ring || rx_frame_size == 0 || rx_frames == 0)
        return nullptr;
    LinuxRingChannel::Config rcfg{0, 0, rx_spin_ns};
    auto ring = std::make_unique<LinuxRingChannel>(fd, ifindex, rcfg);
    ring->adoptRingsForTest(static_cast<uint8_t*>(rx_ring), rx_frame_size,
                            rx_frames,
                            static_cast<uint8_t*>(tx_ring), tx_frame_size,
                            tx_frames, rx_block_size, rx_fpb,
                            tx_block_size, tx_fpb);
    return ring;
}

std::unique_ptr<ICyclicChannel> createCyclicRingChannelV3ForMemory(
    int fd, int ifindex,
    void* rx_ring, uint32_t block_size, uint32_t blocks)
{
    if (fd < 0 || !rx_ring || block_size == 0 || blocks == 0)
        return nullptr;
    LinuxRingChannel::Config rcfg{};
    rcfg.rx_v3 = true;
    auto ring = std::make_unique<LinuxRingChannel>(fd, ifindex, rcfg);
    ring->adoptV3RingForTest(static_cast<uint8_t*>(rx_ring), block_size,
                             blocks);
    return ring;
}

} // namespace EtherCAT

#else  // !__linux__

namespace EtherCAT {

bool cyclicChannelAttachCyclicFilter(int) { return false; }
bool cyclicChannelAttachAsyncFilter(int)  { return false; }

size_t cyclicChannelBpfProgram(bool, CyclicBpfInsn*, size_t) { return 0; }

std::unique_ptr<ICyclicChannel> createCyclicChannel(
    const CyclicChannelConfig& cfg)
{
    (void)cfg;
    return nullptr;   // platform channels (ESP32, Windows) are separate impls
}

// Unsupported platform: channels unavailable — fd ownership stays with the
// caller (nullptr is not a channel, nothing was taken).
std::unique_ptr<ICyclicChannel> createCyclicSocketChannelForFd(
    int, int) { return nullptr; }

std::unique_ptr<ICyclicChannel> createCyclicRingChannelForFd(
    int, int, uint32_t, uint32_t, uint32_t) { return nullptr; }

std::unique_ptr<ICyclicChannel> createCyclicRingChannelV3ForMemory(
    int, int, void*, uint32_t, uint32_t) { return nullptr; }

std::unique_ptr<ICyclicChannel> createCyclicRingChannelForMemory(
    int, int, void*, uint32_t, uint32_t, void*, uint32_t, uint32_t,
    uint32_t, uint32_t, uint32_t, uint32_t, uint32_t) { return nullptr; }

} // namespace EtherCAT

#endif // __linux__

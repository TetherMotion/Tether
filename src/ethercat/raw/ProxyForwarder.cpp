/**
 * @file ProxyForwarder.cpp
 * @brief Linux AF_PACKET forwarder for the external-source PDO path.
 */

#include "raw/ProxyForwarder.hpp"
#include "raw/WireBpfSpec.hpp"
#include "tether/ethercat/CBPFProgramFactory.hpp"
#include "tether/ethercat/EtherCATProtocolConstants.hpp"
#include "tether/hal/IEthernet.hpp"
#include "logging/Logger.hpp"

#include <cerrno>
#include <cstring>
#include <format>

#if defined(__linux__)
#include <arpa/inet.h>
#include <fcntl.h>
#include <linux/if_ether.h>
#include <linux/if_packet.h>
#include <net/if.h>
#include <netinet/in.h>
#include <poll.h>
#include <sys/eventfd.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>

#ifndef SOL_PACKET
#define SOL_PACKET 263
#endif
#ifndef PACKET_IGNORE_OUTGOING
#define PACKET_IGNORE_OUTGOING 23
#endif
#endif // __linux__

namespace EtherCAT {

static const char* TAG = "ecat_proxy";

namespace {

// Wire offsets — same layout the cyclic-channel demux uses.
constexpr size_t   kEthTypeOff     = 12;
constexpr size_t   kFirstIdxOff    = 17;  // untagged first-dg idx
constexpr size_t   kVlanInnerOff   = 16;  // inner EtherType
constexpr size_t   kVlanFirstIdxOff = 21; // tagged first-dg idx
constexpr size_t   kDgHeaderLen    = 10;  // cmd|idx|adp|ado|lenFlags|irq
constexpr size_t   kWkcLen         = 2;
constexpr uint16_t kLenMask        = 0x07FF;
constexpr uint16_t kMoreBit        = 0x8000;
constexpr size_t   kRxBufSize      = 16384; // covers jumbo + tag

// PDO traffic only — the three logical commands.
constexpr bool isPdoCommand(uint8_t cmd) {
    return cmd == static_cast<uint8_t>(Command::LRD) ||
           cmd == static_cast<uint8_t>(Command::LWR) ||
           cmd == static_cast<uint8_t>(Command::LRW);
}

/// First datagram index for logging — -1 when the frame is too short.
[[maybe_unused]] int firstIdxOf(const uint8_t* f, size_t len) {
    if (len <= kFirstIdxOff) return -1;
    const uint16_t et = static_cast<uint16_t>(
        (f[kEthTypeOff] << 8) | f[kEthTypeOff + 1]);
    if (et == kEtherType8021Q)
        return len > kVlanFirstIdxOff ? f[kVlanFirstIdxOff] : -1;
    return f[kFirstIdxOff];
}

} // namespace

// ============================================================================
// Validation (platform-independent — the BPF fallback)
// ============================================================================

bool ProxyForwarder::frameIsInBandPdo(const uint8_t* f, size_t len,
                                      uint8_t idx_lo, uint8_t idx_hi)
{
    // [eth 14][+802.1Q 4][EtherCAT frame header 2][datagram chain]
    if (!f || len < 14 + 2 + kDgHeaderLen) return false;
    const uint16_t et = static_cast<uint16_t>(
        (f[kEthTypeOff] << 8) | f[kEthTypeOff + 1]);
    size_t base = 14;
    if (et == kEtherType8021Q) {
        if (len < 18 + 2 + kDgHeaderLen) return false;
        const uint16_t inner = static_cast<uint16_t>(
            (f[kVlanInnerOff] << 8) | f[kVlanInnerOff + 1]);
        if (inner != kEtherTypeEtherCAT) return false;
        base = 18;
    } else if (et != kEtherTypeEtherCAT) {
        return false;
    }
    const uint16_t ec_len = static_cast<uint16_t>(
        (f[base] | (f[base + 1] << 8)) & kLenMask);
    const size_t dg_end = base + 2 + ec_len;
    if (ec_len < kDgHeaderLen + kWkcLen || dg_end > len) return false;

    // Walk the datagram chain — every datagram must be a PDO command
    // carrying an in-band index, and the chain must end exactly at
    // dg_end (M bit clear on the last datagram).
    for (size_t off = base + 2;;) {
        if (off + kDgHeaderLen + kWkcLen > dg_end) return false;
        if (!isPdoCommand(f[off])) return false;
        const uint8_t idx = f[off + 1];
        if (idx < idx_lo || idx > idx_hi) return false;
        const uint16_t lf = static_cast<uint16_t>(
            f[off + 6] | (f[off + 7] << 8));
        const size_t dsz = kDgHeaderLen + (lf & kLenMask) + kWkcLen;
        if (off + dsz > dg_end) return false;
        off += dsz;
        if (!(lf & kMoreBit)) return off == dg_end;
    }
}

/**
 * Second pass over a frame that already passed frameIsInBandPdo —
 * applies the address/value whitelist per datagram.  The 32-bit
 * logical address sits at dg+2 (ADP | ADO<<16) for every logical
 * command — the only commands that reach this point.
 */
bool ProxyForwarder::chainPassesFilter(const uint8_t* f, size_t len,
                                       const ProxyPdoFilter& filter)
{
    const uint16_t et = static_cast<uint16_t>(
        (f[kEthTypeOff] << 8) | f[kEthTypeOff + 1]);
    const size_t base = (et == kEtherType8021Q) ? 18 : 14;
    const uint16_t ec_len = static_cast<uint16_t>(
        (f[base] | (f[base + 1] << 8)) & kLenMask);
    const size_t dg_end = base + 2 + ec_len;
    (void)dg_end;   // bounds already proven by frameIsInBandPdo

    for (size_t off = base + 2;;) {
        const uint8_t cmd = f[off];
        const uint16_t lf = static_cast<uint16_t>(
            f[off + 6] | (f[off + 7] << 8));
        const uint16_t dlen = lf & kLenMask;
        const size_t dsz = kDgHeaderLen + dlen + kWkcLen;
        const uint32_t adr = static_cast<uint32_t>(
            f[off + 2] | (f[off + 3] << 8) |
            (static_cast<uint32_t>(f[off + 4]) << 16) |
            (static_cast<uint32_t>(f[off + 5]) << 24));
        const Command c = static_cast<Command>(cmd);
        const bool need_read  = (c == Command::LRD || c == Command::LRW);
        const bool need_write = (c == Command::LWR || c == Command::LRW);
        if (!filter.allows(need_read, need_write, adr,
                           f + off + kDgHeaderLen, dlen))
            return false;
        if (!(lf & kMoreBit)) return true;
        off += dsz;
    }
}

bool ProxyForwarder::framePassesFilter(const uint8_t* f, size_t len,
                                       uint8_t idx_lo, uint8_t idx_hi,
                                       const ProxyPdoFilter* filter)
{
    if (!frameIsInBandPdo(f, len, idx_lo, idx_hi)) return false;
    return !filter || chainPassesFilter(f, len, *filter);
}

// ============================================================================
// Linux implementation
// ============================================================================

#if defined(__linux__)

ProxyForwardStats ProxyForwarder::stats() const
{
    ProxyForwardStats s;
    s.rx_forwarded = rx_forwarded_.load(std::memory_order_relaxed);
    s.tx_forwarded = tx_forwarded_.load(std::memory_order_relaxed);
    s.tx_rejected  = tx_rejected_.load(std::memory_order_relaxed);
    s.filtered     = filtered_.load(std::memory_order_relaxed);
    s.out_of_band  = out_of_band_.load(std::memory_order_relaxed);
    s.send_errors  = send_errors_.load(std::memory_order_relaxed);
    return s;
}

int ProxyForwarder::openFilteredSocket(int ifindex, const char* diag_name)
{
    const int fd = ::socket(AF_PACKET, SOCK_RAW | SOCK_CLOEXEC,
                            htons(ETH_P_ALL));
    if (fd < 0) return -1;

    int flags = fcntl(fd, F_GETFL, 0);
    fcntl(fd, F_SETFL, flags | O_NONBLOCK);

    int one = 1;
    // Drop TX echoes upstream of the BPF: without this every forwarded
    // frame would bounce back into the pump and loop forever.
    if (setsockopt(fd, SOL_PACKET, PACKET_IGNORE_OUTGOING,
                   &one, sizeof(one)) < 0) {
        TETHER_LOGW(TAG, "{}: PACKET_IGNORE_OUTGOING unsupported ({}) — "
                    "own forwards will echo once", diag_name,
                    strerror(errno));
    }
    // Bigger kernel RX queue — forwarding must not drop under bursts.
    int sz = 8 * 1024 * 1024;
    if (setsockopt(fd, SOL_SOCKET, SO_RCVBUFFORCE, &sz, sizeof(sz)) < 0)
        setsockopt(fd, SOL_SOCKET, SO_RCVBUF, &sz, sizeof(sz));

    sockaddr_ll sll{};
    sll.sll_family   = AF_PACKET;
    sll.sll_protocol = htons(ETH_P_ALL);
    sll.sll_ifindex  = ifindex;
    if (::bind(fd, reinterpret_cast<sockaddr*>(&sll), sizeof(sll)) < 0) {
        ::close(fd);
        return -1;
    }
    HAL::registerDiagPacketSocket(fd, diag_name);
    return fd;
}

bool ProxyForwarder::start(const ProxyForwardConfig& cfg, int ecat_ifindex,
                           const EtherCAT::WireEncap& encap,
                           const ProxyPdoFilter* filter)
{
    if (running()) {
        last_error_ = "already running";
        return false;
    }
    band_lo_      = cfg.idx_base;
    band_hi_      = cfg.end();
    ecat_ifindex_ = ecat_ifindex;
    filter_       = filter ? std::optional<ProxyPdoFilter>(*filter)
                           : std::nullopt;

    const unsigned pif = ::if_nametoindex(cfg.ifname.c_str());
    if (pif == 0) {
        last_error_ = std::format("proxy interface '{}' not found",
                                  cfg.ifname);
        return false;
    }
    proxy_ifindex_ = static_cast<int>(pif);
    if (proxy_ifindex_ == ecat_ifindex_) {
        last_error_ = std::format("proxy interface '{}' IS the EtherCAT "
                                  "bus interface — pick a separate NIC",
                                  cfg.ifname);
        return false;
    }

    // ---- Kernel demux programs ------------------------------------------
    // ecat side: encap clause ∧ first-idx ∈ band — the slave responses.
    CBPFSpec es = wireEncapBpfSpec(encap);
    es.first_idx_range = CBPFIdxRange{band_lo_, band_hi_};
    const auto ecat_prog = CBPFProgramFactory::build(es);

    // proxy side, in-band: untagged/any-VID EtherCAT ∧ idx ∈ band.
    CBPFSpec ps;
    ps.first_idx_range = CBPFIdxRange{band_lo_, band_hi_};
    const auto proxy_prog = CBPFProgramFactory::build(ps);

    // proxy side, monitor: EtherCAT ∧ idx ∉ band — counted + logged.
    ps.first_idx_exclude = true;
    const auto mon_prog = CBPFProgramFactory::build(ps);

    if (ecat_prog.empty() || proxy_prog.empty() || mon_prog.empty()) {
        last_error_ = "empty BPF program (spec bug)";
        return false;
    }

    // ---- Sockets ---------------------------------------------------------
    // Every attach is a hard requirement here — the forwarder must never
    // see (and leak) unfiltered traffic in either direction.
    ecat_fd_ = openFilteredSocket(ecat_ifindex_, "pxy-ecat");
    if (ecat_fd_ < 0) {
        last_error_ = std::format("ecat socket: {}", strerror(errno));
        return false;
    }
    if (!CBPFProgramFactory::attach(ecat_fd_, ecat_prog)) {
        last_error_ = std::format("ecat BPF attach: {}", strerror(errno));
        stop(); return false;
    }
    proxy_fd_ = openFilteredSocket(proxy_ifindex_, "pxy-fwd");
    if (proxy_fd_ < 0) {
        last_error_ = std::format("proxy socket: {}", strerror(errno));
        stop(); return false;
    }
    if (!CBPFProgramFactory::attach(proxy_fd_, proxy_prog)) {
        last_error_ = std::format("proxy BPF attach: {}", strerror(errno));
        stop(); return false;
    }
    mon_fd_ = openFilteredSocket(proxy_ifindex_, "pxy-mon");
    if (mon_fd_ < 0 || !CBPFProgramFactory::attach(mon_fd_, mon_prog)) {
        last_error_ = std::format("monitor socket/BPF: {}", strerror(errno));
        stop(); return false;
    }

    if (cfg.udp_input_port != 0 && !openUdpListener(cfg, encap)) {
        stop(); return false;
    }

    stop_fd_ = ::eventfd(0, EFD_NONBLOCK | EFD_CLOEXEC);
    if (stop_fd_ < 0) {
        last_error_ = std::format("eventfd: {}", strerror(errno));
        stop(); return false;
    }

    stop_flag_.store(false, std::memory_order_relaxed);
    running_.store(true, std::memory_order_release);
    pump_ = std::thread(&ProxyForwarder::pumpLoop, this);

    TETHER_LOGI(TAG, "proxy forwarding up: {} idx [0x{:02X},0x{:02X}] "
                "(ecat ifindex {}, proxy ifindex {})",
                cfg.ifname, band_lo_, band_hi_,
                ecat_ifindex_, proxy_ifindex_);
    return true;
}

void ProxyForwarder::stop()
{
    stop_flag_.store(true, std::memory_order_release);
    if (stop_fd_ >= 0) {
        const uint64_t one = 1;
        ::write(stop_fd_, &one, sizeof(one));
    }
    if (pump_.joinable()) pump_.join();
    for (int* p : {&ecat_fd_, &proxy_fd_, &mon_fd_}) {
        if (*p >= 0) {
            HAL::unregisterDiagPacketSocket(*p);
            ::close(*p);
            *p = -1;
        }
    }
    if (stop_fd_ >= 0) { ::close(stop_fd_); stop_fd_ = -1; }
    if (udp_fd_ >= 0) { ::close(udp_fd_); udp_fd_ = -1; }
    udp_port_ = 0;
    udp_peer_ip_.store(0, std::memory_order_release);
    udp_peer_port_.store(0, std::memory_order_release);
    filter_.reset();
    running_.store(false, std::memory_order_release);
}

// ============================================================================
// Pump loop
// ============================================================================

void ProxyForwarder::pumpLoop()
{
    pollfd fds[5] = {
        {ecat_fd_,  POLLIN, 0},
        {proxy_fd_, POLLIN, 0},
        {mon_fd_,   POLLIN, 0},
        {stop_fd_,  POLLIN, 0},
        {udp_fd_,   POLLIN, 0},    // -1 → ignored by poll
    };
    while (!stop_flag_.load(std::memory_order_relaxed)) {
        const int n = ::poll(fds, 5, -1);
        if (n < 0) {
            if (errno == EINTR) continue;
            TETHER_LOGE(TAG, "poll failed: {} — proxy pump exiting",
                        strerror(errno));
            break;
        }
        if (fds[3].revents & POLLIN) break;
        if (fds[0].revents & POLLIN) drainEcat();
        if (fds[1].revents & POLLIN) drainProxy();
        if (fds[2].revents & POLLIN) drainMonitor();
        if (udp_fd_ >= 0 && (fds[4].revents & POLLIN)) drainUdp();
    }
}

void ProxyForwarder::drainEcat()
{
    // In-band frames arriving on the bus interface = slave responses to
    // injected datagrams → verbatim to the proxy interface.
    uint8_t buf[kRxBufSize];
    for (int i = 0; i < 64; ++i) {          // bounded per wake
        const ssize_t n = ::recv(ecat_fd_, buf, sizeof(buf), MSG_DONTWAIT);
        if (n <= 0) break;
        if (sendRaw(proxy_fd_, proxy_ifindex_, buf, static_cast<size_t>(n)))
            rx_forwarded_.fetch_add(1, std::memory_order_relaxed);
        sendUdpReply(buf, static_cast<size_t>(n));
    }
}

void ProxyForwarder::drainProxy()
{
    // In-band frames from the external source → re-validate the full
    // datagram chain (PDO-only + every idx in-band — the BPF keys on the
    // FIRST index only) → inject onto the bus interface.
    uint8_t buf[kRxBufSize];
    for (int i = 0; i < 64; ++i) {
        const ssize_t n = ::recv(proxy_fd_, buf, sizeof(buf), MSG_DONTWAIT);
        if (n <= 0) break;
        const size_t len = static_cast<size_t>(n);
        bool ok = frameIsInBandPdo(buf, len, band_lo_, band_hi_);
        bool acl_fail = false;
        if (ok && filter_) {
            ok = chainPassesFilter(buf, len, *filter_);
            acl_fail = !ok;
        }
        if (!ok) {
            auto& ctr = acl_fail ? filtered_ : tx_rejected_;
            const uint64_t rej = ctr.fetch_add(
                1, std::memory_order_relaxed) + 1;
            if (rej <= 8 || (rej & 0xFF) == 0) {
                TETHER_LOGE(TAG, "proxy: {} frame on proxy interface "
                            "rejected (idx={}, len={}) — not forwarded [{}]",
                            acl_fail ? "address-filtered"
                                     : "non-PDO/malformed",
                            firstIdxOf(buf, len), len, rej);
            }
            continue;
        }
        if (sendRaw(ecat_fd_, ecat_ifindex_, buf, len))
            tx_forwarded_.fetch_add(1, std::memory_order_relaxed);
    }
}

void ProxyForwarder::drainMonitor()
{
    // EtherCAT frames OUTSIDE the band on the proxy interface — the
    // contract: ignore them and report errors.  Rate-limited so a
    // misconfigured source cannot flood the log.
    uint8_t buf[2048];
    for (int i = 0; i < 64; ++i) {
        const ssize_t n = ::recv(mon_fd_, buf, sizeof(buf), MSG_DONTWAIT);
        if (n <= 0) break;
        const uint64_t hits = out_of_band_.fetch_add(
            1, std::memory_order_relaxed) + 1;
        if (hits <= 8 || (hits & 0xFF) == 0) {
            TETHER_LOGE(TAG, "proxy monitor: out-of-band EtherCAT frame "
                        "(first idx={}, len={}) outside band "
                        "[0x{:02X},0x{:02X}] — ignored [{}]",
                        firstIdxOf(buf, static_cast<size_t>(n)),
                        static_cast<size_t>(n), band_lo_, band_hi_, hits);
        }
    }
}

bool ProxyForwarder::sendRaw(int fd, int ifindex, const uint8_t* f, size_t len)
{
    sockaddr_ll sll{};
    sll.sll_family  = AF_PACKET;
    sll.sll_ifindex = ifindex;
    sll.sll_halen   = 6;
    std::memcpy(sll.sll_addr, f, 6);            // dst MAC = frame's dst
    const ssize_t r = ::sendto(fd, f, len, MSG_DONTWAIT,
                               reinterpret_cast<sockaddr*>(&sll),
                               sizeof(sll));
    if (r < 0) {
        const uint64_t errs = send_errors_.fetch_add(
            1, std::memory_order_relaxed) + 1;
        if (errs <= 8 || (errs & 0xFF) == 0) {
            TETHER_LOGE(TAG, "proxy sendto(ifindex {}) failed: {} [{}]",
                        ifindex, strerror(errno), errs);
        }
        return false;
    }
    return true;
}

bool ProxyForwarder::openUdpListener(const ProxyForwardConfig& cfg,
                                     const EtherCAT::WireEncap& encap)
{
    udp_fd_ = ::socket(AF_INET, SOCK_DGRAM | SOCK_NONBLOCK | SOCK_CLOEXEC,
                       IPPROTO_UDP);
    if (udp_fd_ < 0) {
        last_error_ = std::format("udp socket: {}", strerror(errno));
        return false;
    }
    if (setsockopt(udp_fd_, SOL_SOCKET, SO_BINDTODEVICE,
                   cfg.ifname.c_str(),
                   static_cast<socklen_t>(cfg.ifname.size() + 1)) < 0) {
        last_error_ = std::format("udp SO_BINDTODEVICE '{}': {}",
                                  cfg.ifname, strerror(errno));
        return false;
    }
    sockaddr_in sa{};
    sa.sin_family      = AF_INET;
    sa.sin_addr.s_addr = htonl(INADDR_ANY);
    sa.sin_port        = htons(cfg.udp_input_port);
    if (::bind(udp_fd_, reinterpret_cast<sockaddr*>(&sa), sizeof(sa)) < 0) {
        last_error_ = std::format("udp bind port {}: {}",
                                  cfg.udp_input_port, strerror(errno));
        return false;
    }

    // Synthesised outer header for decapsulated input: broadcast dst,
    // the BUS interface's MAC as src, EtherType 0x88A4 — plus an
    // 802.1Q tag (TX VID) when the wire runs VLAN-encapsulated.
    memset(udp_eth_hdr_, 0xFF, 6);
    std::memset(udp_eth_hdr_ + 6, 0, 6);
    char ename[IFNAMSIZ] = {};
    if (::if_indextoname(static_cast<unsigned>(ecat_ifindex_), ename)) {
        int ctl = ::socket(AF_INET, SOCK_DGRAM | SOCK_CLOEXEC, 0);
        if (ctl >= 0) {
            ifreq ifr{};
            std::strncpy(ifr.ifr_name, ename, IFNAMSIZ - 1);
            if (::ioctl(ctl, SIOCGIFHWADDR, &ifr) == 0)
                std::memcpy(udp_eth_hdr_ + 6, ifr.ifr_hwaddr.sa_data, 6);
            ::close(ctl);
        }
    }
    if (encap.tagged()) {
        udp_eth_hdr_[12] = 0x81; udp_eth_hdr_[13] = 0x00;
        udp_eth_hdr_[14] = static_cast<uint8_t>(encap.tx_vlan >> 8);
        udp_eth_hdr_[15] = static_cast<uint8_t>(encap.tx_vlan);
        udp_eth_hdr_[16] = 0x88; udp_eth_hdr_[17] = 0xA4;
        udp_eth_len_ = 18;
    } else {
        udp_eth_hdr_[12] = 0x88; udp_eth_hdr_[13] = 0xA4;
        udp_eth_len_ = 14;
    }
    udp_port_ = cfg.udp_input_port;
    TETHER_LOGI(TAG, "proxy UDP input on {}:{} (ETG.1000.3)",
                cfg.ifname, udp_port_);
    return true;
}

void ProxyForwarder::drainUdp()
{
    // EtherCAT-over-UDP input: payload = bare EtherCAT frame
    // ([ecat hdr][datagrams]).  Wrap it in the synthesised Ethernet
    // header and run it through the exact same validation + filter
    // path as raw proxy-interface frames.
    uint8_t buf[kRxBufSize];
    uint8_t frame[kRxBufSize];
    for (int i = 0; i < 64; ++i) {
        sockaddr_in peer{};
        socklen_t plen = sizeof(peer);
        const ssize_t n = ::recvfrom(udp_fd_, buf, sizeof(buf),
                                     MSG_DONTWAIT,
                                     reinterpret_cast<sockaddr*>(&peer),
                                     &plen);
        if (n <= 0) break;
        const size_t len = static_cast<size_t>(n);
        if (udp_eth_len_ == 0 || len > sizeof(frame) - udp_eth_len_) {
            tx_rejected_.fetch_add(1, std::memory_order_relaxed);
            continue;
        }
        // Last-seen peer wins — single UDP source at a time.
        udp_peer_port_.store(peer.sin_port, std::memory_order_relaxed);
        udp_peer_ip_.store(peer.sin_addr.s_addr,
                           std::memory_order_release);

        std::memcpy(frame, udp_eth_hdr_, udp_eth_len_);
        std::memcpy(frame + udp_eth_len_, buf, len);
        const size_t flen = udp_eth_len_ + len;
        bool ok = frameIsInBandPdo(frame, flen, band_lo_, band_hi_);
        bool acl_fail = false;
        if (ok && filter_) {
            ok = chainPassesFilter(frame, flen, *filter_);
            acl_fail = !ok;
        }
        if (!ok) {
            auto& ctr = acl_fail ? filtered_ : tx_rejected_;
            const uint64_t rej = ctr.fetch_add(
                1, std::memory_order_relaxed) + 1;
            if (rej <= 8 || (rej & 0xFF) == 0) {
                TETHER_LOGE(TAG, "proxy: UDP-encapsulated {} frame "
                            "rejected (idx={}, len={}) [{}]",
                            acl_fail ? "address-filtered"
                                     : "non-PDO/malformed",
                            firstIdxOf(frame, flen), len, rej);
            }
            continue;
        }
        if (sendRaw(ecat_fd_, ecat_ifindex_, frame, flen))
            tx_forwarded_.fetch_add(1, std::memory_order_relaxed);
    }
}

void ProxyForwarder::sendUdpReply(const uint8_t* frame, size_t len)
{
    const uint32_t ip = udp_peer_ip_.load(std::memory_order_acquire);
    if (ip == 0 || len <= kEthTypeOff + 2) return;   // no UDP peer yet
    // Strip the outer Ethernet (+802.1Q) header — UDP payload is the
    // bare EtherCAT frame again.
    const uint16_t et = static_cast<uint16_t>(
        (frame[kEthTypeOff] << 8) | frame[kEthTypeOff + 1]);
    const size_t off = (et == kEtherType8021Q) ? 18 : 14;
    if (et != kEtherTypeEtherCAT && et != kEtherType8021Q) return;
    if (len <= off) return;
    sockaddr_in peer{};
    peer.sin_family      = AF_INET;
    peer.sin_addr.s_addr = ip;
    peer.sin_port        = udp_peer_port_.load(std::memory_order_relaxed);
    if (::sendto(udp_fd_, frame + off, len - off, MSG_DONTWAIT,
                 reinterpret_cast<sockaddr*>(&peer), sizeof(peer)) < 0) {
        const uint64_t errs = send_errors_.fetch_add(
            1, std::memory_order_relaxed) + 1;
        if (errs <= 8 || (errs & 0xFF) == 0) {
            TETHER_LOGE(TAG, "proxy UDP reply failed: {} [{}]",
                        strerror(errno), errs);
        }
    }
}

#else // !__linux__

ProxyForwardStats ProxyForwarder::stats() const { return {}; }

bool ProxyForwarder::start(const ProxyForwardConfig&, int,
                           const EtherCAT::WireEncap&, const ProxyPdoFilter*)
{
    last_error_ = "proxy forwarding requires Linux AF_PACKET sockets";
    return false;
}

void ProxyForwarder::stop() {}

#endif // __linux__

} // namespace EtherCAT

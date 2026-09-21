/**
 * @file WindowsEthernet.cpp
 * @brief Windows Ethernet HAL implementation over Npcap (wpcap.dll)
 *
 * Raw Ethernet on Windows goes through the Npcap driver: pcap_open() on a
 * \\Device\\NPF_{GUID} adapter gives a capture/inject handle; TX is
 * pcap_sendpacket(), RX is pcap_next_ex() in non-blocking mode.
 *
 * Requirements: Npcap installed with "WinPcap API-compatible mode" (ships
 * wpcap.dll + packet.dll).  Link against wpcap + iphlpapi; the CMake
 * component adds both for TETHER_PLATFORM_WINDOWS.
 *
 * Scope note (Q12): this is the async/HAL path.  The zero-copy cyclic ring
 * channel remains Linux-only (TPACKET_V2); on Windows the cyclic datapath
 * uses this socket-style channel — TX is one pcap_sendpacket() syscall and
 * RX drains via recvFrame()/poll(), same semantics as LinuxRawSocketEthernet.
 */

#ifdef _WIN32

#include "hal/IEthernet.hpp"
#include "hal/HALTypes.hpp"
#include "logging/Logger.hpp"

#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <winsock2.h>
#include <windows.h>
#include <iphlpapi.h>
#include <pcap.h>

#include <atomic>
#include <chrono>
#include <cstring>
#include <string>
#include <thread>
#include <vector>

#pragma comment(lib, "wpcap.lib")
#pragma comment(lib, "iphlpapi.lib")
#pragma comment(lib, "ws2_32.lib")

namespace EtherCAT {
namespace HAL {

namespace {

/// Resolve a friendly name ("Ethernet0") or GUID to the Npcap device path
/// ("\\Device\\NPF_{...}").  Returns empty string when not found.
std::string resolveNpcapDevice(const char* ifname) {
    if (!ifname || !ifname[0]) return {};
    std::string name(ifname);
    if (name.rfind("\\Device\\NPF_", 0) == 0) return name;   // already a path

    pcap_if_t* all = nullptr;
    char errbuf[PCAP_ERRBUF_SIZE] = {};
    if (pcap_findalldevs(&all, errbuf) != 0) return {};

    std::string result;
    for (pcap_if_t* d = all; d; d = d->next) {
        // Match on device path tail ({GUID}) or the Npcap description.
        if (name == d->name ||
            (d->description && name == d->description)) {
            result = d->name;
            break;
        }
    }
    pcap_freealldevs(all);
    return result;
}

/// MAC + link state via GetAdaptersAddresses; key is the {GUID} substring
/// of the Npcap device path (adapter names are "Ethernet {GUID}"-style —
/// the GUID is what survives in both listings).
bool queryAdapter(const std::string& npcap_name, MacAddress& mac,
                  bool& link_up) {
    // Extract {GUID} — the token between "NPF_" and the end.
    const auto pos = npcap_name.find("NPF_");
    const std::string guid =
        pos != std::string::npos ? npcap_name.substr(pos + 4) : npcap_name;

    ULONG buf_len = 15 * 1024;
    std::vector<uint8_t> buf(buf_len);
    auto* addrs = reinterpret_cast<IP_ADAPTER_ADDRESSES*>(buf.data());
    ULONG ret = GetAdaptersAddresses(AF_UNSPEC,
        GAA_FLAG_SKIP_ANYCAST | GAA_FLAG_SKIP_MULTICAST |
        GAA_FLAG_SKIP_DNS_SERVER,
        nullptr, addrs, &buf_len);
    if (ret == ERROR_BUFFER_OVERFLOW) {
        buf.resize(buf_len);
        addrs = reinterpret_cast<IP_ADAPTER_ADDRESSES*>(buf.data());
        ret = GetAdaptersAddresses(AF_UNSPEC,
            GAA_FLAG_SKIP_ANYCAST | GAA_FLAG_SKIP_MULTICAST |
            GAA_FLAG_SKIP_DNS_SERVER,
            nullptr, addrs, &buf_len);
    }
    if (ret != NO_ERROR) return false;

    for (auto* a = addrs; a; a = a->Next) {
        if (!a->AdapterName) continue;
        if (!strstr(a->AdapterName, guid.c_str())) continue;
        if (a->PhysicalAddressLength == 6) {
            memcpy(mac.bytes, a->PhysicalAddress, 6);
        }
        link_up = (a->OperStatus == IfOperStatusUp);
        return true;
    }
    return false;
}

} // namespace

class WindowsNpcapEthernet : public IEthernet {
public:
    WindowsNpcapEthernet() = default;
    ~WindowsNpcapEthernet() override { shutdown(); }

    Error init(const EthernetConfig& config) override {
        if (m_initialized) return Error::AlreadyInitialized;

        const std::string dev =
            resolveNpcapDevice(config.interfaceName);
        if (dev.empty()) return Error::InterfaceNotFound;
        strncpy_s(m_ifname, dev.c_str(), sizeof(m_ifname) - 1);

        char errbuf[PCAP_ERRBUF_SIZE] = {};
        const int flags = config.promiscuous ? PCAP_OPENFLAG_PROMISCUOUS : 0;
        // snaplen covers jumbo; immediate + non-blocking for RX polling.
        m_pcap = pcap_open(dev.c_str(), 65536, flags,
                           /*read_timeout_ms*/1, nullptr, errbuf);
        if (!m_pcap) return Error::PermissionDenied;

        if (pcap_setnonblock(m_pcap, 1, errbuf) != 0) {
            pcap_close(m_pcap);
            m_pcap = nullptr;
            return Error::InternalError;
        }
        pcap_set_immediate_mode(m_pcap, 1);

        MacAddress mac{};
        bool up = false;
        if (queryAdapter(dev, mac, up)) m_mac = mac;

        m_ethertypeFilter = config.ethertypeFilter;
        m_promiscuous = config.promiscuous;
        m_initialized = true;
        return Error::OK;
    }

    void shutdown() override {
        m_running = false;
        if (m_pcap) {
            pcap_close(m_pcap);
            m_pcap = nullptr;
        }
        m_initialized = false;
        m_rxCallback = nullptr;
        m_linkCallback = nullptr;
    }

    bool isInitialized() const override { return m_initialized; }

    Error getMacAddress(MacAddress& mac) const override {
        if (!m_initialized) return Error::NotInitialized;
        mac = m_mac;
        return Error::OK;
    }

    Error setMacAddress(const MacAddress&) override {
        // Npcap injects the frame verbatim — the source MAC in the frame
        // is whatever the caller put there; rewriting the adapter's own
        // MAC is a driver/NDIS operation, not a HAL concern.
        return Error::NotSupported;
    }

    Error transmit(const uint8_t* frame, size_t length) override {
        if (!m_initialized) return Error::NotInitialized;
        if (!frame || length < kMinFrameSize) return Error::InvalidArgument;
        if (length > kMaxFrameSize) return Error::BufferTooSmall;

        if (pcap_sendpacket(m_pcap, frame, static_cast<int>(length)) != 0) {
            m_stats.txErrors++;
            return Error::TransmitFailed;
        }
        m_stats.txFrames++;
        m_stats.txBytes += length;
        return Error::OK;
    }

    Error transmitVlan(const uint8_t* frame, size_t length,
                       uint16_t vlanId, uint8_t priority) override {
        if (!m_initialized) return Error::NotInitialized;
        if (!frame || length < kMinFrameSize) return Error::InvalidArgument;
        if (length + kVlanTagSize > kMaxFrameSizeVlan)
            return Error::BufferTooSmall;

        uint8_t vlanFrame[kMaxFrameSizeVlan];
        memcpy(vlanFrame, frame, 12);
        vlanFrame[12] = 0x81;
        vlanFrame[13] = 0x00;
        const uint16_t tci =
            static_cast<uint16_t>(((priority & 0x07) << 13) |
                                  (vlanId & 0x0FFF));
        vlanFrame[14] = static_cast<uint8_t>(tci >> 8);
        vlanFrame[15] = static_cast<uint8_t>(tci & 0xFF);
        memcpy(vlanFrame + 16, frame + 12, length - 12);
        return transmit(vlanFrame, length + kVlanTagSize);
    }

    Error transmitGather(const BufferDesc* iov, size_t count) override {
        if (!m_initialized) return Error::NotInitialized;
        if (!iov || count == 0) return Error::InvalidArgument;

        size_t total = 0;
        for (size_t i = 0; i < count; ++i) total += iov[i].length;
        if (total < kMinFrameSize || total > kMaxFrameSize)
            return Error::InvalidArgument;

        uint8_t frame[kMaxFrameSize];
        size_t off = 0;
        for (size_t i = 0; i < count; ++i) {
            memcpy(frame + off, iov[i].data, iov[i].length);
            off += iov[i].length;
        }
        return transmit(frame, total);
    }

    void setRxCallback(RxCallback cb, void* userData) override {
        m_rxCallback = std::move(cb);
        m_rxUserData = userData;
    }

    int poll(Milliseconds timeoutMs) override {
        if (!m_initialized || !m_pcap) return 0;
        // pcap handle is non-blocking; timeout waits via a light sleep so
        // the caller's deadline isn't tied to Npcap's buffered dispatch.
        const auto deadline = std::chrono::steady_clock::now() +
            std::chrono::milliseconds(timeoutMs);
        int count = 0;
        for (;;) {
            count += drainOnce();
            if (count > 0 || timeoutMs == 0 ||
                std::chrono::steady_clock::now() >= deadline) {
                return count;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
    }

    int recvFrame(uint8_t* buffer, size_t capacity,
                  RxFrameInfo* info) override {
        if (!m_initialized || !m_pcap) return -1;

        pcap_pkthdr* hdr = nullptr;
        const u_char* data = nullptr;
        const int r = pcap_next_ex(m_pcap, &hdr, &data);
        if (r == 0) return 0;                    // timeout / none pending
        if (r < 0 || !hdr || !data) return -1;   // error / EOF
        if (hdr->caplen < 14 || hdr->caplen > capacity) return 0;

        // EtherType filter (inner type if VLAN-tagged inline).
        uint16_t ethertype =
            static_cast<uint16_t>((data[12] << 8) | data[13]);
        if (m_ethertypeFilter != 0) {
            uint16_t check = ethertype;
            if (check == 0x8100 && hdr->caplen >= 18) {
                check = static_cast<uint16_t>((data[16] << 8) | data[17]);
            }
            if (check != m_ethertypeFilter) {
                m_stats.rxFiltered++;
                return 0;
            }
        }

        memcpy(buffer, data, hdr->caplen);
        m_stats.rxFrames++;
        m_stats.rxBytes += hdr->caplen;

        if (info) {
            info->timestamp = static_cast<Timestamp>(hdr->ts.tv_sec) *
                              1000000ull + hdr->ts.tv_usec;
            info->vlanTagPresent = (ethertype == 0x8100);
            if (info->vlanTagPresent && hdr->caplen >= 18) {
                const uint16_t tci =
                    static_cast<uint16_t>((data[14] << 8) | data[15]);
                info->vlanId = tci & 0x0FFF;
                info->vlanPriority = (tci >> 13) & 0x07;
            }
        }
        return static_cast<int>(hdr->caplen);
    }

    void setEthertypeFilter(uint16_t ethertype) override {
        m_ethertypeFilter = ethertype;
    }

    Error setPromiscuous(bool enable) override {
        // wpcap sets promiscuity at pcap_open() time; toggling later needs
        // Packet32 (PacketSetHwFilter).  Report NotSupported when the
        // change differs from the open-time state.
        return enable == m_promiscuous ? Error::OK : Error::NotSupported;
    }

    Error addMulticastAddress(const MacAddress&) override {
        return Error::NotSupported;   // needs Packet32 PacketSetMulticastList
    }
    Error removeMulticastAddress(const MacAddress&) override {
        return Error::NotSupported;
    }
    Error setAllMulticast(bool) override {
        return Error::NotSupported;
    }

    LinkStatus getLinkStatus() const override {
        LinkStatus s;
        if (!m_initialized) return s;
        MacAddress mac{};
        bool up = false;
        if (queryAdapter(m_ifname, mac, up)) s.up = up;
        return s;
    }

    void setLinkCallback(LinkCallback cb, void* userData) override {
        m_linkCallback = std::move(cb);
        m_linkUserData = userData;
    }

    Error waitForLinkUp(Milliseconds timeoutMs) override {
        const auto deadline = std::chrono::steady_clock::now() +
            std::chrono::milliseconds(timeoutMs);
        while (std::chrono::steady_clock::now() < deadline) {
            if (getLinkStatus().up) return Error::OK;
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
        return getLinkStatus().up ? Error::OK : Error::Timeout;
    }

    EthernetStats getStats() const override { return m_stats; }
    void resetStats() override { m_stats = {}; }

    void* nativeHandle() override { return m_pcap; }
    const char* getInterfaceName() const override { return m_ifname; }

private:
    /// Drain all queued frames through the Rx callback (poll() path).
    int drainOnce() {
        int count = 0;
        uint8_t buf[kMaxFrameSizeVlan];
        for (;;) {
            RxFrameInfo info;
            const int n = recvFrame(buf, sizeof(buf), &info);
            if (n <= 0) break;
            ++count;
            if (m_rxCallback) {
                m_rxCallback(buf, static_cast<size_t>(n), info,
                             m_rxUserData);
            }
        }
        return count;
    }

    bool m_initialized = false;
    pcap_t* m_pcap = nullptr;
    char m_ifname[160] = {0};   // Npcap device path
    MacAddress m_mac{};
    bool m_promiscuous = false;
    uint16_t m_ethertypeFilter = 0;
    std::atomic<bool> m_running{false};

    RxCallback m_rxCallback;
    void* m_rxUserData = nullptr;
    LinkCallback m_linkCallback;
    void* m_linkUserData = nullptr;

    EthernetStats m_stats;
};

std::unique_ptr<IEthernet> createDefaultEthernet() {
    return std::make_unique<WindowsNpcapEthernet>();
}

} // namespace HAL
} // namespace EtherCAT

#endif // _WIN32

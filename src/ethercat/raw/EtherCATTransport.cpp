/**
 * @file EtherCATTransport.cpp
 * @brief Implementation of UDP encapsulation for EtherCATTransport
 */

#include "tether/ethercat/EtherCATTransport.hpp"

#if TETHER_ENABLE_UDP_ENCAPSULATION

#include "raw/internal.hpp" // RawWireFormat: EthernetHeader, IPv4Header, UDPHeader, etc.

#include <cstring>

namespace EtherCAT {

// ============================================================================
// UDP Encapsulation (ETG.1000.3)
// ============================================================================

uint16_t EtherCATTransport::computeIpChecksum(const uint8_t* ip_header)
{
    // Standard IPv4 header checksum: ones-complement sum of all 16-bit words.
    uint32_t sum = 0;
    for (int i = 0; i < 20; i += 2) {
        sum += static_cast<uint32_t>(ip_header[i]) << 8;
        sum += static_cast<uint32_t>(ip_header[i + 1]);
    }
    while (sum >> 16) {
        sum = (sum & 0xFFFF) + (sum >> 16);
    }
    return static_cast<uint16_t>(~sum & 0xFFFF);
}

bool EtherCATTransport::encapsulateFrame(const uint8_t* in_frame, size_t in_len,
                                          uint8_t* out_buf, size_t out_cap, size_t* out_len) const
{
    using namespace Raw;

    // Input frame must have at least an Ethernet header (14 bytes).
    if (in_len < sizeof(EtherCAT::EthernetHeader)) return false;

    // The EtherCAT payload is everything after the 14-byte Ethernet header.
    const size_t ecat_payload_len = in_len - sizeof(EtherCAT::EthernetHeader);
    const size_t encap_len = sizeof(EtherCAT::EthernetHeader) +
                             sizeof(IPv4Header) + sizeof(UDPHeader) + ecat_payload_len;

    // Minimum Ethernet frame size (without FCS).
    constexpr size_t kMinEthFrame = 60;
    const size_t padded_len = (encap_len < kMinEthFrame) ? kMinEthFrame : encap_len;

    if (padded_len > out_cap) return false;

    std::memset(out_buf, 0, padded_len);

    // --- Ethernet header (reuse dst/src MAC from input, change EtherType to IPv4) ---
    const auto* in_eth = reinterpret_cast<const EtherCAT::EthernetHeader*>(in_frame);
    auto* out_eth = reinterpret_cast<EtherCAT::EthernetHeader*>(out_buf);
    std::memcpy(out_eth->dst, in_eth->dst, 6);
    std::memcpy(out_eth->src, in_eth->src, 6);
    out_eth->etherType_be = host_to_be16(kEtherTypeIPv4);

    // --- IPv4 header ---
    auto* ip = reinterpret_cast<IPv4Header*>(out_buf + sizeof(EtherCAT::EthernetHeader));
    ip->version_ihl = 0x45;
    ip->tos = 0;
    const uint16_t ip_total_len = static_cast<uint16_t>(sizeof(IPv4Header) + sizeof(UDPHeader) + ecat_payload_len);
    ip->total_length_be = host_to_be16(ip_total_len);
    ip->identification_be = host_to_be16(ip_id_counter_.fetch_add(1, std::memory_order_relaxed));
    ip->flags_fragment_be = host_to_be16(0x4000); // Don't Fragment, no offset
    ip->ttl = 64;
    ip->protocol = 0x11; // UDP
    ip->checksum_be = 0; // Will be computed below
    ip->src_ip_be = host_to_be32(udp_config_->source_ip);
    ip->dst_ip_be = host_to_be32(udp_config_->destination_ip);
    // Compute checksum over the 20-byte header.
    ip->checksum_be = host_to_be16(computeIpChecksum(reinterpret_cast<const uint8_t*>(ip)));

    // --- UDP header ---
    auto* udp = reinterpret_cast<UDPHeader*>(out_buf + sizeof(EtherCAT::EthernetHeader) + sizeof(IPv4Header));
    udp->src_port_be = host_to_be16(udp_config_->source_port);
    udp->dst_port_be = host_to_be16(udp_config_->destination_port);
    const uint16_t udp_len = static_cast<uint16_t>(sizeof(UDPHeader) + ecat_payload_len);
    udp->length_be = host_to_be16(udp_len);
    udp->checksum_be = 0; // Not computed (optional for IPv4 UDP)

    // --- EtherCAT payload (everything from the original frame after the Ethernet header) ---
    uint8_t* ecat_dst = out_buf + sizeof(EtherCAT::EthernetHeader) + sizeof(IPv4Header) + sizeof(UDPHeader);
    std::memcpy(ecat_dst, in_frame + sizeof(EtherCAT::EthernetHeader), ecat_payload_len);

    *out_len = padded_len;
    return true;
}

bool EtherCATTransport::sendWithEncapsulation(const uint8_t* frame, size_t len) const
{
    if (!udp_config_ || !udp_config_->enabled) {
        return (iface_ && iface_->send) ? iface_->send(frame, len) : false;
    }

    // Build the encapsulated frame in a stack buffer.
    // Max frame: 1514 (max Ethernet) + 28 (IP+UDP overhead) = 1542, round up to 1600.
    constexpr size_t kEncapBufSize = 1600;
    uint8_t encap_buf[kEncapBufSize] = {0};
    size_t encap_len = 0;
    if (!encapsulateFrame(frame, len, encap_buf, kEncapBufSize, &encap_len)) {
        return false;
    }
    return (iface_ && iface_->send) ? iface_->send(encap_buf, encap_len) : false;
}

} // namespace EtherCAT

#endif // TETHER_ENABLE_UDP_ENCAPSULATION

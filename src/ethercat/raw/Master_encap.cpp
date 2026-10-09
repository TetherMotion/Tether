/**
 * @file Master_encap.cpp
 * @brief Master — EtherCAT-over-UDP encapsulation helpers.
 *
 * Thin forwarding layer over the Master's EtherCATTransport: IP checksum,
 * frame encapsulation, encapsulated send (serialized on send_mutex_), and the
 * per-frame payload ceiling.  Compiled out when
 * TETHER_ENABLE_UDP_ENCAPSULATION is off (Master.hpp provides inline
 * fallbacks).
 */

#include "tether/ethercat/Master.hpp"
#include "tether/ethercat/EtherCATTransport.hpp"
#include "raw/internal.hpp"

#include <mutex>

namespace EtherCAT {

// ============================================================================
// EtherCAT-over-UDP encapsulation helpers
// ============================================================================

#if TETHER_ENABLE_UDP_ENCAPSULATION
uint16_t Master::computeIpChecksum(const uint8_t* ip_header)
{
    return EtherCATTransport::computeIpChecksum(ip_header);
}

bool Master::encapsulateFrame(const uint8_t* in_frame, size_t in_len,
                              uint8_t* out_buf, size_t out_cap, size_t* out_len) const
{
    if (!transport_) return false;
    return transport_->encapsulateFrame(in_frame, in_len, out_buf, out_cap, out_len);
}

bool Master::sendWithEncapsulation(const uint8_t* frame, size_t len)
{
    std::lock_guard<std::mutex> lock(send_mutex_);
    if (!transport_) {
        // Fallback before start() — direct send without encapsulation
        return iface_.send ? iface_.send(frame, len) : false;
    }
    return transport_->send(frame, len);
}

size_t Master::maxEtherCATPayloadPerFrame() const
{
    if (!transport_) return Raw::kMaxEtherCATPayloadPerFrame;
    return transport_->maxEtherCATPayloadPerFrame();
}
#endif // TETHER_ENABLE_UDP_ENCAPSULATION

} // namespace EtherCAT


#include "tether/ethercat/EtherCATMaster.hpp"
#include "tether/ethercat/EtherCATTypes.hpp"
#include <cstring>

namespace EtherCAT {
namespace Raw {

#if !defined(ESP_PLATFORM)

static const NetworkInterface* s_registered_iface = nullptr;
static uint8_t s_registered_src_mac[6] = {0};

void set_network_interface(const NetworkInterface* iface)
{
    s_registered_iface = iface;
}

const NetworkInterface* network_interface()
{
    return s_registered_iface;
}

void set_src_mac(const uint8_t src_mac[6])
{
    if (src_mac) std::memcpy(s_registered_src_mac, src_mac, 6);
}

const uint8_t* get_src_mac()
{
    return s_registered_src_mac;
}

void parse_ethercat_frame(const uint8_t* frame, size_t length)
{
    if (!frame || length == 0) return;

    // Prefer the master associated with the registered NetworkInterface
    if (s_registered_iface) {
        EtherCATMaster* m = EtherCATMaster::findByNetworkInterface(s_registered_iface);
        if (m) {
            m->handleRxFrame(frame, length);
            return;
        }
    }

    // No matching master found for the registered iface — nothing to do on host.
}

#endif // !ESP_PLATFORM

} // namespace Raw
} // namespace EtherCAT

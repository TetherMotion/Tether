/**
 * @file EL2004.cpp
 * @brief EL2004 factories.  All device logic lives in OutputTerminal.cpp —
 *        the EL2004 is just a OutputTerminal with a bound identity.
 */

#include "tether/Beckhoff/EL2004.hpp"

#include "tether/ethercat/Master.hpp"

namespace EtherCAT {
namespace Beckhoff {

EL2004::Result<EL2004> EL2004::findFirst(Master& master) {
    auto scan = master.discovery().discover(
        {DiscoveryOption::VendorId, DiscoveryOption::ProductCode});
    return findFirst(master, scan);
}

EL2004::Result<EL2004> EL2004::findFirst(
    Master& master, std::span<const DiscoveredSlave> scan) {
    for (const auto& s : scan) {
        if (matches(s)) return EL2004(master, s);
    }
    return std::unexpected(Error::NoDeviceFound);
}

} // namespace Beckhoff

} // namespace EtherCAT

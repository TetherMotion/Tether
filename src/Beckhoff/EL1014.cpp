/**
 * @file EL1014.cpp
 * @brief EL1014 factories.  All device logic lives in PackedInput.cpp —
 *        the EL1014 is just a PackedInput with a bound identity.
 */

#include "tether/Beckhoff/EL1014.hpp"

#include "tether/ethercat/Master.hpp"

namespace EtherCAT {
namespace Beckhoff {

EL1014::Result<EL1014> EL1014::findFirst(Master& master) {
    auto scan = master.discovery().discover(
        {DiscoveryOption::VendorId, DiscoveryOption::ProductCode});
    return findFirst(master, scan);
}

EL1014::Result<EL1014> EL1014::findFirst(
    Master& master, std::span<const DiscoveredSlave> scan) {
    for (const auto& s : scan) {
        if (matches(s)) return EL1014(master, s);
    }
    return std::unexpected(Error::NoDeviceFound);
}

} // namespace Beckhoff

} // namespace EtherCAT

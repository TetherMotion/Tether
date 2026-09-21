/**
 * @file SlaveESIHelpers.hpp
 * @brief ESI/SII lookup helpers shared by Slave.cpp and Slave_pdo.cpp.
 *
 * @internal Internal header — not installed, not part of the public API.
 */

#pragma once

#include "tether/ethercat/ESITypes.hpp"
#include "tether/sii/SIIReader.hpp"

#include <cstdint>
#include <string>

namespace EtherCAT {

class Master;

/// Find a SyncManagerEntry by name substring (e.g. "MBoxOut", "MBoxIn",
/// "Outputs", "Inputs").
inline const ESI::SyncManagerEntry* findSmByName(const ESI::DeviceInfo& dev,
                                               const char* needle) {
    for (const auto& sm : dev.syncManagers) {
        if (sm.name.find(needle) != std::string::npos) return &sm;
    }
    return nullptr;
}

/// Read SII identity for matching; returns {vendorId, productCode} or {0,0}.
struct ESIIdentityMatch {
    uint32_t vendorId{0};
    uint32_t productCode{0};
};
inline ESIIdentityMatch readIdentityForESIMatch(EtherCAT::Master& master,
                                              uint16_t idx) {
    ESIIdentityMatch m;
    EtherCAT::SII::SIIIdentity id;
    if (EtherCAT::SII::readSIIIdentity(master, idx, id)) {
        m.vendorId = id.vendor_id;
        m.productCode = id.product_code;
    }
    return m;
}

} // namespace EtherCAT

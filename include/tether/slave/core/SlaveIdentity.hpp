// SPDX-License-Identifier: MIT
#pragma once

#include <cstdint>
#include <string>

namespace EtherCAT { namespace slave {

struct SlaveIdentity {
    uint32_t vendorId = 0x00000000;
    uint32_t productCode = 0x00000000;
    uint32_t revisionNumber = 0x00000000;
    uint32_t serialNumber = 0x00000000;

    std::string deviceName = "EtherCAT Slave";
    std::string hwVersion = "1.0";
    std::string swVersion = "1.0";
};

}} // namespace EtherCAT::slave

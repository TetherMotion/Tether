// SPDX-License-Identifier: MIT
#pragma once

#include <cstdint>
#include <vector>
#include <bit>

#include "tether/ethercat/SIIRegisters.hpp"

namespace EtherCAT { namespace slave {

namespace SIIControl {
    constexpr uint16_t ReadOperation    = 0x0100;
    constexpr uint16_t WriteOperation   = 0x0200;
    constexpr uint16_t ReloadOperation  = 0x0400;
    constexpr uint16_t CRCError         = 0x0800;
    constexpr uint16_t LoadingError     = 0x1000;
    constexpr uint16_t AckError         = 0x2000;
    constexpr uint16_t WriteError       = 0x4000;
    constexpr uint16_t Busy             = 0x8000;
}

struct SIIState {
    uint16_t config = 0;
    EtherCAT::SII::SIIControlReg control{};
    uint32_t address = 0;
    uint8_t  data[8] = {0};

    std::vector<uint8_t> eepromData;

    bool isBusy() const { return control.busy; }
    bool isReadOperation() const { return control.read_op; }
    bool isWriteOperation() const { return control.write_op; }
};

}} // namespace EtherCAT::slave

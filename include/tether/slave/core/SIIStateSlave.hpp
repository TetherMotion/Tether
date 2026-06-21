// SPDX-License-Identifier: MIT
#pragma once

#include <cstdint>
#include <vector>
#include <bit>

#include "tether/ethercat/SIIRegisters.hpp"

namespace EtherCAT { namespace slave {

namespace SIIControl {
    constexpr uint16_t ReadOperation    = 0x0001;
    constexpr uint16_t WriteOperation   = 0x0002;
    constexpr uint16_t ReloadOperation  = 0x0004;
    constexpr uint16_t CRCError         = 0x0008;
    constexpr uint16_t LoadingError     = 0x0010;
    constexpr uint16_t AckError         = 0x0020;
    constexpr uint16_t WriteError       = 0x0040;
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

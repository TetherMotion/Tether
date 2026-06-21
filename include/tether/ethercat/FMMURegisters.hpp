// SPDX-License-Identifier: MIT
#pragma once

#include <cstdint>
#include <bit>

namespace EtherCAT { namespace FMMU {

enum class FMMUType : uint8_t {
    Unused   = 0x00,
    Output   = 0x01,
    Input    = 0x02,
    MboxSync = 0x03,
};

struct FMMUTypeReg {
    uint8_t read_enable  : 1;
    uint8_t write_enable : 1;
    uint8_t reserved     : 6;

    bool operator==(const FMMUTypeReg& o) const {
        return std::bit_cast<uint8_t>(*this) == std::bit_cast<uint8_t>(o);
    }
    bool operator!=(const FMMUTypeReg& o) const { return !(*this == o); }
};
static_assert(sizeof(FMMUTypeReg) == 1, "FMMUTypeReg must be 1 byte");

struct FMMUActivateReg {
    uint8_t enable   : 1;
    uint8_t reserved : 7;

    bool operator==(const FMMUActivateReg& o) const {
        return std::bit_cast<uint8_t>(*this) == std::bit_cast<uint8_t>(o);
    }
    bool operator!=(const FMMUActivateReg& o) const { return !(*this == o); }
};
static_assert(sizeof(FMMUActivateReg) == 1, "FMMUActivateReg must be 1 byte");

}} // namespace EtherCAT::FMMU

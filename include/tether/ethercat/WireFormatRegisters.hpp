// SPDX-License-Identifier: MIT
#pragma once

#include <cstdint>
#include <bit>

namespace EtherCAT { namespace Wire {

struct FrameHeaderReg {
    uint16_t length   : 11;
    uint16_t reserved : 1;
    uint16_t type     : 4;

    bool operator==(const FrameHeaderReg& o) const {
        return std::bit_cast<uint16_t>(*this) == std::bit_cast<uint16_t>(o);
    }
    bool operator!=(const FrameHeaderReg& o) const { return !(*this == o); }
};
static_assert(sizeof(FrameHeaderReg) == 2, "FrameHeaderReg must be 2 bytes");

struct DatagramLenFlagsReg {
    uint16_t length     : 11;
    uint16_t reserved   : 3;
    uint16_t circulating : 1;
    uint16_t more       : 1;

    bool operator==(const DatagramLenFlagsReg& o) const {
        return std::bit_cast<uint16_t>(*this) == std::bit_cast<uint16_t>(o);
    }
    bool operator!=(const DatagramLenFlagsReg& o) const { return !(*this == o); }
};
static_assert(sizeof(DatagramLenFlagsReg) == 2, "DatagramLenFlagsReg must be 2 bytes");

struct CoeHeaderReg {
    uint16_t number   : 9;
    uint16_t reserved : 3;
    uint16_t service  : 4;

    bool operator==(const CoeHeaderReg& o) const {
        return std::bit_cast<uint16_t>(*this) == std::bit_cast<uint16_t>(o);
    }
    bool operator!=(const CoeHeaderReg& o) const { return !(*this == o); }
};
static_assert(sizeof(CoeHeaderReg) == 2, "CoeHeaderReg must be 2 bytes");

struct MbxTypeReg {
    uint8_t type     : 4;
    uint8_t counter  : 3;
    uint8_t reserved : 1;

    bool operator==(const MbxTypeReg& o) const {
        return std::bit_cast<uint8_t>(*this) == std::bit_cast<uint8_t>(o);
    }
    bool operator!=(const MbxTypeReg& o) const { return !(*this == o); }
};
static_assert(sizeof(MbxTypeReg) == 1, "MbxTypeReg must be 1 byte");

}} // namespace EtherCAT::Wire

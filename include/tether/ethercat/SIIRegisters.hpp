// SPDX-License-Identifier: MIT
#pragma once

#include <cstdint>
#include <bit>

namespace EtherCAT { namespace SII {

struct SIIControlReg {
    uint16_t reserved_0      : 8;
    uint16_t read_op         : 1;  // bit 8  (0x0100)
    uint16_t write_op        : 1;  // bit 9  (0x0200)
    uint16_t reload_op       : 1;  // bit 10 (0x0400)
    uint16_t crc_error       : 1;  // bit 11 (0x0800)
    uint16_t loading_error   : 1;  // bit 12 (0x1000)
    uint16_t ack_error       : 1;  // bit 13 (0x2000)
    uint16_t write_error     : 1;  // bit 14 (0x4000)
    uint16_t busy            : 1;  // bit 15 (0x8000)

    bool operator==(const SIIControlReg& o) const {
        return std::bit_cast<uint16_t>(*this) == std::bit_cast<uint16_t>(o);
    }
    bool operator!=(const SIIControlReg& o) const { return !(*this == o); }
};
static_assert(sizeof(SIIControlReg) == 2, "SIIControlReg must be 2 bytes");

}} // namespace EtherCAT::SII

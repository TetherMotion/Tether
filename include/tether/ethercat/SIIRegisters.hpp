// SPDX-License-Identifier: MIT
#pragma once

#include <cstdint>
#include <bit>

namespace EtherCAT { namespace SII {

struct SIIControlReg {
    uint16_t read_op       : 1;
    uint16_t write_op      : 1;
    uint16_t reload_op     : 1;
    uint16_t crc_error     : 1;
    uint16_t loading_error : 1;
    uint16_t ack_error     : 1;
    uint16_t write_error   : 1;
    uint16_t reserved_7    : 8;
    uint16_t busy          : 1;

    bool operator==(const SIIControlReg& o) const {
        return std::bit_cast<uint16_t>(*this) == std::bit_cast<uint16_t>(o);
    }
    bool operator!=(const SIIControlReg& o) const { return !(*this == o); }
};
static_assert(sizeof(SIIControlReg) == 2, "SIIControlReg must be 2 bytes");

}} // namespace EtherCAT::SII

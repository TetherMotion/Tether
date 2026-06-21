// SPDX-License-Identifier: MIT
#pragma once

#include <cstdint>
#include <bit>

namespace EtherCAT { namespace AL {

enum class SlaveState : uint8_t {
    INIT    = 0x01,
    PRE_OP  = 0x02,
    BOOT    = 0x03,
    SAFE_OP = 0x04,
    OP      = 0x08,
};

inline const char* slaveStateToString(SlaveState state) {
    switch (state) {
        case SlaveState::INIT:    return "INIT";
        case SlaveState::PRE_OP:  return "PRE-OP";
        case SlaveState::BOOT:    return "BOOT";
        case SlaveState::SAFE_OP: return "SAFE-OP";
        case SlaveState::OP:      return "OP";
        default: return "UNKNOWN";
    }
}

struct ALStatusReg {
    uint16_t state         : 4;
    uint16_t error         : 1;
    uint16_t id_request    : 1;
    uint16_t reserved      : 10;

    bool operator==(const ALStatusReg& o) const {
        return std::bit_cast<uint16_t>(*this) == std::bit_cast<uint16_t>(o);
    }
    bool operator!=(const ALStatusReg& o) const { return !(*this == o); }
};
static_assert(sizeof(ALStatusReg) == 2, "ALStatusReg must be 2 bytes");

struct ALControlReg {
    uint16_t requested_state : 4;
    uint16_t ack_error       : 1;
    uint16_t id_request      : 1;
    uint16_t reserved        : 10;

    bool operator==(const ALControlReg& o) const {
        return std::bit_cast<uint16_t>(*this) == std::bit_cast<uint16_t>(o);
    }
    bool operator!=(const ALControlReg& o) const { return !(*this == o); }
};
static_assert(sizeof(ALControlReg) == 2, "ALControlReg must be 2 bytes");

}} // namespace EtherCAT::AL

// SPDX-License-Identifier: MIT
#pragma once

#include <cstdint>
#include <bit>

namespace EtherCAT { namespace DC {

struct DCSyncActReg8 {
    uint8_t enable         : 1;
    uint8_t sync0_enable   : 1;
    uint8_t sync1_enable   : 1;
    uint8_t auto_activate  : 1;
    uint8_t reserved       : 4;

    bool operator==(const DCSyncActReg8& o) const {
        return std::bit_cast<uint8_t>(*this) == std::bit_cast<uint8_t>(o);
    }
    bool operator!=(const DCSyncActReg8& o) const { return !(*this == o); }
};
static_assert(sizeof(DCSyncActReg8) == 1, "DCSyncActReg8 must be 1 byte");

struct DCSyncActReg16 {
    uint16_t cyclic_operation : 1;
    uint16_t reserved_1       : 7;
    uint16_t sync0_enable     : 1;
    uint16_t sync1_enable     : 1;
    uint16_t sync0_start_time : 1;
    uint16_t reserved_11      : 5;

    bool operator==(const DCSyncActReg16& o) const {
        return std::bit_cast<uint16_t>(*this) == std::bit_cast<uint16_t>(o);
    }
    bool operator!=(const DCSyncActReg16& o) const { return !(*this == o); }
};
static_assert(sizeof(DCSyncActReg16) == 2, "DCSyncActReg16 must be 2 bytes");

struct DCLatchCtrlReg {
    uint8_t latch0_pos : 1;
    uint8_t latch0_neg : 1;
    uint8_t latch1_pos : 1;
    uint8_t latch1_neg : 1;
    uint8_t reserved   : 4;

    bool operator==(const DCLatchCtrlReg& o) const {
        return std::bit_cast<uint8_t>(*this) == std::bit_cast<uint8_t>(o);
    }
    bool operator!=(const DCLatchCtrlReg& o) const { return !(*this == o); }
};
static_assert(sizeof(DCLatchCtrlReg) == 1, "DCLatchCtrlReg must be 1 byte");

struct DCLatchStatusReg {
    uint8_t latch0_pos_valid : 1;
    uint8_t latch0_neg_valid : 1;
    uint8_t latch1_pos_valid : 1;
    uint8_t latch1_neg_valid : 1;
    uint8_t reserved         : 4;

    bool operator==(const DCLatchStatusReg& o) const {
        return std::bit_cast<uint8_t>(*this) == std::bit_cast<uint8_t>(o);
    }
    bool operator!=(const DCLatchStatusReg& o) const { return !(*this == o); }
};
static_assert(sizeof(DCLatchStatusReg) == 1, "DCLatchStatusReg must be 1 byte");

}} // namespace EtherCAT::DC

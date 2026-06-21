// SPDX-License-Identifier: MIT
#pragma once

#include <cstdint>
#include <bit>

namespace EtherCAT { namespace SyncManager {

enum class SMMode : uint8_t {
    Buffered = 0x00,
    Mailbox  = 0x02,
    ThreePDO = 0x03,
};

struct SMControlReg {
    uint8_t mode       : 2;
    uint8_t direction  : 1;
    uint8_t ecat_irq   : 1;
    uint8_t pdi_irq    : 1;
    uint8_t watchdog   : 1;
    uint8_t repeat_req : 1;
    uint8_t reserved   : 1;

    bool operator==(const SMControlReg& o) const {
        return std::bit_cast<uint8_t>(*this) == std::bit_cast<uint8_t>(o);
    }
    bool operator!=(const SMControlReg& o) const { return !(*this == o); }
};
static_assert(sizeof(SMControlReg) == 1, "SMControlReg must be 1 byte");

struct SMStatusReg {
    uint8_t write_event    : 1;
    uint8_t read_event     : 1;
    uint8_t reserved_2     : 1;
    uint8_t mailbox_full   : 1;
    uint8_t reserved_4     : 1;
    uint8_t reserved_5     : 1;
    uint8_t read_buf_full  : 1;
    uint8_t write_buf_full : 1;

    bool operator==(const SMStatusReg& o) const {
        return std::bit_cast<uint8_t>(*this) == std::bit_cast<uint8_t>(o);
    }
    bool operator!=(const SMStatusReg& o) const { return !(*this == o); }
};
static_assert(sizeof(SMStatusReg) == 1, "SMStatusReg must be 1 byte");

struct SMActivateReg {
    uint8_t enable   : 1;
    uint8_t reserved : 7;

    bool operator==(const SMActivateReg& o) const {
        return std::bit_cast<uint8_t>(*this) == std::bit_cast<uint8_t>(o);
    }
    bool operator!=(const SMActivateReg& o) const { return !(*this == o); }
};
static_assert(sizeof(SMActivateReg) == 1, "SMActivateReg must be 1 byte");

struct SMPDICtrlReg {
    uint8_t pdi_event  : 1;
    uint8_t repeat_req : 1;
    uint8_t reserved   : 6;

    bool operator==(const SMPDICtrlReg& o) const {
        return std::bit_cast<uint8_t>(*this) == std::bit_cast<uint8_t>(o);
    }
    bool operator!=(const SMPDICtrlReg& o) const { return !(*this == o); }
};
static_assert(sizeof(SMPDICtrlReg) == 1, "SMPDICtrlReg must be 1 byte");

}} // namespace EtherCAT::SyncManager

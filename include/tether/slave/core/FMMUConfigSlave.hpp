// SPDX-License-Identifier: MIT
#pragma once

#include <cstdint>
#include <cstddef>
#include <bit>

#include "tether/ethercat/FMMURegisters.hpp"

namespace EtherCAT { namespace slave {

struct FMMUConfig {
    uint32_t logicalStartAddr = 0;
    uint16_t length = 0;
    uint8_t  logicalStartBit = 0;
    uint8_t  logicalEndBit = 7;
    uint16_t physicalStartAddr = 0;
    uint8_t  physicalStartBit = 0;
    EtherCAT::FMMU::FMMUTypeReg   type{};
    EtherCAT::FMMU::FMMUActivateReg activate{};
    uint8_t  reserved[3] = {0};

    bool isEnabled() const { return activate.enable; }
    bool isReadEnabled() const { return type.read_enable; }
    bool isWriteEnabled() const { return type.write_enable; }

    void setEnabled(bool enable) {
        activate.enable = enable;
    }

    bool containsLogicalAddress(uint32_t addr, uint16_t len) const {
        if (!isEnabled()) return false;
        return addr < (logicalStartAddr + length) && (addr + len) > logicalStartAddr;
    }

    uint16_t translateToPhysical(uint32_t logicalAddr) const {
        return physicalStartAddr + (logicalAddr - logicalStartAddr);
    }

    void toBytes(uint8_t* out) const {
        out[0] = logicalStartAddr & 0xFF;
        out[1] = (logicalStartAddr >> 8) & 0xFF;
        out[2] = (logicalStartAddr >> 16) & 0xFF;
        out[3] = (logicalStartAddr >> 24) & 0xFF;
        out[4] = length & 0xFF;
        out[5] = (length >> 8) & 0xFF;
        out[6] = logicalStartBit;
        out[7] = logicalEndBit;
        out[8] = physicalStartAddr & 0xFF;
        out[9] = (physicalStartAddr >> 8) & 0xFF;
        out[10] = physicalStartBit;
        out[11] = std::bit_cast<uint8_t>(type);
        out[12] = std::bit_cast<uint8_t>(activate);
        out[13] = 0; out[14] = 0; out[15] = 0;
    }

    void fromBytes(const uint8_t* in) {
        logicalStartAddr = in[0] | (in[1] << 8) | (in[2] << 16) | (in[3] << 24);
        length = in[4] | (in[5] << 8);
        logicalStartBit = in[6];
        logicalEndBit = in[7];
        physicalStartAddr = in[8] | (in[9] << 8);
        physicalStartBit = in[10];
        type = std::bit_cast<EtherCAT::FMMU::FMMUTypeReg>(in[11]);
        activate = std::bit_cast<EtherCAT::FMMU::FMMUActivateReg>(in[12]);
    }
};

}} // namespace EtherCAT::slave

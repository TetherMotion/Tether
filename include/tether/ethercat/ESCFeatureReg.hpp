// SPDX-License-Identifier: MIT
#pragma once

#include <cstdint>
#include <bit>

namespace EtherCAT { namespace ESC {

struct ESCFeatureReg {
    uint16_t fmmu           : 1;
    uint16_t sync_manager   : 1;
    uint16_t dc             : 1;
    uint16_t dc_width64     : 1;
    uint16_t low_jitter     : 1;
    uint16_t enhanced_link  : 1;
    uint16_t dc_enhanced    : 1;
    uint16_t fmmu_ex_fcs    : 1;
    uint16_t enhanced_sm    : 1;
    uint16_t reserved       : 7;

    bool operator==(const ESCFeatureReg& o) const {
        return std::bit_cast<uint16_t>(*this) == std::bit_cast<uint16_t>(o);
    }
    bool operator!=(const ESCFeatureReg& o) const { return !(*this == o); }
};
static_assert(sizeof(ESCFeatureReg) == 2, "ESCFeatureReg must be 2 bytes");

}} // namespace EtherCAT::ESC

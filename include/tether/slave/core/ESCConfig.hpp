// SPDX-License-Identifier: MIT
#pragma once

#include <cstdint>
#include <bit>

#include "tether/ethercat/ESCFeatureReg.hpp"

namespace EtherCAT { namespace slave {

namespace ESCFeature {
    constexpr uint16_t FMMU           = 0x0001;
    constexpr uint16_t SyncManager    = 0x0002;
    constexpr uint16_t DC             = 0x0004;
    constexpr uint16_t DCWidth64      = 0x0008;
    constexpr uint16_t LowJitter      = 0x0010;
    constexpr uint16_t EnhancedLink   = 0x0020;
    constexpr uint16_t DCEnhanced     = 0x0040;
    constexpr uint16_t FMMUExFCS      = 0x0080;
    constexpr uint16_t EnhancedSM     = 0x0100;
}

struct ESCConfig {
    uint8_t  type = 0x02;
    uint8_t  revision = 0x01;
    uint16_t build = 0x0001;
    uint8_t  fmmuCount = 8;
    uint8_t  smCount = 8;
    uint8_t  ramSizeKB = 8;
    uint8_t  portDescriptor = 0x04;
    EtherCAT::ESC::ESCFeatureReg features = std::bit_cast<EtherCAT::ESC::ESCFeatureReg>(
        static_cast<uint16_t>(ESCFeature::FMMU | ESCFeature::SyncManager | ESCFeature::DC));

    uint32_t processDataRamSize() const { return static_cast<uint32_t>(ramSizeKB) * 1024; }
};

}} // namespace EtherCAT::slave

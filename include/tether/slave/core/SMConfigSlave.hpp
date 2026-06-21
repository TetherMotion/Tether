// SPDX-License-Identifier: MIT
#pragma once

#include <cstdint>
#include <bit>

#include "tether/ethercat/SMRegisters.hpp"

namespace EtherCAT { namespace slave {

enum class SyncManagerType : uint8_t {
    Unused       = 0x00,
    MailboxOut   = 0x01,
    MailboxIn    = 0x02,
    ProcessOut   = 0x03,
    ProcessIn    = 0x04,
};

namespace SMControl {
    constexpr uint8_t OperationMode  = 0x03;
    constexpr uint8_t MailboxMode    = 0x02;
    constexpr uint8_t Direction      = 0x04;
    constexpr uint8_t IntECAT        = 0x08;
    constexpr uint8_t IntPDI         = 0x10;
    constexpr uint8_t WatchdogEnable = 0x20;
}

namespace SMStatus {
    constexpr uint8_t IntWrite       = 0x01;
    constexpr uint8_t IntRead        = 0x02;
    constexpr uint8_t MailboxStatus  = 0x08;
    constexpr uint8_t BufferedState  = 0x30;
}

struct SyncManagerConfig {
    uint16_t physicalAddr = 0;
    uint16_t length = 0;
    EtherCAT::SyncManager::SMControlReg  control{};
    EtherCAT::SyncManager::SMStatusReg   status{};
    EtherCAT::SyncManager::SMActivateReg activate{};
    EtherCAT::SyncManager::SMPDICtrlReg  pdoDisable{};

    SyncManagerType type = SyncManagerType::Unused;

    bool isEnabled() const { return activate.enable; }
    bool isMailbox() const { return control.mode == static_cast<uint8_t>(EtherCAT::SyncManager::SMMode::Mailbox); }
    bool isBuffered() const { return control.mode == static_cast<uint8_t>(EtherCAT::SyncManager::SMMode::Buffered); }
    bool isDirectionWrite() const { return control.direction; }
    bool isDirectionRead() const { return !control.direction; }
    bool watchdogEnabled() const { return control.watchdog; }

    void setEnabled(bool enable) {
        activate.enable = enable;
    }

    void toBytes(uint8_t* out) const {
        out[0] = physicalAddr & 0xFF;
        out[1] = (physicalAddr >> 8) & 0xFF;
        out[2] = length & 0xFF;
        out[3] = (length >> 8) & 0xFF;
        out[4] = std::bit_cast<uint8_t>(control);
        out[5] = std::bit_cast<uint8_t>(status);
        out[6] = std::bit_cast<uint8_t>(activate);
        out[7] = std::bit_cast<uint8_t>(pdoDisable);
    }

    void fromBytes(const uint8_t* in) {
        physicalAddr = in[0] | (in[1] << 8);
        length = in[2] | (in[3] << 8);
        control = std::bit_cast<EtherCAT::SyncManager::SMControlReg>(in[4]);
        status = std::bit_cast<EtherCAT::SyncManager::SMStatusReg>(in[5]);
        activate = std::bit_cast<EtherCAT::SyncManager::SMActivateReg>(in[6]);
        pdoDisable = std::bit_cast<EtherCAT::SyncManager::SMPDICtrlReg>(in[7]);
    }
};

}} // namespace EtherCAT::slave

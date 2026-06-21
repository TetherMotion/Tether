// SPDX-License-Identifier: MIT
#pragma once

#include <cstdint>
#include <bit>

#include "tether/ethercat/ALRegisters.hpp"

namespace EtherCAT { namespace slave {

using EtherCAT::AL::SlaveState;
using EtherCAT::AL::slaveStateToString;

struct ALStatus {
    SlaveState state = SlaveState::INIT;
    bool error = false;
    bool identificationRequest = false;

    uint16_t toRegister() const {
        EtherCAT::AL::ALStatusReg reg{};
        reg.state = static_cast<uint16_t>(state);
        reg.error = error;
        reg.id_request = identificationRequest;
        return std::bit_cast<uint16_t>(reg);
    }

    static ALStatus fromRegister(uint16_t raw) {
        auto reg = std::bit_cast<EtherCAT::AL::ALStatusReg>(raw);
        ALStatus s;
        s.state = static_cast<SlaveState>(reg.state);
        s.error = reg.error;
        s.identificationRequest = reg.id_request;
        return s;
    }
};

struct ALControl {
    SlaveState requestedState = SlaveState::INIT;
    bool acknowledgeError = false;
    bool identificationRequest = false;

    uint16_t toRegister() const {
        EtherCAT::AL::ALControlReg reg{};
        reg.requested_state = static_cast<uint16_t>(requestedState);
        reg.ack_error = acknowledgeError;
        reg.id_request = identificationRequest;
        return std::bit_cast<uint16_t>(reg);
    }

    static ALControl fromRegister(uint16_t raw) {
        auto reg = std::bit_cast<EtherCAT::AL::ALControlReg>(raw);
        ALControl c;
        c.requestedState = static_cast<SlaveState>(reg.requested_state);
        c.acknowledgeError = reg.ack_error;
        c.identificationRequest = reg.id_request;
        return c;
    }
};

enum class ALStatusCode : uint16_t {
    NoError                     = 0x0000,
    UnspecifiedError            = 0x0001,
    NoMemory                    = 0x0002,
    InvalidStateChange          = 0x0011,
    UnknownRequestedState       = 0x0012,
    BootstrapNotSupported       = 0x0013,
    NoValidFirmware             = 0x0014,
    InvalidMailboxConfig        = 0x0016,
    InvalidMailboxConfig2       = 0x0017,
    InvalidSyncManagerConfig    = 0x0018,
    NoValidInputs               = 0x0019,
    NoValidOutputs              = 0x001A,
    SynchronizationError        = 0x001B,
    SyncManagerWatchdog         = 0x001C,
    InvalidSyncTypes            = 0x001D,
    InvalidOutputConfig         = 0x001E,
    InvalidInputConfig          = 0x001F,
    InvalidWatchdogConfig       = 0x0020,
    SlaveNeedsColdStart         = 0x0021,
    SlaveNeedsInit              = 0x0022,
    SlaveNeedsPreOp             = 0x0023,
    SlaveNeedsSafeOp            = 0x0024,
    InvalidInputMapping         = 0x0025,
    InvalidOutputMapping        = 0x0026,
    InconsistentSettings        = 0x0027,
    FreeRunNotSupported         = 0x0028,
    SyncNotSupported            = 0x0029,
    FreeRunNeeds3Buffer         = 0x002A,
    BackgroundWatchdog          = 0x002B,
    NoValidInputsOutputs        = 0x002C,
    FatalSyncError              = 0x002D,
    NoSyncError                 = 0x002E,
    InvalidDCSync               = 0x0030,
    InvalidDCLatch              = 0x0031,
    PLLError                    = 0x0032,
    DCSync0Missing              = 0x0033,
    DCSync1Missing              = 0x0034,
    DCSyncIOError               = 0x0035,
    ApplicationControllerAvail  = 0x0050,
    InvalidOutputData           = 0x0060,
    InvalidInputData            = 0x0061,
};

}} // namespace EtherCAT::slave

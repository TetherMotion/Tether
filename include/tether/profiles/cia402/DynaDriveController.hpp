// SPDX-License-Identifier: MIT
#pragma once

/**
 * @file DynaDriveController.hpp
 * @brief DynaDrive custom FSM controller extracted from CiA402Drive
 *
 * @details
 * Encapsulates the DynaDrive (rsl_drive_sdk / ANYdrive) custom finite-state
 * machine protocol that rides on top of the standard CiA 402 statusword /
 * controlword SDO interface.  The DynaDrive FSM has its own state enum
 * (Standby, MotorOp, ControlOp, etc.) and transition controlwords that
 * differ from the standard CiA 402 state machine.
 */

#include "tether/profiles/cia402/CiA402Drive.hpp" // DynaDriveState typedef
#include "tether/ethercat/Master.hpp"

#include <cstdint>

namespace EtherCAT {

class DynaDriveController {
public:
    DynaDriveController(Master& master, uint16_t slave_index,
                        uint32_t sdo_timeout_ms = 1000);

    // --- State decode helpers (static, no instance state needed) ---

    using DynaDriveState = EtherCAT::Drives::Registers::DynaDrive::Status::StateOptions;

    static DynaDriveState decodeState(uint32_t statusword);
    static const char* getStateName(DynaDriveState state);

    // --- SDO-based status / control ---

    bool readStatusword(uint32_t& statusword);
    bool sendControlword(EtherCAT::Drives::Registers::DynaDrive::Controlword::Options controlword);

    // --- High-level FSM transitions ---

    /// @brief Enable DynaDrive: walk Standby -> MotorOp -> ControlOp.
    /// @param timeout_ms Total timeout for the entire sequence.
    /// @return true if ControlOp was reached.
    bool enable(uint32_t timeout_ms = 10000);

    /// @brief Disable DynaDrive: ControlOp -> Standby.
    /// @return true if the transition command was sent (or already not in ControlOp).
    bool disable();

    /// @brief Check if the DynaDrive is currently in ControlOp state.
    bool isControlOp();

private:
    Master&  master_;
    uint16_t slave_index_;
    uint32_t sdo_timeout_ms_;
};

} // namespace EtherCAT

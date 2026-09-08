/**
 * @file SynapticonDriveInitializer.hpp
 * @brief Shared initialization helper for Synapticon SOMANET drives
 *
 * Encapsulates the common EtherCAT initialization sequence used by all
 * Synapticon examples (NoFSOE, WithFSOE, synapticon_cst_fsoe):
 *   1. Reset to INIT (if not already)
 *   2. Configure mailbox (ESI values: 512B, 0x1000/0x1400, CoE|FoE)
 *   3. Transition to PRE_OP
 *   4. Configure PDOs + transition to OP (via MultiPDOAssignment)
 *   5. Enable drive (CiA 402 state machine)
 *   6. Disengage/engage brake (0x2004:7)
 *
 * FSoE-specific logic (MainInstance setup, Data state wait, STO/SBC-gated
 * brake release) is NOT included here — it belongs in the example because
 * it is tightly coupled to the FSoE state machine and application logic.
 *
 * Usage (non-FSoE):
 * @code
 *   EtherCAT::DS402Master master;
 *   // ... start host session, discover slaves ...
 *   EtherCAT::Drives::Synapticon::SynapticonDriveInitializer init(
 *       master, slave_idx, "my_example");
 *   if (!init.initStandard()) return 1;
 *   init.disengageBrake();
 *   // ... run motion loop ...
 *   init.engageBrake();
 * @endcode
 *
 * Usage (FSoE, custom PDO assignment):
 * @code
 *   EtherCAT::DS402Master master;
 *   // ... start host session, discover slaves ...
 *   EtherCAT::Drives::Synapticon::SynapticonDriveInitializer init(
 *       master, slave_idx, "my_fsoe_example");
 *   init.resetToInit();
 *   init.configureMailbox();
 *   init.transitionToPreOp();
 *   // ... optional: diagnostic SDO reads ...
 *   init.configurePDOsAndOp(
 *       EtherCAT::Drives::SynapticonPDO::makeCombinedPDOAssignment());
 *   // ... FSoE setup, motion loop, wait for Data ...
 *   init.enableDrive();
 *   // ... brake release gated on FSoE STO/SBC status ...
 * @endcode
 */

#pragma once

#include <cstdint>
#include <thread>

#include "tether/drives/Synapticon.hpp"
#include "tether/drives/Synapticon/BrakeControl.hpp"
#include "tether/drives/Synapticon/SynapticonPDO.hpp"
#include "tether/ethercat/ALResetController.hpp"
#include "tether/ethercat/CoEManager.hpp"
#include "tether/ethercat/Master.hpp"
#include "tether/ethercat/Slave.hpp"
#include "tether/ethercat/Types.hpp"
#include "tether/profiles/cia402/CiA402Drive.hpp"
#include "tether/profiles/cia402/DS402Master.hpp"

#include "logging/Logger.hpp"

namespace EtherCAT {
namespace Drives {
namespace Synapticon {

/**
 * @brief Shared initialization helper for Synapticon SOMANET drives.
 *
 * Wraps the common EtherCAT init sequence (reset → mailbox → PRE_OP →
 * PDO config → OP → enable → brake) around a DS402Master and slave index.
 * Each step can be called individually for custom flows, or use the
 * convenience methods (initStandard / initCombined) for the full sequence.
 */
class SynapticonDriveInitializer {
public:
    /**
     * @brief Construct the initializer.
     *
     * @param master     DS402Master (must be started and have discovered slaves)
     * @param slave_idx  0-based slave index
     * @param tag        Log tag (defaults to "SynapticonInit")
     */
    SynapticonDriveInitializer(DS402Master& master, uint16_t slave_idx,
                              const char* tag = "SynapticonInit")
        : master_(master), slave_idx_(slave_idx), tag_(tag) {}

    // ------------------------------------------------------------------
    // Individual steps
    // ------------------------------------------------------------------

    /// @brief Reset the slave to INIT if it's currently in a higher state.
    ///
    /// Reads the current AL state and, if not INIT, uses ALResetController
    /// to force a reset.  Waits 100ms after a successful reset for the
    /// slave to settle.
    /// @return true if the slave is in INIT (or was successfully reset).
    bool resetToInit() {
        uint8_t current_state = 0;
        if (!master_.ethercatMaster().readSlaveApplicationLayerState(
                slave_idx_, current_state)) {
            TETHER_LOGW(tag_, "Could not read AL state for slave {} — continuing anyway",
                        slave_idx_);
            return true;  // non-fatal
        }

        TETHER_LOGI(tag_, "Slave {} current AL state: 0x{:02X} ({})",
                    slave_idx_, current_state,
                    slaveStateToString(static_cast<SlaveState>(current_state)));

        if (current_state == static_cast<uint8_t>(SlaveState::INIT)) {
            return true;  // already in INIT
        }

        TETHER_LOGI(tag_, "Slave {} is not in INIT — resetting to INIT before configuration",
                    slave_idx_);

        ALResetController reset_ctrl(master_.ethercatMaster());
        const auto result = reset_ctrl.resetSlave(
            slave_idx_, static_cast<uint8_t>(SlaveState::INIT));

        if (!result.success) {
            TETHER_LOGE(tag_, "Slave {} reset to INIT FAILED ({}, {} iterations, "
                              "final AL_STATUS=0x{:04X}, AL_STATUS_CODE=0x{:04X})",
                        slave_idx_, result.message.c_str(), result.iterations_used,
                        result.final_al_status, result.final_al_status_code);
            return false;
        }

        TETHER_LOGI(tag_, "Slave {} reset to INIT OK ({}, {} iterations)",
                    slave_idx_, result.message.c_str(), result.iterations_used);

        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        return true;
    }

    /// @brief Configure the mailbox with SOMANET ESI values.
    ///
    /// Uses the constants from Synapticon.hpp:
    ///   SM0 (M→S): 0x1000, 512 bytes
    ///   SM1 (S→M): 0x1400, 512 bytes
    ///   Protocols: CoE | FoE (0x000C)
    ///
    /// @return true on success.
    bool configureMailbox() {
        TETHER_LOGI(tag_, "Configuring mailbox for slave {}...", slave_idx_);

        auto& slave = master_.ethercatMaster().slave(slave_idx_);
        const auto mb_err = slave.configureMailbox(
            {.address = kMailboxReadAddr,  .length = kMailboxReadSize},   // SM1 (S→M)
            {.address = kMailboxWriteAddr, .length = kMailboxWriteSize}, // SM0 (M→S)
            kMailboxProtocols);

        if (mb_err != SlaveError::Ok) {
            TETHER_LOGE(tag_, "Mailbox config failed: {}", slaveErrorToString(mb_err));
            return false;
        }

        TETHER_LOGI(tag_, "Mailbox configured: SM0(M->S)=0x{:04X}/{} SM1(S->M)=0x{:04X}/{} proto=0x{:04X}",
                    kMailboxWriteAddr, kMailboxWriteSize,
                    kMailboxReadAddr, kMailboxReadSize, kMailboxProtocols);
        return true;
    }

    /// @brief Transition the slave to PRE_OP.
    /// @return true on success.
    bool transitionToPreOp() {
        auto& slave = master_.ethercatMaster().slave(slave_idx_);
        const auto err = slave.transitionToPreOp();
        if (err != SlaveError::Ok) {
            TETHER_LOGE(tag_, "PRE-OP transition failed: {}", slaveErrorToString(err));
            return false;
        }
        TETHER_LOGI(tag_, "Slave {} transitioned to PRE_OP", slave_idx_);
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
        return true;
    }

    /// @brief Configure PDOs and transition to OP.
    ///
    /// Creates a CiA402Drive via ensureDrive(), sets the SDO timeout,
    /// and calls drive.transitionToOp(assignment) which handles:
    ///   - configureMultiPDOs (SM2/SM3 + 0x1C12/0x1C13 + FMMU)
    ///   - SAFE_OP transition
    ///   - OP transition
    ///
    /// @param assignment  Multi-PDO assignment (use makeStandardPDOAssignment(),
    ///                     makeCombinedPDOAssignment(), or a custom one)
    /// @return true on success.
    bool configurePDOsAndOp(const Slave::MultiPDOAssignment& assignment) {
        auto& drive = master_.ensureDrive(slave_idx_);
        drive.setSDOTimeout(kSdoTimeoutMs);

        if (!drive.transitionToOp(assignment)) {
            TETHER_LOGE(tag_, "PDO config / OP transition failed for slave {}", slave_idx_);
            return false;
        }

        TETHER_LOGI(tag_, "Slave {} configured PDOs and transitioned to OP", slave_idx_);
        return true;
    }

    /// @brief Enable the drive (CiA 402 state machine).
    ///
    /// Performs the standard CiA 402 enable sequence via SDO:
    ///   fault reset → shutdown → switch on → enable operation
    ///
    /// @param timeout_ms  Timeout for each state transition (default 5000)
    /// @return true on success.
    bool enableDrive(uint32_t timeout_ms = 5000) {
        if (!master_.enableDrive(slave_idx_, timeout_ms)) {
            TETHER_LOGE(tag_, "Drive enable failed for slave {}", slave_idx_);
            return false;
        }
        TETHER_LOGI(tag_, "Slave {} drive enabled", slave_idx_);
        return true;
    }

    /// @brief Disable the drive (CiA 402 state machine).
    /// @return true on success.
    bool disableDrive() {
        if (!master_.disableDrive(slave_idx_)) {
            TETHER_LOGW(tag_, "Drive disable failed for slave {}", slave_idx_);
            return false;
        }
        return true;
    }

    /// @brief Disengage (release) the brake via CoE (0x2004:7 = 2).
    /// @param timeout_ms  SDO timeout (default: kSdoTimeoutMs = 6000)
    /// @return true on success.
    bool disengageBrake(uint32_t timeout_ms = kSdoTimeoutMs) {
        TETHER_LOGI(tag_, "Disengaging brake via CoE (0x2004:7)...");
        auto& sdo = master_.ethercatMaster().sdoManager(slave_idx_);
        if (!BrakeControl::disengageBrake(sdo, timeout_ms)) {
            TETHER_LOGW(tag_, "Brake disengage failed or unverified");
            return false;
        }
        return true;
    }

    /// @brief Engage the brake via CoE (0x2004:7 = 1).
    /// @param timeout_ms  SDO timeout (default: kSdoTimeoutMs = 6000)
    /// @return true on success.
    bool engageBrake(uint32_t timeout_ms = kSdoTimeoutMs) {
        TETHER_LOGI(tag_, "Engaging brake via CoE (0x2004:7)...");
        auto& sdo = master_.ethercatMaster().sdoManager(slave_idx_);
        if (!BrakeControl::engageBrake(sdo, timeout_ms)) {
            TETHER_LOGW(tag_, "Brake engage failed or unverified");
            return false;
        }
        return true;
    }

    // ------------------------------------------------------------------
    // Convenience: full init sequences
    // ------------------------------------------------------------------

    /// @brief Full init for non-FSoE: reset → mailbox → PRE_OP → standard PDOs → OP.
    ///
    /// Uses makeStandardPDOAssignment() (all CiA 402 PDOs, no FSoE).
    /// Does NOT enable the drive or disengage the brake — call those
    /// separately after starting the motion loop.
    /// @return true on success.
    bool initStandard() {
        if (!resetToInit()) return false;
        if (!configureMailbox()) return false;
        if (!transitionToPreOp()) return false;
        if (!configurePDOsAndOp(SynapticonPDO::makeStandardPDOAssignment())) return false;
        return true;
    }

    /// @brief Full init for FSoE: reset → mailbox → PRE_OP → combined PDOs → OP.
    ///
    /// Uses makeCombinedPDOAssignment() (FSoE + CiA 402 PDOs, FSoE first).
    /// Does NOT enable the drive or disengage the brake — those are
    /// FSoE-gated and must be called after the FSoE state machine reaches Data.
    /// @return true on success.
    bool initCombined() {
        if (!resetToInit()) return false;
        if (!configureMailbox()) return false;
        if (!transitionToPreOp()) return false;
        if (!configurePDOsAndOp(SynapticonPDO::makeCombinedPDOAssignment())) return false;
        return true;
    }

    // ------------------------------------------------------------------
    // Accessors
    // ------------------------------------------------------------------

    /// @brief Get the CiA402Drive handle (created by configurePDOsAndOp).
    CiA402Drive& drive() { return master_.ensureDrive(slave_idx_); }

    /// @brief Get the raw EtherCAT Slave.
    EtherCAT::Slave& slave() { return master_.ethercatMaster().slave(slave_idx_); }

    /// @brief Get the CoE/SDO manager for this slave.
    CoE::CoEManager& sdo() { return master_.ethercatMaster().sdoManager(slave_idx_); }

    /// @brief Get the slave index.
    uint16_t slaveIndex() const { return slave_idx_; }

private:
    DS402Master& master_;
    uint16_t slave_idx_;
    const char* tag_;
};

} // namespace Synapticon
} // namespace Drives
} // namespace EtherCAT

/**
 * @file AS715NDriveInitializer.hpp
 * @brief Shared initialization helper for ANCTL AS715N drives
 *
 * Encapsulates the common EtherCAT initialization sequence used by the
 * AS715N native examples:
 *   1. Reset to INIT (if not already)
 *   2. Configure mailbox from SII
 *   3. Transition to PRE_OP
 *   4. Configure PDOs + transition to OP (via MultiPDOAssignment)
 *   5. Enable drive (CiA 402 state machine)
 *
 * Usage:
 * @code
 *   EtherCAT::DS402Master master;
 *   // ... start host session, discover slaves, start DC ...
 *   EtherCAT::Drives::AS715N::AS715NDriveInitializer init(master, slave_idx, "my_example");
 *   if (!init.init()) return 1;
 *   if (!init.drive().setOperatingMode(static_cast<int8_t>(CiA402::OperatingMode::CyclicSyncVelocity))) return 2;
 *   if (!init.enableDrive()) return 3;
 * @endcode
 */

#pragma once

#include <cstdint>
#include <chrono>
#include <thread>

#include "tether/drives/AS715N.hpp"
#include "tether/drives/AS715N/AS715NPDO.hpp"
#include "tether/ethercat/ALResetController.hpp"
#include "tether/ethercat/CoEManager.hpp"
#include "tether/ethercat/Master.hpp"
#include "tether/ethercat/Slave.hpp"
#include "tether/ethercat/Types.hpp"
#include "tether/platform/EspCompat.hpp"
#include "tether/profiles/cia402/CiA402Drive.hpp"
#include "tether/profiles/cia402/DS402Master.hpp"

#include "logging/Logger.hpp"

namespace EtherCAT {
namespace Drives {
namespace AS715N {

/**
 * @brief Shared initialization helper for ANCTL AS715N drives.
 *
 * Wraps the common EtherCAT init sequence (reset -> mailbox -> PRE_OP ->
 * PDO config -> OP -> enable) around a DS402Master and slave index.
 * Each step can be called individually for custom flows, or use the
 * convenience methods (init / initAndEnable) for the full sequence.
 */
class AS715NDriveInitializer {
public:
    /**
     * @brief Construct the initializer.
     *
     * @param master     DS402Master (must be started and have discovered slaves)
     * @param slave_idx  0-based slave index
     * @param tag        Log tag (defaults to "AS715NInit")
     */
    AS715NDriveInitializer(DS402Master& master, uint16_t slave_idx,
                          const char* tag = "AS715NInit")
        : master_(master), slave_idx_(slave_idx), tag_(tag) {}

    // ------------------------------------------------------------------
    // Individual steps
    // ------------------------------------------------------------------

    /// @brief Reset the slave to INIT if it is not already there.
    bool resetToInit() {
        uint8_t current_state = 0;
        if (!master_.ethercatMaster().readSlaveApplicationLayerState(slave_idx_, current_state)) {
            TETHER_LOGW(tag_, "Could not read AL state for slave {} -- continuing anyway",
                        slave_idx_);
            return true;  // non-fatal
        }

        TETHER_LOGI(tag_, "Slave {} current AL state: 0x{:02X} ({})",
                    slave_idx_, current_state,
                    slaveStateToString(static_cast<SlaveState>(current_state)));

        if (current_state == static_cast<uint8_t>(SlaveState::INIT)) {
            return true;
        }

        TETHER_LOGI(tag_, "Slave {} is not in INIT -- resetting to INIT before configuration",
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

    /// @brief Configure the mailbox (SM0/SM1) from SII automatically.
    bool configureMailbox() {
        TETHER_LOGI(tag_, "Configuring mailbox for slave {}...", slave_idx_);

        auto& slave = master_.ethercatMaster().slave(slave_idx_);
        const auto err = slave.configureMailbox(Tether::Platform::LogLevel::Info);
        if (err != SlaveError::Ok) {
            TETHER_LOGE(tag_, "Mailbox config failed: {}", slaveErrorToString(err));
            return false;
        }

        TETHER_LOGI(tag_, "Mailbox configured for slave {}", slave_idx_);
        return true;
    }

    /// @brief Transition the slave to PRE_OP.
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
    /// Uses CiA402Drive::transitionToOp(const Slave::MultiPDOAssignment&), which:
    ///   - ensures the slave is in PRE_OP
    ///   - calls Slave::configureMultiPDOs (SM2/SM3 + 0x1C12/0x1C13 + FMMU)
    ///   - registers the process-data buffers
    ///   - transitions SAFE_OP -> OP
    ///
    /// @param assignment  Multi-PDO assignment (use makeDefaultPDOAssignment()
    ///                    or a custom one)
    /// @return true on success
    bool configurePDOsAndOp(const Slave::MultiPDOAssignment& assignment) {
        auto& drive = master_.ensureDrive(slave_idx_);
        drive.setSDOTimeout(3000);

        if (!drive.transitionToOp(assignment)) {
            TETHER_LOGE(tag_, "PDO config / OP transition failed for slave {}", slave_idx_);
            return false;
        }

        const int opmode_offset = AS715N_pdo::opmodeOffsetFor(drive.getRxPDOIndex());
        if (opmode_offset >= 0) {
            drive.setOpmodePDOOffset(opmode_offset);
            TETHER_LOGI(tag_, "Slave {}: opmode offset set to {}", slave_idx_, opmode_offset);
        }

        const int statusword_offset = AS715N_pdo::statuswordOffsetFor(drive.getTxPDOIndex());
        if (statusword_offset >= 0) {
            drive.setStatuswordPDOOffset(statusword_offset);
            TETHER_LOGI(tag_, "Slave {}: statusword offset set to {}", slave_idx_, statusword_offset);
        }

        TETHER_LOGI(tag_, "Slave {} configured PDOs and transitioned to OP", slave_idx_);
        return true;
    }

    /// Which fault-reset procedure the fault handler runs for a given
    /// condition.  Installed via setFaultResetPolicy().
    enum class FaultResetAction : uint8_t {
        /// Do nothing — leave the fault latched (enable() keeps retrying
        /// until its timeout).
        None,
        /// Benign controlword FLTR (bit-7) toggle with Enable Operation
        /// kept asserted (0x0F -> 0x8F -> 0x0F).  The AS715N probably
        /// ignores it, but it is safe while the drive still runs.
        FltrToggle,
        /// CiA402 standard bit-7 edge on the current controlword.
        FltrStandard,
        /// AS715N F31.00 (0x2031:01) sequence — clears S-ON first per the
        /// A6-EC manual, so it briefly disables the drive.
        F31Sequence,
    };

    /// Policy controlling which reset procedure runs when.  The fault
    /// handler is invoked from CiA402Drive::enable()'s fault loop; the
    /// condition is evaluated on every invocation.
    struct FaultResetPolicy {
        /// Action for real manufacturer faults (0x203F external != 0).
        FaultResetAction mfr_error_action = FaultResetAction::F31Sequence;
        /// Action for phantom faults (statusword fault bit set but
        /// 0x203F = NoError) while the drive is not provably stuck.
        FaultResetAction phantom_action = FaultResetAction::FltrToggle;
        /// Action for a stuck phantom fault: fault bit set AND the drive
        /// disabled (Fault / FaultReactionActive / SwitchOnDisabled /
        /// NotReadyToSwitchOn) even though the enable controlword
        /// (0x000F) was commanded.
        FaultResetAction stuck_action = FaultResetAction::F31Sequence;
        /// Also treat a phantom fault as stuck after this many handler
        /// invocations (0 = disabled — only the statusword condition
        /// counts).
        uint32_t stuck_after_attempts = 0;
    };

    void setFaultResetPolicy(const FaultResetPolicy& policy) {
        fault_reset_policy_ = policy;
    }
    const FaultResetPolicy& faultResetPolicy() const { return fault_reset_policy_; }

    /// @brief Enable the drive (CiA 402 state machine).
    ///
    /// Installs the AS715N fault reset as the drive's fault-reset handler:
    /// the AS715N does NOT clear faults via the CiA402 controlword bit-7
    /// edge, so enable()'s fault loop would otherwise retry a no-op until
    /// timeout.  Which procedure runs for which condition is controlled
    /// by setFaultResetPolicy().
    /// @param timeout_ms  Timeout for each state transition (default 5000)
    /// @return true on success
    bool enableDrive(uint32_t timeout_ms = 5000) {
        fault_reset_attempts_ = 0;
        auto& drive = master_.ensureDrive(slave_idx_);
        drive.setFaultResetCallback([this](CiA402Drive& d) {
            return as715nFaultReset(d);
        });
        if (!master_.enableDrive(slave_idx_, timeout_ms)) {
            TETHER_LOGE(tag_, "Drive enable failed for slave {}", slave_idx_);
            return false;
        }
        TETHER_LOGI(tag_, "Slave {} drive enabled", slave_idx_);
        return true;
    }

    /// @brief Disable the drive.
    /// @return true on success
    bool disableDrive() {
        if (!master_.disableDrive(slave_idx_)) {
            TETHER_LOGW(tag_, "Drive disable failed for slave {}", slave_idx_);
            return false;
        }
        return true;
    }

    // ------------------------------------------------------------------
    // Convenience: full init sequences
    // ------------------------------------------------------------------

    /// @brief Full init: reset -> mailbox -> PRE_OP -> default PDOs -> OP.
    ///
    /// Uses makeDefaultPDOAssignment() (RxPDO 0x1705, TxPDO 0x1B04).
    /// Does NOT enable the drive -- call enableDrive() after setting the
    /// desired operating mode.
    /// @return true on success
    bool init() {
        return init(AS715N_pdo::makeDefaultPDOAssignment());
    }

    /// @brief Full init: reset -> mailbox -> PRE_OP -> custom PDOs -> OP.
    ///
    /// @param assignment  Multi-PDO assignment for this drive
    /// @return true on success
    bool init(const Slave::MultiPDOAssignment& assignment) {
        if (!resetToInit()) return false;
        if (!configureMailbox()) return false;
        if (!transitionToPreOp()) return false;
        if (!configurePDOsAndOp(assignment)) return false;
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
    /// Fault reset invoked from CiA402Drive::enable()'s fault loop.
    /// Evaluates the fault condition (manufacturer error vs. phantom vs.
    /// stuck phantom) and runs the procedure selected by the policy.
    bool as715nFaultReset(CiA402Drive& drive) {
        auto& sdo_mgr = sdo();
        const uint16_t mfr =
            AS715NFaultHandler::readManufacturerFault(sdo_mgr, slave_idx_);
        ++fault_reset_attempts_;

        if (mfr != 0) {
            return runFaultResetAction(fault_reset_policy_.mfr_error_action,
                                       drive, sdo_mgr);
        }

        // Phantom fault: statusword fault bit set but no manufacturer
        // error.  Stuck = drive disabled (faulted / not switchable on)
        // even though Enable Operation (0x000F) was commanded.
        const DriveState st = drive.getDriveState();
        const bool drive_disabled =
            st == DriveState::Fault || st == DriveState::FaultReactionActive ||
            st == DriveState::SwitchOnDisabled ||
            st == DriveState::NotReadyToSwitchOn;
        const bool enable_commanded =
            (drive.getControlword() & 0x000F) == 0x000F;
        const bool stuck =
            (drive_disabled && enable_commanded) ||
            (fault_reset_policy_.stuck_after_attempts != 0 &&
             fault_reset_attempts_ >= fault_reset_policy_.stuck_after_attempts);

        if (stuck) {
            TETHER_LOGW(tag_, "Slave {}: phantom fault stuck (state={} "
                              "sw=0x{:04X} cw=0x{:04X} attempts={})",
                        slave_idx_, static_cast<int>(st),
                        drive.getStatusword(), drive.getControlword(),
                        fault_reset_attempts_);
            return runFaultResetAction(fault_reset_policy_.stuck_action,
                                       drive, sdo_mgr);
        }
        return runFaultResetAction(fault_reset_policy_.phantom_action,
                                   drive, sdo_mgr);
    }

    bool runFaultResetAction(FaultResetAction action, CiA402Drive& drive,
                             CoE::CoEManager& sdo_mgr) {
        switch (action) {
            case FaultResetAction::None:
                return true;
            case FaultResetAction::FltrToggle:
                return drive.resetFaultKeepEnabled();
            case FaultResetAction::FltrStandard:
                return drive.resetFault();
            case FaultResetAction::F31Sequence:
                // Per the A6-EC manual the S-ON bit must be cleared before
                // the F31.00 0->1->0 sequence is accepted.
                if (auto cw = sdo_mgr.readU16(0x6040, 0x00, {.timeout_ms = 3000});
                    cw.has_value() && (*cw & 0x0001u)) {
                    (void)sdo_mgr.writeU16(
                        0x6040, 0x00,
                        static_cast<uint16_t>(*cw & ~0x0001u),
                        {.timeout_ms = 3000});
                    drive.setControlword(static_cast<uint16_t>(*cw & ~0x0001u));
                    Tether::Platform::Clock::instance().delayMilliseconds(50);
                }
                return AS715NFaultHandler::resetFault(sdo_mgr, slave_idx_);
        }
        return false;
    }

    uint32_t fault_reset_attempts_ = 0;
    FaultResetPolicy fault_reset_policy_;
    DS402Master& master_;
    uint16_t slave_idx_;
    const char* tag_;
};

} // namespace AS715N
} // namespace Drives
} // namespace EtherCAT

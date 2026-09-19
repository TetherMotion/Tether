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

    /// @brief Enable the drive (CiA 402 state machine).
    ///
    /// Installs the AS715N fault reset (F31.00 / 0x2031:01) as the drive's
    /// fault-reset handler: the AS715N does NOT clear faults via the CiA402
    /// controlword bit-7 edge, so enable()'s fault loop would otherwise
    /// retry a no-op until timeout.
    /// @param timeout_ms  Timeout for each state transition (default 5000)
    /// @return true on success
    bool enableDrive(uint32_t timeout_ms = 5000) {
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
    /// The F31.00 (0x2031:01) sequence only runs when the manufacturer
    /// error code (0x203F external) is nonzero — a statusword fault with
    /// 0x203F = NoError (e.g. a pure CiA402 fault like 0x603F=0x8700)
    /// falls back to the standard controlword bit-7 edge instead.
    bool as715nFaultReset(CiA402Drive& drive) {
        auto& sdo_mgr = sdo();
        const uint16_t mfr =
            AS715NFaultHandler::readManufacturerFault(sdo_mgr, slave_idx_);
        if (mfr == 0) {
            return drive.resetFault();
        }
        // Per the A6-EC manual the S-ON bit must be cleared before the
        // F31.00 0->1->0 sequence is accepted.
        auto cw = sdo_mgr.readU16(0x6040, 0x00, {.timeout_ms = 3000});
        if (cw.has_value() && (*cw & 0x0001u)) {
            (void)sdo_mgr.writeU16(0x6040, 0x00,
                                   static_cast<uint16_t>(*cw & ~0x0001u),
                                   {.timeout_ms = 3000});
            drive.setControlword(static_cast<uint16_t>(*cw & ~0x0001u));
            Tether::Platform::Clock::instance().delayMilliseconds(50);
        }
        return AS715NFaultHandler::resetFault(sdo_mgr, slave_idx_);
    }

    DS402Master& master_;
    uint16_t slave_idx_;
    const char* tag_;
};

} // namespace AS715N
} // namespace Drives
} // namespace EtherCAT

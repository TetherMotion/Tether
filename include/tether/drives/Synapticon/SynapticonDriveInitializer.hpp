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
#include <functional>
#include <span>
#include <thread>

#include "tether/drives/Synapticon.hpp"
#include "tether/drives/Synapticon/BrakeControl.hpp"
#include "tether/drives/Synapticon/SynapticonPDO.hpp"
#include "tether/ethercat/ALResetController.hpp"
#include "tether/ethercat/CoEManager.hpp"
#include "tether/ethercat/Master.hpp"
#include "tether/ethercat/Slave.hpp"
#include "tether/ethercat/SlaveGroup.hpp"
#include "tether/ethercat/Types.hpp"
#include "tether/profiles/cia301/CiA301Defs.hpp"
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
            TETHER_LOGW(tag_, "Could not read AL state for {} — continuing anyway",
                        master_.ethercatMaster().slaveLogPrefix(slave_idx_).c_str());
            return true;  // non-fatal
        }

        TETHER_LOGI(tag_, "{} current AL state: 0x{:02X} ({})",
                    master_.ethercatMaster().slaveLogPrefix(slave_idx_).c_str(), current_state,
                    slaveStateToString(static_cast<SlaveState>(current_state)));

        if (current_state == static_cast<uint8_t>(SlaveState::INIT)) {
            return true;  // already in INIT
        }

        TETHER_LOGI(tag_, "{} is not in INIT — resetting to INIT before configuration",
                    master_.ethercatMaster().slaveLogPrefix(slave_idx_).c_str());

        ALResetController reset_ctrl(master_.ethercatMaster());
        const auto result = reset_ctrl.resetSlave(
            slave_idx_, static_cast<uint8_t>(SlaveState::INIT));

        if (!result.success) {
            TETHER_LOGE(tag_, "{} reset to INIT FAILED ({}, {} iterations, "
                              "final AL_STATUS=0x{:04X}, AL_STATUS_CODE=0x{:04X})",
                        master_.ethercatMaster().slaveLogPrefix(slave_idx_).c_str(), result.message.c_str(), result.iterations_used,
                        result.final_al_status, result.final_al_status_code);
            return false;
        }

        TETHER_LOGI(tag_, "{} reset to INIT OK ({}, {} iterations)",
                    master_.ethercatMaster().slaveLogPrefix(slave_idx_).c_str(), result.message.c_str(), result.iterations_used);

        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        return true;
    }

    /// @brief Reset a set of initializers to INIT, one slave at a time.
    ///
    /// Per-slave fallback used when a group INIT request
    /// (EtherCAT::SlaveGroup::requestState) could not be confirmed —
    /// e.g. a member slave stuck with a latched AL error.
    /// @return true when every initializer's slave is in INIT.
    static bool resetAllToInit(std::span<SynapticonDriveInitializer> inits) {
        for (auto& ini : inits) {
            if (!ini.resetToInit()) return false;
        }
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
        TETHER_LOGI(tag_, "Configuring mailbox for {}...", master_.ethercatMaster().slaveLogPrefix(slave_idx_).c_str());

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
        TETHER_LOGI(tag_, "{} transitioned to PRE_OP", master_.ethercatMaster().slaveLogPrefix(slave_idx_).c_str());
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
        return true;
    }

    /// @brief Configure PDOs and bring the slave to SAFE_OP (no OP request).
    ///
    /// Same work as configurePDOsAndOp() — configureMultiPDOs (SM/FMMU +
    /// 0x1C12/0x1C13), PDO buffer registration, SAFE_OP transition, DC
    /// reconfig, PDO exchange enable — but stops short of requesting OP.
    /// Used for synchronised multi-drive bring-up: prepare every drive to
    /// SAFE_OP first, then issue a single group OP request (e.g. via
    /// EtherCAT::SlaveGroup).
    ///
    /// @param assignment  Multi-PDO assignment
    /// @return true on success.
    bool configurePDOsAndSafeOp(const Slave::MultiPDOAssignment& assignment) {
        auto& drive = master_.ensureDrive(slave_idx_);
        drive.setSDOTimeout(kSdoTimeoutMs);

        if (!drive.prepareForOp(assignment)) {
            TETHER_LOGE(tag_, "PDO config / SAFE_OP transition failed for {}", master_.ethercatMaster().slaveLogPrefix(slave_idx_).c_str());
            return false;
        }

        TETHER_LOGI(tag_, "{} configured PDOs and reached SAFE_OP (awaiting group OP request)", master_.ethercatMaster().slaveLogPrefix(slave_idx_).c_str());
        return true;
    }

    /// @brief Configure PDOs while staying in PRE_OP (no SAFE_OP request).
    ///
    /// Everything configurePDOsAndSafeOp() does before the SAFE_OP request:
    /// configureMultiPDOs (SM/FMMU + 0x1C12/0x1C13) and PDO buffer
    /// registration.  Used for synchronised multi-drive bring-up: run this
    /// per slave, issue the SAFE_OP request for the whole group in one
    /// packet (EtherCAT::SlaveGroup::requestState), then run
    /// postSafeOpPrepare() per slave before the group OP request.
    bool configurePDOsInPreOp(const Slave::MultiPDOAssignment& assignment) {
        auto& drive = master_.ensureDrive(slave_idx_);
        drive.setSDOTimeout(kSdoTimeoutMs);

        if (!drive.prepareForSafeOp(assignment)) {
            TETHER_LOGE(tag_, "PDO config failed for {}", master_.ethercatMaster().slaveLogPrefix(slave_idx_).c_str());
            return false;
        }

        TETHER_LOGI(tag_, "{} configured PDOs (staying in PRE_OP, awaiting group SAFE_OP request)", master_.ethercatMaster().slaveLogPrefix(slave_idx_).c_str());
        return true;
    }

    /// @brief Post-SAFE_OP OP preparation — DC reconfig, PDO exchange
    ///        enable, diagnostics, error-ack.  Call once the slave has
    ///        reached SAFE_OP (e.g. after a group requestState +
    ///        waitForState), before the (group) OP request.
    bool postSafeOpPrepare() {
        auto& drive = master_.ensureDrive(slave_idx_);
        if (!drive.postSafeOpForOp()) {
            TETHER_LOGE(tag_, "post-SAFE_OP preparation failed for {}", master_.ethercatMaster().slaveLogPrefix(slave_idx_).c_str());
            return false;
        }
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
            TETHER_LOGE(tag_, "PDO config / OP transition failed for {}", master_.ethercatMaster().slaveLogPrefix(slave_idx_).c_str());
            return false;
        }

        TETHER_LOGI(tag_, "{} configured PDOs and transitioned to OP", master_.ethercatMaster().slaveLogPrefix(slave_idx_).c_str());
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
            TETHER_LOGE(tag_, "Drive enable failed for {}", master_.ethercatMaster().slaveLogPrefix(slave_idx_).c_str());
            return false;
        }
        TETHER_LOGI(tag_, "{} drive enabled", master_.ethercatMaster().slaveLogPrefix(slave_idx_).c_str());
        return true;
    }

    /// @brief Disable the drive (CiA 402 state machine).
    /// @return true on success.
    bool disableDrive() {
        if (!master_.disableDrive(slave_idx_)) {
            TETHER_LOGW(tag_, "Drive disable failed for {}", master_.ethercatMaster().slaveLogPrefix(slave_idx_).c_str());
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
    // Group bring-up — all AL transitions as single-packet group requests
    // ------------------------------------------------------------------

    /**
     * @brief INIT + mailbox + PRE_OP for a whole drive group.
     *
     *   1. Group INIT request (one packet) with resetAllToInit() fallback.
     *   2. Per-slave mailbox configuration (ESC register writes).
     *   3. Group PRE_OP request (one packet).
     *   4. SlaveGroup::waitForMailboxReady() — no blind settle delay.
     *
     * Also prepares each drive's CiA402Drive handle with the SOMANET PDO
     * field offsets (controlword=0, statusword=0, opmode=2).  SDO
     * configuration should be applied next (e.g. via SlaveGroup
     * writeEntryAll) before bringGroupToSafeOp().
     *
     * @return true when every slave reached PRE_OP with a working mailbox.
     */
    static bool initGroupToPreOp(std::span<SynapticonDriveInitializer> inits,
                                 EtherCAT::SlaveGroup& group) {
        const char* tag = "SynapticonInit";

        if (group.requestState(SlaveState::INIT, /*ack_error=*/true) == 0 ||
            !group.waitForState(SlaveState::INIT, /*timeout_ms=*/3000,
                                /*poll_ms=*/10, /*resend_ms=*/500)) {
            TETHER_LOGW(tag, "Group INIT request failed — falling back to "
                             "per-slave reset");
            if (!resetAllToInit(inits)) return false;
        }

        for (auto& ini : inits) {
            if (!ini.configureMailbox()) return false;
        }

        if (group.requestState(SlaveState::PRE_OP, /*ack_error=*/true) == 0 ||
            !group.waitForState(SlaveState::PRE_OP, /*timeout_ms=*/3000,
                                /*poll_ms=*/10, /*resend_ms=*/500)) {
            TETHER_LOGE(tag, "Group PRE_OP transition failed");
            return false;
        }
        if (!group.waitForMailboxReady(2000, 25)) {
            TETHER_LOGE(tag, "Mailbox did not come up on all group members");
            return false;
        }

        // SOMANET standard PDO layout: controlword@0, statusword@0,
        // opmode@2 — applies to both the 0x1600/0x1A00 motion PDOs and the
        // combined FSoE assignment (motion region at offset 0 there).
        for (auto& ini : inits) {
            auto& drive = ini.drive();
            drive.setSDOTimeout(kSdoTimeoutMs);
            drive.setControlwordPDOOffset(0);
            drive.setStatuswordPDOOffset(0);
            drive.setOpmodePDOOffset(2);
        }

        // Firmware version (0x100A) — read now while the mailbox is
        // guaranteed to work (PRE_OP).  SOMANET >= 5.6 drops mailbox
        // service in SAFE_OP; the flag makes every mailbox-dependent
        // diagnostic (SDO liveness probe, updateIdentity, ...) skip the
        // guaranteed-timeout window.
        for (auto& ini : inits) {
            ini.readAndApplyFirmwareVersion();
        }
        return true;
    }

    /**
     * @brief PDO config + group SAFE_OP + post-SAFE_OP prep.
     *
     *   1. configurePDOsInPreOp(assignment) per slave (SM/FMMU + 0x1C12/
     *      0x1C13 + PDO buffer registration — slaves stay in PRE_OP).
     *   2. Group SAFE_OP request (one packet — the slaves validate the
     *      PDO assignment at the same instant).
     *   3. postSafeOpPrepare() per slave (DC SYNC reconfig, PDO exchange
     *      enable, SM readback, error-ack).
     *
     * Afterwards: register application PDO buffers (the entries replace the
     * CiA402Drive's own), then request OP via requestGroupOp().  A PDO
     * keep-alive exchange should be running from SAFE_OP onward — pass it
     * via @p on_safe_op, which is invoked immediately after the group
     * reaches SAFE_OP and before postSafeOpPrepare().
     *
     * @return true when every slave is ready for the OP request.
     */
    static bool bringGroupToSafeOp(
        std::span<SynapticonDriveInitializer> inits,
        const Slave::MultiPDOAssignment& assignment,
        EtherCAT::SlaveGroup& group,
        std::function<void()> on_safe_op = {},
        std::function<void()> pre_safe_op = {}) {
        const char* tag = "SynapticonInit";

        for (auto& ini : inits) {
            if (!ini.configurePDOsInPreOp(assignment)) return false;
        }

        // Last window with a guaranteed-live mailbox: firmware >= 5.6
        // stops servicing SDO in SAFE_OP, so anything mailbox-dependent
        // (interface construction / updateIdentity, config reads) hooks
        // in here, before the group request goes out.
        if (pre_safe_op) pre_safe_op();

        if (group.requestState(SlaveState::SAFE_OP, /*ack_error=*/true) == 0 ||
            !group.waitForState(SlaveState::SAFE_OP, /*timeout_ms=*/5000,
                                /*poll_ms=*/10, /*resend_ms=*/1000)) {
            TETHER_LOGE(tag, "Group SAFE_OP transition failed");
            return false;
        }

        if (on_safe_op) on_safe_op();

        for (auto& ini : inits) {
            if (!ini.postSafeOpPrepare()) return false;
        }
        return true;
    }

    /**
     * @brief OP request + confirmation for a group — ONE EtherCAT packet.
     * @return true when every member reached OP.
     */
    static bool requestGroupOp(EtherCAT::SlaveGroup& group,
                               const char* tag = "SynapticonInit",
                               uint32_t timeout_ms = 5000) {
        TETHER_LOGI(tag, "Requesting OP on {} slave(s) with a single "
                         "EtherCAT packet", group.size());
        if (group.requestState(SlaveState::OP, /*ack_error=*/true) == 0) {
            TETHER_LOGE(tag, "Group OP request failed");
            return false;
        }
        if (!group.waitForState(SlaveState::OP, timeout_ms,
                                /*poll_ms=*/10, /*resend_ms=*/1000)) {
            TETHER_LOGE(tag, "Group OP transition timed out");
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
    /// @param fsoe_frame  FSoE status-frame variant the slave is configured
    ///                    with (LW2 = 35-byte TxPDO with safe torque data,
    ///                    LW1 = 31-byte TxPDO).
    /// @return true on success.
    bool initCombined(
        SynapticonPDO::FSoEFrameVariant fsoe_frame =
            SynapticonPDO::FSoEFrameVariant::LW2) {
        if (!resetToInit()) return false;
        if (!configureMailbox()) return false;
        if (!transitionToPreOp()) return false;
        if (!configurePDOsAndOp(
                SynapticonPDO::makeCombinedPDOAssignment(fsoe_frame))) return false;
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

    /// @brief Read 0x100A firmware version, store it, and apply the
    ///        mailbox-in-SAFE_OP policy to the slave object.
    ///
    /// Must run while the slave is in PRE_OP (mailbox up).  SOMANET
    /// firmware >= 5.6 stops servicing the CoE mailbox in SAFE_OP —
    /// Slave::setMailboxServicedInSafeOp() then makes every
    /// mailbox-dependent diagnostic skip that window instead of
    /// burning seconds in guaranteed timeouts.
    FirmwareVersion readAndApplyFirmwareVersion() {
        auto& slave = master_.ethercatMaster().slave(slave_idx_);
        char buf[64] = {};
        size_t len = sizeof(buf) - 1;
        FirmwareVersion ver;
        std::string raw;
        if (slave.sdoRead(CiA301::ManufacturerSWVersion, 0, buf, len)
                == SlaveError::Ok && len > 0) {
            raw.assign(buf, len);
            ver = parseFirmwareVersion(raw);
        }

        firmware_version_ = ver;
        slave.setMailboxServicedInSafeOp(mailboxServicedInSafeOp(ver));

        TETHER_LOGI(tag_, "{} firmware: '{}' (parsed {}.{}.{}"
                    "{}) — mailbox in SAFE_OP: {}",
                    master_.ethercatMaster().slaveLogPrefix(slave_idx_).c_str(),
                    raw.empty() ? "unreadable" : raw.c_str(),
                    ver.major, ver.minor, ver.patch,
                    ver.valid ? "" : " [unparsed]",
                    slave.mailboxServicedInSafeOp() ? "yes" : "NO (fw >= 5.6)");
        return ver;
    }

    /// Parsed firmware version — valid only after
    /// readAndApplyFirmwareVersion() ran (initGroupToPreOp does it).
    FirmwareVersion firmwareVersion() const { return firmware_version_; }

private:
    DS402Master& master_;
    uint16_t slave_idx_;
    const char* tag_;
    FirmwareVersion firmware_version_{};
};

} // namespace Synapticon
} // namespace Drives
} // namespace EtherCAT

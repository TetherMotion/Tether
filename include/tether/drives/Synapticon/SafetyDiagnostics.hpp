/**
 * @file SafetyDiagnostics.hpp
 * @brief Synapticon SOMANET — safety module state diagnostics helper
 *
 * Reads and interprets object 0x2611 "Safety Module input diagnostics"
 * from a Synapticon SOMANET drive via CoE SDO.
 *
 * Per the Synapticon documentation
 * (https://doc.synapticon.com/circulo_safe_motion/sw5.1/objects_html/2xxx/2611.html):
 *
 *   0x2611 — Safety Module input diagnostics
 *     "Safe state diagnostics - state of safety module (0 = safe state)."
 *
 *     0x2611:1  Input 1   BOOL  readonly (default)
 *     0x2611:2  Input 2   BOOL  readonly (default)
 *
 * A value of 0 means the corresponding safety channel is in its *safe
 * state* — i.e. the safety function is active and torque output is
 * inhibited.  When both inputs are 0 the drive is fully in safe state
 * and must not be enabled; attempting to activate the drive in this
 * condition is an error and the caller should trigger a shutdown.
 *
 * A value of 1 means the safety channel is *not* in safe state (safety
 * function inactive, motion allowed for that channel).
 *
 * In addition to 0x2611, this helper also reads object 0x2620:2 "Safe
 * fieldbus" (per Synapticon documentation
 * https://doc.synapticon.com/circulo_safe_motion/sw5.1/objects_html/2xxx/2620.html),
 * which reports whether the FSoE (Safety-over-EtherCAT) connection is
 * active on the drive.  A non-zero value means FSoE is active; zero means
 * it is inactive.  This is reported for diagnostics only — it does *not*
 * gate the activation decision (that is driven solely by 0x2611), but it
 * is logged when the drive is found to be in safe state so the operator
 * can tell whether the safe state is due to an active FSoE safety
 * function or some other cause.
 *
 * Usage:
 * @code
 *   auto& slave = master.ethercatMaster().slave(slave_idx);
 *   auto result = EtherCAT::Drives::Synapticon::readSafetyModuleState(slave);
 *   if (!result.ok) {
 *       // SDO read failed — cannot verify safety state
 *   } else if (result.is_in_safe_state) {
 *       // Drive is in safe state — DO NOT enable, trigger shutdown.
 *       // result.fsoeActive() tells whether FSoE is the cause.
 *   }
 * @endcode
 */

#pragma once

#include <cstdint>
#include "tether/ethercat/Slave.hpp"

namespace EtherCAT {
namespace Drives {
namespace Synapticon {

/// Object 0x2611 "Safety Module input diagnostics" — index and subindices.
static constexpr uint16_t kSafetyModuleDiagnosticsIndex = 0x2611;
static constexpr uint8_t  kSafetyModuleInput1Subindex   = 0x01;
static constexpr uint8_t  kSafetyModuleInput2Subindex   = 0x02;

/// Object 0x2620 "General safety" — subindex 0x02 "Safe fieldbus".
/// Reports whether the FSoE (Safety-over-EtherCAT) connection is active
/// on the drive (non-zero = active, zero = inactive).  Diagnostics only.
static constexpr uint16_t kGeneralSafetyIndex    = 0x2620;
static constexpr uint8_t  kSafeFieldbusSubindex  = 0x02;

/// Result of reading the safety module input diagnostics (0x2611) and the
/// FSoE active indicator (0x2620:2 "Safe fieldbus").
struct SafetyModuleState {
    /// True if both 0x2611 SDO reads completed successfully.  This gates the
    /// activation decision; the 0x2620:2 read does not affect it.
    bool ok = false;

    /// Raw value of 0x2611:1 (Input 1).  0 = safe state, 1 = not safe state.
    uint8_t input1 = 0;

    /// Raw value of 0x2611:2 (Input 2).  0 = safe state, 1 = not safe state.
    uint8_t input2 = 0;

    /// True if the 0x2620:2 "Safe fieldbus" SDO read succeeded.
    bool fsoe_read_ok = false;

    /// Raw value of 0x2620:2 "Safe fieldbus".  Non-zero = FSoE active,
    /// zero = FSoE inactive.  Only meaningful when `fsoe_read_ok` is true.
    uint8_t safe_fieldbus = 0;

    /// True if either safety channel reports safe state (input == 0).
    /// When true, the safety function is active and motion is inhibited.
    [[nodiscard]] bool isInSafeState() const noexcept {
        return (input1 == 0) || (input2 == 0);
    }

    /// True only when both safety channels are NOT in safe state,
    /// meaning the safety function is fully inactive and motion is allowed.
    [[nodiscard]] bool motionAllowed() const noexcept {
        return (input1 != 0) && (input2 != 0);
    }

    /// True if the FSoE (Safety-over-EtherCAT) connection is active on the
    /// drive, per 0x2620:2.  Returns false if the read failed (fail-safe).
    [[nodiscard]] bool fsoeActive() const noexcept {
        return fsoe_read_ok && (safe_fieldbus != 0);
    }

    /// Human-readable summary of the FSoE active state, for logging.
    [[nodiscard]] const char* fsoeStateSummary() const noexcept {
        if (!fsoe_read_ok) return "unknown (SDO read failed)";
        return fsoeActive() ? "active" : "inactive";
    }

    /// Human-readable summary suitable for logging.
    [[nodiscard]] const char* stateSummary() const noexcept {
        if (!ok) return "unknown (SDO read failed)";
        if (isInSafeState()) return "SAFE STATE (motion inhibited)";
        return "operational (motion allowed)";
    }
};

/**
 * @brief Read the safety module input diagnostics (0x2611) and FSoE active
 *        indicator (0x2620:2) from a SOMANET drive.
 *
 * Performs two SDO reads (0x2611:1 and 0x2611:2) and interprets the result
 * according to the Synapticon documentation:
 *   0 = safe state (safety function active, torque inhibited)
 *   1 = not safe state (safety function inactive, motion allowed)
 *
 * It also reads 0x2620:2 "Safe fieldbus" to determine whether the FSoE
 * connection is active on the drive.  This is reported via `fsoeActive()`
 * and `fsoeStateSummary()` for diagnostics; it does not affect the
 * activation decision, which is driven solely by the 0x2611 reads.
 *
 * @param slave  Reference to the EtherCAT slave (SOMANET drive).
 * @return SafetyModuleState with the read values.  If either 0x2611 SDO
 *         read fails, `.ok` is false and the input values are left at 0
 *         (which would report safe state — fail-safe behavior).  A failure
 *         of the 0x2620:2 read only affects `fsoe_read_ok` / `fsoeActive()`,
 *         not `.ok`.
 */
inline SafetyModuleState readSafetyModuleState(::EtherCAT::Slave& slave) {
    SafetyModuleState state;

    const auto err1 = slave.sdoReadU8(
        kSafetyModuleDiagnosticsIndex, kSafetyModuleInput1Subindex,
        state.input1);
    const auto err2 = slave.sdoReadU8(
        kSafetyModuleDiagnosticsIndex, kSafetyModuleInput2Subindex,
        state.input2);

    state.ok = (err1 == ::EtherCAT::SlaveError::Ok) &&
               (err2 == ::EtherCAT::SlaveError::Ok);

    // 0x2620:2 "Safe fieldbus" — FSoE active indicator (diagnostics only).
    // A failure here must not gate the activation decision, so it is tracked
    // separately via fsoe_read_ok rather than via .ok.
    const auto err_fsoe = slave.sdoReadU8(
        kGeneralSafetyIndex, kSafeFieldbusSubindex,
        state.safe_fieldbus);
    state.fsoe_read_ok = (err_fsoe == ::EtherCAT::SlaveError::Ok);

    // Fail-safe: if a read fails, leave the input at 0 (safe state).
    // This ensures we never falsely report "motion allowed" when we
    // couldn't actually read the safety state.
    return state;
}

} // namespace Synapticon
} // namespace Drives
} // namespace EtherCAT

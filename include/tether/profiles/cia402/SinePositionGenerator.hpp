#pragma once

/**
 * @file SinePositionGenerator.hpp
 * @brief Sinusoidal CSP position command generator with a
 *        statusword-driven CiA 402 enable walk.
 *
 * Generates a sinusoidal **position** command for a CiA 402 drive in
 * Cyclic Synchronous Position mode (CSP, mode 8): the shaft sweeps a
 * small arc around the position it had when the generator first ran.
 *
 * `range_deg` is the total peak-to-peak swing in degrees of shaft
 * rotation (e.g. 5.0 → ±2.5° around the center).  `counts_per_rev`
 * converts degrees to encoder counts (object 0x608F:1/0x608F:2 or a
 * vendor-specific encoder resolution object).
 *
 * ## Enable strategy
 *
 * The enable sequence is performed via PDO controlword writes (not SDO)
 * so it can run inside the cyclic exchange without interfering with
 * mailbox traffic.  It is **statusword-driven**: every cycle the
 * controlword is chosen from the drive's reported CiA 402 state,
 * walking
 *
 *   SwitchOnDisabled --0x06--> ReadyToSwitchOn --0x07--> SwitchedOn
 *     --0x0F--> OperationEnabled
 *
 * and re-walking automatically whenever a safety demand (STO/quick
 * stop) or a fault drops the drive back down.  A time-based sequence
 * would deadlock: `Enable Operation` is not a legal command out of
 * SwitchOnDisabled, so once a timed window is missed the drive can
 * never leave that state.
 *
 * On drives with an FSoE safety layer, actual motion is additionally
 * gated by the drive's safety firmware — the shaft will not follow the
 * target until the safety connection is in the Data state and no stop
 * function is demanded.  The generator keeps issuing the
 * state-appropriate controlword, so the drive enables automatically
 * once safety releases.
 *
 * ## Phase sharing
 *
 * Pass the same `epoch` to every drive's generator so all shafts run
 * the identical waveform in phase; leave the default for a private
 * epoch anchored at the first `update()` call.
 */

#include <chrono>
#include <cmath>
#include <cstdint>
#include <numbers>

#include "tether/profiles/cia402/MotionPdoTypes.hpp"

namespace CiA402 {

class SinePositionGenerator {
public:
    /**
     * @param range_deg      Total peak-to-peak swing in degrees (shaft).
     * @param frequency_hz   Sine frequency in Hz.
     * @param counts_per_rev Encoder counts per shaft revolution.
     * @param epoch          Shared time origin for the sine phase — pass
     *                       the same instant to every generator that must
     *                       run phase-locked.  Default {} = private epoch
     *                       set on first update().
     */
    SinePositionGenerator(double range_deg, double frequency_hz,
                          double counts_per_rev,
                          std::chrono::steady_clock::time_point epoch = {})
        : range_deg_(range_deg)
        , frequency_hz_(frequency_hz)
        , counts_per_rev_(counts_per_rev)
        , epoch_(epoch)
    {
        if (counts_per_rev_ > 0) {
            amplitude_counts_ = range_deg_ / 360.0 * counts_per_rev_ / 2.0;
        }
    }

    /**
     * @brief Generate the next position command into a motion buffer.
     *
     * Chooses the CiA 402 controlword from the reported statusword state
     * and writes a CSP command (mode 8) with the sine-offset target
     * position.  The first call homes the sine around the current shaft
     * position so there is no jump at enable.
     *
     * @param dt_seconds      Cycle time in seconds (unused — phase is
     *                        derived from wall time vs `epoch_`).
     * @param statusword      Latest 0x6041 feedback from the drive.
     * @param position_actual Latest 0x6064 feedback (encoder counts).
     * @param cmd             Motion command buffer to fill; unused
     *                        fields are zeroed.
     */
    void update(double /*dt_seconds*/, uint16_t statusword,
                int32_t position_actual, MotionCommand& cmd) {
        if (epoch_ == std::chrono::steady_clock::time_point{}) {
            epoch_ = std::chrono::steady_clock::now();
        }
        const double elapsed_s_ =
            std::chrono::duration<double>(
                std::chrono::steady_clock::now() - epoch_).count();

        if (!center_valid_) {
            // Home the sine around wherever the shaft currently sits so the
            // first target is the current position — no jump at enable.
            center_ = position_actual;
            center_valid_ = true;
        }

        // CiA 402 enable sequence via PDO, driven by the reported state:
        //   SwitchOnDisabled  --Shutdown(0x06)-->     ReadyToSwitchOn
        //   ReadyToSwitchOn   --SwitchOn(0x07)-->     SwitchedOn
        //   SwitchedOn        --EnableOp(0x0F)-->     OperationEnabled
        //   QuickStopActive   --EnableOp(0x0F)-->     OperationEnabled
        //   Fault             --FaultReset(0x80)-->   SwitchOnDisabled
        // The drive won't actually transition while a safety function is
        // demanded (STO active / FSoE not in Data); the walk resumes from
        // wherever the statusword reports once safety releases.
        uint16_t controlword;
        const uint16_t sw = statusword;
        if ((sw & 0x004F) == 0x0008 || (sw & 0x004F) == 0x000F) {
            controlword = 0x0080;  // Fault / FaultReactionActive → reset
        } else if ((sw & 0x006F) == 0x0027 || (sw & 0x006F) == 0x0007) {
            controlword = 0x000F;  // OperationEnabled / QuickStopActive
        } else if ((sw & 0x006F) == 0x0023) {
            controlword = 0x000F;  // SwitchedOn → Enable Operation
        } else if ((sw & 0x006F) == 0x0021) {
            controlword = 0x0007;  // ReadyToSwitchOn → Switch On
        } else {
            controlword = 0x0006;  // SoD / NotReady / other → Shutdown
        }

        const double offset =
            amplitude_counts_ *
            std::sin(2.0 * std::numbers::pi * frequency_hz_ * elapsed_s_);

        cmd.controlword = controlword;
        cmd.modes_of_operation = 8;  // Cyclic Sync Position (CSP)
        cmd.target_position =
            center_ + static_cast<int32_t>(std::lround(offset));
        cmd.target_torque = 0;
        cmd.target_velocity = 0;
        cmd.torque_offset = 0;
    }

    /// Peak-to-peak swing in encoder counts (for diagnostics).
    double amplitudeCounts() const { return amplitude_counts_; }
    /// Encoder counts per shaft revolution.
    double countsPerRev() const { return counts_per_rev_; }

private:
    double   range_deg_        = 0.0;
    double   frequency_hz_     = 0.0;
    double   counts_per_rev_   = 0.0;
    double   amplitude_counts_ = 0.0;  // half the peak-to-peak range
    std::chrono::steady_clock::time_point epoch_;
    int32_t  center_           = 0;
    bool     center_valid_     = false;
};

} // namespace CiA402

#pragma once

/**
 * @file FSoEDriveEmulator.hpp
 * @brief Self-contained emulated Synapticon FSoE drive answering one
 *        FSoE channel.
 *
 * Thin synchronous wrapper around `SafeMotionServoEmulator`: each
 * `process()` call feeds the newest master→slave wire frame into the
 * FSoE slave state machine (identical frames still refresh the slave's
 * watchdog — `FSoESlave::processRxFrame` detects duplicates internally),
 * advances the watchdog timer, mirrors the command into the published
 * status, and regenerates the slave→master response frame.
 *
 * The emulator also models the CiA 402 power-stage side:
 * `processMotion()` consumes a cyclic `CiA402::MotionCommand`, walks a
 * minimal statusword state machine off the controlword, tracks
 * `target_position` only while Operation Enabled AND the FSoE slave
 * reports `controlMotionAllowed()`, and steps the servo emulator so the
 * safe position/velocity published in the FSoE frame moves like a real
 * shaft.  The resulting `MotionFeedback` can be published into the
 * emulated channel's feedback slot.
 *
 * Used by black-channel / relay tools to serve FSoE channels that have
 * no physical drive on the bus (hybrid or all-emulated topologies), and
 * reusable by any tool needing a self-contained emulated FSoE drive.
 *
 * Thread safety: none — drive the emulator from a single processing
 * thread.
 */

#include <array>
#include <cstddef>
#include <cstdint>
#include <memory>

#include "tether/fsoe/Synapticon/SafeMotionFSoE.hpp"
#include "tether/profiles/cia402/MotionPdoTypes.hpp"

namespace EtherCAT::Drives::Synapticon::SafeMotion {

/**
 * @brief Configuration for one emulated FSoE channel's drive.
 */
struct FSoEDriveEmulatorConfig {
    size_t   index = 0;            ///< channel index (for logging)
    uint16_t conn_id = 0;          ///< expected FSoE connection ID
    uint16_t safety_addr = 0;      ///< FSoE safety address
    uint16_t slave_addr = 0;       ///< nominal EtherCAT station address
    size_t   resp_frame_len = 31;  ///< slave→master wire length (31=LW1, 35=LW2)
    uint16_t watchdog_ms = 50;     ///< FSoE slave watchdog
    /// Status polarity (SDD-derived): true → mirror the zero-active
    /// command bit inverted so the status bit reads one-active.
    bool invert_sto = true;
    bool invert_sos = true;
    /// Emulated power stage ignores motion-gating FSoE demands — the
    /// wire/status path still reports demanded functions active (same
    /// end result as wire suppression applied to physical drives).
    bool suppress_demands = false;
};

class FSoEDriveEmulator {
public:
    static constexpr size_t kMaxFrame = 64;

    explicit FSoEDriveEmulator(const FSoEDriveEmulatorConfig& cfg);

    /// Build the emulator and publish an initial Reset-state response.
    bool initialize();

    /// Hard reset: FSoE slave state machine reset + safe state (STO/SBC
    /// active) + fresh initial response.  Call after a master restart.
    void reset();

    /**
     * @brief One processing cycle.
     *
     * `master_frame`/`len` is the channel's current master→slave wire
     * frame (may be nullptr/0 to just advance the watchdog).  Every
     * frame is fed to `processRxFrame()` — even byte-identical repeats
     * are the master's keep-alives and refresh the watchdog.  `time_ms`
     * is the cumulative tick time in milliseconds.
     */
    void process(const uint8_t* master_frame, size_t len, uint64_t time_ms);

    /// Latest slave→master wire frame.
    const uint8_t* response() const { return response_.data(); }
    size_t responseLen() const { return response_len_; }

    /**
     * @brief Consume one motion-command cycle like a real CiA402+CSP
     *        drive.
     *
     * Walks a minimal statusword state machine off the controlword,
     * tracks `target_position` only while Operation Enabled AND
     * `controlMotionAllowed()`, and steps the servo emulator so the
     * published safe position/velocity in the FSoE frame moves too.
     * Updates `motionFeedback()`.
     */
    void processMotion(const CiA402::MotionCommand& cmd, double dt_seconds);

    /// Latest emulated CiA402 motion feedback (statusword, mode display,
    /// position/velocity/torque actual).
    const CiA402::MotionFeedback& motionFeedback() const {
        return motion_fb_;
    }

    /// True when the last process() call consumed a new master frame.
    bool newFrame() const { return new_frame_; }
    /// FSoE slave state (FSoE::SlaveState ordinal).
    uint8_t state() const;
    /// Last master command byte that was processed (0 when none yet).
    uint8_t lastMasterCmd() const { return last_master_[0]; }

    const SafeMotionServoEmulator& emulator() const { return *emu_; }

private:
    FSoEDriveEmulatorConfig cfg_;
    std::unique_ptr<SafeMotionServoEmulator> emu_;

    std::array<uint8_t, kMaxFrame> last_master_{};
    size_t last_master_len_ = 0;
    bool have_master_ = false;
    bool new_frame_ = false;

    std::array<uint8_t, kMaxFrame> response_{};
    size_t response_len_ = 0;

    // Emulated CiA402 state (motion side): the FSoE emulator only models
    // the safety layer — this walks the power-stage state machine.
    CiA402::MotionFeedback motion_fb_{};
    double emulated_position_ = 0.0;
};

} // namespace EtherCAT::Drives::Synapticon::SafeMotion

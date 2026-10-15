#pragma once

/**
 * @file FSoEFrameFilter.hpp
 * @brief Per-channel serialized FSoE frame filters for black-channel and
 *        relay applications.
 *
 * A filter sits on the serialized FSoE frame path between a physical
 * FSoE slave (e.g. a Synapticon drive) and the response assembly sent
 * upstream to the FSoE master.  It adapts the slave's actual wire-frame
 * variant to the variant the master's connection slot expects.
 *
 * Provided implementations:
 *
 *   - `NoFSoEFilter` (default): verbatim passthrough, clamped to the
 *     output capacity.  Use whenever the slave's wire variant matches
 *     the master's slot.
 *   - `LW1ToLW2FSoEFilter`: the physical slave speaks the 31-byte LW1
 *     response (14 safe-data bytes) while the master slot expects the
 *     35-byte LW2 frame (16 safe-data bytes).  The filter appends a
 *     zeroed safe-torque word (Synapticon object 0x6616 — no torque
 *     feedback available on LW1) and re-serializes the frame with a
 *     valid CRC chain via an embedded `SlaveFrameRewriter` fed with the
 *     master's outgoing frames through `noteMasterFrame()`.
 *
 * Master→slave command frames share an identical 11-byte layout for
 * both variants, so `filterMasterToSlave()` is a passthrough in all
 * current filters — the interface keeps the direction hook so future
 * adapters (e.g. safe-data payload reshaping) can plug in symmetrically.
 *
 * Filters are stateless to create but a re-serializing instance carries
 * CRC chain state — instantiate after the slave's variant is known and
 * drive it from a single processing thread (the rewriter is not
 * thread-safe).
 */

#include <cstddef>
#include <cstdint>

#include "tether/fsoe/FSoECRC.hpp"
#include "tether/fsoe/FSoERewriter.hpp"

namespace FSoE {

/**
 * @brief Bidirectional serialized-frame filter, one instance per
 *        FSoE channel.
 *
 * Call `noteMasterFrame()` / `noteSlaveFrame()` with every observed
 * frame (both directions) so chain-tracking filters keep their CRC
 * state synchronized, then use `filterSlaveToMaster()` /
 * `filterMasterToSlave()` to transform frames for forwarding.
 */
class IFSoEFrameFilter {
public:
    virtual ~IFSoEFrameFilter() = default;

    /// Reset chain-tracking state (call when the channel's processor
    /// resets — e.g. after an FSoE Reset command cycle).
    virtual void reset() {}

    /// Observe the master's latest outgoing frame slice for this
    /// channel — feeds cross-direction CRC chain tracking.
    virtual void noteMasterFrame(const uint8_t* frame, size_t len) {
        (void)frame; (void)len;
    }

    /// Observe the channel's emitted slave→master frame (as the master
    /// received it) — feeds cross-direction CRC chain tracking.
    virtual void noteSlaveFrame(const uint8_t* frame, size_t len) {
        (void)frame; (void)len;
    }

    /// slave→master: transform the slave's raw TX frame into the frame
    /// the master expects.  Returns the output length (0 = no valid
    /// frame this cycle — the caller should emit zeros/a fail-safe
    /// placeholder).
    virtual size_t filterSlaveToMaster(const uint8_t* in, size_t in_len,
                                       uint8_t* out, size_t out_cap) = 0;

    /// master→slave: transform the master's command frame for the slave.
    /// Same wire layout for LW1/LW2 — passthrough in all current filters.
    virtual size_t filterMasterToSlave(const uint8_t* in, size_t in_len,
                                       uint8_t* out, size_t out_cap) = 0;

    /// Short diagnostic name for logging.
    virtual const char* name() const = 0;

    /// False when the filter re-serializes frames (needs CRC chain
    /// tracking via noteMasterFrame/noteSlaveFrame).
    virtual bool isTransparent() const { return true; }
};

/**
 * @brief Verbatim copy filter — the default when wire variants match.
 */
class NoFSoEFilter final : public IFSoEFrameFilter {
public:
    size_t filterSlaveToMaster(const uint8_t* in, size_t in_len,
                               uint8_t* out, size_t out_cap) override;
    size_t filterMasterToSlave(const uint8_t* in, size_t in_len,
                               uint8_t* out, size_t out_cap) override;
    const char* name() const override { return "NoFSoEFilter"; }
};

/**
 * @brief LW1 → LW2 slave→master frame expander.
 *
 * Expands a 31-byte LW1 response (14 safe-data bytes) into the 35-byte
 * LW2 frame by appending a zeroed safe-torque word (Synapticon object
 * 0x6616 — an LW1 slave produces no safe torque feedback; zero reads as
 * "no torque") and rebuilding the CRC chain via an embedded
 * `SlaveFrameRewriter`.
 *
 * Usage contract: call `noteMasterFrame()` with every master→slave
 * frame for the channel so the embedded rewriter can track the
 * master's CRC chain seed; then `filterSlaveToMaster()` produces
 * frames the master validates.
 */
class LW1ToLW2FSoEFilter final : public IFSoEFrameFilter {
public:
    void reset() override { rewriter_.reset(); }
    void noteMasterFrame(const uint8_t* frame, size_t len) override {
        rewriter_.noteMasterFrame(frame, len);
    }
    void noteSlaveFrame(const uint8_t* frame, size_t len) override {
        rewriter_.noteSlaveFrame(frame, len);
    }

    size_t filterSlaveToMaster(const uint8_t* in, size_t in_len,
                               uint8_t* out, size_t out_cap) override;
    size_t filterMasterToSlave(const uint8_t* in, size_t in_len,
                               uint8_t* out, size_t out_cap) override;
    const char* name() const override { return "LW1ToLW2FSoEFilter"; }
    bool isTransparent() const override { return false; }

private:
    SlaveFrameRewriter rewriter_;
};

} // namespace FSoE

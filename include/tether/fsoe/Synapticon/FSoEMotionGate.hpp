#pragma once

/**
 * @file FSoEMotionGate.hpp
 * @brief Wire-level helpers for overriding Synapticon FSoE motion-gating
 *        safety demands on a serialized black-channel frame stream.
 *
 * These functions operate directly on the *serialized* FSoE wire frame —
 * the byte layout exchanged inside the EtherCAT PDO, not the decoded
 * structured PDO objects.  They exist for black-channel / relay
 * applications that want to decouple a drive's power stage from the
 * safety master's stop-function demands while keeping the protocol
 * (CRC chain, connection state machine) fully intact:
 *
 *   master→slave: `suppressMotionGatingDemands()` rewrites the
 *       safety_flags word of a ProcessData frame so no stop/limit
 *       function is demanded, letting CiA 402 motion run even while the
 *       upstream master has STO active.
 *   slave→master: `mirrorDemandedStatus()` sets the status bit of every
 *       function the master currently demands to "active", so the
 *       master sees a compliant drive and raises no discrepancy error.
 *
 * FSoE serialized frame layout:
 *   [CMD][D0][D1][CRC0][D2][D3][CRC1]...[ConnID]
 * Each CRC covers the preceding 2-byte safe-data chunk, so safe-data
 * byte i sits at byte offset `1 + i + 2*(i/2)` in the wire frame.
 *
 * Because `FSoE::CRC::extractFSoEFrame` does not verify CRCs, patching
 * the input frame in place *before* `MasterFrameRewriter` /
 * `SlaveFrameRewriter::rewrite()` is sufficient — the rewritten output
 * gets a freshly computed, valid CRC chain.  Never forward a
 * byte-patched frame without re-running the rewriter: the embedded
 * CRCs would be stale.
 *
 * Active levels are channel-specific and come from the safety
 * controller's SDD (safe-device description): on the observed
 * Synapticon/ESC211 profile, *commands* are one-active (bit=1 demands
 * the function) while *status* bits are zero-active in some profiles —
 * pass the channel's configured levels, do not hardcode.
 */

#include <cstddef>
#include <cstdint>
#include <optional>

#include "tether/drives/Synapticon/SynapticonPDO.hpp"
#include "tether/fsoe/FSoEDefs.hpp"

namespace FSoE::Synapticon {

/**
 * @brief Serialized offset of safe-data byte `i` inside a wire FSoE frame.
 *
 * The frame interleaves a 2-byte CRC after every 2 safe-data bytes:
 * byte i maps to `1 + i + 2*(i/2)` (1 = the leading command byte).
 */
inline constexpr size_t safeDataByteOffset(size_t i) {
    return 1 + i + 2 * (i / 2);
}

/**
 * @brief Master→slave safety_flags bits that gate or limit drive motion.
 *
 * These are the `SOMANET_RxPDO_1700` safety_flags bits (safe-data bytes
 * 0–1) covering STO, SS1, SS2, SOS, SLS1–4 and SBC.  ErrorAck (bit 7),
 * RestartAck (bit 12) and ResetPos (bit 14) are protocol handshake bits,
 * not motion gates — they must be forwarded unchanged.
 */
inline constexpr uint16_t kMotionGatingSafetyFlags =
    EtherCAT::Drives::SynapticonPDO::SOMANET_RxPDO_1700::kSTO |
    EtherCAT::Drives::SynapticonPDO::SOMANET_RxPDO_1700::kSS1 |
    EtherCAT::Drives::SynapticonPDO::SOMANET_RxPDO_1700::kSS2 |
    EtherCAT::Drives::SynapticonPDO::SOMANET_RxPDO_1700::kSOS |
    EtherCAT::Drives::SynapticonPDO::SOMANET_RxPDO_1700::kSLS_Instance1 |
    EtherCAT::Drives::SynapticonPDO::SOMANET_RxPDO_1700::kSLS_Instance2 |
    EtherCAT::Drives::SynapticonPDO::SOMANET_RxPDO_1700::kSLS_Instance3 |
    EtherCAT::Drives::SynapticonPDO::SOMANET_RxPDO_1700::kSLS_Instance4 |
    EtherCAT::Drives::SynapticonPDO::SOMANET_RxPDO_1700::kSBCCommand;

/**
 * @brief Extract the 16-bit safety_flags word from a serialized
 *        master→slave ProcessData frame (safe-data bytes 0–1).
 *
 * @return The flags word, or `std::nullopt` for non-ProcessData commands
 *         (handshake frames carry no live demands) or a frame too short
 *         to hold the flags.
 */
inline std::optional<uint16_t> masterSafetyFlags(const uint8_t* frame,
                                                 size_t len) {
    if (!frame || frame[0] != FSoE::Command::ProcessData ||
        len < safeDataByteOffset(1) + 1) {
        return std::nullopt;
    }
    return static_cast<uint16_t>(
        frame[safeDataByteOffset(0)] |
        (static_cast<uint16_t>(frame[safeDataByteOffset(1)]) << 8));
}

/**
 * @brief Force every motion-gating safety-function demand in a serialized
 *        master→slave frame to "not demanded" (in place).
 *
 * No-op unless the frame is a ProcessData frame carrying the
 * safety_flags word — handshake and FailSafeData frames pass through
 * verbatim (FailSafeData deliberately keeps its safe-state values: they
 * define what the drive applies on a genuine connection loss).
 *
 * The caller must re-serialize the frame's CRC chain afterwards (feed
 * the patched frame through `MasterFrameRewriter::rewrite()` or feed it
 * to an emulator that recomputes CRCs).
 *
 * @param frame           Wire frame to patch in place.
 * @param len             Frame length in bytes.
 * @param cmd_active_high The channel's command active level:
 *                        true → bit=1 demands (release clears gating bits);
 *                        false → zero-active (release sets them).
 */
inline void suppressMotionGatingDemands(uint8_t* frame, size_t len,
                                        bool cmd_active_high) {
    if (!frame || frame[0] != FSoE::Command::ProcessData ||
        len < safeDataByteOffset(1) + 1) {
        return;
    }
    const uint8_t lo = kMotionGatingSafetyFlags & 0xFF;
    const uint8_t hi = kMotionGatingSafetyFlags >> 8;
    if (cmd_active_high) {
        frame[safeDataByteOffset(0)] &= static_cast<uint8_t>(~lo);
        frame[safeDataByteOffset(1)] &= static_cast<uint8_t>(~hi);
    } else {
        frame[safeDataByteOffset(0)] |= lo;
        frame[safeDataByteOffset(1)] |= hi;
    }
}

/**
 * @brief In a serialized slave→master ProcessData frame, force the status
 *        bit of every safety function the master is currently demanding
 *        to "active" (in place).
 *
 * Makes the upstream safety controller observe a compliant drive even
 * though the demands were suppressed before reaching the real drive.
 * All other status bits — error/diagnostic flags, safe I/O,
 * position/velocity data — pass through untouched.  The CRC chain must
 * be rebuilt afterwards (`SlaveFrameRewriter::rewrite()`).
 *
 * @param frame              Wire frame to patch in place.
 * @param len                Frame length in bytes.
 * @param demands            The master's safety_flags word
 *                           (`masterSafetyFlags()`); ignored bits are
 *                           skipped.
 * @param cmd_active_high    Command active level: a SET bit marks a
 *                           demanded function when true; a CLEAR bit
 *                           when false.
 * @param status_active_high Status active level: false → a CLEAR bit
 *                           reports the function active.
 */
inline void mirrorDemandedStatus(uint8_t* frame, size_t len,
                                 uint16_t demands,
                                 bool cmd_active_high,
                                 bool status_active_high) {
    if (!frame || frame[0] != FSoE::Command::ProcessData ||
        len < safeDataByteOffset(2) + 1) {
        return;
    }
    // command bit -> (safe-data byte, status bit)
    static constexpr struct { uint16_t cmd; uint8_t byte, bit; } kMap[] = {
        {0x0001, 0, 0x01},  // STO   -> safety_state_flags bit 0
        {0x0002, 1, 0x01},  // SS1   -> safety_state_flags bit 8
        {0x0004, 1, 0x02},  // SS2   -> safety_state_flags bit 9
        {0x0008, 0, 0x08},  // SOS   -> safety_state_flags bit 3
        {0x0100, 1, 0x10},  // SLS 1 -> safety_state_flags bit 12
        {0x0200, 1, 0x20},  // SLS 2 -> safety_state_flags bit 13
        {0x0400, 1, 0x40},  // SLS 3 -> safety_state_flags bit 14
        {0x0800, 1, 0x80},  // SLS 4 -> safety_state_flags bit 15
        {0x2000, 2, 0x02},  // SBC   -> diagnostic_flags bit 1
    };
    for (const auto& m : kMap) {
        const bool demanded =
            cmd_active_high ? (demands & m.cmd) != 0 : !(demands & m.cmd);
        if (!demanded) continue;
        auto& b = frame[safeDataByteOffset(m.byte)];
        if (status_active_high)
            b |= m.bit;
        else
            b &= static_cast<uint8_t>(~m.bit);
    }
}

} // namespace FSoE::Synapticon

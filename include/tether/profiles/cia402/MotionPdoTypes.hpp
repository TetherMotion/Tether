#pragma once

/**
 * @file MotionPdoTypes.hpp
 * @brief Packed CiA 402 RxPDO/TxPDO structures for cyclic drive control.
 *
 * Field names match the canonical CoE object names (controlword,
 * modes_of_operation, target_position, ...) matching how the
 * Synapticon ESI and most object dictionaries name them.
 *
 * The layouts mirror the Synapticon SOMANET cyclic-time-series PDO
 * mapping:
 *
 *   RxPDO (master → drive, 0x1600+0x1601+0x1602, 15 bytes):
 *     0x6040 controlword | 0x6060 modes_of_operation |
 *     0x6071 target_torque | 0x607A target_position |
 *     0x60FF target_velocity | 0x60B2 torque_offset
 *
 *   TxPDO (drive → master, 0x1A00-family, 11 bytes):
 *     0x6041 statusword | 0x6061 modes_of_operation_display |
 *     0x6064 position_actual | 0x606C velocity_actual | 0x6077 torque_actual
 *
 * Position units are encoder counts; velocity is counts/second;
 * torque is per-mille of rated torque.
 */

#include <cstdint>
#include <type_traits>

namespace CiA402 {

/**
 * @brief Master → drive cyclic motion command (RxPDO payload).
 */
#pragma pack(push, 1)
struct MotionCommand {
    uint16_t controlword = 0;           ///< 0x6040 — shutdown/enable/fault-reset
    int8_t   modes_of_operation = 0;    ///< 0x6060 — e.g. 8 = CSP
    int16_t  target_torque = 0;         ///< 0x6071 — per-mille (CST)
    int32_t  target_position = 0;       ///< 0x607A — counts (CSP/PP)
    int32_t  target_velocity = 0;       ///< 0x60FF — counts/s (CSV / feed-fwd)
    int16_t  torque_offset = 0;         ///< 0x60B2 — per-mille feed-forward
};
#pragma pack(pop)
static_assert(sizeof(MotionCommand) == 15);
static_assert(std::is_trivially_copyable_v<MotionCommand>);

/**
 * @brief Drive → master cyclic motion feedback (TxPDO payload).
 */
#pragma pack(push, 1)
struct MotionFeedback {
    uint16_t statusword = 0;                  ///< 0x6041
    int8_t   modes_of_operation_display = 0;  ///< 0x6061
    int32_t  position_actual = 0;             ///< 0x6064 — counts
    int32_t  velocity_actual = 0;             ///< 0x606C — counts/s
    int16_t  torque_actual = 0;               ///< 0x6077 — per-mille
};
#pragma pack(pop)
static_assert(sizeof(MotionFeedback) == 13);
static_assert(std::is_trivially_copyable_v<MotionFeedback>);

} // namespace CiA402

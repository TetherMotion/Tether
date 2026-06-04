/**
 * @file DynaDrivePDO.hpp
 * @brief DynaDrive (ANYdrive / rsl_drive_sdk) PDO Layout Definitions — Type E
 *
 * These packed structs match the rsl_drive_sdk PDO Type E layout exactly.
 * RxPDO uses index 0x1603 (Type D Rx), TxPDO E uses index 0x1A04.
 */

#pragma once

#include <cstdint>

namespace EtherCAT {
namespace Drives {
namespace DynaDrive_pdo {

// ============================================================================
// RxPDO (0x1603) — Master -> Slave, 40 bytes
// ============================================================================

struct DynaDrive_RxPDO {
    uint16_t controlword_        = 0;
    uint16_t modeOfOperation_    = 0;
    float    desiredMotorCurrent_ = 0.0f;
    float    desiredVelocity_    = 0.0f;
    float    desiredJointTorque_ = 0.0f;
    double   desiredPosition_    = 0.0;
    float    controlParameterA_  = 0.0f;
    float    controlParameterB_  = 0.0f;
    float    controlParameterC_  = 0.0f;
    float    controlParameterD_  = 0.0f;
} __attribute__((packed));

static_assert(sizeof(DynaDrive_RxPDO) == 40, "DynaDrive_RxPDO size mismatch");

// ============================================================================
// TxPDO E (0x1A04) — Slave -> Master, ~204 bytes
// ============================================================================

struct DynaDrive_TxPDO {
    uint32_t statusword_                     = 0;
    uint16_t measuredTemperature_            = 0;
    uint16_t measuredMotorVoltage_           = 0;
    double   measuredMotorPosition_          = 0.0;
    double   measuredGearPosition_           = 0.0;
    double   measuredJointPosition_          = 0.0;
    float    measuredMotorCurrent_           = 0.0f;
    float    measuredMotorVelocity_          = 0.0f;
    float    measuredGearVelocity_           = 0.0f;
    float    measuredJointVelocity_          = 0.0f;
    float    measuredJointAcceleration_      = 0.0f;
    float    measuredJointTorque_            = 0.0f;
    int32_t  measuredGearPositionTicks_      = 0;
    int32_t  measuredJointPositionTicks_     = 0;
    uint64_t timestamp_                      = 0;
    float    desiredCurrentD_                = 0.0f;
    float    measuredCurrentD_               = 0.0f;
    float    desiredCurrentQ_                = 0.0f;
    float    measuredCurrentQ_               = 0.0f;
    float    alpha_                          = 0.0f;
    float    beta_                           = 0.0f;
    float    dutyCycleU_                     = 0.0f;
    float    dutyCycleV_                     = 0.0f;
    float    dutyCycleW_                     = 0.0f;
    float    measuredCurrentPhaseU_          = 0.0f;
    float    measuredCurrentPhaseV_          = 0.0f;
    float    measuredCurrentPhaseW_          = 0.0f;
    float    measuredVoltagePhaseU_          = 0.0f;
    float    measuredVoltagePhaseV_          = 0.0f;
    float    measuredVoltagePhaseW_          = 0.0f;
    float    desiredMotorVelocity_           = 0.0f;
    double   desiredGearPosition_            = 0.0;
    float    desiredGearVelocity_            = 0.0f;
    double   desiredJointPosition_           = 0.0;
    float    desiredJointVelocity_           = 0.0f;
    float    desiredJointTorque_             = 0.0f;
    float    measuredImuLinearAccelerationX_ = 0.0f;
    float    measuredImuLinearAccelerationY_ = 0.0f;
    float    measuredImuLinearAccelerationZ_ = 0.0f;
    float    measuredImuAngularVelocityX_    = 0.0f;
    float    measuredImuAngularVelocityY_    = 0.0f;
    float    measuredImuAngularVelocityZ_    = 0.0f;
    float    measuredImuAngularVelocityW_    = 0.0f;
    int32_t  measuredCoilTemp1_             = 0;
    int32_t  measuredCoilTemp2_             = 0;
    int32_t  measuredCoilTemp3_             = 0;
} __attribute__((packed));

static_assert(sizeof(DynaDrive_TxPDO) == 204, "DynaDrive_TxPDO size mismatch");

// ============================================================================
// PDO Descriptors (constexpr objects, matching AS715N pattern)
// ============================================================================

struct PDODescriptor {
    uint16_t index;
    uint16_t size;
};

static constexpr PDODescriptor RxPDO_1603 = { 0x1603, sizeof(DynaDrive_RxPDO) };
static constexpr PDODescriptor TxPDO_1A04 = { 0x1A04, sizeof(DynaDrive_TxPDO) };

} // namespace DynaDrive_pdo
} // namespace Drives
} // namespace EtherCAT

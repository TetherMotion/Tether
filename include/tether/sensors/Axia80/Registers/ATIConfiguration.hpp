/**
 * @file ATIConfiguration.hpp
 * @brief Axia80 ATI-specific configuration objects (0x2019–0x2021)
 *
 * Product Description, Tool Transform, and Calibration Matrix.
 */

#pragma once

#include "tether/sensors/Axia80/Registers/Common.hpp"

namespace EtherCAT {
namespace Sensors {
namespace Axia80 {
namespace Registers {
namespace ATIConfiguration {

// ============================================================================
// 0x2019: Product Description
// ============================================================================

static constexpr uint16_t ProductDescriptionIndex = 0x2019;

constexpr RegisterEntry ProductDescription_VendorId = {
    .index = ProductDescriptionIndex,
    .subindex = 0x01,
    .name = "Vendor ID",
    .data_type = ObjectDictionaryDataType::Unsigned32,
    .default_value = 0x00000732,
    .unit = Unit_None,
    .options_enum = nullptr,
    .min_value = 0,
    .max_value = 0xFFFFFFFF,
    .modification_mode = ModificationMode::DuringOperation,
    .effective_time = EffectiveTime::Immediately,
    .comment = "Mirrored into Object 0x1018:01. Password-protected via Commit field.",
};

constexpr RegisterEntry ProductDescription_ProductCode = {
    .index = ProductDescriptionIndex,
    .subindex = 0x02,
    .name = "Product code",
    .data_type = ObjectDictionaryDataType::Unsigned32,
    .default_value = 0x26483053,
    .unit = Unit_None,
    .options_enum = nullptr,
    .min_value = 0,
    .max_value = 0xFFFFFFFF,
    .modification_mode = ModificationMode::DuringOperation,
    .effective_time = EffectiveTime::Immediately,
    .comment = "Mirrored into Object 0x1018:02",
};

constexpr RegisterEntry ProductDescription_ProductName = {
    .index = ProductDescriptionIndex,
    .subindex = 0x03,
    .name = "Product name",
    .data_type = ObjectDictionaryDataType::VisibleString,
    .default_value = 0,
    .unit = Unit_None,
    .options_enum = nullptr,
    .min_value = 0,
    .max_value = 0,
    .modification_mode = ModificationMode::DuringOperation,
    .effective_time = EffectiveTime::Immediately,
    .comment = "Default: ATI Axia F/T Sensor (32 chars max). Mirrored into 0x1008.",
};

constexpr RegisterEntry ProductDescription_ProductRevision = {
    .index = ProductDescriptionIndex,
    .subindex = 0x04,
    .name = "Product revision",
    .data_type = ObjectDictionaryDataType::Unsigned32,
    .default_value = 0,
    .unit = Unit_None,
    .options_enum = nullptr,
    .min_value = 0,
    .max_value = 0xFFFFFFFF,
    .modification_mode = ModificationMode::DuringOperation,
    .effective_time = EffectiveTime::Immediately,
    .comment = "Not utilized by ATI",
};

constexpr RegisterEntry ProductDescription_ProductSerial = {
    .index = ProductDescriptionIndex,
    .subindex = 0x05,
    .name = "Product serial number",
    .data_type = ObjectDictionaryDataType::Unsigned32,
    .default_value = 0,
    .unit = Unit_None,
    .options_enum = nullptr,
    .min_value = 0,
    .max_value = 0xFFFFFFFF,
    .modification_mode = ModificationMode::DuringOperation,
    .effective_time = EffectiveTime::Immediately,
    .comment = "System integrator serial pairing field",
};

constexpr RegisterEntry ProductDescription_Manufacturer = {
    .index = ProductDescriptionIndex,
    .subindex = 0x06,
    .name = "Manufacturer",
    .data_type = ObjectDictionaryDataType::VisibleString,
    .default_value = 0,
    .unit = Unit_None,
    .options_enum = nullptr,
    .min_value = 0,
    .max_value = 0,
    .modification_mode = ModificationMode::DuringOperation,
    .effective_time = EffectiveTime::Immediately,
    .comment = "Default: ATI Industrial Automation (32 chars max)",
};

constexpr RegisterEntry ProductDescription_Commit = {
    .index = ProductDescriptionIndex,
    .subindex = 0x07,
    .name = "Commit",
    .data_type = ObjectDictionaryDataType::Unsigned32,
    .default_value = 0x00000000,
    .unit = Unit_None,
    .options_enum = nullptr,
    .min_value = 0,
    .max_value = 0xFFFFFFFF,
    .modification_mode = ModificationMode::DuringOperation,
    .effective_time = EffectiveTime::Immediately,
    .comment = "Accepts changes via password. Write non-zero to commit.",
};

// ============================================================================
// 0x2020: Tool Transformation
// ============================================================================

static constexpr uint16_t ToolTransformIndex = 0x2020;

constexpr RegisterEntry ToolTransform_Dx = {
    .index = ToolTransformIndex,
    .subindex = 0x01,
    .name = "Dx",
    .data_type = ObjectDictionaryDataType::VisibleString,
    .default_value = 0,
    .unit = Unit_None,
    .options_enum = nullptr,
    .min_value = 0,
    .max_value = 0,
    .modification_mode = ModificationMode::DuringOperation,
    .effective_time = EffectiveTime::Immediately,
    .comment = "Displacement X as text-form floating-point (12 chars max)",
};

constexpr RegisterEntry ToolTransform_Dy = {
    .index = ToolTransformIndex,
    .subindex = 0x02,
    .name = "Dy",
    .data_type = ObjectDictionaryDataType::VisibleString,
    .default_value = 0,
    .unit = Unit_None,
    .options_enum = nullptr,
    .min_value = 0,
    .max_value = 0,
    .modification_mode = ModificationMode::DuringOperation,
    .effective_time = EffectiveTime::Immediately,
    .comment = "Displacement Y as text-form floating-point (12 chars max)",
};

constexpr RegisterEntry ToolTransform_Dz = {
    .index = ToolTransformIndex,
    .subindex = 0x03,
    .name = "Dz",
    .data_type = ObjectDictionaryDataType::VisibleString,
    .default_value = 0,
    .unit = Unit_None,
    .options_enum = nullptr,
    .min_value = 0,
    .max_value = 0,
    .modification_mode = ModificationMode::DuringOperation,
    .effective_time = EffectiveTime::Immediately,
    .comment = "Displacement Z as text-form floating-point (12 chars max)",
};

constexpr RegisterEntry ToolTransform_Rx = {
    .index = ToolTransformIndex,
    .subindex = 0x04,
    .name = "Rx",
    .data_type = ObjectDictionaryDataType::VisibleString,
    .default_value = 0,
    .unit = Unit_None,
    .options_enum = nullptr,
    .min_value = 0,
    .max_value = 0,
    .modification_mode = ModificationMode::DuringOperation,
    .effective_time = EffectiveTime::Immediately,
    .comment = "Rotation X as text-form floating-point (12 chars max)",
};

constexpr RegisterEntry ToolTransform_Ry = {
    .index = ToolTransformIndex,
    .subindex = 0x05,
    .name = "Ry",
    .data_type = ObjectDictionaryDataType::VisibleString,
    .default_value = 0,
    .unit = Unit_None,
    .options_enum = nullptr,
    .min_value = 0,
    .max_value = 0,
    .modification_mode = ModificationMode::DuringOperation,
    .effective_time = EffectiveTime::Immediately,
    .comment = "Rotation Y as text-form floating-point (12 chars max)",
};

constexpr RegisterEntry ToolTransform_Rz = {
    .index = ToolTransformIndex,
    .subindex = 0x06,
    .name = "Rz",
    .data_type = ObjectDictionaryDataType::VisibleString,
    .default_value = 0,
    .unit = Unit_None,
    .options_enum = nullptr,
    .min_value = 0,
    .max_value = 0,
    .modification_mode = ModificationMode::DuringOperation,
    .effective_time = EffectiveTime::Immediately,
    .comment = "Rotation Z as text-form floating-point (12 chars max)",
};

// Tool transform distance units
enum class ToolTransformDistUnits : uint8_t {
    Inches = 0,
    Feet = 1,
    Millimeters = 2,
    Centimeters = 3,
    Reserved = 4,
};

constexpr RegisterEntry ToolTransform_DistUnits = {
    .index = ToolTransformIndex,
    .subindex = 0x07,
    .name = "ttDistUnits",
    .data_type = ObjectDictionaryDataType::Unsigned8,
    .default_value = 2,
    .unit = Unit_None,
    .options_enum = std::type_identity<ToolTransformDistUnits>{},
    .min_value = 0,
    .max_value = 4,
    .modification_mode = ModificationMode::DuringOperation,
    .effective_time = EffectiveTime::Immediately,
    .comment = "Distance unit: 0=inches, 1=feet, 2=mm, 3=cm, 4=reserved",
};

// Tool transform angle units
enum class ToolTransformAngUnits : uint8_t {
    Degrees = 0,
    Radians = 1,
};

constexpr RegisterEntry ToolTransform_AngUnits = {
    .index = ToolTransformIndex,
    .subindex = 0x08,
    .name = "ttAngUnits",
    .data_type = ObjectDictionaryDataType::Unsigned8,
    .default_value = 0,
    .unit = Unit_None,
    .options_enum = std::type_identity<ToolTransformAngUnits>{},
    .min_value = 0,
    .max_value = 1,
    .modification_mode = ModificationMode::DuringOperation,
    .effective_time = EffectiveTime::Immediately,
    .comment = "Angle unit: 0=degrees, 1=radians",
};

constexpr RegisterEntry ToolTransform_Commit = {
    .index = ToolTransformIndex,
    .subindex = 0x09,
    .name = "Commit",
    .data_type = ObjectDictionaryDataType::Unsigned8,
    .default_value = 0,
    .unit = Unit_None,
    .options_enum = nullptr,
    .min_value = 0,
    .max_value = 255,
    .modification_mode = ModificationMode::DuringOperation,
    .effective_time = EffectiveTime::Immediately,
    .comment = "Write decimal 123 to commit and activate tool transform changes",
};

// ============================================================================
// 0x2021: Calibration Matrix (selected key subindices)
// ============================================================================

static constexpr uint16_t CalibrationIndex = 0x2021;

constexpr RegisterEntry Calibration_FTSerial = {
    .index = CalibrationIndex,
    .subindex = 0x01,
    .name = "FT Serial",
    .data_type = ObjectDictionaryDataType::VisibleString,
    .default_value = 0,
    .unit = Unit_None,
    .options_enum = nullptr,
    .min_value = 0,
    .max_value = 0,
    .modification_mode = ModificationMode::ReadOnly,
    .effective_time = EffectiveTime::Immediately,
    .comment = "F/T Calibration Serial Number (e.g. FT01234, 8 chars max)",
};

constexpr RegisterEntry Calibration_PartNumber = {
    .index = CalibrationIndex,
    .subindex = 0x02,
    .name = "Calibration part number",
    .data_type = ObjectDictionaryDataType::VisibleString,
    .default_value = 0,
    .unit = Unit_None,
    .options_enum = nullptr,
    .min_value = 0,
    .max_value = 0,
    .modification_mode = ModificationMode::ReadOnly,
    .effective_time = EffectiveTime::Immediately,
    .comment = "Calibration size configuration (e.g. SI-500-20, 30 chars max)",
};

constexpr RegisterEntry Calibration_Family = {
    .index = CalibrationIndex,
    .subindex = 0x03,
    .name = "Calibration family",
    .data_type = ObjectDictionaryDataType::VisibleString,
    .default_value = 0,
    .unit = Unit_None,
    .options_enum = nullptr,
    .min_value = 0,
    .max_value = 0,
    .modification_mode = ModificationMode::ReadOnly,
    .effective_time = EffectiveTime::Immediately,
    .comment = "Hardcoded constant: ECAT (8 chars max)",
};

constexpr RegisterEntry Calibration_Time = {
    .index = CalibrationIndex,
    .subindex = 0x04,
    .name = "Calibration time",
    .data_type = ObjectDictionaryDataType::VisibleString,
    .default_value = 0,
    .unit = Unit_None,
    .options_enum = nullptr,
    .min_value = 0,
    .max_value = 0,
    .modification_mode = ModificationMode::ReadOnly,
    .effective_time = EffectiveTime::Immediately,
    .comment = "Factory calibration timestamp (30 chars max)",
};

// Force units enum
enum class CalibrationForceUnits : uint8_t {
    Lbf = 0,
    Newton = 1,
    Klbf = 2,
    Kilonewton = 3,
    Kilogram = 4,
};

constexpr RegisterEntry Calibration_ForceUnits = {
    .index = CalibrationIndex,
    .subindex = 0x2F,
    .name = "Force units",
    .data_type = ObjectDictionaryDataType::Unsigned8,
    .default_value = 0,
    .unit = Unit_None,
    .options_enum = std::type_identity<CalibrationForceUnits>{},
    .min_value = 0,
    .max_value = 4,
    .modification_mode = ModificationMode::ReadOnly,
    .effective_time = EffectiveTime::Immediately,
    .comment = "0=Lbf, 1=N, 2=Klbf, 3=kN, 4=Kg",
};

// Torque units enum
enum class CalibrationTorqueUnits : uint8_t {
    LbfIn = 0,
    LbfFt = 1,
    NewtonMeter = 2,
    NewtonMillimeter = 3,
    KgfCm = 4,
    KiloNewtonMeter = 5,
};

constexpr RegisterEntry Calibration_TorqueUnits = {
    .index = CalibrationIndex,
    .subindex = 0x30,
    .name = "Torque units",
    .data_type = ObjectDictionaryDataType::Unsigned8,
    .default_value = 0,
    .unit = Unit_None,
    .options_enum = std::type_identity<CalibrationTorqueUnits>{},
    .min_value = 0,
    .max_value = 5,
    .modification_mode = ModificationMode::ReadOnly,
    .effective_time = EffectiveTime::Immediately,
    .comment = "0=lbf-in, 1=lbf-ft, 2=Nm, 3=Nmm, 4=kgf-cm, 5=kNm",
};

constexpr RegisterEntry Calibration_MaxFxCounts = {
    .index = CalibrationIndex,
    .subindex = 0x31,
    .name = "Max Fx counts",
    .data_type = ObjectDictionaryDataType::Integer32,
    .default_value = 0,
    .unit = Unit_None,
    .options_enum = nullptr,
    .min_value = 0,
    .max_value = 0x7FFFFFFF,
    .modification_mode = ModificationMode::ReadOnly,
    .effective_time = EffectiveTime::Immediately,
    .comment = "Maximum rated calibrated sensing limit for Fx",
};

constexpr RegisterEntry Calibration_MaxFyCounts = {
    .index = CalibrationIndex,
    .subindex = 0x32,
    .name = "Max Fy counts",
    .data_type = ObjectDictionaryDataType::Integer32,
    .default_value = 0,
    .unit = Unit_None,
    .options_enum = nullptr,
    .min_value = 0,
    .max_value = 0x7FFFFFFF,
    .modification_mode = ModificationMode::ReadOnly,
    .effective_time = EffectiveTime::Immediately,
    .comment = "Maximum rated calibrated sensing limit for Fy",
};

constexpr RegisterEntry Calibration_MaxFzCounts = {
    .index = CalibrationIndex,
    .subindex = 0x33,
    .name = "Max Fz counts",
    .data_type = ObjectDictionaryDataType::Integer32,
    .default_value = 0,
    .unit = Unit_None,
    .options_enum = nullptr,
    .min_value = 0,
    .max_value = 0x7FFFFFFF,
    .modification_mode = ModificationMode::ReadOnly,
    .effective_time = EffectiveTime::Immediately,
    .comment = "Maximum rated calibrated sensing limit for Fz",
};

constexpr RegisterEntry Calibration_MaxTxCounts = {
    .index = CalibrationIndex,
    .subindex = 0x34,
    .name = "Max Tx counts",
    .data_type = ObjectDictionaryDataType::Integer32,
    .default_value = 0,
    .unit = Unit_None,
    .options_enum = nullptr,
    .min_value = 0,
    .max_value = 0x7FFFFFFF,
    .modification_mode = ModificationMode::ReadOnly,
    .effective_time = EffectiveTime::Immediately,
    .comment = "Maximum rated calibrated sensing limit for Tx",
};

constexpr RegisterEntry Calibration_MaxTyCounts = {
    .index = CalibrationIndex,
    .subindex = 0x35,
    .name = "Max Ty counts",
    .data_type = ObjectDictionaryDataType::Integer32,
    .default_value = 0,
    .unit = Unit_None,
    .options_enum = nullptr,
    .min_value = 0,
    .max_value = 0x7FFFFFFF,
    .modification_mode = ModificationMode::ReadOnly,
    .effective_time = EffectiveTime::Immediately,
    .comment = "Maximum rated calibrated sensing limit for Ty",
};

constexpr RegisterEntry Calibration_MaxTzCounts = {
    .index = CalibrationIndex,
    .subindex = 0x36,
    .name = "Max Tz counts",
    .data_type = ObjectDictionaryDataType::Integer32,
    .default_value = 0,
    .unit = Unit_None,
    .options_enum = nullptr,
    .min_value = 0,
    .max_value = 0x7FFFFFFF,
    .modification_mode = ModificationMode::ReadOnly,
    .effective_time = EffectiveTime::Immediately,
    .comment = "Maximum rated calibrated sensing limit for Tz",
};

constexpr RegisterEntry Calibration_CountsPerForce = {
    .index = CalibrationIndex,
    .subindex = 0x37,
    .name = "Counts per force",
    .data_type = ObjectDictionaryDataType::Integer32,
    .default_value = 0,
    .unit = Unit_None,
    .options_enum = nullptr,
    .min_value = 1,
    .max_value = 0x7FFFFFFF,
    .modification_mode = ModificationMode::ReadOnly,
    .effective_time = EffectiveTime::Immediately,
    .comment = "Scaling factor denominator for engineering force units from counts",
};

constexpr RegisterEntry Calibration_CountsPerTorque = {
    .index = CalibrationIndex,
    .subindex = 0x38,
    .name = "Counts per torque",
    .data_type = ObjectDictionaryDataType::Integer32,
    .default_value = 0,
    .unit = Unit_None,
    .options_enum = nullptr,
    .min_value = 1,
    .max_value = 0x7FFFFFFF,
    .modification_mode = ModificationMode::ReadOnly,
    .effective_time = EffectiveTime::Immediately,
    .comment = "Scaling factor denominator for engineering torque units from counts",
};

// Register list for ATI configuration objects
inline const RegisterList kRegisterList = {
    &ProductDescription_VendorId,
    &ProductDescription_ProductCode,
    &ProductDescription_ProductName,
    &ProductDescription_ProductRevision,
    &ProductDescription_ProductSerial,
    &ProductDescription_Manufacturer,
    &ProductDescription_Commit,
    &ToolTransform_Dx,
    &ToolTransform_Dy,
    &ToolTransform_Dz,
    &ToolTransform_Rx,
    &ToolTransform_Ry,
    &ToolTransform_Rz,
    &ToolTransform_DistUnits,
    &ToolTransform_AngUnits,
    &ToolTransform_Commit,
    &Calibration_FTSerial,
    &Calibration_PartNumber,
    &Calibration_Family,
    &Calibration_Time,
    &Calibration_ForceUnits,
    &Calibration_TorqueUnits,
    &Calibration_MaxFxCounts,
    &Calibration_MaxFyCounts,
    &Calibration_MaxFzCounts,
    &Calibration_MaxTxCounts,
    &Calibration_MaxTyCounts,
    &Calibration_MaxTzCounts,
    &Calibration_CountsPerForce,
    &Calibration_CountsPerTorque,
};

} // namespace ATIConfiguration
} // namespace Registers
} // namespace Axia80
} // namespace Sensors
} // namespace EtherCAT

#pragma once
#include <cstddef>
#include <cstdint>
#include <array>
#include "tether/slave/mailbox/IMailboxHandler.hpp"
// no need for extra forward declarations; avoid pulling in CiA402Config.hpp
// which conflicts with CiA402Defs due to duplicate OperatingMode names.

#include "tether/ethercat/ObjectDictionary.hpp"

/**
 * @file 60xx-Parameters.hpp
 * @brief CiA402 CSV mode object dictionary entries (0x6040-0x60B1)
 *
 * This header mirrors the register‑definition style used for vendor
 * specific drives (see AS715N/Registers/C00-Parameters.hpp) but provides
 * generic CiA 402 entries for the common 60xx control and status objects
 * used when the drive is in CSV (Cyclic Sync Velocity) mode.  The entries
 * are primarily consumed by helpers that need metadata such as min/max
 * values, default values and possible enum options.
 */

namespace CiA402 {
namespace Parameters60xx {

// --- enums -------------------------------------------------------------

// Option enum for operation/drive modes is already defined elsewhere as
// `CiA402::OperatingMode`. Forward-declaration above suffices.

// --- object dictionary entries ----------------------------------------

// optional helpers/enums
// touch probe bitfield definitions (same values as table above)
enum class TouchProbeFunctionFlags : uint16_t {
    Disabled                      = 0,
    Enabled                       = 1 << 0,
    TriggerModeContinuous         = 1 << 1,
    TriggerSignalZ                = 1 << 2,
    LatchPositiveEdge1            = 1 << 4,
    LatchNegativeEdge1            = 1 << 5,
    LatchPositiveEdge2            = 1 << 12,
    LatchNegativeEdge2            = 1 << 13,
};

// mechanical limit position options
enum class MechanicalLimitOptions : uint16_t {
    Disabled = 0,
    Enabled = 1,
    EnabledAfterHoming = 2,
};

// DI function indexes (2004h) are not 60xx but include for completeness
constexpr ::EtherCAT::ObjectDictionary::ObjectDictionaryEntry DIFunction4 = {
    .index = 0x2004,
    .subindex = 0x00,
    .name = "DI4 function",
    .data_type = EtherCAT::slave::ObjectDictionaryDataType::Unsigned16,
    .default_value = 30,
    .min_value = 0,
    .max_value = 0xFFFF,
    .modification_mode = ::EtherCAT::ObjectDictionary::ModificationMode::DuringOperation,
    .effective_time = ::EtherCAT::ObjectDictionary::EffectiveTime::UponRepowerOn,
};

constexpr ::EtherCAT::ObjectDictionary::ObjectDictionaryEntry DIFunction5 = {
    .index = 0x2004,
    .subindex = 0x11,
    .name = "DI5 function",
    .data_type = EtherCAT::slave::ObjectDictionaryDataType::Unsigned16,
    .default_value = 31,
    .min_value = 0,
    .max_value = 0xFFFF,
    .modification_mode = ::EtherCAT::ObjectDictionary::ModificationMode::DuringOperation,
    .effective_time = ::EtherCAT::ObjectDictionary::EffectiveTime::UponRepowerOn,
};

// mechanical limit position (2006.08h)
constexpr ::EtherCAT::ObjectDictionary::ObjectDictionaryEntry MechanicalLimitPosition = {
    .index = 0x2006,
    .subindex = 0x08,
    .name = "Mechanical limit position",
    .data_type = EtherCAT::slave::ObjectDictionaryDataType::Unsigned16,
    .default_value = 0,
    .options_enum = std::type_identity<MechanicalLimitOptions>{},
    .min_value = 0,
    .max_value = 2,
    .modification_mode = ::EtherCAT::ObjectDictionary::ModificationMode::DuringOperation,
    .effective_time = ::EtherCAT::ObjectDictionary::EffectiveTime::Immediately,
};


constexpr ::EtherCAT::ObjectDictionary::ObjectDictionaryEntry ControlWord = {
    .index = 0x6040,
    .subindex = 0x00,
    .name = "Control word",
    .data_type = EtherCAT::slave::ObjectDictionaryDataType::Unsigned16,
    .default_value = 0,
    .min_value = 0,
    .max_value = 0xFFFF,
    .modification_mode = ::EtherCAT::ObjectDictionary::ModificationMode::DuringOperation,
    .effective_time = ::EtherCAT::ObjectDictionary::EffectiveTime::Immediately,
};

constexpr ::EtherCAT::ObjectDictionary::ObjectDictionaryEntry StatusWord = {
    .index = 0x6041,
    .subindex = 0x00,
    .name = "Status word",
    .data_type = EtherCAT::slave::ObjectDictionaryDataType::Unsigned16,
    .default_value = 0,
    .min_value = 0,
    .max_value = 0xFFFF,
    .modification_mode = ::EtherCAT::ObjectDictionary::ModificationMode::ReadOnly,
    .effective_time = ::EtherCAT::ObjectDictionary::EffectiveTime::Immediately,
};

constexpr ::EtherCAT::ObjectDictionary::ObjectDictionaryEntry OperationMode = {
    .index = 0x6060,
    .subindex = 0x00,
    .name = "Operation mode",
    .data_type = EtherCAT::slave::ObjectDictionaryDataType::Integer8,
    .default_value = 0,
    /* options_enum removed to avoid OperatingMode type conflict */
    .min_value = 0,
    .max_value = 10,
    .modification_mode = ::EtherCAT::ObjectDictionary::ModificationMode::DuringOperation,
    .effective_time = ::EtherCAT::ObjectDictionary::EffectiveTime::Immediately,
};

constexpr ::EtherCAT::ObjectDictionary::ObjectDictionaryEntry ModeDisplay = {
    .index = 0x6061,
    .subindex = 0x00,
    .name = "Mode display",
    .data_type = EtherCAT::slave::ObjectDictionaryDataType::Integer8,
    .default_value = 0,
    .min_value = 0,
    .max_value = 10,
    .modification_mode = ::EtherCAT::ObjectDictionary::ModificationMode::ReadOnly,
    .effective_time = ::EtherCAT::ObjectDictionary::EffectiveTime::Immediately,
};

constexpr ::EtherCAT::ObjectDictionary::ObjectDictionaryEntry PositionFeedback = {
    .index = 0x6064,
    .subindex = 0x00,
    .name = "Position feedback",
    .data_type = EtherCAT::slave::ObjectDictionaryDataType::Integer32,
    .default_value = 0,
    .modification_mode = ::EtherCAT::ObjectDictionary::ModificationMode::ReadOnly,
    .effective_time = ::EtherCAT::ObjectDictionary::EffectiveTime::Immediately,
};

constexpr ::EtherCAT::ObjectDictionary::ObjectDictionaryEntry ActualSpeed = {
    .index = 0x606C,
    .subindex = 0x00,
    .name = "Actual speed",
    .data_type = EtherCAT::slave::ObjectDictionaryDataType::Integer32,
    .default_value = 0,
    .modification_mode = ::EtherCAT::ObjectDictionary::ModificationMode::ReadOnly,
    .effective_time = ::EtherCAT::ObjectDictionary::EffectiveTime::Immediately,
};

// Torque-related objects from additional table
constexpr ::EtherCAT::ObjectDictionary::ObjectDictionaryEntry TargetTorque = {
    .index = 0x6071,
    .subindex = 0x00,
    .name = "Target torque",
    .data_type = EtherCAT::slave::ObjectDictionaryDataType::Integer16,
    .default_value = 0,
    .min_value = -4000,
    .max_value = 4000,
    .modification_mode = ::EtherCAT::ObjectDictionary::ModificationMode::DuringOperation,
    .effective_time = ::EtherCAT::ObjectDictionary::EffectiveTime::Immediately,
};

constexpr ::EtherCAT::ObjectDictionary::ObjectDictionaryEntry MaxTorque = {
    .index = 0x6072,
    .subindex = 0x00,
    .name = "Max torque",
    .data_type = EtherCAT::slave::ObjectDictionaryDataType::Unsigned16,
    .default_value = 3000,
    .min_value = 0,
    .max_value = 4000,
    .modification_mode = ::EtherCAT::ObjectDictionary::ModificationMode::DuringOperation,
    .effective_time = ::EtherCAT::ObjectDictionary::EffectiveTime::Immediately,
};

constexpr ::EtherCAT::ObjectDictionary::ObjectDictionaryEntry TorqueReference = {
    .index = 0x6074,
    .subindex = 0x00,
    .name = "Torque reference",
    .data_type = EtherCAT::slave::ObjectDictionaryDataType::Integer16,
    .default_value = 0,
    .modification_mode = ::EtherCAT::ObjectDictionary::ModificationMode::ReadOnly,
    .effective_time = ::EtherCAT::ObjectDictionary::EffectiveTime::Immediately,
};

constexpr ::EtherCAT::ObjectDictionary::ObjectDictionaryEntry ActualTorque = {
    .index = 0x6077,
    .subindex = 0x00,
    .name = "Actual torque",
    .data_type = EtherCAT::slave::ObjectDictionaryDataType::Integer16,
    .default_value = 0,
    .modification_mode = ::EtherCAT::ObjectDictionary::ModificationMode::ReadOnly,
    .effective_time = ::EtherCAT::ObjectDictionary::EffectiveTime::Immediately,
};

// touch probe configuration/function register
constexpr ::EtherCAT::ObjectDictionary::ObjectDictionaryEntry TouchProbeFunction = {
    .index = 0x60B8,
    .subindex = 0x00,
    .name = "Touch probe function",
    .data_type = EtherCAT::slave::ObjectDictionaryDataType::Unsigned16,
    .default_value = 0,
    .unit = ::EtherCAT::ObjectDictionary::Unit_None,
    .options_enum = std::type_identity<TouchProbeFunctionFlags>{},
    .min_value = 0,
    .max_value = 0xFFFF,
    .modification_mode = ::EtherCAT::ObjectDictionary::ModificationMode::DuringOperation,
    .effective_time = ::EtherCAT::ObjectDictionary::EffectiveTime::Immediately,
};

// status word for touch probes (bitfield; read‑only)
constexpr ::EtherCAT::ObjectDictionary::ObjectDictionaryEntry TouchProbeStatus = {
    .index = 0x60B9,
    .subindex = 0x00,
    .name = "Touch probe status",
    .data_type = EtherCAT::slave::ObjectDictionaryDataType::Unsigned16,
    .default_value = 0,
    .unit = ::EtherCAT::ObjectDictionary::Unit_None,
    .min_value = 0,
    .max_value = 0xFFFF,
    .modification_mode = ::EtherCAT::ObjectDictionary::ModificationMode::ReadOnly,
    .effective_time = ::EtherCAT::ObjectDictionary::EffectiveTime::Immediately,
};

// touch probe raw edge positions (read-only)
constexpr ::EtherCAT::ObjectDictionary::ObjectDictionaryEntry TouchProbe1PosEdge = {
    .index = 0x60BA,
    .subindex = 0x00,
    .name = "Touch probe 1 positive edge",
    .data_type = EtherCAT::slave::ObjectDictionaryDataType::Integer32,
    .default_value = 0,
    .modification_mode = ::EtherCAT::ObjectDictionary::ModificationMode::ReadOnly,
    .effective_time = ::EtherCAT::ObjectDictionary::EffectiveTime::Immediately,
};

constexpr ::EtherCAT::ObjectDictionary::ObjectDictionaryEntry TouchProbe1NegEdge = {
    .index = 0x60BB,
    .subindex = 0x00,
    .name = "Touch probe 1 negative edge",
    .data_type = EtherCAT::slave::ObjectDictionaryDataType::Integer32,
    .default_value = 0,
    .modification_mode = ::EtherCAT::ObjectDictionary::ModificationMode::ReadOnly,
    .effective_time = ::EtherCAT::ObjectDictionary::EffectiveTime::Immediately,
};

constexpr ::EtherCAT::ObjectDictionary::ObjectDictionaryEntry TouchProbe2PosEdge = {
    .index = 0x60BC,
    .subindex = 0x00,
    .name = "Touch probe 2 positive edge",
    .data_type = EtherCAT::slave::ObjectDictionaryDataType::Integer32,
    .default_value = 0,
    .modification_mode = ::EtherCAT::ObjectDictionary::ModificationMode::ReadOnly,
    .effective_time = ::EtherCAT::ObjectDictionary::EffectiveTime::Immediately,
};

constexpr ::EtherCAT::ObjectDictionary::ObjectDictionaryEntry TouchProbe2NegEdge = {
    .index = 0x60BD,
    .subindex = 0x00,
    .name = "Touch probe 2 negative edge",
    .data_type = EtherCAT::slave::ObjectDictionaryDataType::Integer32,
    .default_value = 0,
    .modification_mode = ::EtherCAT::ObjectDictionary::ModificationMode::ReadOnly,
    .effective_time = ::EtherCAT::ObjectDictionary::EffectiveTime::Immediately,
};

// touch probe edge counters
constexpr ::EtherCAT::ObjectDictionary::ObjectDictionaryEntry TouchProbe1PosEdgeCounter = {
    .index = 0x60D5,
    .subindex = 0x00,
    .name = "Touch probe 1 positive edge counter",
    .data_type = EtherCAT::slave::ObjectDictionaryDataType::Unsigned16,
    .default_value = 0,
    .modification_mode = ::EtherCAT::ObjectDictionary::ModificationMode::ReadOnly,
    .effective_time = ::EtherCAT::ObjectDictionary::EffectiveTime::Immediately,
};

constexpr ::EtherCAT::ObjectDictionary::ObjectDictionaryEntry TouchProbe1NegEdgeCounter = {
    .index = 0x60D6,
    .subindex = 0x00,
    .name = "Touch probe 1 negative edge counter",
    .data_type = EtherCAT::slave::ObjectDictionaryDataType::Unsigned16,
    .default_value = 0,
    .modification_mode = ::EtherCAT::ObjectDictionary::ModificationMode::ReadOnly,
    .effective_time = ::EtherCAT::ObjectDictionary::EffectiveTime::Immediately,
};

constexpr ::EtherCAT::ObjectDictionary::ObjectDictionaryEntry TouchProbe2PosEdgeCounter = {
    .index = 0x60D7,
    .subindex = 0x00,
    .name = "Touch probe 2 positive edge counter",
    .data_type = EtherCAT::slave::ObjectDictionaryDataType::Unsigned16,
    .default_value = 0,
    .modification_mode = ::EtherCAT::ObjectDictionary::ModificationMode::ReadOnly,
    .effective_time = ::EtherCAT::ObjectDictionary::EffectiveTime::Immediately,
};

constexpr ::EtherCAT::ObjectDictionary::ObjectDictionaryEntry TouchProbe2NegEdgeCounter = {
    .index = 0x60D8,
    .subindex = 0x00,
    .name = "Touch probe 2 negative edge counter",
    .data_type = EtherCAT::slave::ObjectDictionaryDataType::Unsigned16,
    .default_value = 0,
    .modification_mode = ::EtherCAT::ObjectDictionary::ModificationMode::ReadOnly,
    .effective_time = ::EtherCAT::ObjectDictionary::EffectiveTime::Immediately,
};

// additional items later


constexpr ::EtherCAT::ObjectDictionary::ObjectDictionaryEntry ReferencePolarity = {
    .index = 0x607E,
    .subindex = 0x00,
    .name = "Reference polarity",
    .data_type = EtherCAT::slave::ObjectDictionaryDataType::Unsigned8,
    .default_value = 0,
    .min_value = 0,
    .max_value = 255,
    .modification_mode = ::EtherCAT::ObjectDictionary::ModificationMode::DuringOperation,
    .effective_time = ::EtherCAT::ObjectDictionary::EffectiveTime::Immediately,
};

constexpr ::EtherCAT::ObjectDictionary::ObjectDictionaryEntry MaxSpeed = {
    .index = 0x607F,
    .subindex = 0x00,
    .name = "Max speed",
    .data_type = EtherCAT::slave::ObjectDictionaryDataType::Unsigned32,
    .default_value = 104857600u,
    .min_value = 0,
    .max_value = UINT32_MAX,
    .modification_mode = ::EtherCAT::ObjectDictionary::ModificationMode::DuringOperation,
    .effective_time = ::EtherCAT::ObjectDictionary::EffectiveTime::Immediately,
};

// software position limits
constexpr ::EtherCAT::ObjectDictionary::ObjectDictionaryEntry MinSoftwarePositionLimit = {
    .index = 0x607D,
    .subindex = 0x01,
    .name = "Min software position limit",
    .data_type = EtherCAT::slave::ObjectDictionaryDataType::Integer32,
    .default_value = static_cast<uint32_t>(INT32_MIN),
    .min_value = INT32_MIN,
    .max_value = INT32_MAX,
    .modification_mode = ::EtherCAT::ObjectDictionary::ModificationMode::DuringOperation,
    .effective_time = ::EtherCAT::ObjectDictionary::EffectiveTime::Immediately,
};

constexpr ::EtherCAT::ObjectDictionary::ObjectDictionaryEntry MaxSoftwarePositionLimit = {
    .index = 0x607D,
    .subindex = 0x02,
    .name = "Max software position limit",
    .data_type = EtherCAT::slave::ObjectDictionaryDataType::Integer32,
    .default_value = static_cast<uint32_t>(INT32_MAX),
    .min_value = INT32_MIN,
    .max_value = INT32_MAX,
    .modification_mode = ::EtherCAT::ObjectDictionary::ModificationMode::DuringOperation,
    .effective_time = ::EtherCAT::ObjectDictionary::EffectiveTime::Immediately,
};

constexpr ::EtherCAT::ObjectDictionary::ObjectDictionaryEntry PositiveTorqueLimit = {
    .index = 0x60E0,
    .subindex = 0x00,
    .name = "Positive torque limit",
    .data_type = EtherCAT::slave::ObjectDictionaryDataType::Unsigned16,
    .default_value = 3000,
    .min_value = 0,
    .max_value = 4000,
    .modification_mode = ::EtherCAT::ObjectDictionary::ModificationMode::DuringOperation,
    .effective_time = ::EtherCAT::ObjectDictionary::EffectiveTime::Immediately,
};

constexpr ::EtherCAT::ObjectDictionary::ObjectDictionaryEntry NegativeTorqueLimit = {
    .index = 0x60E1,
    .subindex = 0x00,
    .name = "Negative torque limit",
    .data_type = EtherCAT::slave::ObjectDictionaryDataType::Unsigned16,
    .default_value = 3000,
    .min_value = 0,
    .max_value = 4000,
    .modification_mode = ::EtherCAT::ObjectDictionary::ModificationMode::DuringOperation,
    .effective_time = ::EtherCAT::ObjectDictionary::EffectiveTime::Immediately,
};

// 0x60E3 supported homing methods has multiple subindexes
constexpr ::EtherCAT::ObjectDictionary::ObjectDictionaryEntry SupportedHoming_Highest = {
    .index = 0x60E3,
    .subindex = 0x00,
    .name = "Supported homing methods: highest subindex",
    .data_type = EtherCAT::slave::ObjectDictionaryDataType::Unsigned8,
    .default_value = 22,
    .modification_mode = ::EtherCAT::ObjectDictionary::ModificationMode::ReadOnly,
    .effective_time = ::EtherCAT::ObjectDictionary::EffectiveTime::Immediately,
};

constexpr ::EtherCAT::ObjectDictionary::ObjectDictionaryEntry SupportedHoming_1 = {
    .index = 0x60E3,
    .subindex = 0x01,
    .name = "1st supported homing method",
    .data_type = EtherCAT::slave::ObjectDictionaryDataType::Integer16,
    .default_value = 1,
    .modification_mode = ::EtherCAT::ObjectDictionary::ModificationMode::ReadOnly,
    .effective_time = ::EtherCAT::ObjectDictionary::EffectiveTime::Immediately,
};

constexpr ::EtherCAT::ObjectDictionary::ObjectDictionaryEntry SupportedHoming_2 = {
    .index = 0x60E3,
    .subindex = 0x02,
    .name = "2nd supported homing method",
    .data_type = EtherCAT::slave::ObjectDictionaryDataType::Integer16,
    .default_value = 2,
    .modification_mode = ::EtherCAT::ObjectDictionary::ModificationMode::ReadOnly,
    .effective_time = ::EtherCAT::ObjectDictionary::EffectiveTime::Immediately,
};

constexpr ::EtherCAT::ObjectDictionary::ObjectDictionaryEntry SupportedHoming_3 = {
    .index = 0x60E3,
    .subindex = 0x03,
    .name = "3rd supported homing method",
    .data_type = EtherCAT::slave::ObjectDictionaryDataType::Integer16,
    .default_value = 3,
    .modification_mode = ::EtherCAT::ObjectDictionary::ModificationMode::ReadOnly,
    .effective_time = ::EtherCAT::ObjectDictionary::EffectiveTime::Immediately,
};

constexpr ::EtherCAT::ObjectDictionary::ObjectDictionaryEntry SupportedHoming_4 = {
    .index = 0x60E3,
    .subindex = 0x04,
    .name = "4th supported homing method",
    .data_type = EtherCAT::slave::ObjectDictionaryDataType::Integer16,
    .default_value = 4,
    .modification_mode = ::EtherCAT::ObjectDictionary::ModificationMode::ReadOnly,
    .effective_time = ::EtherCAT::ObjectDictionary::EffectiveTime::Immediately,
};

constexpr ::EtherCAT::ObjectDictionary::ObjectDictionaryEntry SupportedHoming_5 = {
    .index = 0x60E3,
    .subindex = 0x05,
    .name = "5th supported homing method",
    .data_type = EtherCAT::slave::ObjectDictionaryDataType::Integer16,
    .default_value = 5,
    .modification_mode = ::EtherCAT::ObjectDictionary::ModificationMode::ReadOnly,
    .effective_time = ::EtherCAT::ObjectDictionary::EffectiveTime::Immediately,
};

constexpr ::EtherCAT::ObjectDictionary::ObjectDictionaryEntry SupportedHoming_6 = {
    .index = 0x60E3,
    .subindex = 0x06,
    .name = "6th supported homing method",
    .data_type = EtherCAT::slave::ObjectDictionaryDataType::Integer16,
    .default_value = 6,
    .modification_mode = ::EtherCAT::ObjectDictionary::ModificationMode::ReadOnly,
    .effective_time = ::EtherCAT::ObjectDictionary::EffectiveTime::Immediately,
};

// additional 6xx profile parameters that are common and not device specific
constexpr ::EtherCAT::ObjectDictionary::ObjectDictionaryEntry EncoderIncrementsAdditional = {
    .index = 0x60E6,
    .subindex = 0x00,
    .name = "Encoder increments additional position",
    .data_type = EtherCAT::slave::ObjectDictionaryDataType::Unsigned16,
    .default_value = 0,
    .min_value = 0,
    .max_value = 1,
    .modification_mode = ::EtherCAT::ObjectDictionary::ModificationMode::DuringOperation,
    .effective_time = ::EtherCAT::ObjectDictionary::EffectiveTime::Immediately,
};

constexpr ::EtherCAT::ObjectDictionary::ObjectDictionaryEntry FollowingErrorActual = {
    .index = 0x60F4,
    .subindex = 0x00,
    .name = "Position deviation",
    .data_type = EtherCAT::slave::ObjectDictionaryDataType::Integer32,
    .default_value = 0,
    .min_value = INT32_MIN,
    .max_value = INT32_MAX,
    .modification_mode = ::EtherCAT::ObjectDictionary::ModificationMode::ReadOnly,
    .effective_time = ::EtherCAT::ObjectDictionary::EffectiveTime::Immediately,
};

constexpr ::EtherCAT::ObjectDictionary::ObjectDictionaryEntry DIStatus = {
    .index = 0x60FC,
    .subindex = 0x00,
    .name = "DI status",
    .data_type = EtherCAT::slave::ObjectDictionaryDataType::Unsigned32,
    .default_value = 0,
    .modification_mode = ::EtherCAT::ObjectDictionary::ModificationMode::ReadOnly,
    .effective_time = ::EtherCAT::ObjectDictionary::EffectiveTime::Immediately,
};

constexpr ::EtherCAT::ObjectDictionary::ObjectDictionaryEntry SupportedDriveModes = {
    .index = 0x6502,
    .subindex = 0x00,
    .name = "Supported drive modes",
    .data_type = EtherCAT::slave::ObjectDictionaryDataType::Unsigned32,
    .default_value = 0,
    .modification_mode = ::EtherCAT::ObjectDictionary::ModificationMode::ReadOnly,
    .effective_time = ::EtherCAT::ObjectDictionary::EffectiveTime::Immediately,
};

// torque offset / speed deviation already defined earlier

constexpr ::EtherCAT::ObjectDictionary::ObjectDictionaryEntry SpeedDeviation = {
    .index = 0x60B1,
    .subindex = 0x00,
    .name = "Speed deviation",
    .data_type = EtherCAT::slave::ObjectDictionaryDataType::Integer32,
    .default_value = 0,
    .min_value = INT32_MIN,
    .max_value = INT32_MAX,
    .modification_mode = ::EtherCAT::ObjectDictionary::ModificationMode::DuringOperation,
    .effective_time = ::EtherCAT::ObjectDictionary::EffectiveTime::Immediately,
};

// new registers from user-provided table
constexpr ::EtherCAT::ObjectDictionary::ObjectDictionaryEntry TorqueOffset = {
    .index = 0x60B2,
    .subindex = 0x00,
    .name = "Torque offset",
    .data_type = EtherCAT::slave::ObjectDictionaryDataType::Integer16,
    .default_value = 0,
    .min_value = -4000,
    .max_value = 4000,
    .modification_mode = ::EtherCAT::ObjectDictionary::ModificationMode::DuringOperation,
    .effective_time = ::EtherCAT::ObjectDictionary::EffectiveTime::Immediately,
};

constexpr ::EtherCAT::ObjectDictionary::ObjectDictionaryEntry TargetVelocity = {
    .index = 0x60FF,
    .subindex = 0x00,
    .name = "Target velocity",
    .data_type = EtherCAT::slave::ObjectDictionaryDataType::Integer32,
    .default_value = 0,
    .min_value = INT32_MIN,
    .max_value = INT32_MAX,
    .modification_mode = ::EtherCAT::ObjectDictionary::ModificationMode::DuringOperation,
    .effective_time = ::EtherCAT::ObjectDictionary::EffectiveTime::Immediately,
};

// additional common CiA402 objects from documentation
constexpr ::EtherCAT::ObjectDictionary::ObjectDictionaryEntry ErrorCode = {
    .index = 0x603F,
    .subindex = 0x00,
    .name = "Error code",
    .data_type = EtherCAT::slave::ObjectDictionaryDataType::Unsigned16,
    .default_value = 0,
    .modification_mode = ::EtherCAT::ObjectDictionary::ModificationMode::ReadOnly,
    .effective_time = ::EtherCAT::ObjectDictionary::EffectiveTime::Immediately,
};

constexpr ::EtherCAT::ObjectDictionary::ObjectDictionaryEntry QuickStopOptionCode = {
    .index = 0x605A,
    .subindex = 0x00,
    .name = "Quick stop option code",
    .data_type = EtherCAT::slave::ObjectDictionaryDataType::Unsigned16,
    .default_value = 0,
    .modification_mode = ::EtherCAT::ObjectDictionary::ModificationMode::DuringOperation,
    .effective_time = ::EtherCAT::ObjectDictionary::EffectiveTime::Immediately,
};

constexpr ::EtherCAT::ObjectDictionary::ObjectDictionaryEntry PositionDemandValue = {
    .index = 0x6062,
    .subindex = 0x00,
    .name = "Position demand value",
    .data_type = EtherCAT::slave::ObjectDictionaryDataType::Integer32,
    .default_value = 0,
    .modification_mode = ::EtherCAT::ObjectDictionary::ModificationMode::ReadOnly,
    .effective_time = ::EtherCAT::ObjectDictionary::EffectiveTime::Immediately,
};

constexpr ::EtherCAT::ObjectDictionary::ObjectDictionaryEntry FollowingErrorWindow = {
    .index = 0x6065,
    .subindex = 0x00,
    .name = "Following error window",
    .data_type = EtherCAT::slave::ObjectDictionaryDataType::Integer32,
    .default_value = 0,
    .modification_mode = ::EtherCAT::ObjectDictionary::ModificationMode::DuringOperation,
    .effective_time = ::EtherCAT::ObjectDictionary::EffectiveTime::Immediately,
};

constexpr ::EtherCAT::ObjectDictionary::ObjectDictionaryEntry FollowingErrorTimeout = {
    .index = 0x6066,
    .subindex = 0x00,
    .name = "Following error timeout",
    .data_type = EtherCAT::slave::ObjectDictionaryDataType::Unsigned32,
    .default_value = 0,
    .modification_mode = ::EtherCAT::ObjectDictionary::ModificationMode::DuringOperation,
    .effective_time = ::EtherCAT::ObjectDictionary::EffectiveTime::Immediately,
};

constexpr ::EtherCAT::ObjectDictionary::ObjectDictionaryEntry PositionWindow = {
    .index = 0x6067,
    .subindex = 0x00,
    .name = "Position window",
    .data_type = EtherCAT::slave::ObjectDictionaryDataType::Integer32,
    .default_value = 0,
    .modification_mode = ::EtherCAT::ObjectDictionary::ModificationMode::DuringOperation,
    .effective_time = ::EtherCAT::ObjectDictionary::EffectiveTime::Immediately,
};

constexpr ::EtherCAT::ObjectDictionary::ObjectDictionaryEntry PositionWindowTime = {
    .index = 0x6068,
    .subindex = 0x00,
    .name = "Position window time",
    .data_type = EtherCAT::slave::ObjectDictionaryDataType::Unsigned32,
    .default_value = 0,
    .modification_mode = ::EtherCAT::ObjectDictionary::ModificationMode::DuringOperation,
    .effective_time = ::EtherCAT::ObjectDictionary::EffectiveTime::Immediately,
};

constexpr ::EtherCAT::ObjectDictionary::ObjectDictionaryEntry VelocityDemandValue = {
    .index = 0x606B,
    .subindex = 0x00,
    .name = "Velocity demand value",
    .data_type = EtherCAT::slave::ObjectDictionaryDataType::Integer32,
    .default_value = 0,
    .modification_mode = ::EtherCAT::ObjectDictionary::ModificationMode::DuringOperation,
    .effective_time = ::EtherCAT::ObjectDictionary::EffectiveTime::Immediately,
};

constexpr ::EtherCAT::ObjectDictionary::ObjectDictionaryEntry VelocityWindow = {
    .index = 0x606D,
    .subindex = 0x00,
    .name = "Velocity window",
    .data_type = EtherCAT::slave::ObjectDictionaryDataType::Integer32,
    .default_value = 0,
    .modification_mode = ::EtherCAT::ObjectDictionary::ModificationMode::DuringOperation,
    .effective_time = ::EtherCAT::ObjectDictionary::EffectiveTime::Immediately,
};

constexpr ::EtherCAT::ObjectDictionary::ObjectDictionaryEntry VelocityWindowTime = {
    .index = 0x606E,
    .subindex = 0x00,
    .name = "Velocity window time",
    .data_type = EtherCAT::slave::ObjectDictionaryDataType::Unsigned32,
    .default_value = 0,
    .modification_mode = ::EtherCAT::ObjectDictionary::ModificationMode::DuringOperation,
    .effective_time = ::EtherCAT::ObjectDictionary::EffectiveTime::Immediately,
};

constexpr ::EtherCAT::ObjectDictionary::ObjectDictionaryEntry VelocityThreshold = {
    .index = 0x606F,
    .subindex = 0x00,
    .name = "Velocity threshold",
    .data_type = EtherCAT::slave::ObjectDictionaryDataType::Integer32,
    .default_value = 0,
    .modification_mode = ::EtherCAT::ObjectDictionary::ModificationMode::DuringOperation,
    .effective_time = ::EtherCAT::ObjectDictionary::EffectiveTime::Immediately,
};

constexpr ::EtherCAT::ObjectDictionary::ObjectDictionaryEntry VelocityThresholdTime = {
    .index = 0x6070,
    .subindex = 0x00,
    .name = "Velocity threshold time",
    .data_type = EtherCAT::slave::ObjectDictionaryDataType::Unsigned32,
    .default_value = 0,
    .modification_mode = ::EtherCAT::ObjectDictionary::ModificationMode::DuringOperation,
    .effective_time = ::EtherCAT::ObjectDictionary::EffectiveTime::Immediately,
};

constexpr ::EtherCAT::ObjectDictionary::ObjectDictionaryEntry MaxCurrent = {
    .index = 0x6073,
    .subindex = 0x00,
    .name = "Max current",
    .data_type = EtherCAT::slave::ObjectDictionaryDataType::Unsigned32,
    .default_value = 0,
    .min_value = 0,
    .max_value = UINT32_MAX,
    .modification_mode = ::EtherCAT::ObjectDictionary::ModificationMode::DuringOperation,
    .effective_time = ::EtherCAT::ObjectDictionary::EffectiveTime::Immediately,
};

constexpr ::EtherCAT::ObjectDictionary::ObjectDictionaryEntry MotorRatedCurrent = {
    .index = 0x6075,
    .subindex = 0x00,
    .name = "Motor rated current",
    .data_type = EtherCAT::slave::ObjectDictionaryDataType::Unsigned32,
    .default_value = 0,
    .min_value = 0,
    .max_value = UINT32_MAX,
    .modification_mode = ::EtherCAT::ObjectDictionary::ModificationMode::ReadOnly,
    .effective_time = ::EtherCAT::ObjectDictionary::EffectiveTime::Immediately,
};

constexpr ::EtherCAT::ObjectDictionary::ObjectDictionaryEntry MotorRatedTorque = {
    .index = 0x6076,
    .subindex = 0x00,
    .name = "Motor rated torque",
    .data_type = EtherCAT::slave::ObjectDictionaryDataType::Unsigned32,
    .default_value = 0,
    .min_value = 0,
    .max_value = UINT32_MAX,
    .modification_mode = ::EtherCAT::ObjectDictionary::ModificationMode::ReadOnly,
    .effective_time = ::EtherCAT::ObjectDictionary::EffectiveTime::Immediately,
};

constexpr ::EtherCAT::ObjectDictionary::ObjectDictionaryEntry DCLinkVoltage = {
    .index = 0x6079,
    .subindex = 0x00,
    .name = "DC link circuit voltage",
    .data_type = EtherCAT::slave::ObjectDictionaryDataType::Unsigned32,
    .default_value = 0,
    .modification_mode = ::EtherCAT::ObjectDictionary::ModificationMode::ReadOnly,
    .effective_time = ::EtherCAT::ObjectDictionary::EffectiveTime::Immediately,
};

constexpr ::EtherCAT::ObjectDictionary::ObjectDictionaryEntry TargetPosition = {
    .index = 0x607A,
    .subindex = 0x00,
    .name = "Target position",
    .data_type = EtherCAT::slave::ObjectDictionaryDataType::Integer32,
    .default_value = 0,
    .modification_mode = ::EtherCAT::ObjectDictionary::ModificationMode::DuringOperation,
    .effective_time = ::EtherCAT::ObjectDictionary::EffectiveTime::Immediately,
};

constexpr ::EtherCAT::ObjectDictionary::ObjectDictionaryEntry PositionRangeLimit = {
    .index = 0x607B,
    .subindex = 0x00,
    .name = "Position range limit",
    .data_type = EtherCAT::slave::ObjectDictionaryDataType::Integer32,
    .default_value = 0,
    .modification_mode = ::EtherCAT::ObjectDictionary::ModificationMode::DuringOperation,
    .effective_time = ::EtherCAT::ObjectDictionary::EffectiveTime::Immediately,
};

constexpr ::EtherCAT::ObjectDictionary::ObjectDictionaryEntry HomeOffset = {
    .index = 0x607C,
    .subindex = 0x00,
    .name = "Home offset",
    .data_type = EtherCAT::slave::ObjectDictionaryDataType::Integer32,
    .default_value = 0,
    .modification_mode = ::EtherCAT::ObjectDictionary::ModificationMode::DuringOperation,
    .effective_time = ::EtherCAT::ObjectDictionary::EffectiveTime::Immediately,
};

constexpr ::EtherCAT::ObjectDictionary::ObjectDictionaryEntry MaxMotorSpeed = {
    .index = 0x6080,
    .subindex = 0x00,
    .name = "Max motor speed",
    .data_type = EtherCAT::slave::ObjectDictionaryDataType::Unsigned32,
    .default_value = 0,
    .modification_mode = ::EtherCAT::ObjectDictionary::ModificationMode::DuringOperation,
    .effective_time = ::EtherCAT::ObjectDictionary::EffectiveTime::Immediately,
};

// more profile-specific definitions could be appended similarly...

constexpr std::array<const ::EtherCAT::ObjectDictionary::ObjectDictionaryEntry*, 49> kRegisterList = {
    &DIFunction4,                    // 0x2004 sub0
    &DIFunction5,                    // 0x2004 sub11
    &MechanicalLimitPosition,        // 0x2006.08
    &ErrorCode,                      // 0x603F
    &ControlWord,                    // 0x6040
    &StatusWord,                     // 0x6041
    &QuickStopOptionCode,            // 0x605A
    &OperationMode,                  // 0x6060
    &ModeDisplay,                    // 0x6061
    &PositionDemandValue,            // 0x6062
    &PositionFeedback,               // 0x6064
    &ActualSpeed,                    // 0x606C
    &TargetTorque,                   // 0x6071
    &MaxTorque,                      // 0x6072
    &TorqueReference,                // 0x6074
    &ActualTorque,                   // 0x6077
    &TargetPosition,                 // 0x607A
    &PositionRangeLimit,             // 0x607B
    &HomeOffset,                     // 0x607C
    &MinSoftwarePositionLimit,       // 0x607D.01
    &MaxSoftwarePositionLimit,       // 0x607D.02
    &ReferencePolarity,              // 0x607E
    &MaxSpeed,                       // 0x607F
    &MaxMotorSpeed,                  // 0x6080
    &SpeedDeviation,                 // 0x60B1
    &TorqueOffset,                   // 0x60B2
    &TouchProbeFunction,             // 0x60B8
    &TouchProbeStatus,               // 0x60B9
    &TouchProbe1PosEdge,             // 0x60BA
    &TouchProbe1NegEdge,             // 0x60BB
    &TouchProbe2PosEdge,             // 0x60BC
    &TouchProbe2NegEdge,             // 0x60BD
    &TouchProbe1PosEdgeCounter,      // 0x60D5
    &TouchProbe1NegEdgeCounter,      // 0x60D6
    &TouchProbe2PosEdgeCounter,      // 0x60D7
    &TouchProbe2NegEdgeCounter,      // 0x60D8
    &NegativeTorqueLimit,            // 0x60E1
    &SupportedHoming_Highest,        // 0x60E3.00
    &SupportedHoming_1,              // 0x60E3.01
    &SupportedHoming_2,              // 0x60E3.02
    &SupportedHoming_3,              // 0x60E3.03
    &SupportedHoming_4,              // 0x60E3.04
    &SupportedHoming_5,              // 0x60E3.05
    &SupportedHoming_6,              // 0x60E3.06
    &EncoderIncrementsAdditional,    // 0x60E6
    &FollowingErrorActual,           // 0x60F4
    &DIStatus,                       // 0x60FC
    &TargetVelocity,                 // 0x60FF
    &SupportedDriveModes,            // 0x6502
};

} // namespace Parameters60xx
} // namespace CiA402

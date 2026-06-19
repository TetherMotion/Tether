#pragma once

#include <cstdint>
#include "tether/drives/NexcobotESC211/Registers/Common.hpp"

namespace EtherCAT {
namespace Drives {
namespace Registers {
namespace NexcobotESC211 {
namespace FSOERx {

static constexpr uint16_t FSOESafetyPDURxIndex = 0x6000;
static constexpr uint16_t InputCounterIndex      = 0x6010;
static constexpr uint16_t SAFEDIIndex          = 0x6020;
static constexpr uint16_t PowerStatusIndex       = 0x6030;
static constexpr uint16_t DOMonitorIndex         = 0x6040;
static constexpr uint16_t DOValueActualIndex     = 0x6050;
static constexpr uint16_t DIValueIndex           = 0x6051;
static constexpr uint16_t DOCommandIndex         = 0x6052;

constexpr ::EtherCAT::ObjectDictionary::ObjectDictionaryEntry FSOESafetyPDURxCount = {
    .index = FSOESafetyPDURxIndex, .subindex = 0x00,
    .name = "FSOE Master SafetyPDU Rx count",
    .data_type = EtherCAT::ObjectDictionary::ObjectDictionaryDataType::Unsigned8,
    .default_value = 16, .unit = Unit_None, .options_enum = nullptr,
    .min_value = 0, .max_value = 16,
    .modification_mode = ModificationMode::ReadOnly,
    .effective_time = EffectiveTime::Immediately,
    .comment = "Number of entries for FSOE RxPDU",
};

#define NEXCOBOT_RXPDU_REG(N) \
    constexpr ::EtherCAT::ObjectDictionary::ObjectDictionaryEntry FSOERxPDU_##N = { \
        .index = FSOESafetyPDURxIndex, .subindex = (N), \
        .name = "FSOE RxPDU Section " #N, \
        .data_type = EtherCAT::ObjectDictionary::ObjectDictionaryDataType::OctetString, \
        .default_value = 0, .unit = Unit_None, .options_enum = nullptr, \
        .min_value = 0, .max_value = 0xFF, \
        .modification_mode = ModificationMode::ReadOnly, \
        .effective_time = EffectiveTime::Immediately, \
        .comment = "FSOE RxPDU section " #N, \
    }

NEXCOBOT_RXPDU_REG(1); NEXCOBOT_RXPDU_REG(2); NEXCOBOT_RXPDU_REG(3); NEXCOBOT_RXPDU_REG(4);
NEXCOBOT_RXPDU_REG(5); NEXCOBOT_RXPDU_REG(6); NEXCOBOT_RXPDU_REG(7); NEXCOBOT_RXPDU_REG(8);
NEXCOBOT_RXPDU_REG(9); NEXCOBOT_RXPDU_REG(10); NEXCOBOT_RXPDU_REG(11); NEXCOBOT_RXPDU_REG(12);
NEXCOBOT_RXPDU_REG(13); NEXCOBOT_RXPDU_REG(14); NEXCOBOT_RXPDU_REG(15); NEXCOBOT_RXPDU_REG(16);
#undef NEXCOBOT_RXPDU_REG

#define NEXCOBOT_UDINT_REG(NAME, IDX, DESC) \
    constexpr ::EtherCAT::ObjectDictionary::ObjectDictionaryEntry NAME = { \
        .index = (IDX), .subindex = 0x00, .name = (DESC), \
        .data_type = EtherCAT::ObjectDictionary::ObjectDictionaryDataType::Unsigned32, \
        .default_value = 0, .unit = Unit_None, .options_enum = nullptr, \
        .min_value = 0, .max_value = 0xFFFFFFFF, \
        .modification_mode = ModificationMode::ReadOnly, \
        .effective_time = EffectiveTime::Immediately, .comment = (DESC), \
    }

NEXCOBOT_UDINT_REG(InputCounter,      InputCounterIndex,      "InputCounter");
NEXCOBOT_UDINT_REG(SAFE_DI,           SAFEDIIndex,            "SAFE_DI");
NEXCOBOT_UDINT_REG(PowerStatus,       PowerStatusIndex,       "Power_Status");
NEXCOBOT_UDINT_REG(DOMonitor,         DOMonitorIndex,         "DO_Monitor");
NEXCOBOT_UDINT_REG(DOValueActual,     DOValueActualIndex,     "DO_Value(Actual)");
NEXCOBOT_UDINT_REG(DIValue,           DIValueIndex,           "DI_Value");
NEXCOBOT_UDINT_REG(DOCommand,         DOCommandIndex,         "DO Command");

#undef NEXCOBOT_UDINT_REG

inline const RegisterList kRegisterList = {
    &FSOESafetyPDURxCount,
    &FSOERxPDU_1, &FSOERxPDU_2, &FSOERxPDU_3, &FSOERxPDU_4,
    &FSOERxPDU_5, &FSOERxPDU_6, &FSOERxPDU_7, &FSOERxPDU_8,
    &FSOERxPDU_9, &FSOERxPDU_10, &FSOERxPDU_11, &FSOERxPDU_12,
    &FSOERxPDU_13, &FSOERxPDU_14, &FSOERxPDU_15, &FSOERxPDU_16,
    &InputCounter, &SAFE_DI, &PowerStatus, &DOMonitor,
    &DOValueActual, &DIValue, &DOCommand,
};

} // namespace FSOERx
} // namespace NexcobotESC211
} // namespace Registers
} // namespace Drives
} // namespace EtherCAT

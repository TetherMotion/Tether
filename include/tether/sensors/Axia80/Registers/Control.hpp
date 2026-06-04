/**
 * @file Control.hpp
 * @brief Axia80 control code objects (0x7010) — RxPDO-mapped
 *
 * Control 1 and Control 2 are mapped into the fixed RxPDO 0x1601 for
 * real-time deterministic access.
 */

#pragma once

#include "tether/sensors/Axia80/Registers/Common.hpp"

namespace EtherCAT {
namespace Sensors {
namespace Axia80 {
namespace Registers {
namespace Control {

static constexpr uint16_t ControlCodesIndex = 0x7010;

// Filter select options (bits 4–7 of Control 1)
enum class ControlFilterSelect : uint8_t {
    NoFilter = 0,  ///< Digital filtering bypassed
    Filter1 = 1,   ///< Light filtering
    Filter2 = 2,   ///< Light-medium filtering
    Filter3 = 3,   ///< Medium filtering
    Filter4 = 4,   ///< Medium-heavy filtering
    Filter5 = 5,   ///< Heavy filtering
    Filter6 = 6,   ///< Very heavy filtering
    Filter7 = 7,   ///< Extremely heavy filtering
    Filter8 = 8,   ///< Maximum filtering
};

// Calibration slot select (bits 8–11 of Control 1)
enum class ControlCalibrationSlot : uint8_t {
    Slot0 = 0,  ///< Calibration stored in hardware slot 0
    Slot1 = 1,  ///< Calibration stored in hardware slot 1
};

// Sample rate select (bits 12–15 of Control 1)
enum class ControlSampleRate : uint8_t {
    Rate488Hz = 0,   ///< 488 Hz
    Rate976Hz = 1,   ///< 976 Hz
    Rate1953Hz = 2,  ///< 1953 Hz
    Rate3906Hz = 3,  ///< 3906 Hz
    Rate7812Hz = 4,  ///< 7812 Hz
};

constexpr RegisterEntry Control_Control1 = {
    .index = ControlCodesIndex,
    .subindex = 0x01,
    .name = "Control 1",
    .data_type = ObjectDictionaryDataType::Unsigned32,
    .default_value = 0,
    .unit = Unit_None,
    .options_enum = nullptr,
    .min_value = 0,
    .max_value = 0xFFFFFFFF,
    .modification_mode = ModificationMode::DuringOperation,
    .effective_time = EffectiveTime::Immediately,
    .comment = "Bit 0=Tare, Bit 2=ClearBias, Bits 4-7=Filter, Bits 8-11=CalSlot, Bits 12-15=SampleRate",
};

constexpr RegisterEntry Control_Control2 = {
    .index = ControlCodesIndex,
    .subindex = 0x02,
    .name = "Control 2",
    .data_type = ObjectDictionaryDataType::Unsigned32,
    .default_value = 0,
    .unit = Unit_None,
    .options_enum = nullptr,
    .min_value = 0,
    .max_value = 0xFFFFFFFF,
    .modification_mode = ModificationMode::DuringOperation,
    .effective_time = EffectiveTime::Immediately,
    .comment = "Bit 31=SimulatedError (injects error into 0x6010 bit 28)",
};

inline const RegisterList kRegisterList = {
    &Control_Control1,
    &Control_Control2,
};

} // namespace Control
} // namespace Registers
} // namespace Axia80
} // namespace Sensors
} // namespace EtherCAT

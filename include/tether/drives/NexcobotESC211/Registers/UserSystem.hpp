#pragma once

#include <cstdint>
#include "tether/drives/NexcobotESC211/Registers/Common.hpp"

namespace EtherCAT {
namespace Drives {
namespace Registers {
namespace NexcobotESC211 {
namespace UserSystem {

static constexpr uint16_t UserControlIndex              = 0xF100;
static constexpr uint16_t SystemCurrentStateIndex       = 0xF101;
static constexpr uint16_t SystemErrorCodeIndex          = 0xF102;
static constexpr uint16_t SystemErrorMessageIndex         = 0xF103;
static constexpr uint16_t LastErrorCodeIndex            = 0xF104;
static constexpr uint16_t UserPasswordInputIndex       = 0xF105;
static constexpr uint16_t UserPasswordOutputIndex       = 0xF106;
static constexpr uint16_t ESCDebugMsgIndex              = 0xF110;
static constexpr uint16_t SystemCurrentStateMPUBIndex   = 0xF111;
static constexpr uint16_t SystemErrorCodeMPUBIndex      = 0xF112;

// ---------------------------------------------------------------------------
// 0xF100: User Control
// ---------------------------------------------------------------------------

constexpr ::EtherCAT::ObjectDictionary::ObjectDictionaryEntry UserControlCount = {
    .index = UserControlIndex,
    .subindex = 0x00,
    .name = "User Control count",
    .data_type = EtherCAT::ObjectDictionary::ObjectDictionaryDataType::Unsigned8,
    .default_value = 2,
    .unit = Unit_None,
    .options_enum = nullptr,
    .min_value = 0,
    .max_value = 2,
    .modification_mode = ModificationMode::ReadOnly,
    .effective_time = EffectiveTime::Immediately,
    .comment = "Number of entries for User Control",
};

constexpr ::EtherCAT::ObjectDictionary::ObjectDictionaryEntry ControlCommand = {
    .index = UserControlIndex,
    .subindex = 0x01,
    .name = "Control Command",
    .data_type = EtherCAT::ObjectDictionary::ObjectDictionaryDataType::Unsigned32,
    .default_value = 0,
    .unit = Unit_None,
    .options_enum = nullptr,
    .min_value = 0,
    .max_value = 0xFFFFFFFF,
    .modification_mode = ModificationMode::DuringOperation,
    .effective_time = EffectiveTime::Immediately,
    .comment = "User control command",
};

constexpr ::EtherCAT::ObjectDictionary::ObjectDictionaryEntry CommandResponse = {
    .index = UserControlIndex,
    .subindex = 0x02,
    .name = "Command Response",
    .data_type = EtherCAT::ObjectDictionary::ObjectDictionaryDataType::Unsigned32,
    .default_value = 0,
    .unit = Unit_None,
    .options_enum = nullptr,
    .min_value = 0,
    .max_value = 0xFFFFFFFF,
    .modification_mode = ModificationMode::ReadOnly,
    .effective_time = EffectiveTime::Immediately,
    .comment = "User command response",
};

// ---------------------------------------------------------------------------
// 0xF101: System Current State
// ---------------------------------------------------------------------------

constexpr ::EtherCAT::ObjectDictionary::ObjectDictionaryEntry SystemCurrentState = {
    .index = SystemCurrentStateIndex,
    .subindex = 0x00,
    .name = "System Current State",
    .data_type = EtherCAT::ObjectDictionary::ObjectDictionaryDataType::Unsigned32,
    .default_value = 0,
    .unit = Unit_None,
    .options_enum = nullptr,
    .min_value = 0,
    .max_value = 0xFFFFFFFF,
    .modification_mode = ModificationMode::ReadOnly,
    .effective_time = EffectiveTime::Immediately,
    .comment = "System current state",
};

// ---------------------------------------------------------------------------
// 0xF102: System Error Code
// ---------------------------------------------------------------------------

constexpr ::EtherCAT::ObjectDictionary::ObjectDictionaryEntry SystemErrorCode = {
    .index = SystemErrorCodeIndex,
    .subindex = 0x00,
    .name = "System Error Code",
    .data_type = EtherCAT::ObjectDictionary::ObjectDictionaryDataType::Integer32,
    .default_value = 0,
    .unit = Unit_None,
    .options_enum = nullptr,
    .min_value = static_cast<int64_t>(-0x80000000LL),
    .max_value = 0x7FFFFFFF,
    .modification_mode = ModificationMode::ReadOnly,
    .effective_time = EffectiveTime::Immediately,
    .comment = "System error code",
};

// ---------------------------------------------------------------------------
// 0xF103: System Error Message
// ---------------------------------------------------------------------------

constexpr ::EtherCAT::ObjectDictionary::ObjectDictionaryEntry SystemErrorMessage = {
    .index = SystemErrorMessageIndex,
    .subindex = 0x00,
    .name = "System Error Message",
    .data_type = EtherCAT::ObjectDictionary::ObjectDictionaryDataType::VisibleString,
    .default_value = 0,
    .unit = Unit_None,
    .options_enum = nullptr,
    .min_value = 0,
    .max_value = 0xFF,
    .modification_mode = ModificationMode::ReadOnly,
    .effective_time = EffectiveTime::Immediately,
    .comment = "System error message (STRING 256)",
};

// ---------------------------------------------------------------------------
// 0xF104: LastErrorCode
// ---------------------------------------------------------------------------

constexpr ::EtherCAT::ObjectDictionary::ObjectDictionaryEntry LastErrorCodeCount = {
    .index = LastErrorCodeIndex,
    .subindex = 0x00,
    .name = "LastErrorCode count",
    .data_type = EtherCAT::ObjectDictionary::ObjectDictionaryDataType::Unsigned8,
    .default_value = 4,
    .unit = Unit_None,
    .options_enum = nullptr,
    .min_value = 0,
    .max_value = 4,
    .modification_mode = ModificationMode::ReadOnly,
    .effective_time = EffectiveTime::Immediately,
    .comment = "Number of entries for LastErrorCode",
};

#define NEXCOBOT_ERRCODE_REG(NUM, NAME) \
    constexpr ::EtherCAT::ObjectDictionary::ObjectDictionaryEntry LastErrorCode_##NAME = { \
        .index = LastErrorCodeIndex, \
        .subindex = (NUM), \
        .name = #NAME, \
        .data_type = EtherCAT::ObjectDictionary::ObjectDictionaryDataType::Unsigned32, \
        .default_value = 0, \
        .unit = Unit_None, \
        .options_enum = nullptr, \
        .min_value = 0, \
        .max_value = 0xFFFFFFFF, \
        .modification_mode = ModificationMode::ReadOnly, \
        .effective_time = EffectiveTime::Immediately, \
        .comment = "Last error code entry " #NUM, \
    }

NEXCOBOT_ERRCODE_REG(1, ErrorCode_A);
NEXCOBOT_ERRCODE_REG(2, ErrorCode_B);
NEXCOBOT_ERRCODE_REG(3, Reserve_1);
NEXCOBOT_ERRCODE_REG(4, Reserve_2);

#undef NEXCOBOT_ERRCODE_REG

// ---------------------------------------------------------------------------
// 0xF105: User Password Input
// ---------------------------------------------------------------------------

constexpr ::EtherCAT::ObjectDictionary::ObjectDictionaryEntry UserPasswordInputCount = {
    .index = UserPasswordInputIndex,
    .subindex = 0x00,
    .name = "User Password Input count",
    .data_type = EtherCAT::ObjectDictionary::ObjectDictionaryDataType::Unsigned8,
    .default_value = 3,
    .unit = Unit_None,
    .options_enum = nullptr,
    .min_value = 0,
    .max_value = 3,
    .modification_mode = ModificationMode::ReadOnly,
    .effective_time = EffectiveTime::Immediately,
    .comment = "Number of entries for User Password Input",
};

#define NEXCOBOT_PW_IN_REG(NUM) \
    constexpr ::EtherCAT::ObjectDictionary::ObjectDictionaryEntry Password_##NUM##_Input = { \
        .index = UserPasswordInputIndex, \
        .subindex = (NUM), \
        .name = "Password_" #NUM "_Input", \
        .data_type = EtherCAT::ObjectDictionary::ObjectDictionaryDataType::VisibleString, \
        .default_value = 0, \
        .unit = Unit_None, \
        .options_enum = nullptr, \
        .min_value = 0, \
        .max_value = 0xFF, \
        .modification_mode = ModificationMode::DuringOperation, \
        .effective_time = EffectiveTime::Immediately, \
        .comment = "User password input " #NUM " (STRING 512)", \
    }

NEXCOBOT_PW_IN_REG(1);
NEXCOBOT_PW_IN_REG(2);
NEXCOBOT_PW_IN_REG(3);

#undef NEXCOBOT_PW_IN_REG

// ---------------------------------------------------------------------------
// 0xF106: User Password Output
// ---------------------------------------------------------------------------

constexpr ::EtherCAT::ObjectDictionary::ObjectDictionaryEntry UserPasswordOutputCount = {
    .index = UserPasswordOutputIndex,
    .subindex = 0x00,
    .name = "User Password Output count",
    .data_type = EtherCAT::ObjectDictionary::ObjectDictionaryDataType::Unsigned8,
    .default_value = 3,
    .unit = Unit_None,
    .options_enum = nullptr,
    .min_value = 0,
    .max_value = 3,
    .modification_mode = ModificationMode::ReadOnly,
    .effective_time = EffectiveTime::Immediately,
    .comment = "Number of entries for User Password Output",
};

#define NEXCOBOT_PW_OUT_REG(NUM) \
    constexpr ::EtherCAT::ObjectDictionary::ObjectDictionaryEntry Password_##NUM##_Output = { \
        .index = UserPasswordOutputIndex, \
        .subindex = (NUM), \
        .name = "Password_" #NUM "_Output", \
        .data_type = EtherCAT::ObjectDictionary::ObjectDictionaryDataType::VisibleString, \
        .default_value = 0, \
        .unit = Unit_None, \
        .options_enum = nullptr, \
        .min_value = 0, \
        .max_value = 0xFF, \
        .modification_mode = ModificationMode::ReadOnly, \
        .effective_time = EffectiveTime::Immediately, \
        .comment = "User password output " #NUM " (STRING 512)", \
    }

NEXCOBOT_PW_OUT_REG(1);
NEXCOBOT_PW_OUT_REG(2);
NEXCOBOT_PW_OUT_REG(3);

#undef NEXCOBOT_PW_OUT_REG

// ---------------------------------------------------------------------------
// 0xF110: ESC Debug Msg
// ---------------------------------------------------------------------------

constexpr ::EtherCAT::ObjectDictionary::ObjectDictionaryEntry ESCDebugMsgCount = {
    .index = ESCDebugMsgIndex,
    .subindex = 0x00,
    .name = "ESC Debug Msg count",
    .data_type = EtherCAT::ObjectDictionary::ObjectDictionaryDataType::Unsigned8,
    .default_value = 16,
    .unit = Unit_None,
    .options_enum = nullptr,
    .min_value = 0,
    .max_value = 16,
    .modification_mode = ModificationMode::ReadOnly,
    .effective_time = EffectiveTime::Immediately,
    .comment = "Number of entries for ESC Debug Msg",
};

#define NEXCOBOT_DBG_MSG_REG(NUM, NAME) \
    constexpr ::EtherCAT::ObjectDictionary::ObjectDictionaryEntry DebugMsg_##NAME = { \
        .index = ESCDebugMsgIndex, \
        .subindex = (NUM), \
        .name = #NAME, \
        .data_type = EtherCAT::ObjectDictionary::ObjectDictionaryDataType::VisibleString, \
        .default_value = 0, \
        .unit = Unit_None, \
        .options_enum = nullptr, \
        .min_value = 0, \
        .max_value = 0xFF, \
        .modification_mode = ModificationMode::ReadOnly, \
        .effective_time = EffectiveTime::Immediately, \
        .comment = "ESC debug message " #NAME " (STRING 512)", \
    }

NEXCOBOT_DBG_MSG_REG(1,  Msg01);
NEXCOBOT_DBG_MSG_REG(2,  Msg02);
NEXCOBOT_DBG_MSG_REG(3,  Msg03);
NEXCOBOT_DBG_MSG_REG(4,  Msg04);
NEXCOBOT_DBG_MSG_REG(5,  Msg05);
NEXCOBOT_DBG_MSG_REG(6,  Msg06);
NEXCOBOT_DBG_MSG_REG(7,  Msg07);
NEXCOBOT_DBG_MSG_REG(8,  Msg08);
NEXCOBOT_DBG_MSG_REG(9,  Msg09);
NEXCOBOT_DBG_MSG_REG(10, Msg10);
NEXCOBOT_DBG_MSG_REG(11, Err01);
NEXCOBOT_DBG_MSG_REG(12, Err02);
NEXCOBOT_DBG_MSG_REG(13, Err03);
NEXCOBOT_DBG_MSG_REG(14, Err04);
NEXCOBOT_DBG_MSG_REG(15, Err05);
NEXCOBOT_DBG_MSG_REG(16, Err06);

#undef NEXCOBOT_DBG_MSG_REG

// ---------------------------------------------------------------------------
// 0xF111: System Current State (MPU_B)
// ---------------------------------------------------------------------------

constexpr ::EtherCAT::ObjectDictionary::ObjectDictionaryEntry SystemCurrentStateMPUB = {
    .index = SystemCurrentStateMPUBIndex,
    .subindex = 0x00,
    .name = "System Current State (MPU_B)",
    .data_type = EtherCAT::ObjectDictionary::ObjectDictionaryDataType::Unsigned32,
    .default_value = 0,
    .unit = Unit_None,
    .options_enum = nullptr,
    .min_value = 0,
    .max_value = 0xFFFFFFFF,
    .modification_mode = ModificationMode::ReadOnly,
    .effective_time = EffectiveTime::Immediately,
    .comment = "System current state (MPU_B)",
};

// ---------------------------------------------------------------------------
// 0xF112: System Error Code (MPU_B)
// ---------------------------------------------------------------------------

constexpr ::EtherCAT::ObjectDictionary::ObjectDictionaryEntry SystemErrorCodeMPUB = {
    .index = SystemErrorCodeMPUBIndex,
    .subindex = 0x00,
    .name = "System Error Code (MPU_B)",
    .data_type = EtherCAT::ObjectDictionary::ObjectDictionaryDataType::Integer32,
    .default_value = 0,
    .unit = Unit_None,
    .options_enum = nullptr,
    .min_value = static_cast<int64_t>(-0x80000000LL),
    .max_value = 0x7FFFFFFF,
    .modification_mode = ModificationMode::ReadOnly,
    .effective_time = EffectiveTime::Immediately,
    .comment = "System error code (MPU_B)",
};

inline const RegisterList kRegisterList = {
    &UserControlCount,
    &ControlCommand,
    &CommandResponse,
    &SystemCurrentState,
    &SystemErrorCode,
    &SystemErrorMessage,
    &LastErrorCodeCount,
    &LastErrorCode_ErrorCode_A,
    &LastErrorCode_ErrorCode_B,
    &LastErrorCode_Reserve_1,
    &LastErrorCode_Reserve_2,
    &UserPasswordInputCount,
    &Password_1_Input,
    &Password_2_Input,
    &Password_3_Input,
    &UserPasswordOutputCount,
    &Password_1_Output,
    &Password_2_Output,
    &Password_3_Output,
    &ESCDebugMsgCount,
    &DebugMsg_Msg01,
    &DebugMsg_Msg02,
    &DebugMsg_Msg03,
    &DebugMsg_Msg04,
    &DebugMsg_Msg05,
    &DebugMsg_Msg06,
    &DebugMsg_Msg07,
    &DebugMsg_Msg08,
    &DebugMsg_Msg09,
    &DebugMsg_Msg10,
    &DebugMsg_Err01,
    &DebugMsg_Err02,
    &DebugMsg_Err03,
    &DebugMsg_Err04,
    &DebugMsg_Err05,
    &DebugMsg_Err06,
    &SystemCurrentStateMPUB,
    &SystemErrorCodeMPUB,
};

} // namespace UserSystem
} // namespace NexcobotESC211
} // namespace Registers
} // namespace Drives
} // namespace EtherCAT

#pragma once

#include <cstdint>
#include <type_traits>
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
static constexpr uint16_t FSoEConnectionStateIndex      = 0xF105;
static constexpr uint16_t FSoEConnectionErrorCodeIndex  = 0xF106;
static constexpr uint16_t ESCDebugMsgIndex              = 0xF110;
static constexpr uint16_t SystemCurrentStateMPUBIndex   = 0xF111;
static constexpr uint16_t SystemErrorCodeMPUBIndex      = 0xF112;

// ---------------------------------------------------------------------------
// Control command values for 0xF100:01 (Control Command)
// ---------------------------------------------------------------------------

enum class ControlCommandCode : uint32_t {
    ResetResponseToStandby      = 0,    // Reset command response to STANDBY
    LoadFNIFromFlash            = 1,    // Load FNI from Flash to system (0xF201)
    LoadRSPAndSDDFromFlash      = 2,    // Load RSP and SDD from Flash to system
    UpdateFNIFromObject         = 3,    // Update FNI from object (0xF200) to system (0xF201)
    DownloadRSPAndSDDByObject   = 4,    // Download RSP and SDD by object (0xF210, 0xF220) to system
    SaveFNIToFlash              = 5,    // Save FNI to Flash
    SaveRSPAndSDDToFlash        = 6,    // Save RSP and SDD to Flash
    ClearApplicationErrorState  = 7,    // Clear application error state (APPERR -> READY)
    ClearLastErrorCode          = 8,    // Clear last error code object (0x104)
    StopSafetyAppAndFSoE        = 9,    // Stop safety application and FSoE connection
    StartFSoEAndSafetyApp       = 10,   // Start FSoE communication and safety application
    StartFSoEConnectionOnly   = 11,   // Start FSoE connection only (for testing)
    FlashMemoryReset            = 9001, // Flash memory reset
};

// ---------------------------------------------------------------------------
// Command response values for 0xF100:02 (Command Response)
// ---------------------------------------------------------------------------

enum class CommandResponseCode : uint32_t {
    Standby  = 0, // Response reset / standby
    Ongoing  = 1, // Command execution in progress
    Done     = 2, // Command completed successfully
    Failed   = 3, // Command failed
};

// ---------------------------------------------------------------------------
// 0xF100: User Control
// ---------------------------------------------------------------------------

constexpr ::EtherCAT::ObjectDictionary::ObjectDictionaryEntry UserControlCount = {
    .index = UserControlIndex,
    .subindex = 0x00,
    .name = "User Control count",
    .data_type = EtherCAT::ObjectDictionary::ObjectDictionaryDataType::Unsigned8,
    .default_value = 3,
    .unit = Unit_None,
    .options_enum = nullptr,
    .min_value = 0,
    .max_value = 3,
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
    .options_enum = std::type_identity<ControlCommandCode>{},
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
    .options_enum = std::type_identity<CommandResponseCode>{},
    .min_value = 0,
    .max_value = 0xFFFFFFFF,
    .modification_mode = ModificationMode::ReadOnly,
    .effective_time = EffectiveTime::Immediately,
    .comment = "User command response",
};

constexpr ::EtherCAT::ObjectDictionary::ObjectDictionaryEntry ResponseErrorCode = {
    .index = UserControlIndex,
    .subindex = 0x03,
    .name = "Response Error Code",
    .data_type = EtherCAT::ObjectDictionary::ObjectDictionaryDataType::Unsigned32,
    .default_value = 0,
    .unit = Unit_None,
    .options_enum = nullptr,
    .min_value = 0,
    .max_value = 0xFFFFFFFF,
    .modification_mode = ModificationMode::ReadOnly,
    .effective_time = EffectiveTime::Immediately,
    .comment = "Response error code",
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
NEXCOBOT_ERRCODE_REG(3, ErrorCode_A_Core1);
NEXCOBOT_ERRCODE_REG(4, ErrorCode_B_Core1);

#undef NEXCOBOT_ERRCODE_REG

// ---------------------------------------------------------------------------
// 0xF105: FSoE Connection State (record, sub 1..32, UDINT each)
// ---------------------------------------------------------------------------

constexpr ::EtherCAT::ObjectDictionary::ObjectDictionaryEntry FSoEConnectionStateCount = {
    .index = FSoEConnectionStateIndex,
    .subindex = 0x00,
    .name = "FSoE Connection State count",
    .data_type = EtherCAT::ObjectDictionary::ObjectDictionaryDataType::Unsigned8,
    .default_value = 32,
    .unit = Unit_None,
    .options_enum = nullptr,
    .min_value = 0,
    .max_value = 32,
    .modification_mode = ModificationMode::ReadOnly,
    .effective_time = EffectiveTime::Immediately,
    .comment = "Number of entries for FSoE Connection State",
};

// ---------------------------------------------------------------------------
// 0xF106: FSoE Connection Error Code (record, sub 1..32, UDINT each)
// ---------------------------------------------------------------------------

constexpr ::EtherCAT::ObjectDictionary::ObjectDictionaryEntry FSoEConnectionErrorCodeCount = {
    .index = FSoEConnectionErrorCodeIndex,
    .subindex = 0x00,
    .name = "FSoE Connection Error Code count",
    .data_type = EtherCAT::ObjectDictionary::ObjectDictionaryDataType::Unsigned8,
    .default_value = 32,
    .unit = Unit_None,
    .options_enum = nullptr,
    .min_value = 0,
    .max_value = 32,
    .modification_mode = ModificationMode::ReadOnly,
    .effective_time = EffectiveTime::Immediately,
    .comment = "Number of entries for FSoE Connection Error Code",
};

// Per-connection state/error entries are plain UDINT values.  ESI v0.9
// defines subindices 1..32 ("Connection 01".."Connection 32") on both
// 0xF105 (state) and 0xF106 (error code); NEXCOBOT_CONN_PAIR stamps the
// matching entry in each object for one subindex.
#define NEXCOBOT_CONN_REG(NAME, IDX, NUM) \
    constexpr ::EtherCAT::ObjectDictionary::ObjectDictionaryEntry NAME##_##NUM = { \
        .index = (IDX), \
        .subindex = (NUM), \
        .name = "Connection " #NUM, \
        .data_type = EtherCAT::ObjectDictionary::ObjectDictionaryDataType::Unsigned32, \
        .default_value = 0, \
        .unit = Unit_None, \
        .options_enum = nullptr, \
        .min_value = 0, \
        .max_value = 0xFFFFFFFF, \
        .modification_mode = ModificationMode::ReadOnly, \
        .effective_time = EffectiveTime::Immediately, \
        .comment = "FSoE connection " #NUM " state/error code", \
    }

#define NEXCOBOT_CONN_PAIR(NUM) \
    NEXCOBOT_CONN_REG(FSoEConnectionState,     FSoEConnectionStateIndex,     NUM); \
    NEXCOBOT_CONN_REG(FSoEConnectionErrorCode, FSoEConnectionErrorCodeIndex, NUM)

NEXCOBOT_CONN_PAIR(1);  NEXCOBOT_CONN_PAIR(2);  NEXCOBOT_CONN_PAIR(3);
NEXCOBOT_CONN_PAIR(4);  NEXCOBOT_CONN_PAIR(5);  NEXCOBOT_CONN_PAIR(6);
NEXCOBOT_CONN_PAIR(7);  NEXCOBOT_CONN_PAIR(8);  NEXCOBOT_CONN_PAIR(9);
NEXCOBOT_CONN_PAIR(10); NEXCOBOT_CONN_PAIR(11); NEXCOBOT_CONN_PAIR(12);
NEXCOBOT_CONN_PAIR(13); NEXCOBOT_CONN_PAIR(14); NEXCOBOT_CONN_PAIR(15);
NEXCOBOT_CONN_PAIR(16); NEXCOBOT_CONN_PAIR(17); NEXCOBOT_CONN_PAIR(18);
NEXCOBOT_CONN_PAIR(19); NEXCOBOT_CONN_PAIR(20); NEXCOBOT_CONN_PAIR(21);
NEXCOBOT_CONN_PAIR(22); NEXCOBOT_CONN_PAIR(23); NEXCOBOT_CONN_PAIR(24);
NEXCOBOT_CONN_PAIR(25); NEXCOBOT_CONN_PAIR(26); NEXCOBOT_CONN_PAIR(27);
NEXCOBOT_CONN_PAIR(28); NEXCOBOT_CONN_PAIR(29); NEXCOBOT_CONN_PAIR(30);
NEXCOBOT_CONN_PAIR(31); NEXCOBOT_CONN_PAIR(32);

#undef NEXCOBOT_CONN_PAIR
#undef NEXCOBOT_CONN_REG

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
    &ResponseErrorCode,
    &SystemCurrentState,
    &SystemErrorCode,
    &SystemErrorMessage,
    &LastErrorCodeCount,
    &LastErrorCode_ErrorCode_A,
    &LastErrorCode_ErrorCode_B,
    &LastErrorCode_ErrorCode_A_Core1,
    &LastErrorCode_ErrorCode_B_Core1,
    &FSoEConnectionStateCount,
    &FSoEConnectionErrorCodeCount,
#define NEXCOBOT_CONN_LIST(NUM) \
    &FSoEConnectionState_##NUM, &FSoEConnectionErrorCode_##NUM,
    NEXCOBOT_CONN_LIST(1)  NEXCOBOT_CONN_LIST(2)  NEXCOBOT_CONN_LIST(3)
    NEXCOBOT_CONN_LIST(4)  NEXCOBOT_CONN_LIST(5)  NEXCOBOT_CONN_LIST(6)
    NEXCOBOT_CONN_LIST(7)  NEXCOBOT_CONN_LIST(8)  NEXCOBOT_CONN_LIST(9)
    NEXCOBOT_CONN_LIST(10) NEXCOBOT_CONN_LIST(11) NEXCOBOT_CONN_LIST(12)
    NEXCOBOT_CONN_LIST(13) NEXCOBOT_CONN_LIST(14) NEXCOBOT_CONN_LIST(15)
    NEXCOBOT_CONN_LIST(16) NEXCOBOT_CONN_LIST(17) NEXCOBOT_CONN_LIST(18)
    NEXCOBOT_CONN_LIST(19) NEXCOBOT_CONN_LIST(20) NEXCOBOT_CONN_LIST(21)
    NEXCOBOT_CONN_LIST(22) NEXCOBOT_CONN_LIST(23) NEXCOBOT_CONN_LIST(24)
    NEXCOBOT_CONN_LIST(25) NEXCOBOT_CONN_LIST(26) NEXCOBOT_CONN_LIST(27)
    NEXCOBOT_CONN_LIST(28) NEXCOBOT_CONN_LIST(29) NEXCOBOT_CONN_LIST(30)
    NEXCOBOT_CONN_LIST(31) NEXCOBOT_CONN_LIST(32)
#undef NEXCOBOT_CONN_LIST
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

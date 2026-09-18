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
static constexpr uint16_t LogMsgIndex                   = 0xF120;
static constexpr uint16_t Core1AppCycTimeIndex          = 0xF121;
static constexpr uint16_t MasterCtrlInputCommandIndex   = 0xF400;
static constexpr uint16_t UserAppCommandIndex           = 0xF401;
static constexpr uint16_t PasswordInputIndex            = 0xF600;
static constexpr uint16_t CTProjectNameIndex            = 0xF601;
static constexpr uint16_t CTVersionIndex                = 0xF602;
static constexpr uint16_t UserSettingIndex              = 0xF605;
static constexpr uint16_t AdminModeIndex                = 0xF610;
static constexpr uint16_t EthernetMACIndex              = 0xF700;
static constexpr uint16_t EthernetIPIndex               = 0xF701;
static constexpr uint16_t EthernetMaskIndex             = 0xF702;
static constexpr uint16_t EthernetGatewayIndex          = 0xF703;

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
        .comment = "ESC debug message " #NAME " (STRING 64)", \
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

// ---------------------------------------------------------------------------
// 0xF120: Log Msg (record, sub 1..128, STRING(128) each)
// ---------------------------------------------------------------------------

constexpr ::EtherCAT::ObjectDictionary::ObjectDictionaryEntry LogMsgCount = {
    .index = LogMsgIndex,
    .subindex = 0x00,
    .name = "Log Msg count",
    .data_type = EtherCAT::ObjectDictionary::ObjectDictionaryDataType::Unsigned8,
    .default_value = 128,
    .unit = Unit_None,
    .options_enum = nullptr,
    .min_value = 0,
    .max_value = 128,
    .modification_mode = ModificationMode::ReadOnly,
    .effective_time = EffectiveTime::Immediately,
    .comment = "Number of entries for Log Msg",
};

#define NEXCOBOT_LOG_MSG_REG(NUM) \
    constexpr ::EtherCAT::ObjectDictionary::ObjectDictionaryEntry LogMsg_##NUM = { \
        .index = LogMsgIndex, \
        .subindex = (NUM), \
        .name = "Log Msg " #NUM, \
        .data_type = EtherCAT::ObjectDictionary::ObjectDictionaryDataType::VisibleString, \
        .default_value = 0, \
        .unit = Unit_None, \
        .options_enum = nullptr, \
        .min_value = 0, \
        .max_value = 0xFF, \
        .modification_mode = ModificationMode::ReadOnly, \
        .effective_time = EffectiveTime::Immediately, \
        .comment = "Log message " #NUM " (STRING 128)", \
    }

NEXCOBOT_LOG_MSG_REG(1);   NEXCOBOT_LOG_MSG_REG(2);   NEXCOBOT_LOG_MSG_REG(3);
NEXCOBOT_LOG_MSG_REG(4);   NEXCOBOT_LOG_MSG_REG(5);   NEXCOBOT_LOG_MSG_REG(6);
NEXCOBOT_LOG_MSG_REG(7);   NEXCOBOT_LOG_MSG_REG(8);   NEXCOBOT_LOG_MSG_REG(9);
NEXCOBOT_LOG_MSG_REG(10);  NEXCOBOT_LOG_MSG_REG(11);  NEXCOBOT_LOG_MSG_REG(12);
NEXCOBOT_LOG_MSG_REG(13);  NEXCOBOT_LOG_MSG_REG(14);  NEXCOBOT_LOG_MSG_REG(15);
NEXCOBOT_LOG_MSG_REG(16);  NEXCOBOT_LOG_MSG_REG(17);  NEXCOBOT_LOG_MSG_REG(18);
NEXCOBOT_LOG_MSG_REG(19);  NEXCOBOT_LOG_MSG_REG(20);  NEXCOBOT_LOG_MSG_REG(21);
NEXCOBOT_LOG_MSG_REG(22);  NEXCOBOT_LOG_MSG_REG(23);  NEXCOBOT_LOG_MSG_REG(24);
NEXCOBOT_LOG_MSG_REG(25);  NEXCOBOT_LOG_MSG_REG(26);  NEXCOBOT_LOG_MSG_REG(27);
NEXCOBOT_LOG_MSG_REG(28);  NEXCOBOT_LOG_MSG_REG(29);  NEXCOBOT_LOG_MSG_REG(30);
NEXCOBOT_LOG_MSG_REG(31);  NEXCOBOT_LOG_MSG_REG(32);  NEXCOBOT_LOG_MSG_REG(33);
NEXCOBOT_LOG_MSG_REG(34);  NEXCOBOT_LOG_MSG_REG(35);  NEXCOBOT_LOG_MSG_REG(36);
NEXCOBOT_LOG_MSG_REG(37);  NEXCOBOT_LOG_MSG_REG(38);  NEXCOBOT_LOG_MSG_REG(39);
NEXCOBOT_LOG_MSG_REG(40);  NEXCOBOT_LOG_MSG_REG(41);  NEXCOBOT_LOG_MSG_REG(42);
NEXCOBOT_LOG_MSG_REG(43);  NEXCOBOT_LOG_MSG_REG(44);  NEXCOBOT_LOG_MSG_REG(45);
NEXCOBOT_LOG_MSG_REG(46);  NEXCOBOT_LOG_MSG_REG(47);  NEXCOBOT_LOG_MSG_REG(48);
NEXCOBOT_LOG_MSG_REG(49);  NEXCOBOT_LOG_MSG_REG(50);  NEXCOBOT_LOG_MSG_REG(51);
NEXCOBOT_LOG_MSG_REG(52);  NEXCOBOT_LOG_MSG_REG(53);  NEXCOBOT_LOG_MSG_REG(54);
NEXCOBOT_LOG_MSG_REG(55);  NEXCOBOT_LOG_MSG_REG(56);  NEXCOBOT_LOG_MSG_REG(57);
NEXCOBOT_LOG_MSG_REG(58);  NEXCOBOT_LOG_MSG_REG(59);  NEXCOBOT_LOG_MSG_REG(60);
NEXCOBOT_LOG_MSG_REG(61);  NEXCOBOT_LOG_MSG_REG(62);  NEXCOBOT_LOG_MSG_REG(63);
NEXCOBOT_LOG_MSG_REG(64);  NEXCOBOT_LOG_MSG_REG(65);  NEXCOBOT_LOG_MSG_REG(66);
NEXCOBOT_LOG_MSG_REG(67);  NEXCOBOT_LOG_MSG_REG(68);  NEXCOBOT_LOG_MSG_REG(69);
NEXCOBOT_LOG_MSG_REG(70);  NEXCOBOT_LOG_MSG_REG(71);  NEXCOBOT_LOG_MSG_REG(72);
NEXCOBOT_LOG_MSG_REG(73);  NEXCOBOT_LOG_MSG_REG(74);  NEXCOBOT_LOG_MSG_REG(75);
NEXCOBOT_LOG_MSG_REG(76);  NEXCOBOT_LOG_MSG_REG(77);  NEXCOBOT_LOG_MSG_REG(78);
NEXCOBOT_LOG_MSG_REG(79);  NEXCOBOT_LOG_MSG_REG(80);  NEXCOBOT_LOG_MSG_REG(81);
NEXCOBOT_LOG_MSG_REG(82);  NEXCOBOT_LOG_MSG_REG(83);  NEXCOBOT_LOG_MSG_REG(84);
NEXCOBOT_LOG_MSG_REG(85);  NEXCOBOT_LOG_MSG_REG(86);  NEXCOBOT_LOG_MSG_REG(87);
NEXCOBOT_LOG_MSG_REG(88);  NEXCOBOT_LOG_MSG_REG(89);  NEXCOBOT_LOG_MSG_REG(90);
NEXCOBOT_LOG_MSG_REG(91);  NEXCOBOT_LOG_MSG_REG(92);  NEXCOBOT_LOG_MSG_REG(93);
NEXCOBOT_LOG_MSG_REG(94);  NEXCOBOT_LOG_MSG_REG(95);  NEXCOBOT_LOG_MSG_REG(96);
NEXCOBOT_LOG_MSG_REG(97);  NEXCOBOT_LOG_MSG_REG(98);  NEXCOBOT_LOG_MSG_REG(99);
NEXCOBOT_LOG_MSG_REG(100); NEXCOBOT_LOG_MSG_REG(101); NEXCOBOT_LOG_MSG_REG(102);
NEXCOBOT_LOG_MSG_REG(103); NEXCOBOT_LOG_MSG_REG(104); NEXCOBOT_LOG_MSG_REG(105);
NEXCOBOT_LOG_MSG_REG(106); NEXCOBOT_LOG_MSG_REG(107); NEXCOBOT_LOG_MSG_REG(108);
NEXCOBOT_LOG_MSG_REG(109); NEXCOBOT_LOG_MSG_REG(110); NEXCOBOT_LOG_MSG_REG(111);
NEXCOBOT_LOG_MSG_REG(112); NEXCOBOT_LOG_MSG_REG(113); NEXCOBOT_LOG_MSG_REG(114);
NEXCOBOT_LOG_MSG_REG(115); NEXCOBOT_LOG_MSG_REG(116); NEXCOBOT_LOG_MSG_REG(117);
NEXCOBOT_LOG_MSG_REG(118); NEXCOBOT_LOG_MSG_REG(119); NEXCOBOT_LOG_MSG_REG(120);
NEXCOBOT_LOG_MSG_REG(121); NEXCOBOT_LOG_MSG_REG(122); NEXCOBOT_LOG_MSG_REG(123);
NEXCOBOT_LOG_MSG_REG(124); NEXCOBOT_LOG_MSG_REG(125); NEXCOBOT_LOG_MSG_REG(126);
NEXCOBOT_LOG_MSG_REG(127); NEXCOBOT_LOG_MSG_REG(128);

#undef NEXCOBOT_LOG_MSG_REG

// ---------------------------------------------------------------------------
// 0xF121: Core1 App Cyc Time (us) (record, sub 1..12, AppN Cur/Max UDINT)
// ---------------------------------------------------------------------------

constexpr ::EtherCAT::ObjectDictionary::ObjectDictionaryEntry Core1AppCycTimeCount = {
    .index = Core1AppCycTimeIndex,
    .subindex = 0x00,
    .name = "Core1 App Cyc Time count",
    .data_type = EtherCAT::ObjectDictionary::ObjectDictionaryDataType::Unsigned8,
    .default_value = 12,
    .unit = Unit_None,
    .options_enum = nullptr,
    .min_value = 0,
    .max_value = 12,
    .modification_mode = ModificationMode::ReadOnly,
    .effective_time = EffectiveTime::Immediately,
    .comment = "Number of entries for Core1 App Cyc Time (us)",
};

#define NEXCOBOT_CYC_REG(NUM, NAME) \
    constexpr ::EtherCAT::ObjectDictionary::ObjectDictionaryEntry Core1AppCycTime_##NAME = { \
        .index = Core1AppCycTimeIndex, \
        .subindex = (NUM), \
        .name = "Core1 App Cyc Time " #NAME, \
        .data_type = EtherCAT::ObjectDictionary::ObjectDictionaryDataType::Unsigned32, \
        .default_value = 0, \
        .unit = Unit_None, \
        .options_enum = nullptr, \
        .min_value = 0, \
        .max_value = 0xFFFFFFFF, \
        .modification_mode = ModificationMode::ReadOnly, \
        .effective_time = EffectiveTime::Immediately, \
        .comment = "Core1 application cycle time " #NAME " (us)", \
    }

NEXCOBOT_CYC_REG(1,  App1Cur); NEXCOBOT_CYC_REG(2,  App1Max);
NEXCOBOT_CYC_REG(3,  App2Cur); NEXCOBOT_CYC_REG(4,  App2Max);
NEXCOBOT_CYC_REG(5,  App3Cur); NEXCOBOT_CYC_REG(6,  App3Max);
NEXCOBOT_CYC_REG(7,  App4Cur); NEXCOBOT_CYC_REG(8,  App4Max);
NEXCOBOT_CYC_REG(9,  App5Cur); NEXCOBOT_CYC_REG(10, App5Max);
NEXCOBOT_CYC_REG(11, App6Cur); NEXCOBOT_CYC_REG(12, App6Max);

#undef NEXCOBOT_CYC_REG

// ---------------------------------------------------------------------------
// 0xF400: Master Controller Input Command (record, sub1 Operation Mode Control)
// ---------------------------------------------------------------------------

constexpr ::EtherCAT::ObjectDictionary::ObjectDictionaryEntry MasterCtrlInputCommandCount = {
    .index = MasterCtrlInputCommandIndex,
    .subindex = 0x00,
    .name = "Master Controller Input Command count",
    .data_type = EtherCAT::ObjectDictionary::ObjectDictionaryDataType::Unsigned8,
    .default_value = 1,
    .unit = Unit_None,
    .options_enum = nullptr,
    .min_value = 0,
    .max_value = 1,
    .modification_mode = ModificationMode::ReadOnly,
    .effective_time = EffectiveTime::Immediately,
    .comment = "Number of entries for Master Controller Input Command",
};

constexpr ::EtherCAT::ObjectDictionary::ObjectDictionaryEntry OperationModeControl = {
    .index = MasterCtrlInputCommandIndex,
    .subindex = 0x01,
    .name = "Operation Mode Control",
    .data_type = EtherCAT::ObjectDictionary::ObjectDictionaryDataType::Unsigned8,
    .default_value = 0,
    .unit = Unit_None,
    .options_enum = nullptr,
    .min_value = 0,
    .max_value = 0xFF,
    .modification_mode = ModificationMode::DuringOperation,
    .effective_time = EffectiveTime::Immediately,
    .comment = "Operation mode control command",
};

// ---------------------------------------------------------------------------
// 0xF401: User Application Command (record, sub 1..3 USINT)
// ---------------------------------------------------------------------------

constexpr ::EtherCAT::ObjectDictionary::ObjectDictionaryEntry UserAppCommandCount = {
    .index = UserAppCommandIndex,
    .subindex = 0x00,
    .name = "User Application Command count",
    .data_type = EtherCAT::ObjectDictionary::ObjectDictionaryDataType::Unsigned8,
    .default_value = 3,
    .unit = Unit_None,
    .options_enum = nullptr,
    .min_value = 0,
    .max_value = 3,
    .modification_mode = ModificationMode::ReadOnly,
    .effective_time = EffectiveTime::Immediately,
    .comment = "Number of entries for User Application Command (Reserve)",
};

#define NEXCOBOT_USERCMD_REG(NUM) \
    constexpr ::EtherCAT::ObjectDictionary::ObjectDictionaryEntry UserCommand_##NUM = { \
        .index = UserAppCommandIndex, \
        .subindex = (NUM), \
        .name = "User Command " #NUM, \
        .data_type = EtherCAT::ObjectDictionary::ObjectDictionaryDataType::Unsigned8, \
        .default_value = 0, \
        .unit = Unit_None, \
        .options_enum = nullptr, \
        .min_value = 0, \
        .max_value = 0xFF, \
        .modification_mode = ModificationMode::DuringOperation, \
        .effective_time = EffectiveTime::Immediately, \
        .comment = "User application command " #NUM, \
    }

NEXCOBOT_USERCMD_REG(1);
NEXCOBOT_USERCMD_REG(2);
NEXCOBOT_USERCMD_REG(3);

#undef NEXCOBOT_USERCMD_REG

// ---------------------------------------------------------------------------
// 0xF600-0xF602: Password / CT project info (STRING(64) each)
// ---------------------------------------------------------------------------

#define NEXCOBOT_STR64_REG(NAME, IDX, DESC) \
    constexpr ::EtherCAT::ObjectDictionary::ObjectDictionaryEntry NAME = { \
        .index = (IDX), .subindex = 0x00, .name = (DESC), \
        .data_type = EtherCAT::ObjectDictionary::ObjectDictionaryDataType::VisibleString, \
        .default_value = 0, .unit = Unit_None, .options_enum = nullptr, \
        .min_value = 0, .max_value = 0xFF, \
        .modification_mode = ModificationMode::ReadOnly, \
        .effective_time = EffectiveTime::Immediately, \
        .comment = DESC " (STRING(64))", \
    }

NEXCOBOT_STR64_REG(PasswordInput,  PasswordInputIndex,  "Password Input");
NEXCOBOT_STR64_REG(CTProjectName,  CTProjectNameIndex,  "CT Project Name");
NEXCOBOT_STR64_REG(CTVersion,      CTVersionIndex,      "CT Version");

#undef NEXCOBOT_STR64_REG

// ---------------------------------------------------------------------------
// 0xF605: User Setting (record, sub 1..3 UINT)
// ---------------------------------------------------------------------------

constexpr ::EtherCAT::ObjectDictionary::ObjectDictionaryEntry UserSettingCount = {
    .index = UserSettingIndex,
    .subindex = 0x00,
    .name = "User Setting count",
    .data_type = EtherCAT::ObjectDictionary::ObjectDictionaryDataType::Unsigned8,
    .default_value = 3,
    .unit = Unit_None,
    .options_enum = nullptr,
    .min_value = 0,
    .max_value = 3,
    .modification_mode = ModificationMode::ReadOnly,
    .effective_time = EffectiveTime::Immediately,
    .comment = "Number of entries for User Setting",
};

#define NEXCOBOT_USET_REG(SUB, NAME, DESC) \
    constexpr ::EtherCAT::ObjectDictionary::ObjectDictionaryEntry NAME = { \
        .index = UserSettingIndex, \
        .subindex = (SUB), \
        .name = (DESC), \
        .data_type = EtherCAT::ObjectDictionary::ObjectDictionaryDataType::Unsigned16, \
        .default_value = 0, \
        .unit = Unit_None, \
        .options_enum = nullptr, \
        .min_value = 0, \
        .max_value = 0xFFFF, \
        .modification_mode = ModificationMode::ReadOnly, \
        .effective_time = EffectiveTime::Immediately, \
        .comment = (DESC), \
    }

NEXCOBOT_USET_REG(1, SRAMSParaNum,  "SRAM SPara Num");
NEXCOBOT_USET_REG(2, SDRAMSParaNum, "SDRAM SPara Num");
NEXCOBOT_USET_REG(3, RxPDOSize,     "RxPDO Size");

#undef NEXCOBOT_USET_REG

// ---------------------------------------------------------------------------
// 0xF610: Admin Mode
// ---------------------------------------------------------------------------

constexpr ::EtherCAT::ObjectDictionary::ObjectDictionaryEntry AdminMode = {
    .index = AdminModeIndex,
    .subindex = 0x00,
    .name = "Admin Mode",
    .data_type = EtherCAT::ObjectDictionary::ObjectDictionaryDataType::Unsigned8,
    .default_value = 0,
    .unit = Unit_None,
    .options_enum = nullptr,
    .min_value = 0,
    .max_value = 0xFF,
    .modification_mode = ModificationMode::ReadOnly,
    .effective_time = EffectiveTime::Immediately,
    .comment = "Admin mode flag",
};

// ---------------------------------------------------------------------------
// 0xF700-0xF703: Ethernet settings (byte arrays)
// ---------------------------------------------------------------------------

#define NEXCOBOT_ETH_REG(NAME, IDX, DESC, LEN) \
    constexpr ::EtherCAT::ObjectDictionary::ObjectDictionaryEntry NAME = { \
        .index = (IDX), .subindex = 0x00, .name = (DESC), \
        .data_type = EtherCAT::ObjectDictionary::ObjectDictionaryDataType::OctetString, \
        .default_value = 0, .unit = Unit_None, .options_enum = nullptr, \
        .min_value = 0, .max_value = 0xFF, \
        .modification_mode = ModificationMode::DuringOperation, \
        .effective_time = EffectiveTime::Immediately, \
        .comment = DESC " (ARRAY[0.." LEN "] OF BYTE)", \
    }

NEXCOBOT_ETH_REG(EthernetMAC,     EthernetMACIndex,     "Ethernet MAC",     "5");
NEXCOBOT_ETH_REG(EthernetIP,      EthernetIPIndex,      "Ethernet IP",      "3");
NEXCOBOT_ETH_REG(EthernetMask,    EthernetMaskIndex,    "Ethernet Mask",    "3");
NEXCOBOT_ETH_REG(EthernetGateway, EthernetGatewayIndex, "Ethernet Gateway", "3");

#undef NEXCOBOT_ETH_REG

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
    &LogMsgCount,
#define NEXCOBOT_LOG_LIST(NUM) &LogMsg_##NUM,
    NEXCOBOT_LOG_LIST(1)   NEXCOBOT_LOG_LIST(2)   NEXCOBOT_LOG_LIST(3)
    NEXCOBOT_LOG_LIST(4)   NEXCOBOT_LOG_LIST(5)   NEXCOBOT_LOG_LIST(6)
    NEXCOBOT_LOG_LIST(7)   NEXCOBOT_LOG_LIST(8)   NEXCOBOT_LOG_LIST(9)
    NEXCOBOT_LOG_LIST(10)  NEXCOBOT_LOG_LIST(11)  NEXCOBOT_LOG_LIST(12)
    NEXCOBOT_LOG_LIST(13)  NEXCOBOT_LOG_LIST(14)  NEXCOBOT_LOG_LIST(15)
    NEXCOBOT_LOG_LIST(16)  NEXCOBOT_LOG_LIST(17)  NEXCOBOT_LOG_LIST(18)
    NEXCOBOT_LOG_LIST(19)  NEXCOBOT_LOG_LIST(20)  NEXCOBOT_LOG_LIST(21)
    NEXCOBOT_LOG_LIST(22)  NEXCOBOT_LOG_LIST(23)  NEXCOBOT_LOG_LIST(24)
    NEXCOBOT_LOG_LIST(25)  NEXCOBOT_LOG_LIST(26)  NEXCOBOT_LOG_LIST(27)
    NEXCOBOT_LOG_LIST(28)  NEXCOBOT_LOG_LIST(29)  NEXCOBOT_LOG_LIST(30)
    NEXCOBOT_LOG_LIST(31)  NEXCOBOT_LOG_LIST(32)  NEXCOBOT_LOG_LIST(33)
    NEXCOBOT_LOG_LIST(34)  NEXCOBOT_LOG_LIST(35)  NEXCOBOT_LOG_LIST(36)
    NEXCOBOT_LOG_LIST(37)  NEXCOBOT_LOG_LIST(38)  NEXCOBOT_LOG_LIST(39)
    NEXCOBOT_LOG_LIST(40)  NEXCOBOT_LOG_LIST(41)  NEXCOBOT_LOG_LIST(42)
    NEXCOBOT_LOG_LIST(43)  NEXCOBOT_LOG_LIST(44)  NEXCOBOT_LOG_LIST(45)
    NEXCOBOT_LOG_LIST(46)  NEXCOBOT_LOG_LIST(47)  NEXCOBOT_LOG_LIST(48)
    NEXCOBOT_LOG_LIST(49)  NEXCOBOT_LOG_LIST(50)  NEXCOBOT_LOG_LIST(51)
    NEXCOBOT_LOG_LIST(52)  NEXCOBOT_LOG_LIST(53)  NEXCOBOT_LOG_LIST(54)
    NEXCOBOT_LOG_LIST(55)  NEXCOBOT_LOG_LIST(56)  NEXCOBOT_LOG_LIST(57)
    NEXCOBOT_LOG_LIST(58)  NEXCOBOT_LOG_LIST(59)  NEXCOBOT_LOG_LIST(60)
    NEXCOBOT_LOG_LIST(61)  NEXCOBOT_LOG_LIST(62)  NEXCOBOT_LOG_LIST(63)
    NEXCOBOT_LOG_LIST(64)  NEXCOBOT_LOG_LIST(65)  NEXCOBOT_LOG_LIST(66)
    NEXCOBOT_LOG_LIST(67)  NEXCOBOT_LOG_LIST(68)  NEXCOBOT_LOG_LIST(69)
    NEXCOBOT_LOG_LIST(70)  NEXCOBOT_LOG_LIST(71)  NEXCOBOT_LOG_LIST(72)
    NEXCOBOT_LOG_LIST(73)  NEXCOBOT_LOG_LIST(74)  NEXCOBOT_LOG_LIST(75)
    NEXCOBOT_LOG_LIST(76)  NEXCOBOT_LOG_LIST(77)  NEXCOBOT_LOG_LIST(78)
    NEXCOBOT_LOG_LIST(79)  NEXCOBOT_LOG_LIST(80)  NEXCOBOT_LOG_LIST(81)
    NEXCOBOT_LOG_LIST(82)  NEXCOBOT_LOG_LIST(83)  NEXCOBOT_LOG_LIST(84)
    NEXCOBOT_LOG_LIST(85)  NEXCOBOT_LOG_LIST(86)  NEXCOBOT_LOG_LIST(87)
    NEXCOBOT_LOG_LIST(88)  NEXCOBOT_LOG_LIST(89)  NEXCOBOT_LOG_LIST(90)
    NEXCOBOT_LOG_LIST(91)  NEXCOBOT_LOG_LIST(92)  NEXCOBOT_LOG_LIST(93)
    NEXCOBOT_LOG_LIST(94)  NEXCOBOT_LOG_LIST(95)  NEXCOBOT_LOG_LIST(96)
    NEXCOBOT_LOG_LIST(97)  NEXCOBOT_LOG_LIST(98)  NEXCOBOT_LOG_LIST(99)
    NEXCOBOT_LOG_LIST(100) NEXCOBOT_LOG_LIST(101) NEXCOBOT_LOG_LIST(102)
    NEXCOBOT_LOG_LIST(103) NEXCOBOT_LOG_LIST(104) NEXCOBOT_LOG_LIST(105)
    NEXCOBOT_LOG_LIST(106) NEXCOBOT_LOG_LIST(107) NEXCOBOT_LOG_LIST(108)
    NEXCOBOT_LOG_LIST(109) NEXCOBOT_LOG_LIST(110) NEXCOBOT_LOG_LIST(111)
    NEXCOBOT_LOG_LIST(112) NEXCOBOT_LOG_LIST(113) NEXCOBOT_LOG_LIST(114)
    NEXCOBOT_LOG_LIST(115) NEXCOBOT_LOG_LIST(116) NEXCOBOT_LOG_LIST(117)
    NEXCOBOT_LOG_LIST(118) NEXCOBOT_LOG_LIST(119) NEXCOBOT_LOG_LIST(120)
    NEXCOBOT_LOG_LIST(121) NEXCOBOT_LOG_LIST(122) NEXCOBOT_LOG_LIST(123)
    NEXCOBOT_LOG_LIST(124) NEXCOBOT_LOG_LIST(125) NEXCOBOT_LOG_LIST(126)
    NEXCOBOT_LOG_LIST(127) NEXCOBOT_LOG_LIST(128)
#undef NEXCOBOT_LOG_LIST
    &Core1AppCycTimeCount,
    &Core1AppCycTime_App1Cur, &Core1AppCycTime_App1Max,
    &Core1AppCycTime_App2Cur, &Core1AppCycTime_App2Max,
    &Core1AppCycTime_App3Cur, &Core1AppCycTime_App3Max,
    &Core1AppCycTime_App4Cur, &Core1AppCycTime_App4Max,
    &Core1AppCycTime_App5Cur, &Core1AppCycTime_App5Max,
    &Core1AppCycTime_App6Cur, &Core1AppCycTime_App6Max,
    &MasterCtrlInputCommandCount,
    &OperationModeControl,
    &UserAppCommandCount,
    &UserCommand_1, &UserCommand_2, &UserCommand_3,
    &PasswordInput, &CTProjectName, &CTVersion,
    &UserSettingCount,
    &SRAMSParaNum, &SDRAMSParaNum, &RxPDOSize,
    &AdminMode,
    &EthernetMAC, &EthernetIP, &EthernetMask, &EthernetGateway,
};

} // namespace UserSystem
} // namespace NexcobotESC211
} // namespace Registers
} // namespace Drives
} // namespace EtherCAT

#pragma once

#include <cstdint>
#include "tether/drives/NexcobotESC211/Registers/Common.hpp"

namespace EtherCAT {
namespace Drives {
namespace Registers {
namespace NexcobotESC211 {
namespace FSOERxChannels {

// ---------------------------------------------------------------------------
// 0x6100 / 0x6101: FSoE0 Frame / SafeData
// ---------------------------------------------------------------------------

static constexpr uint16_t FSOEFrameFSoE0Index  = 0x6100;
static constexpr uint16_t FSOESafeDataFSoE0Index = 0x6101;

constexpr ::EtherCAT::ObjectDictionary::ObjectDictionaryEntry FSOEFrameFSoE0Count = {
    .index = FSOEFrameFSoE0Index, .subindex = 0x00,
    .name = "FSOE Master Frame (FSoE0) count",
    .data_type = EtherCAT::ObjectDictionary::ObjectDictionaryDataType::Unsigned8,
    .default_value = 11, .unit = Unit_None, .options_enum = nullptr,
    .min_value = 0, .max_value = 11,
    .modification_mode = ModificationMode::ReadOnly,
    .effective_time = EffectiveTime::Immediately,
    .comment = "Number of entries for FSOE Master Frame FSoE0",
};

constexpr ::EtherCAT::ObjectDictionary::ObjectDictionaryEntry FSOE0_FSoECommand = {
    .index = FSOEFrameFSoE0Index, .subindex = 0x01, .name = "FSoE0 Command",
    .data_type = EtherCAT::ObjectDictionary::ObjectDictionaryDataType::Unsigned8,
    .default_value = 0, .unit = Unit_None, .options_enum = nullptr,
    .min_value = 0, .max_value = 0xFF,
    .modification_mode = ModificationMode::ReadOnly,
    .effective_time = EffectiveTime::Immediately, .comment = "FSoE0 command byte",
};

constexpr ::EtherCAT::ObjectDictionary::ObjectDictionaryEntry FSOE0_ConnectionID = {
    .index = FSOEFrameFSoE0Index, .subindex = 0x02, .name = "FSoE0 Connection ID",
    .data_type = EtherCAT::ObjectDictionary::ObjectDictionaryDataType::Unsigned16,
    .default_value = 0, .unit = Unit_None, .options_enum = nullptr,
    .min_value = 0, .max_value = 0xFFFF,
    .modification_mode = ModificationMode::ReadOnly,
    .effective_time = EffectiveTime::Immediately, .comment = "FSoE0 connection ID",
};

#define NEXCOBOT_FSOE0_CRC(N) \
    constexpr ::EtherCAT::ObjectDictionary::ObjectDictionaryEntry FSOE0_CRC_##N = { \
        .index = FSOEFrameFSoE0Index, .subindex = (N + 2), .name = "FSoE0 crc_" #N, \
        .data_type = EtherCAT::ObjectDictionary::ObjectDictionaryDataType::Unsigned8, \
        .default_value = 0, .unit = Unit_None, .options_enum = nullptr, \
        .min_value = 0, .max_value = 0xFF, \
        .modification_mode = ModificationMode::ReadOnly, \
        .effective_time = EffectiveTime::Immediately, \
        .comment = "FSoE0 CRC byte " #N, \
    }

NEXCOBOT_FSOE0_CRC(0); NEXCOBOT_FSOE0_CRC(1); NEXCOBOT_FSOE0_CRC(2); NEXCOBOT_FSOE0_CRC(3);
NEXCOBOT_FSOE0_CRC(4); NEXCOBOT_FSOE0_CRC(5); NEXCOBOT_FSOE0_CRC(6); NEXCOBOT_FSOE0_CRC(7);
#undef NEXCOBOT_FSOE0_CRC

constexpr ::EtherCAT::ObjectDictionary::ObjectDictionaryEntry FSOESafeDataFSoE0Count = {
    .index = FSOESafeDataFSoE0Index, .subindex = 0x00,
    .name = "FSOE Master SafeData (FSoE0) count",
    .data_type = EtherCAT::ObjectDictionary::ObjectDictionaryDataType::Unsigned8,
    .default_value = 8, .unit = Unit_None, .options_enum = nullptr,
    .min_value = 0, .max_value = 8,
    .modification_mode = ModificationMode::ReadOnly,
    .effective_time = EffectiveTime::Immediately,
    .comment = "Number of entries for FSOE Master SafeData FSoE0",
};

#define NEXCOBOT_FSOE0_DATA(N) \
    constexpr ::EtherCAT::ObjectDictionary::ObjectDictionaryEntry FSOE0_Data_##N = { \
        .index = FSOESafeDataFSoE0Index, .subindex = (N), .name = "FSoE0 data_" #N, \
        .data_type = EtherCAT::ObjectDictionary::ObjectDictionaryDataType::Unsigned16, \
        .default_value = 0, .unit = Unit_None, .options_enum = nullptr, \
        .min_value = 0, .max_value = 0xFFFF, \
        .modification_mode = ModificationMode::ReadOnly, \
        .effective_time = EffectiveTime::Immediately, \
        .comment = "FSoE0 safe data word " #N, \
    }

NEXCOBOT_FSOE0_DATA(1); NEXCOBOT_FSOE0_DATA(2); NEXCOBOT_FSOE0_DATA(3); NEXCOBOT_FSOE0_DATA(4);
NEXCOBOT_FSOE0_DATA(5); NEXCOBOT_FSOE0_DATA(6); NEXCOBOT_FSOE0_DATA(7); NEXCOBOT_FSOE0_DATA(8);
#undef NEXCOBOT_FSOE0_DATA

// ---------------------------------------------------------------------------
// FSoE1-7 Frame / SafeData (0x6110/0x6111 .. 0x6170/0x6171)
// ---------------------------------------------------------------------------

#define NEXCOBOT_FSOE_CHAN(CH, FRAME_IDX, SAFE_IDX) \
    constexpr ::EtherCAT::ObjectDictionary::ObjectDictionaryEntry FSOE##CH##_FrameCount = { \
        .index = (FRAME_IDX), .subindex = 0x00, \
        .name = "FSOE Master Frame (FSoE" #CH ") count", \
        .data_type = EtherCAT::ObjectDictionary::ObjectDictionaryDataType::Unsigned8, \
        .default_value = 11, .unit = Unit_None, .options_enum = nullptr, \
        .min_value = 0, .max_value = 11, \
        .modification_mode = ModificationMode::ReadOnly, \
        .effective_time = EffectiveTime::Immediately, \
        .comment = "Number of entries for FSOE Master Frame FSoE" #CH, \
    }; \
    constexpr ::EtherCAT::ObjectDictionary::ObjectDictionaryEntry FSOE##CH##_Cmd = { \
        .index = (FRAME_IDX), .subindex = 0x01, .name = "FSoE" #CH " Command", \
        .data_type = EtherCAT::ObjectDictionary::ObjectDictionaryDataType::Unsigned8, \
        .default_value = 0, .unit = Unit_None, .options_enum = nullptr, \
        .min_value = 0, .max_value = 0xFF, \
        .modification_mode = ModificationMode::ReadOnly, \
        .effective_time = EffectiveTime::Immediately, .comment = "FSoE" #CH " command byte", \
    }; \
    constexpr ::EtherCAT::ObjectDictionary::ObjectDictionaryEntry FSOE##CH##_ConnID = { \
        .index = (FRAME_IDX), .subindex = 0x02, .name = "FSoE" #CH " Connection ID", \
        .data_type = EtherCAT::ObjectDictionary::ObjectDictionaryDataType::Unsigned16, \
        .default_value = 0, .unit = Unit_None, .options_enum = nullptr, \
        .min_value = 0, .max_value = 0xFFFF, \
        .modification_mode = ModificationMode::ReadOnly, \
        .effective_time = EffectiveTime::Immediately, .comment = "FSoE" #CH " connection ID", \
    }; \
    constexpr ::EtherCAT::ObjectDictionary::ObjectDictionaryEntry FSOE##CH##_CRC0 = { \
        .index = (FRAME_IDX), .subindex = 0x03, .name = "FSoE" #CH " crc_0", \
        .data_type = EtherCAT::ObjectDictionary::ObjectDictionaryDataType::Unsigned8, \
        .default_value = 0, .unit = Unit_None, .options_enum = nullptr, \
        .min_value = 0, .max_value = 0xFF, \
        .modification_mode = ModificationMode::ReadOnly, \
        .effective_time = EffectiveTime::Immediately, .comment = "FSoE" #CH " CRC byte 0", \
    }; \
    constexpr ::EtherCAT::ObjectDictionary::ObjectDictionaryEntry FSOE##CH##_CRC1 = { \
        .index = (FRAME_IDX), .subindex = 0x04, .name = "FSoE" #CH " crc_1", \
        .data_type = EtherCAT::ObjectDictionary::ObjectDictionaryDataType::Unsigned8, \
        .default_value = 0, .unit = Unit_None, .options_enum = nullptr, \
        .min_value = 0, .max_value = 0xFF, \
        .modification_mode = ModificationMode::ReadOnly, \
        .effective_time = EffectiveTime::Immediately, .comment = "FSoE" #CH " CRC byte 1", \
    }; \
    constexpr ::EtherCAT::ObjectDictionary::ObjectDictionaryEntry FSOE##CH##_CRC2 = { \
        .index = (FRAME_IDX), .subindex = 0x05, .name = "FSoE" #CH " crc_2", \
        .data_type = EtherCAT::ObjectDictionary::ObjectDictionaryDataType::Unsigned8, \
        .default_value = 0, .unit = Unit_None, .options_enum = nullptr, \
        .min_value = 0, .max_value = 0xFF, \
        .modification_mode = ModificationMode::ReadOnly, \
        .effective_time = EffectiveTime::Immediately, .comment = "FSoE" #CH " CRC byte 2", \
    }; \
    constexpr ::EtherCAT::ObjectDictionary::ObjectDictionaryEntry FSOE##CH##_CRC3 = { \
        .index = (FRAME_IDX), .subindex = 0x06, .name = "FSoE" #CH " crc_3", \
        .data_type = EtherCAT::ObjectDictionary::ObjectDictionaryDataType::Unsigned8, \
        .default_value = 0, .unit = Unit_None, .options_enum = nullptr, \
        .min_value = 0, .max_value = 0xFF, \
        .modification_mode = ModificationMode::ReadOnly, \
        .effective_time = EffectiveTime::Immediately, .comment = "FSoE" #CH " CRC byte 3", \
    }; \
    constexpr ::EtherCAT::ObjectDictionary::ObjectDictionaryEntry FSOE##CH##_CRC4 = { \
        .index = (FRAME_IDX), .subindex = 0x07, .name = "FSoE" #CH " crc_4", \
        .data_type = EtherCAT::ObjectDictionary::ObjectDictionaryDataType::Unsigned8, \
        .default_value = 0, .unit = Unit_None, .options_enum = nullptr, \
        .min_value = 0, .max_value = 0xFF, \
        .modification_mode = ModificationMode::ReadOnly, \
        .effective_time = EffectiveTime::Immediately, .comment = "FSoE" #CH " CRC byte 4", \
    }; \
    constexpr ::EtherCAT::ObjectDictionary::ObjectDictionaryEntry FSOE##CH##_CRC5 = { \
        .index = (FRAME_IDX), .subindex = 0x08, .name = "FSoE" #CH " crc_5", \
        .data_type = EtherCAT::ObjectDictionary::ObjectDictionaryDataType::Unsigned8, \
        .default_value = 0, .unit = Unit_None, .options_enum = nullptr, \
        .min_value = 0, .max_value = 0xFF, \
        .modification_mode = ModificationMode::ReadOnly, \
        .effective_time = EffectiveTime::Immediately, .comment = "FSoE" #CH " CRC byte 5", \
    }; \
    constexpr ::EtherCAT::ObjectDictionary::ObjectDictionaryEntry FSOE##CH##_CRC6 = { \
        .index = (FRAME_IDX), .subindex = 0x09, .name = "FSoE" #CH " crc_6", \
        .data_type = EtherCAT::ObjectDictionary::ObjectDictionaryDataType::Unsigned8, \
        .default_value = 0, .unit = Unit_None, .options_enum = nullptr, \
        .min_value = 0, .max_value = 0xFF, \
        .modification_mode = ModificationMode::ReadOnly, \
        .effective_time = EffectiveTime::Immediately, .comment = "FSoE" #CH " CRC byte 6", \
    }; \
    constexpr ::EtherCAT::ObjectDictionary::ObjectDictionaryEntry FSOE##CH##_CRC7 = { \
        .index = (FRAME_IDX), .subindex = 0x0A, .name = "FSoE" #CH " crc_7", \
        .data_type = EtherCAT::ObjectDictionary::ObjectDictionaryDataType::Unsigned8, \
        .default_value = 0, .unit = Unit_None, .options_enum = nullptr, \
        .min_value = 0, .max_value = 0xFF, \
        .modification_mode = ModificationMode::ReadOnly, \
        .effective_time = EffectiveTime::Immediately, .comment = "FSoE" #CH " CRC byte 7", \
    }; \
    constexpr ::EtherCAT::ObjectDictionary::ObjectDictionaryEntry FSOE##CH##_SafeCount = { \
        .index = (SAFE_IDX), .subindex = 0x00, \
        .name = "FSOE Master SafeData (FSoE" #CH ") count", \
        .data_type = EtherCAT::ObjectDictionary::ObjectDictionaryDataType::Unsigned8, \
        .default_value = 8, .unit = Unit_None, .options_enum = nullptr, \
        .min_value = 0, .max_value = 8, \
        .modification_mode = ModificationMode::ReadOnly, \
        .effective_time = EffectiveTime::Immediately, \
        .comment = "Number of entries for FSOE Master SafeData FSoE" #CH, \
    }; \
    constexpr ::EtherCAT::ObjectDictionary::ObjectDictionaryEntry FSOE##CH##_D1 = { \
        .index = (SAFE_IDX), .subindex = 0x01, .name = "FSoE" #CH " data_1", \
        .data_type = EtherCAT::ObjectDictionary::ObjectDictionaryDataType::Unsigned16, \
        .default_value = 0, .unit = Unit_None, .options_enum = nullptr, \
        .min_value = 0, .max_value = 0xFFFF, \
        .modification_mode = ModificationMode::ReadOnly, \
        .effective_time = EffectiveTime::Immediately, .comment = "FSoE" #CH " safe data word 1", \
    }; \
    constexpr ::EtherCAT::ObjectDictionary::ObjectDictionaryEntry FSOE##CH##_D2 = { \
        .index = (SAFE_IDX), .subindex = 0x02, .name = "FSoE" #CH " data_2", \
        .data_type = EtherCAT::ObjectDictionary::ObjectDictionaryDataType::Unsigned16, \
        .default_value = 0, .unit = Unit_None, .options_enum = nullptr, \
        .min_value = 0, .max_value = 0xFFFF, \
        .modification_mode = ModificationMode::ReadOnly, \
        .effective_time = EffectiveTime::Immediately, .comment = "FSoE" #CH " safe data word 2", \
    }; \
    constexpr ::EtherCAT::ObjectDictionary::ObjectDictionaryEntry FSOE##CH##_D3 = { \
        .index = (SAFE_IDX), .subindex = 0x03, .name = "FSoE" #CH " data_3", \
        .data_type = EtherCAT::ObjectDictionary::ObjectDictionaryDataType::Unsigned16, \
        .default_value = 0, .unit = Unit_None, .options_enum = nullptr, \
        .min_value = 0, .max_value = 0xFFFF, \
        .modification_mode = ModificationMode::ReadOnly, \
        .effective_time = EffectiveTime::Immediately, .comment = "FSoE" #CH " safe data word 3", \
    }; \
    constexpr ::EtherCAT::ObjectDictionary::ObjectDictionaryEntry FSOE##CH##_D4 = { \
        .index = (SAFE_IDX), .subindex = 0x04, .name = "FSoE" #CH " data_4", \
        .data_type = EtherCAT::ObjectDictionary::ObjectDictionaryDataType::Unsigned16, \
        .default_value = 0, .unit = Unit_None, .options_enum = nullptr, \
        .min_value = 0, .max_value = 0xFFFF, \
        .modification_mode = ModificationMode::ReadOnly, \
        .effective_time = EffectiveTime::Immediately, .comment = "FSoE" #CH " safe data word 4", \
    }; \
    constexpr ::EtherCAT::ObjectDictionary::ObjectDictionaryEntry FSOE##CH##_D5 = { \
        .index = (SAFE_IDX), .subindex = 0x05, .name = "FSoE" #CH " data_5", \
        .data_type = EtherCAT::ObjectDictionary::ObjectDictionaryDataType::Unsigned16, \
        .default_value = 0, .unit = Unit_None, .options_enum = nullptr, \
        .min_value = 0, .max_value = 0xFFFF, \
        .modification_mode = ModificationMode::ReadOnly, \
        .effective_time = EffectiveTime::Immediately, .comment = "FSoE" #CH " safe data word 5", \
    }; \
    constexpr ::EtherCAT::ObjectDictionary::ObjectDictionaryEntry FSOE##CH##_D6 = { \
        .index = (SAFE_IDX), .subindex = 0x06, .name = "FSoE" #CH " data_6", \
        .data_type = EtherCAT::ObjectDictionary::ObjectDictionaryDataType::Unsigned16, \
        .default_value = 0, .unit = Unit_None, .options_enum = nullptr, \
        .min_value = 0, .max_value = 0xFFFF, \
        .modification_mode = ModificationMode::ReadOnly, \
        .effective_time = EffectiveTime::Immediately, .comment = "FSoE" #CH " safe data word 6", \
    }; \
    constexpr ::EtherCAT::ObjectDictionary::ObjectDictionaryEntry FSOE##CH##_D7 = { \
        .index = (SAFE_IDX), .subindex = 0x07, .name = "FSoE" #CH " data_7", \
        .data_type = EtherCAT::ObjectDictionary::ObjectDictionaryDataType::Unsigned16, \
        .default_value = 0, .unit = Unit_None, .options_enum = nullptr, \
        .min_value = 0, .max_value = 0xFFFF, \
        .modification_mode = ModificationMode::ReadOnly, \
        .effective_time = EffectiveTime::Immediately, .comment = "FSoE" #CH " safe data word 7", \
    }; \
    constexpr ::EtherCAT::ObjectDictionary::ObjectDictionaryEntry FSOE##CH##_D8 = { \
        .index = (SAFE_IDX), .subindex = 0x08, .name = "FSoE" #CH " data_8", \
        .data_type = EtherCAT::ObjectDictionary::ObjectDictionaryDataType::Unsigned16, \
        .default_value = 0, .unit = Unit_None, .options_enum = nullptr, \
        .min_value = 0, .max_value = 0xFFFF, \
        .modification_mode = ModificationMode::ReadOnly, \
        .effective_time = EffectiveTime::Immediately, .comment = "FSoE" #CH " safe data word 8", \
    }

NEXCOBOT_FSOE_CHAN(1, 0x6110, 0x6111);
NEXCOBOT_FSOE_CHAN(2, 0x6120, 0x6121);
NEXCOBOT_FSOE_CHAN(3, 0x6130, 0x6131);
NEXCOBOT_FSOE_CHAN(4, 0x6140, 0x6141);
NEXCOBOT_FSOE_CHAN(5, 0x6150, 0x6151);
NEXCOBOT_FSOE_CHAN(6, 0x6160, 0x6161);
NEXCOBOT_FSOE_CHAN(7, 0x6170, 0x6171);

#undef NEXCOBOT_FSOE_CHAN

inline const RegisterList kRegisterList = {
    &FSOEFrameFSoE0Count, &FSOE0_FSoECommand, &FSOE0_ConnectionID,
    &FSOE0_CRC_0, &FSOE0_CRC_1, &FSOE0_CRC_2, &FSOE0_CRC_3,
    &FSOE0_CRC_4, &FSOE0_CRC_5, &FSOE0_CRC_6, &FSOE0_CRC_7,
    &FSOESafeDataFSoE0Count,
    &FSOE0_Data_1, &FSOE0_Data_2, &FSOE0_Data_3, &FSOE0_Data_4,
    &FSOE0_Data_5, &FSOE0_Data_6, &FSOE0_Data_7, &FSOE0_Data_8,
    &FSOE1_FrameCount, &FSOE1_Cmd, &FSOE1_ConnID,
    &FSOE1_CRC0, &FSOE1_CRC1, &FSOE1_CRC2, &FSOE1_CRC3,
    &FSOE1_CRC4, &FSOE1_CRC5, &FSOE1_CRC6, &FSOE1_CRC7,
    &FSOE1_SafeCount,
    &FSOE1_D1, &FSOE1_D2, &FSOE1_D3, &FSOE1_D4,
    &FSOE1_D5, &FSOE1_D6, &FSOE1_D7, &FSOE1_D8,
    &FSOE2_FrameCount, &FSOE2_Cmd, &FSOE2_ConnID,
    &FSOE2_CRC0, &FSOE2_CRC1, &FSOE2_CRC2, &FSOE2_CRC3,
    &FSOE2_CRC4, &FSOE2_CRC5, &FSOE2_CRC6, &FSOE2_CRC7,
    &FSOE2_SafeCount,
    &FSOE2_D1, &FSOE2_D2, &FSOE2_D3, &FSOE2_D4,
    &FSOE2_D5, &FSOE2_D6, &FSOE2_D7, &FSOE2_D8,
    &FSOE3_FrameCount, &FSOE3_Cmd, &FSOE3_ConnID,
    &FSOE3_CRC0, &FSOE3_CRC1, &FSOE3_CRC2, &FSOE3_CRC3,
    &FSOE3_CRC4, &FSOE3_CRC5, &FSOE3_CRC6, &FSOE3_CRC7,
    &FSOE3_SafeCount,
    &FSOE3_D1, &FSOE3_D2, &FSOE3_D3, &FSOE3_D4,
    &FSOE3_D5, &FSOE3_D6, &FSOE3_D7, &FSOE3_D8,
    &FSOE4_FrameCount, &FSOE4_Cmd, &FSOE4_ConnID,
    &FSOE4_CRC0, &FSOE4_CRC1, &FSOE4_CRC2, &FSOE4_CRC3,
    &FSOE4_CRC4, &FSOE4_CRC5, &FSOE4_CRC6, &FSOE4_CRC7,
    &FSOE4_SafeCount,
    &FSOE4_D1, &FSOE4_D2, &FSOE4_D3, &FSOE4_D4,
    &FSOE4_D5, &FSOE4_D6, &FSOE4_D7, &FSOE4_D8,
    &FSOE5_FrameCount, &FSOE5_Cmd, &FSOE5_ConnID,
    &FSOE5_CRC0, &FSOE5_CRC1, &FSOE5_CRC2, &FSOE5_CRC3,
    &FSOE5_CRC4, &FSOE5_CRC5, &FSOE5_CRC6, &FSOE5_CRC7,
    &FSOE5_SafeCount,
    &FSOE5_D1, &FSOE5_D2, &FSOE5_D3, &FSOE5_D4,
    &FSOE5_D5, &FSOE5_D6, &FSOE5_D7, &FSOE5_D8,
    &FSOE6_FrameCount, &FSOE6_Cmd, &FSOE6_ConnID,
    &FSOE6_CRC0, &FSOE6_CRC1, &FSOE6_CRC2, &FSOE6_CRC3,
    &FSOE6_CRC4, &FSOE6_CRC5, &FSOE6_CRC6, &FSOE6_CRC7,
    &FSOE6_SafeCount,
    &FSOE6_D1, &FSOE6_D2, &FSOE6_D3, &FSOE6_D4,
    &FSOE6_D5, &FSOE6_D6, &FSOE6_D7, &FSOE6_D8,
    &FSOE7_FrameCount, &FSOE7_Cmd, &FSOE7_ConnID,
    &FSOE7_CRC0, &FSOE7_CRC1, &FSOE7_CRC2, &FSOE7_CRC3,
    &FSOE7_CRC4, &FSOE7_CRC5, &FSOE7_CRC6, &FSOE7_CRC7,
    &FSOE7_SafeCount,
    &FSOE7_D1, &FSOE7_D2, &FSOE7_D3, &FSOE7_D4,
    &FSOE7_D5, &FSOE7_D6, &FSOE7_D7, &FSOE7_D8,
};

} // namespace FSOERxChannels
} // namespace NexcobotESC211
} // namespace Registers
} // namespace Drives
} // namespace EtherCAT

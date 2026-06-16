#pragma once

#include <cstdint>
#include "tether/drives/NexcobotESC211/Registers/Common.hpp"

namespace EtherCAT {
namespace Drives {
namespace Registers {
namespace NexcobotESC211 {
namespace FSOETx {

static constexpr uint16_t FSOESafetyPDUTxIndex  = 0x7000;
static constexpr uint16_t OutputCounterIndex     = 0x7010;
static constexpr uint16_t SAFE_DOIndex           = 0x7020;

constexpr ::EtherCAT::ObjectDictionary::ObjectDictionaryEntry FSOESafetyPDUTxCount = {
    .index = FSOESafetyPDUTxIndex, .subindex = 0x00,
    .name = "FSOE Master SafetyPDU Tx count",
    .data_type = EtherCAT::ObjectDictionary::ObjectDictionaryDataType::Unsigned8,
    .default_value = 8, .unit = Unit_None, .options_enum = nullptr,
    .min_value = 0, .max_value = 8,
    .modification_mode = ModificationMode::ReadOnly,
    .effective_time = EffectiveTime::Immediately,
    .comment = "Number of entries for FSOE TxPDU",
};

#define NEXCOBOT_TXPDU_REG(N) \
    constexpr ::EtherCAT::ObjectDictionary::ObjectDictionaryEntry FSOETxPDU_##N = { \
        .index = FSOESafetyPDUTxIndex, .subindex = (N), \
        .name = "FSOE TxPDU Section " #N, \
        .data_type = EtherCAT::ObjectDictionary::ObjectDictionaryDataType::OctetString, \
        .default_value = 0, .unit = Unit_None, .options_enum = nullptr, \
        .min_value = 0, .max_value = 0xFF, \
        .modification_mode = ModificationMode::ReadOnly, \
        .effective_time = EffectiveTime::Immediately, \
        .comment = "FSOE TxPDU section " #N, \
    }

NEXCOBOT_TXPDU_REG(1); NEXCOBOT_TXPDU_REG(2); NEXCOBOT_TXPDU_REG(3); NEXCOBOT_TXPDU_REG(4);
NEXCOBOT_TXPDU_REG(5); NEXCOBOT_TXPDU_REG(6); NEXCOBOT_TXPDU_REG(7); NEXCOBOT_TXPDU_REG(8);
#undef NEXCOBOT_TXPDU_REG

#define NEXCOBOT_UDINT_REG(NAME, IDX, DESC) \
    constexpr ::EtherCAT::ObjectDictionary::ObjectDictionaryEntry NAME = { \
        .index = (IDX), .subindex = 0x00, .name = (DESC), \
        .data_type = EtherCAT::ObjectDictionary::ObjectDictionaryDataType::Unsigned32, \
        .default_value = 0, .unit = Unit_None, .options_enum = nullptr, \
        .min_value = 0, .max_value = 0xFFFFFFFF, \
        .modification_mode = ModificationMode::ReadOnly, \
        .effective_time = EffectiveTime::Immediately, .comment = (DESC), \
    }

NEXCOBOT_UDINT_REG(OutputCounter, OutputCounterIndex, "OutputCounter");
NEXCOBOT_UDINT_REG(SAFE_DO,      SAFE_DOIndex,       "SAFE_DO");

#undef NEXCOBOT_UDINT_REG

// ---------------------------------------------------------------------------
// FSoE0-7 Frame / SafeData for TxPDU
// ---------------------------------------------------------------------------

#define NEXCOBOT_FSOE_TX_CHAN(CH, FRAME_IDX, SAFE_IDX) \
    constexpr ::EtherCAT::ObjectDictionary::ObjectDictionaryEntry FSOETx##CH##_FrameCount = { \
        .index = (FRAME_IDX), .subindex = 0x00, \
        .name = "FSOE Master Frame (FSoE" #CH ") Tx count", \
        .data_type = EtherCAT::ObjectDictionary::ObjectDictionaryDataType::Unsigned8, \
        .default_value = 11, .unit = Unit_None, .options_enum = nullptr, \
        .min_value = 0, .max_value = 11, \
        .modification_mode = ModificationMode::ReadOnly, \
        .effective_time = EffectiveTime::Immediately, \
        .comment = "Number of entries for FSOE Master Frame FSoE" #CH " Tx", \
    }; \
    constexpr ::EtherCAT::ObjectDictionary::ObjectDictionaryEntry FSOETx##CH##_Cmd = { \
        .index = (FRAME_IDX), .subindex = 0x01, .name = "FSoE" #CH " Tx Command", \
        .data_type = EtherCAT::ObjectDictionary::ObjectDictionaryDataType::Unsigned8, \
        .default_value = 0, .unit = Unit_None, .options_enum = nullptr, \
        .min_value = 0, .max_value = 0xFF, \
        .modification_mode = ModificationMode::ReadOnly, \
        .effective_time = EffectiveTime::Immediately, .comment = "FSoE" #CH " Tx command byte", \
    }; \
    constexpr ::EtherCAT::ObjectDictionary::ObjectDictionaryEntry FSOETx##CH##_ConnID = { \
        .index = (FRAME_IDX), .subindex = 0x02, .name = "FSoE" #CH " Tx Connection ID", \
        .data_type = EtherCAT::ObjectDictionary::ObjectDictionaryDataType::Unsigned16, \
        .default_value = 0, .unit = Unit_None, .options_enum = nullptr, \
        .min_value = 0, .max_value = 0xFFFF, \
        .modification_mode = ModificationMode::ReadOnly, \
        .effective_time = EffectiveTime::Immediately, .comment = "FSoE" #CH " Tx connection ID", \
    }; \
    constexpr ::EtherCAT::ObjectDictionary::ObjectDictionaryEntry FSOETx##CH##_CRC0 = { \
        .index = (FRAME_IDX), .subindex = 0x03, .name = "FSoE" #CH " Tx crc_0", \
        .data_type = EtherCAT::ObjectDictionary::ObjectDictionaryDataType::Unsigned8, \
        .default_value = 0, .unit = Unit_None, .options_enum = nullptr, \
        .min_value = 0, .max_value = 0xFF, \
        .modification_mode = ModificationMode::ReadOnly, \
        .effective_time = EffectiveTime::Immediately, .comment = "FSoE" #CH " Tx CRC byte 0", \
    }; \
    constexpr ::EtherCAT::ObjectDictionary::ObjectDictionaryEntry FSOETx##CH##_CRC1 = { \
        .index = (FRAME_IDX), .subindex = 0x04, .name = "FSoE" #CH " Tx crc_1", \
        .data_type = EtherCAT::ObjectDictionary::ObjectDictionaryDataType::Unsigned8, \
        .default_value = 0, .unit = Unit_None, .options_enum = nullptr, \
        .min_value = 0, .max_value = 0xFF, \
        .modification_mode = ModificationMode::ReadOnly, \
        .effective_time = EffectiveTime::Immediately, .comment = "FSoE" #CH " Tx CRC byte 1", \
    }; \
    constexpr ::EtherCAT::ObjectDictionary::ObjectDictionaryEntry FSOETx##CH##_CRC2 = { \
        .index = (FRAME_IDX), .subindex = 0x05, .name = "FSoE" #CH " Tx crc_2", \
        .data_type = EtherCAT::ObjectDictionary::ObjectDictionaryDataType::Unsigned8, \
        .default_value = 0, .unit = Unit_None, .options_enum = nullptr, \
        .min_value = 0, .max_value = 0xFF, \
        .modification_mode = ModificationMode::ReadOnly, \
        .effective_time = EffectiveTime::Immediately, .comment = "FSoE" #CH " Tx CRC byte 2", \
    }; \
    constexpr ::EtherCAT::ObjectDictionary::ObjectDictionaryEntry FSOETx##CH##_CRC3 = { \
        .index = (FRAME_IDX), .subindex = 0x06, .name = "FSoE" #CH " Tx crc_3", \
        .data_type = EtherCAT::ObjectDictionary::ObjectDictionaryDataType::Unsigned8, \
        .default_value = 0, .unit = Unit_None, .options_enum = nullptr, \
        .min_value = 0, .max_value = 0xFF, \
        .modification_mode = ModificationMode::ReadOnly, \
        .effective_time = EffectiveTime::Immediately, .comment = "FSoE" #CH " Tx CRC byte 3", \
    }; \
    constexpr ::EtherCAT::ObjectDictionary::ObjectDictionaryEntry FSOETx##CH##_CRC4 = { \
        .index = (FRAME_IDX), .subindex = 0x07, .name = "FSoE" #CH " Tx crc_4", \
        .data_type = EtherCAT::ObjectDictionary::ObjectDictionaryDataType::Unsigned8, \
        .default_value = 0, .unit = Unit_None, .options_enum = nullptr, \
        .min_value = 0, .max_value = 0xFF, \
        .modification_mode = ModificationMode::ReadOnly, \
        .effective_time = EffectiveTime::Immediately, .comment = "FSoE" #CH " Tx CRC byte 4", \
    }; \
    constexpr ::EtherCAT::ObjectDictionary::ObjectDictionaryEntry FSOETx##CH##_CRC5 = { \
        .index = (FRAME_IDX), .subindex = 0x08, .name = "FSoE" #CH " Tx crc_5", \
        .data_type = EtherCAT::ObjectDictionary::ObjectDictionaryDataType::Unsigned8, \
        .default_value = 0, .unit = Unit_None, .options_enum = nullptr, \
        .min_value = 0, .max_value = 0xFF, \
        .modification_mode = ModificationMode::ReadOnly, \
        .effective_time = EffectiveTime::Immediately, .comment = "FSoE" #CH " Tx CRC byte 5", \
    }; \
    constexpr ::EtherCAT::ObjectDictionary::ObjectDictionaryEntry FSOETx##CH##_CRC6 = { \
        .index = (FRAME_IDX), .subindex = 0x09, .name = "FSoE" #CH " Tx crc_6", \
        .data_type = EtherCAT::ObjectDictionary::ObjectDictionaryDataType::Unsigned8, \
        .default_value = 0, .unit = Unit_None, .options_enum = nullptr, \
        .min_value = 0, .max_value = 0xFF, \
        .modification_mode = ModificationMode::ReadOnly, \
        .effective_time = EffectiveTime::Immediately, .comment = "FSoE" #CH " Tx CRC byte 6", \
    }; \
    constexpr ::EtherCAT::ObjectDictionary::ObjectDictionaryEntry FSOETx##CH##_CRC7 = { \
        .index = (FRAME_IDX), .subindex = 0x0A, .name = "FSoE" #CH " Tx crc_7", \
        .data_type = EtherCAT::ObjectDictionary::ObjectDictionaryDataType::Unsigned8, \
        .default_value = 0, .unit = Unit_None, .options_enum = nullptr, \
        .min_value = 0, .max_value = 0xFF, \
        .modification_mode = ModificationMode::ReadOnly, \
        .effective_time = EffectiveTime::Immediately, .comment = "FSoE" #CH " Tx CRC byte 7", \
    }; \
    constexpr ::EtherCAT::ObjectDictionary::ObjectDictionaryEntry FSOETx##CH##_SafeCount = { \
        .index = (SAFE_IDX), .subindex = 0x00, \
        .name = "FSOE Master SafeData (FSoE" #CH ") Tx count", \
        .data_type = EtherCAT::ObjectDictionary::ObjectDictionaryDataType::Unsigned8, \
        .default_value = 8, .unit = Unit_None, .options_enum = nullptr, \
        .min_value = 0, .max_value = 8, \
        .modification_mode = ModificationMode::ReadOnly, \
        .effective_time = EffectiveTime::Immediately, \
        .comment = "Number of entries for FSOE Master SafeData FSoE" #CH " Tx", \
    }; \
    constexpr ::EtherCAT::ObjectDictionary::ObjectDictionaryEntry FSOETx##CH##_D1 = { \
        .index = (SAFE_IDX), .subindex = 0x01, .name = "FSoE" #CH " Tx data_1", \
        .data_type = EtherCAT::ObjectDictionary::ObjectDictionaryDataType::Unsigned16, \
        .default_value = 0, .unit = Unit_None, .options_enum = nullptr, \
        .min_value = 0, .max_value = 0xFFFF, \
        .modification_mode = ModificationMode::ReadOnly, \
        .effective_time = EffectiveTime::Immediately, .comment = "FSoE" #CH " Tx safe data word 1", \
    }; \
    constexpr ::EtherCAT::ObjectDictionary::ObjectDictionaryEntry FSOETx##CH##_D2 = { \
        .index = (SAFE_IDX), .subindex = 0x02, .name = "FSoE" #CH " Tx data_2", \
        .data_type = EtherCAT::ObjectDictionary::ObjectDictionaryDataType::Unsigned16, \
        .default_value = 0, .unit = Unit_None, .options_enum = nullptr, \
        .min_value = 0, .max_value = 0xFFFF, \
        .modification_mode = ModificationMode::ReadOnly, \
        .effective_time = EffectiveTime::Immediately, .comment = "FSoE" #CH " Tx safe data word 2", \
    }; \
    constexpr ::EtherCAT::ObjectDictionary::ObjectDictionaryEntry FSOETx##CH##_D3 = { \
        .index = (SAFE_IDX), .subindex = 0x03, .name = "FSoE" #CH " Tx data_3", \
        .data_type = EtherCAT::ObjectDictionary::ObjectDictionaryDataType::Unsigned16, \
        .default_value = 0, .unit = Unit_None, .options_enum = nullptr, \
        .min_value = 0, .max_value = 0xFFFF, \
        .modification_mode = ModificationMode::ReadOnly, \
        .effective_time = EffectiveTime::Immediately, .comment = "FSoE" #CH " Tx safe data word 3", \
    }; \
    constexpr ::EtherCAT::ObjectDictionary::ObjectDictionaryEntry FSOETx##CH##_D4 = { \
        .index = (SAFE_IDX), .subindex = 0x04, .name = "FSoE" #CH " Tx data_4", \
        .data_type = EtherCAT::ObjectDictionary::ObjectDictionaryDataType::Unsigned16, \
        .default_value = 0, .unit = Unit_None, .options_enum = nullptr, \
        .min_value = 0, .max_value = 0xFFFF, \
        .modification_mode = ModificationMode::ReadOnly, \
        .effective_time = EffectiveTime::Immediately, .comment = "FSoE" #CH " Tx safe data word 4", \
    }; \
    constexpr ::EtherCAT::ObjectDictionary::ObjectDictionaryEntry FSOETx##CH##_D5 = { \
        .index = (SAFE_IDX), .subindex = 0x05, .name = "FSoE" #CH " Tx data_5", \
        .data_type = EtherCAT::ObjectDictionary::ObjectDictionaryDataType::Unsigned16, \
        .default_value = 0, .unit = Unit_None, .options_enum = nullptr, \
        .min_value = 0, .max_value = 0xFFFF, \
        .modification_mode = ModificationMode::ReadOnly, \
        .effective_time = EffectiveTime::Immediately, .comment = "FSoE" #CH " Tx safe data word 5", \
    }; \
    constexpr ::EtherCAT::ObjectDictionary::ObjectDictionaryEntry FSOETx##CH##_D6 = { \
        .index = (SAFE_IDX), .subindex = 0x06, .name = "FSoE" #CH " Tx data_6", \
        .data_type = EtherCAT::ObjectDictionary::ObjectDictionaryDataType::Unsigned16, \
        .default_value = 0, .unit = Unit_None, .options_enum = nullptr, \
        .min_value = 0, .max_value = 0xFFFF, \
        .modification_mode = ModificationMode::ReadOnly, \
        .effective_time = EffectiveTime::Immediately, .comment = "FSoE" #CH " Tx safe data word 6", \
    }; \
    constexpr ::EtherCAT::ObjectDictionary::ObjectDictionaryEntry FSOETx##CH##_D7 = { \
        .index = (SAFE_IDX), .subindex = 0x07, .name = "FSoE" #CH " Tx data_7", \
        .data_type = EtherCAT::ObjectDictionary::ObjectDictionaryDataType::Unsigned16, \
        .default_value = 0, .unit = Unit_None, .options_enum = nullptr, \
        .min_value = 0, .max_value = 0xFFFF, \
        .modification_mode = ModificationMode::ReadOnly, \
        .effective_time = EffectiveTime::Immediately, .comment = "FSoE" #CH " Tx safe data word 7", \
    }; \
    constexpr ::EtherCAT::ObjectDictionary::ObjectDictionaryEntry FSOETx##CH##_D8 = { \
        .index = (SAFE_IDX), .subindex = 0x08, .name = "FSoE" #CH " Tx data_8", \
        .data_type = EtherCAT::ObjectDictionary::ObjectDictionaryDataType::Unsigned16, \
        .default_value = 0, .unit = Unit_None, .options_enum = nullptr, \
        .min_value = 0, .max_value = 0xFFFF, \
        .modification_mode = ModificationMode::ReadOnly, \
        .effective_time = EffectiveTime::Immediately, .comment = "FSoE" #CH " Tx safe data word 8", \
    }

NEXCOBOT_FSOE_TX_CHAN(0, 0x7100, 0x7101);
NEXCOBOT_FSOE_TX_CHAN(1, 0x7110, 0x7111);
NEXCOBOT_FSOE_TX_CHAN(2, 0x7120, 0x7121);
NEXCOBOT_FSOE_TX_CHAN(3, 0x7130, 0x7131);
NEXCOBOT_FSOE_TX_CHAN(4, 0x7140, 0x7141);
NEXCOBOT_FSOE_TX_CHAN(5, 0x7150, 0x7151);
NEXCOBOT_FSOE_TX_CHAN(6, 0x7160, 0x7161);
NEXCOBOT_FSOE_TX_CHAN(7, 0x7170, 0x7171);

#undef NEXCOBOT_FSOE_TX_CHAN

inline const RegisterList kRegisterList = {
    &FSOESafetyPDUTxCount,
    &FSOETxPDU_1, &FSOETxPDU_2, &FSOETxPDU_3, &FSOETxPDU_4,
    &FSOETxPDU_5, &FSOETxPDU_6, &FSOETxPDU_7, &FSOETxPDU_8,
    &OutputCounter, &SAFE_DO,
    &FSOETx0_FrameCount, &FSOETx0_Cmd, &FSOETx0_ConnID,
    &FSOETx0_CRC0, &FSOETx0_CRC1, &FSOETx0_CRC2, &FSOETx0_CRC3,
    &FSOETx0_CRC4, &FSOETx0_CRC5, &FSOETx0_CRC6, &FSOETx0_CRC7,
    &FSOETx0_SafeCount,
    &FSOETx0_D1, &FSOETx0_D2, &FSOETx0_D3, &FSOETx0_D4,
    &FSOETx0_D5, &FSOETx0_D6, &FSOETx0_D7, &FSOETx0_D8,
    &FSOETx1_FrameCount, &FSOETx1_Cmd, &FSOETx1_ConnID,
    &FSOETx1_CRC0, &FSOETx1_CRC1, &FSOETx1_CRC2, &FSOETx1_CRC3,
    &FSOETx1_CRC4, &FSOETx1_CRC5, &FSOETx1_CRC6, &FSOETx1_CRC7,
    &FSOETx1_SafeCount,
    &FSOETx1_D1, &FSOETx1_D2, &FSOETx1_D3, &FSOETx1_D4,
    &FSOETx1_D5, &FSOETx1_D6, &FSOETx1_D7, &FSOETx1_D8,
    &FSOETx2_FrameCount, &FSOETx2_Cmd, &FSOETx2_ConnID,
    &FSOETx2_CRC0, &FSOETx2_CRC1, &FSOETx2_CRC2, &FSOETx2_CRC3,
    &FSOETx2_CRC4, &FSOETx2_CRC5, &FSOETx2_CRC6, &FSOETx2_CRC7,
    &FSOETx2_SafeCount,
    &FSOETx2_D1, &FSOETx2_D2, &FSOETx2_D3, &FSOETx2_D4,
    &FSOETx2_D5, &FSOETx2_D6, &FSOETx2_D7, &FSOETx2_D8,
    &FSOETx3_FrameCount, &FSOETx3_Cmd, &FSOETx3_ConnID,
    &FSOETx3_CRC0, &FSOETx3_CRC1, &FSOETx3_CRC2, &FSOETx3_CRC3,
    &FSOETx3_CRC4, &FSOETx3_CRC5, &FSOETx3_CRC6, &FSOETx3_CRC7,
    &FSOETx3_SafeCount,
    &FSOETx3_D1, &FSOETx3_D2, &FSOETx3_D3, &FSOETx3_D4,
    &FSOETx3_D5, &FSOETx3_D6, &FSOETx3_D7, &FSOETx3_D8,
    &FSOETx4_FrameCount, &FSOETx4_Cmd, &FSOETx4_ConnID,
    &FSOETx4_CRC0, &FSOETx4_CRC1, &FSOETx4_CRC2, &FSOETx4_CRC3,
    &FSOETx4_CRC4, &FSOETx4_CRC5, &FSOETx4_CRC6, &FSOETx4_CRC7,
    &FSOETx4_SafeCount,
    &FSOETx4_D1, &FSOETx4_D2, &FSOETx4_D3, &FSOETx4_D4,
    &FSOETx4_D5, &FSOETx4_D6, &FSOETx4_D7, &FSOETx4_D8,
    &FSOETx5_FrameCount, &FSOETx5_Cmd, &FSOETx5_ConnID,
    &FSOETx5_CRC0, &FSOETx5_CRC1, &FSOETx5_CRC2, &FSOETx5_CRC3,
    &FSOETx5_CRC4, &FSOETx5_CRC5, &FSOETx5_CRC6, &FSOETx5_CRC7,
    &FSOETx5_SafeCount,
    &FSOETx5_D1, &FSOETx5_D2, &FSOETx5_D3, &FSOETx5_D4,
    &FSOETx5_D5, &FSOETx5_D6, &FSOETx5_D7, &FSOETx5_D8,
    &FSOETx6_FrameCount, &FSOETx6_Cmd, &FSOETx6_ConnID,
    &FSOETx6_CRC0, &FSOETx6_CRC1, &FSOETx6_CRC2, &FSOETx6_CRC3,
    &FSOETx6_CRC4, &FSOETx6_CRC5, &FSOETx6_CRC6, &FSOETx6_CRC7,
    &FSOETx6_SafeCount,
    &FSOETx6_D1, &FSOETx6_D2, &FSOETx6_D3, &FSOETx6_D4,
    &FSOETx6_D5, &FSOETx6_D6, &FSOETx6_D7, &FSOETx6_D8,
    &FSOETx7_FrameCount, &FSOETx7_Cmd, &FSOETx7_ConnID,
    &FSOETx7_CRC0, &FSOETx7_CRC1, &FSOETx7_CRC2, &FSOETx7_CRC3,
    &FSOETx7_CRC4, &FSOETx7_CRC5, &FSOETx7_CRC6, &FSOETx7_CRC7,
    &FSOETx7_SafeCount,
    &FSOETx7_D1, &FSOETx7_D2, &FSOETx7_D3, &FSOETx7_D4,
    &FSOETx7_D5, &FSOETx7_D6, &FSOETx7_D7, &FSOETx7_D8,
};

} // namespace FSOETx
} // namespace NexcobotESC211
} // namespace Registers
} // namespace Drives
} // namespace EtherCAT

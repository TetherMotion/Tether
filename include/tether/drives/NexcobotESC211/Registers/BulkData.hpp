#pragma once

#include <cstdint>
#include "tether/drives/NexcobotESC211/Registers/Common.hpp"

namespace EtherCAT {
namespace Drives {
namespace Registers {
namespace NexcobotESC211 {
namespace BulkData {

static constexpr uint16_t TempFNIDataIndex        = 0xF200;
static constexpr uint16_t ActiveFNIDataIndex      = 0xF201;
static constexpr uint16_t ActiveFNIDataCRCIndex   = 0xF202;
static constexpr uint16_t RSPDataInputIndex       = 0xF210;
static constexpr uint16_t RSPDataOutputIndex       = 0xF211;
static constexpr uint16_t RSPDataCRCIndex         = 0xF212;
static constexpr uint16_t SDDDataInputIndex        = 0xF220;
static constexpr uint16_t SDDDataOutputIndex       = 0xF221;
static constexpr uint16_t SDDDataCRCIndex          = 0xF222;

// ---------------------------------------------------------------------------
// 0xF200: Temp. FNI Data
// ---------------------------------------------------------------------------

constexpr ::EtherCAT::ObjectDictionary::ObjectDictionaryEntry TempFNIDataCount = {
    .index = TempFNIDataIndex,
    .subindex = 0x00,
    .name = "Temp. FNI Data count",
    .data_type = EtherCAT::ObjectDictionary::ObjectDictionaryDataType::Unsigned8,
    .default_value = 8,
    .unit = Unit_None,
    .options_enum = nullptr,
    .min_value = 0,
    .max_value = 8,
    .modification_mode = ModificationMode::ReadOnly,
    .effective_time = EffectiveTime::Immediately,
    .comment = "Number of entries for Temp. FNI Data",
};

#define NEXCOBOT_FNI_TEMP_REG(NUM) \
    constexpr ::EtherCAT::ObjectDictionary::ObjectDictionaryEntry TempFNISection_##NUM = { \
        .index = TempFNIDataIndex, \
        .subindex = (NUM), \
        .name = "Temp. FNI Data Section " #NUM, \
        .data_type = EtherCAT::ObjectDictionary::ObjectDictionaryDataType::OctetString, \
        .default_value = 0, \
        .unit = Unit_None, \
        .options_enum = nullptr, \
        .min_value = 0, \
        .max_value = 0xFF, \
        .modification_mode = ModificationMode::DuringOperation, \
        .effective_time = EffectiveTime::Immediately, \
        .comment = "Temp. FNI data section " #NUM " (ARRAY[0..255] OF BYTE)", \
    }

NEXCOBOT_FNI_TEMP_REG(1);
NEXCOBOT_FNI_TEMP_REG(2);
NEXCOBOT_FNI_TEMP_REG(3);
NEXCOBOT_FNI_TEMP_REG(4);
NEXCOBOT_FNI_TEMP_REG(5);
NEXCOBOT_FNI_TEMP_REG(6);
NEXCOBOT_FNI_TEMP_REG(7);
NEXCOBOT_FNI_TEMP_REG(8);

#undef NEXCOBOT_FNI_TEMP_REG

// ---------------------------------------------------------------------------
// 0xF201: Active FNI Data
// ---------------------------------------------------------------------------

constexpr ::EtherCAT::ObjectDictionary::ObjectDictionaryEntry ActiveFNIDataCount = {
    .index = ActiveFNIDataIndex,
    .subindex = 0x00,
    .name = "Active FNI Data count",
    .data_type = EtherCAT::ObjectDictionary::ObjectDictionaryDataType::Unsigned16,
    .default_value = 8,
    .unit = Unit_None,
    .options_enum = nullptr,
    .min_value = 0,
    .max_value = 8,
    .modification_mode = ModificationMode::ReadOnly,
    .effective_time = EffectiveTime::Immediately,
    .comment = "Number of entries for Active FNI Data",
};

#define NEXCOBOT_FNI_ACT_REG(NUM) \
    constexpr ::EtherCAT::ObjectDictionary::ObjectDictionaryEntry ActiveFNISection_##NUM = { \
        .index = ActiveFNIDataIndex, \
        .subindex = (NUM), \
        .name = "Active FNI Data Section " #NUM, \
        .data_type = EtherCAT::ObjectDictionary::ObjectDictionaryDataType::OctetString, \
        .default_value = 0, \
        .unit = Unit_None, \
        .options_enum = nullptr, \
        .min_value = 0, \
        .max_value = 0xFF, \
        .modification_mode = ModificationMode::ReadOnly, \
        .effective_time = EffectiveTime::Immediately, \
        .comment = "Active FNI data section " #NUM " (ARRAY[0..255] OF BYTE)", \
    }

NEXCOBOT_FNI_ACT_REG(1);
NEXCOBOT_FNI_ACT_REG(2);
NEXCOBOT_FNI_ACT_REG(3);
NEXCOBOT_FNI_ACT_REG(4);
NEXCOBOT_FNI_ACT_REG(5);
NEXCOBOT_FNI_ACT_REG(6);
NEXCOBOT_FNI_ACT_REG(7);
NEXCOBOT_FNI_ACT_REG(8);

#undef NEXCOBOT_FNI_ACT_REG

// ---------------------------------------------------------------------------
// 0xF202: Active FNI Data CRC
// ---------------------------------------------------------------------------

constexpr ::EtherCAT::ObjectDictionary::ObjectDictionaryEntry ActiveFNIDataCRC = {
    .index = ActiveFNIDataCRCIndex,
    .subindex = 0x00,
    .name = "Active FNI Data CRC",
    .data_type = EtherCAT::ObjectDictionary::ObjectDictionaryDataType::Unsigned32,
    .default_value = 0,
    .unit = Unit_None,
    .options_enum = nullptr,
    .min_value = 0,
    .max_value = 0xFFFFFFFF,
    .modification_mode = ModificationMode::ReadOnly,
    .effective_time = EffectiveTime::Immediately,
    .comment = "Active FNI data CRC",
};

// ---------------------------------------------------------------------------
// 0xF210: RSP Data Input
// ---------------------------------------------------------------------------

constexpr ::EtherCAT::ObjectDictionary::ObjectDictionaryEntry RSPDataInputCount = {
    .index = RSPDataInputIndex,
    .subindex = 0x00,
    .name = "RSP Data Input count",
    .data_type = EtherCAT::ObjectDictionary::ObjectDictionaryDataType::Unsigned16,
    .default_value = 48,
    .unit = Unit_None,
    .options_enum = nullptr,
    .min_value = 0,
    .max_value = 48,
    .modification_mode = ModificationMode::ReadOnly,
    .effective_time = EffectiveTime::Immediately,
    .comment = "Number of entries for RSP Data Input",
};

#define NEXCOBOT_RSP_IN_REG(NUM) \
    constexpr ::EtherCAT::ObjectDictionary::ObjectDictionaryEntry RSPDataInputSection_##NUM = { \
        .index = RSPDataInputIndex, \
        .subindex = (NUM), \
        .name = "RSP Data Input Section " #NUM, \
        .data_type = EtherCAT::ObjectDictionary::ObjectDictionaryDataType::OctetString, \
        .default_value = 0, \
        .unit = Unit_None, \
        .options_enum = nullptr, \
        .min_value = 0, \
        .max_value = 0xFF, \
        .modification_mode = ModificationMode::DuringOperation, \
        .effective_time = EffectiveTime::Immediately, \
        .comment = "RSP data input section " #NUM " (ARRAY[0..255] OF BYTE)", \
    }

NEXCOBOT_RSP_IN_REG(1);
NEXCOBOT_RSP_IN_REG(2);
NEXCOBOT_RSP_IN_REG(3);
NEXCOBOT_RSP_IN_REG(4);
NEXCOBOT_RSP_IN_REG(5);
NEXCOBOT_RSP_IN_REG(6);
NEXCOBOT_RSP_IN_REG(7);
NEXCOBOT_RSP_IN_REG(8);
NEXCOBOT_RSP_IN_REG(9);
NEXCOBOT_RSP_IN_REG(10);
NEXCOBOT_RSP_IN_REG(11);
NEXCOBOT_RSP_IN_REG(12);
NEXCOBOT_RSP_IN_REG(13);
NEXCOBOT_RSP_IN_REG(14);
NEXCOBOT_RSP_IN_REG(15);
NEXCOBOT_RSP_IN_REG(16);
NEXCOBOT_RSP_IN_REG(17);
NEXCOBOT_RSP_IN_REG(18);
NEXCOBOT_RSP_IN_REG(19);
NEXCOBOT_RSP_IN_REG(20);
NEXCOBOT_RSP_IN_REG(21);
NEXCOBOT_RSP_IN_REG(22);
NEXCOBOT_RSP_IN_REG(23);
NEXCOBOT_RSP_IN_REG(24);
NEXCOBOT_RSP_IN_REG(25);
NEXCOBOT_RSP_IN_REG(26);
NEXCOBOT_RSP_IN_REG(27);
NEXCOBOT_RSP_IN_REG(28);
NEXCOBOT_RSP_IN_REG(29);
NEXCOBOT_RSP_IN_REG(30);
NEXCOBOT_RSP_IN_REG(31);
NEXCOBOT_RSP_IN_REG(32);
NEXCOBOT_RSP_IN_REG(33);
NEXCOBOT_RSP_IN_REG(34);
NEXCOBOT_RSP_IN_REG(35);
NEXCOBOT_RSP_IN_REG(36);
NEXCOBOT_RSP_IN_REG(37);
NEXCOBOT_RSP_IN_REG(38);
NEXCOBOT_RSP_IN_REG(39);
NEXCOBOT_RSP_IN_REG(40);
NEXCOBOT_RSP_IN_REG(41);
NEXCOBOT_RSP_IN_REG(42);
NEXCOBOT_RSP_IN_REG(43);
NEXCOBOT_RSP_IN_REG(44);
NEXCOBOT_RSP_IN_REG(45);
NEXCOBOT_RSP_IN_REG(46);
NEXCOBOT_RSP_IN_REG(47);
NEXCOBOT_RSP_IN_REG(48);

#undef NEXCOBOT_RSP_IN_REG

// ---------------------------------------------------------------------------
// 0xF211: RSP Data Output (Current Load Data)
// ---------------------------------------------------------------------------

constexpr ::EtherCAT::ObjectDictionary::ObjectDictionaryEntry RSPDataOutputCount = {
    .index = RSPDataOutputIndex,
    .subindex = 0x00,
    .name = "RSP Data Output count",
    .data_type = EtherCAT::ObjectDictionary::ObjectDictionaryDataType::Unsigned16,
    .default_value = 48,
    .unit = Unit_None,
    .options_enum = nullptr,
    .min_value = 0,
    .max_value = 48,
    .modification_mode = ModificationMode::ReadOnly,
    .effective_time = EffectiveTime::Immediately,
    .comment = "Number of entries for RSP Data Output",
};

#define NEXCOBOT_RSP_OUT_REG(NUM) \
    constexpr ::EtherCAT::ObjectDictionary::ObjectDictionaryEntry RSPDataOutputSection_##NUM = { \
        .index = RSPDataOutputIndex, \
        .subindex = (NUM), \
        .name = "RSP Data Output Section " #NUM, \
        .data_type = EtherCAT::ObjectDictionary::ObjectDictionaryDataType::OctetString, \
        .default_value = 0, \
        .unit = Unit_None, \
        .options_enum = nullptr, \
        .min_value = 0, \
        .max_value = 0xFF, \
        .modification_mode = ModificationMode::ReadOnly, \
        .effective_time = EffectiveTime::Immediately, \
        .comment = "RSP data output section " #NUM " (ARRAY[0..255] OF BYTE)", \
    }

NEXCOBOT_RSP_OUT_REG(1);
NEXCOBOT_RSP_OUT_REG(2);
NEXCOBOT_RSP_OUT_REG(3);
NEXCOBOT_RSP_OUT_REG(4);
NEXCOBOT_RSP_OUT_REG(5);
NEXCOBOT_RSP_OUT_REG(6);
NEXCOBOT_RSP_OUT_REG(7);
NEXCOBOT_RSP_OUT_REG(8);
NEXCOBOT_RSP_OUT_REG(9);
NEXCOBOT_RSP_OUT_REG(10);
NEXCOBOT_RSP_OUT_REG(11);
NEXCOBOT_RSP_OUT_REG(12);
NEXCOBOT_RSP_OUT_REG(13);
NEXCOBOT_RSP_OUT_REG(14);
NEXCOBOT_RSP_OUT_REG(15);
NEXCOBOT_RSP_OUT_REG(16);
NEXCOBOT_RSP_OUT_REG(17);
NEXCOBOT_RSP_OUT_REG(18);
NEXCOBOT_RSP_OUT_REG(19);
NEXCOBOT_RSP_OUT_REG(20);
NEXCOBOT_RSP_OUT_REG(21);
NEXCOBOT_RSP_OUT_REG(22);
NEXCOBOT_RSP_OUT_REG(23);
NEXCOBOT_RSP_OUT_REG(24);
NEXCOBOT_RSP_OUT_REG(25);
NEXCOBOT_RSP_OUT_REG(26);
NEXCOBOT_RSP_OUT_REG(27);
NEXCOBOT_RSP_OUT_REG(28);
NEXCOBOT_RSP_OUT_REG(29);
NEXCOBOT_RSP_OUT_REG(30);
NEXCOBOT_RSP_OUT_REG(31);
NEXCOBOT_RSP_OUT_REG(32);
NEXCOBOT_RSP_OUT_REG(33);
NEXCOBOT_RSP_OUT_REG(34);
NEXCOBOT_RSP_OUT_REG(35);
NEXCOBOT_RSP_OUT_REG(36);
NEXCOBOT_RSP_OUT_REG(37);
NEXCOBOT_RSP_OUT_REG(38);
NEXCOBOT_RSP_OUT_REG(39);
NEXCOBOT_RSP_OUT_REG(40);
NEXCOBOT_RSP_OUT_REG(41);
NEXCOBOT_RSP_OUT_REG(42);
NEXCOBOT_RSP_OUT_REG(43);
NEXCOBOT_RSP_OUT_REG(44);
NEXCOBOT_RSP_OUT_REG(45);
NEXCOBOT_RSP_OUT_REG(46);
NEXCOBOT_RSP_OUT_REG(47);
NEXCOBOT_RSP_OUT_REG(48);

#undef NEXCOBOT_RSP_OUT_REG

// ---------------------------------------------------------------------------
// 0xF212: RSP Data CRC
// ---------------------------------------------------------------------------

constexpr ::EtherCAT::ObjectDictionary::ObjectDictionaryEntry RSPDataCRC = {
    .index = RSPDataCRCIndex,
    .subindex = 0x00,
    .name = "RSP Data CRC",
    .data_type = EtherCAT::ObjectDictionary::ObjectDictionaryDataType::Unsigned32,
    .default_value = 0,
    .unit = Unit_None,
    .options_enum = nullptr,
    .min_value = 0,
    .max_value = 0xFFFFFFFF,
    .modification_mode = ModificationMode::ReadOnly,
    .effective_time = EffectiveTime::Immediately,
    .comment = "RSP data CRC",
};

// ---------------------------------------------------------------------------
// 0xF220: SDD Data Input
// ---------------------------------------------------------------------------

constexpr ::EtherCAT::ObjectDictionary::ObjectDictionaryEntry SDDDataInputCount = {
    .index = SDDDataInputIndex,
    .subindex = 0x00,
    .name = "SDD Data Input count",
    .data_type = EtherCAT::ObjectDictionary::ObjectDictionaryDataType::Unsigned16,
    .default_value = 24,
    .unit = Unit_None,
    .options_enum = nullptr,
    .min_value = 0,
    .max_value = 24,
    .modification_mode = ModificationMode::ReadOnly,
    .effective_time = EffectiveTime::Immediately,
    .comment = "Number of entries for SDD Data Input",
};

#define NEXCOBOT_SDD_IN_REG(NUM) \
    constexpr ::EtherCAT::ObjectDictionary::ObjectDictionaryEntry SDDDataInputSection_##NUM = { \
        .index = SDDDataInputIndex, \
        .subindex = (NUM), \
        .name = "SDD Data Input Section " #NUM, \
        .data_type = EtherCAT::ObjectDictionary::ObjectDictionaryDataType::OctetString, \
        .default_value = 0, \
        .unit = Unit_None, \
        .options_enum = nullptr, \
        .min_value = 0, \
        .max_value = 0xFF, \
        .modification_mode = ModificationMode::DuringOperation, \
        .effective_time = EffectiveTime::Immediately, \
        .comment = "SDD data input section " #NUM " (ARRAY[0..255] OF BYTE)", \
    }

NEXCOBOT_SDD_IN_REG(1);
NEXCOBOT_SDD_IN_REG(2);
NEXCOBOT_SDD_IN_REG(3);
NEXCOBOT_SDD_IN_REG(4);
NEXCOBOT_SDD_IN_REG(5);
NEXCOBOT_SDD_IN_REG(6);
NEXCOBOT_SDD_IN_REG(7);
NEXCOBOT_SDD_IN_REG(8);
NEXCOBOT_SDD_IN_REG(9);
NEXCOBOT_SDD_IN_REG(10);
NEXCOBOT_SDD_IN_REG(11);
NEXCOBOT_SDD_IN_REG(12);
NEXCOBOT_SDD_IN_REG(13);
NEXCOBOT_SDD_IN_REG(14);
NEXCOBOT_SDD_IN_REG(15);
NEXCOBOT_SDD_IN_REG(16);
NEXCOBOT_SDD_IN_REG(17);
NEXCOBOT_SDD_IN_REG(18);
NEXCOBOT_SDD_IN_REG(19);
NEXCOBOT_SDD_IN_REG(20);
NEXCOBOT_SDD_IN_REG(21);
NEXCOBOT_SDD_IN_REG(22);
NEXCOBOT_SDD_IN_REG(23);
NEXCOBOT_SDD_IN_REG(24);

#undef NEXCOBOT_SDD_IN_REG

// ---------------------------------------------------------------------------
// 0xF221: SDD Data Output (Current Load Data)
// ---------------------------------------------------------------------------

constexpr ::EtherCAT::ObjectDictionary::ObjectDictionaryEntry SDDDataOutputCount = {
    .index = SDDDataOutputIndex,
    .subindex = 0x00,
    .name = "SDD Data Output count",
    .data_type = EtherCAT::ObjectDictionary::ObjectDictionaryDataType::Unsigned16,
    .default_value = 24,
    .unit = Unit_None,
    .options_enum = nullptr,
    .min_value = 0,
    .max_value = 24,
    .modification_mode = ModificationMode::ReadOnly,
    .effective_time = EffectiveTime::Immediately,
    .comment = "Number of entries for SDD Data Output",
};

#define NEXCOBOT_SDD_OUT_REG(NUM) \
    constexpr ::EtherCAT::ObjectDictionary::ObjectDictionaryEntry SDDDataOutputSection_##NUM = { \
        .index = SDDDataOutputIndex, \
        .subindex = (NUM), \
        .name = "SDD Data Output Section " #NUM, \
        .data_type = EtherCAT::ObjectDictionary::ObjectDictionaryDataType::OctetString, \
        .default_value = 0, \
        .unit = Unit_None, \
        .options_enum = nullptr, \
        .min_value = 0, \
        .max_value = 0xFF, \
        .modification_mode = ModificationMode::ReadOnly, \
        .effective_time = EffectiveTime::Immediately, \
        .comment = "SDD data output section " #NUM " (ARRAY[0..255] OF BYTE)", \
    }

NEXCOBOT_SDD_OUT_REG(1);
NEXCOBOT_SDD_OUT_REG(2);
NEXCOBOT_SDD_OUT_REG(3);
NEXCOBOT_SDD_OUT_REG(4);
NEXCOBOT_SDD_OUT_REG(5);
NEXCOBOT_SDD_OUT_REG(6);
NEXCOBOT_SDD_OUT_REG(7);
NEXCOBOT_SDD_OUT_REG(8);
NEXCOBOT_SDD_OUT_REG(9);
NEXCOBOT_SDD_OUT_REG(10);
NEXCOBOT_SDD_OUT_REG(11);
NEXCOBOT_SDD_OUT_REG(12);
NEXCOBOT_SDD_OUT_REG(13);
NEXCOBOT_SDD_OUT_REG(14);
NEXCOBOT_SDD_OUT_REG(15);
NEXCOBOT_SDD_OUT_REG(16);
NEXCOBOT_SDD_OUT_REG(17);
NEXCOBOT_SDD_OUT_REG(18);
NEXCOBOT_SDD_OUT_REG(19);
NEXCOBOT_SDD_OUT_REG(20);
NEXCOBOT_SDD_OUT_REG(21);
NEXCOBOT_SDD_OUT_REG(22);
NEXCOBOT_SDD_OUT_REG(23);
NEXCOBOT_SDD_OUT_REG(24);

#undef NEXCOBOT_SDD_OUT_REG

// ---------------------------------------------------------------------------
// 0xF222: SDD Data CRC
// ---------------------------------------------------------------------------

constexpr ::EtherCAT::ObjectDictionary::ObjectDictionaryEntry SDDDataCRC = {
    .index = SDDDataCRCIndex,
    .subindex = 0x00,
    .name = "SDD Data CRC",
    .data_type = EtherCAT::ObjectDictionary::ObjectDictionaryDataType::Unsigned32,
    .default_value = 0,
    .unit = Unit_None,
    .options_enum = nullptr,
    .min_value = 0,
    .max_value = 0xFFFFFFFF,
    .modification_mode = ModificationMode::ReadOnly,
    .effective_time = EffectiveTime::Immediately,
    .comment = "SDD data CRC",
};

inline const RegisterList kRegisterList = {
    &TempFNIDataCount,
    &TempFNISection_1, &TempFNISection_2, &TempFNISection_3, &TempFNISection_4,
    &TempFNISection_5, &TempFNISection_6, &TempFNISection_7, &TempFNISection_8,
    &ActiveFNIDataCount,
    &ActiveFNISection_1, &ActiveFNISection_2, &ActiveFNISection_3, &ActiveFNISection_4,
    &ActiveFNISection_5, &ActiveFNISection_6, &ActiveFNISection_7, &ActiveFNISection_8,
    &ActiveFNIDataCRC,
    &RSPDataInputCount,
    &RSPDataInputSection_1, &RSPDataInputSection_2, &RSPDataInputSection_3, &RSPDataInputSection_4,
    &RSPDataInputSection_5, &RSPDataInputSection_6, &RSPDataInputSection_7, &RSPDataInputSection_8,
    &RSPDataInputSection_9, &RSPDataInputSection_10, &RSPDataInputSection_11, &RSPDataInputSection_12,
    &RSPDataInputSection_13, &RSPDataInputSection_14, &RSPDataInputSection_15, &RSPDataInputSection_16,
    &RSPDataInputSection_17, &RSPDataInputSection_18, &RSPDataInputSection_19, &RSPDataInputSection_20,
    &RSPDataInputSection_21, &RSPDataInputSection_22, &RSPDataInputSection_23, &RSPDataInputSection_24,
    &RSPDataInputSection_25, &RSPDataInputSection_26, &RSPDataInputSection_27, &RSPDataInputSection_28,
    &RSPDataInputSection_29, &RSPDataInputSection_30, &RSPDataInputSection_31, &RSPDataInputSection_32,
    &RSPDataInputSection_33, &RSPDataInputSection_34, &RSPDataInputSection_35, &RSPDataInputSection_36,
    &RSPDataInputSection_37, &RSPDataInputSection_38, &RSPDataInputSection_39, &RSPDataInputSection_40,
    &RSPDataInputSection_41, &RSPDataInputSection_42, &RSPDataInputSection_43, &RSPDataInputSection_44,
    &RSPDataInputSection_45, &RSPDataInputSection_46, &RSPDataInputSection_47, &RSPDataInputSection_48,
    &RSPDataOutputCount,
    &RSPDataOutputSection_1, &RSPDataOutputSection_2, &RSPDataOutputSection_3, &RSPDataOutputSection_4,
    &RSPDataOutputSection_5, &RSPDataOutputSection_6, &RSPDataOutputSection_7, &RSPDataOutputSection_8,
    &RSPDataOutputSection_9, &RSPDataOutputSection_10, &RSPDataOutputSection_11, &RSPDataOutputSection_12,
    &RSPDataOutputSection_13, &RSPDataOutputSection_14, &RSPDataOutputSection_15, &RSPDataOutputSection_16,
    &RSPDataOutputSection_17, &RSPDataOutputSection_18, &RSPDataOutputSection_19, &RSPDataOutputSection_20,
    &RSPDataOutputSection_21, &RSPDataOutputSection_22, &RSPDataOutputSection_23, &RSPDataOutputSection_24,
    &RSPDataOutputSection_25, &RSPDataOutputSection_26, &RSPDataOutputSection_27, &RSPDataOutputSection_28,
    &RSPDataOutputSection_29, &RSPDataOutputSection_30, &RSPDataOutputSection_31, &RSPDataOutputSection_32,
    &RSPDataOutputSection_33, &RSPDataOutputSection_34, &RSPDataOutputSection_35, &RSPDataOutputSection_36,
    &RSPDataOutputSection_37, &RSPDataOutputSection_38, &RSPDataOutputSection_39, &RSPDataOutputSection_40,
    &RSPDataOutputSection_41, &RSPDataOutputSection_42, &RSPDataOutputSection_43, &RSPDataOutputSection_44,
    &RSPDataOutputSection_45, &RSPDataOutputSection_46, &RSPDataOutputSection_47, &RSPDataOutputSection_48,
    &RSPDataCRC,
    &SDDDataInputCount,
    &SDDDataInputSection_1, &SDDDataInputSection_2, &SDDDataInputSection_3, &SDDDataInputSection_4,
    &SDDDataInputSection_5, &SDDDataInputSection_6, &SDDDataInputSection_7, &SDDDataInputSection_8,
    &SDDDataInputSection_9, &SDDDataInputSection_10, &SDDDataInputSection_11, &SDDDataInputSection_12,
    &SDDDataInputSection_13, &SDDDataInputSection_14, &SDDDataInputSection_15, &SDDDataInputSection_16,
    &SDDDataInputSection_17, &SDDDataInputSection_18, &SDDDataInputSection_19, &SDDDataInputSection_20,
    &SDDDataInputSection_21, &SDDDataInputSection_22, &SDDDataInputSection_23, &SDDDataInputSection_24,
    &SDDDataOutputCount,
    &SDDDataOutputSection_1, &SDDDataOutputSection_2, &SDDDataOutputSection_3, &SDDDataOutputSection_4,
    &SDDDataOutputSection_5, &SDDDataOutputSection_6, &SDDDataOutputSection_7, &SDDDataOutputSection_8,
    &SDDDataOutputSection_9, &SDDDataOutputSection_10, &SDDDataOutputSection_11, &SDDDataOutputSection_12,
    &SDDDataOutputSection_13, &SDDDataOutputSection_14, &SDDDataOutputSection_15, &SDDDataOutputSection_16,
    &SDDDataOutputSection_17, &SDDDataOutputSection_18, &SDDDataOutputSection_19, &SDDDataOutputSection_20,
    &SDDDataOutputSection_21, &SDDDataOutputSection_22, &SDDDataOutputSection_23, &SDDDataOutputSection_24,
    &SDDDataCRC,
};

} // namespace BulkData
} // namespace NexcobotESC211
} // namespace Registers
} // namespace Drives
} // namespace EtherCAT

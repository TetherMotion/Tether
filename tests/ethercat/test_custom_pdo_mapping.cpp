#include <gtest/gtest.h>
#include "tether/ethercat/CustomPDOMapping.hpp"

using namespace EtherCAT;
using OD = ObjectDictionary::ObjectDictionaryDataType;

TEST(CustomPDOMapping, InferByteSize) {
    EXPECT_EQ(inferByteSize(OD::Boolean), 1);
    EXPECT_EQ(inferByteSize(OD::Integer8), 1);
    EXPECT_EQ(inferByteSize(OD::Unsigned8), 1);
    EXPECT_EQ(inferByteSize(OD::Integer16), 2);
    EXPECT_EQ(inferByteSize(OD::Unsigned16), 2);
    EXPECT_EQ(inferByteSize(OD::Integer32), 4);
    EXPECT_EQ(inferByteSize(OD::Unsigned32), 4);
    EXPECT_EQ(inferByteSize(OD::Real32), 4);
    EXPECT_EQ(inferByteSize(OD::Integer64), 8);
    EXPECT_EQ(inferByteSize(OD::Unsigned64), 8);
    EXPECT_EQ(inferByteSize(OD::Real64), 8);
    EXPECT_EQ(inferByteSize(OD::OctetString), 0);
    EXPECT_EQ(inferByteSize(OD::Domain), 0);
}

TEST(CustomPDOMapping, EncodePDOMappingValue) {
    ObjectDictionary::ObjectDictionaryEntry entry = {
        .index = 0x7010,
        .subindex = 0x00,
        .name = "test",
        .data_type = OD::Unsigned32,
        .default_value = 0,
        .min_value = 0,
        .max_value = 0,
        .modification_mode = ObjectDictionary::ModificationMode::ReadOnly,
        .effective_time = ObjectDictionary::EffectiveTime::Immediately,
    };

    uint32_t val = encodePDOMappingValue(&entry, 4);
    EXPECT_EQ(val, 0x70100020U);  // index=0x7010, sub=0x00, bits=32

    entry.subindex = 0x02;
    val = encodePDOMappingValue(&entry, 2);
    EXPECT_EQ(val, 0x70100210U);  // index=0x7010, sub=0x02, bits=16
}

TEST(CustomPDOMapping, EntryInferSize) {
    ObjectDictionary::ObjectDictionaryEntry entry_u8 = {
        .index = 0x1000, .subindex = 0x01,
        .name = "u8", .data_type = OD::Unsigned8,
        .default_value = 0, .min_value = 0, .max_value = 0,
        .modification_mode = ObjectDictionary::ModificationMode::ReadOnly,
        .effective_time = ObjectDictionary::EffectiveTime::Immediately,
    };
    CustomPDOMappingEntry e(&entry_u8);
    EXPECT_EQ(e.resolvedSize(), 1);

    ObjectDictionary::ObjectDictionaryEntry entry_u32 = {
        .index = 0x2000, .subindex = 0x00,
        .name = "u32", .data_type = OD::Unsigned32,
        .default_value = 0, .min_value = 0, .max_value = 0,
        .modification_mode = ObjectDictionary::ModificationMode::ReadOnly,
        .effective_time = ObjectDictionary::EffectiveTime::Immediately,
    };
    CustomPDOMappingEntry e2(&entry_u32);
    EXPECT_EQ(e2.resolvedSize(), 4);
}

TEST(CustomPDOMapping, EntryExplicitSize) {
    ObjectDictionary::ObjectDictionaryEntry entry_octet = {
        .index = 0x6000, .subindex = 0x01,
        .name = "octet", .data_type = OD::OctetString,
        .default_value = 0, .min_value = 0, .max_value = 0,
        .modification_mode = ObjectDictionary::ModificationMode::ReadOnly,
        .effective_time = ObjectDictionary::EffectiveTime::Immediately,
    };

    CustomPDOMappingEntry e(&entry_octet);       // inferred
    EXPECT_EQ(e.resolvedSize(), 0);              // can't infer OctetString

    CustomPDOMappingEntry e_explicit(&entry_octet, 31);
    EXPECT_EQ(e_explicit.resolvedSize(), 31);
}

TEST(CustomPDOMapping, EntryMatchesExamplePattern) {
    // Simulate the RxPDO 0x1601 mapping from esc211_di_monitor:
    //   0x70100020 (OutputCounter) + 0x70200020 (SAFE_DO)
    ObjectDictionary::ObjectDictionaryEntry output_counter = {
        .index = 0x7010, .subindex = 0x00,
        .name = "OutputCounter", .data_type = OD::Unsigned32,
        .default_value = 0, .min_value = 0, .max_value = 0,
        .modification_mode = ObjectDictionary::ModificationMode::ReadOnly,
        .effective_time = ObjectDictionary::EffectiveTime::Immediately,
    };
    ObjectDictionary::ObjectDictionaryEntry safe_do = {
        .index = 0x7020, .subindex = 0x00,
        .name = "SAFE_DO", .data_type = OD::Unsigned32,
        .default_value = 0, .min_value = 0, .max_value = 0,
        .modification_mode = ObjectDictionary::ModificationMode::ReadOnly,
        .effective_time = ObjectDictionary::EffectiveTime::Immediately,
    };

    CustomPDOMappingEntry entries[] = {
        {&output_counter},
        {&safe_do},
    };

    EXPECT_EQ(entries[0].resolvedSize(), 4);
    EXPECT_EQ(entries[1].resolvedSize(), 4);

    EXPECT_EQ(encodePDOMappingValue(entries[0].entry, entries[0].resolvedSize()), 0x70100020U);
    EXPECT_EQ(encodePDOMappingValue(entries[1].entry, entries[1].resolvedSize()), 0x70200020U);
}

#include <gtest/gtest.h>

#include "tether/io/BinaryStruct.hpp"
#include "tether/io/Datalogging.hpp"
#include "tether/io/FeatureExchange.hpp"
#include "tether/io/Function.hpp"
#include "tether/io/ThresholdFilter.hpp"

#include <array>
#include <cstring>
#include <limits>

using namespace tether::io;

namespace {

template <typename DecodeFn>
bool decodeFrom(const std::vector<uint8_t>& bytes, DecodeFn&& decode) {
    BufReader reader(bytes.data(), bytes.size());
    return decode(reader);
}

std::vector<uint8_t> encodeStruct(const StructDescriptor& descriptor) {
    std::vector<uint8_t> bytes(1024);
    BufWriter writer(bytes.data(), bytes.size());
    descriptor.encode(writer);
    EXPECT_TRUE(writer.ok());
    bytes.resize(writer.pos);
    return bytes;
}

} // namespace

TEST(IOMalformed, BufWriterRejectsNullPayloadWithNonzeroLength) {
    std::array<uint8_t, 8> bytes{};
    BufWriter writer(bytes.data(), bytes.size());
    writer.putBytes(nullptr, 1);
    EXPECT_FALSE(writer.ok());
    EXPECT_EQ(writer.pos, 0u);
}

TEST(IOMalformed, BufWriterAcceptsNullPayloadWithZeroLength) {
    std::array<uint8_t, 8> bytes{};
    BufWriter writer(bytes.data(), bytes.size());
    writer.putBytes(nullptr, 0);
    EXPECT_TRUE(writer.ok());
    EXPECT_EQ(writer.pos, 0u);
}

TEST(IOMalformed, BufReaderZeroLengthAtEndDoesNotFail) {
    const uint8_t byte = 0;
    BufReader reader(&byte, 0);
    EXPECT_NE(reader.getBytes(0), nullptr);
    EXPECT_TRUE(reader.skip(0));
    EXPECT_TRUE(reader.ok());
    EXPECT_EQ(reader.remaining(), 0u);
}

TEST(IOMalformed, BufReaderRemainingDoesNotUnderflowAfterFailure) {
    const uint8_t bytes[] = {1, 2};
    BufReader reader(bytes, sizeof(bytes));
    EXPECT_EQ(reader.getU64(), 0u);
    EXPECT_FALSE(reader.ok());
    EXPECT_EQ(reader.remaining(), 2u);
}

TEST(IOMalformed, VarintRejectsTruncatedContinuationAtEveryLength) {
    for (size_t length = 1; length <= MAX_VARINT_SIZE; ++length) {
        std::vector<uint8_t> bytes(length, 0x80);
        uint32_t value = 0xFFFFFFFFu;
        EXPECT_EQ(decodeVarint(bytes.data(), bytes.size(), value), 0u)
            << "length=" << length;
    }
}

TEST(IOMalformed, VarintRejectsInvalidFifthPayload) {
    const uint8_t bytes[] = {0xFF, 0xFF, 0xFF, 0xFF, 0x10};
    uint32_t value = 0;
    EXPECT_EQ(decodeVarint(bytes, sizeof(bytes), value), 0u);
}

TEST(IOMalformed, VarintRejectsSixthByte) {
    const uint8_t bytes[] = {0x80, 0x80, 0x80, 0x80, 0x80, 0};
    uint32_t value = 0;
    EXPECT_EQ(decodeVarint(bytes, sizeof(bytes), value), 0u);
}

TEST(IOMalformed, ZigzagRoundTripsBoundaries) {
    for (const int32_t value : {std::numeric_limits<int32_t>::min(), -1, 0,
                                1, std::numeric_limits<int32_t>::max()}) {
        EXPECT_EQ(zigzagDecode32(zigzagEncode32(value)), value);
    }
    for (const int64_t value : std::array<int64_t, 5>{
             std::numeric_limits<int64_t>::min(), -1, 0, 1,
             std::numeric_limits<int64_t>::max()}) {
        EXPECT_EQ(zigzagDecode64(zigzagEncode64(value)), value);
    }
}

TEST(IOMalformed, FeatureDecodeRejectsHugeCount) {
    std::array<uint8_t, 4> bytes{};
    BufWriter writer(bytes.data(), bytes.size());
    writer.putU32(1025);
    FeatureSet output;
    EXPECT_FALSE(decodeFrom(std::vector<uint8_t>(bytes.begin(), bytes.end()),
                            [&output](BufReader& reader) {
                                return FeatureSet::decode(reader, output);
                            }));
}

TEST(IOMalformed, FeatureDecodeRejectsTruncatedValue) {
    std::array<uint8_t, 32> bytes{};
    BufWriter writer(bytes.data(), bytes.size());
    writer.putU32(1);
    writer.putStr16("x", 1);
    writer.putU8(static_cast<uint8_t>(ValueType::Binary));
    writer.putU32(4);
    writer.putBytes("ab", 2);
    ASSERT_TRUE(writer.ok());

    FeatureSet output;
    std::vector<uint8_t> encoded(bytes.begin(), bytes.begin() + writer.pos);
    EXPECT_FALSE(decodeFrom(encoded, [&output](BufReader& reader) {
        return FeatureSet::decode(reader, output);
    }));
}

TEST(IOMalformed, FeatureDecodeRejectsTruncatedName) {
    const uint8_t bytes[] = {1, 0, 0, 0, 4, 0, 'a'};
    FeatureSet output;
    EXPECT_FALSE(decodeFrom(std::vector<uint8_t>(std::begin(bytes), std::end(bytes)),
                            [&output](BufReader& reader) {
                                return FeatureSet::decode(reader, output);
                            }));
}

TEST(IOMalformed, FeatureDecodeRejectsTruncatedTypeAndLength) {
    const uint8_t bytes[] = {1, 0, 0, 0, 0, 0};
    FeatureSet output;
    EXPECT_FALSE(decodeFrom(std::vector<uint8_t>(std::begin(bytes), std::end(bytes)),
                            [&output](BufReader& reader) {
                                return FeatureSet::decode(reader, output);
                            }));
}

TEST(IOMalformed, StructDecodeRejectsHugeFieldCount) {
    std::array<uint8_t, 32> bytes{};
    BufWriter writer(bytes.data(), bytes.size());
    writer.putU64(1);
    writer.putStr16("S", 1);
    writer.putU32(0);
    writer.putU32(MAX_COLLECTION_COUNT + 1);
    ASSERT_TRUE(writer.ok());

    StructDescriptor output;
    std::vector<uint8_t> encoded(bytes.begin(), bytes.begin() + writer.pos);
    EXPECT_FALSE(decodeFrom(encoded, [&output](BufReader& reader) {
        return StructDescriptor::decode(reader, output);
    }));
}

TEST(IOMalformed, StructDecodeRejectsTruncatedField) {
    StructDescriptor source;
    source.entryId = 1;
    source.name = "S";
    source.fields.push_back({"x", ValueType::U32, 0, 4, ""});
    auto encoded = encodeStruct(source);
    encoded.pop_back();

    StructDescriptor output;
    EXPECT_FALSE(decodeFrom(encoded, [&output](BufReader& reader) {
        return StructDescriptor::decode(reader, output);
    }));
}

TEST(IOMalformed, DatalogConfigDecodeRejectsHugeEntryCount) {
    std::array<uint8_t, 32> bytes{};
    BufWriter writer(bytes.data(), bytes.size());
    writer.putStr16("log", 3);
    writer.putU32(100);
    writer.putU8(1);
    writer.putU32(MAX_COLLECTION_COUNT + 1);
    ASSERT_TRUE(writer.ok());

    DatalogConfig output;
    std::vector<uint8_t> encoded(bytes.begin(), bytes.begin() + writer.pos);
    EXPECT_FALSE(decodeFrom(encoded, [&output](BufReader& reader) {
        return DatalogConfig::decode(reader, output);
    }));
}

TEST(IOMalformed, DatalogMetadataDecodeRejectsTruncatedField) {
    DatalogMetadata source;
    source.logName = "log";
    source.fields.push_back({1, "field", ValueType::U32, 0, 4, EntryKind::Parameter});
    std::vector<uint8_t> bytes(256);
    BufWriter writer(bytes.data(), bytes.size());
    source.encode(writer);
    ASSERT_TRUE(writer.ok());
    bytes.resize(writer.pos - 1);

    DatalogMetadata output;
    EXPECT_FALSE(decodeFrom(bytes, [&output](BufReader& reader) {
        return DatalogMetadata::decode(reader, output);
    }));
}

TEST(IOMalformed, ThresholdDecodeRejectsHugeRuleCount) {
    std::array<uint8_t, 32> bytes{};
    BufWriter writer(bytes.data(), bytes.size());
    writer.putStr16("threshold", 9);
    writer.putU8(1);
    writer.putU32(MAX_COLLECTION_COUNT + 1);
    ASSERT_TRUE(writer.ok());

    ThresholdConfig output;
    std::vector<uint8_t> encoded(bytes.begin(), bytes.begin() + writer.pos);
    EXPECT_FALSE(decodeFrom(encoded, [&output](BufReader& reader) {
        return ThresholdConfig::decode(reader, output);
    }));
}

TEST(IOMalformed, ThresholdDecodeRejectsTruncatedCustomValue) {
    std::array<uint8_t, 64> bytes{};
    BufWriter writer(bytes.data(), bytes.size());
    writer.putU64(1);
    writer.putU8(static_cast<uint8_t>(ThresholdType::Custom));
    writer.putF64(0.0);
    writer.putStr16("custom", 6);
    writer.putU32(1);
    writer.putStr16("x", 1);
    writer.putU8(static_cast<uint8_t>(ValueType::Binary));
    writer.putU32(4);
    writer.putBytes("ab", 2);
    ASSERT_TRUE(writer.ok());

    ThresholdRule output;
    std::vector<uint8_t> encoded(bytes.begin(), bytes.begin() + writer.pos);
    EXPECT_FALSE(decodeFrom(encoded, [&output](BufReader& reader) {
        return ThresholdRule::decode(reader, output);
    }));
}

TEST(IOMalformed, FunctionSignatureRejectsInvalidOptionalOrdering) {
    FunctionEntry function;
    function.name = "f";
    function.callback = [](const std::vector<FunctionArgument>&) {
        return FunctionCallResult{};
    };
    function.parameters = {
        {"optional", {}, ValueType::U8, true, true, {1}},
        {"required", {}, ValueType::U8, false, false, {}}};
    EXPECT_FALSE(function.validSignature());
}

TEST(IOMalformed, FunctionSignatureRejectsOptionalWithoutDefault) {
    FunctionEntry function;
    function.name = "f";
    function.callback = [](const std::vector<FunctionArgument>&) {
        return FunctionCallResult{};
    };
    function.parameters = {{"optional", {}, ValueType::U8, true, false, {}}};
    EXPECT_FALSE(function.validSignature());
}

TEST(IOMalformed, FunctionTlvRejectsNullPayloadWithNonzeroLength) {
    std::array<uint8_t, FUNCTION_TLV_HEADER_SIZE> bytes{};
    BufWriter writer(bytes.data(), bytes.size());
    EXPECT_FALSE(encodeFunctionTlv(writer, 0, ValueType::Binary, nullptr, 1));
    EXPECT_FALSE(writer.ok());
}

TEST(IOMalformed, FunctionTlvAcceptsZeroLengthPayload) {
    std::array<uint8_t, FUNCTION_TLV_HEADER_SIZE> bytes{};
    BufWriter writer(bytes.data(), bytes.size());
    ASSERT_TRUE(encodeFunctionTlv(writer, 7, ValueType::Binary, nullptr, 0));
    FunctionArgument argument;
    BufReader reader(bytes.data(), bytes.size());
    ASSERT_TRUE(decodeFunctionTlv(reader, argument));
    EXPECT_EQ(argument.position, 7u);
    EXPECT_TRUE(argument.value.empty());
}

TEST(IOMalformed, FunctionTlvRejectsUnknownTypeOnlyAtCallValidation) {
    std::array<uint8_t, FUNCTION_TLV_HEADER_SIZE> bytes{};
    BufWriter writer(bytes.data(), bytes.size());
    ASSERT_TRUE(encodeFunctionTlv(writer, 0, static_cast<ValueType>(0xFF), nullptr, 0));

    FunctionArgument argument;
    BufReader reader(bytes.data(), bytes.size());
    ASSERT_TRUE(decodeFunctionTlv(reader, argument));
    EXPECT_EQ(argument.type, static_cast<ValueType>(0xFF));
}

TEST(IOMalformed, FunctionTlvRejectsTruncatedHeader) {
    const std::array<uint8_t, FUNCTION_TLV_HEADER_SIZE - 1> bytes{};
    FunctionArgument argument;
    BufReader reader(bytes.data(), bytes.size());
    EXPECT_FALSE(decodeFunctionTlv(reader, argument));
}

TEST(IOMalformed, RecursiveValueDescriptorRoundTrips) {
    const auto descriptor = ValueDescriptor::array(ValueDescriptor::structure({
        {"id", ValueDescriptor::scalar(ValueType::U32)},
        {"values", ValueDescriptor::array(ValueDescriptor::scalar(ValueType::F32))}}));
    std::array<uint8_t, 256> bytes{};
    BufWriter writer(bytes.data(), bytes.size());
    ASSERT_TRUE(encodeValueDescriptor(writer, descriptor));

    ValueDescriptor decoded;
    BufReader reader(bytes.data(), writer.pos);
    ASSERT_TRUE(decodeValueDescriptor(reader, decoded));
    EXPECT_EQ(reader.remaining(), 0u);
    ASSERT_TRUE(decoded.element);
    EXPECT_EQ(decoded.element->type, ValueType::Struct);
    ASSERT_EQ(decoded.element->fields.size(), 2u);
    EXPECT_EQ(decoded.element->fields[1].second->type, ValueType::Array);
}

TEST(IOMalformed, AggregatePayloadRejectsDuplicateStructField) {
    const auto descriptor = ValueDescriptor::structure({
        {"x", ValueDescriptor::scalar(ValueType::U8)},
        {"y", ValueDescriptor::scalar(ValueType::U8)}});
    std::array<uint8_t, 64> bytes{};
    BufWriter writer(bytes.data(), bytes.size());
    writer.putU32(2);
    writer.putU32(0); writer.putU8(static_cast<uint8_t>(ValueType::U8)); writer.putU32(1); writer.putU8(1);
    writer.putU32(0); writer.putU8(static_cast<uint8_t>(ValueType::U8)); writer.putU32(1); writer.putU8(2);
    ASSERT_TRUE(writer.ok());
    EXPECT_FALSE(validateValuePayload(descriptor, bytes.data(), writer.pos));
}
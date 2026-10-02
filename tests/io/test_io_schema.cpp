#include "tether/io/Schema.hpp"
#include "tether/io/SchemaDigest.hpp"

#include <gtest/gtest.h>

namespace tether::io {
namespace {

SchemaKey key(uint8_t value) {
    SchemaKey result{};
    result[0] = value;
    return result;
}

SchemaRef ref(uint8_t value) {
    return SchemaRef{key(value), {}};
}

TEST(IOSchema, CanonicalDescriptorIsDeterministic) {
    SchemaNode node;
    node.key = key(1);
    node.kind = SchemaKind::Struct;
    node.name = "Outer";
    node.annotations["unit"] = "mm";
    node.fields.push_back({1, SchemaFieldFlags::Required, ref(2), "inner", "Nested value"});

    const auto first = canonicalDescriptor(node);
    const auto second = canonicalDescriptor(node);
    EXPECT_EQ(first, second);
    EXPECT_FALSE(first.empty());
}

TEST(IOSchema, ValidatesNestedStructAndArrays) {
    SchemaNode scalar;
    scalar.key = key(1);
    scalar.kind = SchemaKind::Scalar;
    scalar.scalarType = ValueType::F64;

    SchemaNode inner;
    inner.key = key(2);
    inner.kind = SchemaKind::Struct;
    inner.structEncoding = StructEncoding::Packed;
    inner.fields.push_back({1, SchemaFieldFlags::Required, ref(1), "position", ""});

    SchemaNode array;
    array.key = key(3);
    array.kind = SchemaKind::DynamicArray;
    array.element = ref(2);
    array.minCount = 0;
    array.maxCount = 8;

    SchemaNode outer;
    outer.key = key(4);
    outer.kind = SchemaKind::Struct;
    outer.fields.push_back({1, SchemaFieldFlags::Required, ref(3), "axes", ""});

    const auto result = validateSchemaGraph(SchemaGraph{{scalar, inner, array, outer}});
    EXPECT_TRUE(result) << result.message;
}

TEST(IOSchema, RejectsDuplicateFieldKeysAndCycles) {
    SchemaNode badFields;
    badFields.key = key(1);
    badFields.kind = SchemaKind::Struct;
    badFields.fields = {
        {2, SchemaFieldFlags::Required, ref(2), "second", ""},
        {2, SchemaFieldFlags::Required, ref(2), "duplicate", ""},
    };
    SchemaNode target;
    target.key = key(2);
    target.kind = SchemaKind::Scalar;
    target.scalarType = ValueType::U32;

    auto result = validateSchemaGraph(SchemaGraph{{badFields, target}});
    EXPECT_FALSE(result);
    EXPECT_EQ(result.message, "struct fields must have increasing non-zero keys");

    SchemaNode first;
    first.key = key(3);
    first.kind = SchemaKind::Optional;
    first.element = ref(4);
    SchemaNode second;
    second.key = key(4);
    second.kind = SchemaKind::Optional;
    second.element = ref(3);

    result = validateSchemaGraph(SchemaGraph{{first, second}});
    EXPECT_FALSE(result);
    EXPECT_EQ(result.message, "schema graph contains a cycle");
}

TEST(IOSchema, RejectsInvalidArrayBoundsAndMissingReferences) {
    SchemaNode array;
    array.key = key(1);
    array.kind = SchemaKind::DynamicArray;
    array.element = ref(9);
    array.minCount = 4;
    array.maxCount = 2;

    auto result = validateSchemaGraph(SchemaGraph{{array}});
    EXPECT_FALSE(result);
    EXPECT_EQ(result.message, "invalid dynamic array bounds");
}

TEST(IOSchema, ComputesPackedFixedSizeAndRejectsVariableSchemas) {
    SchemaNode scalar;
    scalar.key = key(1);
    scalar.kind = SchemaKind::Scalar;
    scalar.scalarType = ValueType::U32;

    SchemaNode inner;
    inner.key = key(2);
    inner.kind = SchemaKind::Struct;
    inner.structEncoding = StructEncoding::Packed;
    inner.fields.push_back({1, SchemaFieldFlags::Required, ref(1), "value", ""});

    SchemaNode outer;
    outer.key = key(3);
    outer.kind = SchemaKind::Struct;
    outer.structEncoding = StructEncoding::Packed;
    outer.fields.push_back({1, SchemaFieldFlags::Required, ref(2), "inner", ""});
    outer.fields.push_back({2, SchemaFieldFlags::Required, ref(1), "tail", ""});

    const SchemaGraph graph{{scalar, inner, outer}};
    ASSERT_TRUE(validateSchemaGraph(graph));
    ASSERT_TRUE(fixedSchemaSize(graph, key(3)));
    EXPECT_EQ(*fixedSchemaSize(graph, key(3)), 8U);

    SchemaNode variable;
    variable.key = key(4);
    variable.kind = SchemaKind::String;
    variable.maxBytes = 32;
    const SchemaGraph variableGraph{{variable}};
    EXPECT_FALSE(fixedSchemaSize(variableGraph, key(4)));
}

TEST(IOSchema, ValidatesMapsOneOfAndFieldRestrictions) {
    SchemaNode string;
    string.key = key(1);
    string.kind = SchemaKind::String;

    SchemaNode scalar;
    scalar.key = key(2);
    scalar.kind = SchemaKind::Scalar;
    scalar.scalarType = ValueType::U32;

    SchemaNode map;
    map.key = key(3);
    map.kind = SchemaKind::Map;
    map.mapKey = ref(1);
    map.mapValue = ref(2);
    map.minCount = 1;
    map.maxCount = 8;

    SchemaNode oneOf;
    oneOf.key = key(4);
    oneOf.kind = SchemaKind::OneOf;
    oneOf.oneOfMembers = {{10, ref(1)}, {20, ref(2)}};

    SchemaNode record;
    record.key = key(5);
    record.kind = SchemaKind::Struct;
    SchemaField mapField{1, 0, ref(3), "values", ""};
    mapField.presence = FieldPresence::Optional;
    mapField.restrictions = {{RestrictionKind::LengthRange, {1, 8}}};
    SchemaField choiceField{2, 0, ref(4), "choice", ""};
    choiceField.restrictions = {{RestrictionKind::AllowedMembers, {10, 20}}};
    record.fields = {mapField, choiceField};

    const auto result = validateSchemaGraph(SchemaGraph{{string, scalar, map, oneOf, record}});
    EXPECT_TRUE(result) << result.message;

    map.mapKey = ref(4);
    EXPECT_FALSE(validateSchemaGraph(SchemaGraph{{string, scalar, map, oneOf, record}}));
}

TEST(IOSchema, ComputesStableBlake3DigestFromCanonicalDescriptor) {
    SchemaNode node;
    node.key = key(7);
    node.kind = SchemaKind::String;
    node.name = "machine.label";

    const auto first = computeSchemaDigest(node);
    const auto second = computeSchemaDigest(node);
    EXPECT_EQ(first, second);
    EXPECT_NE(first, SchemaDigest{});

    node.name = "machine.other_label";
    EXPECT_NE(computeSchemaDigest(node), first);
}

// ---------------------------------------------------------------------------
// Cross-language digest vectors
//
// These graphs mirror web/tether-io-dashboard/test-fixtures/
// schema-digest-vectors.json.  The TypeScript client reconstructs the same
// nodes and asserts identical BLAKE3 digests, so a drift between the two
// canonical-descriptor encodings fails on both sides.
// ---------------------------------------------------------------------------

SchemaKey lastByteKey(uint8_t value) {
    SchemaKey result{};
    result[15] = value;
    return result;
}

std::string hexDigest(const SchemaNode& node) {
    const auto digest = computeSchemaDigest(node);
    static constexpr char digits[] = "0123456789abcdef";
    std::string out;
    out.reserve(digest.size() * 2);
    for (uint8_t byte : digest) {
        out += digits[byte >> 4];
        out += digits[byte & 0x0f];
    }
    return out;
}

SchemaNode scalarNode(uint8_t keyByte, ValueType type, std::string name = {},
                    uint32_t revision = 1) {
    SchemaNode node;
    node.key = lastByteKey(keyByte);
    node.revision = revision;
    node.kind = SchemaKind::Scalar;
    node.name = std::move(name);
    node.scalarType = type;
    node.structEncoding = StructEncoding::Packed;
    return node;
}

TEST(IOSchema, DigestVectorScalarU32) {
    const auto node = scalarNode(0x01, ValueType::U32, "Counter");
    EXPECT_EQ(hexDigest(node),
              "30a0601a31662c4c2475fd6f379dd015e242fb1b4d1801c164928ed05015c01a");
}

TEST(IOSchema, DigestVectorAnnotatedUtf8String) {
    SchemaNode node;
    node.key = lastByteKey(0x02);
    node.revision = 3;
    node.kind = SchemaKind::String;
    node.name = "Label \xce\xb1"; // "Label α"
    node.description = "UTF-8 annotation ordering";
    node.annotations = {{"Z-last", "1"}, {"a-first", "2"}, {"unit", "mm"}};
    node.maxBytes = 64;
    node.structEncoding = StructEncoding::Packed;
    EXPECT_EQ(hexDigest(node),
              "8552872e497106262afbf1b6ce4fc4eeed8dfdb067238c7ab07ce53f82c74347");
}

TEST(IOSchema, DigestVectorBoundedDynArray) {
    auto i16 = scalarNode(0x10, ValueType::I16);

    SchemaNode samples;
    samples.key = lastByteKey(0x11);
    samples.kind = SchemaKind::DynamicArray;
    samples.name = "Samples";
    samples.minCount = 1;
    samples.maxCount = 64;
    samples.structEncoding = StructEncoding::Packed;
    samples.element = i16.ref();

    SchemaGraph graph{{i16, samples}};
    ASSERT_TRUE(resolveSchemaDigests(graph));
    EXPECT_EQ(hexDigest(graph.nodes[1]),
              "c7d65db615f35021f3a808e697e636a0e527ec369f0360eeb03df69489017e12");
}

TEST(IOSchema, DigestVectorMapStringToU32) {
    SchemaNode keyStr;
    keyStr.key = lastByteKey(0x20);
    keyStr.kind = SchemaKind::String;
    keyStr.maxBytes = 32;
    keyStr.structEncoding = StructEncoding::Packed;
    auto valU32 = scalarNode(0x21, ValueType::U32);

    SchemaNode lookup;
    lookup.key = lastByteKey(0x22);
    lookup.kind = SchemaKind::Map;
    lookup.name = "Lookup";
    lookup.maxCount = 128;
    lookup.structEncoding = StructEncoding::Packed;
    lookup.mapKey = keyStr.ref();
    lookup.mapValue = valU32.ref();

    SchemaGraph graph{{keyStr, valU32, lookup}};
    ASSERT_TRUE(resolveSchemaDigests(graph));
    EXPECT_EQ(hexDigest(graph.nodes[2]),
              "95696de9b778b1a4a4afefeb501f8d8c7b6fbbf9f15143a78c349ee31197ef08");
}

TEST(IOSchema, DigestVectorTaggedStructWithRestrictions) {
    auto u32 = scalarNode(0x30, ValueType::U32);
    SchemaNode nameNode;
    nameNode.key = lastByteKey(0x31);
    nameNode.kind = SchemaKind::String;
    nameNode.maxBytes = 48;
    nameNode.structEncoding = StructEncoding::Packed;

    SchemaNode config;
    config.key = lastByteKey(0x32);
    config.revision = 2;
    config.kind = SchemaKind::Struct;
    config.name = "Config";
    config.annotations = {{"machine.profile", "test"}};
    config.structEncoding = StructEncoding::Tagged;

    SchemaField rate{1, 0, u32.ref(), "rate", ""};
    rate.restrictions = {{RestrictionKind::NumericRange,
                          {0x01, 0x00, 0x00, 0x00, 0x00, 0x01, 0x10, 0x27, 0x00, 0x00}}};
    SchemaField label{2, 0, nameNode.ref(), "label", ""};
    label.presence = FieldPresence::Optional;
    label.defaultValue = {0x6e, 0x61, 0x6d, 0x65, 0x00}; // "name\0"
    config.fields = {rate, label};

    SchemaGraph graph{{u32, nameNode, config}};
    ASSERT_TRUE(resolveSchemaDigests(graph));
    EXPECT_EQ(hexDigest(graph.nodes[2]),
              "9cf8cb945fa4b034896e3601645f827dd521e1aad4e4334de6659d8dea7a5037");
}

TEST(IOSchema, DigestVectorOneOfAndAlias) {
    auto u32 = scalarNode(0x40, ValueType::U32);
    auto f64 = scalarNode(0x41, ValueType::F64);

    SchemaNode alias;
    alias.key = lastByteKey(0x42);
    alias.kind = SchemaKind::Alias;
    alias.name = "AliasU32";
    alias.structEncoding = StructEncoding::Packed;
    alias.target = u32.ref();

    SchemaNode root;
    root.key = lastByteKey(0x43);
    root.kind = SchemaKind::OneOf;
    root.name = "Choice";
    root.structEncoding = StructEncoding::Packed;
    root.oneOfMembers = {{1, alias.ref()}, {5, f64.ref()}};

    SchemaGraph graph{{u32, f64, alias, root}};
    ASSERT_TRUE(resolveSchemaDigests(graph));
    EXPECT_EQ(hexDigest(graph.nodes[3]),
              "fce3b894dfec724b080d742b12d07277d58a8689aae56b11a8371d44e7e0643c");
}

TEST(IOSchema, DigestVectorPackedStructOptionalAndArray) {
    auto u8 = scalarNode(0x50, ValueType::U8);

    SchemaNode vec4;
    vec4.key = lastByteKey(0x51);
    vec4.kind = SchemaKind::FixedArray;
    vec4.fixedCount = 4;
    vec4.structEncoding = StructEncoding::Packed;
    vec4.element = u8.ref();

    SchemaNode maybeVec;
    maybeVec.key = lastByteKey(0x52);
    maybeVec.kind = SchemaKind::Optional;
    maybeVec.structEncoding = StructEncoding::Packed;
    maybeVec.element = vec4.ref();

    SchemaNode snapshot;
    snapshot.key = lastByteKey(0x53);
    snapshot.revision = 7;
    snapshot.kind = SchemaKind::Struct;
    snapshot.name = "Snapshot";
    snapshot.structEncoding = StructEncoding::Packed;

    SchemaField vec{1, 0, maybeVec.ref(), "vec", ""};
    SchemaField state{2, 0, u8.ref(), "state", ""};
    state.restrictions = {{RestrictionKind::MultipleOf, {0x04}},
                          {RestrictionKind::AllowedValues, {0x02, 0x01, 0x01, 0x01, 0x02}}};
    snapshot.fields = {vec, state};

    SchemaGraph graph{{u8, vec4, maybeVec, snapshot}};
    ASSERT_TRUE(resolveSchemaDigests(graph));
    EXPECT_EQ(hexDigest(graph.nodes[3]),
              "69964085564112dc1a8f28c1476e914df3492a5f57b354c6fe1c883424142610");
}

} // namespace
} // namespace tether::io

#include "tether/io/Schema.hpp"

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

} // namespace
} // namespace tether::io

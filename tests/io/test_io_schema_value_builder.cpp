// SchemaValueBuilder: NativeValue → SchemaValue against every schema
// kind, encode/decode round-trips, asX accessors, and contract failures.
#include <gtest/gtest.h>

#include <cstring>
#include <span>
#include <string>
#include <vector>

#include "tether/io/SchemaValueBuilder.hpp"

using namespace tether::io;

namespace {

SchemaKey keyOf(uint8_t v) { SchemaKey k{}; k[0] = v; return k; }
SchemaRef refOf(uint8_t v) { return SchemaRef{keyOf(v), {}}; }

//   1 f64 | 2 string | 3 bytes | 4 i32 | 5 u16 | 7 u32
//   6 tagged struct {1:u32 speed, 2:string mode, 3:f64 gain, 4:opt u16 extra}
//   8 packed struct {1:f64 x, 2:f64 y}
//   9 dyn array u16 | 10 fixed f64[4] | 11 map string→i32
//   12 optional f64 | 13 oneof {1:f64, 2:string} | 14 alias→f64
//  15 dyn array string | 16 i8
class BuilderTest : public ::testing::Test {
protected:
    SchemaGraph g_;
    void SetUp() override {
        auto scalar = [&](uint8_t k, ValueType t, const char* name = "") {
            SchemaNode n;
            n.key = keyOf(k);
            n.kind = SchemaKind::Scalar;
            n.scalarType = t;
            n.name = name;
            return n;
        };
        SchemaNode str;
        str.key = keyOf(2); str.kind = SchemaKind::String;
        SchemaNode bytes;
        bytes.key = keyOf(3); bytes.kind = SchemaKind::Bytes;

        SchemaNode tagged;
        tagged.key = keyOf(6); tagged.kind = SchemaKind::Struct;
        tagged.fields = {{1, 0, refOf(7), "speed", ""},
                         {2, 0, refOf(2), "mode", ""},
                         {3, 0, refOf(1), "gain", ""}};
        SchemaField extra{4, 0, refOf(5), "extra", ""};
        extra.presence = FieldPresence::Optional;
        tagged.fields.push_back(std::move(extra));

        SchemaNode packed;
        packed.key = keyOf(8); packed.kind = SchemaKind::Struct;
        packed.structEncoding = StructEncoding::Packed;
        packed.fields = {{1, 0, refOf(1), "x", ""}, {2, 0, refOf(1), "y", ""}};

        SchemaNode dyn;
        dyn.key = keyOf(9); dyn.kind = SchemaKind::DynamicArray;
        dyn.element = refOf(5);
        SchemaNode fixed;
        fixed.key = keyOf(10); fixed.kind = SchemaKind::FixedArray;
        fixed.element = refOf(1); fixed.fixedCount = 4;
        SchemaNode map;
        map.key = keyOf(11); map.kind = SchemaKind::Map;
        map.mapKey = refOf(2); map.mapValue = refOf(4);
        SchemaNode opt;
        opt.key = keyOf(12); opt.kind = SchemaKind::Optional;
        opt.element = refOf(1);
        SchemaNode one;
        one.key = keyOf(13); one.kind = SchemaKind::OneOf;
        one.oneOfMembers = {{1, refOf(1)}, {2, refOf(2)}};
        SchemaNode alias;
        alias.key = keyOf(14); alias.kind = SchemaKind::Alias;
        alias.target = refOf(1);
        SchemaNode strarr;
        strarr.key = keyOf(15); strarr.kind = SchemaKind::DynamicArray;
        strarr.element = refOf(2);

        g_.nodes = {scalar(1, ValueType::F64), str, bytes,
                    scalar(4, ValueType::I32), scalar(5, ValueType::U16),
                    tagged, scalar(7, ValueType::U32), packed, dyn, fixed,
                    map, opt, one, alias, strarr, scalar(16, ValueType::I8)};
    }
    const SchemaNode* node(uint8_t k) const { return g_.find(keyOf(k)); }
};

TEST_F(BuilderTest, ScalarRoundTrips) {
    auto v = buildSchemaValue(g_, *node(1), nv::f64(2.5));
    ASSERT_TRUE(v);
    const auto* s = std::get_if<SchemaValue::Scalar>(&v->payload);
    ASSERT_TRUE(s);
    double d;
    std::memcpy(&d, &s->bits, 8);
    EXPECT_DOUBLE_EQ(d, 2.5);
    // Signed + unsigned paths.
    EXPECT_TRUE(buildSchemaValue(g_, *node(4), nv::i32(-7)));
    EXPECT_TRUE(buildSchemaValue(g_, *node(5), nv::u16(65000)));
}

TEST_F(BuilderTest, ScalarRangeChecks) {
    EXPECT_FALSE(buildSchemaValue(g_, *node(5), nv::u(1u << 20)));   // U16 ovf
    EXPECT_FALSE(buildSchemaValue(g_, *node(4), nv::u64(5)));        // signedness
    EXPECT_FALSE(buildSchemaValue(g_, *node(4), nv::i64(int64_t(1) << 40)));
    EXPECT_FALSE(buildSchemaValue(g_, *node(16), nv::i(-200)));      // I8 ovf
    EXPECT_FALSE(buildSchemaValue(g_, *node(1), nv::u64(5)));        // float≠int
}

TEST_F(BuilderTest, StringAndBytes) {
    EXPECT_TRUE(buildSchemaValue(g_, *node(2), nv::str("hello")));
    // A real embedded NUL needs an explicit-length view (a literal
    // "a\0b" truncates to "a" at the boundary).
    EXPECT_FALSE(buildSchemaValue(
        g_, *node(2), nv::str(std::string_view("a\0b", 3))));
    const uint8_t blob[] = {1, 2, 3};
    auto b = buildSchemaValue(g_, *node(3), nv::bytes(blob));
    ASSERT_TRUE(b);
    EXPECT_EQ(std::get<SchemaValue::Bytes>(b->payload).value.size(), 3u);
}

TEST_F(BuilderTest, PackedStructRoundTrip) {
    auto bytes = encodeNative(g_, *node(8), nv::fields(
        nv::field("x", nv::f64(1.5)),
        nv::field("y", nv::f64(-2.5))));
    ASSERT_TRUE(bytes);
    auto back = decodeSchemaValue(g_, *node(8), bytes->data(), bytes->size());
    ASSERT_TRUE(back);
    const SchemaValue* x = fieldValue(*back, "x");
    const SchemaValue* y = fieldValue(*back, "y");
    ASSERT_TRUE(x && y);
    EXPECT_DOUBLE_EQ(*asF64(*x), 1.5);
    EXPECT_DOUBLE_EQ(*asF64(*y), -2.5);
}

TEST_F(BuilderTest, PackedStructRequiresAllFields) {
    EXPECT_FALSE(buildSchemaValue(g_, *node(8), nv::fields(nv::field("x", nv::f64(1)))));
    EXPECT_FALSE(buildSchemaValue(
        g_, *node(8),
        nv::fields(nv::field("x", nv::f64(1)), nv::field("nope", nv::f64(2)))));
}

TEST_F(BuilderTest, TaggedStructRoundTripWithOptional) {
    auto bytes = encodeNative(g_, *node(6), nv::fields(
        nv::field("speed", nv::u32(42)),
        nv::field("mode", nv::str("fast")),
        nv::field("gain", nv::f64(0.75)),
        nv::field("extra", nv::u16(9))));
    ASSERT_TRUE(bytes);
    auto back = decodeSchemaValue(g_, *node(6), bytes->data(), bytes->size());
    ASSERT_TRUE(back);
    EXPECT_EQ(*asU64(*fieldValue(*back, "speed")), 42u);
    EXPECT_EQ(*asString(*fieldValue(*back, "mode")), "fast");
    EXPECT_DOUBLE_EQ(*asF64(*fieldValue(*back, "gain")), 0.75);
}

TEST_F(BuilderTest, FixedArrayCountChecked) {
    const double good[] = {1, 2, 3, 4};
    EXPECT_TRUE(buildSchemaValue(g_, *node(10), nv::f64v(good)));
    const double shortArr[] = {1, 2, 3};
    EXPECT_FALSE(buildSchemaValue(g_, *node(10), nv::f64v(shortArr)));
}

TEST_F(BuilderTest, DynamicArrayAndAsF64Array) {
    const uint64_t vals[] = {10, 20, 30};
    auto bytes = encodeNative(g_, *node(9), nv::u64v(vals));
    ASSERT_TRUE(bytes);
    auto back = decodeSchemaValue(g_, *node(9), bytes->data(), bytes->size());
    ASSERT_TRUE(back);
    const auto* arr = std::get_if<SchemaValue::Array>(&back->payload);
    ASSERT_TRUE(arr);
    EXPECT_EQ(arr->elements.size(), 3u);
    // And the f64 fast path through asF64Array:
    auto f = buildSchemaValue(g_, *node(15), nv::strv(
        std::vector<std::string>{"a", "b"}));
    ASSERT_TRUE(f);
    auto strings = asStringArray(*f);
    ASSERT_TRUE(strings);
    EXPECT_EQ(strings->size(), 2u);
    EXPECT_EQ((*strings)[1], "b");
}

TEST_F(BuilderTest, OptionalAndOneOf) {
    auto none = buildSchemaValue(g_, *node(12), nv::none());
    ASSERT_TRUE(none);
    EXPECT_FALSE(std::get<SchemaValue::Optional>(none->payload).present);
    auto some = buildSchemaValue(g_, *node(12), nv::f64(3.0));
    ASSERT_TRUE(some);
    const auto& o = std::get<SchemaValue::Optional>(some->payload);
    ASSERT_TRUE(o.present && o.value);
    EXPECT_DOUBLE_EQ(*asF64(*o.value), 3.0);

    auto choice = encodeNative(g_, *node(13), nv::oneOf(2, nv::str("yes")));
    ASSERT_TRUE(choice);
    auto back = decodeSchemaValue(g_, *node(13), choice->data(), choice->size());
    ASSERT_TRUE(back);
    const auto& oo = std::get<SchemaValue::OneOf>(back->payload);
    EXPECT_EQ(oo.memberKey, 2u);
    EXPECT_EQ(*asString(*oo.value), "yes");
    EXPECT_FALSE(buildSchemaValue(g_, *node(13), nv::oneOf(9, nv::f64(1))));
}

TEST_F(BuilderTest, MapAndNestedItems) {
    auto m = buildSchemaValue(g_, *node(11), nv::map(
        nv::mapEntry(nv::str("k1"), nv::i32(3)),
        nv::mapEntry(nv::str("k2"), nv::i32(-4))));
    ASSERT_TRUE(m);
    const auto& mm = std::get<SchemaValue::Map>(m->payload);
    EXPECT_EQ(mm.entries.size(), 2u);
}

TEST_F(BuilderTest, AliasChasesToTarget) {
    auto v = buildSchemaValue(g_, *node(14), nv::f64(7.0));
    ASSERT_TRUE(v);
    EXPECT_DOUBLE_EQ(*asF64(*v), 7.0);
}

TEST_F(BuilderTest, AccessorsRejectWrongKinds) {
    auto v = buildSchemaValue(g_, *node(2), nv::str("x"));
    ASSERT_TRUE(v);
    EXPECT_FALSE(asF64(*v));
    EXPECT_FALSE(asU64(*v));
    EXPECT_FALSE(asF64Array(*v));
    EXPECT_EQ(fieldValue(*v, "anything"), nullptr);
}

}  // namespace

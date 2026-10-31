// SchemaLayout: fixed-offset computation, static-row encode/decode, the
// packed head/tail ordering rule, FixedString and fixed-slot Optional.
#include <gtest/gtest.h>

#include "tether/io/SchemaLayout.hpp"
#include "tether/io/SchemaValue.hpp"
#include "tether/io/SchemaValueBuilder.hpp"

using namespace tether::io;

namespace {

SchemaKey keyOf(uint8_t k) {
    SchemaKey k2{};
    k2[15] = k;
    return k2;
}
SchemaRef refOf(uint8_t k) { return SchemaRef{keyOf(k), {}}; }

// Graph:
//   1 f64 | 2 u64 | 3 u8 | 4 uvarint | 5 fixedstring[8]
//   6 fixed f64[4] | 7 optional u64 | 8 dynarray u16(9)
//  10 packed Pose {1:f64 x, 2:f64 y, 3:f64[4] q}
//  11 packed HeadTail {1:u64 seq, 2:fixedstring[8] name, 3:opt<u64> extra,
//                      4:uvarint serial, 5:dynarray<u16> log}
//  12 packed Bad {1:dynarray<u16> a, 2:f64 b}   (violates ordering)
//  13 packed Varint {1:uvarint a}
//  14 dynarray Pose
//  15 alias → Pose
class LayoutTest : public ::testing::Test {
protected:
    SchemaGraph g_;
    void SetUp() override {
        auto scalar = [&](uint8_t k, ValueType t) {
            SchemaNode n;
            n.key = keyOf(k);
            n.kind = SchemaKind::Scalar;
            n.scalarType = t;
            return n;
        };
        SchemaNode fstr = scalar(5, ValueType::FixedString);
        fstr.fixedCount = 8;

        SchemaNode farr;
        farr.key = keyOf(6); farr.kind = SchemaKind::FixedArray;
        farr.element = refOf(1); farr.fixedCount = 4;
        SchemaNode opt;
        opt.key = keyOf(7); opt.kind = SchemaKind::Optional;
        opt.element = refOf(2);
        SchemaNode dyn;
        dyn.key = keyOf(8); dyn.kind = SchemaKind::DynamicArray;
        dyn.element = refOf(9);

        SchemaNode pose;
        pose.key = keyOf(10); pose.kind = SchemaKind::Struct;
        pose.structEncoding = StructEncoding::Packed;
        pose.fields = {{1, 0, refOf(1), "x", ""},
                       {2, 0, refOf(1), "y", ""},
                       {3, 0, refOf(6), "q", ""}};

        SchemaNode headtail;
        headtail.key = keyOf(11); headtail.kind = SchemaKind::Struct;
        headtail.structEncoding = StructEncoding::Packed;
        headtail.fields = {{1, 0, refOf(2), "seq", ""},
                           {2, 0, refOf(5), "name", ""},
                           {3, 0, refOf(7), "extra", ""},
                           {4, 0, refOf(4), "serial", ""},
                           {5, 0, refOf(8), "log", ""}};

        SchemaNode varint;
        varint.key = keyOf(13); varint.kind = SchemaKind::Struct;
        varint.structEncoding = StructEncoding::Packed;
        varint.fields = {{1, 0, refOf(4), "a", ""}};

        SchemaNode poseArr;
        poseArr.key = keyOf(14); poseArr.kind = SchemaKind::DynamicArray;
        poseArr.element = refOf(10);

        SchemaNode alias;
        alias.key = keyOf(15); alias.kind = SchemaKind::Alias;
        alias.target = refOf(10);

        g_.nodes = {scalar(1, ValueType::F64), scalar(2, ValueType::U64),
                    scalar(3, ValueType::U8),   scalar(4, ValueType::UVarint),
                    fstr,   farr, opt,  dyn,
                    scalar(9, ValueType::U16),
                    pose, headtail, varint, poseArr, alias};
    }
    const SchemaNode* node(uint8_t k) const { return g_.find(keyOf(k)); }
};

TEST_F(LayoutTest, FullyStaticOffsets) {
    auto l = computeSchemaLayout(g_, *node(10));
    ASSERT_TRUE(l);
    EXPECT_TRUE(l->fixedLayout);
    EXPECT_TRUE(l->memcpyImage);
    EXPECT_EQ(l->wireSize, 8 + 8 + 32);
    ASSERT_EQ(l->head.size(), 3u);
    EXPECT_EQ(l->head[0].offset, 0u);
    EXPECT_EQ(l->head[1].offset, 8u);
    EXPECT_EQ(l->head[2].offset, 16u);
    EXPECT_EQ(l->head[2].size, 32u);
    EXPECT_TRUE(l->tail.empty());
    EXPECT_EQ(*l->fieldOffset(2), 8u);
    EXPECT_FALSE(l->fieldOffset(99));
}

TEST_F(LayoutTest, StaticRowEncodeMatchesTreeCodec) {
    auto l = computeSchemaLayout(g_, *node(10));
    ASSERT_TRUE(l);
    const double x = 1.5, y = -2.25, q[4] = {0, 1, 2, 3};
    const RowField fields[] = {{&x, 8}, {&y, 8}, {q, sizeof(q)}};
    std::vector<uint8_t> row(l->wireSize);
    auto n = encodeStaticRow(*l, fields, row);
    ASSERT_TRUE(n);
    EXPECT_EQ(*n, 48u);

    // Same bytes as the tree codec — offsets come from the schema.
    const double qv[4] = {0, 1, 2, 3};
    auto enc = encodeNative(g_, *node(10),
                            nv::fields(nv::field("x", nv::f64(x)),
                                       nv::field("y", nv::f64(y)),
                                       nv::field("q", nv::f64v(qv))));
    ASSERT_TRUE(enc);
    EXPECT_EQ(row, *enc);

    // And the static row decodes with the generic codec.
    auto v = decodeSchemaValue(g_, *node(10), row.data(), row.size());
    ASSERT_TRUE(v);
}

TEST_F(LayoutTest, StaticRowSizeChecks) {
    auto l = computeSchemaLayout(g_, *node(10));
    ASSERT_TRUE(l);
    const double x = 0, q[4] = {};
    std::vector<uint8_t> row(l->wireSize);
    const RowField one[] = {{&x, 8}};
    EXPECT_FALSE(encodeStaticRow(*l, one, row));            // count
    const RowField badSize[] = {{&x, 4}, {&x, 8}, {q, 32}};
    EXPECT_FALSE(encodeStaticRow(*l, badSize, row));
    const RowField three[] = {{&x, 8}, {&x, 8}, {q, 32}};
    std::vector<uint8_t> small(8);
    EXPECT_FALSE(encodeStaticRow(*l, three, small));
}

TEST_F(LayoutTest, StaticFieldBytesReadback) {
    auto l = computeSchemaLayout(g_, *node(10));
    ASSERT_TRUE(l);
    const double x = 7.0, y = 8.0, q[4] = {9, 9, 9, 9};
    const RowField fields[] = {{&x, 8}, {&y, 8}, {q, sizeof(q)}};
    std::vector<uint8_t> row(l->wireSize);
    ASSERT_TRUE(encodeStaticRow(*l, fields, row));
    auto slice = staticFieldBytes(*l, 2, row);
    ASSERT_TRUE(slice);
    double out;
    std::memcpy(&out, slice->data(), 8);
    EXPECT_DOUBLE_EQ(out, 8.0);
    EXPECT_FALSE(staticFieldBytes(*l, 42, row));
}

TEST_F(LayoutTest, FixedStringLayoutAndCodec) {
    auto l = computeSchemaLayout(g_, *node(5));
    ASSERT_TRUE(l);
    EXPECT_TRUE(l->fixedLayout);
    EXPECT_EQ(l->wireSize, 8u);

    auto v = buildSchemaValue(g_, *node(5), nv::str("pose"));
    ASSERT_TRUE(v);
    auto enc = encodeSchemaValue(g_, *v);
    ASSERT_TRUE(enc);
    ASSERT_EQ(enc->size(), 8u);
    EXPECT_EQ(std::memcmp(enc->data(), "pose\0\0\0\0", 8), 0);

    auto dv = decodeSchemaValue(g_, *node(5), enc->data(), enc->size());
    ASSERT_TRUE(dv);
    auto s = asString(*dv);
    ASSERT_TRUE(s);
    EXPECT_EQ(*s, "pose");

    // Full capacity (no terminator needed) still works.
    EXPECT_TRUE(buildSchemaValue(g_, *node(5), nv::str("12345678")));
    // Overflow + embedded NUL rejected.
    EXPECT_FALSE(buildSchemaValue(g_, *node(5), nv::str("123456789")));
    EXPECT_FALSE(buildSchemaValue(
        g_, *node(5), nv::str(std::string_view("ab\0cd", 5))));
}

TEST_F(LayoutTest, FixedStringNeedsCapacity) {
    SchemaNode n;
    n.key = keyOf(20); n.kind = SchemaKind::Scalar;
    n.scalarType = ValueType::FixedString;   // fixedCount = 0 → invalid
    g_.nodes.push_back(n);
    EXPECT_FALSE(validateSchemaGraph(g_));
}

TEST_F(LayoutTest, OptionalStaticSlotWire) {
    // Optional<U64>: [flag][8-byte slot] — 9 bytes either way.
    auto l = computeSchemaLayout(g_, *node(7));
    ASSERT_TRUE(l);
    EXPECT_TRUE(l->fixedLayout);
    EXPECT_EQ(l->wireSize, 9u);

    auto absent = encodeNative(g_, *node(7), nv::none());
    ASSERT_TRUE(absent);
    ASSERT_EQ(absent->size(), 9u);
    EXPECT_EQ((*absent)[0], 0);
    for (size_t i = 1; i < 9; ++i) EXPECT_EQ((*absent)[i], 0);

    auto present = encodeNative(g_, *node(7), nv::u64(0xAB));
    ASSERT_TRUE(present);
    ASSERT_EQ(present->size(), 9u);
    EXPECT_EQ((*present)[0], 1);
    uint64_t got;
    std::memcpy(&got, present->data() + 1, 8);
    EXPECT_EQ(got, 0xABu);

    // Decode skips the slot when absent.
    auto dv = decodeSchemaValue(g_, *node(7), absent->data(), absent->size());
    ASSERT_TRUE(dv);
    const auto* o = std::get_if<SchemaValue::Optional>(&dv->payload);
    ASSERT_TRUE(o);
    EXPECT_FALSE(o->present);
}

TEST_F(LayoutTest, HeadTailMixedStruct) {
    auto l = computeSchemaLayout(g_, *node(11));
    ASSERT_TRUE(l);
    EXPECT_FALSE(l->fixedLayout);           // has a dynamic tail
    EXPECT_FALSE(l->memcpyImage);
    ASSERT_EQ(l->head.size(), 3u);          // seq, name, extra
    EXPECT_EQ(l->head[0].offset, 0u);
    EXPECT_EQ(l->head[0].size, 8u);         // seq u64
    EXPECT_EQ(l->head[1].offset, 8u);
    EXPECT_EQ(l->head[1].size, 8u);         // name fixedstring[8]
    EXPECT_EQ(l->head[2].offset, 16u);
    EXPECT_EQ(l->head[2].size, 9u);         // extra opt<u64>
    EXPECT_EQ(l->wireSize, 25u);            // head size
    ASSERT_EQ(l->tail.size(), 2u);          // serial (uvarint), log (dynarr)
    EXPECT_EQ(l->tail[0]->key, 4u);
    EXPECT_EQ(l->tail[1]->key, 5u);

    EXPECT_TRUE(validateSchemaGraph(g_));
}

TEST_F(LayoutTest, OrderingRuleRejectsDynamicThenFixed) {
    EXPECT_TRUE(validateSchemaGraph(g_));
    SchemaNode bad;
    bad.key = keyOf(12); bad.kind = SchemaKind::Struct;
    bad.structEncoding = StructEncoding::Packed;
    bad.fields = {{1, 0, refOf(8), "a", ""}, {2, 0, refOf(1), "b", ""}};
    g_.nodes.push_back(bad);
    EXPECT_FALSE(validateSchemaGraph(g_));
}

TEST_F(LayoutTest, VarintFieldIsDynamic) {
    auto l = computeSchemaLayout(g_, *node(13));
    ASSERT_TRUE(l);
    EXPECT_FALSE(l->fixedLayout);
    EXPECT_TRUE(l->head.empty());
    EXPECT_EQ(l->wireSize, 0u);
    ASSERT_EQ(l->tail.size(), 1u);
}

TEST_F(LayoutTest, AliasResolves) {
    auto l = computeSchemaLayout(g_, *node(15));
    ASSERT_TRUE(l);
    EXPECT_TRUE(l->fixedLayout);
    EXPECT_EQ(l->wireSize, 48u);
    EXPECT_EQ(l->node, node(10));
}

TEST_F(LayoutTest, DynArrayElementStride) {
    auto el = computeSchemaLayout(g_, *node(10));
    ASSERT_TRUE(el);
    const size_t stride = staticElementStride(el->wireSize);
    EXPECT_EQ(stride, 1 + 48);  // elemSize < 128 → 1-byte varint

    // Encode a 2-element array via the tree codec and stride-slice it.
    const double q4[4] = {0, 0, 0, 0};
    auto mkPose = [&](double x, double y) {
        return nv::fields(nv::field("x", nv::f64(x)),
                          nv::field("y", nv::f64(y)),
                          nv::field("q", nv::f64v(q4)));
    };
    auto enc = encodeNative(g_, *node(14),
                            nv::items(mkPose(1.0, 2.0), mkPose(3.0, 4.0)));
    ASSERT_TRUE(enc);
    // [count=2][len][elem0][len][elem1]
    ASSERT_EQ(enc->size(), 1 + 2 * stride);
    double x0, x1;
    std::memcpy(&x0, enc->data() + 1 + 1, 8);          // elem0 +0
    std::memcpy(&x1, enc->data() + 1 + stride + 1, 8); // elem1 +0
    EXPECT_DOUBLE_EQ(x0, 1.0);
    EXPECT_DOUBLE_EQ(x1, 3.0);
}

}  // namespace

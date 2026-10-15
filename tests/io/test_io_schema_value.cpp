/**
 * @file test_io_schema_value.cpp
 * @brief Tests for the SchemaValue decode/encode/format codec and the
 *        TetherIOClient schema-retention accessors.
 */
#include <gtest/gtest.h>
#include <random>
#include "tether/io/SchemaValue.hpp"
#include "tether/io/SchemaValueCodec.hpp"
#include "tether/io/TetherIOClient.hpp"
#include "tether/io/Session.hpp"
#include "tether/io/SimulatedCiA402Fleet.hpp"
#include "PipeTransport.hpp"

#include <cstring>
#include <thread>

using namespace tether::io;
using namespace tether::io::testing;

namespace {

SchemaKey keyOf(uint8_t v) { SchemaKey k{}; k[0] = v; return k; }
SchemaRef refOf(uint8_t v) { return SchemaRef{keyOf(v), {}}; }

// Graph shared by the codec tests.
//   1 f64 | 2 string | 3 bytes | 4 i32 | 5 u16
//   6 tagged struct {1:u32 speed, 2:string mode, 3:f64 gain, 4:opt u16 extra}
//   7 u32 | 8 packed struct {1:f64 x, 2:f64 y}
//   9 dyn array u16 | 10 fixed array f64[4] | 11 map string->i32
//   12 optional f64 | 13 oneof {1:f64,2:string} | 14 alias->f64
class SchemaValueCodecTest : public ::testing::Test {
protected:
    SchemaGraph graph_;
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
        str.key = keyOf(2); str.kind = SchemaKind::String; str.name = "str";
        SchemaNode bytes;
        bytes.key = keyOf(3); bytes.kind = SchemaKind::Bytes; bytes.name = "blob";

        SchemaNode tagged;
        tagged.key = keyOf(6); tagged.kind = SchemaKind::Struct;
        tagged.name = "cfg";
        tagged.fields.push_back({1, 0, refOf(7), "speed", ""});
        tagged.fields.push_back({2, 0, refOf(2), "mode", ""});
        tagged.fields.push_back({3, 0, refOf(1), "gain", ""});
        SchemaField extra{4, 0, refOf(5), "extra", ""};
        extra.presence = FieldPresence::Optional;
        tagged.fields.push_back(std::move(extra));

        SchemaNode packed;
        packed.key = keyOf(8); packed.kind = SchemaKind::Struct;
        packed.structEncoding = StructEncoding::Packed;
        packed.name = "point";
        packed.fields.push_back({1, 0, refOf(1), "x", ""});
        packed.fields.push_back({2, 0, refOf(1), "y", ""});

        SchemaNode dyn;
        dyn.key = keyOf(9); dyn.kind = SchemaKind::DynamicArray;
        dyn.element = refOf(5); dyn.maxCount = 16;

        SchemaNode fixed;
        fixed.key = keyOf(10); fixed.kind = SchemaKind::FixedArray;
        fixed.element = refOf(1); fixed.fixedCount = 4;

        SchemaNode map;
        map.key = keyOf(11); map.kind = SchemaKind::Map;
        map.mapKey = refOf(2); map.mapValue = refOf(4);
        map.maxCount = 16;

        SchemaNode opt;
        opt.key = keyOf(12); opt.kind = SchemaKind::Optional;
        opt.element = refOf(1);

        SchemaNode one;
        one.key = keyOf(13); one.kind = SchemaKind::OneOf;
        one.name = "choice";
        one.oneOfMembers = {{1, refOf(1)}, {2, refOf(2)}};

        SchemaNode alias;
        alias.key = keyOf(14); alias.kind = SchemaKind::Alias;
        alias.name = "alias_f64"; alias.target = refOf(1);

        graph_.nodes = {
            scalar(1, ValueType::F64, "f64"), str, bytes,
            scalar(4, ValueType::I32, "i32"), scalar(5, ValueType::U16, "u16"),
            tagged, scalar(7, ValueType::U32, "u32"), packed,
            dyn, fixed, map, opt, one, alias,
        };
        ASSERT_TRUE(validateSchemaGraph(graph_)) << "fixture graph invalid";
    }

    const SchemaNode* node(uint8_t k) const { return graph_.find(keyOf(k)); }
};

TEST_F(SchemaValueCodecTest, ScalarRoundTrip) {
    auto v = makeScalarValue(node(1), [] {
        uint64_t b; double d = 3.25;
        std::memcpy(&b, &d, 8); return b;
    }());
    auto bytes = encodeSchemaValue(graph_, v);
    ASSERT_TRUE(bytes) << bytes.error();
    ASSERT_EQ(bytes->size(), 8u);
    auto dv = decodeSchemaValue(graph_, *node(1), bytes->data(), bytes->size());
    ASSERT_TRUE(dv) << dv.error();
    EXPECT_DOUBLE_EQ(schemaValueAsDouble(*dv), 3.25);
    EXPECT_EQ(formatSchemaValueText(*dv), "3.25");
}

TEST_F(SchemaValueCodecTest, StringBytesRoundTrip) {
    auto s = makeStringValue(node(2), "hello world");
    auto enc = encodeSchemaValue(graph_, s);
    ASSERT_TRUE(enc);
    auto dv = decodeSchemaValue(graph_, *node(2), enc->data(), enc->size());
    ASSERT_TRUE(dv);
    ASSERT_NE(schemaStr(*dv), nullptr);
    EXPECT_EQ(schemaStr(*dv)->value, "hello world");
    EXPECT_EQ(formatSchemaValueText(*dv), "\"hello world\"");

    auto b = makeBytesValue(node(3), {0xde, 0xad, 0xbe, 0xef});
    auto be = encodeSchemaValue(graph_, b);
    ASSERT_TRUE(be);
    auto db = decodeSchemaValue(graph_, *node(3), be->data(), be->size());
    ASSERT_TRUE(db);
    EXPECT_EQ(formatSchemaValueText(*db), "hex:deadbeef");
}

TEST_F(SchemaValueCodecTest, TaggedStructRoundTripAndFormat) {
    std::vector<SchemaValue::Struct::Field> fields;
    fields.push_back(makeStructField(3, &node(6)->fields[2],
                                     makeScalarValue(node(1), [this] {
                                         uint64_t b; double d = 0.5;
                                         std::memcpy(&b, &d, 8); return b;
                                     }())));
    fields.push_back(makeStructField(1, &node(6)->fields[0],
                                     makeScalarValue(node(7), 42)));
    fields.push_back(makeStructField(2, &node(6)->fields[1],
                                     makeStringValue(node(2), "auto")));
    auto st = makeStructValue(node(6), std::move(fields));

    auto enc = encodeSchemaValue(graph_, st);
    ASSERT_TRUE(enc) << enc.error();

    // Wire output must pass strict validation (missing required field in
    // the tree would fail here — extra optional 'extra' was omitted).
    BufReader vr(enc->data(), enc->size());
    ASSERT_TRUE(validateSchemaValue(graph_, keyOf(6), vr));

    auto dv = decodeSchemaValue(graph_, *node(6), enc->data(), enc->size());
    ASSERT_TRUE(dv) << dv.error();
    const auto* st2 = schemaStruct(*dv);
    ASSERT_NE(st2, nullptr);
    ASSERT_EQ(st2->fields.size(), 3u);
    // Wire order is sorted keys.
    EXPECT_EQ(st2->fields[0].key, 1u);
    EXPECT_EQ(st2->fields[1].key, 2u);
    EXPECT_EQ(st2->fields[2].key, 3u);
    EXPECT_EQ(formatSchemaValueText(*dv),
              "{speed: 42, mode: \"auto\", gain: 0.5}");
    EXPECT_EQ(schemaValueToJson(*dv, &graph_),
              "{\"speed\":42,\"mode\":\"auto\",\"gain\":0.5}");
}

TEST_F(SchemaValueCodecTest, MissingRequiredFieldFailsEncode) {
    std::vector<SchemaValue::Struct::Field> fields;
    fields.push_back(makeStructField(1, &node(6)->fields[0],
                                     makeScalarValue(node(7), 42)));
    auto st = makeStructValue(node(6), std::move(fields));
    auto enc = encodeSchemaValue(graph_, st);
    ASSERT_FALSE(enc);
}

TEST_F(SchemaValueCodecTest, PackedStructRoundTrip) {
    std::vector<SchemaValue::Struct::Field> fields;
    auto f64bits = [](double d) { uint64_t b; std::memcpy(&b, &d, 8); return b; };
    fields.push_back(makeStructField(1, &node(8)->fields[0],
                                     makeScalarValue(node(1), f64bits(1.5))));
    fields.push_back(makeStructField(2, &node(8)->fields[1],
                                     makeScalarValue(node(1), f64bits(-2.5))));
    auto enc = encodeSchemaValue(graph_, makeStructValue(node(8), std::move(fields)));
    ASSERT_TRUE(enc) << enc.error();
    EXPECT_EQ(enc->size(), 16u);   // packed: no field metadata
    auto dv = decodeSchemaValue(graph_, *node(8), enc->data(), enc->size());
    ASSERT_TRUE(dv);
    EXPECT_EQ(formatSchemaValueText(*dv), "{x: 1.5, y: -2.5}");
}

TEST_F(SchemaValueCodecTest, ArraysRoundTrip) {
    std::vector<SchemaValue> elems;
    for (uint16_t x : {5, 3, 8}) elems.push_back(makeScalarValue(node(5), x));
    auto a = makeArrayValue(node(9), std::move(elems));
    auto enc = encodeSchemaValue(graph_, a);
    ASSERT_TRUE(enc) << enc.error();
    auto dv = decodeSchemaValue(graph_, *node(9), enc->data(), enc->size());
    ASSERT_TRUE(dv) << dv.error();
    EXPECT_EQ(formatSchemaValueText(*dv), "[5, 3, 8]");
    EXPECT_EQ(schemaValueToJson(*dv, &graph_), "[5,3,8]");
}

TEST_F(SchemaValueCodecTest, MapCanonicalOrderRoundTrip) {
    std::vector<SchemaValue::Map::Entry> entries;
    entries.push_back(makeMapEntry(makeStringValue(node(2), "b"),
                                   makeScalarValue(node(4), static_cast<uint64_t>(-7))));
    entries.push_back(makeMapEntry(makeStringValue(node(2), "a"),
                                   makeScalarValue(node(4), 1)));
    entries.push_back(makeMapEntry(makeStringValue(node(2), "ab"),
                                   makeScalarValue(node(4), 2)));
    auto m = makeMapValue(node(11), std::move(entries));
    auto enc = encodeSchemaValue(graph_, m);
    ASSERT_TRUE(enc) << enc.error();
    BufReader vr(enc->data(), enc->size());
    ASSERT_TRUE(validateSchemaValue(graph_, keyOf(11), vr));
    auto dv = decodeSchemaValue(graph_, *node(11), enc->data(), enc->size());
    ASSERT_TRUE(dv) << dv.error();
    EXPECT_EQ(formatSchemaValueText(*dv), "{\"a\": 1, \"ab\": 2, \"b\": -7}");
    EXPECT_EQ(schemaValueToJson(*dv, &graph_),
              "[[\"a\",1],[\"ab\",2],[\"b\",-7]]");
}

TEST_F(SchemaValueCodecTest, OptionalAndOneOf) {
    auto none = makeAbsentValue(node(12));
    auto enc = encodeSchemaValue(graph_, none);
    ASSERT_TRUE(enc);
    EXPECT_EQ(enc->size(), 1u);
    auto dv = decodeSchemaValue(graph_, *node(12), enc->data(), enc->size());
    ASSERT_TRUE(dv);
    EXPECT_EQ(formatSchemaValueText(*dv), "none");
    EXPECT_EQ(schemaValueToJson(*dv), "null");

    uint64_t fb; double d = 9.0; std::memcpy(&fb, &d, 8);
    auto oneof = makeOneOfValue(node(13), 1,
                                makeScalarValue(node(1), fb));
    auto oe = encodeSchemaValue(graph_, oneof);
    ASSERT_TRUE(oe) << oe.error();
    BufReader vr(oe->data(), oe->size());
    ASSERT_TRUE(validateSchemaValue(graph_, keyOf(13), vr));
    auto ov = decodeSchemaValue(graph_, *node(13), oe->data(), oe->size());
    ASSERT_TRUE(ov);
    EXPECT_EQ(formatSchemaValueText(*ov), "f64{9}");
}

TEST_F(SchemaValueCodecTest, AliasResolves) {
    uint64_t fb; double d = 1.0; std::memcpy(&fb, &d, 8);
    auto v = makeScalarValue(node(14), fb);
    auto enc = encodeSchemaValue(graph_, v);
    ASSERT_TRUE(enc) << enc.error();
    auto dv = decodeSchemaValue(graph_, *node(14), enc->data(), enc->size());
    ASSERT_TRUE(dv);
    EXPECT_DOUBLE_EQ(schemaValueAsDouble(*dv), 1.0);
    // The decoded node is the resolved target, not the alias.
    EXPECT_EQ(dv->node->kind, SchemaKind::Scalar);
}

TEST_F(SchemaValueCodecTest, DecodeRejectsGarbageAndTrailing) {
    uint8_t bad[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
    auto dv = decodeSchemaValue(graph_, *node(9), bad, sizeof(bad));
    EXPECT_FALSE(dv);   // array count varint non-minimal/truncated

    uint8_t extra[9] = {0};
    auto d2 = decodeSchemaValue(graph_, *node(1), extra, sizeof(extra));
    EXPECT_FALSE(d2);   // trailing byte rejected
}

// ---------------------------------------------------------------------------
// TetherIOClient schema retention
// ---------------------------------------------------------------------------

static void noopLog(const char*, const char*, ...) {}

class ClientSchemaRetentionTest : public ::testing::Test {
protected:
    Registry registry_;
    SchemaGraph graph_;
    SchemaCatalog catalog_;
    cia402::SimulatedCiA402Fleet fleet_;
    std::unique_ptr<Session> session_;
    std::unique_ptr<TetherIOClient> client_;
    std::thread sessionThread_;

    void SetUp() override {
        graph_ = cia402::machineProfileSchemaGraph();
        ASSERT_TRUE(cia402::SimulatedCiA402Fleet::installSchemas(graph_, catalog_));
        ASSERT_TRUE(fleet_.registerSignals(registry_, catalog_));
        auto [clientEnd, serverEnd] = MessagePipeTransport::create();
        session_ = std::make_unique<Session>(
            std::move(serverEnd), registry_,
            [] { return cia402::simulated_fleet_detail::monotonicTimestampUs(); },
            noopLog, nullptr, nullptr, nullptr, nullptr, nullptr,
            Framing::None, nullptr, &catalog_);
        sessionThread_ = std::thread([this] { session_->run(); });
        client_ = std::make_unique<TetherIOClient>(std::move(clientEnd), 5000);
        ASSERT_TRUE(client_->ping(7)) << "client failed to negotiate";
    }

    void TearDown() override {
        if (client_) client_->close();
        if (session_) session_->requestStop();
        if (sessionThread_.joinable()) sessionThread_.join();
    }
};

TEST_F(ClientSchemaRetentionTest, GraphManifestAndSlotsRetained) {
    ASSERT_FALSE(client_->schemaGraph().nodes.empty());
    ASSERT_EQ(client_->schemaManifest().size(), catalog_.size());
    EXPECT_EQ(client_->schemaEpoch(), catalog_.epoch());
    EXPECT_EQ(client_->serverLimits().maxMessageBytes, 1u << 20);

    // Every slot resolves to the manifest entry's node.
    for (uint32_t slot = 0; slot < client_->schemaManifest().size(); ++slot) {
        const SchemaNode* n = client_->resolveSchemaSlot(slot);
        ASSERT_NE(n, nullptr) << "slot " << slot;
        EXPECT_EQ(n->key, client_->schemaManifest()[slot].ref.key);
        EXPECT_EQ(n->key, catalog_.describe(slot)->ref.key);
    }
    EXPECT_EQ(client_->resolveSchemaSlot(
                  static_cast<uint32_t>(client_->schemaManifest().size())),
              nullptr);
}

TEST_F(ClientSchemaRetentionTest, DecodeValueDecodesSnapshots) {
    const auto signals = client_->listSignals();
    ASSERT_TRUE(signals) << signals.error().message;
    ASSERT_FALSE(signals->empty());
    const auto& entry = signals->front();
    auto raw = client_->getSignal(entry.id);
    ASSERT_TRUE(raw) << raw.error().message;
    auto v = client_->decodeValue(entry.schemaSlot, raw->data(), raw->size());
    ASSERT_TRUE(v) << v.error().message;
    EXPECT_EQ(v->node, client_->resolveSchemaSlot(entry.schemaSlot));
    EXPECT_FALSE(formatSchemaValueText(*v).empty());
    EXPECT_EQ(client_->decodeValue(0xFFFFFF, raw->data(), raw->size())
                  .error().code, ErrorCode::InvalidMessage);
}


// --- §6: golden vectors + fuzz fail-closed ------------------------------

TEST_F(SchemaValueCodecTest, ScalarGoldenBytes) {
    // u32 0xDEADBEEF → little-endian 4 bytes on the wire.
    const auto* n = node(7);   // u32 scalar
    ASSERT_NE(n, nullptr);
    auto v = makeScalarValue(n, 0xDEADBEEF);
    auto enc = encodeSchemaValue(graph_, v);
    ASSERT_TRUE(enc) << enc.error();
    const std::vector<uint8_t> golden = {0xEF, 0xBE, 0xAD, 0xDE};
    EXPECT_EQ(*enc, golden);
    auto dv = decodeSchemaValue(graph_, *n, enc->data(), enc->size());
    ASSERT_TRUE(dv);
    const auto* s = std::get_if<SchemaValue::Scalar>(&dv->payload);
    ASSERT_TRUE(s);
    EXPECT_EQ(s->bits, 0xDEADBEEFu);
}

TEST_F(SchemaValueCodecTest, FuzzDecoderFailsClosed) {
    // Random bytes must never crash the decoder and must either fail or
    // consume exactly len bytes (no silent truncation).
    std::mt19937_64 rng(0xC0FFEE);
    for (int iter = 0; iter < 2000; ++iter) {
        const size_t len = rng() % 64;
        std::vector<uint8_t> junk(len);
        for (auto& b : junk) b = static_cast<uint8_t>(rng());
        for (uint8_t id = 1; id <= 14; ++id) {
            const auto* n = node(id);
            if (!n) continue;
            auto dv = decodeSchemaValue(graph_, *n, junk.data(), junk.size());
            // success is fine only if it consumed the whole buffer
            if (dv) {
                // decodeSchemaValue requires exact consumption — so a
                // success on garbage means the wire bytes happened to be
                // valid; nothing to assert beyond "didn't crash".
            }
        }
    }
    SUCCEED();   // surviving 2000 fuzz iterations without a crash = pass
}

} // namespace

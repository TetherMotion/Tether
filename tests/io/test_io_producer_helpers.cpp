// Producer conveniences: entryId, buildSchemaManifest + slotForKey,
// LatestSignal, atomicParam, makeAction, makeStringSignal,
// steadyClockUs/stderrLog.
#include <gtest/gtest.h>

#include <atomic>
#include <cstring>
#include <thread>
#include <vector>

#include "tether/io/LatestSignal.hpp"
#include "tether/io/ProducerHelpers.hpp"
#include "tether/io/SchemaCatalog.hpp"

using namespace tether::io;

namespace {

// ---------------------------------------------------------------------------
// entryId — canonical FNV-1a 64 over the dotted name.
// ---------------------------------------------------------------------------

TEST(EntryId, KnownVectors) {
    // FNV-1a64 reference values (independently computed).
    EXPECT_EQ(entryId(""), 14695981039346656037ull);  // offset basis
    EXPECT_EQ(entryId("a"), 12638187200555641996ull);
    EXPECT_EQ(entryId("state.frame"), 0xa5df44a673b8ebb5ull);
}
TEST(EntryId, DistinctNamesDistinctIds) {
    EXPECT_NE(entryId("a"), entryId("b"));
    EXPECT_NE(entryId("cmd.vx"), entryId("cmd.vy"));
    EXPECT_NE(entryId("state.frame"), entryId("stats.tick"));
}
TEST(EntryId, ConstexprAndStable) {
    static_assert(entryId("sim.reset") == entryId("sim.reset"));
    EXPECT_EQ(entryId("loop.paused"), entryId(std::string("loop.paused")));
}

// ---------------------------------------------------------------------------
// steadyClockUs / stderrLog
// ---------------------------------------------------------------------------

TEST(ProducerDefaults, SteadyClockMonotonic) {
    const uint64_t t0 = steadyClockUs();
    EXPECT_GT(t0, 0u);
    for (int i = 0; i < 1000; ++i)
        EXPECT_GE(steadyClockUs(), t0);  // non-decreasing
}
TEST(ProducerDefaults, SteadyClockMicrosRange) {
    // Plausible µs magnitude: between boot and ~1e9 years of uptime.
    EXPECT_LT(steadyClockUs(), 0xFFFFFFFFFFFFFFFFull / 2);
}
TEST(ProducerDefaults, StderrLogSmoke) {
    stderrLog("test", "value=%d", 42);   // must not crash
    stderrLog(nullptr, "no tag");        // null tag handled
}

// ---------------------------------------------------------------------------
// atomicParam — atomic shadow + read/write fns + onWrite hook.
// ---------------------------------------------------------------------------

TEST(AtomicParam, TypeMapping) {
    static_assert(valueTypeOf<bool>() == ValueType::Bool);
    static_assert(valueTypeOf<uint8_t>() == ValueType::U8);
    static_assert(valueTypeOf<uint16_t>() == ValueType::U16);
    static_assert(valueTypeOf<uint32_t>() == ValueType::U32);
    static_assert(valueTypeOf<uint64_t>() == ValueType::U64);
    static_assert(valueTypeOf<int8_t>() == ValueType::I8);
    static_assert(valueTypeOf<int16_t>() == ValueType::I16);
    static_assert(valueTypeOf<int32_t>() == ValueType::I32);
    static_assert(valueTypeOf<int64_t>() == ValueType::I64);
    static_assert(valueTypeOf<float>() == ValueType::F32);
    static_assert(valueTypeOf<double>() == ValueType::F64);
}

TEST(AtomicParam, EntryFieldsAndFlags) {
    std::atomic<double> slot{1.5};
    SchemaRef ref{}; ref.key[15] = 9;
    auto e = atomicParam(42, "imp.engagement", "control", ref, 3, slot);
    EXPECT_EQ(e.id, 42u);
    EXPECT_EQ(e.name, "imp.engagement");
    EXPECT_EQ(e.group, "control");
    EXPECT_EQ(e.valueType, ValueType::F64);
    EXPECT_EQ(e.schema.key, ref.key);
    EXPECT_EQ(e.schemaSlot, 3u);
    EXPECT_TRUE(e.writable());
    EXPECT_FALSE(e.isVariableLength());
    EXPECT_TRUE(e.flags() & EntryFlags::Readable);
    EXPECT_TRUE(e.flags() & EntryFlags::Writable);
}

TEST(AtomicParam, ReadLoadsSlot) {
    std::atomic<double> slot{2.25};
    auto e = atomicParam(1, "p", "g", SchemaRef{}, 0, slot);
    double v = 0;
    e.readFn(&v);
    EXPECT_DOUBLE_EQ(v, 2.25);
    slot.store(7.5);
    e.readFn(&v);
    EXPECT_DOUBLE_EQ(v, 7.5);
}

TEST(AtomicParam, WriteStoresSlotAndFiresHook) {
    std::atomic<uint32_t> slot{0};
    std::vector<uint32_t> seen;
    auto e = atomicParam(1, "p", "g", SchemaRef{}, 0, slot,
                         [&](uint32_t v) { seen.push_back(v); });
    const uint32_t in = 77;
    e.writeFn(&in);
    EXPECT_EQ(slot.load(), 77u);
    ASSERT_EQ(seen.size(), 1u);
    EXPECT_EQ(seen[0], 77u);
    const uint32_t in2 = 88;
    e.writeFn(&in2);
    EXPECT_EQ(slot.load(), 88u);
    EXPECT_EQ(seen.size(), 2u);
}

TEST(AtomicParam, WriteWithoutHookStillStores) {
    std::atomic<uint8_t> slot{0};
    auto e = atomicParam(1, "p", "g", SchemaRef{}, 0, slot);  // no hook
    const uint8_t in = 9;
    e.writeFn(&in);
    EXPECT_EQ(slot.load(), 9);
}

TEST(AtomicParam, BoolRoundTrip) {
    std::atomic<bool> slot{false};
    auto e = atomicParam(1, "loop.paused", "g", SchemaRef{}, 0, slot);
    EXPECT_EQ(e.valueType, ValueType::Bool);
    const bool in = true;
    e.writeFn(&in);
    EXPECT_TRUE(slot.load());
    bool out = false;
    e.readFn(&out);
    EXPECT_TRUE(out);
}

// ---------------------------------------------------------------------------
// makeAction — parameterless function entries.
// ---------------------------------------------------------------------------

TEST(MakeAction, VoidBodyRunsAndSucceeds) {
    int calls = 0;
    auto e = makeAction(entryId("sim.reset"), "sim.reset", "control",
                        [&] { ++calls; });
    EXPECT_EQ(e.name, "sim.reset");
    EXPECT_EQ(e.group, "control");
    EXPECT_TRUE(e.parameters.empty());
    auto r = e.callback({});
    EXPECT_TRUE(r.success);
    EXPECT_EQ(calls, 1);
}

TEST(MakeAction, ResultBodyControlsOutcome) {
    auto ok = makeAction(1, "engage", "g", [] {
        FunctionCallResult r;
        r.success = true;
        r.returnValue = {1, 2, 3};
        return r;
    });
    auto rok = ok.callback({});
    EXPECT_TRUE(rok.success);
    EXPECT_EQ(rok.returnValue.size(), 3u);

    auto fail = makeAction(2, "engage", "g", [] {
        FunctionCallResult r;
        r.success = false;
        r.errorMessage = "drive fault";
        return r;
    });
    auto rfail = fail.callback({});
    EXPECT_FALSE(rfail.success);
    EXPECT_EQ(rfail.errorMessage, "drive fault");
}

TEST(MakeAction, ValidSignatureAndRegistration) {
    Registry reg;
    auto e = makeAction(entryId("sim.pause"), "sim.pause", "control",
                        [] {});
    ASSERT_TRUE(e.validSignature());
    EXPECT_TRUE(reg.addFunction(std::move(e)));
}

// ---------------------------------------------------------------------------
// makeStringSignal — callback-produced NUL-terminated strings.
// ---------------------------------------------------------------------------

TEST(StringSignal, RoundTrip) {
    auto e = makeStringSignal(7, "meta.model", "meta", SchemaRef{}, 0,
                              [] { return std::string("alpha_v2"); });
    EXPECT_EQ(e.valueType, ValueType::String);
    EXPECT_EQ(e.maxValueSize, 4096u);
    uint8_t buf[64];
    const size_t n = e.varReadFn(buf, sizeof(buf));
    ASSERT_EQ(n, 9u);                      // "alpha_v2" + NUL
    EXPECT_EQ(buf[8], 0);
    EXPECT_STREQ(reinterpret_cast<char*>(buf), "alpha_v2");
}

TEST(StringSignal, TruncatesAtMaxLen) {
    auto e = makeStringSignal(1, "s", "g", SchemaRef{}, 0,
                              [] { return std::string("abcdefghij"); });
    uint8_t buf[5];
    const size_t n = e.varReadFn(buf, sizeof(buf));
    EXPECT_EQ(n, 5u);
    EXPECT_EQ(buf[4], 0);                  // NUL preserved at cap
    EXPECT_EQ(std::memcmp(buf, "abcd", 4), 0);
}

TEST(StringSignal, EmptyAndNullProducer) {
    auto e = makeStringSignal(1, "s", "g", SchemaRef{}, 0,
                              std::function<std::string()>{});
    uint8_t buf[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
    const size_t n = e.varReadFn(buf, sizeof(buf));
    EXPECT_EQ(n, 1u);                      // lone NUL
    EXPECT_EQ(buf[0], 0);
}

// ---------------------------------------------------------------------------
// LatestSignal — ring + latest snapshot.
// ---------------------------------------------------------------------------

TEST(LatestSignal, ProduceUpdatesLatestWhenRingInactive) {
    LatestSignal<64> sig;
    const uint8_t row[4] = {1, 2, 3, 4};
    EXPECT_TRUE(sig.produce(100, row, 4));
    // Ring inactive (no start()) — nothing drains…
    uint8_t dst[128];
    EXPECT_EQ(sig.source().drainRows(dst, 8), 0u);
    // …but latest is updated for polled reads.
    uint8_t snap[8];
    EXPECT_EQ(sig.readLatest(snap, sizeof(snap)), 4u);
    EXPECT_EQ(std::memcmp(snap, row, 4), 0);
    EXPECT_EQ(sig.latestSize(), 4u);
}

TEST(LatestSignal, StreamedRowsCarryTimestamp) {
    LatestSignal<64> sig;
    sig.setSchema({1, 2}, {4, 4});
    sig.source().start();
    const uint8_t a[4] = {9, 8, 7, 6}, b[4] = {5, 4, 3, 2};
    sig.produce(111, a, 4);
    sig.produce(222, b, 4);

    std::vector<uint8_t> buf(2 * sizeof(LatestSignal<64>::Row));
    const size_t rows = sig.source().drainRows(buf.data(), 2);
    ASSERT_EQ(rows, 2u);
    const auto* r0 = reinterpret_cast<const LatestSignal<64>::Row*>(buf.data());
    const auto* r1 = reinterpret_cast<const LatestSignal<64>::Row*>(
        buf.data() + sizeof(LatestSignal<64>::Row));
    EXPECT_EQ(r0->tsUs, 111u);
    EXPECT_EQ(r1->tsUs, 222u);
    EXPECT_EQ(std::memcmp(r0->payload.data(), a, 4), 0);
    EXPECT_EQ(std::memcmp(r1->payload.data(), b, 4), 0);
}

TEST(LatestSignal, OversizeProduceRejected) {
    LatestSignal<8> sig;
    const uint8_t big[16] = {};
    EXPECT_FALSE(sig.produce(1, big, sizeof(big)));
    EXPECT_EQ(sig.latestSize(), 0u);       // nothing committed
}

TEST(LatestSignal, ReadLatestTruncatesAtMaxLen) {
    LatestSignal<64> sig;
    const uint8_t row[10] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9};
    sig.produce(1, row, 10);
    uint8_t snap[4] = {};
    EXPECT_EQ(sig.readLatest(snap, sizeof(snap)), 4u);
    EXPECT_EQ(std::memcmp(snap, row, 4), 0);
}

TEST(LatestSignal, DroppedCountsRingOverflow) {
    LatestSignal<8, 4> sig;                // tiny ring
    sig.source().start();
    const uint8_t row[2] = {1, 2};
    for (int i = 0; i < 10; ++i) sig.produce(i, row, 2);
    EXPECT_EQ(sig.dropped(), 10u - sig.source().ring().size());
    // Latest still holds the most recent produce.
    uint8_t snap[2];
    EXPECT_EQ(sig.readLatest(snap, 2), 2u);
    EXPECT_EQ(snap[0], 1);
}

TEST(LatestSignal, VarReadFnReadsLatest) {
    LatestSignal<64> sig;
    auto rd = sig.varReadFn();
    const uint8_t row[3] = {7, 7, 7};
    sig.produce(1, row, 3);
    uint8_t snap[8];
    EXPECT_EQ(rd(snap, sizeof(snap)), 3u);
    EXPECT_EQ(std::memcmp(snap, row, 3), 0);
}

TEST(LatestSignal, InitialLatestSizePresized) {
    LatestSignal<64> sig(16);
    EXPECT_EQ(sig.latestSize(), 16u);      // declared bound before produce
    uint8_t snap[20];
    EXPECT_EQ(sig.readLatest(snap, sizeof(snap)), 16u);
}

TEST(LatestSignal, ConcurrentProducerAndReader) {
    LatestSignal<128> sig;
    std::atomic<bool> stop{false};
    std::atomic<size_t> reads{0};
    std::thread reader([&] {
        uint8_t buf[128];
        while (!stop.load()) { sig.readLatest(buf, sizeof(buf)); ++reads; }
    });
    uint8_t row[32] = {};
    for (int i = 0; i < 5000; ++i) {
        row[0] = static_cast<uint8_t>(i);
        ASSERT_TRUE(sig.produce(i, row, sizeof(row)));
    }
    stop.store(true);
    reader.join();
    EXPECT_GT(reads.load(), 0u);
}

// ---------------------------------------------------------------------------
// buildSchemaManifest + slotForKey
// ---------------------------------------------------------------------------

namespace {
SchemaKey keyOf(uint8_t k) {
    SchemaKey key{};
    key[15] = k;
    return key;
}
}  // namespace

TEST(BuildSchemaManifest, CoversAllNodesWithDigests) {
    SchemaGraph g;
    for (uint8_t i = 1; i <= 3; ++i) {
        SchemaNode n;
        n.key = keyOf(i);
        n.kind = SchemaKind::Scalar;
        n.scalarType = ValueType::F64;
        n.revision = i;
        g.nodes.push_back(n);
    }
    ASSERT_TRUE(resolveSchemaDigests(g));
    auto m = buildSchemaManifest(g);
    ASSERT_EQ(m.size(), 3u);
    for (size_t i = 0; i < m.size(); ++i) {
        EXPECT_EQ(m[i].ref.key, g.nodes[i].key);
        EXPECT_EQ(m[i].ref.digest, computeSchemaDigest(g.nodes[i]));
        EXPECT_EQ(m[i].revision, g.nodes[i].revision);
    }
}

TEST(BuildSchemaManifest, EmptyGraph) {
    SchemaGraph g;
    EXPECT_TRUE(buildSchemaManifest(g).empty());
}

TEST(SlotForKey, ResolvesInstalledKey) {
    SchemaGraph g;
    for (uint8_t i = 1; i <= 2; ++i) {
        SchemaNode n;
        n.key = keyOf(i);
        n.kind = SchemaKind::Scalar;
        n.scalarType = ValueType::F64;
        g.nodes.push_back(n);
    }
    ASSERT_TRUE(resolveSchemaDigests(g));
    SchemaCatalog cat;
    ASSERT_TRUE(cat.install(g, buildSchemaManifest(g)));
    auto s = cat.slotForKey(keyOf(2));
    ASSERT_TRUE(s.has_value());
    EXPECT_NE(cat.resolve(cat.epoch(), *s), nullptr);
    EXPECT_FALSE(cat.slotForKey(keyOf(99)).has_value());
}

TEST(SlotForKey, IgnoresDigestMismatch) {
    // slotForKey is the key-only lookup: a stale SchemaRef digest does not
    // matter when registering entries against the installed catalog.
    SchemaGraph g;
    SchemaNode n;
    n.key = keyOf(1);
    n.kind = SchemaKind::Scalar;
    n.scalarType = ValueType::F64;
    g.nodes.push_back(n);
    ASSERT_TRUE(resolveSchemaDigests(g));
    SchemaCatalog cat;
    ASSERT_TRUE(cat.install(g, buildSchemaManifest(g)));

    SchemaRef stale{keyOf(1), {}};
    stale.digest[0] ^= 0xFF;
    EXPECT_FALSE(cat.slotFor(stale).has_value());   // digest-gated
    EXPECT_TRUE(cat.slotForKey(keyOf(1)).has_value());
}

}  // namespace

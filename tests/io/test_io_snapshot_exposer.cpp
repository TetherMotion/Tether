/**
 * @file test_io_snapshot_exposer.cpp
 * @brief Unit tests for SnapshotExposer: self-describing Binary snapshot
 *        signals plus per-field scalar signals.
 */

#include <gtest/gtest.h>

#include "tether/io/Registry.hpp"
#include "tether/io/exposers/SnapshotExposer.hpp"

#include <cstring>

using namespace tether::io;
using namespace tether::io::exposers;

namespace {

/// Representative POD: mixed widths, a bool, an enum-valued byte.
struct DemoStats {
    uint64_t cycles;
    uint32_t max_work_us;
    uint16_t count;
    uint8_t  state;
    uint8_t  realtime_ok;
    uint8_t  _pad[2];
};

class SnapshotExposerTest : public ::testing::Test {
protected:
    static constexpr uint64_t kBase = ModuleId::Master;

    void SetUp() override {
        exposer_ = std::make_unique<SnapshotExposer>("demo");
        ASSERT_TRUE(exposer_->addSnapshot(
            0x0001, "demo.stats", "Demo stats snapshot", "demo",
            {
                {"cycles",      ValueType::U64, offsetof(DemoStats, cycles),      8, "cycles"},
                {"max_work_us", ValueType::U32, offsetof(DemoStats, max_work_us), 4, "us"},
                {"count",       ValueType::U16, offsetof(DemoStats, count),       2},
                {"state",       ValueType::U8,  offsetof(DemoStats, state),       1,
                 "", "0=Idle,1=Running,2=Fault"},
                {"realtime_ok", ValueType::Bool,offsetof(DemoStats, realtime_ok), 1},
            },
            sizeof(DemoStats),
            [this](void* d) { std::memcpy(d, &stats_, sizeof(stats_)); }));
        exposer_->expose(registry_, kBase);
    }

    DemoStats stats_{};
    Registry  registry_;
    std::unique_ptr<SnapshotExposer> exposer_;
};

TEST_F(SnapshotExposerTest, BinarySnapshotCarriesDescriptorAndPayload) {
    stats_.cycles       = 0x1122334455667788;
    stats_.max_work_us  = 4242;
    stats_.count        = 7;
    stats_.state        = 1;
    stats_.realtime_ok  = 1;

    EntryView snap = registry_.findSignal(makeId(kBase, 0x0001));
    ASSERT_TRUE(static_cast<bool>(snap));
    EXPECT_EQ(snap.valueType(), ValueType::Binary);
    EXPECT_TRUE(snap.flags() & EntryFlags::HasStruct);
    EXPECT_TRUE(snap.isVariableLength());
    EXPECT_EQ(snap.name(), "demo.stats");
    EXPECT_EQ(snap.group(), "demo");

    // Descriptor decodes the image without external schema.
    const StructDescriptor* sd = snap.structDesc();
    ASSERT_NE(sd, nullptr);
    EXPECT_EQ(sd->entryId, makeId(kBase, 0x0001));
    EXPECT_EQ(sd->totalSize, sizeof(DemoStats));
    ASSERT_EQ(sd->fields.size(), 5u);
    EXPECT_EQ(sd->fields[0].name, "cycles");
    EXPECT_EQ(sd->fields[0].type, ValueType::U64);
    EXPECT_EQ(sd->fields[0].offset, offsetof(DemoStats, cycles));
    EXPECT_EQ(sd->fields[0].size, 8u);

    uint8_t buf[sizeof(DemoStats)];
    ASSERT_EQ(snap.readVar(buf, sizeof(buf)), sizeof(DemoStats));
    EXPECT_EQ(std::memcmp(buf, &stats_, sizeof(stats_)), 0);

    // Too-small buffer reports nothing.
    EXPECT_EQ(snap.readVar(buf, 4), 0u);
}

TEST_F(SnapshotExposerTest, ScalarFieldSignalsReadThrough) {
    stats_.cycles      = 99;
    stats_.state       = 2;
    stats_.realtime_ok = 1;

    // Field signals occupy consecutive IDs after the snapshot.
    uint64_t v64 = 0;
    registry_.findSignal(makeId(kBase, 0x0002)).read(&v64);   // cycles
    EXPECT_EQ(v64, 99u);

    uint8_t v8 = 0;
    registry_.findSignal(makeId(kBase, 0x0005)).read(&v8);    // state
    EXPECT_EQ(v8, 2);
    registry_.findSignal(makeId(kBase, 0x0006)).read(&v8);    // realtime_ok
    EXPECT_EQ(v8, 1);

    EntryView st = registry_.findSignal(makeId(kBase, 0x0005));
    EXPECT_EQ(st.name(), "demo.stats.state");
}

TEST_F(SnapshotExposerTest, EnumLabelsBecomeMetadata) {
    bool sawIdle = false, sawFault = false;
    registry_.findSignal(makeId(kBase, 0x0005))
        .forEachMetadata([&](std::string_view k, std::string_view v) {
            if (k == "enum.0" && v == "Idle")  sawIdle  = true;
            if (k == "enum.2" && v == "Fault") sawFault = true;
        });
    EXPECT_TRUE(sawIdle);
    EXPECT_TRUE(sawFault);

    // Unit metadata lands on scalar fields.
    bool sawUs = false;
    registry_.findSignal(makeId(kBase, 0x0003))   // max_work_us
        .forEachMetadata([&](std::string_view k, std::string_view v) {
            if (k == "unit" && v == "us") sawUs = true;
        });
    EXPECT_TRUE(sawUs);
}

TEST_F(SnapshotExposerTest, InvalidSpecsRejected) {
    SnapshotExposer bad("bad");
    EXPECT_FALSE(bad.addSnapshot(1, "x", "d", "g", {}, 8,
                                 [](void*) {}));                    // no fields
    EXPECT_FALSE(bad.addSnapshot(1, "x", "d", "g",
                                 {{"f", ValueType::U8, 0, 1}}, 0,
                                 [](void*) {}));                    // zero size
    EXPECT_FALSE(bad.addSnapshot(1, "x", "d", "g",
                                 {{"f", ValueType::U8, 0, 1}},
                                 SnapshotExposer::kMaxSnapshotSize + 1,
                                 [](void*) {}));                    // oversized
    EXPECT_FALSE(bad.addSnapshot(1, "x", "d", "g",
                                 {{"f", ValueType::U8, 0, 1}}, 8,
                                 {}));                              // no callback
}

TEST_F(SnapshotExposerTest, ScalarFieldsOptional) {
    Registry reg;
    SnapshotExposer exp("mini");
    ASSERT_TRUE(exp.addSnapshot(0x10, "mini.s", "d", "g",
                                {{"f", ValueType::U8, 0, 1}},
                                sizeof(uint8_t),
                                [](void* d) { *static_cast<uint8_t*>(d) = 42; },
                                /*exposeScalarFields=*/false));
    exp.expose(reg, kBase);
    EXPECT_TRUE(static_cast<bool>(reg.findSignal(makeId(kBase, 0x10))));
    EXPECT_FALSE(static_cast<bool>(reg.findSignal(makeId(kBase, 0x11))));
}

} // namespace

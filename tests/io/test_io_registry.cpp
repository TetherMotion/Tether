/**
 * @file test_io_registry.cpp
 * @brief Unit tests for Registry — add, find, page, change listeners.
 */
#include <gtest/gtest.h>
#include "tether/io/Registry.hpp"
#include <atomic>

using namespace tether::io;

// ===========================================================================
// Basic add/find
// ===========================================================================

TEST(IORegistry, AddParamAndFind) {
    Registry reg;
    double value = 42.0;

    ParamEntry entry;
    entry.id = 0x0001;
    entry.name = "test_param";
    entry.description = "A test parameter";
    entry.group = "test";
    entry.valueType = ValueType::F64;
    entry.readFn = [&value](void* d) { std::memcpy(d, &value, 8); };
    entry.writeFn = [&value](const void* s) { std::memcpy(&value, s, 8); };

    EXPECT_TRUE(reg.addParam(std::move(entry)));
    EXPECT_EQ(reg.paramCount(), 1u);
    EXPECT_EQ(reg.signalCount(), 0u);
    EXPECT_EQ(reg.totalCount(), 1u);

    EntryView view = reg.findParam(0x0001);
    ASSERT_TRUE(static_cast<bool>(view));
    EXPECT_EQ(view.id(), 0x0001u);
    EXPECT_EQ(view.kind(), EntryKind::Parameter);
    EXPECT_EQ(view.valueType(), ValueType::F64);
    EXPECT_EQ(view.valueSize(), 8u);
    EXPECT_EQ(view.name(), "test_param");
    EXPECT_EQ(view.description(), "A test parameter");
    EXPECT_EQ(view.group(), "test");
    EXPECT_TRUE(view.writable());
}

TEST(IORegistry, AddSignalAndFind) {
    Registry reg;
    int32_t value = -99;

    SignalEntry entry;
    entry.id = 0x0002;
    entry.name = "test_signal";
    entry.description = "A test signal";
    entry.group = "test";
    entry.valueType = ValueType::I32;
    entry.readFn = [&value](void* d) { std::memcpy(d, &value, 4); };

    EXPECT_TRUE(reg.addSignal(std::move(entry)));
    EXPECT_EQ(reg.paramCount(), 0u);
    EXPECT_EQ(reg.signalCount(), 1u);

    EntryView view = reg.findSignal(0x0002);
    ASSERT_TRUE(static_cast<bool>(view));
    EXPECT_EQ(view.kind(), EntryKind::Signal);
    EXPECT_FALSE(view.writable());
}

TEST(IORegistry, DuplicateIdRejected) {
    Registry reg;
    ParamEntry p1;
    p1.id = 0x0001;
    p1.name = "p1";
    p1.valueType = ValueType::U8;
    p1.readFn = [](void*) {};
    EXPECT_TRUE(reg.addParam(std::move(p1)));

    ParamEntry p2;
    p2.id = 0x0001;
    p2.name = "p2";
    p2.valueType = ValueType::U8;
    p2.readFn = [](void*) {};
    EXPECT_FALSE(reg.addParam(std::move(p2)));

    // Also can't add a signal with the same ID
    SignalEntry s1;
    s1.id = 0x0001;
    s1.name = "s1";
    s1.valueType = ValueType::U8;
    s1.readFn = [](void*) {};
    EXPECT_FALSE(reg.addSignal(std::move(s1)));
}

TEST(IORegistry, FindNonexistent) {
    Registry reg;
    EntryView v = reg.find(0x9999);
    EXPECT_FALSE(static_cast<bool>(v));

    EntryView vp = reg.findParam(0x9999);
    EXPECT_FALSE(static_cast<bool>(vp));

    EntryView vs = reg.findSignal(0x9999);
    EXPECT_FALSE(static_cast<bool>(vs));
}

// ===========================================================================
// Read/Write through EntryView
// ===========================================================================

TEST(IORegistry, ReadWriteParam) {
    Registry reg;
    uint32_t value = 100;

    ParamEntry entry;
    entry.id = 1;
    entry.name = "rw";
    entry.valueType = ValueType::U32;
    entry.readFn = [&value](void* d) { std::memcpy(d, &value, 4); };
    entry.writeFn = [&value](const void* s) { std::memcpy(&value, s, 4); };
    reg.addParam(std::move(entry));

    EntryView v = reg.findParam(1);
    ASSERT_TRUE(static_cast<bool>(v));

    // Read
    uint32_t readVal = 0;
    v.read(&readVal);
    EXPECT_EQ(readVal, 100u);

    // Write
    uint32_t newVal = 999;
    v.write(&newVal);
    EXPECT_EQ(value, 999u);

    // Read back
    v.read(&readVal);
    EXPECT_EQ(readVal, 999u);
}

TEST(IORegistry, ReadSignal) {
    Registry reg;
    float value = 3.14f;

    SignalEntry entry;
    entry.id = 1;
    entry.name = "sig";
    entry.valueType = ValueType::F32;
    entry.readFn = [&value](void* d) { std::memcpy(d, &value, 4); };
    reg.addSignal(std::move(entry));

    EntryView v = reg.findSignal(1);
    ASSERT_TRUE(static_cast<bool>(v));
    float readVal = 0;
    v.read(&readVal);
    EXPECT_FLOAT_EQ(readVal, 3.14f);

    // write on signal should be no-op
    float newVal = 99.0f;
    v.write(&newVal);
    v.read(&readVal);
    EXPECT_FLOAT_EQ(readVal, 3.14f);  // unchanged
}

// ===========================================================================
// Pagination
// ===========================================================================

TEST(IORegistry, ParamPage) {
    Registry reg;
    for (uint64_t i = 0; i < 10; ++i) {
        ParamEntry e;
        e.id = i;
        e.name = "p" + std::to_string(i);
        e.valueType = ValueType::U8;
        e.readFn = [](void*) {};
        reg.addParam(std::move(e));
    }

    auto page = reg.paramPage(0, 5);
    EXPECT_EQ(page.size(), 5u);

    page = reg.paramPage(5, 5);
    EXPECT_EQ(page.size(), 5u);

    page = reg.paramPage(8, 5);
    EXPECT_EQ(page.size(), 2u);

    page = reg.paramPage(10, 5);
    EXPECT_EQ(page.size(), 0u);
}

TEST(IORegistry, SignalPage) {
    Registry reg;
    for (uint64_t i = 0; i < 3; ++i) {
        SignalEntry e;
        e.id = i;
        e.name = "s" + std::to_string(i);
        e.valueType = ValueType::U16;
        e.readFn = [](void*) {};
        reg.addSignal(std::move(e));
    }

    auto page = reg.signalPage(0, 100);
    EXPECT_EQ(page.size(), 3u);
}

// ===========================================================================
// Find (unified) — works for both params and signals
// ===========================================================================

TEST(IORegistry, FindUnified) {
    Registry reg;
    ParamEntry p;
    p.id = 1;
    p.name = "param";
    p.valueType = ValueType::U8;
    p.readFn = [](void*) {};
    reg.addParam(std::move(p));

    SignalEntry s;
    s.id = 2;
    s.name = "signal";
    s.valueType = ValueType::U16;
    s.readFn = [](void*) {};
    reg.addSignal(std::move(s));

    EntryView vp = reg.find(1);
    ASSERT_TRUE(static_cast<bool>(vp));
    EXPECT_EQ(vp.kind(), EntryKind::Parameter);

    EntryView vs = reg.find(2);
    ASSERT_TRUE(static_cast<bool>(vs));
    EXPECT_EQ(vs.kind(), EntryKind::Signal);
}

// ===========================================================================
// Change listeners
// ===========================================================================

TEST(IORegistry, ChangeListenerNotified) {
    Registry reg;
    std::atomic<int> callCount{0};

    auto handle = reg.addChangeListener([&callCount]() {
        callCount.fetch_add(1);
    });

    ParamEntry p;
    p.id = 1;
    p.name = "p";
    p.valueType = ValueType::U8;
    p.readFn = [](void*) {};
    reg.addParam(std::move(p));
    EXPECT_EQ(callCount.load(), 1);

    SignalEntry s;
    s.id = 2;
    s.name = "s";
    s.valueType = ValueType::U8;
    s.readFn = [](void*) {};
    reg.addSignal(std::move(s));
    EXPECT_EQ(callCount.load(), 2);

    reg.removeChangeListener(handle);
    ParamEntry p2;
    p2.id = 3;
    p2.name = "p2";
    p2.valueType = ValueType::U8;
    p2.readFn = [](void*) {};
    reg.addParam(std::move(p2));
    EXPECT_EQ(callCount.load(), 2);  // listener removed, no more calls
}

TEST(IORegistry, Revision) {
    Registry reg;
    EXPECT_EQ(reg.revision(), 0u);

    ParamEntry p;
    p.id = 1;
    p.name = "p";
    p.valueType = ValueType::U8;
    p.readFn = [](void*) {};
    reg.addParam(std::move(p));
    EXPECT_EQ(reg.revision(), 1u);

    SignalEntry s;
    s.id = 2;
    s.name = "s";
    s.valueType = ValueType::U8;
    s.readFn = [](void*) {};
    reg.addSignal(std::move(s));
    EXPECT_EQ(reg.revision(), 2u);
}

// ===========================================================================
// Entry flags
// ===========================================================================

TEST(IORegistry, ParamFlags) {
    ParamEntry p;
    p.id = 1;
    p.valueType = ValueType::U32;
    p.readFn = [](void*) {};
    p.writeFn = [](const void*) {};

    uint8_t f = p.flags();
    EXPECT_TRUE(f & EntryFlags::Readable);
    EXPECT_TRUE(f & EntryFlags::Writable);
    EXPECT_FALSE(f & EntryFlags::VariableLen);
    EXPECT_FALSE(f & EntryFlags::HasStruct);
}

TEST(IORegistry, SignalFlags) {
    SignalEntry s;
    s.id = 1;
    s.valueType = ValueType::String;
    s.readFn = [](void*) {};

    uint8_t f = s.flags();
    EXPECT_TRUE(f & EntryFlags::Readable);
    EXPECT_FALSE(f & EntryFlags::Writable);
    EXPECT_TRUE(f & EntryFlags::VariableLen);
}

// ===========================================================================
// Metadata
// ===========================================================================

TEST(IORegistry, MetadataAccessViaEntryView) {
    Registry reg;

    ParamEntry p;
    p.id = 1;
    p.name = "test";
    p.valueType = ValueType::F64;
    p.readFn = [](void*) {};
    p.metadata["unit"] = "mm/s";
    p.metadata["range"] = "0..1000";
    reg.addParam(std::move(p));

    EntryView v = reg.findParam(1);
    ASSERT_TRUE(static_cast<bool>(v));
    EXPECT_EQ(v.metadataCount(), 2u);

    std::map<std::string, std::string> md;
    v.forEachMetadata([&md](std::string_view k, std::string_view v) {
        md[std::string(k)] = std::string(v);
    });
    EXPECT_EQ(md["unit"], "mm/s");
    EXPECT_EQ(md["range"], "0..1000");
}

/**
 * @file test_io_registry_extra.cpp
 * @brief Additional coverage tests for Registry and EntryView.
 */
#include <gtest/gtest.h>
#include "tether/io/Registry.hpp"
#include <cstring>

using namespace tether::io;

TEST(IORegistryExtra, FunctionSignatureAndStableLookup) {
    Registry reg;
    FunctionEntry function;
    function.id = 100;
    function.name = "move";
    function.parameters = {
        {"axis", "Axis index", ValueType::U8},
        {"speed", "Optional speed", ValueType::U32, true, true, {10, 0, 0, 0}}};
    function.callback = [](const std::vector<FunctionArgument>&) {
        FunctionCallResult result;
        result.success = true;
        return result;
    };
    ASSERT_TRUE(reg.addFunction(std::move(function)));
    EXPECT_EQ(reg.functionCount(), 1u);
    ASSERT_TRUE(reg.findFunction(100));
    EXPECT_EQ(reg.findFunction(100).requiredParameterCount(), 1u);

    SignalEntry duplicate;
    duplicate.id = 100;
    duplicate.name = "duplicate";
    duplicate.valueType = ValueType::U8;
    duplicate.readFn = [](void*) {};
    EXPECT_FALSE(reg.addSignal(std::move(duplicate)));
}

TEST(IORegistryExtra, FunctionRejectsRequiredAfterOptional) {
    Registry reg;
    FunctionEntry function;
    function.id = 101;
    function.name = "invalid";
    function.parameters = {
        {"optional", "", ValueType::U8, true, true, {1}},
        {"required", "", ValueType::U8, false, false, {}}};
    function.callback = [](const std::vector<FunctionArgument>&) { return FunctionCallResult{}; };
    EXPECT_FALSE(reg.addFunction(std::move(function)));
}

// ===========================================================================
// EntryView edge cases
// ===========================================================================

TEST(IORegistryExtra, DefaultEntryViewIsFalse) {
    EntryView v;
    EXPECT_FALSE(static_cast<bool>(v));
}

TEST(IORegistryExtra, EntryViewAsParamAsSignal) {
    Registry reg;
    uint32_t val = 42;

    ParamEntry p;
    p.id = 1;
    p.name = "p1";
    p.valueType = ValueType::U32;
    p.readFn = [&val](void* d) { std::memcpy(d, &val, 4); };
    reg.addParam(std::move(p));

    SignalEntry s;
    s.id = 2;
    s.name = "s1";
    s.valueType = ValueType::U32;
    s.readFn = [&val](void* d) { std::memcpy(d, &val, 4); };
    reg.addSignal(std::move(s));

    EntryView vp = reg.findParam(1);
    EXPECT_NE(vp.asParam(), nullptr);
    EXPECT_EQ(vp.asSignal(), nullptr);

    EntryView vs = reg.findSignal(2);
    EXPECT_EQ(vs.asParam(), nullptr);
    EXPECT_NE(vs.asSignal(), nullptr);
}

// ===========================================================================
// Variable-length entries
// ===========================================================================

TEST(IORegistryExtra, VariableLengthParam) {
    Registry reg;
    std::string value = "hello";

    ParamEntry p;
    p.id = 1;
    p.name = "var_param";
    p.valueType = ValueType::String;
    p.readFn = [](void*) {};
    p.varReadFn = [&value](void* d, size_t maxLen) -> size_t {
        size_t n = std::min(value.size(), maxLen);
        std::memcpy(d, value.data(), n);
        return n;
    };
    p.varWriteFn = [&value](const void* s, size_t len) {
        value = std::string(reinterpret_cast<const char*>(s), len);
    };
    p.maxValueSize = 256;
    reg.addParam(std::move(p));

    EntryView v = reg.findParam(1);
    ASSERT_TRUE(static_cast<bool>(v));
    EXPECT_TRUE(v.isVariableLength());
    EXPECT_EQ(v.maxValueSize(), 256u);

    // Read variable
    char buf[32];
    size_t n = v.readVar(buf, sizeof(buf));
    EXPECT_EQ(n, 5u);
    EXPECT_EQ(std::string(buf, n), "hello");

    // Write variable
    std::string newVal = "world!";
    v.writeVar(newVal.data(), newVal.size());
    EXPECT_EQ(value, "world!");
}

TEST(IORegistryExtra, VariableLengthSignal) {
    Registry reg;
    std::string value = "signal_data";

    SignalEntry s;
    s.id = 1;
    s.name = "var_signal";
    s.valueType = ValueType::Binary;
    s.readFn = [](void*) {};
    s.varReadFn = [&value](void* d, size_t maxLen) -> size_t {
        size_t n = std::min(value.size(), maxLen);
        std::memcpy(d, value.data(), n);
        return n;
    };
    s.maxValueSize = 128;
    reg.addSignal(std::move(s));

    EntryView v = reg.findSignal(1);
    ASSERT_TRUE(static_cast<bool>(v));
    EXPECT_TRUE(v.isVariableLength());

    char buf[64];
    size_t n = v.readVar(buf, sizeof(buf));
    EXPECT_EQ(n, 11u);
}

TEST(IORegistryExtra, ReadVarOnFixedSizeParam) {
    Registry reg;
    float val = 1.0f;

    ParamEntry p;
    p.id = 1;
    p.name = "fixed";
    p.valueType = ValueType::F32;
    p.readFn = [&val](void* d) { std::memcpy(d, &val, 4); };
    reg.addParam(std::move(p));

    EntryView v = reg.findParam(1);
    char buf[32];
    size_t n = v.readVar(buf, sizeof(buf));
    EXPECT_EQ(n, 0u);  // No varReadFn
}

TEST(IORegistryExtra, WriteVarOnSignalNoOp) {
    Registry reg;
    SignalEntry s;
    s.id = 1;
    s.name = "sig";
    s.valueType = ValueType::U32;
    s.readFn = [](void*) {};
    reg.addSignal(std::move(s));

    EntryView v = reg.findSignal(1);
    // writeVar on a signal should be a no-op
    uint32_t data = 42;
    v.writeVar(&data, 4);  // should not crash
}

// ===========================================================================
// Struct entry with descriptor
// ===========================================================================

TEST(IORegistryExtra, ParamWithStructDescriptor) {
    Registry reg;

    StructDescriptor sd;
    sd.entryId = 1;
    sd.name = "MyStruct";
    sd.totalSize = 8;
    sd.fields = {
        {"x", ValueType::F32, 0, 4, "m"},
        {"y", ValueType::F32, 4, 4, "m"},
    };

    ParamEntry p;
    p.id = 1;
    p.name = "struct_param";
    p.valueType = ValueType::Struct;
    p.readFn = [](void*) {};
    p.structDesc = &sd;
    reg.addParam(std::move(p));

    EntryView v = reg.findParam(1);
    ASSERT_TRUE(static_cast<bool>(v));
    EXPECT_NE(v.structDesc(), nullptr);
    EXPECT_EQ(v.structDesc()->name, "MyStruct");
    EXPECT_TRUE(v.flags() & EntryFlags::HasStruct);
    EXPECT_TRUE(v.isVariableLength());  // Struct is variable
}

// ===========================================================================
// FindParam returns nothing for signals and vice versa
// ===========================================================================

TEST(IORegistryExtra, FindParamDoesNotReturnSignal) {
    Registry reg;
    SignalEntry s;
    s.id = 1;
    s.name = "sig";
    s.valueType = ValueType::U8;
    s.readFn = [](void*) {};
    reg.addSignal(std::move(s));

    EntryView v = reg.findParam(1);
    EXPECT_FALSE(static_cast<bool>(v));
}

TEST(IORegistryExtra, FindSignalDoesNotReturnParam) {
    Registry reg;
    ParamEntry p;
    p.id = 1;
    p.name = "param";
    p.valueType = ValueType::U8;
    p.readFn = [](void*) {};
    reg.addParam(std::move(p));

    EntryView v = reg.findSignal(1);
    EXPECT_FALSE(static_cast<bool>(v));
}

// ===========================================================================
// Multiple change listeners
// ===========================================================================

TEST(IORegistryExtra, MultipleChangeListeners) {
    Registry reg;
    int count1 = 0, count2 = 0;

    auto h1 = reg.addChangeListener([&count1]() { ++count1; });
    auto h2 = reg.addChangeListener([&count2]() { ++count2; });

    ParamEntry p;
    p.id = 1;
    p.name = "p";
    p.valueType = ValueType::U8;
    p.readFn = [](void*) {};
    reg.addParam(std::move(p));

    EXPECT_EQ(count1, 1);
    EXPECT_EQ(count2, 1);

    reg.removeChangeListener(h1);

    SignalEntry s;
    s.id = 2;
    s.name = "s";
    s.valueType = ValueType::U8;
    s.readFn = [](void*) {};
    reg.addSignal(std::move(s));

    EXPECT_EQ(count1, 1);
    EXPECT_EQ(count2, 2);

    reg.removeChangeListener(h2);
}

// ===========================================================================
// TotalCount
// ===========================================================================

TEST(IORegistryExtra, TotalCount) {
    Registry reg;

    ParamEntry p;
    p.id = 1;
    p.name = "p";
    p.valueType = ValueType::U8;
    p.readFn = [](void*) {};
    reg.addParam(std::move(p));

    SignalEntry s;
    s.id = 2;
    s.name = "s";
    s.valueType = ValueType::U8;
    s.readFn = [](void*) {};
    reg.addSignal(std::move(s));

    EXPECT_EQ(reg.totalCount(), 2u);
}

// ===========================================================================
// Entry with no metadata
// ===========================================================================

TEST(IORegistryExtra, NoMetadata) {
    Registry reg;
    ParamEntry p;
    p.id = 1;
    p.name = "p";
    p.valueType = ValueType::U8;
    p.readFn = [](void*) {};
    reg.addParam(std::move(p));

    EntryView v = reg.findParam(1);
    EXPECT_EQ(v.metadataCount(), 0u);

    int count = 0;
    v.forEachMetadata([&count](std::string_view, std::string_view) { ++count; });
    EXPECT_EQ(count, 0);
}

// ===========================================================================
// SignalEntry flags with structDesc
// ===========================================================================

TEST(IORegistryExtra, SignalWithStructDescFlags) {
    StructDescriptor sd;
    sd.entryId = 1;
    sd.name = "S";

    SignalEntry s;
    s.id = 1;
    s.name = "sig";
    s.valueType = ValueType::Struct;
    s.readFn = [](void*) {};
    s.structDesc = &sd;

    EXPECT_TRUE(s.flags() & EntryFlags::HasStruct);
    EXPECT_TRUE(s.isVariableLength());
}

// ===========================================================================
// ParamEntry writable with only varWriteFn
// ===========================================================================

TEST(IORegistryExtra, ParamWritableViaVarWriteFn) {
    ParamEntry p;
    p.id = 1;
    p.name = "p";
    p.valueType = ValueType::String;
    p.readFn = [](void*) {};
    p.varWriteFn = [](const void*, size_t) {};

    EXPECT_TRUE(p.writable());
    EXPECT_TRUE(p.flags() & EntryFlags::Writable);
}

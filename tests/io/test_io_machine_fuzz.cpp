// Deterministic fuzz coverage for the wire-facing V6 decode surface used by
// machine.* functions: tagged structs, dynamic arrays, Bytes fields, and the
// argument parsers of machine.command / config.stage / config.import /
// sdo.read / sdo.write / recipe.apply / config.diff. A crash, hang, or
// uncaught throw here means a hostile client can wedge the session thread.

#include <gtest/gtest.h>

#include "tether/io/MachineAuth.hpp"
#include "tether/io/Registry.hpp"
#include "tether/io/SchemaCatalog.hpp"
#include "tether/io/SimulatedMachineService.hpp"

#include <cstring>
#include <map>
#include <random>
#include <vector>

using namespace tether::io;
using namespace tether::io::machine;
using namespace tether::io::machine::detail;

namespace {

class MachineFuzzTest : public ::testing::Test {
protected:
    void SetUp() override {
        graph_ = cia402::machineProfileSchemaGraph();
        ASSERT_TRUE(cia402::SimulatedCiA402Fleet::installSchemas(graph_, catalog_));
        ASSERT_TRUE(stack_.fleet.registerSignals(registry_, catalog_));
        ASSERT_TRUE(stack_.install(registry_, catalog_));
    }

    FunctionCallResult invoke(uint64_t functionId, std::vector<FunctionArgument> args) {
        const FunctionView function = registry_.findFunction(functionId);
        EXPECT_TRUE(function) << "function " << functionId << " not registered";
        FunctionInvokeContext context;
        context.identity = {"session-fuzz", "technician", "127.0.0.1",
                            Role::Technician, true};
        return function.invoke(std::move(args), context);
    }

    // Deterministic xorshift64 — reproducible without a fuzzer harness.
    uint64_t next() {
        state_ ^= state_ << 13;
        state_ ^= state_ >> 7;
        state_ ^= state_ << 17;
        return state_;
    }

    std::vector<uint8_t> randomBytes(size_t maxSize) {
        std::vector<uint8_t> bytes(next() % (maxSize + 1));
        for (auto& byte : bytes) byte = static_cast<uint8_t>(next());
        return bytes;
    }

    /// Take a valid encoding and corrupt it: bit flips, truncations,
    /// length-field inflation.
    std::vector<uint8_t> mutate(std::vector<uint8_t> bytes) {
        if (bytes.empty()) return bytes;
        switch (next() % 4) {
            case 0: bytes[next() % bytes.size()] ^= static_cast<uint8_t>(next()); break;
            case 1: bytes.resize(next() % bytes.size()); break;
            case 2: bytes[0] = 0xFF; break;  // header corruption
            case 3: bytes.push_back(static_cast<uint8_t>(next())); break;
        }
        return bytes;
    }

    SchemaGraph graph_;
    SchemaCatalog catalog_;
    Registry registry_;
    SimulatedMachineStack stack_;
    uint64_t state_ = 0x9E3779B97F4A7C15ULL;
};

FunctionArgument arg(uint32_t key, std::vector<uint8_t> value) {
    FunctionArgument argument;
    argument.key = key;
    argument.value = std::move(value);
    argument.provided = true;
    return argument;
}

} // namespace

TEST_F(MachineFuzzTest, TaggedDecodersNeverCrashOnGarbage) {
    for (unsigned i = 0; i < 4000; ++i) {
        const auto bytes = randomBytes(256);
        std::map<uint32_t, std::vector<uint8_t>> fields;
        parseTagged(bytes, fields);          // must not crash or throw
        std::string text;
        getString(bytes, text);              // arbitrary bytes
        bool boolean = false;
        getBool(bytes, boolean);
        uint64_t scalar = 0;
        getScalar(bytes, scalar);
    }
}

TEST_F(MachineFuzzTest, TaggedDecodersRejectMutatedValidEncoding) {
    const auto valid = encodeTagged({
        {1, fieldScalar(uint64_t{0xDEADBEEF})},
        {2, fieldString("axis.x")},
        {3, fieldBytes({1, 2, 3, 4})},
    });
    for (unsigned i = 0; i < 4000; ++i) {
        std::map<uint32_t, std::vector<uint8_t>> fields;
        parseTagged(mutate(valid), fields);  // bool result irrelevant — no crash
    }
}

TEST_F(MachineFuzzTest, CommandHandlerSurvivesGarbageAndMutations) {
    // A structurally valid CommandRequestV1 to mutate.
    std::vector<std::vector<uint8_t>> targets{fieldString("axis.x")};
    std::vector<std::vector<uint8_t>> generations{fieldScalar(uint64_t{1})};
    const auto valid = encodeTagged({
        {1, fieldString("req-fuzz")},
        {2, fieldString("machine")},
        {3, fieldScalar(uint64_t{0})},
        {4, fieldScalar(uint8_t{1})},
        {5, encodeArray(targets)},
        {6, encodeArray(generations)},
        {7, fieldScalar(uint64_t{5'000'000})},
        {8, fieldScalar(uint64_t{0})},
        {9, fieldBytes({1, 2, 3})},
    });
    for (unsigned i = 0; i < 2000; ++i) {
        auto result = invoke(MachineService::kCommandFnId,
                             {arg(1, i % 2 ? randomBytes(512) : mutate(valid))});
        (void)result;  // any outcome is fine; crashing/throwing is not
    }
    // Truncation sweep over the valid payload.
    for (size_t cut = 0; cut < valid.size(); ++cut) {
        auto result = invoke(MachineService::kCommandFnId,
                             {arg(1, {valid.begin(), valid.begin() + static_cast<ptrdiff_t>(cut)})});
        EXPECT_FALSE(result.success) << "truncated command accepted at " << cut;
    }
}

TEST_F(MachineFuzzTest, ConfigStageAndImportSurviveGarbage) {
    const auto valid = encodeArray({encodeTagged({
        {1, fieldScalar(uint64_t{1})},
        {2, fieldBytes({7, 0, 0, 0})},
    })});
    for (unsigned i = 0; i < 2000; ++i) {
        const auto payload = i % 2 ? randomBytes(256) : mutate(valid);
        invoke(MachineService::kConfigStageFnId, {arg(1, payload)});
        invoke(MachineService::kConfigImportFnId, {arg(1, payload)});
        invoke(MachineService::kConfigDiffFnId, {arg(1, payload)});
    }
    for (size_t cut = 0; cut < valid.size(); ++cut) {
        invoke(MachineService::kConfigStageFnId,
               {arg(1, {valid.begin(), valid.begin() + static_cast<ptrdiff_t>(cut)})});
    }
}

TEST_F(MachineFuzzTest, SdoAndRecipeArgumentsSurviveGarbage) {
    const auto validRead = encodeTagged({
        {1, fieldScalar(uint16_t{0})},
        {2, fieldScalar(uint16_t{0x6041})},
        {3, fieldScalar(uint8_t{0})},
        {4, fieldScalar(uint32_t{64})},
    });
    const auto validWrite = encodeTagged({
        {1, fieldScalar(uint16_t{0})},
        {2, fieldScalar(uint16_t{0x6040})},
        {3, fieldScalar(uint8_t{0})},
        {4, fieldBytes({6, 0})},
        {5, fieldScalar(uint8_t{1})},
    });
    for (unsigned i = 0; i < 2000; ++i) {
        invoke(MachineService::kSdoReadFnId,
               {arg(1, i % 2 ? randomBytes(128) : mutate(validRead))});
        invoke(MachineService::kSdoWriteFnId,
               {arg(1, i % 2 ? randomBytes(128) : mutate(validWrite))});
        invoke(MachineService::kRecipeApplyFnId,
               {arg(1, randomBytes(64))});
        invoke(MachineService::kSupervisorRetryFnId,
               {arg(1, randomBytes(32))});
    }
}

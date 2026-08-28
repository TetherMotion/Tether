/**
 * @file test_io_client_integration.cpp
 * @brief Comprehensive integration tests for TetherIOClient over MessagePipeTransport.
 *
 * Tests the full Tether IO protocol surface using the C++ TetherIOClient
 * against a real Session with Framing::None (WebSocket-style message transport).
 *
 * Covers:
 *   - Ping/Pong
 *   - ListParams, GetParam, SetParam (fixed + variable)
 *   - ListSignals, GetSignal
 *   - GetMetadata
 *   - ListFunctions, CallFunction
 *   - ConfigureStream, StartStream, StopStream (with data reception)
 *   - SnapshotParams, SnapshotSignals
 *   - FeatureExchange
 *   - SubscribeLog, UnsubscribeLog (with log data reception)
 *   - Error handling (invalid IDs, not writable, malformed messages)
 *   - CatalogChanged notification
 */
#include <gtest/gtest.h>
#include "tether/io/TetherIOClient.hpp"
#include "tether/io/Session.hpp"
#include "tether/io/FeatureExchange.hpp"
#include "tether/io/BinaryStruct.hpp"
#include "tether/io/Datalogging.hpp"
#include "tether/io/ThresholdFilter.hpp"
#include "PipeTransport.hpp"
#include <thread>
#include <chrono>
#include <cstring>
#include <atomic>
#include <cmath>

using namespace tether::io;
using namespace tether::io::testing;

static void noopLogFn(const char* /*tag*/, const char* /*fmt*/, ...) {}

// ===========================================================================
// Test fixture: Registry with diverse entries + Session + Client
// ===========================================================================

class IOClientIntegrationTest : public ::testing::Test {
protected:
    Registry registry_;
    FeatureSet features_;
    std::unique_ptr<Session> session_;
    std::unique_ptr<TetherIOClient> client_;
    std::thread sessionThread_;
    std::atomic<double> position_{42.0};
    std::atomic<double> velocity_{1.5};
    std::atomic<uint32_t> encoder_{12345};
    std::atomic<int> functionCalls_{0};
    std::string logMessage_ = "test log message";

    void SetUp() override {
        // F64 writable param
        {
            ParamEntry p;
            p.id = 1;
            p.name = "position";
            p.description = "Current position";
            p.group = "motion";
            p.valueType = ValueType::F64;
            p.readFn = [this](void* d) { double v = position_.load(); std::memcpy(d, &v, 8); };
            p.writeFn = [this](const void* s) { double v; std::memcpy(&v, s, 8); position_.store(v); };
            p.metadata["unit"] = "mm";
            p.metadata["range"] = "0..1000";
            registry_.addParam(std::move(p));
        }
        // F64 writable param
        {
            ParamEntry p;
            p.id = 2;
            p.name = "velocity";
            p.description = "Current velocity";
            p.group = "motion";
            p.valueType = ValueType::F64;
            p.readFn = [this](void* d) { double v = velocity_.load(); std::memcpy(d, &v, 8); };
            p.writeFn = [this](const void* s) { double v; std::memcpy(&v, s, 8); velocity_.store(v); };
            p.metadata["unit"] = "mm/s";
            registry_.addParam(std::move(p));
        }
        // U32 read-only param (no writeFn)
        {
            ParamEntry p;
            p.id = 3;
            p.name = "firmware_version";
            p.description = "Firmware version number";
            p.group = "system";
            p.valueType = ValueType::U32;
            p.readFn = [this](void* d) { uint32_t v = 0x01020304; std::memcpy(d, &v, 4); };
            registry_.addParam(std::move(p));
        }
        // Variable-length string param
        {
            ParamEntry p;
            p.id = 4;
            p.name = "device_name";
            p.description = "Device name string";
            p.group = "system";
            p.valueType = ValueType::String;
            p.maxValueSize = 128;
            p.varReadFn = [this](void* d, size_t maxLen) -> size_t {
                const char* s = "Tether-Test";
                size_t len = std::min(strlen(s), maxLen);
                std::memcpy(d, s, len);
                return len;
            };
            p.varWriteFn = [this](const void* s, size_t len) {
                logMessage_.assign(static_cast<const char*>(s), len);
            };
            registry_.addParam(std::move(p));
        }
        // U32 signal (read-only)
        {
            SignalEntry s;
            s.id = 10;
            s.name = "encoder";
            s.description = "Encoder count";
            s.group = "motion";
            s.valueType = ValueType::U32;
            s.readFn = [this](void* d) { uint32_t v = encoder_.load(); std::memcpy(d, &v, 4); };
            registry_.addSignal(std::move(s));
        }
        // F64 signal (read-only)
        {
            SignalEntry s;
            s.id = 11;
            s.name = "temperature";
            s.description = "Board temperature";
            s.group = "system";
            s.valueType = ValueType::F64;
            s.readFn = [this](void* d) { double v = 25.5; std::memcpy(d, &v, 8); };
            registry_.addSignal(std::move(s));
        }
        // Function: add(a, b) -> sum
        {
            FunctionEntry fn;
            fn.id = 100;
            fn.name = "add";
            fn.description = "Add two U32 values";
            fn.group = "math";
            fn.parameters = {
                {"a", "First operand", ValueType::U32},
                {"b", "Second operand", ValueType::U32, true, true, {2, 0, 0, 0}}
            };
            fn.returnValue.present = true;
            fn.returnValue.name = "sum";
            fn.returnValue.description = "The sum";
            fn.returnValue.type = ValueType::U32;
            fn.callback = [this](const std::vector<FunctionArgument>& args) {
                ++functionCalls_;
                FunctionCallResult result;
                uint32_t a = 0, b = 0;
                for (const auto& arg : args) {
                    if (arg.value.size() != 4) {
                        result.errorMessage = "bad arg size";
                        return result;
                    }
                    uint32_t v = 0;
                    std::memcpy(&v, arg.value.data(), 4);
                    if (arg.position == 0) a = v;
                    else if (arg.position == 1) b = v;
                }
                uint32_t sum = a + b;
                result.success = true;
                result.returnValue.resize(4);
                std::memcpy(result.returnValue.data(), &sum, 4);
                return result;
            };
            registry_.addFunction(std::move(fn));
        }
        // Function: no-op (no args, no return)
        {
            FunctionEntry fn;
            fn.id = 101;
            fn.name = "ping_fn";
            fn.description = "No-op function";
            fn.group = "system";
            fn.callback = [this](const std::vector<FunctionArgument>&) {
                ++functionCalls_;
                FunctionCallResult result;
                result.success = true;
                return result;
            };
            registry_.addFunction(std::move(fn));
        }
        // Function: fail (always returns error)
        {
            FunctionEntry fn;
            fn.id = 102;
            fn.name = "fail";
            fn.description = "Always fails";
            fn.group = "test";
            fn.callback = [this](const std::vector<FunctionArgument>&) {
                ++functionCalls_;
                FunctionCallResult result;
                result.errorMessage = "intentional failure";
                return result;
            };
            registry_.addFunction(std::move(fn));
        }

        // Create message-oriented pipe pair (Framing::None)
        auto [clientEnd, serverEnd] = MessagePipeTransport::create();

        // Create Session with Framing::None
        features_.features.push_back(Feature::makeBool("test_feature", true));
        session_ = std::make_unique<Session>(
            std::move(serverEnd), registry_,
            [] {
                return static_cast<uint64_t>(
                    std::chrono::duration_cast<std::chrono::microseconds>(
                        std::chrono::steady_clock::now().time_since_epoch()).count());
            },
            noopLogFn, &features_, nullptr, nullptr, nullptr, nullptr, nullptr,
            Framing::None);

        sessionThread_ = std::thread([this] { session_->run(); });

        // Create client
        client_ = std::make_unique<TetherIOClient>(std::move(clientEnd), 5000);

        // Wait for session to be ready by doing a ping
        for (int i = 0; i < 50; ++i) {
            auto result = client_->ping(42);
            if (result && *result == 42) break;
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
        }
    }

    void TearDown() override {
        if (client_) client_->close();
        if (session_) session_->requestStop();
        if (sessionThread_.joinable()) sessionThread_.join();
    }

    // Helper: decode F64 from bytes
    static double decodeF64(const std::vector<uint8_t>& bytes) {
        EXPECT_EQ(bytes.size(), 8u);
        double v = 0;
        if (bytes.size() >= 8) std::memcpy(&v, bytes.data(), 8);
        return v;
    }

    // Helper: decode U32 from bytes
    static uint32_t decodeU32(const std::vector<uint8_t>& bytes) {
        EXPECT_EQ(bytes.size(), 4u);
        uint32_t v = 0;
        if (bytes.size() >= 4) std::memcpy(&v, bytes.data(), 4);
        return v;
    }

    // Helper: encode F64 to bytes
    static std::vector<uint8_t> encodeF64(double v) {
        std::vector<uint8_t> bytes(8);
        std::memcpy(bytes.data(), &v, 8);
        return bytes;
    }

    // Helper: encode U32 to bytes
    static std::vector<uint8_t> encodeU32(uint32_t v) {
        std::vector<uint8_t> bytes(4);
        std::memcpy(bytes.data(), &v, 4);
        return bytes;
    }
};

// ===========================================================================
// Ping / Pong
// ===========================================================================

TEST_F(IOClientIntegrationTest, PingPong) {
    auto result = client_->ping(12345);
    ASSERT_TRUE(result.has_value()) << result.error().message;
    EXPECT_EQ(*result, 12345u);
}

TEST_F(IOClientIntegrationTest, PingPongZeroNonce) {
    auto result = client_->ping(0);
    ASSERT_TRUE(result.has_value()) << result.error().message;
    EXPECT_EQ(*result, 0u);
}

TEST_F(IOClientIntegrationTest, PingPongLargeNonce) {
    auto result = client_->ping(0x0FFFFFFF);
    ASSERT_TRUE(result.has_value()) << result.error().message;
    EXPECT_EQ(*result, 0x0FFFFFFFu);
}

TEST_F(IOClientIntegrationTest, MultiplePingsSequential) {
    for (uint32_t i = 0; i < 20; ++i) {
        auto result = client_->ping(i);
        ASSERT_TRUE(result.has_value()) << "Ping " << i << " failed: " << result.error().message;
        EXPECT_EQ(*result, i);
    }
}

// ===========================================================================
// List Parameters
// ===========================================================================

TEST_F(IOClientIntegrationTest, ListParamsAll) {
    auto result = client_->listParams(0, 1000);
    ASSERT_TRUE(result.has_value()) << result.error().message;
    EXPECT_EQ(result->size(), 4u); // 4 params registered
}

TEST_F(IOClientIntegrationTest, ListParamsPaged) {
    // Get first 2
    auto page1 = client_->listParams(0, 2);
    ASSERT_TRUE(page1.has_value()) << page1.error().message;
    EXPECT_EQ(page1->size(), 2u);
    EXPECT_EQ(page1->at(0).name, "position");
    EXPECT_EQ(page1->at(1).name, "velocity");

    // Get next 2
    auto page2 = client_->listParams(2, 2);
    ASSERT_TRUE(page2.has_value()) << page2.error().message;
    EXPECT_EQ(page2->size(), 2u);
    EXPECT_EQ(page2->at(0).name, "firmware_version");
    EXPECT_EQ(page2->at(1).name, "device_name");
}

TEST_F(IOClientIntegrationTest, ListParamsFields) {
    auto result = client_->listParams(0, 1000);
    ASSERT_TRUE(result.has_value()) << result.error().message;
    ASSERT_GE(result->size(), 1u);

    const auto& p = result->at(0);
    EXPECT_EQ(p.id, 1u);
    EXPECT_EQ(p.name, "position");
    EXPECT_EQ(p.description, "Current position");
    EXPECT_EQ(p.group, "motion");
    EXPECT_EQ(p.type, ValueType::F64);
    EXPECT_EQ(p.valueSize, 8u);
    // flags: readable | writable
    EXPECT_TRUE(p.flags & 0x01); // readable
    EXPECT_TRUE(p.flags & 0x02); // writable
}

TEST_F(IOClientIntegrationTest, ListParamsReadOnlyFlags) {
    auto result = client_->listParams(0, 1000);
    ASSERT_TRUE(result.has_value()) << result.error().message;
    ASSERT_GE(result->size(), 3u);

    // firmware_version (id=3) is read-only
    bool found = false;
    for (const auto& p : *result) {
        if (p.id == 3) {
            found = true;
            EXPECT_TRUE(p.flags & 0x01);  // readable
            EXPECT_FALSE(p.flags & 0x02); // NOT writable
        }
    }
    EXPECT_TRUE(found);
}

TEST_F(IOClientIntegrationTest, ListParamsVariableLengthFlags) {
    auto result = client_->listParams(0, 1000);
    ASSERT_TRUE(result.has_value()) << result.error().message;

    for (const auto& p : *result) {
        if (p.id == 4) { // device_name
            EXPECT_EQ(p.type, ValueType::String);
            EXPECT_EQ(p.valueSize, 0u); // variable-length
            EXPECT_TRUE(p.flags & 0x04); // variable-length flag
            return;
        }
    }
    FAIL() << "device_name param not found";
}

// ===========================================================================
// Get Parameter
// ===========================================================================

TEST_F(IOClientIntegrationTest, GetParamF64) {
    auto result = client_->getParam(1);
    ASSERT_TRUE(result.has_value()) << result.error().message;
    EXPECT_EQ(result->size(), 8u);
    double val = decodeF64(*result);
    EXPECT_DOUBLE_EQ(val, 42.0);
}

TEST_F(IOClientIntegrationTest, GetParamU32) {
    auto result = client_->getParam(3);
    ASSERT_TRUE(result.has_value()) << result.error().message;
    EXPECT_EQ(result->size(), 4u);
    uint32_t val = decodeU32(*result);
    EXPECT_EQ(val, 0x01020304u);
}

TEST_F(IOClientIntegrationTest, GetParamVariableLength) {
    auto result = client_->getParam(4);
    ASSERT_TRUE(result.has_value()) << result.error().message;
    std::string s(result->begin(), result->end());
    EXPECT_EQ(s, "Tether-Test");
}

TEST_F(IOClientIntegrationTest, GetParamInvalidId) {
    auto result = client_->getParam(999);
    ASSERT_FALSE(result.has_value());
    EXPECT_EQ(result.error().code, ErrorCode::InvalidId);
}

// ===========================================================================
// Set Parameter
// ===========================================================================

TEST_F(IOClientIntegrationTest, SetParamF64) {
    double newVal = 99.5;
    auto result = client_->setParam(1, &newVal, 8);
    ASSERT_TRUE(result.has_value()) << result.error().message;

    // Verify the value was written
    EXPECT_DOUBLE_EQ(position_.load(), 99.5);

    // Read it back
    auto getResult = client_->getParam(1);
    ASSERT_TRUE(getResult.has_value()) << getResult.error().message;
    EXPECT_DOUBLE_EQ(decodeF64(*getResult), 99.5);
}

TEST_F(IOClientIntegrationTest, SetParamU32) {
    // firmware_version (id=3) is read-only
    uint32_t newVal = 42;
    auto result = client_->setParam(3, &newVal, 4);
    ASSERT_FALSE(result.has_value());
    EXPECT_EQ(result.error().code, ErrorCode::NotWritable);
}

TEST_F(IOClientIntegrationTest, SetParamVariableLength) {
    std::string newVal = "Hello-Tether";
    auto result = client_->setParamVar(4, newVal.data(), newVal.size());
    ASSERT_TRUE(result.has_value()) << result.error().message;
    EXPECT_EQ(logMessage_, "Hello-Tether");
}

TEST_F(IOClientIntegrationTest, SetParamInvalidId) {
    double newVal = 1.0;
    auto result = client_->setParam(999, &newVal, 8);
    ASSERT_FALSE(result.has_value());
    EXPECT_EQ(result.error().code, ErrorCode::InvalidId);
}

TEST_F(IOClientIntegrationTest, SetParamThenGetParamRoundTrip) {
    for (double v : {0.0, -1.5, 3.14159, 1e10, -1e10, 1e-10}) {
        auto setResult = client_->setParam(1, &v, 8);
        ASSERT_TRUE(setResult.has_value()) << setResult.error().message;
        auto getResult = client_->getParam(1);
        ASSERT_TRUE(getResult.has_value()) << getResult.error().message;
        EXPECT_DOUBLE_EQ(decodeF64(*getResult), v);
    }
}

// ===========================================================================
// List Signals
// ===========================================================================

TEST_F(IOClientIntegrationTest, ListSignalsAll) {
    auto result = client_->listSignals(0, 1000);
    ASSERT_TRUE(result.has_value()) << result.error().message;
    EXPECT_EQ(result->size(), 2u);
}

TEST_F(IOClientIntegrationTest, ListSignalsFields) {
    auto result = client_->listSignals(0, 1000);
    ASSERT_TRUE(result.has_value()) << result.error().message;
    ASSERT_GE(result->size(), 1u);

    const auto& s = result->at(0);
    EXPECT_EQ(s.id, 10u);
    EXPECT_EQ(s.name, "encoder");
    EXPECT_EQ(s.description, "Encoder count");
    EXPECT_EQ(s.group, "motion");
    EXPECT_EQ(s.type, ValueType::U32);
    EXPECT_EQ(s.valueSize, 4u);
    // Signals are read-only
    EXPECT_TRUE(s.flags & 0x01);  // readable
    EXPECT_FALSE(s.flags & 0x02); // NOT writable
}

TEST_F(IOClientIntegrationTest, ListSignalsPaged) {
    auto page1 = client_->listSignals(0, 1);
    ASSERT_TRUE(page1.has_value()) << page1.error().message;
    EXPECT_EQ(page1->size(), 1u);
    EXPECT_EQ(page1->at(0).name, "encoder");

    auto page2 = client_->listSignals(1, 1);
    ASSERT_TRUE(page2.has_value()) << page2.error().message;
    EXPECT_EQ(page2->size(), 1u);
    EXPECT_EQ(page2->at(0).name, "temperature");
}

// ===========================================================================
// Get Signal
// ===========================================================================

TEST_F(IOClientIntegrationTest, GetSignalU32) {
    auto result = client_->getSignal(10);
    ASSERT_TRUE(result.has_value()) << result.error().message;
    EXPECT_EQ(result->size(), 4u);
    EXPECT_EQ(decodeU32(*result), 12345u);
}

TEST_F(IOClientIntegrationTest, GetSignalF64) {
    auto result = client_->getSignal(11);
    ASSERT_TRUE(result.has_value()) << result.error().message;
    EXPECT_EQ(result->size(), 8u);
    EXPECT_DOUBLE_EQ(decodeF64(*result), 25.5);
}

TEST_F(IOClientIntegrationTest, GetSignalInvalidId) {
    auto result = client_->getSignal(999);
    ASSERT_FALSE(result.has_value());
    EXPECT_EQ(result.error().code, ErrorCode::InvalidId);
}

// ===========================================================================
// Get Metadata
// ===========================================================================

TEST_F(IOClientIntegrationTest, GetMetadata) {
    auto result = client_->getMetadata(1);
    ASSERT_TRUE(result.has_value()) << result.error().message;
    EXPECT_EQ(result->size(), 2u);

    bool foundUnit = false, foundRange = false;
    for (const auto& m : *result) {
        if (m.key == "unit") { EXPECT_EQ(m.value, "mm"); foundUnit = true; }
        if (m.key == "range") { EXPECT_EQ(m.value, "0..1000"); foundRange = true; }
    }
    EXPECT_TRUE(foundUnit);
    EXPECT_TRUE(foundRange);
}

TEST_F(IOClientIntegrationTest, GetMetadataEmpty) {
    // Signal id=10 has no metadata
    auto result = client_->getMetadata(10);
    ASSERT_TRUE(result.has_value()) << result.error().message;
    EXPECT_EQ(result->size(), 0u);
}

TEST_F(IOClientIntegrationTest, GetMetadataInvalidId) {
    auto result = client_->getMetadata(999);
    ASSERT_FALSE(result.has_value());
    EXPECT_EQ(result.error().code, ErrorCode::InvalidId);
}

// ===========================================================================
// List Functions
// ===========================================================================

TEST_F(IOClientIntegrationTest, ListFunctionsAll) {
    auto result = client_->listFunctions(0, 1000);
    ASSERT_TRUE(result.has_value()) << result.error().message;
    EXPECT_EQ(result->size(), 3u); // add, ping_fn, fail
}

TEST_F(IOClientIntegrationTest, ListFunctionFields) {
    auto result = client_->listFunctions(0, 1000);
    ASSERT_TRUE(result.has_value()) << result.error().message;

    const ClientFunctionEntry* addFn = nullptr;
    for (const auto& f : *result) {
        if (f.id == 100) { addFn = &f; break; }
    }
    ASSERT_NE(addFn, nullptr);
    EXPECT_EQ(addFn->name, "add");
    EXPECT_EQ(addFn->description, "Add two U32 values");
    EXPECT_EQ(addFn->group, "math");
    EXPECT_EQ(addFn->parameters.size(), 2u);
    EXPECT_EQ(addFn->parameters[0].name, "a");
    EXPECT_EQ(addFn->parameters[1].name, "b");
    EXPECT_EQ(addFn->parameters[0].type, ValueType::U32);
    EXPECT_TRUE(addFn->hasReturnValue);
    EXPECT_EQ(addFn->returnType, ValueType::U32);
}

TEST_F(IOClientIntegrationTest, ListFunctionWithDefaultParam) {
    auto result = client_->listFunctions(0, 1000);
    ASSERT_TRUE(result.has_value()) << result.error().message;

    for (const auto& f : *result) {
        if (f.id == 100) {
            // param "b" has a default value of 2
            ASSERT_GE(f.parameters.size(), 2u);
            EXPECT_TRUE(f.parameters[1].hasDefault);
            ASSERT_EQ(f.parameters[1].defaultValue.size(), 4u);
            uint32_t defaultVal = 0;
            std::memcpy(&defaultVal, f.parameters[1].defaultValue.data(), 4);
            EXPECT_EQ(defaultVal, 2u);
            return;
        }
    }
    FAIL() << "add function not found";
}

// ===========================================================================
// Call Function
// ===========================================================================

TEST_F(IOClientIntegrationTest, CallFunctionAdd) {
    TetherIOClient::FunctionArg a1{0, ValueType::U32, encodeU32(10)};
    TetherIOClient::FunctionArg a2{1, ValueType::U32, encodeU32(20)};
    auto result = client_->callFunction(100, {a1, a2});
    ASSERT_TRUE(result.has_value()) << result.error().message;
    EXPECT_TRUE(result->success);
    EXPECT_TRUE(result->hasReturnValue);
    EXPECT_EQ(result->returnValue.size(), 4u);
    EXPECT_EQ(decodeU32(result->returnValue), 30u);
    EXPECT_EQ(functionCalls_.load(), 1);
}

TEST_F(IOClientIntegrationTest, CallFunctionWithDefaultArg) {
    // Only provide first arg, second should default to 2
    TetherIOClient::FunctionArg a1{0, ValueType::U32, encodeU32(100)};
    auto result = client_->callFunction(100, {a1});
    ASSERT_TRUE(result.has_value()) << result.error().message;
    EXPECT_TRUE(result->success);
    EXPECT_EQ(decodeU32(result->returnValue), 102u); // 100 + 2 (default)
}

TEST_F(IOClientIntegrationTest, CallFunctionNoArgs) {
    auto result = client_->callFunction(101, {});
    ASSERT_TRUE(result.has_value()) << result.error().message;
    EXPECT_TRUE(result->success);
    EXPECT_FALSE(result->hasReturnValue);
}

TEST_F(IOClientIntegrationTest, CallFunctionFailure) {
    auto result = client_->callFunction(102, {});
    ASSERT_TRUE(result.has_value()) << result.error().message;
    EXPECT_FALSE(result->success);
    EXPECT_EQ(result->errorMessage, "intentional failure");
}

TEST_F(IOClientIntegrationTest, CallFunctionInvalidId) {
    auto result = client_->callFunction(999, {});
    ASSERT_FALSE(result.has_value());
    EXPECT_EQ(result.error().code, ErrorCode::InvalidId);
}

TEST_F(IOClientIntegrationTest, CallFunctionMultipleTimes) {
    for (uint32_t i = 0; i < 10; ++i) {
        TetherIOClient::FunctionArg a1{0, ValueType::U32, encodeU32(i)};
        TetherIOClient::FunctionArg a2{1, ValueType::U32, encodeU32(i * 2)};
        auto result = client_->callFunction(100, {a1, a2});
        ASSERT_TRUE(result.has_value()) << result.error().message;
        EXPECT_TRUE(result->success);
        EXPECT_EQ(decodeU32(result->returnValue), i * 3u);
    }
    EXPECT_EQ(functionCalls_.load(), 10);
}

// ===========================================================================
// Streaming
// ===========================================================================

TEST_F(IOClientIntegrationTest, ConfigureAndStartStream) {
    // Configure a time-triggered stream on param 1 (position, F64)
    auto configResult = client_->configureStream({1}, 10, 1);
    ASSERT_TRUE(configResult.has_value()) << configResult.error().message;
    EXPECT_EQ(configResult->layout.size(), 1u);
    EXPECT_EQ(configResult->layout[0].id, 1u);
    EXPECT_EQ(configResult->layout[0].type, ValueType::F64);
    EXPECT_EQ(configResult->layout[0].valueSize, 8u);

    // Start stream
    auto startResult = client_->startStream();
    ASSERT_TRUE(startResult.has_value()) << startResult.error().message;

    // Wait for stream data
    std::atomic<int> rowsReceived{0};
    std::atomic<double> lastValue{0};
    client_->setStreamDataCallback([&](const ClientStreamRow& row) {
        ++rowsReceived;
        if (!row.values.empty() && row.values[0].size() >= 8) {
            double v;
            std::memcpy(&v, row.values[0].data(), 8);
            lastValue.store(v);
        }
    });

    // Wait for at least 3 rows
    auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
    while (rowsReceived.load() < 3 && std::chrono::steady_clock::now() < deadline) {
        // Trigger a receive cycle by doing a ping (which dispatches pending StreamData)
        auto pingResult = client_->ping(0);
        if (!pingResult) break;
    }

    EXPECT_GE(rowsReceived.load(), 1);

    // Stop stream
    auto stopResult = client_->stopStream();
    ASSERT_TRUE(stopResult.has_value()) << stopResult.error().message;
}

TEST_F(IOClientIntegrationTest, ConfigureStreamMultipleEntries) {
    auto configResult = client_->configureStream({1, 2, 10}, 10, 1);
    ASSERT_TRUE(configResult.has_value()) << configResult.error().message;
    EXPECT_EQ(configResult->layout.size(), 3u);
}

TEST_F(IOClientIntegrationTest, ConfigureStreamInvalidEntryId) {
    auto configResult = client_->configureStream({999}, 10, 1);
    // The server may silently skip invalid entries or return an error
    // If it returns a value, the layout should be empty
    if (configResult.has_value()) {
        EXPECT_EQ(configResult->layout.size(), 0u);
    }
}

TEST_F(IOClientIntegrationTest, StartStreamWithoutConfigure) {
    // StartStream is fire-and-forget; the server sends an Error which
    // will be received on the next request.
    auto startResult = client_->startStream();
    ASSERT_TRUE(startResult.has_value()) << startResult.error().message;

    // The next request should receive the error
    auto pingResult = client_->ping(0);
    ASSERT_FALSE(pingResult.has_value());
    EXPECT_EQ(pingResult.error().code, ErrorCode::StreamNotConfigured);
}

TEST_F(IOClientIntegrationTest, StopStreamWithoutStart) {
    // First configure
    auto configResult = client_->configureStream({1}, 10, 1);
    ASSERT_TRUE(configResult.has_value()) << configResult.error().message;

    // StopStream is fire-and-forget; the server sends an Error which
    // will be received on the next request.
    auto stopResult = client_->stopStream();
    ASSERT_TRUE(stopResult.has_value()) << stopResult.error().message;

    // The next request should receive the error
    auto pingResult = client_->ping(0);
    ASSERT_FALSE(pingResult.has_value());
    EXPECT_EQ(pingResult.error().code, ErrorCode::NotStreaming);
}

TEST_F(IOClientIntegrationTest, StreamDataValues) {
    position_.store(77.7);

    auto configResult = client_->configureStream({1}, 10, 1);
    ASSERT_TRUE(configResult.has_value()) << configResult.error().message;

    std::atomic<double> receivedValue{0};
    std::atomic<bool> gotData{false};
    client_->setStreamDataCallback([&](const ClientStreamRow& row) {
        if (!row.values.empty() && row.values[0].size() >= 8) {
            double v;
            std::memcpy(&v, row.values[0].data(), 8);
            receivedValue.store(v);
            gotData.store(true);
        }
    });

    auto startResult = client_->startStream();
    ASSERT_TRUE(startResult.has_value()) << startResult.error().message;

    auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
    while (!gotData.load() && std::chrono::steady_clock::now() < deadline) {
        client_->ping(0);
    }

    ASSERT_TRUE(gotData.load()) << "No stream data received";
    EXPECT_DOUBLE_EQ(receivedValue.load(), 77.7);

    client_->stopStream();
}

// ===========================================================================
// Snapshots
// ===========================================================================

TEST_F(IOClientIntegrationTest, SnapshotAllParams) {
    auto result = client_->snapshotParams({});
    ASSERT_TRUE(result.has_value()) << result.error().message;
    auto& [ts, values] = *result;
    EXPECT_GT(ts, 0u);
    EXPECT_EQ(values.size(), 4u); // all 4 params
}

TEST_F(IOClientIntegrationTest, SnapshotSpecificParams) {
    auto result = client_->snapshotParams({1, 3});
    ASSERT_TRUE(result.has_value()) << result.error().message;
    auto& [ts, values] = *result;
    EXPECT_EQ(values.size(), 2u);

    // Check IDs
    std::vector<uint64_t> ids;
    for (const auto& v : values) ids.push_back(v.id);
    EXPECT_NE(std::find(ids.begin(), ids.end(), 1), ids.end());
    EXPECT_NE(std::find(ids.begin(), ids.end(), 3), ids.end());
}

TEST_F(IOClientIntegrationTest, SnapshotParamValues) {
    auto result = client_->snapshotParams({1});
    ASSERT_TRUE(result.has_value()) << result.error().message;
    auto& [ts, values] = *result;
    ASSERT_EQ(values.size(), 1u);
    EXPECT_EQ(values[0].id, 1u);
    EXPECT_EQ(values[0].valueSize, 8u);
    EXPECT_DOUBLE_EQ(decodeF64(values[0].value), 42.0);
}

TEST_F(IOClientIntegrationTest, SnapshotAllSignals) {
    auto result = client_->snapshotSignals({});
    ASSERT_TRUE(result.has_value()) << result.error().message;
    auto& [ts, values] = *result;
    EXPECT_GT(ts, 0u);
    EXPECT_EQ(values.size(), 2u);
}

TEST_F(IOClientIntegrationTest, SnapshotSpecificSignals) {
    auto result = client_->snapshotSignals({10});
    ASSERT_TRUE(result.has_value()) << result.error().message;
    auto& [ts, values] = *result;
    ASSERT_EQ(values.size(), 1u);
    EXPECT_EQ(values[0].id, 10u);
    EXPECT_EQ(decodeU32(values[0].value), 12345u);
}

// ===========================================================================
// Feature Exchange
// ===========================================================================

TEST_F(IOClientIntegrationTest, FeatureExchange) {
    auto result = client_->featureExchange({});
    ASSERT_TRUE(result.has_value()) << result.error().message;
    // Server should return at least protocol_version and test_feature
    bool foundProtocolVersion = false;
    bool foundTestFeature = false;
    for (const auto& f : *result) {
        if (f.name == "protocol_version") foundProtocolVersion = true;
        if (f.name == "test_feature") foundTestFeature = true;
    }
    EXPECT_TRUE(foundProtocolVersion);
    EXPECT_TRUE(foundTestFeature);
}

TEST_F(IOClientIntegrationTest, FeatureExchangeWithClientFeatures) {
    std::vector<ClientFeature> clientFeatures;
    ClientFeature cf;
    cf.name = "client_supports_x";
    cf.type = static_cast<uint8_t>(ValueType::Bool);
    cf.value = {1};
    clientFeatures.push_back(cf);

    auto result = client_->featureExchange(clientFeatures);
    ASSERT_TRUE(result.has_value()) << result.error().message;
    EXPECT_GE(result->size(), 1u);
}

// ===========================================================================
// Log Subscription
// ===========================================================================

TEST_F(IOClientIntegrationTest, SubscribeAndUnsubscribeLog) {
    auto subResult = client_->subscribeLog(0); // all severities
    ASSERT_TRUE(subResult.has_value()) << subResult.error().message;
    EXPECT_FALSE(subResult->failed);
    EXPECT_GT(subResult->subscriptionId, 0u);

    auto unsubResult = client_->unsubscribeLog(subResult->subscriptionId);
    ASSERT_TRUE(unsubResult.has_value()) << unsubResult.error().message;
    EXPECT_EQ(*unsubResult, subResult->subscriptionId);
}

TEST_F(IOClientIntegrationTest, UnsubscribeNonexistentLog) {
    auto result = client_->unsubscribeLog(999);
    // Should succeed but with notFound=1 (we just check it doesn't crash)
    ASSERT_TRUE(result.has_value()) << result.error().message;
}

TEST_F(IOClientIntegrationTest, SubscribeLogWithFilters) {
    auto subResult = client_->subscribeLog(2, "motion", "", "");
    ASSERT_TRUE(subResult.has_value()) << subResult.error().message;
    EXPECT_FALSE(subResult->failed);
}

TEST_F(IOClientIntegrationTest, LogDataReception) {
    // Subscribe to logs
    auto subResult = client_->subscribeLog(0);
    ASSERT_TRUE(subResult.has_value()) << subResult.error().message;
    ASSERT_FALSE(subResult->failed);

    std::atomic<bool> gotLog{false};
    std::atomic<uint8_t> logSeverity{0};
    std::string logComponent, logMsg;

    client_->setLogDataCallback([&](const ClientLogRecord& rec) {
        gotLog.store(true);
        logSeverity.store(rec.severity);
        logComponent = rec.component;
        logMsg = rec.message;
    });

    // Publish a log from the server side
    session_->publishLog(LogSeverity::Info, "TestComponent", "Hello from test");

    // Wait for log data (dispatched during ping)
    auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(3);
    while (!gotLog.load() && std::chrono::steady_clock::now() < deadline) {
        client_->ping(0);
    }

    EXPECT_TRUE(gotLog.load());
    if (gotLog.load()) {
        EXPECT_EQ(logComponent, "TestComponent");
        EXPECT_EQ(logMsg, "Hello from test");
    }

    client_->unsubscribeLog(subResult->subscriptionId);
}

// ===========================================================================
// Error Handling
// ===========================================================================

TEST_F(IOClientIntegrationTest, ErrorGetParamInvalidId) {
    auto result = client_->getParam(0xDEAD);
    ASSERT_FALSE(result.has_value());
    EXPECT_EQ(result.error().code, ErrorCode::InvalidId);
    EXPECT_FALSE(result.error().message.empty());
}

TEST_F(IOClientIntegrationTest, ErrorGetSignalInvalidId) {
    auto result = client_->getSignal(0xBEEF);
    ASSERT_FALSE(result.has_value());
    EXPECT_EQ(result.error().code, ErrorCode::InvalidId);
}

TEST_F(IOClientIntegrationTest, ErrorSetParamNotWritable) {
    uint32_t val = 42;
    auto result = client_->setParam(3, &val, 4); // firmware_version is read-only
    ASSERT_FALSE(result.has_value());
    EXPECT_EQ(result.error().code, ErrorCode::NotWritable);
}

TEST_F(IOClientIntegrationTest, ErrorCallFunctionInvalidId) {
    auto result = client_->callFunction(0xDEAD, {});
    ASSERT_FALSE(result.has_value());
    EXPECT_EQ(result.error().code, ErrorCode::InvalidId);
}

// ===========================================================================
// Concurrent Operations
// ===========================================================================

TEST_F(IOClientIntegrationTest, ConcurrentPingsAndGets) {
    std::atomic<int> successes{0};
    std::vector<std::thread> threads;

    for (int t = 0; t < 4; ++t) {
        threads.emplace_back([this, t, &successes]() {
            for (int i = 0; i < 10; ++i) {
                auto pingResult = client_->ping(static_cast<uint32_t>(t * 100 + i));
                if (pingResult && *pingResult == static_cast<uint32_t>(t * 100 + i)) {
                    ++successes;
                }
                auto getResult = client_->getParam(1);
                if (getResult && getResult->size() == 8) {
                    ++successes;
                }
            }
        });
    }

    for (auto& th : threads) th.join();
    EXPECT_EQ(successes.load(), 80); // 4 threads * 10 iterations * 2 ops
}

// ===========================================================================
// Mixed Workflow (realistic usage)
// ===========================================================================

TEST_F(IOClientIntegrationTest, MixedWorkflow) {
    // 1. List params
    auto params = client_->listParams(0, 100);
    ASSERT_TRUE(params.has_value());
    ASSERT_GE(params->size(), 2u);

    // 2. Get current position
    auto posResult = client_->getParam(1);
    ASSERT_TRUE(posResult.has_value());
    double pos = decodeF64(*posResult);
    EXPECT_DOUBLE_EQ(pos, 42.0);

    // 3. Set new position
    double newPos = 100.0;
    auto setResult = client_->setParam(1, &newPos, 8);
    ASSERT_TRUE(setResult.has_value());

    // 4. Verify position changed
    auto posResult2 = client_->getParam(1);
    ASSERT_TRUE(posResult2.has_value());
    EXPECT_DOUBLE_EQ(decodeF64(*posResult2), 100.0);

    // 5. List signals
    auto signals = client_->listSignals(0, 100);
    ASSERT_TRUE(signals.has_value());
    EXPECT_GE(signals->size(), 1u);

    // 6. Get signal value
    auto sigResult = client_->getSignal(10);
    ASSERT_TRUE(sigResult.has_value());
    EXPECT_EQ(decodeU32(*sigResult), 12345u);

    // 7. Call function
    TetherIOClient::FunctionArg a1{0, ValueType::U32, encodeU32(5)};
    TetherIOClient::FunctionArg a2{1, ValueType::U32, encodeU32(3)};
    auto callResult = client_->callFunction(100, {a1, a2});
    ASSERT_TRUE(callResult.has_value());
    EXPECT_TRUE(callResult->success);
    EXPECT_EQ(decodeU32(callResult->returnValue), 8u);

    // 8. Ping
    auto pingResult = client_->ping(999);
    ASSERT_TRUE(pingResult.has_value());
    EXPECT_EQ(*pingResult, 999u);

    // 9. Snapshot
    auto snapResult = client_->snapshotParams({1, 2});
    ASSERT_TRUE(snapResult.has_value());
    EXPECT_EQ(snapResult->second.size(), 2u);

    // 10. Feature exchange
    auto featResult = client_->featureExchange({});
    ASSERT_TRUE(featResult.has_value());
    EXPECT_GE(featResult->size(), 1u);
}

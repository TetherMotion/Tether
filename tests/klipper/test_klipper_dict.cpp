/**
 * @file test_klipper_dict.cpp
 * @brief DataDictionary, IdentifyProtocol, and CommandTable tests.
 */

#include <gtest/gtest.h>
#include "tether/klipper/protocol/DataDictionary.hpp"
#include "tether/klipper/protocol/IdentifyProtocol.hpp"
#include "tether/klipper/protocol/CommandTable.hpp"
#include "tether/klipper/protocol/Constants.hpp"

#include <vector>

using namespace tether::klipper::protocol;

// ============================================================================
// DataDictionary
// ============================================================================

TEST(KlipperDataDictionary, AddAndLookupCommand) {
    DataDictionary dict;
    uint16_t msgid = dict.addCommand("get_clock");
    EXPECT_GT(msgid, 0u);
    auto found = dict.lookupCommand("get_clock");
    ASSERT_TRUE(found.has_value());
    EXPECT_EQ(*found, msgid);
    auto entry = dict.lookupMsgid(msgid);
    ASSERT_NE(entry, nullptr);
    EXPECT_EQ(entry->direction, MessageDirection::Command);
}

TEST(KlipperDataDictionary, AddAndLookupResponse) {
    DataDictionary dict;
    uint16_t msgid = dict.addResponse("clock_response clock=%u");
    EXPECT_GT(msgid, 0u);
    auto found = dict.lookupResponse("clock_response clock=%u");
    ASSERT_TRUE(found.has_value());
    EXPECT_EQ(*found, msgid);
}

TEST(KlipperDataDictionary, AddAndLookupOutput) {
    DataDictionary dict;
    uint16_t msgid = dict.addOutput("debug_output msg=%s");
    EXPECT_GT(msgid, 0u);
    auto found = dict.lookupOutput("debug_output msg=%s");
    ASSERT_TRUE(found.has_value());
    EXPECT_EQ(*found, msgid);
}

TEST(KlipperDataDictionary, DuplicateCommandReturnsZero) {
    DataDictionary dict;
    uint16_t id1 = dict.addCommand("test_cmd val=%u");
    ASSERT_GT(id1, 0u);
    uint16_t id2 = dict.addCommand("test_cmd val=%u");
    // Per API: addCommand returns 0 if format is already present
    EXPECT_EQ(id2, 0u);
    // But the original is still there
    auto found = dict.lookupCommand("test_cmd val=%u");
    ASSERT_TRUE(found.has_value());
    EXPECT_EQ(*found, id1);
}

TEST(KlipperDataDictionary, LookupNonexistent) {
    DataDictionary dict;
    auto found = dict.lookupCommand("nonexistent");
    EXPECT_FALSE(found.has_value());
    auto entry = dict.lookupMsgid(999);
    EXPECT_EQ(entry, nullptr);
}

TEST(KlipperDataDictionary, SequentialMsgIds) {
    DataDictionary dict;
    uint16_t id1 = dict.addCommand("cmd1");
    uint16_t id2 = dict.addCommand("cmd2");
    uint16_t id3 = dict.addResponse("resp1");
    EXPECT_EQ(id2, id1 + 1);
    EXPECT_EQ(id3, id2 + 1);
}

TEST(KlipperDataDictionary, FirstDynamicMsgId) {
    DataDictionary dict;
    uint16_t id = dict.addCommand("first");
    EXPECT_GE(id, kFirstDynamicMsgId);
}

TEST(KlipperDataDictionary, JsonRoundTrip) {
    DataDictionary dict;
    dict.setVersion("1.0");
    dict.setApp("test_app");
    dict.setBuildVersions("build1");
    dict.addCommand("test_cmd val=%u");
    dict.addResponse("test_resp result=%u");
    dict.addConstant("MY_CONST", 42);
    dict.addConstantString("MY_STR", "hello");

    std::string json = dict.toJson();
    ASSERT_FALSE(json.empty());

    DataDictionary dict2;
    ASSERT_TRUE(dict2.fromJson(json));
    EXPECT_EQ(dict2.version(), "1.0");
    EXPECT_EQ(dict2.app(), "test_app");

    auto cmd = dict2.lookupCommand("test_cmd val=%u");
    ASSERT_TRUE(cmd.has_value());
    auto resp = dict2.lookupResponse("test_resp result=%u");
    ASSERT_TRUE(resp.has_value());
}

TEST(KlipperDataDictionary, WireBlobRoundTrip) {
    DataDictionary dict;
    dict.addCommand("get_clock");
    dict.addResponse("clock_response clock=%u");
    dict.addConstant("CLOCK_FREQ", 180000000);

    auto wire = dict.toWire();
    ASSERT_FALSE(wire.empty());

    auto json = DataDictionary::fromWire(wire);
    ASSERT_FALSE(json.empty());

    DataDictionary dict2;
    ASSERT_TRUE(dict2.fromJson(json));
    auto cmd = dict2.lookupCommand("get_clock");
    ASSERT_TRUE(cmd.has_value());
}

TEST(KlipperDataDictionary, EnumValues) {
    DataDictionary dict;
    dict.addEnumValue("pins", "PA0", 0);
    dict.addEnumValue("pins", "PA1", 1);
    dict.addEnumValue("pins", "PA2", 2);

    auto val = dict.resolveEnum("pins", "PA1");
    ASSERT_TRUE(val.has_value());
    EXPECT_EQ(*val, 1u);

    auto val2 = dict.resolveEnum("pins", "PA0");
    ASSERT_TRUE(val2.has_value());
    EXPECT_EQ(*val2, 0u);
}

TEST(KlipperDataDictionary, EnumRange) {
    DataDictionary dict;
    dict.addEnumRange("pins", "PA", 0, 16); // PA0..PA15

    auto val = dict.resolveEnum("pins", "PA0");
    ASSERT_TRUE(val.has_value());
    EXPECT_EQ(*val, 0u);

    auto val15 = dict.resolveEnum("pins", "PA15");
    ASSERT_TRUE(val15.has_value());
    EXPECT_EQ(*val15, 15u);
}

TEST(KlipperDataDictionary, EnumLookupNonexistent) {
    DataDictionary dict;
    dict.addEnumValue("pins", "PA0", 0);
    auto val = dict.resolveEnum("pins", "PA99");
    EXPECT_FALSE(val.has_value());
    auto val2 = dict.resolveEnum("nonexistent", "x");
    EXPECT_FALSE(val2.has_value());
}

TEST(KlipperDataDictionary, Constants) {
    DataDictionary dict;
    dict.addConstant("INT_CONST", 12345);
    dict.addConstantString("STR_CONST", "test_value");

    auto intVal = dict.lookupConstant("INT_CONST");
    ASSERT_TRUE(intVal.has_value());
    auto* intPtr = std::get_if<int64_t>(&*intVal);
    ASSERT_NE(intPtr, nullptr);
    EXPECT_EQ(*intPtr, 12345);

    auto strVal = dict.lookupConstant("STR_CONST");
    ASSERT_TRUE(strVal.has_value());
    auto* strPtr = std::get_if<std::string>(&*strVal);
    ASSERT_NE(strPtr, nullptr);
    EXPECT_EQ(*strPtr, "test_value");
}

TEST(KlipperDataDictionary, LookupNonexistentConstant) {
    DataDictionary dict;
    auto val = dict.lookupConstant("NONEXISTENT");
    EXPECT_FALSE(val.has_value());
}

TEST(KlipperDataDictionary, NextMsgId) {
    DataDictionary dict;
    uint16_t initial = dict.nextMsgid();
    dict.addCommand("cmd1");
    EXPECT_EQ(dict.nextMsgid(), initial + 1);
    dict.addCommand("cmd2");
    EXPECT_EQ(dict.nextMsgid(), initial + 2);
}

TEST(KlipperDataDictionary, MessagesMap) {
    DataDictionary dict;
    dict.addCommand("cmd1");
    dict.addResponse("resp1");
    const auto& msgs = dict.messages();
    EXPECT_EQ(msgs.size(), 2u);
}

// ============================================================================
// IdentifyProtocol
// ============================================================================

TEST(KlipperIdentifyProtocol, ServerSize) {
    std::vector<uint8_t> blob(100, 0xAA);
    IdentifyServer server(blob);
    EXPECT_EQ(server.size(), 100u);
}

TEST(KlipperIdentifyProtocol, ServerChunkServing) {
    std::vector<uint8_t> blob(100, 0xBB);
    IdentifyServer server(blob);

    auto chunk = server.buildResponseContent(0, 40);
    ASSERT_FALSE(chunk.empty());
    // First 4 bytes should be offset (VLQ encoded)
    // Then up to 40 bytes of data
    EXPECT_LE(chunk.size(), 44u);
}

TEST(KlipperIdentifyProtocol, ServerChunkAtEnd) {
    std::vector<uint8_t> blob(50, 0xCC);
    IdentifyServer server(blob);

    // Request from offset 40, count 40 → only 10 bytes left
    auto chunk = server.buildResponseContent(40, 40);
    ASSERT_FALSE(chunk.empty());
    // Should contain offset + 10 bytes of data
}

TEST(KlipperIdentifyProtocol, ServerEmptyBlob) {
    std::vector<uint8_t> blob;
    IdentifyServer server(blob);
    EXPECT_EQ(server.size(), 0u);
    auto chunk = server.buildResponseContent(0, 40);
    // Should return offset 0 with no data
    ASSERT_FALSE(chunk.empty());
}

TEST(KlipperIdentifyProtocol, ClientServerRoundTrip) {
    // Create a blob
    std::vector<uint8_t> blob(200);
    for (size_t i = 0; i < 200; ++i) blob[i] = static_cast<uint8_t>(i);

    IdentifyServer server(blob);
    IdentifyClient client;

    while (!client.complete()) {
        // buildRequestContent encodes [msgid, offset, count]
        // We use nextOffset() to know what offset will be requested
        uint32_t offset = client.nextOffset();
        (void)offset; // The server uses the offset we pass to buildResponseContent

        // Build response using the client's next offset
        auto resp = server.buildResponseContent(client.nextOffset(), 40);
        ASSERT_TRUE(client.consumeResponseContent(resp));
    }

    EXPECT_TRUE(client.complete());
    EXPECT_EQ(client.wireBlob(), blob);
}

TEST(KlipperIdentifyProtocol, ClientOffsetMismatch) {
    IdentifyClient client;
    // Build a response with offset=1 (client expects 0)
    // The response format is: msgid(VLQ) + offset(VLQ) + length(VLQ) + data
    // kMsgIdIdentifyResponse = 0, so VLQ encodes as 0x00
    // offset=1 encodes as 0x01
    // length=0 encodes as 0x00
    std::vector<uint8_t> wrongResp = {0x00, 0x01, 0x00}; // msgid=0, offset=1, len=0
    bool ok = client.consumeResponseContent(wrongResp);
    EXPECT_FALSE(ok); // offset mismatch (1 != 0)
}

TEST(KlipperIdentifyProtocol, ClientDecodeDictionary) {
    // Build a dictionary, get its wire blob, serve it, decode it
    DataDictionary dict;
    dict.addCommand("test_cmd val=%u");
    dict.addConstant("TEST", 99);
    auto wire = dict.toWire();

    IdentifyServer server(wire);
    IdentifyClient client;

    while (!client.complete()) {
        auto resp = server.buildResponseContent(client.nextOffset(), 40);
        client.consumeResponseContent(resp);
    }

    auto decoded = client.decodeDictionary();
    ASSERT_TRUE(decoded.has_value());
    auto cmd = decoded->lookupCommand("test_cmd val=%u");
    ASSERT_TRUE(cmd.has_value());
}

// ============================================================================
// CommandTable
// ============================================================================

TEST(KlipperCommandTable, EncodeDecodeRoundTrip) {
    DataDictionary dict;
    uint16_t msgid = dict.addCommand("test_cmd oid=%c val=%u");

    std::vector<ParamValue> params;
    ParamValue p1; p1.integer = 5; p1.isInteger = true;
    ParamValue p2; p2.integer = 1000; p2.isInteger = true;
    params.push_back(p1);
    params.push_back(p2);

    std::vector<uint8_t> encoded;
    ASSERT_TRUE(encodeMessage(dict, msgid, params, encoded));
    ASSERT_FALSE(encoded.empty());

    auto decoded = decodeMessages(dict, encoded);
    ASSERT_EQ(decoded.size(), 1u);
    EXPECT_EQ(decoded[0].msgid, msgid);
    ASSERT_EQ(decoded[0].params.size(), 2u);
    EXPECT_EQ(decoded[0].params[0].integer, 5);
    EXPECT_EQ(decoded[0].params[1].integer, 1000);
}

TEST(KlipperCommandTable, DispatchCommand) {
    DataDictionary dict;
    uint16_t msgid = dict.addCommand("test_cmd val=%u");

    CommandTable table(dict);
    bool called = false;
    int32_t receivedVal = 0;
    table.registerCommand(msgid, [&](const std::vector<ParamValue>& params) {
        called = true;
        if (!params.empty()) receivedVal = params[0].integer;
    });

    std::vector<ParamValue> params;
    ParamValue p; p.integer = 42; p.isInteger = true;
    params.push_back(p);

    std::vector<uint8_t> encoded;
    encodeMessage(dict, msgid, params, encoded);
    auto decoded = decodeMessages(dict, encoded);
    ASSERT_EQ(decoded.size(), 1u);
    table.dispatchCommand(decoded[0]);

    EXPECT_TRUE(called);
    EXPECT_EQ(receivedVal, 42);
}

TEST(KlipperCommandTable, DispatchResponse) {
    DataDictionary dict;
    uint16_t msgid = dict.addResponse("test_resp result=%u");

    CommandTable table(dict);
    bool called = false;
    table.registerResponse(msgid, [&](const std::vector<ParamValue>&) {
        called = true;
    });

    std::vector<ParamValue> params;
    ParamValue p; p.integer = 1; params.push_back(p);
    std::vector<uint8_t> encoded;
    encodeMessage(dict, msgid, params, encoded);
    auto decoded = decodeMessages(dict, encoded);
    table.dispatchResponse(decoded[0]);
    EXPECT_TRUE(called);
}

TEST(KlipperCommandTable, DispatchUnregisteredHandler) {
    DataDictionary dict;
    uint16_t msgid = dict.addCommand("test_cmd val=%u");
    CommandTable table(dict);

    // Dispatch without registering handler - should not crash
    DecodedMessage msg;
    msg.msgid = msgid;
    table.dispatchCommand(msg); // No-op
}

TEST(KlipperCommandTable, ArityMismatch) {
    DataDictionary dict;
    uint16_t msgid = dict.addCommand("test_cmd a=%u b=%u");

    // Provide wrong number of params
    std::vector<ParamValue> params;
    ParamValue p; p.integer = 1; params.push_back(p); // Only 1, expected 2

    std::vector<uint8_t> encoded;
    bool ok = encodeMessage(dict, msgid, params, encoded);
    EXPECT_FALSE(ok); // Should fail
}

TEST(KlipperCommandTable, Clear) {
    DataDictionary dict;
    uint16_t msgid = dict.addCommand("test_cmd val=%u");
    CommandTable table(dict);

    bool called = false;
    table.registerCommand(msgid, [&](const std::vector<ParamValue>&) {
        called = true;
    });
    table.clear();

    DecodedMessage msg;
    msg.msgid = msgid;
    table.dispatchCommand(msg);
    EXPECT_FALSE(called); // Should not be called after clear
}

TEST(KlipperCommandTable, DecodeUnknownMsgid) {
    DataDictionary dict;
    std::vector<uint8_t> content = {0xFF, 0xFF}; // Unknown msgid
    auto decoded = decodeMessages(dict, content);
    // Should return empty or handle gracefully
    EXPECT_TRUE(decoded.empty() || decoded.size() == 0);
}

TEST(KlipperCommandTable, EncodeMultipleMessages) {
    DataDictionary dict;
    uint16_t msgid1 = dict.addCommand("cmd1 val=%u");
    uint16_t msgid2 = dict.addCommand("cmd2 val=%u");

    std::vector<uint8_t> buf;
    std::vector<ParamValue> params1 = {ParamValue{42, {}, true}};
    std::vector<ParamValue> params2 = {ParamValue{99, {}, true}};

    ASSERT_TRUE(encodeMessage(dict, msgid1, params1, buf));
    ASSERT_TRUE(encodeMessage(dict, msgid2, params2, buf));

    auto decoded = decodeMessages(dict, buf);
    ASSERT_EQ(decoded.size(), 2u);
    EXPECT_EQ(decoded[0].msgid, msgid1);
    EXPECT_EQ(decoded[0].params[0].integer, 42);
    EXPECT_EQ(decoded[1].msgid, msgid2);
    EXPECT_EQ(decoded[1].params[0].integer, 99);
}

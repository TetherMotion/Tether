/**
 * @file test_glaze_adapter.cpp
 * @brief Unit tests for the GlazeJsonValueAdapter.
 */

#include <gtest/gtest.h>

#include "tether/klipper/http/GlazeAdapter.hpp"
#include "tether/klipper/http/ResponseBuilder.hpp"
#include "tether/klipper/http/JsonRpcDispatcher.hpp"
#include "tether/klipper/http/WsSessionManager.hpp"

#include <sstream>

using namespace tether::klipper::http;
using namespace tether::klipper::klippy;

// ============================================================================
// GlazeAdapter tests
// ============================================================================

TEST(GlazeAdapterTest, NullRoundTrip) {
    JsonValue val;
    auto gen = toJsonGeneric(val);
    EXPECT_TRUE(gen.is_null());
    auto back = fromJsonGeneric(gen);
    EXPECT_TRUE(back.isNull());
}

TEST(GlazeAdapterTest, BoolRoundTrip) {
    JsonValue val(true);
    auto gen = toJsonGeneric(val);
    EXPECT_TRUE(gen.is_boolean());
    auto back = fromJsonGeneric(gen);
    EXPECT_TRUE(back.isBool());
    EXPECT_TRUE(back.asBool());
}

TEST(GlazeAdapterTest, IntRoundTrip) {
    JsonValue val(static_cast<int64_t>(42));
    auto gen = toJsonGeneric(val);
    EXPECT_TRUE(gen.is_number());
    auto back = fromJsonGeneric(gen);
    EXPECT_TRUE(back.isInt());
    EXPECT_EQ(back.asInt(), 42);
}

TEST(GlazeAdapterTest, DoubleRoundTrip) {
    JsonValue val(3.14);
    auto gen = toJsonGeneric(val);
    EXPECT_TRUE(gen.is_number());
    auto back = fromJsonGeneric(gen);
    EXPECT_TRUE(back.isDouble());
    EXPECT_NEAR(back.asDouble(), 3.14, 1e-9);
}

TEST(GlazeAdapterTest, StringRoundTrip) {
    JsonValue val("hello world");
    auto gen = toJsonGeneric(val);
    EXPECT_TRUE(gen.is_string());
    auto back = fromJsonGeneric(gen);
    EXPECT_TRUE(back.isString());
    EXPECT_EQ(back.asString(), "hello world");
}

TEST(GlazeAdapterTest, EmptyStringRoundTrip) {
    JsonValue val("");
    auto gen = toJsonGeneric(val);
    EXPECT_TRUE(gen.is_string());
    auto back = fromJsonGeneric(gen);
    EXPECT_TRUE(back.isString());
    EXPECT_EQ(back.asString(), "");
}

TEST(GlazeAdapterTest, ArrayRoundTrip) {
    std::vector<JsonValue> arr;
    arr.push_back(JsonValue(1));
    arr.push_back(JsonValue("two"));
    arr.push_back(JsonValue(true));
    JsonValue val(arr);

    auto gen = toJsonGeneric(val);
    EXPECT_TRUE(gen.is_array());
    auto back = fromJsonGeneric(gen);
    EXPECT_TRUE(back.isArray());
    ASSERT_EQ(back.asArray().size(), 3u);
    EXPECT_EQ(back.asArray()[0].asInt(), 1);
    EXPECT_EQ(back.asArray()[1].asString(), "two");
    EXPECT_EQ(back.asArray()[2].asBool(), true);
}

TEST(GlazeAdapterTest, EmptyArrayRoundTrip) {
    JsonValue val(std::vector<JsonValue>{});
    auto gen = toJsonGeneric(val);
    EXPECT_TRUE(gen.is_array());
    auto back = fromJsonGeneric(gen);
    EXPECT_TRUE(back.isArray());
    EXPECT_EQ(back.asArray().size(), 0u);
}

TEST(GlazeAdapterTest, ObjectRoundTrip) {
    std::map<std::string, JsonValue> obj;
    obj["a"] = JsonValue(1);
    obj["b"] = JsonValue("hello");
    obj["c"] = JsonValue(false);
    JsonValue val(obj);

    auto gen = toJsonGeneric(val);
    EXPECT_TRUE(gen.is_object());
    auto back = fromJsonGeneric(gen);
    EXPECT_TRUE(back.isObject());
    ASSERT_EQ(back.asObject().size(), 3u);
    EXPECT_EQ(back.asObject().at("a").asInt(), 1);
    EXPECT_EQ(back.asObject().at("b").asString(), "hello");
    EXPECT_EQ(back.asObject().at("c").asBool(), false);
}

TEST(GlazeAdapterTest, EmptyObjectRoundTrip) {
    JsonValue val(std::map<std::string, JsonValue>{});
    auto gen = toJsonGeneric(val);
    EXPECT_TRUE(gen.is_object());
    auto back = fromJsonGeneric(gen);
    EXPECT_TRUE(back.isObject());
    EXPECT_EQ(back.asObject().size(), 0u);
}

TEST(GlazeAdapterTest, NestedObjectRoundTrip) {
    std::map<std::string, JsonValue> inner;
    inner["x"] = JsonValue(10);
    inner["y"] = JsonValue(20.5);

    std::map<std::string, JsonValue> outer;
    outer["nested"] = JsonValue(inner);
    outer["name"] = JsonValue("test");

    JsonValue val(outer);

    auto gen = toJsonGeneric(val);
    auto back = fromJsonGeneric(gen);
    EXPECT_TRUE(back.isObject());
    EXPECT_EQ(back.asObject().at("name").asString(), "test");
    EXPECT_TRUE(back.asObject().at("nested").isObject());
    EXPECT_EQ(back.asObject().at("nested").asObject().at("x").asInt(), 10);
    EXPECT_NEAR(back.asObject().at("nested").asObject().at("y").asDouble(), 20.5, 1e-9);
}

TEST(GlazeAdapterTest, ArrayOfObjectsRoundTrip) {
    std::map<std::string, JsonValue> obj1;
    obj1["id"] = JsonValue(static_cast<int64_t>(1));
    std::map<std::string, JsonValue> obj2;
    obj2["id"] = JsonValue(static_cast<int64_t>(2));

    std::vector<JsonValue> arr;
    arr.push_back(JsonValue(obj1));
    arr.push_back(JsonValue(obj2));

    JsonValue val(arr);

    auto gen = toJsonGeneric(val);
    auto back = fromJsonGeneric(gen);
    EXPECT_TRUE(back.isArray());
    ASSERT_EQ(back.asArray().size(), 2u);
    EXPECT_EQ(back.asArray()[0].asObject().at("id").asInt(), 1);
    EXPECT_EQ(back.asArray()[1].asObject().at("id").asInt(), 2);
}

TEST(GlazeAdapterTest, DumpJsonBasic) {
    JsonValue val(static_cast<int64_t>(42));
    std::string json = dumpJson(val);
    EXPECT_EQ(json, "42");
}

TEST(GlazeAdapterTest, DumpJsonString) {
    JsonValue val("hello");
    std::string json = dumpJson(val);
    EXPECT_EQ(json, "\"hello\"");
}

TEST(GlazeAdapterTest, DumpJsonObject) {
    std::map<std::string, JsonValue> obj;
    obj["key"] = JsonValue("value");
    JsonValue val(obj);
    std::string json = dumpJson(val);
    // Order may vary with ordered_small_map, but should contain both
    EXPECT_NE(json.find("\"key\""), std::string::npos);
    EXPECT_NE(json.find("\"value\""), std::string::npos);
}

TEST(GlazeAdapterTest, ParseJsonValid) {
    auto result = parseJson(R"({"key": "value", "num": 42})");
    ASSERT_TRUE(result.has_value());
    EXPECT_TRUE(result->isObject());
    EXPECT_EQ(result->asObject().at("key").asString(), "value");
    EXPECT_EQ(result->asObject().at("num").asInt(), 42);
}

TEST(GlazeAdapterTest, ParseJsonInvalid) {
    auto result = parseJson("not valid json{{{");
    EXPECT_FALSE(result.has_value());
}

TEST(GlazeAdapterTest, ParseJsonArray) {
    auto result = parseJson(R"([1, 2, 3])");
    ASSERT_TRUE(result.has_value());
    EXPECT_TRUE(result->isArray());
    ASSERT_EQ(result->asArray().size(), 3u);
    EXPECT_EQ(result->asArray()[0].asInt(), 1);
    EXPECT_EQ(result->asArray()[1].asInt(), 2);
    EXPECT_EQ(result->asArray()[2].asInt(), 3);
}

TEST(GlazeAdapterTest, ParseJsonNested) {
    auto result = parseJson(R"({"a": {"b": [1, 2, {"c": "d"}]}})");
    ASSERT_TRUE(result.has_value());
    EXPECT_TRUE(result->isObject());
    EXPECT_TRUE(result->asObject().at("a").isObject());
    EXPECT_TRUE(result->asObject().at("a").asObject().at("b").isArray());
    EXPECT_EQ(result->asObject().at("a").asObject().at("b").asArray()[2].asObject().at("c").asString(), "d");
}

// ============================================================================
// ResponseBuilder tests
// ============================================================================

TEST(ResponseBuilderTest, SuccessResponse) {
    JsonValue result(static_cast<int64_t>(42));
    std::string resp = buildSuccessResponse(result);
    EXPECT_NE(resp.find("\"result\""), std::string::npos);
    EXPECT_NE(resp.find("42"), std::string::npos);
}

TEST(ResponseBuilderTest, ErrorResponse) {
    std::string resp = buildErrorResponse(404, "Not found");
    EXPECT_NE(resp.find("\"error\""), std::string::npos);
    EXPECT_NE(resp.find("404"), std::string::npos);
    EXPECT_NE(resp.find("Not found"), std::string::npos);
}

TEST(ResponseBuilderTest, JsonRpcSuccess) {
    JsonValue id(static_cast<int64_t>(1));
    JsonValue result("ok");
    std::string resp = buildJsonRpcSuccess(id, result);
    EXPECT_NE(resp.find("\"jsonrpc\""), std::string::npos);
    EXPECT_NE(resp.find("\"2.0\""), std::string::npos);
    EXPECT_NE(resp.find("\"id\""), std::string::npos);
    EXPECT_NE(resp.find("\"result\""), std::string::npos);
}

TEST(ResponseBuilderTest, JsonRpcError) {
    JsonValue id(static_cast<int64_t>(1));
    std::string resp = buildJsonRpcError(id, -32601, "Method not found");
    EXPECT_NE(resp.find("\"jsonrpc\""), std::string::npos);
    EXPECT_NE(resp.find("\"error\""), std::string::npos);
    EXPECT_NE(resp.find("-32601"), std::string::npos);
    EXPECT_NE(resp.find("Method not found"), std::string::npos);
}

TEST(ResponseBuilderTest, JsonRpcNotification) {
    JsonValue params(std::map<std::string, JsonValue>{{"key", JsonValue("value")}});
    std::string msg = buildJsonRpcNotification("notify_test", params);
    EXPECT_NE(msg.find("\"jsonrpc\""), std::string::npos);
    EXPECT_NE(msg.find("\"method\""), std::string::npos);
    EXPECT_NE(msg.find("notify_test"), std::string::npos);
    EXPECT_NE(msg.find("\"params\""), std::string::npos);
}

// ============================================================================
// JsonRpcDispatcher tests
// ============================================================================

TEST(JsonRpcDispatcherTest, DottedToSlash) {
    EXPECT_EQ(JsonRpcDispatcher::dottedToSlash("server.info"), "server/info");
    EXPECT_EQ(JsonRpcDispatcher::dottedToSlash("printer.objects.query"), "printer/objects/query");
    EXPECT_EQ(JsonRpcDispatcher::dottedToSlash("server.info"), "server/info");
    EXPECT_EQ(JsonRpcDispatcher::dottedToSlash("simple"), "simple");
}

TEST(JsonRpcDispatcherTest, SlashToDotted) {
    EXPECT_EQ(JsonRpcDispatcher::slashToDotted("server/info"), "server.info");
    EXPECT_EQ(JsonRpcDispatcher::slashToDotted("printer/objects/query"), "printer.objects.query");
    EXPECT_EQ(JsonRpcDispatcher::slashToDotted("simple"), "simple");
}

TEST(JsonRpcDispatcherTest, DispatchSuccess) {
    // Create a mock endpoint callable
    auto endpointCall = [](const std::string& method, const JsonValue& params) -> JsonValue {
        if (method == "server/info") {
            std::map<std::string, JsonValue> result;
            result["status"] = JsonValue("ready");
            return JsonValue(result);
        }
        return JsonValue();
    };

    JsonRpcDispatcher dispatcher(endpointCall);

    std::string request = R"({"jsonrpc":"2.0","method":"server.info","id":1})";
    std::string response = dispatcher.dispatch(request);

    EXPECT_NE(response.find("\"jsonrpc\""), std::string::npos);
    EXPECT_NE(response.find("\"2.0\""), std::string::npos);
    EXPECT_NE(response.find("\"result\""), std::string::npos);
    EXPECT_NE(response.find("ready"), std::string::npos);
}

TEST(JsonRpcDispatcherTest, DispatchMethodNotFound) {
    auto endpointCall = [](const std::string& method, const JsonValue& params) -> JsonValue {
        throw EndpointError("Method not found: " + method);
    };

    JsonRpcDispatcher dispatcher(endpointCall);

    std::string request = R"({"jsonrpc":"2.0","method":"unknown.method","id":2})";
    std::string response = dispatcher.dispatch(request);

    EXPECT_NE(response.find("\"error\""), std::string::npos);
    EXPECT_NE(response.find("Method not found"), std::string::npos);
}

TEST(JsonRpcDispatcherTest, DispatchParseError) {
    auto endpointCall = [](const std::string& method, const JsonValue& params) -> JsonValue {
        return JsonValue();
    };

    JsonRpcDispatcher dispatcher(endpointCall);

    std::string response = dispatcher.dispatch("not valid json");
    EXPECT_NE(response.find("\"error\""), std::string::npos);
    // Should have parse error code -32700
    EXPECT_NE(response.find("-32700"), std::string::npos);
}

TEST(JsonRpcDispatcherTest, DispatchInvalidRequest) {
    auto endpointCall = [](const std::string& method, const JsonValue& params) -> JsonValue {
        return JsonValue();
    };

    JsonRpcDispatcher dispatcher(endpointCall);

    // Missing jsonrpc version
    std::string request = R"({"method":"server.info","id":1})";
    std::string response = dispatcher.dispatch(request);
    EXPECT_NE(response.find("\"error\""), std::string::npos);
    EXPECT_NE(response.find("-32600"), std::string::npos);
}

TEST(JsonRpcDispatcherTest, DispatchNotification) {
    bool called = false;
    auto endpointCall = [&called](const std::string& method, const JsonValue& params) -> JsonValue {
        called = true;
        return JsonValue();
    };

    JsonRpcDispatcher dispatcher(endpointCall);

    // Notification — no id
    std::string request = R"({"jsonrpc":"2.0","method":"server.info"})";
    std::string response = dispatcher.dispatch(request);
    EXPECT_TRUE(response.empty());  // No response for notifications
    EXPECT_TRUE(called);
}

TEST(JsonRpcDispatcherTest, DispatchWithParams) {
    auto endpointCall = [](const std::string& method, const JsonValue& params) -> JsonValue {
        if (method == "printer/gcode/script") {
            EXPECT_TRUE(params.isObject());
            EXPECT_EQ(params.asObject().at("script").asString(), "G28");
        }
        return JsonValue("ok");
    };

    JsonRpcDispatcher dispatcher(endpointCall);

    std::string request = R"({"jsonrpc":"2.0","method":"printer.gcode.script","params":{"script":"G28"},"id":5})";
    std::string response = dispatcher.dispatch(request);
    EXPECT_NE(response.find("\"result\""), std::string::npos);
}

TEST(JsonRpcDispatcherTest, DispatchConnectionIdentify) {
    auto endpointCall = [](const std::string& method, const JsonValue& params) -> JsonValue {
        return JsonValue();
    };

    JsonRpcDispatcher dispatcher(endpointCall);

    std::string request = R"({"jsonrpc":"2.0","method":"server.connection.identify","params":{"client_type":"web"},"id":1})";
    std::string response = dispatcher.dispatch(request);
    EXPECT_NE(response.find("connection_id"), std::string::npos);
    EXPECT_NE(response.find("websocket_id"), std::string::npos);
}

// ============================================================================
// WsSessionManager tests
// ============================================================================

TEST(WsSessionManagerTest, CreateAndRemoveSession) {
    WsSessionManager mgr;
    // Without a real WebSocket connection, we pass nullptr
    auto id = mgr.createSession(nullptr);
    EXPECT_GT(id, 0);
    EXPECT_EQ(mgr.sessionCount(), 1u);

    auto session = mgr.getSession(id);
    ASSERT_NE(session, nullptr);
    EXPECT_EQ(session->id, id);

    mgr.removeSession(id);
    EXPECT_EQ(mgr.sessionCount(), 0u);
    EXPECT_EQ(mgr.getSession(id), nullptr);
}

TEST(WsSessionManagerTest, MultipleSessions) {
    WsSessionManager mgr;
    auto id1 = mgr.createSession(nullptr);
    auto id2 = mgr.createSession(nullptr);
    auto id3 = mgr.createSession(nullptr);

    EXPECT_EQ(mgr.sessionCount(), 3u);
    EXPECT_NE(id1, id2);
    EXPECT_NE(id2, id3);
    EXPECT_NE(id1, id3);

    mgr.removeSession(id2);
    EXPECT_EQ(mgr.sessionCount(), 2u);
    EXPECT_EQ(mgr.getSession(id2), nullptr);
    EXPECT_NE(mgr.getSession(id1), nullptr);
    EXPECT_NE(mgr.getSession(id3), nullptr);
}

TEST(WsSessionManagerTest, Subscriptions) {
    WsSessionManager mgr;
    auto id = mgr.createSession(nullptr);

    std::map<std::string, std::vector<std::string>> subs;
    subs["toolhead"] = {"position", "status"};
    subs["extruder"] = {};  // all fields

    mgr.setSubscriptions(id, subs);

    auto retrieved = mgr.getSubscriptions(id);
    EXPECT_EQ(retrieved.size(), 2u);
    EXPECT_EQ(retrieved["toolhead"].size(), 2u);
    EXPECT_EQ(retrieved["toolhead"][0], "position");
    EXPECT_EQ(retrieved["toolhead"][1], "status");
    EXPECT_EQ(retrieved["extruder"].size(), 0u);
}

TEST(WsSessionManagerTest, GcodeSubscription) {
    WsSessionManager mgr;
    auto id1 = mgr.createSession(nullptr);
    auto id2 = mgr.createSession(nullptr);

    mgr.setGcodeSubscribed(id1, true);

    auto gcodeSessions = mgr.getGcodeSubscribedSessions();
    EXPECT_EQ(gcodeSessions.size(), 1u);
    EXPECT_EQ(gcodeSessions[0]->id, id1);

    mgr.setGcodeSubscribed(id1, false);
    EXPECT_EQ(mgr.getGcodeSubscribedSessions().size(), 0u);
}

TEST(WsSessionManagerTest, SetIdentified) {
    WsSessionManager mgr;
    auto id = mgr.createSession(nullptr);

    mgr.setIdentified(id, "web", "Mainsail", "2.9.0");

    auto session = mgr.getSession(id);
    ASSERT_NE(session, nullptr);
    EXPECT_TRUE(session->identified);
    EXPECT_EQ(session->clientType, "web");
    EXPECT_EQ(session->clientName, "Mainsail");
    EXPECT_EQ(session->version, "2.9.0");
}

TEST(WsSessionManagerTest, BaselineUpdate) {
    WsSessionManager mgr;
    auto id = mgr.createSession(nullptr);

    std::map<std::string, JsonValue> fields;
    fields["temperature"] = JsonValue(25.0);
    fields["target"] = JsonValue(static_cast<int64_t>(200));

    mgr.updateBaseline(id, "extruder", fields);

    auto baseline = mgr.getBaseline(id);
    EXPECT_EQ(baseline.size(), 1u);
    EXPECT_EQ(baseline["extruder"].size(), 2u);
    EXPECT_NEAR(baseline["extruder"]["temperature"].asDouble(), 25.0, 1e-9);
    EXPECT_EQ(baseline["extruder"]["target"].asInt(), 200);
}

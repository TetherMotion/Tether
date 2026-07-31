/**
 * @file test_klipper_uds.cpp
 * @brief Tests for the KlippyUdsServer (Moonraker UDS interface).
 */

#include "tether/klipper/klippy/KlippyUdsServer.hpp"

#include <gtest/gtest.h>
#include <sys/socket.h>
#include <sys/un.h>
#include <thread>
#include <unistd.h>

using namespace tether::klipper::klippy;

namespace {
/// @brief Helper to connect a client to the UDS server.
int connectClient(const std::string& path) {
    int fd = ::socket(AF_UNIX, SOCK_STREAM, 0);
    if (fd < 0) return -1;
    struct sockaddr_un addr{};
    addr.sun_family = AF_UNIX;
    strncpy(addr.sun_path, path.c_str(), sizeof(addr.sun_path) - 1);
    if (::connect(fd, reinterpret_cast<struct sockaddr*>(&addr), sizeof(addr)) < 0) {
        ::close(fd);
        return -1;
    }
    return fd;
}

/// @brief Send a JSON frame (terminated by ETX) to a socket.
bool sendFrame(int fd, const std::string& json) {
    std::string data = json + "\x03";
    size_t total = 0;
    while (total < data.size()) {
        ssize_t n = ::write(fd, data.data() + total, data.size() - total);
        if (n <= 0) return false;
        total += n;
    }
    return true;
}

/// @brief Read a frame from a socket (blocking with timeout).
std::string readFrame(int fd, int timeoutMs = 2000) {
    // Set receive timeout
    struct timeval tv;
    tv.tv_sec = timeoutMs / 1000;
    tv.tv_usec = (timeoutMs % 1000) * 1000;
    ::setsockopt(fd, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));

    std::string result;
    char buf[4096];
    while (true) {
        ssize_t n = ::read(fd, buf, sizeof(buf));
        if (n <= 0) break;
        result.append(buf, n);
        if (result.find('\x03') != std::string::npos) break;
    }
    size_t etx = result.find('\x03');
    if (etx != std::string::npos) result = result.substr(0, etx);
    return result;
}

/// @brief Parse a JSON response frame.
std::optional<JsonValue> sendAndReceive(int fd, const std::string& json) {
    if (!sendFrame(fd, json)) return std::nullopt;
    auto resp = readFrame(fd);
    if (resp.empty()) return std::nullopt;
    return JsonValue::parse(resp);
}

} // anonymous namespace

// ============================================================================
// JSON value tests
// ============================================================================

TEST(KlippyUdsJson, ParseString) {
    auto v = JsonValue::parse("\"hello world\"");
    ASSERT_TRUE(v.has_value());
    EXPECT_TRUE(v->isString());
    EXPECT_EQ(v->asString(), "hello world");
}

TEST(KlippyUdsJson, ParseInt) {
    auto v = JsonValue::parse("42");
    ASSERT_TRUE(v.has_value());
    EXPECT_TRUE(v->isInt());
    EXPECT_EQ(v->asInt(), 42);
}

TEST(KlippyUdsJson, ParseDouble) {
    auto v = JsonValue::parse("3.14");
    ASSERT_TRUE(v.has_value());
    EXPECT_TRUE(v->isDouble());
    EXPECT_NEAR(v->asDouble(), 3.14, 0.001);
}

TEST(KlippyUdsJson, ParseBool) {
    auto v1 = JsonValue::parse("true");
    ASSERT_TRUE(v1.has_value());
    EXPECT_TRUE(v1->isBool());
    EXPECT_TRUE(v1->asBool());

    auto v2 = JsonValue::parse("false");
    ASSERT_TRUE(v2.has_value());
    EXPECT_TRUE(v2->asBool() == false);
}

TEST(KlippyUdsJson, ParseNull) {
    auto v = JsonValue::parse("null");
    ASSERT_TRUE(v.has_value());
    EXPECT_TRUE(v->isNull());
}

TEST(KlippyUdsJson, ParseArray) {
    auto v = JsonValue::parse("[1, 2, 3]");
    ASSERT_TRUE(v.has_value());
    EXPECT_TRUE(v->isArray());
    EXPECT_EQ(v->asArray().size(), 3u);
    EXPECT_EQ(v->asArray()[0].asInt(), 1);
    EXPECT_EQ(v->asArray()[1].asInt(), 2);
    EXPECT_EQ(v->asArray()[2].asInt(), 3);
}

TEST(KlippyUdsJson, ParseObject) {
    auto v = JsonValue::parse("{\"key\": \"value\", \"num\": 42}");
    ASSERT_TRUE(v.has_value());
    EXPECT_TRUE(v->isObject());
    EXPECT_EQ(v->asObject().size(), 2u);
    EXPECT_EQ(v->asObject()["key"].asString(), "value");
    EXPECT_EQ(v->asObject()["num"].asInt(), 42);
}

TEST(KlippyUdsJson, ParseNestedObject) {
    auto v = JsonValue::parse("{\"outer\": {\"inner\": \"val\"}}");
    ASSERT_TRUE(v.has_value());
    EXPECT_TRUE(v->isObject());
    EXPECT_TRUE(v->asObject()["outer"].isObject());
    EXPECT_EQ(v->asObject()["outer"].asObject().at("inner").asString(), "val");
}

TEST(KlippyUdsJson, DumpString) {
    JsonValue v("hello");
    EXPECT_EQ(v.dump(), "\"hello\"");
}

TEST(KlippyUdsJson, DumpObject) {
    std::map<std::string, JsonValue> obj;
    obj["a"] = JsonValue(1);
    obj["b"] = JsonValue("two");
    JsonValue v(obj);
    std::string dumped = v.dump();
    EXPECT_TRUE(dumped.find("\"a\":1") != std::string::npos);
    EXPECT_TRUE(dumped.find("\"b\":\"two\"") != std::string::npos);
}

TEST(KlippyUdsJson, RoundTrip) {
    std::string json = "{\"method\":\"info\",\"params\":{\"client_info\":{\"program\":\"test\"}},\"id\":1}";
    auto v = JsonValue::parse(json);
    ASSERT_TRUE(v.has_value());
    EXPECT_TRUE(v->isObject());
    EXPECT_EQ(v->asObject()["method"].asString(), "info");
    EXPECT_EQ(v->asObject()["id"].asInt(), 1);
    // Re-dump and re-parse
    auto v2 = JsonValue::parse(v->dump());
    ASSERT_TRUE(v2.has_value());
    EXPECT_EQ(v2->asObject()["method"].asString(), "info");
}

TEST(KlippyUdsJson, FindKey) {
    std::map<std::string, JsonValue> obj;
    obj["x"] = JsonValue(10);
    obj["y"] = JsonValue(20);
    JsonValue v(obj);
    auto* x = v.find("x");
    ASSERT_NE(x, nullptr);
    EXPECT_EQ(x->asInt(), 10);
    EXPECT_EQ(v.find("z"), nullptr);
    EXPECT_TRUE(v.has("x"));
    EXPECT_FALSE(v.has("z"));
}

TEST(KlippyUdsJson, EscapedString) {
    auto v = JsonValue::parse("\"hello\\nworld\"");
    ASSERT_TRUE(v.has_value());
    EXPECT_EQ(v->asString(), "hello\nworld");
}

// ============================================================================
// UDS server tests
// ============================================================================

class KlippyUdsTest : public ::testing::Test {
protected:
    void SetUp() override {
        // Use a unique socket path per test
        socketPath_ = "/tmp/klippy_uds_test_" + std::to_string(getpid()) +
                       "_" + std::to_string(counter_++);
        UdsServerConfig cfg;
        cfg.socketPath = socketPath_;
        cfg.refreshIntervalMs = 50; // Fast refresh for tests
        server_ = std::make_unique<KlippyUdsServer>(cfg);
        ASSERT_TRUE(server_->start());
        // Give the server a moment to start
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }

    void TearDown() override {
        server_->stop();
        server_.reset();
        ::unlink(socketPath_.c_str());
    }

    int connectClient_() {
        int fd = ::socket(AF_UNIX, SOCK_STREAM, 0);
        if (fd < 0) return -1;
        struct sockaddr_un addr{};
        addr.sun_family = AF_UNIX;
        strncpy(addr.sun_path, socketPath_.c_str(), sizeof(addr.sun_path) - 1);
        // Retry connect a few times
        for (int i = 0; i < 10; ++i) {
            if (::connect(fd, reinterpret_cast<struct sockaddr*>(&addr), sizeof(addr)) == 0)
                return fd;
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
        ::close(fd);
        return -1;
    }

    bool sendFrame_(int fd, const std::string& json) {
        std::string data = json + "\x03";
        size_t total = 0;
        while (total < data.size()) {
            ssize_t n = ::write(fd, data.data() + total, data.size() - total);
            if (n <= 0) return false;
            total += n;
        }
        return true;
    }

    std::string readFrame_(int fd, int timeoutMs = 2000) {
        struct timeval tv;
        tv.tv_sec = timeoutMs / 1000;
        tv.tv_usec = (timeoutMs % 1000) * 1000;
        ::setsockopt(fd, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));

        std::string result;
        char buf[4096];
        while (true) {
            ssize_t n = ::read(fd, buf, sizeof(buf));
            if (n <= 0) break;
            result.append(buf, n);
            if (result.find('\x03') != std::string::npos) break;
        }
        size_t etx = result.find('\x03');
        if (etx != std::string::npos) result = result.substr(0, etx);
        return result;
    }

    std::optional<JsonValue> sendRecv_(int fd, const std::string& json) {
        if (!sendFrame_(fd, json)) return std::nullopt;
        auto resp = readFrame_(fd);
        if (resp.empty()) return std::nullopt;
        return JsonValue::parse(resp);
    }

    std::string socketPath_;
    std::unique_ptr<KlippyUdsServer> server_;
    static int counter_;
};

int KlippyUdsTest::counter_ = 0;

TEST_F(KlippyUdsTest, ServerStartsAndStops) {
    EXPECT_TRUE(server_->isRunning());
    server_->stop();
    EXPECT_FALSE(server_->isRunning());
}

TEST_F(KlippyUdsTest, ClientCanConnect) {
    int fd = connectClient_();
    ASSERT_GE(fd, 0);
    ::close(fd);
}

TEST_F(KlippyUdsTest, InfoEndpoint) {
    int fd = connectClient_();
    ASSERT_GE(fd, 0);

    auto resp = sendRecv_(fd, R"({"id":1,"method":"info","params":{"client_info":{"program":"test"}}})");
    ASSERT_TRUE(resp.has_value());
    EXPECT_TRUE(resp->isObject());
    EXPECT_TRUE(resp->has("id"));
    EXPECT_EQ(resp->find("id")->asInt(), 1);
    EXPECT_TRUE(resp->has("result"));
    auto& result = resp->find("result")->asObject();
    EXPECT_EQ(result.at("state").asString(), "startup");
    EXPECT_TRUE(result.at("state_message").isString());
    EXPECT_TRUE(result.at("software_version").isString());
    EXPECT_TRUE(result.at("process_id").isInt());

    ::close(fd);
}

TEST_F(KlippyUdsTest, ListEndpointsEndpoint) {
    int fd = connectClient_();
    ASSERT_GE(fd, 0);

    auto resp = sendRecv_(fd, R"({"id":2,"method":"list_endpoints"})");
    ASSERT_TRUE(resp.has_value());
    EXPECT_TRUE(resp->has("result"));
    auto& result = resp->find("result")->asObject();
    EXPECT_TRUE(result.at("endpoints").isArray());
    auto& endpoints = result.at("endpoints").asArray();
    // Should contain core endpoints
    bool foundInfo = false, foundGcodeScript = false, foundObjectsList = false;
    for (const auto& e : endpoints) {
        if (e.asString() == "info") foundInfo = true;
        if (e.asString() == "gcode/script") foundGcodeScript = true;
        if (e.asString() == "objects/list") foundObjectsList = true;
    }
    EXPECT_TRUE(foundInfo);
    EXPECT_TRUE(foundGcodeScript);
    EXPECT_TRUE(foundObjectsList);

    ::close(fd);
}

TEST_F(KlippyUdsTest, ObjectsListEndpoint) {
    int fd = connectClient_();
    ASSERT_GE(fd, 0);

    auto resp = sendRecv_(fd, R"({"id":3,"method":"objects/list"})");
    ASSERT_TRUE(resp.has_value());
    EXPECT_TRUE(resp->has("result"));
    auto& result = resp->find("result")->asObject();
    EXPECT_TRUE(result.at("objects").isArray());
    auto& objs = result.at("objects").asArray();
    // Should contain built-in objects
    bool foundWebhooks = false, foundToolhead = false, foundGcodeMove = false;
    for (const auto& o : objs) {
        if (o.asString() == "webhooks") foundWebhooks = true;
        if (o.asString() == "toolhead") foundToolhead = true;
        if (o.asString() == "gcode_move") foundGcodeMove = true;
    }
    EXPECT_TRUE(foundWebhooks);
    EXPECT_TRUE(foundToolhead);
    EXPECT_TRUE(foundGcodeMove);

    ::close(fd);
}

TEST_F(KlippyUdsTest, ObjectsQueryEndpoint) {
    int fd = connectClient_();
    ASSERT_GE(fd, 0);

    auto resp = sendRecv_(fd,
        R"({"id":4,"method":"objects/query","params":{"objects":{"webhooks":null}}})");
    ASSERT_TRUE(resp.has_value());
    EXPECT_TRUE(resp->has("result"));
    auto& result = resp->find("result")->asObject();
    EXPECT_TRUE(result.at("status").isObject());
    auto& status = result.at("status").asObject();
    EXPECT_TRUE(status.count("webhooks") > 0);
    auto& wh = status.at("webhooks").asObject();
    EXPECT_TRUE(wh.count("state") > 0);
    EXPECT_EQ(wh.at("state").asString(), "startup");

    ::close(fd);
}

TEST_F(KlippyUdsTest, ObjectsSubscribeEndpoint) {
    int fd = connectClient_();
    ASSERT_GE(fd, 0);

    auto resp = sendRecv_(fd,
        R"({"id":5,"method":"objects/subscribe","params":{"objects":{"webhooks":null},"response_template":{"method":"process_status_update"}}})");
    ASSERT_TRUE(resp.has_value());
    EXPECT_TRUE(resp->has("result"));
    auto& result = resp->find("result")->asObject();
    EXPECT_TRUE(result.at("status").isObject());
    EXPECT_TRUE(result.at("eventtime").isDouble());

    // Wait for subscription count to update
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    EXPECT_EQ(server_->subscriptionCount(), 1u);

    ::close(fd);
}

TEST_F(KlippyUdsTest, SubscriptionPushOnStateChange) {
    int fd = connectClient_();
    ASSERT_GE(fd, 0);

    // Subscribe to webhooks
    auto resp = sendRecv_(fd,
        R"({"id":6,"method":"objects/subscribe","params":{"objects":{"webhooks":null},"response_template":{"method":"process_status_update"}}})");
    ASSERT_TRUE(resp.has_value());

    // Change state to ready
    server_->setState(PrinterState::Ready, "Printer is ready");

    // Wait for push message
    auto pushed = readFrame_(fd, 1000);
    ASSERT_FALSE(pushed.empty());
    auto pv = JsonValue::parse(pushed);
    ASSERT_TRUE(pv.has_value());
    EXPECT_EQ(pv->find("method")->asString(), "process_status_update");
    auto& params = pv->find("params")->asObject();
    auto& status = params.at("status").asObject();
    auto& wh = status.at("webhooks").asObject();
    EXPECT_EQ(wh.at("state").asString(), "ready");

    ::close(fd);
}

TEST_F(KlippyUdsTest, GcodeScriptEndpoint) {
    int fd = connectClient_();
    ASSERT_GE(fd, 0);

    std::string receivedScript;
    server_->setGcodeScriptHandler([&](const std::string& script) {
        receivedScript = script;
    });

    auto resp = sendRecv_(fd,
        R"({"id":7,"method":"gcode/script","params":{"script":"G28\nG1 X100 F600"}})");
    ASSERT_TRUE(resp.has_value());
    EXPECT_TRUE(resp->has("result"));
    EXPECT_EQ(receivedScript, "G28\nG1 X100 F600");

    ::close(fd);
}

TEST_F(KlippyUdsTest, EmergencyStopEndpoint) {
    int fd = connectClient_();
    ASSERT_GE(fd, 0);

    bool stopCalled = false;
    server_->setEmergencyStopHandler([&]() { stopCalled = true; });

    auto resp = sendRecv_(fd, R"({"id":8,"method":"emergency_stop"})");
    ASSERT_TRUE(resp.has_value());
    EXPECT_TRUE(resp->has("result"));
    EXPECT_TRUE(stopCalled);
    EXPECT_EQ(server_->state(), PrinterState::Shutdown);

    ::close(fd);
}

TEST_F(KlippyUdsTest, GcodeSubscribeOutput) {
    int fd = connectClient_();
    ASSERT_GE(fd, 0);

    // Subscribe to G-code output
    auto resp = sendRecv_(fd,
        R"({"id":9,"method":"gcode/subscribe_output","params":{"response_template":{"method":"process_gcode_response"}}})");
    ASSERT_TRUE(resp.has_value());
    EXPECT_TRUE(resp->has("result"));

    // Emit a G-code response
    server_->emitGcodeResponse("// Hello from G-code");

    // Read the push
    auto pushed = readFrame_(fd, 1000);
    ASSERT_FALSE(pushed.empty());
    auto pv = JsonValue::parse(pushed);
    ASSERT_TRUE(pv.has_value());
    EXPECT_EQ(pv->find("method")->asString(), "process_gcode_response");
    EXPECT_EQ(pv->find("params")->asObject().at("response").asString(),
              "// Hello from G-code");

    ::close(fd);
}

TEST_F(KlippyUdsTest, RegisterRemoteMethod) {
    int fd = connectClient_();
    ASSERT_GE(fd, 0);

    // Register a remote method
    auto resp = sendRecv_(fd,
        R"({"id":10,"method":"register_remote_method","params":{"remote_method":"test_callback","response_template":{"method":"test_callback"}}})");
    ASSERT_TRUE(resp.has_value());
    EXPECT_TRUE(resp->has("result"));

    // Invoke the remote method
    std::map<std::string, JsonValue> params;
    params["value"] = JsonValue(42);
    server_->invokeRemoteMethod("test_callback", JsonValue(params));

    // Read the push
    auto pushed = readFrame_(fd, 1000);
    ASSERT_FALSE(pushed.empty());
    auto pv = JsonValue::parse(pushed);
    ASSERT_TRUE(pv.has_value());
    EXPECT_EQ(pv->find("method")->asString(), "test_callback");
    EXPECT_EQ(pv->find("params")->asObject().at("value").asInt(), 42);

    ::close(fd);
}

TEST_F(KlippyUdsTest, UnknownMethodReturnsError) {
    int fd = connectClient_();
    ASSERT_GE(fd, 0);

    auto resp = sendRecv_(fd, R"({"id":11,"method":"nonexistent/method"})");
    ASSERT_TRUE(resp.has_value());
    EXPECT_TRUE(resp->has("error"));
    auto& err = resp->find("error")->asObject();
    EXPECT_TRUE(err.at("message").isString());
    EXPECT_EQ(err.at("error").asString(), "WebRequestError");

    ::close(fd);
}

TEST_F(KlippyUdsTest, StateTransitions) {
    // Initial state is startup
    EXPECT_EQ(server_->state(), PrinterState::Startup);

    // Transition to ready
    server_->setState(PrinterState::Ready, "Printer is ready");
    EXPECT_EQ(server_->state(), PrinterState::Ready);
    EXPECT_EQ(server_->stateMessage(), "Printer is ready");

    // Transition to shutdown
    server_->setState(PrinterState::Shutdown, "Emergency stop");
    EXPECT_EQ(server_->state(), PrinterState::Shutdown);
    EXPECT_EQ(server_->stateMessage(), "Emergency stop");
}

TEST_F(KlippyUdsTest, InvalidStateTransitionIgnored) {
    EXPECT_EQ(server_->state(), PrinterState::Startup);

    // Cannot go from startup to ready then back to startup
    server_->setState(PrinterState::Ready);
    EXPECT_EQ(server_->state(), PrinterState::Ready);

    // Cannot go from ready back to startup
    server_->setState(PrinterState::Startup);
    EXPECT_EQ(server_->state(), PrinterState::Ready);
}

TEST_F(KlippyUdsTest, CustomEndpointRegistration) {
    // Register a custom endpoint
    server_->registerEndpoint("custom/test", [](const JsonValue& params) {
        std::map<std::string, JsonValue> result;
        result["echo"] = params;
        return JsonValue(result);
    });

    int fd = connectClient_();
    ASSERT_GE(fd, 0);

    auto resp = sendRecv_(fd, R"({"id":12,"method":"custom/test","params":{"msg":"hello"}})");
    ASSERT_TRUE(resp.has_value());
    EXPECT_TRUE(resp->has("result"));
    auto& result = resp->find("result")->asObject();
    EXPECT_TRUE(result.at("echo").isObject());
    EXPECT_EQ(result.at("echo").asObject().at("msg").asString(), "hello");

    ::close(fd);
}

TEST_F(KlippyUdsTest, MultipleClients) {
    int fd1 = connectClient_();
    int fd2 = connectClient_();
    ASSERT_GE(fd1, 0);
    ASSERT_GE(fd2, 0);

    // Both should get responses
    auto r1 = sendRecv_(fd1, R"({"id":1,"method":"info"})");
    auto r2 = sendRecv_(fd2, R"({"id":1,"method":"info"})");
    ASSERT_TRUE(r1.has_value());
    ASSERT_TRUE(r2.has_value());
    EXPECT_EQ(r1->find("id")->asInt(), 1);
    EXPECT_EQ(r2->find("id")->asInt(), 1);

    ::close(fd1);
    ::close(fd2);
}

TEST_F(KlippyUdsTest, CustomPrinterObject) {
    class TestObject : public PrinterObject {
    public:
        std::string name() const override { return "test_obj"; }
        std::map<std::string, JsonValue> status(
            const std::vector<std::string>& fields) const override {
            std::map<std::string, JsonValue> result;
            result["value"] = JsonValue(123);
            result["name"] = JsonValue("test");
            return result;
        }
        std::vector<std::string> availableFields() const override {
            return {"value", "name"};
        }
    };

    server_->registerObject(std::make_shared<TestObject>());

    int fd = connectClient_();
    ASSERT_GE(fd, 0);

    auto resp = sendRecv_(fd,
        R"({"id":13,"method":"objects/query","params":{"objects":{"test_obj":null}}})");
    ASSERT_TRUE(resp.has_value());
    auto& status = resp->find("result")->asObject().at("status").asObject();
    EXPECT_TRUE(status.count("test_obj") > 0);
    EXPECT_EQ(status.at("test_obj").asObject().at("value").asInt(), 123);

    ::close(fd);
}

TEST_F(KlippyUdsTest, GcodeHelpEndpoint) {
    int fd = connectClient_();
    ASSERT_GE(fd, 0);

    auto resp = sendRecv_(fd, R"({"id":14,"method":"gcode/help"})");
    ASSERT_TRUE(resp.has_value());
    EXPECT_TRUE(resp->has("result"));
    auto& result = resp->find("result")->asObject();
    EXPECT_TRUE(result.count("G28") > 0);
    EXPECT_TRUE(result.count("M112") > 0);

    ::close(fd);
}

TEST_F(KlippyUdsTest, RestartEndpoint) {
    int fd = connectClient_();
    ASSERT_GE(fd, 0);

    bool restartCalled = false;
    server_->setRestartHandler([&]() { restartCalled = true; });

    auto resp = sendRecv_(fd, R"({"id":15,"method":"gcode/restart"})");
    ASSERT_TRUE(resp.has_value());
    EXPECT_TRUE(resp->has("result"));
    EXPECT_TRUE(restartCalled);

    ::close(fd);
}

TEST_F(KlippyUdsTest, NotificationNoResponse) {
    int fd = connectClient_();
    ASSERT_GE(fd, 0);

    // Send a notification (no id) - should get no response
    EXPECT_TRUE(sendFrame_(fd, R"({"method":"some_notification","params":{}})"));

    // Try to read - should timeout (no response for notifications)
    auto resp = readFrame_(fd, 500);
    EXPECT_TRUE(resp.empty()); // No response expected

    ::close(fd);
}

TEST_F(KlippyUdsTest, MalformedFrameIgnored) {
    int fd = connectClient_();
    ASSERT_GE(fd, 0);

    // Send malformed JSON
    EXPECT_TRUE(sendFrame_(fd, "not valid json"));

    // Server should still work - send a valid request
    auto resp = sendRecv_(fd, R"({"id":16,"method":"info"})");
    ASSERT_TRUE(resp.has_value());
    EXPECT_TRUE(resp->has("result"));

    ::close(fd);
}

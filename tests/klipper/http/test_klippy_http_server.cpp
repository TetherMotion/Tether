/**
 * @file test_klippy_http_server.cpp
 * @brief End-to-end coverage of KlippyHttpServer.cpp — a real
 *        KlippyHttpServer bound to 127.0.0.1 exercised through
 *        Drogon's HttpClient over loopback HTTP.
 *
 * drogon::app() can only run once per process, so the server is
 * brought up once in SetUpTestSuite and torn down in
 * TearDownTestSuite; every test shares the running instance.
 */

#include <gtest/gtest.h>

#include <chrono>
#include <atomic>
#include <functional>
#include <condition_variable>
#include <deque>
#include <future>
#include <mutex>
#include <memory>
#include <string>
#include <thread>
#include <unistd.h>

#include <drogon/HttpClient.h>
#include <drogon/HttpRequest.h>
#include <drogon/HttpResponse.h>
#include <drogon/WebSocketClient.h>

#include "tether/klipper/http/KlippyHttpServer.hpp"
#include "tether/klipper/klippy/KlippyServer.hpp"

using namespace tether::klipper;
using namespace tether::klipper::http;
using namespace std::chrono_literals;

namespace {

constexpr uint16_t kPort = 17125;

class KlippyHttpServerE2ETest : public ::testing::Test {
protected:
    static void SetUpTestSuite() {
        klippy::UdsServerConfig uds;
        uds.socketPath = "/tmp/tether_http_e2e_" +
                         std::to_string(getpid()) + ".sock";
        sockPath_ = uds.socketPath;
        server_ = std::make_unique<klippy::KlippyServer>(uds);

        HttpServerConfig cfg;
        cfg.port = kPort;
        cfg.bindAddress = "127.0.0.1";
        cfg.requireAuth = false;
        http_ = std::make_unique<KlippyHttpServer>(*server_, cfg);
        ASSERT_TRUE(http_->start());
        client_ = drogon::HttpClient::newHttpClient(
            "http://127.0.0.1:" + std::to_string(kPort));
        waitReady();
    }
    static void TearDownTestSuite() {
        client_.reset();
        http_->stop();
        http_.reset();
        server_.reset();
        ::unlink(sockPath_.c_str());
    }

    /// Block until the listener answers (Drogon starts async).
    static void waitReady() {
        for (int i = 0; i < 100; ++i) {
            auto req = drogon::HttpRequest::newHttpRequest();
            req->setPath("/server/info");
            std::promise<drogon::ReqResult> p;
            client_->sendRequest(req, [&](drogon::ReqResult r,
                                          const drogon::HttpResponsePtr&) {
                p.set_value(r);
            });
            auto fut = p.get_future();
            if (fut.wait_for(2s) == std::future_status::ready &&
                fut.get() == drogon::ReqResult::Ok)
                return;
            std::this_thread::sleep_for(50ms);
        }
        FAIL() << "HTTP server did not become ready";
    }

    /// Synchronous request helper; returns {status, body}.
    static std::pair<int, std::string> request(
        drogon::HttpMethod method, const std::string& path,
        const std::string& body = {},
        const std::string& contentType = "application/json") {
        auto req = drogon::HttpRequest::newHttpRequest();
        req->setMethod(method);
        req->setPath(path);
        if (!body.empty()) {
            req->setBody(body);
            req->setContentTypeString(contentType);
        }
        std::promise<std::pair<int, std::string>> p;
        client_->sendRequest(
            req, [&](drogon::ReqResult r,
                     const drogon::HttpResponsePtr& resp) {
                if (r == drogon::ReqResult::Ok && resp) {
                    p.set_value({static_cast<int>(resp->getStatusCode()),
                                 std::string(resp->getBody())});
                } else {
                    p.set_value({-1, {}});
                }
            });
        auto fut = p.get_future();
        EXPECT_EQ(fut.wait_for(10s), std::future_status::ready) << path;
        return fut.get();
    }

    static drogon::HttpResponsePtr get(const std::string& path) {
        auto req = drogon::HttpRequest::newHttpRequest();
        req->setPath(path);
        std::promise<drogon::HttpResponsePtr> p;
        client_->sendRequest(req, [&](drogon::ReqResult,
                                      const drogon::HttpResponsePtr& r) {
            p.set_value(r);
        });
        return p.get_future().get();
    }

    static std::unique_ptr<klippy::KlippyServer> server_;
    static std::unique_ptr<KlippyHttpServer> http_;
    static drogon::HttpClientPtr client_;
    static std::string sockPath_;
};

std::unique_ptr<klippy::KlippyServer> KlippyHttpServerE2ETest::server_;
std::unique_ptr<KlippyHttpServer> KlippyHttpServerE2ETest::http_;
drogon::HttpClientPtr KlippyHttpServerE2ETest::client_;
std::string KlippyHttpServerE2ETest::sockPath_;

// ============================================================================
// Server metadata
// ============================================================================

TEST_F(KlippyHttpServerE2ETest, ServerInfoReturnsMoonrakerShape) {
    auto [code, body] = request(drogon::Get, "/server/info");
    ASSERT_EQ(code, 200);
    EXPECT_NE(body.find("klippy_connected"), std::string::npos);
    EXPECT_NE(body.find("moonraker"), std::string::npos);
    EXPECT_NE(body.find("api_version"), std::string::npos);
}

TEST_F(KlippyHttpServerE2ETest, ServerConfigReturnsFlags) {
    auto [code, body] = request(drogon::Get, "/server/config");
    ASSERT_EQ(code, 200);
    EXPECT_NE(body.find("result"), std::string::npos);
}

TEST_F(KlippyHttpServerE2ETest, MachineSystemInfoResponds) {
    auto [code, body] = request(drogon::Get, "/machine/system_info");
    ASSERT_EQ(code, 200);
    EXPECT_NE(body.find("result"), std::string::npos);
}

TEST_F(KlippyHttpServerE2ETest, MachineProcStatsResponds) {
    auto [code, body] = request(drogon::Get, "/machine/proc_stats");
    ASSERT_EQ(code, 200);
    EXPECT_NE(body.find("result"), std::string::npos);
}

TEST_F(KlippyHttpServerE2ETest, ApiVersionResponds) {
    auto [code, body] = request(drogon::Get, "/api/version");
    ASSERT_EQ(code, 200);
}

// ============================================================================
// Printer endpoints
// ============================================================================

TEST_F(KlippyHttpServerE2ETest, PrinterInfoResponds) {
    auto [code, body] = request(drogon::Get, "/printer/info");
    ASSERT_EQ(code, 200);
    EXPECT_NE(body.find("state"), std::string::npos);
}

TEST_F(KlippyHttpServerE2ETest, PrinterObjectsListResponds) {
    auto [code, body] = request(drogon::Get, "/printer/objects/list");
    ASSERT_EQ(code, 200);
    EXPECT_NE(body.find("objects"), std::string::npos);
}

TEST_F(KlippyHttpServerE2ETest, PrinterObjectsQueryResponds) {
    auto [code, body] = request(
        drogon::Get, "/printer/objects/query?gcode_macro");
    ASSERT_EQ(code, 200);
}

TEST_F(KlippyHttpServerE2ETest, GcodeHelpResponds) {
    auto [code, body] = request(drogon::Get, "/printer/gcode/help");
    ASSERT_EQ(code, 200);
}

TEST_F(KlippyHttpServerE2ETest, GcodeScriptExecutes) {
    auto [code, body] = request(
        drogon::Post, "/printer/gcode/script",
        R"({"script":"RESPOND MSG=http_e2e"})");
    ASSERT_EQ(code, 200);
    EXPECT_NE(body.find("result"), std::string::npos);
}

TEST_F(KlippyHttpServerE2ETest, QueryEndstopsResponds) {
    auto [code, body] = request(
        drogon::Get, "/printer/query_endstops/status");
    ASSERT_EQ(code, 200);
}

// ============================================================================
// File endpoints
// ============================================================================

TEST_F(KlippyHttpServerE2ETest, FilesRootsAndList) {
    auto [code, body] = request(drogon::Get, "/server/files/roots");
    ASSERT_EQ(code, 200);
    auto [code2, body2] = request(drogon::Get, "/server/files/list");
    ASSERT_EQ(code2, 200);
    EXPECT_NE(body2.find("result"), std::string::npos);
}

TEST_F(KlippyHttpServerE2ETest, FilesDirectoryResponds) {
    auto [code, body] = request(
        drogon::Get, "/server/files/get_directory");
    ASSERT_EQ(code, 200);
}

// ============================================================================
// Access endpoints (auth disabled → permissive)
// ============================================================================

TEST_F(KlippyHttpServerE2ETest, AccessInfoAndApiKey) {
    auto [code, body] = request(drogon::Get, "/access/info");
    ASSERT_EQ(code, 200);
    auto [code2, body2] = request(drogon::Get, "/access/api_key");
    ASSERT_EQ(code2, 200);
}

TEST_F(KlippyHttpServerE2ETest, AccessUsersListResponds) {
    auto [code, body] = request(drogon::Get, "/access/users/list");
    ASSERT_EQ(code, 200);
}

TEST_F(KlippyHttpServerE2ETest, OneshotTokenResponds) {
    auto [code, body] = request(drogon::Get, "/access/oneshot_token");
    ASSERT_EQ(code, 200);
}

// ============================================================================
// Misc endpoints
// ============================================================================

TEST_F(KlippyHttpServerE2ETest, NotepadListResponds) {
    auto [code, body] = request(drogon::Get, "/server/notepad/list");
    ASSERT_EQ(code, 200);
}

TEST_F(KlippyHttpServerE2ETest, DatabaseListResponds) {
    auto [code, body] = request(drogon::Get, "/server/database/list");
    ASSERT_EQ(code, 200);
}

TEST_F(KlippyHttpServerE2ETest, WebcamsListResponds) {
    auto [code, body] = request(drogon::Get, "/server/webcams/list");
    ASSERT_EQ(code, 200);
}

TEST_F(KlippyHttpServerE2ETest, HistoryTotalsResponds) {
    auto [code, body] = request(drogon::Get, "/server/history/totals");
    ASSERT_EQ(code, 200);
}

TEST_F(KlippyHttpServerE2ETest, JobQueueStatusResponds) {
    auto [code, body] = request(drogon::Get, "/server/job_queue/status");
    ASSERT_EQ(code, 200);
}

TEST_F(KlippyHttpServerE2ETest, AnnouncementsListResponds) {
    auto [code, body] = request(drogon::Get, "/server/announcements/list");
    ASSERT_EQ(code, 200);
}

TEST_F(KlippyHttpServerE2ETest, TemperatureStoreResponds) {
    auto [code, body] = request(drogon::Get, "/server/temperature_store");
    ASSERT_EQ(code, 200);
}

TEST_F(KlippyHttpServerE2ETest, GcodeStoreResponds) {
    auto [code, body] = request(drogon::Get, "/server/gcode_store");
    ASSERT_EQ(code, 200);
}

TEST_F(KlippyHttpServerE2ETest, SensorsListResponds) {
    auto [code, body] = request(drogon::Get, "/server/sensors/list");
    ASSERT_EQ(code, 200);
}

TEST_F(KlippyHttpServerE2ETest, LogsListResponds) {
    auto [code, body] = request(drogon::Get, "/server/logs/list");
    ASSERT_EQ(code, 200);
}

TEST_F(KlippyHttpServerE2ETest, DevicePowerListResponds) {
    auto [code, body] = request(
        drogon::Get, "/machine/device_power/devices");
    ASSERT_EQ(code, 200);
}

TEST_F(KlippyHttpServerE2ETest, ExtensionsListResponds) {
    auto [code, body] = request(drogon::Get, "/server/extensions/list");
    ASSERT_EQ(code, 200);
}

TEST_F(KlippyHttpServerE2ETest, UpdateStatusResponds) {
    auto [code, body] = request(drogon::Get, "/machine/update/status");
    ASSERT_EQ(code, 200);
}

// ============================================================================
// JSON-RPC dispatch
// ============================================================================

TEST_F(KlippyHttpServerE2ETest, JsonRpcServerInfo) {
    auto [code, body] = request(
        drogon::Post, "/server/jsonrpc",
        R"({"jsonrpc":"2.0","method":"server.info","id":1})");
    ASSERT_EQ(code, 200);
    EXPECT_NE(body.find("jsonrpc"), std::string::npos);
    EXPECT_NE(body.find("result"), std::string::npos);
}

TEST_F(KlippyHttpServerE2ETest, JsonRpcUnknownMethodReturnsError) {
    auto [code, body] = request(
        drogon::Post, "/server/jsonrpc",
        R"({"jsonrpc":"2.0","method":"nonexistent/method","id":2})");
    ASSERT_EQ(code, 200);
    EXPECT_NE(body.find("error"), std::string::npos);
}

TEST_F(KlippyHttpServerE2ETest, JsonRpcPrinterObjectsQuery) {
    auto [code, body] = request(
        drogon::Post, "/server/jsonrpc",
        R"({"jsonrpc":"2.0","method":"printer.objects.query","params":{"objects":{}},"id":3})");
    ASSERT_EQ(code, 200);
}

// ============================================================================
// Error paths
// ============================================================================

TEST_F(KlippyHttpServerE2ETest, UnknownRouteReturns404) {
    auto [code, body] = request(drogon::Get, "/definitely/not/a/route");
    EXPECT_EQ(code, 404);
}

TEST_F(KlippyHttpServerE2ETest, MalformedJsonRpcHandled) {
    auto [code, body] = request(
        drogon::Post, "/server/jsonrpc", "this is not json");
    // Must not crash; either an error body or a handled status.
    EXPECT_TRUE(code == 200 || code == 400) << code;
}

TEST_F(KlippyHttpServerE2ETest, OptionsPreflightResponds) {
    auto [code, body] = request(drogon::Options, "/server/options");
    EXPECT_TRUE(code == 200 || code == 204) << code;
}
// ============================================================================
// WebSocket — KlippyWsController / handleWsMessage / WsSessionManager wire
// ============================================================================

/// Connect a WebSocket to /websocket and hand the connection + a
/// "next message" receiver to `exchange`.  The connection is shut down
/// only after exchange returns, so exchanges can await responses.
static void withWebSocket(
    const std::function<void(
        const drogon::WebSocketConnectionPtr&,
        const std::function<std::string(std::chrono::milliseconds)>&)>&
        exchange) {
    auto ws = drogon::WebSocketClient::newWebSocketClient(
        "127.0.0.1", kPort, false);
    std::promise<bool> connP;
    auto queue = std::make_shared<std::mutex>();
    auto cv = std::make_shared<std::condition_variable>();
    auto messages = std::make_shared<std::deque<std::string>>();
    ws->setMessageHandler(
        [queue, cv, messages](std::string&& msg,
                              const drogon::WebSocketClientPtr&,
                              const drogon::WebSocketMessageType&) {
            std::lock_guard lk(*queue);
            messages->push_back(std::move(msg));
            cv->notify_all();
        });
    auto req = drogon::HttpRequest::newHttpRequest();
    req->setPath("/websocket");
    ws->connectToServer(
        req, [&](drogon::ReqResult r, const drogon::HttpResponsePtr&,
                 const drogon::WebSocketClientPtr&) {
            connP.set_value(r == drogon::ReqResult::Ok);
        });
    auto fut = connP.get_future();
    EXPECT_EQ(fut.wait_for(10s), std::future_status::ready);
    EXPECT_TRUE(fut.get());
    auto conn = ws->getConnection();
    EXPECT_NE(conn, nullptr);
    if (!conn) return;

    // Pop the next server message, waiting up to `timeout`.  Returns ""
    // on timeout.
    auto nextMessage = [queue, cv, messages](
                           std::chrono::milliseconds timeout) -> std::string {
        std::unique_lock lk(*queue);
        if (!cv->wait_for(lk, timeout, [&] { return !messages->empty(); }))
            return {};
        auto m = std::move(messages->front());
        messages->pop_front();
        return m;
    };

    exchange(conn, nextMessage);
    conn->shutdown();
    ws->stop();
    // Let the close handshake propagate before the client destructs.
    std::this_thread::sleep_for(50ms);
}

TEST_F(KlippyHttpServerE2ETest, WsIdentifyGetsJsonRpcResponse) {
    withWebSocket(
        [](const drogon::WebSocketConnectionPtr& conn, auto next) {
            conn->send(R"({"jsonrpc":"2.0","method":"server.connection.identify","params":{"client_name":"e2e","version":"1.0"},"id":7})");
            const std::string resp = next(5s);
            ASSERT_FALSE(resp.empty());
            EXPECT_NE(resp.find("\"id\":7"), std::string::npos);
        });
}

TEST_F(KlippyHttpServerE2ETest, WsObjectsSubscribeGetsResponse) {
    withWebSocket(
        [](const drogon::WebSocketConnectionPtr& conn, auto next) {
            conn->send(R"({"jsonrpc":"2.0","method":"printer.objects.subscribe","params":{"objects":{"gcode_macro":null}},"id":9})");
            const std::string resp = next(5s);
            ASSERT_FALSE(resp.empty());
            EXPECT_NE(resp.find("\"id\":9"), std::string::npos);
        });
}

TEST_F(KlippyHttpServerE2ETest, WsServerInfoRoundTrips) {
    withWebSocket(
        [](const drogon::WebSocketConnectionPtr& conn, auto next) {
            conn->send(R"({"jsonrpc":"2.0","method":"server.info","id":13})");
            const std::string resp = next(5s);
            ASSERT_FALSE(resp.empty());
            EXPECT_NE(resp.find("klippy_connected"), std::string::npos);
        });
}

TEST_F(KlippyHttpServerE2ETest, WsDisconnectRemovesSession) {
    // Connect + immediately close; the server must not crash or leak —
    // a subsequent connection still works.
    {
        auto ws = drogon::WebSocketClient::newWebSocketClient(
            "127.0.0.1", kPort, false);
        std::promise<bool> connP;
        auto req = drogon::HttpRequest::newHttpRequest();
        req->setPath("/websocket");
        ws->connectToServer(
            req, [&](drogon::ReqResult r, const drogon::HttpResponsePtr&,
                     const drogon::WebSocketClientPtr&) {
                connP.set_value(r == drogon::ReqResult::Ok);
            });
        EXPECT_TRUE(connP.get_future().get());
        ws->getConnection()->shutdown();
        ws->stop();
    }
    std::this_thread::sleep_for(100ms);
    withWebSocket(
        [](const drogon::WebSocketConnectionPtr& conn, auto next) {
            conn->send(R"({"jsonrpc":"2.0","method":"server.info","id":14})");
            const std::string resp = next(5s);
            ASSERT_FALSE(resp.empty());
            EXPECT_NE(resp.find("\"id\":14"), std::string::npos);
        });
}

} // namespace

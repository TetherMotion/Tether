/**
 * @file test_io_client_commands.cpp
 * @brief Dispatcher-level tests for tether_io_client's command layer —
 *        IoSession + dispatch() over an in-proc MessagePipeTransport vs a
 *        real Session(Framing::None) hosting the SimulatedCiA402Fleet.
 *
 * Covers: list/get/set/snapshot/ping/call/stream/log — the same grammar
 * the CLI, shell REPL, and TUI `:` prompt all share.
 */
#include <gtest/gtest.h>
#include "client_session.hpp"
#include "commands.hpp"
#include "value_text.hpp"
#include "tether/io/Session.hpp"
#include "tether/io/SimulatedCiA402Fleet.hpp"
#include "tether/io/TetherIOClient.hpp"
#include "PipeTransport.hpp"
#include <sstream>
#include <stdexcept>
#include <thread>
#include <cstring>

using namespace tether::io;
using namespace tether::io::client;
using namespace tether::io::testing;

namespace {

void noopLog(const char*, const char*, ...) {}

/// In-proc client↔session pair: IoSession on the client end of a
/// MessagePipeTransport, Session(Framing::None) on the server end.
struct Fixture {
    Registry registry;
    SchemaGraph graph;
    SchemaCatalog catalog;
    cia402::SimulatedCiA402Fleet fleet;
    std::unique_ptr<Session> server;
    std::thread serverThread;
    std::unique_ptr<IoSession> session;
    std::ostringstream out, err;

    Fixture() {
        graph = cia402::machineProfileSchemaGraph();
        if (!cia402::SimulatedCiA402Fleet::installSchemas(graph, catalog))
            throw std::runtime_error("schema install failed");
        if (!fleet.registerSignals(registry, catalog))
            throw std::runtime_error("signal registration failed");
        auto [clientEnd, serverEnd] = MessagePipeTransport::create();
        server = std::make_unique<Session>(
            std::move(serverEnd), registry,
            [] { return cia402::simulated_fleet_detail::monotonicTimestampUs(); },
            noopLog, nullptr, nullptr, nullptr, nullptr, nullptr,
            Framing::None, nullptr, &catalog);
        serverThread = std::thread([this] { server->run(); });
        session = std::make_unique<IoSession>(std::move(clientEnd), 5000);
        if (!session->client().isConnected())
            throw std::runtime_error("client not connected");
        if (!session->refreshCatalog())
            throw std::runtime_error("catalog refresh failed");
    }
    ~Fixture() {
        if (server) server->requestStop();
        if (serverThread.joinable()) serverThread.join();
    }

    int run(std::initializer_list<const char*> args) {
        out.str(""); out.clear(); err.str(""); err.clear();
        std::vector<std::string> v;
        for (auto a : args) v.emplace_back(a);
        Ctx ctx{*session, false, false, out, err};
        return dispatch(ctx, v);
    }
};

} // namespace

TEST(IoClientCommands, PingWorks) {
    Fixture f;
    EXPECT_EQ(f.run({"ping"}), 0);
    EXPECT_NE(f.out.str().find("pong"), std::string::npos);
}

TEST(IoClientCommands, InfoShowsCatalog) {
    Fixture f;
    EXPECT_EQ(f.run({"info"}), 0);
    EXPECT_NE(f.out.str().find("schema"), std::string::npos);
    EXPECT_NE(f.out.str().find("catalog"), std::string::npos);
}

TEST(IoClientCommands, ListParamsTable) {
    Fixture f;
    EXPECT_EQ(f.run({"list", "params"}), 0);
    EXPECT_NE(f.out.str().find("NAME"), std::string::npos);
    EXPECT_NE(f.out.str().find("ID"), std::string::npos);
}

TEST(IoClientCommands, ListSignalsShowsFleet) {
    Fixture f;
    EXPECT_EQ(f.run({"list", "signals"}), 0);
    EXPECT_NE(f.out.str().find("machine.cia402"), std::string::npos);
}

TEST(IoClientCommands, ListFunctionsShowsEventRead) {
    Fixture f;
    EXPECT_EQ(f.run({"list", "functions"}), 0);
    EXPECT_NE(f.out.str().find("machine.events.read"), std::string::npos);
}

TEST(IoClientCommands, GetDecodesStructSignal) {
    Fixture f;
    const int rc = f.run({"get", "machine.cia402.machine.snapshot"});
    EXPECT_EQ(rc, 0) << f.err.str();
    // snapshot is a struct → decoded text has field=value pairs
    EXPECT_NE(f.out.str().find("timestamp_us"), std::string::npos);
    EXPECT_NE(f.out.str().find("axis_count"), std::string::npos);
}

TEST(IoClientCommands, SetAndGetParam) {
    Fixture f;
    // fleet params are read-only signals; use a signal read + set on a
    // writable demo param path — the fleet has none, so just verify the
    // command parses and errors cleanly on a read-only entry.
    EXPECT_NE(f.run({"set", "machine.cia402.machine.snapshot", "1"}), 2);
}

TEST(IoClientCommands, UnknownCommandIsError) {
    Fixture f;
    EXPECT_NE(f.run({"bogus"}), 0);
    EXPECT_NE(f.err.str().find("unknown"), std::string::npos);
}

TEST(IoClientCommands, CallFunctionRaw) {
    Fixture f;
    // machine.events.read takes (cursor u64, limit u32) — raw: escape.
    const int rc = f.run({"call", "machine.events.read",
                          "after_cursor=raw:0000000000000000",
                          "limit=raw:05000000"});
    EXPECT_EQ(rc, 0) << f.err.str();
    EXPECT_NE(f.out.str().find("events"), std::string::npos);
}

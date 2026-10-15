/**
 * @file main.cpp
 * @brief tether_io_client — CLI entry point.
 *
 *   tether_io_client [global opts] <command> [args...]
 *
 * Global opts: --url/-u URL, --token T, --role R, --timeout-ms N,
 *              --connect-timeout-ms N, -j/--json
 *
 * Commands: ping, info, list, get, set, snapshot, metadata, call,
 *           stream, watch, log tail, datalog, threshold, shell, tui
 */
#include "client_session.hpp"
#include "commands.hpp"

#include <argparse/argparse.hpp>

#include <cstdio>
#include <format>
#include <iostream>
#include <string>
#include <vector>

using namespace tether::io::client;

namespace {

constexpr const char* kUsage = R"(commands:
  ping [--count N]                          connectivity + latency
  info                                      session/limits/catalog summary
  list params|signals|functions [-g GLOB] [--filter STR] [-j]
  get NAME... [-r|--raw] [-j]               read param/signal values
  set NAME VALUE                            write a parameter
  snapshot [params|signals] [-g GLOB]       one-shot bulk read
  metadata NAME                             metadata map for an entry
  call NAME [k=v ...] [--invoke-ex]         invoke a catalog function
  stream NAME... [--interval-ms N]          live stream to stdout
    [--chunk N] [--duration S] [--count N] [-j]
  watch NAME... [--interval-ms N]           polling live view
  log tail [--severity L]                   subscribe to the log stream
  datalog status|configure ...
  threshold set NAME --rule=type=abs,id=5,val=1.5 ...
  instream create NAME | send ID <hex> | close ID
  peer register NAME [--id N]               host a client-side function
  stress [ping|catalog] [--ops N]           throughput/latency probe
  shell                                     interactive REPL (same grammar)
  tui                                       full-screen explorer (P4)

URLs: ws://host[:port]/path[?token=..&role=..]   (default port 80)
      tcp://host[:port]                          (default port 4000, SLIP)
      serial:///dev/ttyX[?baud=N]                (SLIP)
      host[:port]                                (= tcp://)

Names resolve as: exact name, group.name, numeric id, or p:/s:/f: prefix.
)";

int runShell(IoSession& session, bool json, bool verbose) {
    Ctx ctx{session, json, verbose, std::cout, std::cerr};
    std::cout << "tether_io_client shell — 'quit'/'exit' to leave\n";
    std::string line;
    while (std::cout << "io> " && std::getline(std::cin, line)) {
        const auto args = tokenize(line);
        if (args.empty()) continue;
        if (args[0] == "quit" || args[0] == "exit") break;
        if (args[0] == "help") { std::cout << kUsage; continue; }
        if (dispatchTimed(ctx, args) != 0) { /* error already printed */ }
    }
    return 0;
}

} // namespace

int main(int argc, char** argv) {
    argparse::ArgumentParser prog("tether_io_client");
    prog.add_argument("-u", "--url")
        .help("server URL (ws://host:port/path, tcp://host:port, serial:///dev/x?baud=N)")
        .default_value(std::string("ws://127.0.0.1:8080/tether-io"));
    prog.add_argument("--token").help("bearer token for ws://").default_value(std::string{});
    prog.add_argument("--role").help("role hint appended as ?role= (dev mode)")
        .default_value(std::string{});
    prog.add_argument("--timeout-ms").help("request timeout")
        .default_value(std::string("5000"));
    prog.add_argument("--connect-timeout-ms").help("connect/handshake timeout")
        .default_value(std::string("5000"));
    prog.add_argument("-j", "--json").help("JSON output").flag();
    prog.add_argument("--verbose").help("protocol timing to stderr").flag();
    prog.add_argument("--no-confirm").help("skip mutation prompts").flag();
    prog.add_argument("command_and_args")
        .help("command followed by its arguments")
        .remaining();

    try {
        prog.parse_args(argc, argv);
    } catch (const std::exception& e) {
        std::cerr << e.what() << "\n" << kUsage;
        return 2;
    }

    auto cmdArgs = prog.get<std::vector<std::string>>("command_and_args");
    if (cmdArgs.empty()) {
        std::cerr << "no command given\n\n" << kUsage;
        return 2;
    }

    if (cmdArgs[0] == "help" || cmdArgs[0] == "--help" || cmdArgs[0] == "-h") {
        std::cout << kUsage;
        return 0;
    }
    if (cmdArgs[0] == "version") {
        std::cout << "tether_io_client 0.1.0  (TetherIO V6)\n";
        return 0;
    }

    ConnectSpec spec;
    // TETHER_IO_ADDR env provides the default URL when -u is omitted.
    const char* env = std::getenv("TETHER_IO_ADDR");
    std::string url = prog.get<std::string>("--url");
    if (env && *env &&
        url == "ws://127.0.0.1:8080/tether-io") url = env;
    spec.url = url;
    spec.token = prog.get<std::string>("--token");
    spec.role = prog.get<std::string>("--role");
    spec.requestTimeoutMs = static_cast<uint32_t>(
        std::stoul(prog.get<std::string>("--timeout-ms")));
    spec.connectTimeoutMs = static_cast<uint32_t>(
        std::stoul(prog.get<std::string>("--connect-timeout-ms")));
    const bool json = prog.get<bool>("--json");
    const bool verbose = prog.get<bool>("--verbose");

    auto transport = makeIoTransport(spec);
    if (!transport) {
        std::cerr << "connect: " << transport.error() << '\n';
        return 1;
    }
    IoSession session(std::move(*transport), spec.requestTimeoutMs);
    if (!session.client().isConnected()) {
        std::cerr << "connect: transport not connected\n";
        return 1;
    }

    const std::string& cmd = cmdArgs[0];
    if (cmd == "shell") return runShell(session, json, verbose);
    if (cmd == "tui")   return runTui(session);

    Ctx ctx{session, json, verbose, std::cout, std::cerr};
    return dispatchTimed(ctx, cmdArgs);
}

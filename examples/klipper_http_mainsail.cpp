/**
 * @file klipper_http_mainsail.cpp
 * @brief Example: Run Tether as a Moonraker replacement for Mainsail/Fluidd.
 *
 * @details
 * This example demonstrates the new transport-agnostic architecture:
 *
 *   KlippyServer (business logic)
 *     |
 *     +-- KlippyUdsServer  (UDS transport, for Klipper companion processes)
 *     +-- KlippyHttpServer (HTTP/WebSocket transport, for Mainsail/Fluidd)
 *
 * The example:
 *   1. Creates a single KlippyServer instance with all business logic.
 *   2. Wraps it in a KlippyUdsServer for Klipper companion compatibility.
 *   3. Creates a KlippyHttpServer that shares the same KlippyServer.
 *   4. Starts both transports.
 *   5. Optionally serves Mainsail static assets from a web root directory.
 *
 * Usage:
 *   klipper_http_mainsail [--port PORT] [--uds-path PATH] [--web-root DIR]
 *                         [--gcodes-root DIR] [--api-key KEY]
 *
 * Once running, point Mainsail at http://localhost:PORT/ and it will
 * connect via WebSocket as if talking to a real Moonraker instance.
 *
 * Docker deployment:
 *   See docs/MainsailDocker.md for a complete Docker Compose setup that
 *   runs this example alongside a Mainsail container.
 */

#include "tether/klipper/klippy/KlippyServer.hpp"
#include "tether/klipper/klippy/KlippyUdsServer.hpp"
#include "tether/klipper/http/KlippyHttpServer.hpp"

#include <atomic>
#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <memory>
#include <string>
#include <thread>

using namespace tether::klipper::klippy;
using namespace tether::klipper::http;

static std::atomic<bool> g_running{true};

static void signalHandler(int sig) {
    (void)sig;
    g_running = false;
}

static void printUsage(const char* prog) {
    std::printf(
        "Usage: %s [OPTIONS]\n"
        "\n"
        "Options:\n"
        "  --port PORT         HTTP listen port (default: 7125)\n"
        "  --uds-path PATH     UDS socket path (default: /tmp/klippy_uds)\n"
        "  --web-root DIR      Directory containing Mainsail static assets\n"
        "  --gcodes-root DIR   G-code file root (default: /tmp/tether_sdcard)\n"
        "  --config-root DIR   Config file root (default: /etc/tether)\n"
        "  --logs-root DIR     Log file root (default: /var/log)\n"
        "  --api-key KEY       API key for authentication (default: tether_default_api_key)\n"
        "  --no-auth           Disable authentication\n"
        "  --help              Show this help message\n"
        "\n"
        "Example:\n"
        "  %s --port 7125 --web-root /opt/mainsail --gcodes-root /home/pi/gcodes\n"
        "\n"
        "Then open http://localhost:7125/ in a browser to access Mainsail.\n",
        prog, prog);
}

int main(int argc, char* argv[]) {
    // Defaults
    uint16_t port = 7125;
    std::string udsPath = "/tmp/klippy_uds";
    std::string webRoot;
    std::string gcodesRoot = "/tmp/tether_sdcard";
    std::string configRoot = "/etc/tether";
    std::string logsRoot = "/var/log";
    std::string apiKey = "tether_default_api_key";
    bool requireAuth = true;

    // Parse arguments
    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];
        auto nextArg = [&]() -> std::string {
            if (i + 1 >= argc) {
                std::fprintf(stderr, "Error: %s requires a value\n", argv[i]);
                std::exit(1);
            }
            return argv[++i];
        };

        if (arg == "--help" || arg == "-h") {
            printUsage(argv[0]);
            return 0;
        } else if (arg == "--port") {
            port = static_cast<uint16_t>(std::atoi(nextArg().c_str()));
        } else if (arg == "--uds-path") {
            udsPath = nextArg();
        } else if (arg == "--web-root") {
            webRoot = nextArg();
        } else if (arg == "--gcodes-root") {
            gcodesRoot = nextArg();
        } else if (arg == "--config-root") {
            configRoot = nextArg();
        } else if (arg == "--logs-root") {
            logsRoot = nextArg();
        } else if (arg == "--api-key") {
            apiKey = nextArg();
        } else if (arg == "--no-auth") {
            requireAuth = false;
        } else {
            std::fprintf(stderr, "Unknown option: %s\n", arg.c_str());
            printUsage(argv[0]);
            return 1;
        }
    }

    // Install signal handlers
    std::signal(SIGINT, signalHandler);
    std::signal(SIGTERM, signalHandler);

    // Ensure the G-code directory exists
    std::filesystem::create_directories(gcodesRoot);

    std::printf("=== Tether Moonraker Replacement ===\n\n");
    std::printf("Configuration:\n");
    std::printf("  HTTP port:       %u\n", port);
    std::printf("  UDS path:        %s\n", udsPath.c_str());
    std::printf("  Web root:        %s\n",
                webRoot.empty() ? "(disabled)" : webRoot.c_str());
    std::printf("  G-code root:     %s\n", gcodesRoot.c_str());
    std::printf("  Config root:     %s\n", configRoot.c_str());
    std::printf("  Logs root:       %s\n", logsRoot.c_str());
    std::printf("  Auth required:   %s\n", requireAuth ? "yes" : "no");
    std::printf("  API key:         %s\n\n", apiKey.c_str());
    std::fflush(stdout);

    // ------------------------------------------------------------------
    // Step 1: Create the shared KlippyServer (business logic)
    // ------------------------------------------------------------------
    UdsServerConfig serverCfg;
    serverCfg.socketPath = udsPath;
    serverCfg.configFile = configRoot + "/printer.cfg";

    auto server = std::make_unique<KlippyServer>(serverCfg);
    server->setFileRoot(gcodesRoot);

    // Set the printer to ready state
    server->setState(PrinterState::Ready, "Tether printer is ready");

    // Register some sample data so Mainsail has something to display
    server->registerWebcam("webcam1", "http://localhost:8080/?action=stream");
    server->registerPowerDevice("printer", "on");
    server->registerPowerDevice("lights", "off");
    server->recordTemperature("extruder", 25.0, 0.0);
    server->recordTemperature("heater_bed", 22.0, 0.0);
    server->emitGcodeResponse("// Tether Moonraker replacement started");

    std::printf("KlippyServer created with %zu endpoints and %zu objects\n",
                server->listEndpoints().size(),
                server->listObjects().size());
    std::fflush(stdout);

    // ------------------------------------------------------------------
    // Step 2: Create the UDS transport (for Klipper companion processes)
    // ------------------------------------------------------------------
    KlippyUdsServer udsTransport(*server, serverCfg);

    if (!udsTransport.start()) {
        std::fprintf(stderr, "Failed to start UDS transport at %s\n",
                     udsPath.c_str());
        return 1;
    }
    std::printf("UDS transport listening at %s\n", udsPath.c_str());

    // ------------------------------------------------------------------
    // Step 3: Create the HTTP/WebSocket transport (for Mainsail/Fluidd)
    // ------------------------------------------------------------------
    HttpServerConfig httpCfg;
    httpCfg.port = port;
    httpCfg.apiKey = apiKey;
    httpCfg.requireAuth = requireAuth;
    httpCfg.gcodesRoot = gcodesRoot;
    httpCfg.configRoot = configRoot;
    httpCfg.logsRoot = logsRoot;
    httpCfg.webRoot = webRoot;
    // Trust localhost for development
    httpCfg.trustedClients = {"127.0.0.1", "::1", "172.16.0.0/12", "192.168.0.0/16"};

    auto httpServer = std::make_shared<KlippyHttpServer>(*server, httpCfg);

    if (!httpServer->start()) {
        std::fprintf(stderr, "Failed to start HTTP server on port %u\n", port);
        udsTransport.stop();
        return 1;
    }
    std::printf("HTTP/WebSocket server listening on port %u\n", port);
    if (!webRoot.empty()) {
        std::printf("Serving Mainsail static assets from %s\n", webRoot.c_str());
    }
    std::printf("\nOpen http://localhost:%u/ in a browser to access the web UI.\n",
                port);
    std::printf("Press Ctrl+C to stop.\n\n");
    std::fflush(stdout);

    // ------------------------------------------------------------------
    // Step 4: Run until interrupted
    // ------------------------------------------------------------------
    while (g_running.load()) {
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }

    std::printf("\nShutting down...\n");
    httpServer->stop();
    udsTransport.stop();
    std::printf("Done.\n");

    return 0;
}

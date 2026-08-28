/**
 * @file web_dashboard_example.cpp
 * @brief Standalone web dashboard example serving static files + WebSocket.
 *
 * @details
 * This example starts a single Drogon HTTP + WebSocket server that:
 *
 *   1. Serves the pre-built Tether IO dashboard (HTML/CSS/JS) from
 *      web/tether-io-dashboard/dist/ as static files.
 *   2. Exposes a binary WebSocket endpoint at /tether-io that speaks
 *      the Tether IO protocol.
 *
 * The dashboard lets you browse parameters, signals, and functions in
 * real time.  This example registers:
 *
 *   Parameters (writable from the UI):
 *     - amplitude    — sine/cosine amplitude [V]
 *     - frequency    — wave frequency [Hz]
 *     - sampleRate   — display sample rate [Hz] (metadata only)
 *     - phaseOffset  — phase offset [rad]
 *     - offset       — DC offset added to both signals
 *
 *   Signals (read-only, streamed):
 *     - sine_wave     — amplitude * sin(2π f t + phase) + offset
 *     - cosine_wave   — amplitude * cos(2π f t + phase) + offset
 *
 *   Functions:
 *     - reset_params  — restore all parameters to defaults
 *
 * Usage:
 *   web_dashboard_example [--port PORT] [--web-root PATH]
 *
 * Then open http://127.0.0.1:8080/ in a browser.
 */

#include "TetherIOWebSocketController.hpp"

#include <drogon/drogon.h>

#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <csignal>
#include <iostream>
#include <string>

using namespace tether::io;
using tether::io::example::TetherIOWebSocketController;

namespace {

std::atomic<bool> g_stopRequested{false};
bool g_verbose = false;

void installSignalHandlers() {
    std::signal(SIGINT,  [](int) { g_stopRequested.store(true); });
    std::signal(SIGTERM, [](int) { g_stopRequested.store(true); });
}

uint64_t nowUs() {
    return static_cast<uint64_t>(
        std::chrono::duration_cast<std::chrono::microseconds>(
            std::chrono::steady_clock::now().time_since_epoch()).count());
}

/// LogFn callback — prints to stderr when --verbose is enabled.
void verboseLog(const char* tag, const char* fmt, ...) {
    if (!g_verbose) return;
    const auto now = std::chrono::steady_clock::now();
    const auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(
        now.time_since_epoch()).count() % 1000;
    const auto t = std::time(nullptr);
    std::tm tm{};
    localtime_r(&t, &tm);
    std::fprintf(stderr, "[%02d:%02d:%02d.%03lld] [%s] ",
                 tm.tm_hour, tm.tm_min, tm.tm_sec, (long long)ms, tag);
    va_list ap;
    va_start(ap, fmt);
    std::vfprintf(stderr, fmt, ap);
    va_end(ap);
    std::fputc('\n', stderr);
}

} // namespace

int main(int argc, char** argv) {
    uint16_t port = 8080;
    std::string webRoot = WEB_DASHBOARD_DIST_DIR;

    // Parse simple command-line flags
    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];
        if ((arg == "--port" || arg == "-p") && i + 1 < argc) {
            port = static_cast<uint16_t>(std::stoi(argv[++i]));
        } else if ((arg == "--web-root" || arg == "-w") && i + 1 < argc) {
            webRoot = argv[++i];
        } else if (arg == "--verbose" || arg == "-v") {
            g_verbose = true;
        } else if (arg == "--help" || arg == "-h") {
            std::cout <<
                "web_dashboard_example — Tether IO web dashboard\n\n"
                "Usage: web_dashboard_example [OPTIONS]\n\n"
                "Options:\n"
                "  --port <N>, -p <N>       HTTP/WebSocket port (default 8080)\n"
                "  --web-root <PATH>, -w <PATH>  Dashboard static file directory\n"
                "                             (default: built-in dist)\n"
                "  --verbose, -v            Enable verbose protocol logging\n"
                "  --help, -h               Show this help\n\n"
                "Open http://127.0.0.1:<port>/ in a browser to view the dashboard.\n";
            return 0;
        }
    }

    installSignalHandlers();
    if (g_verbose) {
        std::cerr << "Verbose logging enabled (stderr)" << std::endl;
    }

    // -----------------------------------------------------------------------
    // Registry: parameters and signals
    // -----------------------------------------------------------------------
    Registry registry;

    // Mutable state (atomic so the UI writer thread and signal reader are safe)
    std::atomic<double> amplitude{1.0};
    std::atomic<double> frequency{0.5};
    std::atomic<double> sampleRate{100.0};
    std::atomic<double> phaseOffset{0.0};
    std::atomic<double> dcOffset{0.0};

    const auto started = std::chrono::steady_clock::now();
    const auto elapsed = [&started]() {
        return std::chrono::duration<double>(
            std::chrono::steady_clock::now() - started).count();
    };

    // Helper to register a writable F64 parameter backed by an atomic.
    const auto addAtomicParam = [&](uint64_t id, const char* name,
                                     const char* desc, const char* group,
                                     std::atomic<double>& storage,
                                     const char* unit = nullptr) {
        ParamEntry p;
        p.id = id;
        p.name = name;
        p.description = desc;
        p.group = group;
        p.valueType = ValueType::F64;
        p.readFn = [&storage](void* dst) {
            const double v = storage.load(std::memory_order_relaxed);
            std::memcpy(dst, &v, sizeof(v));
        };
        p.writeFn = [&storage](const void* src) {
            double v{};
            std::memcpy(&v, src, sizeof(v));
            storage.store(v, std::memory_order_relaxed);
        };
        if (unit) p.metadata["unit"] = unit;
        registry.addParam(std::move(p));
    };

    addAtomicParam(1, "amplitude",   "Sine/cosine wave amplitude",
                   "wave", amplitude, "V");
    addAtomicParam(2, "frequency",   "Wave frequency in Hz",
                   "wave", frequency, "Hz");
    addAtomicParam(3, "sample_rate", "Display sample rate",
                   "wave", sampleRate, "Hz");
    addAtomicParam(4, "phase_offset","Phase offset in radians",
                   "wave", phaseOffset, "rad");
    addAtomicParam(5, "dc_offset",   "DC offset added to both signals",
                   "wave", dcOffset, "V");

    // Helper to register a signal that reads the current wave state.
    const auto addWaveSignal = [&](uint64_t id, const char* name,
                                    const char* desc, bool isSine) {
        SignalEntry s;
        s.id = id;
        s.name = name;
        s.description = desc;
        s.group = "wave";
        s.valueType = ValueType::F64;
        s.readFn = [&, isSine](void* dst) {
            const double t = elapsed();
            const double a = amplitude.load(std::memory_order_relaxed);
            const double f = frequency.load(std::memory_order_relaxed);
            const double ph = phaseOffset.load(std::memory_order_relaxed);
            const double off = dcOffset.load(std::memory_order_relaxed);
            const double phase = 2.0 * M_PI * f * t + ph;
            const double value = off + a * (isSine
                ? std::sin(phase)
                : std::cos(phase));
            std::memcpy(dst, &value, sizeof(value));
        };
        s.metadata["unit"] = "V";
        registry.addSignal(std::move(s));
    };

    addWaveSignal(10, "sine_wave",   "Live sine wave signal", true);
    addWaveSignal(11, "cosine_wave", "Live cosine wave signal", false);

    // A simple monotonic ramp signal for timing/debugging
    registry.addSignal({12, "elapsed_time", "Elapsed time since start [s]",
        "wave", ValueType::F64,
        [&elapsed](void* dst) {
            const double v = elapsed();
            std::memcpy(dst, &v, sizeof(v));
        }});

    // Function: reset all wave parameters to defaults
    FunctionEntry resetFn;
    resetFn.id = 20;
    resetFn.name = "reset_params";
    resetFn.description = "Reset all wave parameters to their default values";
    resetFn.group = "wave";
    resetFn.callback = [&amplitude, &frequency, &sampleRate,
                         &phaseOffset, &dcOffset](
            const std::vector<FunctionArgument>&) {
        amplitude.store(1.0);
        frequency.store(0.5);
        sampleRate.store(100.0);
        phaseOffset.store(0.0);
        dcOffset.store(0.0);
        return FunctionCallResult{true, ErrorCode::None, {}, {}};
    };
    registry.addFunction(std::move(resetFn));

    // -----------------------------------------------------------------------
    // Drogon setup: static files + WebSocket
    // -----------------------------------------------------------------------

    // Register the binary WebSocket controller for the Tether IO protocol
    drogon::app().registerWebSocketController(
        "/tether-io", "tether::io::example::TetherIOWebSocketController", {});
    drogon::DrClassMap::setSingleInstance(
        std::make_shared<TetherIOWebSocketController>(registry, g_verbose ? verboseLog : nullptr));

    // Serve the pre-built dashboard from the web root directory
    drogon::app().setDocumentRoot(webRoot);
    drogon::app().setHomePage("index.html");

    // SPA fallback: serve index.html for any unmatched GET route so that
    // the dashboard works even if the user navigates to /signals etc.
    drogon::app().registerHandler("/",
        [&webRoot](const drogon::HttpRequestPtr&,
                   std::function<void(const drogon::HttpResponsePtr&)>&& cb) {
            auto resp = drogon::HttpResponse::newFileResponse(
                webRoot + "/index.html");
            cb(resp);
        }, {drogon::Get});

    std::cout <<
        "\n"
        "╔══════════════════════════════════════════════════════════╗\n"
        "║         Tether IO Web Dashboard Example                  ║\n"
        "╠══════════════════════════════════════════════════════════╣\n"
        "║  Dashboard:  http://127.0.0.1:" << port << "/                       ║\n"
        "║  WebSocket:  ws://127.0.0.1:" << port << "/tether-io               ║\n"
        "║  Static:     " << webRoot << "\n"
        "╠══════════════════════════════════════════════════════════╣\n"
        "║  Signals:    sine_wave, cosine_wave, elapsed_time        ║\n"
        "║  Params:     amplitude, frequency, sample_rate,          ║\n"
        "║              phase_offset, dc_offset                     ║\n"
        "║  Function:   reset_params                                ║\n"
        "╠══════════════════════════════════════════════════════════╣\n"
        "║  Press Ctrl+C to stop.                                   ║\n"
        "╚══════════════════════════════════════════════════════════╝\n"
        << std::endl;

    drogon::app().addListener("0.0.0.0", port);

    // Run Drogon in a separate thread so we can check for Ctrl+C
    std::thread drogonThread([] {
        drogon::app().run();
    });

    // Wait for Ctrl+C / SIGTERM
    while (!g_stopRequested.load()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    std::cout << "\nShutting down..." << std::endl;
    drogon::app().quit();
    drogonThread.join();

    return 0;
}

/**
 * @file tether_io_log_subscription.cpp
 * @brief Log subscriptions: server-published records with filtered fan-out.
 *
 * Server::publishLog() fans a LogRecord out to every session whose
 * subscriptions match. Client-side subscribeLog() sets the filter:
 *
 *   - minSeverity: records below it never arrive
 *   - componentFilter / messageFilter / locationFilter: substring
 *     contains(); empty matches everything
 *
 * LogData records dispatch inside receiveMessage() waits via
 * setLogDataCallback — same pump discipline as StreamData.
 *
 *   ./tether_io_log_subscription [--port 42790]
 */

#include "tether/io/ProducerHelpers.hpp"
#include "tether/io/Registry.hpp"
#include "tether/io/Server.hpp"
#include "tether/io/SlipTransport.hpp"
#include "tether/io/TcpTransport.hpp"
#include "tether/io/TetherIOClient.hpp"

#include <atomic>
#include <chrono>
#include <cstdio>
#include <string>
#include <thread>

using namespace tether::io;
using namespace std::chrono_literals;

int main(int argc, char** argv) {
    uint16_t port = 42790;
    for (int i = 1; i < argc; ++i)
        if (std::string(argv[i]) == "--port" && i + 1 < argc)
            port = static_cast<uint16_t>(std::stoi(argv[++i]));

    Registry registry;
    ServerConfig sc;
    sc.timestampFn = steadyClockUs;
    sc.framing = Framing::Slip;
    Server server(registry,
                  std::make_unique<TcpTransportServer>(port, "127.0.0.1"),
                  sc);
    if (!server.start()) { std::fprintf(stderr, "server start failed\n"); return 1; }

    int rc = 0;
    {
        auto transport = tcpConnect("127.0.0.1", port, 2000);
        if (!transport) rc = 1;
        if (rc == 0) {
            TetherIOClient client(
                std::make_unique<SlipFramedTransport>(std::move(transport)),
                2000);

            std::atomic<uint32_t> hits{0};
            client.setLogDataCallback([&](const ClientLogRecord& rec) {
                ++hits;
                std::printf("[client] sev=%u [%s] %s\n", rec.severity,
                            rec.component.c_str(), rec.message.c_str());
            });

            // Warn+ only, component containing "drive".
            const auto sub = client.subscribeLog(
                static_cast<uint8_t>(LogSeverity::Warn), "drive");
            if (!sub) { std::fprintf(stderr, "subscribeLog failed\n"); rc = 1; }
            std::this_thread::sleep_for(50ms);

            if (rc == 0) {
                server.publishLog(LogSeverity::Info, "drive.axis0",
                                  "info below threshold");      // dropped
                server.publishLog(LogSeverity::Error, "thermal",
                                  "overtemp");                  // wrong comp
                server.publishLog(LogSeverity::Error, "drive.axis0",
                                  "overcurrent at 3.2A", "axis0.cpp:412");
                server.publishLog(LogSeverity::Critical, "drive.bus",
                                  "link lost");

                for (int i = 0; i < 40 && hits < 2; ++i)
                    (void)client.receiveMessage(25);
                std::printf("[client] %u/4 records passed the filter\n",
                            hits.load());
                if (hits != 2) rc = 1;

                (void)client.unsubscribeLog(sub->subscriptionId);
            }
        }
    }
    server.stop();
    return rc;
}

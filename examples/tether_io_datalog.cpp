/**
 * @file tether_io_datalog.cpp
 * @brief Datalog: client-armed config, application-owned record sink.
 *
 * Server::datalogRecorder() is one DatalogRecorder shared by all
 * sessions. The wire RPC configureDatalog(logName, rateHz, enabled,
 * entryIds) lets a client arm WHAT is recorded and WHEN — the session
 * configures the recorder with a registry readFn and a null sink, so
 * the RPC alone emits nothing. The application owns WHERE records go:
 * it calls configure() with a DatalogSinkFn and pumps sampleOnce() at
 * the configured rate. Each record is [u64 ts][entry values in
 * entryIds order]; status() reports recordsWritten/state.
 *
 *   ./tether_io_datalog [--port 42792]
 */

#include "tether/io/Datalogging.hpp"
#include "tether/io/ProducerHelpers.hpp"
#include "tether/io/Registry.hpp"
#include "tether/io/Server.hpp"
#include "tether/io/SlipTransport.hpp"
#include "tether/io/TcpTransport.hpp"
#include "tether/io/TetherIOClient.hpp"

#include <atomic>
#include <chrono>
#include <cstring>
#include <cstdio>
#include <string>
#include <thread>
#include <vector>

using namespace tether::io;
using namespace std::chrono_literals;

int main(int argc, char** argv) {
    uint16_t port = 42792;
    for (int i = 1; i < argc; ++i)
        if (std::string(argv[i]) == "--port" && i + 1 < argc)
            port = static_cast<uint16_t>(std::stoi(argv[++i]));

    std::atomic<double> pos{0.0};
    Registry registry;
    {
        SignalEntry s;
        s.id = 1;
        s.name = "drive.pos";
        s.group = "drive";
        s.valueType = ValueType::F64;
        s.readFn = [&pos](void* d) {
            const double v = pos.load();
            std::memcpy(d, &v, 8);
        };
        if (!registry.addSignal(s)) return 1;
    }

    ServerConfig sc;
    sc.timestampFn = steadyClockUs;
    sc.framing = Framing::Slip;
    Server server(registry,
                  std::make_unique<TcpTransportServer>(port, "127.0.0.1"),
                  sc);
    if (!server.start()) { std::fprintf(stderr, "server start failed\n"); return 1; }

    DatalogRecorder* rec = server.datalogRecorder();

    // Application-owned output: a sink counting rows plus a 1 kHz pump
    // that samples whatever config is currently armed. Re-arming via
    // the wire replaces the config (and the sink), so the sampler
    // re-attaches the sink whenever the armed config changes.
    std::vector<std::vector<uint8_t>> rows;
    std::mutex rowsMtx;
    auto attachSink = [&] {
        DatalogConfig cfg;
        cfg.logName = rec->metadata().logName;
        cfg.sampleRateHz = rec->metadata().sampleRateHz;
        for (const auto& f : rec->metadata().fields)
            cfg.entryIds.push_back(f.entryId);
        cfg.enabled = true;
        rec->configure(
            cfg,
            [&registry](uint64_t id, void* d, size_t max) -> bool {
                EntryView e = registry.find(id);
                if (!e || e.isVariableLength() ||
                    e.valueSize() > max)
                    return false;
                e.read(d);
                return true;
            },
            steadyClockUs,
            [&](const uint8_t* d, size_t n) {
                std::lock_guard lk(rowsMtx);
                rows.emplace_back(d, d + n);
            });
        rec->start();
    };

    std::atomic<bool> run{true};
    std::thread sampler([&] {
        std::string lastName;
        while (run) {
            if (rec->status().state == DatalogState::Recording &&
                rec->metadata().logName != lastName) {
                lastName = rec->metadata().logName;
                attachSink();
            }
            pos.store(pos.load() + 0.001);
            rec->sampleOnce();
            std::this_thread::sleep_for(1ms);
        }
    });

    int rc = 0;
    {
        auto transport = tcpConnect("127.0.0.1", port, 2000);
        if (!transport) rc = 1;
        if (rc == 0) {
            TetherIOClient client(
                std::make_unique<SlipFramedTransport>(std::move(transport)),
                2000);

            // Arm + start: log "run1" at 1 kHz over entry id 1.
            if (!client.configureDatalog("run1", 1000, true, {1})) {
                std::fprintf(stderr, "configureDatalog failed\n");
                rc = 1;
            }
            std::this_thread::sleep_for(150ms);
            std::printf("[app] armed name=%s rate=%u state=%d wrote=%llu\n",
                        rec->metadata().logName.c_str(),
                        rec->metadata().sampleRateHz,
                        static_cast<int>(rec->status().state),
                        static_cast<unsigned long long>(
                            rec->status().recordsWritten));

            // Disarm: enabled=false stops the recorder (and a fresh
            // configure() resets the counters).
            (void)client.configureDatalog("run1", 1000, false, {1});
            std::this_thread::sleep_for(50ms);
        }
    }

    run = false;
    sampler.join();
    {
        std::lock_guard lk(rowsMtx);
        std::printf("[app] sink received %zu rows (final state=%d)\n",
                    rows.size(), static_cast<int>(rec->status().state));
        if (!rows.empty() && rows[0].size() >= 16) {
            uint64_t ts;
            double v;
            std::memcpy(&ts, rows[0].data(), 8);
            std::memcpy(&v, rows[0].data() + 8, 8);
            std::printf("[app] first row: ts=%llu pos=%.3f\n",
                        static_cast<unsigned long long>(ts), v);
        }
        if (rows.size() < 50) rc = 1;
    }
    server.stop();
    return rc;
}

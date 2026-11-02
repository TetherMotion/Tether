/**
 * @file tether_io_param_varlen.cpp
 * @brief Variable-length params: strings/binary blobs over get/setParam.
 *
 * Fixed-size params use readFn/writeFn with a canonical byte width;
 * String/Binary params instead use varReadFn (writes up to maxValueSize
 * bytes, returns actual length) and varWriteFn (receives the raw
 * payload). The wire carries a length prefix. A missing write side
 * makes the entry read-only — setParam fails at the boundary, not in
 * the callback.
 *
 *   ./tether_io_param_varlen [--port 42795]
 */

#include "tether/io/ProducerHelpers.hpp"
#include "tether/io/Registry.hpp"
#include "tether/io/Server.hpp"
#include "tether/io/SlipTransport.hpp"
#include "tether/io/TcpTransport.hpp"
#include "tether/io/TetherIOClient.hpp"

#include <algorithm>
#include <cstring>
#include <cstdio>
#include <mutex>
#include <string>

using namespace tether::io;

int main(int argc, char** argv) {
    uint16_t port = 42795;
    for (int i = 1; i < argc; ++i)
        if (std::string(argv[i]) == "--port" && i + 1 < argc)
            port = static_cast<uint16_t>(std::stoi(argv[++i]));

    std::mutex mtx;
    std::string devName = "vesoma-01";
    const std::string serial = "VSM-2026-0042";

    Registry registry;
    {
        // RW string param
        ParamEntry p;
        p.id = 1;
        p.name = "device.name";
        p.description = "Configurable device name";
        p.group = "sys";
        p.valueType = ValueType::String;
        p.maxValueSize = 64;
        p.varReadFn = [&](void* d, size_t maxLen) -> size_t {
            std::lock_guard lk(mtx);
            const size_t n = std::min(devName.size(), maxLen);
            std::memcpy(d, devName.data(), n);
            return n;
        };
        p.varWriteFn = [&](const void* s, size_t len) {
            std::lock_guard lk(mtx);
            devName.assign(static_cast<const char*>(s), len);
        };
        if (!registry.addParam(p)) return 1;

        // RO binary param — no write side at all.
        ParamEntry q;
        q.id = 2;
        q.name = "device.serial";
        q.description = "Factory serial (read-only)";
        q.group = "sys";
        q.valueType = ValueType::Binary;
        q.maxValueSize = 32;
        q.varReadFn = [&](void* d, size_t maxLen) -> size_t {
            const size_t n = std::min(serial.size(), maxLen);
            std::memcpy(d, serial.data(), n);
            return n;
        };
        if (!registry.addParam(q)) return 1;
    }

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
            auto getStr = [&](uint64_t id) {
                const auto v = client.getParam(id);
                return v ? std::string(
                               reinterpret_cast<const char*>(v->data()),
                               v->size())
                         : std::string{"<err>"};
            };
            std::printf("[client] device.name   = %s\n", getStr(1).c_str());
            std::printf("[client] device.serial = %s\n", getStr(2).c_str());

            const std::string nn = "vesoma-lab";
            if (!client.setParamVar(1, nn.data(), nn.size())) {
                std::fprintf(stderr, "setParamVar(device.name) failed\n");
                rc = 1;
            }
            std::printf("[client] after set: device.name = %s\n",
                        getStr(1).c_str());

            // Read-only: the write is rejected at the entry boundary.
            const auto ro = client.setParamVar(2, serial.data(),
                                               serial.size());
            std::printf("[client] write to RO serial -> %s\n",
                        ro ? "accepted (BAD)" : "rejected");
            if (ro) rc = 1;
        }
    }
    server.stop();
    return rc;
}

/**
 * @file Gateway.hpp
 * @brief Modbus TCP↔RTU gateway: TCP server front-end that forwards each
 *        request PDU to a downstream bus (an IModbusInvoker — usually the
 *        serialized RTU executor) and relays the response.
 */
#pragma once

#include "tether/modbus/client/Invoker.hpp"
#include "tether/modbus/server/Server.hpp"

namespace tether::modbus {

struct GatewayOpts {
    int downstreamTimeoutMs = 1000;
    int downstreamRetries   = 0;
    /// Downstream unit mapping: default identity; optional remap hook.
    std::function<uint8_t(uint8_t)> mapUnit;
    /// Serialize downstream access across gateway worker threads.
    bool serialize = true;
};

/**
 * @brief Serves TCP clients and forwards PDUs downstream.
 *
 * Requests are serialized through a mutex by default (RTU buses are
 * single-master). Exception PDUs from downstream are relayed verbatim;
 * transport-level failures → ServerDeviceFailure (0x04).
 */
class Gateway {
public:
    Gateway(std::unique_ptr<ILinkListener> listener,
            IModbusInvoker& downstream,
            GatewayOpts opts = {},
            size_t maxClients = 8);
    ~Gateway() { stop(); }

    Expected<void> start();
    void           stop();
    bool           running() const { return running_; }
    ServerStats&   stats() { return stats_; }

    /// Serve one request synchronously (test/sim path).
    std::vector<uint8_t> forward(uint8_t unit,
                                 std::span<const uint8_t> reqPdu);

private:
    void serveClient(std::unique_ptr<ILink> link);

    std::unique_ptr<ILinkListener> listener_;
    IModbusInvoker&                downstream_;
    GatewayOpts                    opts_;
    size_t                         maxClients_;
    std::mutex                     downstreamMu_;
    std::atomic<bool>              running_{false};
    std::atomic<bool>              stop_{false};
    std::thread                    acceptTh_;
    std::vector<std::thread>       workers_;
    std::mutex                     workersM_;
    ServerStats                    stats_;
};

} // namespace tether::modbus

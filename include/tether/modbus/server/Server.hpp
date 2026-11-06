/**
 * @file Server.hpp
 * @brief ModbusServer (RTU slave on serial) + TcpServer (multi-client).
 */
#pragma once

#include <atomic>
#include <memory>
#include <thread>
#include <vector>

#include "tether/modbus/link/Channel.hpp"
#include "tether/modbus/link/TcpLink.hpp"
#include "tether/modbus/server/Dispatcher.hpp"

namespace tether::modbus {

struct ServerStats {
    std::atomic<uint64_t> requests{0};
    std::atomic<uint64_t> responses{0};
    std::atomic<uint64_t> exceptions{0};
    std::atomic<uint64_t> broadcasts{0};
    std::atomic<uint64_t> badFrames{0};
};

/**@brief RTU slave: serve a dispatcher over an RtuChannel. */
class ModbusServer {
public:
    ModbusServer(std::unique_ptr<RtuChannel> ch, RequestDispatcher& disp,
                 int turnaroundDelayMs = 0)
        : ch_(std::move(ch)), disp_(disp),
          turnaroundMs_(turnaroundDelayMs) {}

    Expected<void> start();          ///< open link + spawn serve thread
    void           stop();           ///< signal + join (unblocks read)
    bool           running() const { return running_; }
    ServerStats&   stats() { return stats_; }
    /// One iteration of the serve loop (test-friendly).
    Expected<void> serveOnce(int timeoutMs);
    /// Process one already-received request ADU.
    std::vector<uint8_t> handleRequest(const Adu& req);

private:
    std::unique_ptr<RtuChannel> ch_;
    RequestDispatcher&          disp_;
    int                         turnaroundMs_ = 0;
    std::atomic<bool>           running_{false};
    std::atomic<bool>           stop_{false};
    std::thread                 th_;
    ServerStats                 stats_;
};

/**@brief Modbus/TCP server: listener + a worker thread per connection. */
class TcpServer {
public:
    TcpServer(std::unique_ptr<ILinkListener> listener,
              RequestDispatcher& disp,
              size_t maxClients = 8)
        : listener_(std::move(listener)), disp_(disp),
          maxClients_(maxClients) {}
    ~TcpServer() { stop(); }

    Expected<void> start();
    void           stop();
    bool           running() const { return running_; }
    size_t         clientCount() const;
    ServerStats&   stats() { return stats_; }

private:
    void serveClient(std::unique_ptr<ILink> link);

    std::unique_ptr<ILinkListener> listener_;
    RequestDispatcher&             disp_;
    size_t                         maxClients_;
    std::atomic<bool>              running_{false};
    std::atomic<bool>              stop_{false};
    std::thread                    acceptTh_;
    std::vector<std::thread>       workers_;
    std::mutex                     workersM_;
    ServerStats                    stats_;
};

} // namespace tether::modbus

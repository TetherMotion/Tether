#include "tether/modbus/server/Server.hpp"

#include <thread>

namespace tether::modbus {

// ---- ModbusServer (RTU) --------------------------------------------------------
Expected<void> ModbusServer::start() {
    if (running_) return {};
    if (auto e = ch_->open(); !e) return e;
    stop_ = false;
    running_ = true;
    th_ = std::thread([this] {
        while (!stop_) {
            auto r = serveOnce(50);
            if (!r && r.error().isTransport()) break;   // link died
        }
        running_ = false;
    });
    return {};
}

void ModbusServer::stop() {
    stop_ = true;
    ch_->close();          // unblock read
    if (th_.joinable()) th_.join();
    running_ = false;
}

std::vector<uint8_t> ModbusServer::handleRequest(const Adu& req) {
    stats_.requests++;
    auto res = disp_.dispatch(req.unit, req.pdu);
    if (!res.respond) { stats_.broadcasts++; return {}; }
    stats_.responses++;
    if (res.pdu.size() >= 2 && (res.pdu[0] & 0x80)) stats_.exceptions++;
    return std::move(res.pdu);
}

Expected<void> ModbusServer::serveOnce(int timeoutMs) {
    auto req = ch_->receiveFrame(timeoutMs);
    if (!req) {
        if (req.error().isTimeout()) return {};   // idle poll
        return err<void>(req.error());
    }
    auto rspPdu = handleRequest(*req);
    if (rspPdu.empty()) return {};                // broadcast / filtered
    if (turnaroundMs_ > 0)
        std::this_thread::sleep_for(
            std::chrono::milliseconds(turnaroundMs_));
    return ch_->sendOnly(req->unit, rspPdu);
}

// ---- TcpServer -------------------------------------------------------------------
Expected<void> TcpServer::start() {
    if (running_) return {};
    if (auto e = listener_->listen(); !e) return e;
    stop_ = false;
    running_ = true;
    acceptTh_ = std::thread([this] {
        while (!stop_) {
            auto link = listener_->accept(50);
            if (!link) continue;
            std::lock_guard g(workersM_);
            if (workers_.size() >= maxClients_) { (*link)->close(); continue; }
            workers_.emplace_back([this, l = std::move(*link)]() mutable {
                serveClient(std::move(l));
            });
        }
        running_ = false;
    });
    return {};
}

void TcpServer::serveClient(std::unique_ptr<ILink> link) {
    MbapChannel ch(std::move(link));
    while (!stop_) {
        auto req = ch.receiveAny(50);
        if (!req) {
            if (req.error().isTransport()) return;   // client gone
            continue;
        }
        stats_.requests++;
        auto res = disp_.dispatch(req->unit, req->pdu);
        if (!res.respond) { stats_.broadcasts++; continue; }
        stats_.responses++;
        if (res.pdu.size() >= 2 && (res.pdu[0] & 0x80)) stats_.exceptions++;
        if (auto e = ch.sendAdu(*req, res.pdu); !e) return;
    }
}

void TcpServer::stop() {
    stop_ = true;
    listener_->close();   // unblock accept
    if (acceptTh_.joinable()) acceptTh_.join();
    // Connections unblock via stop_ poll (50ms slices).
    std::lock_guard g(workersM_);
    for (auto& t : workers_) if (t.joinable()) t.join();
    workers_.clear();
    running_ = false;
}

size_t TcpServer::clientCount() const {
    std::lock_guard g(const_cast<std::mutex&>(workersM_));
    return workers_.size();
}

} // namespace tether::modbus

#include "tether/modbus/server/Gateway.hpp"

namespace tether::modbus {

Gateway::Gateway(std::unique_ptr<ILinkListener> listener,
                 IModbusInvoker& downstream, GatewayOpts opts,
                 size_t maxClients)
    : listener_(std::move(listener)), downstream_(downstream),
      opts_(std::move(opts)), maxClients_(maxClients) {}

std::vector<uint8_t> Gateway::forward(uint8_t unit,
                                    std::span<const uint8_t> reqPdu) {
    stats_.requests++;
    if (reqPdu.empty()) return {};
    const auto fc = static_cast<FunctionCode>(reqPdu[0]);
    const auto body = reqPdu.subspan(1);
    const uint8_t dunit = opts_.mapUnit ? opts_.mapUnit(unit) : unit;

    InvokeOptions io{opts_.downstreamTimeoutMs, opts_.downstreamRetries,
                     0, {}};
    auto invoke = [&]() -> Expected<Pdu> {
        if (opts_.serialize) {
            std::lock_guard g(downstreamMu_);
            return downstream_.invoke(dunit, fc, body, io);
        }
        return downstream_.invoke(dunit, fc, body, io);
    };

    auto rsp = invoke();
    stats_.responses++;
    if (!rsp) {
        stats_.exceptions++;
        const auto& e = rsp.error();
        const uint8_t code = e.isException()
            ? uint8_t(e.code() & 0xFF)
            : uint8_t(ExceptionCode::ServerDeviceFailure);
        return {uint8_t(uint8_t(fc) | 0x80), code};
    }
    return {rsp->bytes().begin(), rsp->bytes().end()};
}

Expected<void> Gateway::start() {
    if (running_) return {};
    if (!listener_)
        return err(ModbusError::config("gateway: no listener"));
    if (auto e = listener_->listen(); !e) return e;
    stop_ = false;
    running_ = true;
    acceptTh_ = std::thread([this] {
        while (!stop_) {
            auto link = listener_->accept(50);
            if (!link) continue;
            std::lock_guard g(workersM_);
            if (workers_.size() >= maxClients_) {
                (*link)->close();
                continue;
            }
            workers_.emplace_back([this, l = std::move(*link)]() mutable {
                serveClient(std::move(l));
            });
        }
        running_ = false;
    });
    return {};
}

void Gateway::serveClient(std::unique_ptr<ILink> link) {
    MbapChannel ch(std::move(link));
    while (!stop_) {
        auto req = ch.receiveAny(50);
        if (!req) {
            if (req.error().isTransport()) return;
            continue;
        }
        auto rspPdu = forward(req->unit, req->pdu);
        if (rspPdu.empty()) continue;    // broadcast
        if (auto e = ch.sendAdu(*req, rspPdu); !e) return;
    }
}

void Gateway::stop() {
    stop_ = true;
    if (listener_) listener_->close();
    if (acceptTh_.joinable()) acceptTh_.join();
    std::lock_guard g(workersM_);
    for (auto& t : workers_) if (t.joinable()) t.join();
    workers_.clear();
    running_ = false;
}

} // namespace tether::modbus

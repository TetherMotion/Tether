#include "tether/modbus/sim/Sim.hpp"

#include <thread>

#include "tether/modbus/protocol/ExceptionCode.hpp"

namespace tether::modbus {

std::optional<std::vector<uint8_t>>
SimSlave::handlePdu(std::span<const uint8_t> reqPdu) {
    if (traits_.delayMs > 0)
        std::this_thread::sleep_for(
            std::chrono::milliseconds(traits_.delayMs));
    if (uni_(rng_) < traits_.dropRequest) return std::nullopt;
    if (uni_(rng_) < traits_.excBusy)
        return std::vector<uint8_t>{
            uint8_t(reqPdu.empty() ? 0x80 : (reqPdu[0] | 0x80)),
            uint8_t(ExceptionCode::ServerDeviceBusy)};
    auto res = disp_.dispatch(unit_, reqPdu);
    if (!res.respond) return std::vector<uint8_t>{};
    if (uni_(rng_) < traits_.dropResponse) return std::nullopt;
    if (uni_(rng_) < traits_.corrupt && res.pdu.size() > 1)
        res.pdu[1 + rng_() % (res.pdu.size() - 1)] ^= 0x55;
    return res.pdu;
}

Expected<Pdu> SimInvoker::invoke(uint8_t unit, FunctionCode fc,
                                 std::span<const uint8_t> body,
                                 const TransactionOptions&) {
    if (checkUnit_ && unit != slave_.unit())
        return err<Pdu>(ModbusError::timedOut("no such unit"));
    std::vector<uint8_t> req(body.size() + 1);
    req[0] = uint8_t(fc);
    std::copy(body.begin(), body.end(), req.begin() + 1);
    auto rsp = slave_.handlePdu(req);
    if (!rsp) return err<Pdu>(ModbusError::timedOut("sim drop"));
    if (rsp->empty())
        return err<Pdu>(ModbusError::timedOut("broadcast"));
    const uint8_t rfc = (*rsp)[0];
    if (rfc == (uint8_t(fc) | 0x80))
        return err<Pdu>(exceptionToError(rsp->size() > 1 ? (*rsp)[1] : 0));
    if (rfc != uint8_t(fc))
        return err<Pdu>(ModbusError::codec("sim fc mismatch"));
    Pdu p{fc};
    p.append(std::span(rsp->data() + 1, rsp->size() - 1));
    return p;
}

SimSlave& SimBus::add(uint8_t unit, SimTraits t) {
    auto s = std::make_unique<SimSlave>(unit, t);
    auto& ref = *s;
    slaves_[unit] = std::move(s);
    return ref;
}
SimSlave* SimBus::find(uint8_t unit) {
    auto it = slaves_.find(unit);
    return it == slaves_.end() ? nullptr : it->second.get();
}
Expected<Pdu> SimBus::BusInvoker::invoke(uint8_t u, FunctionCode fc,
                                         std::span<const uint8_t> b,
                                         const TransactionOptions& o) {
    auto* s = bus_.find(u);
    if (!s) return err<Pdu>(ModbusError::timedOut("no slave"));
    std::vector<uint8_t> req(b.size() + 1);
    req[0] = uint8_t(fc);
    std::copy(b.begin(), b.end(), req.begin() + 1);
    auto rsp = s->handlePdu(req);
    if (!rsp || rsp->empty())
        return err<Pdu>(ModbusError::timedOut("sim"));
    if ((*rsp)[0] == (uint8_t(fc) | 0x80))
        return err<Pdu>(exceptionToError(rsp->size() > 1 ? (*rsp)[1] : 0));
    Pdu p{fc};
    p.append(std::span(rsp->data() + 1, rsp->size() - 1));
    return p;
}

// ---- SimLink --------------------------------------------------------------------
#include "tether/modbus/protocol/Crc.hpp"

Expected<void> SimLink::write(std::span<const uint8_t> bytes, int) {
    if (!open_) return err(ModbusError::transport("sim link closed"));
    if (tap_) tap_(true, bytes);
    std::vector<uint8_t> wire(bytes.begin(), bytes.end());
    if (traits_.mutateRequest) traits_.mutateRequest(wire);
    std::uniform_real_distribution<double> uni{0.0, 1.0};
    if (uni(rng_) < traits_.dropRequest) return {};

    // --- decode one frame -------------------------------------------------
    uint8_t unit = 0;
    uint16_t txn = 0;
    std::vector<uint8_t> pdu;
    if (rtu_) {
        if (wire.size() < 4) return {};
        unit = wire[0];
        pdu.assign(wire.begin() + 1, wire.end() - 2);   // lenient re: CRC
    } else {
        if (wire.size() < 8) return {};
        txn  = uint16_t(wire[0] << 8) | wire[1];
        unit = wire[6];
        pdu.assign(wire.begin() + 7, wire.end());
    }

    auto* slave = bus_.find(unit);
    if (!slave) return {};                        // no device → silence
    auto rsp = slave->handlePdu(pdu);
    if (!rsp || rsp->empty()) return {};          // dropped / broadcast

    if (uni(rng_) < traits_.dropResponse) return {};
    if (traits_.turnaroundMs > 0)
        std::this_thread::sleep_for(
            std::chrono::milliseconds(traits_.turnaroundMs));

    // --- encode response frame ---------------------------------------------
    std::vector<uint8_t> frame;
    if (rtu_) {
        frame.push_back(unit);
        frame.insert(frame.end(), rsp->begin(), rsp->end());
        const uint16_t crc = Crc16::of(frame);
        frame.push_back(uint8_t(crc & 0xFF));
        frame.push_back(uint8_t(crc >> 8));
    } else {
        const uint16_t len = uint16_t(rsp->size() + 1);
        frame = {uint8_t(txn >> 8), uint8_t(txn), 0, 0,
                 uint8_t(len >> 8), uint8_t(len), unit};
        frame.insert(frame.end(), rsp->begin(), rsp->end());
    }
    if (traits_.mutateResponse) traits_.mutateResponse(frame);
    // probabilistic single-byte faults
    if (!frame.empty()) {
        if (uni(rng_) < traits_.corruptByte)
            frame[rng_() % frame.size()] ^= 0x55;
        if (uni(rng_) < traits_.dropByte && frame.size() > 1)
            frame.erase(frame.begin() + int(rng_() % frame.size()));
        if (uni(rng_) < traits_.dupByte) {
            const size_t i = rng_() % frame.size();
            frame.insert(frame.begin() + int(i), frame[i]);
        }
    }
    for (uint8_t b : frame) rx_.push_back(b);
    return {};
}

Expected<size_t> SimLink::read(std::span<uint8_t> buf, int timeoutMs) {
    if (!open_) return err<size_t>(ModbusError::transport("closed"));
    if (rx_.empty()) {
        // Nothing staged — responses appear synchronously in write(),
        // so empty here means "no response": burn a small slice of the
        // deadline instead of spinning the caller.
        std::this_thread::sleep_for(std::chrono::milliseconds(
            std::min(timeoutMs, 5)));
        return size_t{0};
    }
    size_t n = std::min(buf.size(), rx_.size());
    if (traits_.dribbleBytes > 0)
        n = std::min<size_t>(n, size_t(traits_.dribbleBytes));
    for (size_t i = 0; i < n; ++i) {
        buf[i] = rx_.front();
        rx_.pop_front();
    }
    if (tap_) tap_(false, buf.first(n));
    return n;
}

} // namespace tether::modbus

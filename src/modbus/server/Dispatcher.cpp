#include "tether/modbus/server/Dispatcher.hpp"

#include "tether/modbus/protocol/BitPacker.hpp"

namespace tether::modbus {
namespace {
std::vector<uint8_t> excPdu(FunctionCode fc, uint8_t code) {
    return {uint8_t(uint8_t(fc) | 0x80), code};
}
} // namespace

RequestDispatcher::Result
RequestDispatcher::dispatch(uint8_t unit, std::span<const uint8_t> reqPdu) {
    const bool broadcast = (unit == 0);
    // Broadcast (unit 0) addresses every slave on the bus — the unit-id
    // filter only applies to addressed requests.
    if (!broadcast && !unitOk(unit))
        return {false, {}};

    if (reqPdu.empty())
        return {false, {}};
    const auto fcByte = reqPdu[0];
    const auto fc = static_cast<FunctionCode>(fcByte);
    if (broadcast && !opts_.allowBroadcastWrites)
        return {false, {}};

    // Decode (validates shapes + quantities).
    auto req = decodeRequest(fc, reqPdu.subspan(1));
    if (!req) {
        if (broadcast) return {false, {}};
        return {true, excPdu(fc, uint8_t(ExceptionCode::IllegalDataValue))};
    }
    if (opts_.allow && !opts_.allow(unit, fc)) {
        if (broadcast) return {false, {}};
        return {true, excPdu(fc, uint8_t(ExceptionCode::IllegalFunction))};
    }

    auto rsp = handle(unit, fc, *req);
    if (broadcast) return {false, {}};           // never respond to bcast
    if (!rsp) {
        const auto& e = rsp.error();
        const uint8_t code = e.isException() ? uint8_t(e.code() & 0xFF)
                             : uint8_t(ExceptionCode::ServerDeviceFailure);
        return {true, excPdu(fc, code)};
    }
    auto body = encodeResponse(fc, *rsp, *req);
    if (!body)
        return {true,
                excPdu(fc, uint8_t(ExceptionCode::ServerDeviceFailure))};
    std::vector<uint8_t> pdu;
    pdu.reserve(body->size() + 1);
    pdu.push_back(uint8_t(fc));
    pdu.insert(pdu.end(), body->begin(), body->end());
    return {true, std::move(pdu)};
}

Expected<Response> RequestDispatcher::handle(uint8_t unit, FunctionCode fc,
                                             const Request& req) {
    switch (fc) {
        case FunctionCode::ReadCoils:
            return doRead(TableId::Coil, fc,
                          std::get<ReadRangeRequest>(req));
        case FunctionCode::ReadDiscreteInputs:
            return doRead(TableId::DiscreteInput, fc,
                          std::get<ReadRangeRequest>(req));
        case FunctionCode::ReadHoldingRegisters:
            return doRead(TableId::HoldingRegister, fc,
                          std::get<ReadRangeRequest>(req));
        case FunctionCode::ReadInputRegisters:
            return doRead(TableId::InputRegister, fc,
                          std::get<ReadRangeRequest>(req));
        case FunctionCode::WriteSingleCoil:
        case FunctionCode::WriteSingleRegister:
        case FunctionCode::WriteMultipleCoils:
        case FunctionCode::WriteMultipleRegisters:
        case FunctionCode::MaskWriteRegister:
        case FunctionCode::ReadWriteMultipleRegs:
            return doWrite(unit, fc == FunctionCode::ReadWriteMultipleRegs
                               ? TableId::HoldingRegister
                           : fc == FunctionCode::WriteSingleCoil ||
                             fc == FunctionCode::WriteMultipleCoils
                               ? TableId::Coil
                               : TableId::HoldingRegister, fc, req);
        case FunctionCode::Diagnostics: {
            const auto& d = std::get<DiagRequest>(req);
            if (opts_.onDiagnostics)
                return opts_.onDiagnostics(unit, d.sub, d.data);
            return err<Response>(exceptionToError(
                uint8_t(ExceptionCode::IllegalFunction)));
        }
        case FunctionCode::ReportServerId: {
            ServerIdResponse r;
            r.id = opts_.onServerId ? opts_.onServerId(unit)
                                    : std::vector<uint8_t>{};
            return Response{std::move(r)};
        }
        case FunctionCode::EncapsulatedInterface: {
            const auto& d = std::get<DevIdRequest>(req);
            if (opts_.onDeviceId) return opts_.onDeviceId(unit, d);
            return err<Response>(exceptionToError(
                uint8_t(ExceptionCode::IllegalFunction)));
        }
        default:
            return err<Response>(exceptionToError(
                uint8_t(ExceptionCode::IllegalFunction)));
    }
}

Expected<Response>
RequestDispatcher::doRead(TableId t, FunctionCode fc,
                          const ReadRangeRequest& r) {
    if (r.qty == 0)
        return err<Response>(exceptionToError(
            uint8_t(ExceptionCode::IllegalDataValue)));
    if (!model_.covers(t, r.addr, r.qty))
        return err<Response>(exceptionToError(
            uint8_t(ExceptionCode::IllegalDataAddress)));
    if (isBitTable(t)) {
        std::vector<uint8_t> tmp(r.qty);
        if (auto e = model_.read(t, r.addr, r.qty, {},
                                 std::span(tmp.data(), tmp.size())); !e)
            return err<Response>(e.error());
        BitsResponse br;
        br.bits.reserve(r.qty);
        for (uint16_t i = 0; i < r.qty; ++i) br.bits.push_back(tmp[i] != 0);
        return Response{std::move(br)};
    }
    RegistersResponse rr;
    rr.regs.resize(r.qty);
    if (auto e = model_.read(t, r.addr, r.qty,
                             std::span(rr.regs.data(), rr.regs.size()), {});
        !e)
        return err<Response>(e.error());
    (void)fc;
    return Response{std::move(rr)};
}

Expected<Response> RequestDispatcher::doWrite(uint8_t, TableId t,
                                              FunctionCode fc,
                                              const Request& req) {
    auto checkCover = [&](uint16_t a, uint16_t n) -> Expected<void> {
        if (!model_.covers(t, a, n))
            return err<void>(exceptionToError(
                uint8_t(ExceptionCode::IllegalDataAddress)));
        return {};
    };
    switch (fc) {
        case FunctionCode::WriteSingleCoil: {
            const auto& w = std::get<WriteSingleCoilRequest>(req);
            if (auto e = checkCover(w.addr, 1); !e)
                return err<Response>(e.error());
            const uint8_t b = w.on ? 1 : 0;
            if (auto e = model_.write(t, w.addr, 1, {},
                                      std::span(&b, 1)); !e)
                return err<Response>(e.error());
            return Response{EchoResponse{w.addr,
                            uint16_t(w.on ? 0xFF00 : 0x0000)}};
        }
        case FunctionCode::WriteSingleRegister: {
            const auto& w = std::get<WriteSingleRegRequest>(req);
            if (auto e = checkCover(w.addr, 1); !e)
                return err<Response>(e.error());
            if (auto e = model_.write(t, w.addr, 1,
                                      std::span(&w.val, 1), {}); !e)
                return err<Response>(e.error());
            return Response{EchoResponse{w.addr, w.val}};
        }
        case FunctionCode::WriteMultipleCoils: {
            const auto& w = std::get<WriteMultiCoilsRequest>(req);
            if (w.qty == 0 || w.qty > kMaxWriteBitsQty ||
                w.qty != w.bits.size())
                return err<Response>(exceptionToError(
                    uint8_t(ExceptionCode::IllegalDataValue)));
            if (auto e = checkCover(w.addr, w.qty); !e)
                return err<Response>(e.error());
            std::vector<uint8_t> bytes(w.qty);
            for (uint16_t i = 0; i < w.qty; ++i)
                bytes[i] = w.bits[i] ? 1 : 0;
            if (auto e = model_.write(t, w.addr, w.qty, {},
                                      std::span(bytes.data(), bytes.size()));
                !e)
                return err<Response>(e.error());
            return Response{RangeAckResponse{w.addr, w.qty}};
        }
        case FunctionCode::WriteMultipleRegisters: {
            const auto& w = std::get<WriteMultiRegsRequest>(req);
            if (w.qty == 0 || w.qty > kMaxWriteRegsQty ||
                w.qty != w.regs.size())
                return err<Response>(exceptionToError(
                    uint8_t(ExceptionCode::IllegalDataValue)));
            if (auto e = checkCover(w.addr, w.qty); !e)
                return err<Response>(e.error());
            if (auto e = model_.write(t, w.addr, w.qty,
                                      std::span(w.regs.data(), w.regs.size()),
                                      {});
                !e)
                return err<Response>(e.error());
            return Response{RangeAckResponse{w.addr, w.qty}};
        }
        case FunctionCode::MaskWriteRegister: {
            const auto& w = std::get<MaskWriteRequest>(req);
            if (auto e = checkCover(w.addr, 1); !e)
                return err<Response>(e.error());
            uint16_t cur = 0;
            if (auto e = model_.read(t, w.addr, 1, std::span(&cur, 1), {});
                !e)
                return err<Response>(e.error());
            const uint16_t next = uint16_t((cur & w.andMask) |
                                           (w.orMask & ~w.andMask));
            if (auto e = model_.write(t, w.addr, 1, std::span(&next, 1), {});
                !e)
                return err<Response>(e.error());
            return Response{EchoResponse{w.addr, w.andMask}};
        }
        case FunctionCode::ReadWriteMultipleRegs: {
            const auto& w = std::get<ReadWriteMultiRequest>(req);
            if (w.rqty == 0 || w.rqty > kMaxRwReadQty || w.wqty == 0 ||
                w.wqty > kMaxRwWriteQty || w.wqty != w.regs.size())
                return err<Response>(exceptionToError(
                    uint8_t(ExceptionCode::IllegalDataValue)));
            if (auto e = checkCover(w.waddr, w.wqty); !e)
                return err<Response>(e.error());
            if (auto e = checkCover(w.raddr, w.rqty); !e)
                return err<Response>(e.error());
            if (auto e = model_.write(t, w.waddr, w.wqty,
                                      std::span(w.regs.data(), w.regs.size()),
                                      {});
                !e)
                return err<Response>(e.error());
            RegistersResponse rr;
            rr.regs.resize(w.rqty);
            if (auto e = model_.read(t, w.raddr, w.rqty,
                                     std::span(rr.regs.data(),
                                               rr.regs.size()), {});
                !e)
                return err<Response>(e.error());
            return Response{std::move(rr)};
        }
        default:
            return err<Response>(exceptionToError(
                uint8_t(ExceptionCode::IllegalFunction)));
    }
}

} // namespace tether::modbus

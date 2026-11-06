#include "tether/modbus/protocol/Codec.hpp"

#include "tether/modbus/protocol/BitPacker.hpp"
#include "tether/modbus/protocol/Constants.hpp"

namespace tether::modbus {
namespace {

Expected<Request> decodeRange(std::span<const uint8_t> b,
                              uint16_t maxQty) {
    if (b.size() != 4) return err<Request>(ModbusError::codec("range req len != 4"));
    ReadRangeRequest r{be16(b, 0), be16(b, 2)};
    if (r.qty == 0 || r.qty > maxQty)
        return err<Request>(ModbusError::codec("qty out of range"));
    if (uint32_t(r.addr) + r.qty > kAddressSpace)
        return err<Request>(ModbusError::codec("addr+qty overflow"));
    return Request{r};
}

Expected<void> checkQtyAddr(uint16_t addr, uint32_t qty, uint32_t maxQty,
                            const char* what) {
    if (qty == 0 || qty > maxQty)
        return err(ModbusError::codec(std::string(what) + " qty out of range"));
    if (uint32_t(addr) + qty > kAddressSpace)
        return err(ModbusError::codec(std::string(what) + " addr+qty overflow"));
    return {};
}

} // namespace

Expected<Request> decodeRequest(FunctionCode fc,
                                std::span<const uint8_t> b) {
    switch (fc) {
        case FunctionCode::ReadCoils:
        case FunctionCode::ReadDiscreteInputs:
            return decodeRange(b, kMaxReadBitsQty);
        case FunctionCode::ReadHoldingRegisters:
        case FunctionCode::ReadInputRegisters:
            return decodeRange(b, kMaxReadRegsQty);

        case FunctionCode::WriteSingleCoil: {
            if (b.size() != 4) return err<Request>(ModbusError::codec("fc5 len != 4"));
            const uint16_t val = be16(b, 2);
            if (val != 0x0000 && val != 0xFF00)
                return err<Request>(ModbusError::codec("fc5 val not FF00/0000"));
            return Request{WriteSingleCoilRequest{be16(b, 0), val == 0xFF00}};
        }
        case FunctionCode::WriteSingleRegister: {
            if (b.size() != 4) return err<Request>(ModbusError::codec("fc6 len != 4"));
            return Request{WriteSingleRegRequest{be16(b, 0), be16(b, 2)}};
        }

        case FunctionCode::ReadExceptionStatus: {
            if (!b.empty()) return err<Request>(ModbusError::codec("fc7 body"));
            return Request{UnknownPdu{0x07, {}}};
        }
        case FunctionCode::Diagnostics: {
            if (b.size() < 2) return err<Request>(ModbusError::codec("fc8 short"));
            return Request{DiagRequest{be16(b, 0),
                           {b.begin() + 2, b.end()}}};
        }
        case FunctionCode::GetCommEventCounter:
        case FunctionCode::GetCommEventLog:
        case FunctionCode::ReportServerId: {
            if (!b.empty()) return err<Request>(ModbusError::codec("empty-body fc"));
            return Request{UnknownPdu{static_cast<uint8_t>(fc), {}}};
        }

        case FunctionCode::WriteMultipleCoils: {
            if (b.size() < 5) return err<Request>(ModbusError::codec("fc15 short"));
            WriteMultiCoilsRequest r{be16(b, 0), be16(b, 2), {}};
            if (auto e = checkQtyAddr(r.addr, r.qty, kMaxWriteBitsQty, "fc15"); !e)
                return err<Request>(e.error());
            const uint8_t bc = b[4];
            if (bc != bitByteCount(r.qty) || b.size() != size_t(5 + bc))
                return err<Request>(ModbusError::codec("fc15 byteCount mismatch"));
            r.bits = unpackBits(b.subspan(5), r.qty);
            if (r.bits.size() != r.qty)
                return err<Request>(ModbusError::codec("fc15 bits short"));
            return Request{std::move(r)};
        }
        case FunctionCode::WriteMultipleRegisters: {
            if (b.size() < 5) return err<Request>(ModbusError::codec("fc16 short"));
            WriteMultiRegsRequest r{be16(b, 0), be16(b, 2), {}};
            if (auto e = checkQtyAddr(r.addr, r.qty, kMaxWriteRegsQty, "fc16"); !e)
                return err<Request>(e.error());
            const uint8_t bc = b[4];
            if (bc != r.qty * 2 || b.size() != size_t(5 + bc))
                return err<Request>(ModbusError::codec("fc16 byteCount mismatch"));
            r.regs.reserve(r.qty);
            for (uint16_t i = 0; i < r.qty; ++i)
                r.regs.push_back(be16(b, 5 + i * 2));
            return Request{std::move(r)};
        }

        case FunctionCode::MaskWriteRegister: {
            if (b.size() != 6) return err<Request>(ModbusError::codec("fc22 len != 6"));
            return Request{MaskWriteRequest{be16(b, 0), be16(b, 2), be16(b, 4)}};
        }
        case FunctionCode::ReadWriteMultipleRegs: {
            if (b.size() < 9) return err<Request>(ModbusError::codec("fc23 short"));
            ReadWriteMultiRequest r{be16(b, 0), be16(b, 2), be16(b, 4),
                                    be16(b, 6), {}};
            if (auto e = checkQtyAddr(r.raddr, r.rqty, kMaxRwReadQty, "fc23r"); !e)
                return err<Request>(e.error());
            if (auto e = checkQtyAddr(r.waddr, r.wqty, kMaxRwWriteQty, "fc23w"); !e)
                return err<Request>(e.error());
            const uint8_t bc = b[8];
            if (bc != r.wqty * 2 || b.size() != size_t(9 + bc))
                return err<Request>(ModbusError::codec("fc23 byteCount mismatch"));
            r.regs.reserve(r.wqty);
            for (uint16_t i = 0; i < r.wqty; ++i)
                r.regs.push_back(be16(b, 9 + i * 2));
            return Request{std::move(r)};
        }
        case FunctionCode::ReadFifoQueue: {
            if (b.size() != 2) return err<Request>(ModbusError::codec("fc24 len != 2"));
            return Request{FifoRequest{be16(b, 0)}};
        }

        case FunctionCode::ReadFileRecord:
        case FunctionCode::WriteFileRecord: {
            if (b.size() < 1) return err<Request>(ModbusError::codec("file bc"));
            const uint8_t bc = b[0];
            if (b.size() != size_t(1 + bc))
                return err<Request>(ModbusError::codec("file byteCount mismatch"));
            FileRequest fr;
            size_t off = 1;
            while (off < b.size()) {
                FileRef ref;
                ref.type = b[off];
                if (ref.type != 6)
                    return err<Request>(ModbusError::codec("file ref type != 6"));
                if (b.size() - off < 7)
                    return err<Request>(ModbusError::codec("file ref short"));
                ref.file = be16(b, off + 1);
                ref.record = be16(b, off + 3);
                ref.len = be16(b, off + 5);
                off += 7;
                if (fc == FunctionCode::WriteFileRecord) {
                    if (b.size() - off < size_t(ref.len) * 2)
                        return err<Request>(ModbusError::codec("file data short"));
                    for (uint16_t i = 0; i < ref.len; ++i)
                        ref.data.push_back(be16(b, off + i * 2));
                    off += ref.len * 2;
                }
                fr.refs.push_back(std::move(ref));
            }
            if (fr.refs.empty())
                return err<Request>(ModbusError::codec("empty file req"));
            return Request{std::move(fr)};
        }
        case FunctionCode::EncapsulatedInterface: {
            if (b.size() < 1) return err<Request>(ModbusError::codec("mei short"));
            if (b[0] != 0x0E)
                return err<Request>(ModbusError::codec("mei != 0x0E unsupported"));
            if (b.size() != 3)
                return err<Request>(ModbusError::codec("devid req len != 3"));
            if (b[1] < 1 || b[1] > 4)
                return err<Request>(ModbusError::codec("bad devid read_code"));
            return Request{DevIdRequest{b[1], b[2]}};
        }
    }
    return err<Request>(ModbusError::codec("unknown fc"));
}

Expected<std::vector<uint8_t>> encodeRequest(FunctionCode fc,
                                             const Request& req) {
    std::vector<uint8_t> b;
    auto put16 = [&](uint16_t v) { b.push_back(uint8_t(v >> 8));
                                   b.push_back(uint8_t(v)); };
    switch (fc) {
        case FunctionCode::ReadCoils:
        case FunctionCode::ReadDiscreteInputs:
        case FunctionCode::ReadHoldingRegisters:
        case FunctionCode::ReadInputRegisters: {
            auto* r = std::get_if<ReadRangeRequest>(&req);
            if (!r) return err<std::vector<uint8_t>>(ModbusError::codec("wrong variant"));
            put16(r->addr); put16(r->qty);
            return b;
        }
        case FunctionCode::WriteSingleCoil: {
            auto* r = std::get_if<WriteSingleCoilRequest>(&req);
            if (!r) return err<std::vector<uint8_t>>(ModbusError::codec("wrong variant"));
            put16(r->addr); put16(r->on ? 0xFF00 : 0x0000);
            return b;
        }
        case FunctionCode::WriteSingleRegister: {
            auto* r = std::get_if<WriteSingleRegRequest>(&req);
            if (!r) return err<std::vector<uint8_t>>(ModbusError::codec("wrong variant"));
            put16(r->addr); put16(r->val);
            return b;
        }
        case FunctionCode::WriteMultipleCoils: {
            auto* r = std::get_if<WriteMultiCoilsRequest>(&req);
            if (!r || r->bits.size() != r->qty)
                return err<std::vector<uint8_t>>(ModbusError::codec("fc15 qty/bits"));
            put16(r->addr); put16(r->qty);
            const auto packed = packBits(r->bits);
            b.push_back(uint8_t(packed.size()));
            b.insert(b.end(), packed.begin(), packed.end());
            return b;
        }
        case FunctionCode::WriteMultipleRegisters: {
            auto* r = std::get_if<WriteMultiRegsRequest>(&req);
            if (!r || r->regs.size() != r->qty)
                return err<std::vector<uint8_t>>(ModbusError::codec("fc16 qty/regs"));
            put16(r->addr); put16(r->qty);
            b.push_back(uint8_t(r->qty * 2));
            for (uint16_t v : r->regs) put16(v);
            return b;
        }
        case FunctionCode::MaskWriteRegister: {
            auto* r = std::get_if<MaskWriteRequest>(&req);
            if (!r) return err<std::vector<uint8_t>>(ModbusError::codec("wrong variant"));
            put16(r->addr); put16(r->andMask); put16(r->orMask);
            return b;
        }
        case FunctionCode::ReadWriteMultipleRegs: {
            auto* r = std::get_if<ReadWriteMultiRequest>(&req);
            if (!r || r->regs.size() != r->wqty)
                return err<std::vector<uint8_t>>(ModbusError::codec("fc23 qty/regs"));
            put16(r->raddr); put16(r->rqty); put16(r->waddr); put16(r->wqty);
            b.push_back(uint8_t(r->wqty * 2));
            for (uint16_t v : r->regs) put16(v);
            return b;
        }
        case FunctionCode::ReadFifoQueue: {
            auto* r = std::get_if<FifoRequest>(&req);
            if (!r) return err<std::vector<uint8_t>>(ModbusError::codec("wrong variant"));
            put16(r->addr);
            return b;
        }
        case FunctionCode::ReadFileRecord:
        case FunctionCode::WriteFileRecord: {
            auto* r = std::get_if<FileRequest>(&req);
            if (!r || r->refs.empty())
                return err<std::vector<uint8_t>>(ModbusError::codec("empty file req"));
            std::vector<uint8_t> body;
            for (const FileRef& ref : r->refs) {
                if (ref.type != 6)
                    return err<std::vector<uint8_t>>(ModbusError::codec("ref type != 6"));
                body.push_back(6);
                body.push_back(uint8_t(ref.file >> 8));
                body.push_back(uint8_t(ref.file));
                body.push_back(uint8_t(ref.record >> 8));
                body.push_back(uint8_t(ref.record));
                body.push_back(uint8_t(ref.len >> 8));
                body.push_back(uint8_t(ref.len));
                if (fc == FunctionCode::WriteFileRecord) {
                    if (ref.data.size() != ref.len)
                        return err<std::vector<uint8_t>>(
                            ModbusError::codec("file len/data"));
                    for (uint16_t v : ref.data) {
                        body.push_back(uint8_t(v >> 8));
                        body.push_back(uint8_t(v));
                    }
                }
            }
            if (body.size() > 251)
                return err<std::vector<uint8_t>>(ModbusError::codec("file req > 251B"));
            b.push_back(uint8_t(body.size()));
            b.insert(b.end(), body.begin(), body.end());
            return b;
        }
        case FunctionCode::Diagnostics: {
            auto* r = std::get_if<DiagRequest>(&req);
            if (!r) return err<std::vector<uint8_t>>(ModbusError::codec("wrong variant"));
            put16(r->sub);
            b.insert(b.end(), r->data.begin(), r->data.end());
            return b;
        }
        case FunctionCode::EncapsulatedInterface: {
            auto* r = std::get_if<DevIdRequest>(&req);
            if (!r) return err<std::vector<uint8_t>>(ModbusError::codec("wrong variant"));
            b = {0x0E, r->readCode, r->objectId};
            return b;
        }
        case FunctionCode::ReadExceptionStatus:
        case FunctionCode::GetCommEventCounter:
        case FunctionCode::GetCommEventLog:
        case FunctionCode::ReportServerId:
            return b; // empty body
    }
    return err<std::vector<uint8_t>>(ModbusError::codec("unknown fc"));
}

Expected<Pdu> requestPdu(FunctionCode fc, const Request& req) {
    auto body = encodeRequest(fc, req);
    if (!body) return err<Pdu>(body.error());
    return Pdu{fc, *body};
}

Expected<Response> decodeResponse(FunctionCode fc,
                                  std::span<const uint8_t> b) {
    auto badBc = [&]() {
        return err<Response>(ModbusError::codec("byteCount mismatch"));
    };
    switch (fc) {
        case FunctionCode::ReadCoils:
        case FunctionCode::ReadDiscreteInputs: {
            if (b.size() < 1) return badBc();
            const uint8_t bc = b[0];
            if (b.size() != size_t(1 + bc)) return badBc();
            BitsResponse r;
            r.bits = unpackBits(b.subspan(1), uint16_t(bc * 8));
            return Response{std::move(r)};
        }
        case FunctionCode::ReadHoldingRegisters:
        case FunctionCode::ReadInputRegisters:
        case FunctionCode::ReadWriteMultipleRegs: {
            if (b.size() < 1) return badBc();
            const uint8_t bc = b[0];
            if (bc % 2 != 0 || b.size() != size_t(1 + bc)) return badBc();
            RegistersResponse r;
            r.regs.reserve(bc / 2);
            for (size_t i = 0; i < bc; i += 2)
                r.regs.push_back(be16(b, 1 + i));
            return Response{std::move(r)};
        }
        case FunctionCode::WriteSingleCoil:
        case FunctionCode::WriteSingleRegister: {
            if (b.size() != 4)
                return err<Response>(ModbusError::codec("echo len != 4"));
            return Response{EchoResponse{be16(b, 0), be16(b, 2)}};
        }
        case FunctionCode::WriteMultipleCoils:
        case FunctionCode::WriteMultipleRegisters: {
            if (b.size() != 4)
                return err<Response>(ModbusError::codec("ack len != 4"));
            return Response{RangeAckResponse{be16(b, 0), be16(b, 2)}};
        }
        case FunctionCode::ReadExceptionStatus: {
            if (b.size() != 1)
                return err<Response>(ModbusError::codec("fc7 len != 1"));
            return Response{ExceptionStatusRsp{b[0]}};
        }
        case FunctionCode::Diagnostics: {
            if (b.size() < 2)
                return err<Response>(ModbusError::codec("fc8 short"));
            return Response{DiagResponse{be16(b, 0), {b.begin() + 2, b.end()}}};
        }
        case FunctionCode::GetCommEventCounter: {
            if (b.size() != 4)
                return err<Response>(ModbusError::codec("fc11 len != 4"));
            return Response{EventCtrResponse{be16(b, 0), be16(b, 2)}};
        }
        case FunctionCode::GetCommEventLog: {
            if (b.size() < 7) return badBc();
            const uint8_t bc = b[0];
            if (b.size() != size_t(1 + bc) || bc < 6) return badBc();
            EventLogResponse r{be16(b, 1), be16(b, 3), be16(b, 5),
                               {b.begin() + 7, b.end()}};
            return Response{std::move(r)};
        }
        case FunctionCode::ReportServerId: {
            if (b.size() < 1) return badBc();
            const uint8_t bc = b[0];
            if (b.size() != size_t(1 + bc)) return badBc();
            return Response{ServerIdResponse{{b.begin() + 1, b.end()}}};
        }
        case FunctionCode::ReadFileRecord:
        case FunctionCode::WriteFileRecord: {
            if (fc == FunctionCode::WriteFileRecord) {
                // echo of the request — caller validates via verifyEcho
                return Response{EchoResponse{0, 0}};
            }
            if (b.size() < 1) return badBc();
            const uint8_t bc = b[0];
            if (b.size() != size_t(1 + bc)) return badBc();
            FileRecordResponse fr;
            size_t off = 1;
            while (off < b.size()) {
                const uint8_t flen = b[off];
                if (flen < 1 || off + flen > b.size())
                    return err<Response>(ModbusError::codec("file resp flen"));
                if (b[off + 1] != 6)
                    return err<Response>(ModbusError::codec("file resp type != 6"));
                FileRef ref;
                const uint16_t nWords = uint16_t(flen - 1) / 2;
                for (uint16_t i = 0; i < nWords; ++i)
                    ref.data.push_back(be16(b, off + 2 + i * 2));
                ref.len = nWords;
                fr.refs.push_back(std::move(ref));
                off += size_t(flen) + 1;
            }
            return Response{std::move(fr)};
        }
        case FunctionCode::MaskWriteRegister: {
            if (b.size() != 6)
                return err<Response>(ModbusError::codec("fc22 len != 6"));
            return Response{EchoResponse{be16(b, 0), be16(b, 2)}};
        }
        case FunctionCode::ReadFifoQueue: {
            if (b.size() < 4) return badBc();
            const uint16_t bc = be16(b, 0);
            const uint16_t n = be16(b, 2);
            if (n > kMaxFifoQty || b.size() != size_t(2 + bc) ||
                bc != 2 + n * 2)
                return err<Response>(ModbusError::codec("fc24 counts"));
            FifoResponse r;
            for (uint16_t i = 0; i < n; ++i)
                r.regs.push_back(be16(b, 4 + i * 2));
            return Response{std::move(r)};
        }
        case FunctionCode::EncapsulatedInterface: {
            if (b.size() < 1)
                return err<Response>(ModbusError::codec("mei short"));
            MeiResponse m{b[0], {b.begin() + 1, b.end()}, {}};
            if (m.mei == 0x0E) {
                if (m.data.size() < 5)
                    return err<Response>(ModbusError::codec("devid short"));
                m.devid.conformity = m.data[1];
                m.devid.more = m.data[2] != 0;
                m.devid.nextObjectId = m.data[3];
                const uint8_t cnt = m.data[4];
                size_t off = 5;
                for (uint8_t i = 0; i < cnt; ++i) {
                    if (m.data.size() - off < 2)
                        return err<Response>(ModbusError::codec("devid obj short"));
                    const uint8_t id = m.data[off];
                    const uint8_t len = m.data[off + 1];
                    off += 2;
                    if (m.data.size() - off < len)
                        return err<Response>(ModbusError::codec("devid str short"));
                    m.devid.objects[id] = std::string(
                        reinterpret_cast<const char*>(m.data.data() + off), len);
                    off += len;
                }
            }
            return Response{std::move(m)};
        }
    }
    return err<Response>(ModbusError::codec("unknown fc"));
}

Expected<std::vector<uint8_t>> encodeResponse(FunctionCode fc,
                                              const Response& resp,
                                              const Request& reqCtx) {
    std::vector<uint8_t> b;
    auto put16 = [&](uint16_t v) { b.push_back(uint8_t(v >> 8));
                                   b.push_back(uint8_t(v)); };
    switch (fc) {
        case FunctionCode::ReadCoils:
        case FunctionCode::ReadDiscreteInputs: {
            auto* r = std::get_if<BitsResponse>(&resp);
            if (!r) return err<std::vector<uint8_t>>(ModbusError::codec("wrong variant"));
            const auto packed = packBits(r->bits);
            // byteCount derived from the REQUEST qty, not the bit vector.
            uint16_t qty = uint16_t(r->bits.size());
            if (auto* rr = std::get_if<ReadRangeRequest>(&reqCtx))
                qty = rr->qty;
            b.push_back(bitByteCount(qty));
            b.insert(b.end(), packed.begin(), packed.end());
            return b;
        }
        case FunctionCode::ReadHoldingRegisters:
        case FunctionCode::ReadInputRegisters:
        case FunctionCode::ReadWriteMultipleRegs: {
            auto* r = std::get_if<RegistersResponse>(&resp);
            if (!r) return err<std::vector<uint8_t>>(ModbusError::codec("wrong variant"));
            b.push_back(uint8_t(r->regs.size() * 2));
            for (uint16_t v : r->regs) put16(v);
            return b;
        }
        case FunctionCode::WriteSingleCoil:
        case FunctionCode::WriteSingleRegister:
        case FunctionCode::MaskWriteRegister:
        case FunctionCode::WriteFileRecord: {
            // echoes the request body verbatim
            return encodeRequest(fc, reqCtx);
        }
        case FunctionCode::WriteMultipleCoils:
        case FunctionCode::WriteMultipleRegisters: {
            auto* r = std::get_if<RangeAckResponse>(&resp);
            if (!r) return err<std::vector<uint8_t>>(ModbusError::codec("wrong variant"));
            put16(r->addr); put16(r->qty);
            return b;
        }
        case FunctionCode::ReadExceptionStatus: {
            auto* r = std::get_if<ExceptionStatusRsp>(&resp);
            if (!r) return err<std::vector<uint8_t>>(ModbusError::codec("wrong variant"));
            b.push_back(r->status);
            return b;
        }
        case FunctionCode::Diagnostics: {
            auto* r = std::get_if<DiagResponse>(&resp);
            if (!r) return err<std::vector<uint8_t>>(ModbusError::codec("wrong variant"));
            put16(r->sub);
            b.insert(b.end(), r->data.begin(), r->data.end());
            return b;
        }
        case FunctionCode::GetCommEventCounter: {
            auto* r = std::get_if<EventCtrResponse>(&resp);
            if (!r) return err<std::vector<uint8_t>>(ModbusError::codec("wrong variant"));
            put16(r->status); put16(r->count);
            return b;
        }
        case FunctionCode::GetCommEventLog: {
            auto* r = std::get_if<EventLogResponse>(&resp);
            if (!r) return err<std::vector<uint8_t>>(ModbusError::codec("wrong variant"));
            const size_t bc = 6 + r->events.size();
            if (bc > 255)
                return err<std::vector<uint8_t>>(ModbusError::codec("event log >255"));
            b.push_back(uint8_t(bc));
            put16(r->status); put16(r->eventCnt); put16(r->msgCnt);
            b.insert(b.end(), r->events.begin(), r->events.end());
            return b;
        }
        case FunctionCode::ReportServerId: {
            auto* r = std::get_if<ServerIdResponse>(&resp);
            if (!r || r->id.size() > 250)
                return err<std::vector<uint8_t>>(ModbusError::codec("id >250"));
            b.push_back(uint8_t(r->id.size()));
            b.insert(b.end(), r->id.begin(), r->id.end());
            return b;
        }
        case FunctionCode::ReadFileRecord: {
            auto* r = std::get_if<FileRecordResponse>(&resp);
            if (!r) return err<std::vector<uint8_t>>(ModbusError::codec("wrong variant"));
            std::vector<uint8_t> body;
            for (const FileRef& ref : r->refs) {
                const uint8_t flen = uint8_t(1 + ref.data.size() * 2);
                body.push_back(flen);
                body.push_back(6);
                for (uint16_t v : ref.data) {
                    body.push_back(uint8_t(v >> 8));
                    body.push_back(uint8_t(v));
                }
            }
            if (body.size() > 252)
                return err<std::vector<uint8_t>>(ModbusError::codec("file resp >252"));
            b.push_back(uint8_t(body.size()));
            b.insert(b.end(), body.begin(), body.end());
            return b;
        }
        case FunctionCode::ReadFifoQueue: {
            auto* r = std::get_if<FifoResponse>(&resp);
            if (!r || r->regs.size() > kMaxFifoQty)
                return err<std::vector<uint8_t>>(ModbusError::codec("fifo >31"));
            put16(uint16_t(2 + r->regs.size() * 2));
            put16(uint16_t(r->regs.size()));
            for (uint16_t v : r->regs) put16(v);
            return b;
        }
        case FunctionCode::EncapsulatedInterface: {
            auto* m = std::get_if<MeiResponse>(&resp);
            if (!m) return err<std::vector<uint8_t>>(ModbusError::codec("wrong variant"));
            b.push_back(m->mei);
            if (m->mei == 0x0E) {
                const DevIdRequest* dr = std::get_if<DevIdRequest>(&reqCtx);
                b.push_back(dr ? dr->readCode : 4);
                b.push_back(m->devid.conformity);
                b.push_back(m->devid.more ? 0xFF : 0x00);
                b.push_back(m->devid.nextObjectId);
                b.push_back(uint8_t(m->devid.objects.size()));
                for (const auto& [id, s] : m->devid.objects) {
                    b.push_back(id);
                    b.push_back(uint8_t(s.size()));
                    b.insert(b.end(), s.begin(), s.end());
                }
            } else {
                b.insert(b.end(), m->data.begin(), m->data.end());
            }
            return b;
        }
    }
    return err<std::vector<uint8_t>>(ModbusError::codec("unknown fc"));
}

Expected<void> verifyEcho(FunctionCode fc,
                          std::span<const uint8_t> reqBody,
                          std::span<const uint8_t> rspBody) {
    switch (fc) {
        case FunctionCode::WriteSingleCoil:
        case FunctionCode::WriteSingleRegister:
        case FunctionCode::MaskWriteRegister:
        case FunctionCode::WriteFileRecord:
            if (reqBody.size() != rspBody.size() ||
                !std::equal(reqBody.begin(), reqBody.end(), rspBody.begin()))
                return err(ModbusError::codec("echo mismatch"));
            return {};
        default:
            return err(ModbusError::codec("fc not an echo fc"));
    }
}

std::optional<FunctionCode> parseFunctionCode(std::string_view n) {
    struct { std::string_view n; FunctionCode fc; } kNames[] = {
        {"read_coils", FunctionCode::ReadCoils},
        {"read_discrete_inputs", FunctionCode::ReadDiscreteInputs},
        {"read_holding_regs", FunctionCode::ReadHoldingRegisters},
        {"read_input_regs", FunctionCode::ReadInputRegisters},
        {"write_single_coil", FunctionCode::WriteSingleCoil},
        {"write_single_reg", FunctionCode::WriteSingleRegister},
        {"read_exception_status", FunctionCode::ReadExceptionStatus},
        {"diagnostics", FunctionCode::Diagnostics},
        {"comm_event_ctr", FunctionCode::GetCommEventCounter},
        {"comm_event_log", FunctionCode::GetCommEventLog},
        {"write_multi_coils", FunctionCode::WriteMultipleCoils},
        {"write_multi_regs", FunctionCode::WriteMultipleRegisters},
        {"report_server_id", FunctionCode::ReportServerId},
        {"read_file", FunctionCode::ReadFileRecord},
        {"write_file", FunctionCode::WriteFileRecord},
        {"mask_write", FunctionCode::MaskWriteRegister},
        {"read_write_multi", FunctionCode::ReadWriteMultipleRegs},
        {"read_fifo", FunctionCode::ReadFifoQueue},
        {"encapsulated", FunctionCode::EncapsulatedInterface},
    };
    for (const auto& e : kNames)
        if (e.n == n) return e.fc;
    return std::nullopt;
}

} // namespace tether::modbus

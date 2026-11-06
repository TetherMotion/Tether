/**
 * @file Builder.hpp
 * @brief Client-side PDU builders (exact byte shapes, plan §6.10).
 */
#pragma once

#include <cstdint>
#include <span>

#include "tether/modbus/protocol/Codec.hpp"
#include "tether/modbus/protocol/Pdu.hpp"

namespace tether::modbus::build {

inline Expected<Pdu> readRange(FunctionCode fc, uint16_t addr, uint16_t qty) {
    const bool bits = fc == FunctionCode::ReadCoils ||
                      fc == FunctionCode::ReadDiscreteInputs;
    const uint16_t max = bits ? kMaxReadBitsQty : kMaxReadRegsQty;
    if (qty == 0 || qty > max)
        return err<Pdu>(ModbusError::codec("qty out of range"));
    return requestPdu(fc, ReadRangeRequest{addr, qty});
}
inline Expected<Pdu> writeSingleCoil(uint16_t addr, bool on) {
    return requestPdu(FunctionCode::WriteSingleCoil,
                      WriteSingleCoilRequest{addr, on});
}
inline Expected<Pdu> writeSingleReg(uint16_t addr, uint16_t val) {
    return requestPdu(FunctionCode::WriteSingleRegister,
                      WriteSingleRegRequest{addr, val});
}
inline Expected<Pdu> writeMultiCoils(uint16_t addr,
                                   std::span<const bool> bits) {
    WriteMultiCoilsRequest r{addr, uint16_t(bits.size()),
                             {bits.begin(), bits.end()}};
    return requestPdu(FunctionCode::WriteMultipleCoils, std::move(r));
}
inline Expected<Pdu> writeMultiRegs(uint16_t addr,
                                    std::span<const uint16_t> regs) {
    WriteMultiRegsRequest r{addr, uint16_t(regs.size()),
                            {regs.begin(), regs.end()}};
    return requestPdu(FunctionCode::WriteMultipleRegisters, std::move(r));
}
inline Expected<Pdu> maskWrite(uint16_t addr, uint16_t andM, uint16_t orM) {
    return requestPdu(FunctionCode::MaskWriteRegister,
                      MaskWriteRequest{addr, andM, orM});
}
inline Expected<Pdu> readWriteMulti(uint16_t raddr, uint16_t rqty,
                                    uint16_t waddr,
                                    std::span<const uint16_t> wregs) {
    ReadWriteMultiRequest r{raddr, rqty, waddr, uint16_t(wregs.size()),
                            {wregs.begin(), wregs.end()}};
    return requestPdu(FunctionCode::ReadWriteMultipleRegs, std::move(r));
}
inline Pdu emptyBody(FunctionCode fc) { return Pdu{fc}; }
inline Expected<Pdu> diag(uint16_t sub, std::span<const uint8_t> data = {}) {
    DiagRequest r{sub, {data.begin(), data.end()}};
    return requestPdu(FunctionCode::Diagnostics, std::move(r));
}
inline Expected<Pdu> fifo(uint16_t addr) {
    return requestPdu(FunctionCode::ReadFifoQueue, FifoRequest{addr});
}
inline Expected<Pdu> devId(uint8_t readCode, uint8_t objectId) {
    return requestPdu(FunctionCode::EncapsulatedInterface,
                      DevIdRequest{readCode, objectId});
}
inline Expected<Pdu> fileRecord(FunctionCode fc, std::vector<FileRef> refs) {
    return requestPdu(fc, FileRequest{std::move(refs)});
}
/// Exception PDU (server-side convenience).
inline Pdu exception(uint8_t fc, ExceptionCode code) {
    return Pdu::exception(fc, static_cast<uint8_t>(code));
}

} // namespace tether::modbus::build

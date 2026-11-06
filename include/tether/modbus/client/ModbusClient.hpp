/**
 * @file ModbusClient.hpp
 * @brief Typed client API over IModbusInvoker (plan §8.6).
 */
#pragma once

#include "tether/modbus/client/BusExecutor.hpp"
#include "tether/modbus/client/Invoker.hpp"
#include "tether/modbus/client/TransactionManager.hpp"
#include "tether/modbus/link/Channel.hpp"
#include "tether/modbus/link/SerialConfig.hpp"
#include "tether/modbus/protocol/Builder.hpp"
#include "tether/modbus/protocol/Codec.hpp"
#include "tether/modbus/protocol/Responses.hpp"

namespace tether::modbus {

class ModbusClient {
public:
    explicit ModbusClient(IModbusInvoker& inv, uint8_t unit = 1,
                          TransactionOptions defaults = {})
        : inv_(inv), unit_(unit), defaults_(defaults) {}

    uint8_t unit() const { return unit_; }
    void    setUnit(uint8_t u) { unit_ = u; }
    TransactionOptions& defaults() { return defaults_; }
    IModbusInvoker& invoker() { return inv_; }

    // ---- bit reads (FC1/FC2) ------------------------------------------
    Expected<std::vector<bool>>
    readCoils(uint16_t addr, uint16_t qty,
              const TransactionOptions& o = {}) {
        return readBits(FunctionCode::ReadCoils, addr, qty, o);
    }
    Expected<std::vector<bool>>
    readDiscreteInputs(uint16_t addr, uint16_t qty,
                       const TransactionOptions& o = {}) {
        return readBits(FunctionCode::ReadDiscreteInputs, addr, qty, o);
    }

    // ---- register reads (FC3/FC4) ---------------------------------------
    Expected<std::vector<uint16_t>>
    readHolding(uint16_t addr, uint16_t qty,
                const TransactionOptions& o = {}) {
        return readRegs(FunctionCode::ReadHoldingRegisters, addr, qty, o);
    }
    Expected<std::vector<uint16_t>>
    readInput(uint16_t addr, uint16_t qty,
              const TransactionOptions& o = {}) {
        return readRegs(FunctionCode::ReadInputRegisters, addr, qty, o);
    }
    Expected<uint16_t> readHoldingOne(uint16_t a, const TransactionOptions& o = {}) {
        auto r = readHolding(a, 1, o);
        if (!r) return err<uint16_t>(r.error());
        return r->front();
    }

    // ---- writes -----------------------------------------------------------
    Expected<void> writeCoil(uint16_t addr, bool on,
                             const TransactionOptions& o = {}) {
        auto b = build::writeSingleCoil(addr, on);
        if (!b) return err<void>(b.error());
        auto p = invoke(FunctionCode::WriteSingleCoil, b->body(), o);
        return p ? Expected<void>{} : err<void>(p.error());
    }
    Expected<void> writeReg(uint16_t addr, uint16_t val,
                            const TransactionOptions& o = {}) {
        auto b = build::writeSingleReg(addr, val);
        if (!b) return err<void>(b.error());
        auto p = invoke(FunctionCode::WriteSingleRegister, b->body(), o);
        return p ? Expected<void>{} : err<void>(p.error());
    }
    Expected<void> writeCoils(uint16_t addr, std::span<const bool> bits,
                              const TransactionOptions& o = {}) {
        auto b = build::writeMultiCoils(addr, bits);
        if (!b) return err<void>(b.error());
        auto p = invoke(FunctionCode::WriteMultipleCoils, b->body(), o);
        return p ? Expected<void>{} : err<void>(p.error());
    }
    /// vector<bool> convenience (no span/data() exists for it).
    Expected<void> writeCoils(uint16_t addr,
                             const std::vector<bool>& bits,
                             const TransactionOptions& o = {}) {
        auto tmp = std::make_unique<bool[]>(std::max<size_t>(1, bits.size()));
        for (size_t i = 0; i < bits.size(); ++i) tmp[i] = bits[i];
        return writeCoils(addr,
                          std::span<const bool>(tmp.get(), bits.size()),
                          o);
    }
    Expected<void> writeRegs(uint16_t addr,
                             std::span<const uint16_t> regs,
                             const TransactionOptions& o = {}) {
        auto b = build::writeMultiRegs(addr, regs);
        if (!b) return err<void>(b.error());
        auto p = invoke(FunctionCode::WriteMultipleRegisters, b->body(), o);
        return p ? Expected<void>{} : err<void>(p.error());
    }
    Expected<void> maskWrite(uint16_t addr, uint16_t andM, uint16_t orM,
                             const TransactionOptions& o = {}) {
        auto b = build::maskWrite(addr, andM, orM);
        if (!b) return err<void>(b.error());
        auto p = invoke(FunctionCode::MaskWriteRegister, b->body(), o);
        return p ? Expected<void>{} : err<void>(p.error());
    }
    Expected<std::vector<uint16_t>>
    readWriteMulti(uint16_t raddr, uint16_t rqty, uint16_t waddr,
                   std::span<const uint16_t> wregs,
                   const TransactionOptions& o = {}) {
        auto b = build::readWriteMulti(raddr, rqty, waddr, wregs);
        if (!b) return err<std::vector<uint16_t>>(b.error());
        auto p = invoke(FunctionCode::ReadWriteMultipleRegs, b->body(), o);
        if (!p) return err<std::vector<uint16_t>>(p.error());
        auto r = decodeResponse(FunctionCode::ReadWriteMultipleRegs,
                                p->body());
        if (!r) return err<std::vector<uint16_t>>(r.error());
        return std::get<RegistersResponse>(*r).regs;
    }

    // ---- broadcast -------------------------------------------------------
    Expected<void> broadcastWriteRegs(uint16_t addr,
                                      std::span<const uint16_t> regs) {
        auto b = build::writeMultiRegs(addr, regs);
        if (!b) return err<void>(b.error());
        return inv_.invokeBroadcast(0, FunctionCode::WriteMultipleRegisters,
                                  b->body(), defaults_);
    }
    Expected<void> broadcastWriteReg(uint16_t addr, uint16_t val) {
        auto b = build::writeSingleReg(addr, val);
        if (!b) return err<void>(b.error());
        return inv_.invokeBroadcast(0, FunctionCode::WriteSingleRegister,
                                  b->body(), defaults_);
    }

    // ---- meta -------------------------------------------------------------
    Expected<DeviceIdentification>
    readDeviceId(uint8_t readCode = 1, uint8_t objectId = 0,
                 const TransactionOptions& o = {}) {
        auto b = build::devId(readCode, objectId);
        if (!b) return err<DeviceIdentification>(b.error());
        auto p = invoke(FunctionCode::EncapsulatedInterface, b->body(), o);
        if (!p) return err<DeviceIdentification>(p.error());
        auto r = decodeResponse(FunctionCode::EncapsulatedInterface,
                                p->body());
        if (!r) return err<DeviceIdentification>(r.error());
        return std::get<MeiResponse>(*r).devid;
    }
    Expected<DiagResponse> diagnostics(uint16_t sub,
                                       std::span<const uint8_t> data = {},
                                       const TransactionOptions& o = {}) {
        auto b = build::diag(sub, data);
        if (!b) return err<DiagResponse>(b.error());
        auto p = invoke(FunctionCode::Diagnostics, b->body(), o);
        if (!p) return err<DiagResponse>(p.error());
        auto r = decodeResponse(FunctionCode::Diagnostics, p->body());
        if (!r) return err<DiagResponse>(r.error());
        return std::get<DiagResponse>(*r);
    }
    Expected<std::vector<uint8_t>> reportServerId(
        const TransactionOptions& o = {}) {
        auto p = invoke(FunctionCode::ReportServerId,
                        std::span<const uint8_t>{}, o);
        if (!p) return err<std::vector<uint8_t>>(p.error());
        auto r = decodeResponse(FunctionCode::ReportServerId, p->body());
        if (!r) return err<std::vector<uint8_t>>(r.error());
        return std::get<ServerIdResponse>(*r).id;
    }

    /// Raw escape hatch. Client-level defaults sit between the call's
    /// TransactionOptions and the connection's (executor) defaults.
    Expected<Pdu> invoke(FunctionCode fc, std::span<const uint8_t> body,
                       const TransactionOptions& o) {
        return inv_.invoke(unit_, fc, body, overlayOpts(defaults_, o));
    }

private:
    Expected<std::vector<bool>> readBits(FunctionCode fc, uint16_t a,
                                         uint16_t q,
                                         const TransactionOptions& o) {
        auto b = build::readRange(fc, a, q);
        if (!b) return err<std::vector<bool>>(b.error());
        auto p = invoke(fc, b->body(), o);
        if (!p) return err<std::vector<bool>>(p.error());
        auto r = decodeResponse(fc, p->body());
        if (!r) return err<std::vector<bool>>(r.error());
        return std::get<BitsResponse>(*r).bits;
    }
    Expected<std::vector<uint16_t>> readRegs(FunctionCode fc, uint16_t a,
                                             uint16_t q,
                                             const TransactionOptions& o) {
        auto b = build::readRange(fc, a, q);
        if (!b) return err<std::vector<uint16_t>>(b.error());
        auto p = invoke(fc, b->body(), o);
        if (!p) return err<std::vector<uint16_t>>(p.error());
        auto r = decodeResponse(fc, p->body());
        if (!r) return err<std::vector<uint16_t>>(r.error());
        return std::get<RegistersResponse>(*r).regs;
    }

    IModbusInvoker& inv_;
    uint8_t         unit_;
    TransactionOptions   defaults_;
};

/// Helper: RtuChannel + executor + client bundle for serial masters.
struct RtuMaster {
    std::unique_ptr<RtuChannel> channel;
    std::unique_ptr<BusExecutor> executor;
    std::unique_ptr<ModbusClient> client;
};
struct TcpMaster {
    std::unique_ptr<MbapChannel> channel;
    std::unique_ptr<BusExecutor> executor;         ///< serialized path
    std::unique_ptr<TransactionManager> txn;       ///< pipelined path
    std::unique_ptr<ModbusClient> client;
};

Expected<RtuMaster> makeRtuMaster(const SerialConfig& cfg, uint8_t unit,
                                  const ChannelOpts& copt = {},
                                  const OpenOpts& oopt = {},
                                  const TransactionOptions& iopt = {});
Expected<TcpMaster> makeTcpMaster(std::string_view host, uint16_t port,
                                  uint8_t unit, int connectMs = 5000,
                                  const TransactionOptions& iopt = {});
/// TCP master with pipelining: `maxInflight` concurrent transactions.
Expected<TcpMaster> makeTcpMasterPipelined(
    std::string_view host, uint16_t port, uint8_t unit,
    int maxInflight = 16, int connectMs = 5000,
    const TransactionOptions& iopt = {});

} // namespace tether::modbus

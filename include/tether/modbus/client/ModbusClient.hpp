/**
 * @file ModbusClient.hpp
 * @brief Typed client API over IModbusInvoker (plan §8.6).
 *
 * Every operation exists in two forms:
 *  - synchronous ("wait until done"):  readHolding(...)  → Expected<T>
 *  - std::future async:                readHoldingAsync(...) → future<Expected<T>>
 *
 * Async calls run the full sync path (request build → invoke → decode)
 * on a std::async task. Span arguments are snapshotted at call time, so
 * the caller may mutate/free them immediately. Futures must be consumed
 * before the client (and its invoker/channel) is destroyed.
 */
#pragma once

#include <future>

#include "tether/modbus/client/BusExecutor.hpp"
#include "tether/modbus/client/Invoker.hpp"
#include "tether/modbus/client/TransactionManager.hpp"
#include "tether/modbus/link/Channel.hpp"
#include "tether/modbus/link/SerialConfig.hpp"
#include "tether/modbus/protocol/Builder.hpp"
#include "tether/modbus/protocol/Codec.hpp"
#include "tether/modbus/protocol/Responses.hpp"

namespace tether::modbus {

namespace detail {
/// Launch `f` on a std::async task — the shared implementation behind
/// every *Async method (invoker-level invokeAsync likewise wraps
/// invoke() in std::async when not overridden).
template <typename F>
auto spawnAsync(F&& f) {
    return std::async(std::launch::async, std::forward<F>(f));
}
} // namespace detail

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
    std::future<Expected<std::vector<bool>>>
    readCoilsAsync(uint16_t addr, uint16_t qty, TransactionOptions o = {}) {
        return detail::spawnAsync([this, addr, qty, o = std::move(o)] {
            return readCoils(addr, qty, o);
        });
    }
    std::future<Expected<std::vector<bool>>>
    readDiscreteInputsAsync(uint16_t addr, uint16_t qty,
                            TransactionOptions o = {}) {
        return detail::spawnAsync([this, addr, qty, o = std::move(o)] {
            return readDiscreteInputs(addr, qty, o);
        });
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
    std::future<Expected<std::vector<uint16_t>>>
    readHoldingAsync(uint16_t addr, uint16_t qty, TransactionOptions o = {}) {
        return detail::spawnAsync([this, addr, qty, o = std::move(o)] {
            return readHolding(addr, qty, o);
        });
    }
    std::future<Expected<std::vector<uint16_t>>>
    readInputAsync(uint16_t addr, uint16_t qty, TransactionOptions o = {}) {
        return detail::spawnAsync([this, addr, qty, o = std::move(o)] {
            return readInput(addr, qty, o);
        });
    }
    std::future<Expected<uint16_t>>
    readHoldingOneAsync(uint16_t a, TransactionOptions o = {}) {
        return detail::spawnAsync([this, a, o = std::move(o)] {
            return readHoldingOne(a, o);
        });
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

    // ---- async writes (span args snapshotted at call time) ---------------
    std::future<Expected<void>>
    writeCoilAsync(uint16_t addr, bool on, TransactionOptions o = {}) {
        return detail::spawnAsync([this, addr, on, o = std::move(o)] {
            return writeCoil(addr, on, o);
        });
    }
    std::future<Expected<void>>
    writeRegAsync(uint16_t addr, uint16_t val, TransactionOptions o = {}) {
        return detail::spawnAsync([this, addr, val, o = std::move(o)] {
            return writeReg(addr, val, o);
        });
    }
    std::future<Expected<void>>
    writeCoilsAsync(uint16_t addr, std::span<const bool> bits,
                    TransactionOptions o = {}) {
        return detail::spawnAsync([this, addr,
                      buf = std::vector<bool>(bits.begin(), bits.end()),
                      o = std::move(o)] {
            return writeCoils(addr, buf, o);
        });
    }
    std::future<Expected<void>>
    writeCoilsAsync(uint16_t addr, std::vector<bool> bits,
                    TransactionOptions o = {}) {
        return detail::spawnAsync([this, addr, buf = std::move(bits),
                      o = std::move(o)] {
            return writeCoils(addr, buf, o);
        });
    }
    std::future<Expected<void>>
    writeRegsAsync(uint16_t addr, std::span<const uint16_t> regs,
                   TransactionOptions o = {}) {
        return detail::spawnAsync([this, addr,
                      buf = std::vector<uint16_t>(regs.begin(), regs.end()),
                      o = std::move(o)] {
            return writeRegs(addr, buf, o);
        });
    }
    std::future<Expected<void>>
    maskWriteAsync(uint16_t addr, uint16_t andM, uint16_t orM,
                   TransactionOptions o = {}) {
        return detail::spawnAsync([this, addr, andM, orM, o = std::move(o)] {
            return maskWrite(addr, andM, orM, o);
        });
    }
    std::future<Expected<std::vector<uint16_t>>>
    readWriteMultiAsync(uint16_t raddr, uint16_t rqty, uint16_t waddr,
                        std::span<const uint16_t> wregs,
                        TransactionOptions o = {}) {
        return detail::spawnAsync([this, raddr, rqty, waddr,
                      buf = std::vector<uint16_t>(wregs.begin(), wregs.end()),
                      o = std::move(o)] {
            return readWriteMulti(raddr, rqty, waddr, buf, o);
        });
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
    std::future<Expected<void>>
    broadcastWriteRegsAsync(uint16_t addr, std::span<const uint16_t> regs) {
        return detail::spawnAsync([this, addr,
                      buf = std::vector<uint16_t>(regs.begin(), regs.end())] {
            return broadcastWriteRegs(addr, buf);
        });
    }
    std::future<Expected<void>>
    broadcastWriteRegAsync(uint16_t addr, uint16_t val) {
        return detail::spawnAsync([this, addr, val] {
            return broadcastWriteReg(addr, val);
        });
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
    std::future<Expected<DeviceIdentification>>
    readDeviceIdAsync(uint8_t readCode = 1, uint8_t objectId = 0,
                      TransactionOptions o = {}) {
        return detail::spawnAsync([this, readCode, objectId, o = std::move(o)] {
            return readDeviceId(readCode, objectId, o);
        });
    }
    std::future<Expected<DiagResponse>>
    diagnosticsAsync(uint16_t sub, std::span<const uint8_t> data = {},
                     TransactionOptions o = {}) {
        return detail::spawnAsync([this, sub,
                      buf = std::vector<uint8_t>(data.begin(), data.end()),
                      o = std::move(o)] {
            return diagnostics(sub, buf, o);
        });
    }
    std::future<Expected<std::vector<uint8_t>>>
    reportServerIdAsync(TransactionOptions o = {}) {
        return detail::spawnAsync([this, o = std::move(o)] {
            return reportServerId(o);
        });
    }

    /// Raw escape hatch. Client-level defaults sit between the call's
    /// TransactionOptions and the connection's (executor) defaults.
    Expected<Pdu> invoke(FunctionCode fc, std::span<const uint8_t> body,
                       const TransactionOptions& o) {
        return inv_.invoke(unit_, fc, body, overlayOpts(defaults_, o));
    }
    /// Raw async invoke — body is copied; defaults merge as in invoke().
    std::future<Expected<Pdu>>
    invokeAsync(FunctionCode fc, std::span<const uint8_t> body,
                TransactionOptions o = {}) {
        return inv_.invokeAsync(unit_, fc,
            std::vector<uint8_t>(body.begin(), body.end()),
            overlayOpts(defaults_, o));
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
